#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
高频统一传感器读取脚本（hf_recorder）

目标：
1) 以 IMU 到包为主时钟触发记录，避免固定 10Hz 轮询导致的丢样；
2) 压力不同步时允许降级记录（使用最近值并标记 stale），避免整行丢弃；
3) 保持与 recorder.py 兼容的输出行格式，便于后续融合脚本复用。
"""

import queue
import re
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional

import serial

# =========================
# 串口配置
# =========================
IMU_PORT = "/dev/ttyUSB3"
IMU_BAUD_RATE = 9600  # 按要求固定为9600

PRESSURE_PORTS = {
    "front": "/dev/ttyUSB0",
    "left": "/dev/ttyUSB1",
    "right": "/dev/ttyUSB2",
}
PRESSURE_BAUD_RATE = 115200
TIMEOUT = 0.2

# =========================
# 同步/记录策略
# =========================
MAX_PRESSURE_AGE = 0.8  # 压力超过该时间视为 stale，但仍可记录
ALLOW_STALE_PRESSURE = True

# 如需人为限频，设置为正数（例如 100）；None 或 <=0 表示不主动限频
OUTPUT_MAX_HZ: Optional[float] = None

# 队列容量，避免内存无界增长
IMU_QUEUE_MAXSIZE = 20000
PRESSURE_QUEUE_MAXSIZE = 20000


@dataclass
class IMUDataPacket:
    timestamp: float
    gyro: Dict[str, float]
    accel: Dict[str, float]
    velocity: Dict[str, float]


@dataclass
class PressureDataPacket:
    timestamp: float
    pressure: float
    temperature: float
    depth: float
    status: str


@dataclass
class SyncedDataPacket:
    timestamp: float
    vel_x: float
    vel_y: float
    vel_z: float
    acc_x: float
    acc_y: float
    acc_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    pressure_front: float
    pressure_left: float
    pressure_right: float
    pressure_front_temp: float
    pressure_left_temp: float
    pressure_right_temp: float
    pressure_front_stale: int
    pressure_left_stale: int
    pressure_right_stale: int


imu_queue: queue.Queue = queue.Queue(maxsize=IMU_QUEUE_MAXSIZE)
pressure_queues: Dict[str, queue.Queue] = {
    "front": queue.Queue(maxsize=PRESSURE_QUEUE_MAXSIZE),
    "left": queue.Queue(maxsize=PRESSURE_QUEUE_MAXSIZE),
    "right": queue.Queue(maxsize=PRESSURE_QUEUE_MAXSIZE),
}

all_data_packets: List[SyncedDataPacket] = []
packets_lock = threading.Lock()

running = True


class JY901SParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.packet_len = 11
        self.gravity = 9.8

    def update(self, data: bytes) -> None:
        self.buffer.extend(data)

    def verify(self, packet: bytes) -> bool:
        return (sum(packet[:-1]) & 0xFF) == packet[-1]

    @staticmethod
    def _int16(high_byte: int, low_byte: int) -> int:
        v = (high_byte << 8) | low_byte
        if v & 0x8000:
            v -= 0x10000
        return v

    def parse_packet(self, packet: bytes) -> Optional[Dict[str, float]]:
        t = packet[1]
        if t == 0x51:
            ax = self._int16(packet[3], packet[2]) * (16.0 * self.gravity / 32768.0)
            ay = self._int16(packet[5], packet[4]) * (16.0 * self.gravity / 32768.0)
            az = self._int16(packet[7], packet[6]) * (16.0 * self.gravity / 32768.0)
            return {"type": "acc", "acc_x": ax, "acc_y": ay, "acc_z": az}
        if t == 0x52:
            gx = self._int16(packet[3], packet[2]) * (2000.0 / 32768.0)
            gy = self._int16(packet[5], packet[4]) * (2000.0 / 32768.0)
            gz = self._int16(packet[7], packet[6]) * (2000.0 / 32768.0)
            return {"type": "gyro", "gyro_x": gx, "gyro_y": gy, "gyro_z": gz}
        if t == 0x53:
            return {"type": "angle"}  # 高频记录仅需要用其作为一帧完成触发
        return None

    def find_packets(self) -> List[bytes]:
        out: List[bytes] = []
        while len(self.buffer) >= 2:
            if self.buffer[0] != 0x55:
                del self.buffer[0]
                continue
            if self.buffer[1] not in (0x51, 0x52, 0x53, 0x54):
                del self.buffer[0]
                continue
            if len(self.buffer) < self.packet_len:
                break
            pkt = bytes(self.buffer[: self.packet_len])
            del self.buffer[: self.packet_len]
            if self.verify(pkt):
                out.append(pkt)
        return out


def _put_latest(q: queue.Queue, item) -> None:
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        try:
            q.put_nowait(item)
        except queue.Full:
            pass


def read_imu_serial() -> None:
    ser = None
    try:
        ser = serial.Serial(
            port=IMU_PORT,
            baudrate=IMU_BAUD_RATE,
            timeout=TIMEOUT,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        print(f"[IMU] 监听IMU传感器: {IMU_PORT} @ {IMU_BAUD_RATE}")
        ser.reset_input_buffer()

        parser = JY901SParser()
        latest = {}
        vel = {"vel_x": 0.0, "vel_y": 0.0, "vel_z": 0.0}
        last_ts: Optional[float] = None

        while running:
            n = ser.in_waiting
            if n <= 0:
                time.sleep(0.0005)
                continue
            data = ser.read(n)
            parser.update(data)
            for pkt in parser.find_packets():
                parsed = parser.parse_packet(pkt)
                if not parsed:
                    continue
                latest[parsed["type"]] = parsed

                # 当至少收到了 gyro + acc，并且看到 angle 帧时，输出一包
                if "gyro" in latest and "acc" in latest and "angle" in latest:
                    ts = time.time()
                    dt = 0.0 if last_ts is None else (ts - last_ts)
                    if 0 < dt <= 1.0:
                        vel["vel_x"] += latest["acc"]["acc_x"] * dt
                        vel["vel_y"] += latest["acc"]["acc_y"] * dt
                        vel["vel_z"] += latest["acc"]["acc_z"] * dt
                    elif dt > 1.0:
                        vel = {"vel_x": 0.0, "vel_y": 0.0, "vel_z": 0.0}
                    last_ts = ts

                    imu_pkt = IMUDataPacket(
                        timestamp=ts,
                        gyro={
                            "gyro_x": latest["gyro"]["gyro_x"],
                            "gyro_y": latest["gyro"]["gyro_y"],
                            "gyro_z": latest["gyro"]["gyro_z"],
                        },
                        accel={
                            "acc_x": latest["acc"]["acc_x"],
                            "acc_y": latest["acc"]["acc_y"],
                            "acc_z": latest["acc"]["acc_z"],
                        },
                        velocity={"vel_x": vel["vel_x"], "vel_y": vel["vel_y"], "vel_z": vel["vel_z"]},
                    )
                    _put_latest(imu_queue, imu_pkt)
                    latest.pop("angle", None)  # 避免重复触发
    except Exception as e:
        print(f"[IMU] 处理错误: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()


def hex_to_ascii(hex_data) -> str:
    try:
        if isinstance(hex_data, bytes):
            return hex_data.decode("ascii", errors="ignore")
        return str(hex_data)
    except Exception:
        return str(hex_data)


def clean_ascii(data: str) -> str:
    data = data.replace("..", "\n")
    data = re.sub(r"\s+", " ", data)
    data = re.sub(r"C\.", "°C", data)
    return data


def parse_pressure_data(data: str) -> Dict[str, float]:
    out: Dict[str, float] = {}
    d = clean_ascii(data)
    m = re.search(r"Final result:\s*Temperature=([\d.]+)C,\s*Pressure=([\d.]+)\s*mbar", d, re.IGNORECASE)
    if m:
        out["temperature"] = float(m.group(1))
        out["pressure"] = float(m.group(2))
        return out
    m = re.search(r"Pressure:\s*([\d.]+)\s*mbar", d, re.IGNORECASE)
    if m:
        out["pressure"] = float(m.group(1))
    m = re.search(r"Temperature:\s*([\d.]+)\s*°?C", d, re.IGNORECASE)
    if m:
        out["temperature"] = float(m.group(1))
    m = re.search(r"Water Depth:\s*([\d.-]+)\s*m", d, re.IGNORECASE)
    if m:
        out["depth"] = float(m.group(1))
    return out


def read_pressure_serial(name: str, serial_port: str) -> None:
    ser = None
    try:
        ser = serial.Serial(
            port=serial_port,
            baudrate=PRESSURE_BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=TIMEOUT,
        )
        print(f"[{name}] 监听压力传感器: {serial_port} @ {PRESSURE_BAUD_RATE}")
        ser.reset_input_buffer()
        current = b""
        started = False
        last_data_time = time.time()

        while running:
            if ser.in_waiting > 0:
                raw = ser.read(ser.in_waiting)
                current += raw
                last_data_time = time.time()
            else:
                if time.time() - last_data_time > 2.0 and current:
                    current = b""
                    started = False
                time.sleep(0.001)
                continue

            txt = hex_to_ascii(current)
            if "===" in txt and not started:
                started = True
                idx = txt.find("===")
                if idx > 0:
                    current = current[idx:]
                    txt = hex_to_ascii(current)

            if started and "=====================================" in txt:
                parsed = parse_pressure_data(txt)
                if "pressure" in parsed:
                    pkt = PressureDataPacket(
                        timestamp=time.time(),
                        pressure=parsed["pressure"],
                        temperature=parsed.get("temperature", 0.0),
                        depth=parsed.get("depth", 0.0),
                        status="OK",
                    )
                    _put_latest(pressure_queues[name], pkt)
                current = b""
                started = False

            if len(current) > 4000:
                current = b""
                started = False
    except Exception as e:
        print(f"[{name}] 压力处理错误: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()


def _drain_pressure_queues(pressure_buffers: Dict[str, deque]) -> None:
    for name, q in pressure_queues.items():
        try:
            while True:
                pressure_buffers[name].append(q.get_nowait())
        except queue.Empty:
            pass


def _align_pressure_to_imu(buffer: deque, t_ref: float) -> (Optional[PressureDataPacket], int):
    """
    将压力数据对齐到 IMU 时间戳：
    1) 优先使用 t_ref 前后两个压力样本做线性插值；
    2) 若无法插值（只有单侧样本），退化到最近邻；
    3) 返回 (对齐后的压力包, stale标记)。
    """
    if not buffer:
        return None, 1

    # 取时间升序样本
    arr = sorted(buffer, key=lambda p: p.timestamp)
    prev_pkt: Optional[PressureDataPacket] = None
    next_pkt: Optional[PressureDataPacket] = None
    best_pkt: Optional[PressureDataPacket] = None
    min_dt = float("inf")

    for p in arr:
        d = abs(p.timestamp - t_ref)
        if d < min_dt:
            min_dt = d
            best_pkt = p
        if p.timestamp <= t_ref:
            prev_pkt = p
        if p.timestamp >= t_ref and next_pkt is None:
            next_pkt = p

    # 情况A：存在前后点，做线性插值
    if prev_pkt is not None and next_pkt is not None:
        t0 = prev_pkt.timestamp
        t1 = next_pkt.timestamp
        if t1 > t0:
            w = (t_ref - t0) / (t1 - t0)
            w = max(0.0, min(1.0, w))
            aligned = PressureDataPacket(
                timestamp=t_ref,
                pressure=prev_pkt.pressure + w * (next_pkt.pressure - prev_pkt.pressure),
                temperature=prev_pkt.temperature + w * (next_pkt.temperature - prev_pkt.temperature),
                depth=prev_pkt.depth + w * (next_pkt.depth - prev_pkt.depth),
                status="INTERP",
            )
            # 插值覆盖的跨度过大时仍标记 stale
            stale = 1 if (t1 - t0) > (2.0 * MAX_PRESSURE_AGE) else 0
            return aligned, stale

    # 情况B：退化到最近邻
    if best_pkt is None:
        return None, 1
    stale = 1 if min_dt > MAX_PRESSURE_AGE else 0
    return best_pkt, stale


def synchronize_and_record() -> None:
    print("=" * 120)
    print("高频统一传感器数据监测系统（IMU触发记录）")
    print("=" * 120)
    print("时间戳-xyz速度-xyz加速度-角速度-三个压力传感器压力")
    print("-" * 120)

    pressure_buffers = {"front": deque(maxlen=2000), "left": deque(maxlen=2000), "right": deque(maxlen=2000)}
    last_out_ts: Optional[float] = None

    while running:
        _drain_pressure_queues(pressure_buffers)

        try:
            imu_pkt: IMUDataPacket = imu_queue.get(timeout=0.2)
        except queue.Empty:
            continue

        # 可选人为限频
        if OUTPUT_MAX_HZ and OUTPUT_MAX_HZ > 0:
            if last_out_ts is not None and (imu_pkt.timestamp - last_out_ts) < (1.0 / OUTPUT_MAX_HZ):
                continue

        synced = {}
        stale_flags = {}
        for name in ("front", "left", "right"):
            aligned_pkt, stale = _align_pressure_to_imu(pressure_buffers[name], imu_pkt.timestamp)
            if aligned_pkt is None:
                synced[name] = None
                stale_flags[name] = 1
                continue
            stale_flags[name] = stale
            synced[name] = aligned_pkt

        if not ALLOW_STALE_PRESSURE and any(synced[n] is None or stale_flags[n] == 1 for n in ("front", "left", "right")):
            continue

        # 缺失时回填NaN（文本写成 nan）
        def pval(name: str, key: str) -> float:
            pkt = synced[name]
            if pkt is None:
                return float("nan")
            return getattr(pkt, key)

        timestamp_str = datetime.fromtimestamp(imu_pkt.timestamp).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        line = (
            f"{timestamp_str}-"
            f"{imu_pkt.velocity['vel_x']:.4f},{imu_pkt.velocity['vel_y']:.4f},{imu_pkt.velocity['vel_z']:.4f}-"
            f"{imu_pkt.accel['acc_x']:.4f},{imu_pkt.accel['acc_y']:.4f},{imu_pkt.accel['acc_z']:.4f}-"
            f"{imu_pkt.gyro['gyro_x']:.4f},{imu_pkt.gyro['gyro_y']:.4f},{imu_pkt.gyro['gyro_z']:.4f}-"
            f"{pval('front','pressure'):.2f},{pval('left','pressure'):.2f},{pval('right','pressure'):.2f}mbar"
        )
        print(line)

        pkt = SyncedDataPacket(
            timestamp=imu_pkt.timestamp,
            vel_x=imu_pkt.velocity["vel_x"],
            vel_y=imu_pkt.velocity["vel_y"],
            vel_z=imu_pkt.velocity["vel_z"],
            acc_x=imu_pkt.accel["acc_x"],
            acc_y=imu_pkt.accel["acc_y"],
            acc_z=imu_pkt.accel["acc_z"],
            gyro_x=imu_pkt.gyro["gyro_x"],
            gyro_y=imu_pkt.gyro["gyro_y"],
            gyro_z=imu_pkt.gyro["gyro_z"],
            pressure_front=pval("front", "pressure"),
            pressure_left=pval("left", "pressure"),
            pressure_right=pval("right", "pressure"),
            pressure_front_temp=pval("front", "temperature"),
            pressure_left_temp=pval("left", "temperature"),
            pressure_right_temp=pval("right", "temperature"),
            pressure_front_stale=stale_flags["front"],
            pressure_left_stale=stale_flags["left"],
            pressure_right_stale=stale_flags["right"],
        )
        with packets_lock:
            all_data_packets.append(pkt)
        last_out_ts = imu_pkt.timestamp


def save_data_to_file(output_file: str) -> None:
    with packets_lock:
        if not all_data_packets:
            print("没有数据包需要保存")
            return
        print(f"\n正在保存 {len(all_data_packets)} 个数据包到文件: {output_file}")
        with open(output_file, "w", encoding="utf-8") as f:
            f.write("=" * 120 + "\n")
            f.write("高频统一传感器数据日志\n")
            f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"数据包总数: {len(all_data_packets)}\n")
            f.write("=" * 120 + "\n\n")
            f.write("时间戳-xyz速度-xyz加速度-角速度-三个压力传感器压力\n")
            f.write("-" * 120 + "\n")
            for p in all_data_packets:
                ts = datetime.fromtimestamp(p.timestamp).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                f.write(
                    f"{ts}-"
                    f"{p.vel_x:.4f},{p.vel_y:.4f},{p.vel_z:.4f}-"
                    f"{p.acc_x:.4f},{p.acc_y:.4f},{p.acc_z:.4f}-"
                    f"{p.gyro_x:.4f},{p.gyro_y:.4f},{p.gyro_z:.4f}-"
                    f"{p.pressure_front:.2f},{p.pressure_left:.2f},{p.pressure_right:.2f}mbar\n"
                )
            f.write("\n" + "=" * 120 + "\n")
            f.write("附加字段（压力stale标记：1=过期/降级，0=新鲜）\n")
            f.write("=" * 120 + "\n")
            for i, p in enumerate(all_data_packets, 1):
                ts = datetime.fromtimestamp(p.timestamp).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                f.write(
                    f"#{i} {ts} "
                    f"stale(front,left,right)=({p.pressure_front_stale},{p.pressure_left_stale},{p.pressure_right_stale})\n"
                )
    print(f"数据已保存: {output_file}")


def main() -> None:
    global running
    ts_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_file = f"{ts_name}—hf_datalog.txt"

    def _sig_handler(_sig, _frame):
        nonlocal output_file
        global running
        print("\n接收到退出信号，准备保存数据...")
        running = False
        time.sleep(0.2)
        save_data_to_file(output_file)
        sys.exit(0)

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    imu_t = threading.Thread(target=read_imu_serial, daemon=True)
    imu_t.start()
    p_threads = []
    for n, port in PRESSURE_PORTS.items():
        t = threading.Thread(target=read_pressure_serial, args=(n, port), daemon=True)
        t.start()
        p_threads.append(t)

    time.sleep(1.0)
    print("\n所有传感器线程已启动（高频模式）")
    print(f"输出文件: {output_file}")
    print("按 Ctrl+C 退出并保存\n")

    try:
        synchronize_and_record()
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        time.sleep(0.2)
        save_data_to_file(output_file)
        print("hf_recorder 已退出")


if __name__ == "__main__":
    main()

