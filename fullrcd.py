#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
时间戳 + 完整 IMU + 三路压力 记录脚本
- 控制台实时打印；Ctrl+C 保存 TSV 日志（与控制台列一致）
- Linux 默认 /dev/ttyUSB*；Windows 请改为 COM 口
依赖: pyserial
"""

import serial
import threading
import time
import re
import signal
import sys
import queue
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional

# ========== 串口（按实际修改）==========
IMU_PORT = "/dev/ttyUSB3"
IMU_BAUD = 9600

PRESSURE_PORTS = {
    "front": "/dev/ttyUSB0",
    "left": "/dev/ttyUSB1",
    "right": "/dev/ttyUSB2",
}
PRESSURE_BAUD = 115200
TIMEOUT = 1.0

SYNC_INTERVAL = 0.1
MAX_DATA_AGE = 0.5

imu_queue: queue.Queue = queue.Queue()
pressure_queues = {k: queue.Queue() for k in PRESSURE_PORTS}


@dataclass
class IMUPacket:
    t: float
    roll: float
    pitch: float
    yaw: float
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float
    mag_x: Optional[float] = None
    mag_y: Optional[float] = None
    mag_z: Optional[float] = None
    imu_temp_c: Optional[float] = None


@dataclass
class PressurePacket:
    t: float
    pressure_mbar: float
    temp_c: float


class JY901S_Parser:
    def __init__(self):
        self.buf = bytearray()
        self.L = 11
        self.g = 9.8

    def feed(self, data: bytes):
        self.buf.extend(data)

    def packets(self):
        out = []
        while len(self.buf) >= 2:
            if self.buf[0] != 0x55:
                del self.buf[0]
                continue
            if self.buf[1] not in (0x51, 0x52, 0x53, 0x54):
                del self.buf[0]
                continue
            if len(self.buf) < self.L:
                break
            pkt = bytes(self.buf[: self.L])
            del self.buf[: self.L]
            if sum(pkt[:-1]) & 0xFF == pkt[-1]:
                out.append(pkt)
        return out

    @staticmethod
    def i16(hi: int, lo: int) -> int:
        v = (hi << 8) | lo
        return v - 0x10000 if v & 0x8000 else v

    def parse(self, pkt: bytes) -> Optional[Dict[str, Any]]:
        t = pkt[1]
        if t == 0x51:
            ax, ay, az = [self.i16(pkt[i + 1], pkt[i]) for i in (2, 4, 6)]
            tmp = self.i16(pkt[9], pkt[8]) / 100.0
            s = 16.0 * self.g / 32768.0
            return {
                "type": "加速度",
                "acc_x": ax * s,
                "acc_y": ay * s,
                "acc_z": az * s,
                "temperature": tmp,
            }
        if t == 0x52:
            wx, wy, wz = [self.i16(pkt[i + 1], pkt[i]) for i in (2, 4, 6)]
            s = 2000.0 / 32768.0
            return {"type": "角速度", "gx": wx * s, "gy": wy * s, "gz": wz * s}
        if t == 0x53:
            r, p, y = [self.i16(pkt[i + 1], pkt[i]) for i in (2, 4, 6)]
            s = 180.0 / 32768.0
            return {"type": "姿态角", "roll": r * s, "pitch": p * s, "yaw": y * s}
        if t == 0x54:
            hx, hy, hz = [self.i16(pkt[i + 1], pkt[i]) for i in (2, 4, 6)]
            return {"type": "磁场", "mx": float(hx), "my": float(hy), "mz": float(hz)}
        return None


def hex_to_ascii(b: bytes) -> str:
    try:
        return b.decode("ascii", errors="ignore")
    except Exception:
        return "".join(chr(x) if 32 <= x < 127 else "." for x in b)


def clean_ascii(s: str) -> str:
    s = s.replace("..", "\n")
    s = re.sub(r"\s+", " ", s)
    s = re.sub(r"(?<!\n)\n(?!\n)", " ", s)
    s = re.sub(r"C\.", "°C", s)
    return re.sub(r"([a-zA-Z])(\d)", r"\1 \2", s)


def parse_pressure_text(text: str) -> Dict[str, float]:
    r: Dict[str, float] = {}
    c = clean_ascii(text)
    m = re.search(
        r"Final result:\s*Temperature=([\d.]+)C,\s*Pressure=([\d.]+)\s*mbar",
        c,
        re.I,
    )
    if m:
        r["temperature"] = float(m.group(1))
        r["pressure"] = float(m.group(2))
        return r
    m = re.search(r"Pressure:\s*([\d.]+)\s*mbar", c, re.I)
    if m:
        r["pressure"] = float(m.group(1))
    m = re.search(r"Temperature:\s*([\d.]+)\s*°?C", c, re.I)
    if m:
        r["temperature"] = float(m.group(1))
    return r


def read_imu():
    ser = serial.Serial(
        IMU_PORT,
        IMU_BAUD,
        timeout=TIMEOUT,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    ser.reset_input_buffer()
    p = JY901S_Parser()
    latest: Dict[str, Any] = {}
    print(f"[IMU] {IMU_PORT}")
    try:
        while True:
            if ser.in_waiting:
                p.feed(ser.read(ser.in_waiting))
            for pkt in p.packets():
                d = p.parse(pkt)
                if not d:
                    continue
                latest[d["type"]] = d
                if "姿态角" in latest and "角速度" in latest:
                    a = latest.get("加速度", {})
                    m = latest.get("磁场")
                    imu = IMUPacket(
                        t=time.time(),
                        roll=latest["姿态角"]["roll"],
                        pitch=latest["姿态角"]["pitch"],
                        yaw=latest["姿态角"]["yaw"],
                        gx=latest["角速度"]["gx"],
                        gy=latest["角速度"]["gy"],
                        gz=latest["角速度"]["gz"],
                        ax=a.get("acc_x", 0.0),
                        ay=a.get("acc_y", 0.0),
                        az=a.get("acc_z", 0.0),
                        mag_x=m["mx"] if m else None,
                        mag_y=m["my"] if m else None,
                        mag_z=m["mz"] if m else None,
                        imu_temp_c=a.get("temperature"),
                    )
                    try:
                        imu_queue.put_nowait(imu)
                    except queue.Full:
                        pass
            time.sleep(0.001)
    finally:
        ser.close()


def read_pressure(name: str, port: str):
    ser = serial.Serial(
        port,
        PRESSURE_BAUD,
        timeout=TIMEOUT,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
    )
    ser.reset_input_buffer()
    buf = b""
    started = False
    last_t = time.time()
    print(f"[{name}] {port}")
    try:
        while True:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting)
                last_t = time.time()
            else:
                if time.time() - last_t > 2.0 and buf and started:
                    asc = hex_to_ascii(buf)
                    if "Final result" in asc or "Pressure:" in asc:
                        pr = parse_pressure_text(asc)
                        if "pressure" in pr:
                            try:
                                pressure_queues[name].put_nowait(
                                    PressurePacket(
                                        last_t,
                                        pr["pressure"],
                                        pr.get("temperature", 0.0),
                                    )
                                )
                            except queue.Full:
                                pass
                    buf = b""
                    started = False
                time.sleep(0.01)
                continue
            asc = hex_to_ascii(buf)
            if "===" in asc and not started:
                started = True
                i = asc.find("===")
                if i > 0:
                    buf = buf[i:]
                    asc = hex_to_ascii(buf)
            if started and "=====================================" in asc:
                pr = parse_pressure_text(asc)
                if "pressure" in pr:
                    try:
                        pressure_queues[name].put_nowait(
                            PressurePacket(
                                last_t,
                                pr["pressure"],
                                pr.get("temperature", 0.0),
                            )
                        )
                    except queue.Full:
                        pass
                buf = b""
                started = False
            if len(buf) > 2000:
                buf = b""
                started = False
    finally:
        ser.close()


def fmt_opt(x: Optional[float], nd: int = 4) -> str:
    if x is None:
        return ""
    return f"{x:.{nd}f}"


def row_tsv(imu: IMUPacket, pf: float, pl: float, pr: float) -> str:
    dt = datetime.fromtimestamp(imu.t).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    return "\t".join(
        [
            f"{imu.t:.6f}",
            dt,
            f"{imu.roll:.4f}",
            f"{imu.pitch:.4f}",
            f"{imu.yaw:.4f}",
            f"{imu.gx:.4f}",
            f"{imu.gy:.4f}",
            f"{imu.gz:.4f}",
            f"{imu.ax:.6f}",
            f"{imu.ay:.6f}",
            f"{imu.az:.6f}",
            fmt_opt(imu.mag_x, 2),
            fmt_opt(imu.mag_y, 2),
            fmt_opt(imu.mag_z, 2),
            fmt_opt(imu.imu_temp_c, 2),
            f"{pf:.4f}",
            f"{pl:.4f}",
            f"{pr:.4f}",
        ]
    )


def main():
    rows: List[str] = []
    lock = threading.Lock()
    out_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "—imu_pressure.tsv"

    def save_and_exit(*_):
        path = out_name
        hdr = "\t".join(
            [
                "unix_ts",
                "datetime_local",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
                "gyro_x_dps",
                "gyro_y_dps",
                "gyro_z_dps",
                "acc_x_m_s2",
                "acc_y_m_s2",
                "acc_z_m_s2",
                "mag_x_raw",
                "mag_y_raw",
                "mag_z_raw",
                "imu_temp_c",
                "pressure_front_mbar",
                "pressure_left_mbar",
                "pressure_right_mbar",
            ]
        )
        with lock:
            with open(path, "w", encoding="utf-8") as f:
                f.write("# timestamp + full IMU + 3 pressures (TSV)\n")
                f.write(hdr + "\n")
                f.write("\n".join(rows) + ("\n" if rows else ""))
        print(f"\n已保存 {len(rows)} 行 -> {path}")
        sys.exit(0)

    signal.signal(signal.SIGINT, save_and_exit)
    signal.signal(signal.SIGTERM, save_and_exit)

    threading.Thread(target=read_imu, daemon=True).start()
    for n, pt in PRESSURE_PORTS.items():
        threading.Thread(target=read_pressure, args=(n, pt), daemon=True).start()

    time.sleep(2)
    print("列: unix_ts | 本地时间 | 姿态(°) | 陀螺(°/s) | 加速度(m/s²) | 磁(raw) | IMU温度 | 前/左/右压力(mbar)")
    print("Ctrl+C 保存 TSV\n")

    imu_buf: deque = deque(maxlen=10)
    pbuf = {k: deque(maxlen=10) for k in PRESSURE_PORTS}

    hdr_printed = False
    while True:
        try:
            while True:
                imu_buf.append(imu_queue.get_nowait())
        except queue.Empty:
            pass
        for k in pbuf:
            try:
                while True:
                    pbuf[k].append(pressure_queues[k].get_nowait())
            except queue.Empty:
                pass

        if not imu_buf:
            time.sleep(SYNC_INTERVAL)
            continue

        imu = imu_buf[-1]
        sp: Dict[str, PressurePacket] = {}
        for name in PRESSURE_PORTS:
            if not pbuf[name]:
                continue
            best = None
            best_d = float("inf")
            for q in pbuf[name]:
                d = abs(q.t - imu.t)
                if d < best_d and d < MAX_DATA_AGE:
                    best_d = d
                    best = q
            if best:
                sp[name] = best

        if len(sp) != 3:
            time.sleep(SYNC_INTERVAL)
            continue

        pf, pl, pr = sp["front"].pressure_mbar, sp["left"].pressure_mbar, sp["right"].pressure_mbar
        line = row_tsv(imu, pf, pl, pr)
        with lock:
            rows.append(line)
        if not hdr_printed:
            print(row_tsv.__doc__ or "")  # noqa: placeholder
            hdr_printed = True
        print(line.replace("\t", " | "))
        time.sleep(SYNC_INTERVAL)


if __name__ == "__main__":
    main()