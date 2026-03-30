#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
合并传感器系统
同时读取IMU和三个压力传感器数据，并基于压力变化估计角度
输出格式：时间——yaw角度——z角速度——一号压力传感器——二号压力传感器——三号压力传感器——预测角度
"""

import serial
import threading
import time
from datetime import datetime
import re
from collections import deque
from dataclasses import dataclass
from typing import Optional, Dict, List
import queue
import signal
import sys
import numpy as np
import math

# 串口配置
IMU_PORT = '/dev/ttyUSB3'  # IMU串口（USB3）
IMU_BAUD_RATE = 9600

PRESSURE_PORTS = {
    'front': '/dev/ttyUSB0',   # 正前方压力传感器
    'left': '/dev/ttyUSB1',    # 左侧压力传感器
    'right': '/dev/ttyUSB2'    # 右侧压力传感器
}
PRESSURE_BAUD_RATE = 115200
TIMEOUT = 1

# 数据同步配置
SYNC_INTERVAL = 0.1  # 同步间隔（秒）
MAX_DATA_AGE = 0.5   # 数据最大年龄（秒），超过此时间的数据将被丢弃

# 传感器配置（单位：米）
TRIANGLE_SIDE_LENGTH = 0.2  # 等边三角形边长（米）
TRIANGLE_HEIGHT = TRIANGLE_SIDE_LENGTH * math.sqrt(3) / 2  # 等边三角形高度 ≈ 0.173m

# 数据队列用于时间同步
imu_queue = queue.Queue()
pressure_queues = {
    'front': queue.Queue(),
    'left': queue.Queue(),
    'right': queue.Queue()
}

# 压力传感器历史数据（用于角度估计）
pressure_history = {
    'front': deque(maxlen=50),
    'left': deque(maxlen=50),
    'right': deque(maxlen=50)
}
history_lock = threading.Lock()

# 压力预测角度的状态（用于积分，避免始终输出0）
_pred_yaw_state_deg = 0.0
_pred_last_pair_ts: Optional[float] = None
_pred_last_dp: Optional[float] = None  # 上一次(右-左)压力差，单位：mbar

# 在线标定（用 IMU 的 gyro_z 拟合 k_rate，使 k_rate * d(Pr-Pl)/dt ≈ gyro_z）
AUTO_CALIBRATE = True
_k_rate = 0.5  # (mbar/s) -> (deg/s) 初始系数（需要标定）
_k_rate_num = 0.0
_k_rate_den = 0.0
_k_rate_beta = 0.98  # 越接近1越平滑
_forward_threshold_mbar = 0.5  # 前向动压指示阈值（mbar），低于该值不更新标定

@dataclass
class IMUDataPacket:
    """IMU数据包"""
    timestamp: float
    angles: Dict[str, float]  # roll, pitch, yaw
    gyro: Dict[str, float]    # gyro_x, gyro_y, gyro_z
    accel: Dict[str, float]   # acc_x, acc_y, acc_z
    mag: Optional[Dict[str, float]] = None

@dataclass
class PressureDataPacket:
    """压力传感器数据包"""
    timestamp: float
    pressure: float  # mbar
    temperature: float  # °C
    depth: float  # m
    status: str

@dataclass
class PressureDataPoint:
    """压力数据点（用于历史记录）"""
    timestamp: float
    pressure: float  # mbar
    temperature: float  # °C

@dataclass
class AngleEstimate:
    """角度估计结果（基于压力变化）"""
    pred_yaw_inst: float       # “瞬时映射角”(deg)：由当前压差直接映射（不积分）
    pred_yaw: float  # “积分角”(deg)：由角速度积分得到（相对角，从启动时0累计）
    angular_velocity: float  # 角速度 (度/秒)：由压差变化率估计
    confidence: float  # 估计置信度 (0-1)
    dp_mbar: float = 0.0  # 右-左压差（mbar）
    dp_dt_mbar_s: float = 0.0  # 压差变化率（mbar/s）
    forward_dp_mbar: float = 0.0  # 前向动压指示（mbar）

@dataclass
class SyncedDataPacket:
    """同步后的完整数据包"""
    timestamp: float
    yaw: float  # IMU测量的偏航角（度）
    gyro_z: float  # Z轴角速度（度/秒）
    pressure_front: float
    pressure_left: float
    pressure_right: float
    pressure_front_temp: float
    pressure_left_temp: float
    pressure_right_temp: float
    pred_yaw: float  # 基于压力预测的偏航角（积分角，deg）
    pred_confidence: float  # 预测置信度
    pred_yaw_inst: float  # 基于压差映射的“瞬时角”(deg)

# 存储所有同步后的数据包（线程安全）
all_data_packets: List[SyncedDataPacket] = []
packets_lock = threading.Lock()

class JY901S_Parser:
    """JY901S IMU数据解析器"""
    def __init__(self):
        self.buffer = bytearray()
        self.packet_length = 11
        self.gravity = 9.8
        
    def update(self, data):
        """添加新数据到缓冲区"""
        self.buffer.extend(data)
        
    def find_packets(self):
        """在缓冲区中查找有效数据包"""
        packets = []
        
        while len(self.buffer) >= 2:
            if self.buffer[0] != 0x55:
                del self.buffer[0]
                continue
                
            data_type = self.buffer[1]
            if data_type not in [0x51, 0x52, 0x53, 0x54]:
                del self.buffer[0]
                continue
                
            if len(self.buffer) < self.packet_length:
                break
                
            packet = self.buffer[:self.packet_length]
            del self.buffer[:self.packet_length]
            
            if self.verify_checksum(packet):
                packets.append(packet)
                
        return packets
        
    def verify_checksum(self, packet):
        """验证数据包校验和"""
        calc_sum = sum(packet[:-1]) & 0xFF
        return calc_sum == packet[-1]
        
    def parse_packet(self, packet):
        """解析数据包"""
        data_type = packet[1]
        
        if data_type == 0x51:
            return self.parse_acceleration(packet)
        elif data_type == 0x52:
            return self.parse_angular_velocity(packet)
        elif data_type == 0x53:
            return self.parse_angles(packet)
        elif data_type == 0x54:
            return self.parse_magnetic_field(packet)
        return None
        
    def parse_acceleration(self, packet):
        """解析加速度数据包"""
        ax = self.bytes_to_int16(packet[3], packet[2])
        ay = self.bytes_to_int16(packet[5], packet[4])
        az = self.bytes_to_int16(packet[7], packet[6])
        
        scale = 16.0 * self.gravity / 32768.0
        ax = ax * scale
        ay = ay * scale
        az = az * scale
        
        temp = self.bytes_to_int16(packet[9], packet[8]) / 100.0
        
        return {
            'type': '加速度',
            'acc_x': ax,
            'acc_y': ay,
            'acc_z': az,
            'temperature': temp
        }
        
    def parse_angular_velocity(self, packet):
        """解析角速度数据包"""
        wx = self.bytes_to_int16(packet[3], packet[2])
        wy = self.bytes_to_int16(packet[5], packet[4])
        wz = self.bytes_to_int16(packet[7], packet[6])
        
        scale = 2000.0 / 32768.0
        wx = wx * scale
        wy = wy * scale
        wz = wz * scale
        
        voltage = self.bytes_to_int16(packet[9], packet[8]) / 100.0
        
        return {
            'type': '角速度',
            'gyro_x': wx,
            'gyro_y': wy,
            'gyro_z': wz,
            'voltage': voltage
        }
        
    def parse_angles(self, packet):
        """解析角度数据包"""
        roll = self.bytes_to_int16(packet[3], packet[2])
        pitch = self.bytes_to_int16(packet[5], packet[4])
        yaw = self.bytes_to_int16(packet[7], packet[6])
        
        scale = 180.0 / 32768.0
        roll = roll * scale
        pitch = pitch * scale
        yaw = yaw * scale
        
        version = self.bytes_to_int16(packet[9], packet[8])
        
        return {
            'type': '姿态角',
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'version': version
        }
        
    def parse_magnetic_field(self, packet):
        """解析磁场数据包"""
        hx = self.bytes_to_int16(packet[3], packet[2])
        hy = self.bytes_to_int16(packet[5], packet[4])
        hz = self.bytes_to_int16(packet[7], packet[6])
        
        temp = self.bytes_to_int16(packet[9], packet[8]) / 100.0
        
        return {
            'type': '磁场',
            'mag_x': hx,
            'mag_y': hy,
            'mag_z': hz,
            'temperature': temp
        }
        
    def bytes_to_int16(self, high_byte, low_byte):
        """将两个字节组合为有符号16位整数"""
        value = (high_byte << 8) | low_byte
        if value & 0x8000:
            value -= 0x10000
        return value

def hex_to_ascii(hex_data):
    """将HEX格式的数据转换为ASCII字符串"""
    try:
        if isinstance(hex_data, bytes):
            try:
                ascii_str = hex_data.decode('ascii', errors='ignore')
                return ascii_str
            except:
                ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in hex_data)
                return ascii_str
        elif isinstance(hex_data, str):
            try:
                bytes_data = bytes.fromhex(hex_data.replace(' ', '').replace('\n', '').replace('\r', ''))
                return bytes_data.decode('ascii', errors='ignore')
            except:
                return hex_data
        else:
            return str(hex_data)
    except Exception as e:
        return str(hex_data)

def clean_ascii(data):
    """清理ASCII数据"""
    data = data.replace('..', '\n')
    data = re.sub(r'\s+', ' ', data)
    data = re.sub(r'(?<!\n)\n(?!\n)', ' ', data)
    data = re.sub(r'C\.', '°C', data)
    data = re.sub(r'([a-zA-Z])(\d)', r'\1 \2', data)
    return data

def parse_pressure_data(data):
    """解析压力传感器数据"""
    results = {}
    cleaned_data = clean_ascii(data)

    final_match = re.search(r'Final result:\s*Temperature=([\d.]+)C,\s*Pressure=([\d.]+)\s*mbar', cleaned_data, re.IGNORECASE)
    if final_match:
        results['pressure'] = float(final_match.group(2))
        results['temperature'] = float(final_match.group(1))
        return results
    
    pressure_match = re.search(r'Pressure:\s*([\d.]+)\s*mbar', cleaned_data, re.IGNORECASE)
    if pressure_match:
        results['pressure'] = float(pressure_match.group(1))
    
    temp_match = re.search(r'Temperature:\s*([\d.]+)\s*°?C', cleaned_data, re.IGNORECASE)
    if temp_match:
        results['temperature'] = float(temp_match.group(1))
    
    depth_match = re.search(r'Water Depth:\s*([\d.-]+)\s*m', cleaned_data, re.IGNORECASE)
    if depth_match:
        results['depth'] = float(depth_match.group(1))
    
    status_match = re.search(r'Status:\s*(.+?)(?:\s*---|\s*$)', cleaned_data, re.IGNORECASE)
    if status_match:
        results['status'] = status_match.group(1).strip()

    return results

def estimate_angle_from_pressure() -> AngleEstimate:
    """
    基于压力变化和时间戳估计角度（xy平面转动）
    
    原理：
    - 当AUV在xy平面转动时，左右压力传感器的压力差会发生变化
    - 压力差的变化率可以反映角速度
    - 通过积分可以得到角度变化
    
    Returns:
        AngleEstimate: 角度估计结果
    """
    try:
        # 用“最近两组左右传感器匹配点”计算 dp/dt，并对角速度积分得到角度
        # 这样即使采样率较低/时间戳不完全对齐，也不会一直返回0
        global _pred_yaw_state_deg, _pred_last_pair_ts, _pred_last_dp, _k_rate

        with history_lock:
            if len(pressure_history['left']) < 1 or len(pressure_history['right']) < 1:
                return AngleEstimate(
                    pred_yaw_inst=0.0,
                    pred_yaw=_pred_yaw_state_deg,
                    angular_velocity=0.0,
                    confidence=0.0,
                    dp_mbar=0.0,
                    dp_dt_mbar_s=0.0,
                    forward_dp_mbar=0.0
                )

            left_data = list(pressure_history['left'])
            right_data = list(pressure_history['right'])
            front_data = list(pressure_history['front']) if len(pressure_history['front']) > 0 else []

        # 在最近N个点里做“最近邻时间匹配”
        N = 15
        time_match_threshold_s = 0.5  # 放宽到0.5s，避免匹配不到导致恒0

        best = None  # (pair_ts, dp, time_diff)
        for lp in left_data[-N:]:
            # 右边找一个最近的时间戳
            closest_rp = min(right_data[-N:], key=lambda rp: abs(rp.timestamp - lp.timestamp))
            td = abs(closest_rp.timestamp - lp.timestamp)
            if best is None or td < best[2]:
                best = ((lp.timestamp + closest_rp.timestamp) / 2.0, closest_rp.pressure - lp.pressure, td)

        if best is None:
            return AngleEstimate(
                pred_yaw_inst=0.0,
                pred_yaw=_pred_yaw_state_deg,
                angular_velocity=0.0,
                confidence=0.0,
                dp_mbar=0.0,
                dp_dt_mbar_s=0.0,
                forward_dp_mbar=0.0
            )

        pair_ts, dp_mbar, td = best

        # 前向动压指示：front - avg(left,right)（mbar）
        forward_dp_mbar = 0.0
        if front_data:
            fp = min(front_data[-N:], key=lambda p: abs(p.timestamp - pair_ts))
            if abs(fp.timestamp - pair_ts) < time_match_threshold_s:
                lp_nn = min(left_data[-N:], key=lambda p: abs(p.timestamp - pair_ts))
                rp_nn = min(right_data[-N:], key=lambda p: abs(p.timestamp - pair_ts))
                forward_dp_mbar = float(fp.pressure - 0.5 * (lp_nn.pressure + rp_nn.pressure))

        # “瞬时映射角”：用当前压差直接映射（便于理解/标定）
        # 注意：该角并不等同于IMU的raw yaw，只是压差->角度的线性近似
        angle_calibration_deg_per_mbar = 0.1  # 需要根据实际测试调整
        pred_yaw_inst = float(max(-180.0, min(180.0, dp_mbar * angle_calibration_deg_per_mbar)))

        # 计算角速度（mbar/s -> deg/s）
        angular_velocity = 0.0
        dp_dt_mbar_s = 0.0
        if _pred_last_pair_ts is not None and _pred_last_dp is not None:
            dt = pair_ts - _pred_last_pair_ts
            if dt > 1e-3:
                dp_dt = (dp_mbar - _pred_last_dp) / dt  # mbar/s
                dp_dt_mbar_s = float(dp_dt)
                angular_velocity = float(dp_dt * _k_rate)

                # 限幅
                angular_velocity = max(-180.0, min(180.0, angular_velocity))

                # 积分得到角度（预测角度是“相对角度”，从启动时0开始累计）
                _pred_yaw_state_deg += angular_velocity * dt
                _pred_yaw_state_deg = max(-180.0, min(180.0, _pred_yaw_state_deg))

        # 更新历史对
        _pred_last_pair_ts = pair_ts
        _pred_last_dp = dp_mbar

        # 置信度：匹配时间差越小越好；dp变化越大越好
        # td超过阈值直接降置信度，但仍输出估计值（避免恒0）
        time_score = max(0.0, 1.0 - (td / time_match_threshold_s))
        dp_score = min(1.0, abs(dp_mbar) / 5.0)  # 约5mbar认为“明显”
        confidence = float(max(0.0, min(1.0, 0.2 + 0.6 * time_score + 0.2 * dp_score)))

        return AngleEstimate(
            pred_yaw_inst=pred_yaw_inst,
            pred_yaw=_pred_yaw_state_deg,
            angular_velocity=angular_velocity,
            confidence=confidence,
            dp_mbar=float(dp_mbar),
            dp_dt_mbar_s=float(dp_dt_mbar_s),
            forward_dp_mbar=float(forward_dp_mbar)
        )

    except Exception as e:
        print(f"角度估计错误: {e}")
        import traceback
        traceback.print_exc()
        return AngleEstimate(
            pred_yaw_inst=0.0,
            pred_yaw=_pred_yaw_state_deg,
            angular_velocity=0.0,
            confidence=0.0,
            dp_mbar=0.0,
            dp_dt_mbar_s=0.0,
            forward_dp_mbar=0.0
        )


def update_k_rate_from_imu(dp_dt_mbar_s: float, imu_gyro_z_deg_s: float, forward_dp_mbar: float):
    """用 IMU 的 Z 轴角速度在线标定 _k_rate，使 _k_rate * dp_dt ≈ gyro_z（仅在前进时更新）"""
    global _k_rate, _k_rate_num, _k_rate_den
    if not AUTO_CALIBRATE:
        return
    if abs(forward_dp_mbar) < _forward_threshold_mbar:
        return
    if abs(dp_dt_mbar_s) < 1e-3:
        return

    _k_rate_num = _k_rate_beta * _k_rate_num + (1.0 - _k_rate_beta) * (imu_gyro_z_deg_s * dp_dt_mbar_s)
    _k_rate_den = _k_rate_beta * _k_rate_den + (1.0 - _k_rate_beta) * (dp_dt_mbar_s * dp_dt_mbar_s)
    if _k_rate_den > 1e-6:
        _k_rate = float(_k_rate_num / _k_rate_den)

def read_imu_serial():
    """读取IMU数据"""
    ser = None
    try:
        ser = serial.Serial(
            port=IMU_PORT,
            baudrate=IMU_BAUD_RATE,
            timeout=TIMEOUT,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        print(f"[IMU] 监听IMU传感器: {IMU_PORT}")
        ser.reset_input_buffer()
        
        parser = JY901S_Parser()
        latest_data = {}
        
        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                parser.update(data)
                
                packets = parser.find_packets()
                for packet in packets:
                    parsed = parser.parse_packet(packet)
                    if parsed:
                        latest_data[parsed['type']] = parsed
                        
                        # 当收集到所有必要数据时，创建数据包
                        if '姿态角' in latest_data and '角速度' in latest_data:
                            timestamp = time.time()
                            
                            angles = latest_data['姿态角']
                            gyro = latest_data['角速度']
                            accel = latest_data.get('加速度', {})
                            mag = latest_data.get('磁场', None)
                            
                            packet = IMUDataPacket(
                                timestamp=timestamp,
                                angles={
                                    'roll': angles['roll'],
                                    'pitch': angles['pitch'],
                                    'yaw': angles['yaw']
                                },
                                gyro={
                                    'gyro_x': gyro['gyro_x'],
                                    'gyro_y': gyro['gyro_y'],
                                    'gyro_z': gyro['gyro_z']
                                },
                                accel={
                                    'acc_x': accel.get('acc_x', 0),
                                    'acc_y': accel.get('acc_y', 0),
                                    'acc_z': accel.get('acc_z', 0)
                                },
                                mag=mag
                            )
                            
                            try:
                                imu_queue.put_nowait(packet)
                            except queue.Full:
                                try:
                                    imu_queue.get_nowait()
                                    imu_queue.put_nowait(packet)
                                except queue.Empty:
                                    pass
                                    
    except serial.SerialException as e:
        print(f"[IMU] IMU串口错误: {str(e)}")
    except Exception as e:
        print(f"[IMU] IMU处理错误: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        if ser and ser.is_open:
            ser.close()

def read_pressure_serial(port_name, serial_port):
    """读取压力传感器数据"""
    ser = None
    try:
        ser = serial.Serial(
            port=serial_port,
            baudrate=PRESSURE_BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=TIMEOUT
        )
        print(f"[{port_name}] 监听压力传感器: {serial_port}")
        ser.reset_input_buffer()
        current_packet = b""
        packet_started = False
        last_data_time = time.time()

        while True:
            if ser.in_waiting > 0:
                raw_data = ser.read(ser.in_waiting)
                current_packet += raw_data
                last_data_time = time.time()
            else:
                if time.time() - last_data_time > 2.0 and current_packet:
                    if packet_started:
                        try:
                            ascii_packet = hex_to_ascii(current_packet)
                            if "Final result" in ascii_packet or "Pressure:" in ascii_packet:
                                parsed = parse_pressure_data(ascii_packet)
                                if 'pressure' in parsed:
                                    timestamp = time.time()
                                    packet = PressureDataPacket(
                                        timestamp=timestamp,
                                        pressure=parsed['pressure'],
                                        temperature=parsed.get('temperature', 0.0),
                                        depth=parsed.get('depth', 0.0),
                                        status=parsed.get('status', 'Unknown')
                                    )
                                    
                                    # 保存到历史记录
                                    with history_lock:
                                        pressure_history[port_name].append(PressureDataPoint(
                                            timestamp=timestamp,
                                            pressure=parsed['pressure'],
                                            temperature=parsed.get('temperature', 0.0)
                                        ))
                                    
                                    try:
                                        pressure_queues[port_name].put_nowait(packet)
                                    except queue.Full:
                                        try:
                                            pressure_queues[port_name].get_nowait()
                                            pressure_queues[port_name].put_nowait(packet)
                                        except queue.Empty:
                                            pass
                        except:
                            pass
                        current_packet = b""
                        packet_started = False
                time.sleep(0.01)
                continue
            
            try:
                ascii_packet = hex_to_ascii(current_packet)
                
                if "===" in ascii_packet and not packet_started:
                    packet_started = True
                    start_idx = ascii_packet.find("===")
                    if start_idx > 0:
                        current_packet = current_packet[start_idx:]
                        ascii_packet = hex_to_ascii(current_packet)
                
                if packet_started and "=====================================" in ascii_packet:
                    parsed = parse_pressure_data(ascii_packet)
                    if 'pressure' in parsed:
                        timestamp = time.time()
                        packet = PressureDataPacket(
                            timestamp=timestamp,
                            pressure=parsed['pressure'],
                            temperature=parsed.get('temperature', 0.0),
                            depth=parsed.get('depth', 0.0),
                            status=parsed.get('status', 'Unknown')
                        )
                        
                        # 保存到历史记录
                        with history_lock:
                            pressure_history[port_name].append(PressureDataPoint(
                                timestamp=timestamp,
                                pressure=parsed['pressure'],
                                temperature=parsed.get('temperature', 0.0)
                            ))
                        
                        try:
                            pressure_queues[port_name].put_nowait(packet)
                        except queue.Full:
                            try:
                                pressure_queues[port_name].get_nowait()
                                pressure_queues[port_name].put_nowait(packet)
                            except queue.Empty:
                                pass
                    
                    current_packet = b""
                    packet_started = False
                
                if len(current_packet) > 2000:
                    current_packet = b""
                    packet_started = False
                    
            except Exception as e:
                pass

    except serial.SerialException as e:
        print(f"[{port_name}] 压力传感器串口错误: {str(e)}")
    except Exception as e:
        print(f"[{port_name}] 压力传感器处理错误: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        if ser and ser.is_open:
            ser.close()

def synchronize_and_display():
    """同步数据并显示（包含预测角度）"""
    print("=" * 140)
    print("合并传感器数据监测系统（xy平面转动模式 + 压力预测角度）")
    print("=" * 140)
    print("时间                    Yaw角度               Z轴角速度              一号压力传感器(前)    二号压力传感器(左)    三号压力传感器(右)    pred_yaw / pred_yaw_inst")
    print("-" * 140)
    
    imu_buffer = deque(maxlen=10)
    pressure_buffers = {
        'front': deque(maxlen=10),
        'left': deque(maxlen=10),
        'right': deque(maxlen=10)
    }
    
    while True:
        current_time = time.time()
        
        # 从队列中获取最新数据
        try:
            while True:
                imu_packet = imu_queue.get_nowait()
                imu_buffer.append(imu_packet)
        except queue.Empty:
            pass
        
        for port_name in ['front', 'left', 'right']:
            try:
                while True:
                    pressure_packet = pressure_queues[port_name].get_nowait()
                    pressure_buffers[port_name].append(pressure_packet)
            except queue.Empty:
                pass
        
        # 查找时间同步的数据
        if len(imu_buffer) > 0:
            latest_imu = imu_buffer[-1]
            
            # 查找最接近IMU时间戳的压力数据
            synced_pressures = {}
            for port_name in ['front', 'left', 'right']:
                if len(pressure_buffers[port_name]) > 0:
                    best_match = None
                    min_time_diff = float('inf')
                    
                    for pressure_packet in pressure_buffers[port_name]:
                        time_diff = abs(pressure_packet.timestamp - latest_imu.timestamp)
                        if time_diff < min_time_diff and time_diff < MAX_DATA_AGE:
                            min_time_diff = time_diff
                            best_match = pressure_packet
                    
                    if best_match:
                        synced_pressures[port_name] = best_match
            
            # 如果所有数据都可用，显示并保存
            if len(synced_pressures) == 3:
                timestamp_str = datetime.fromtimestamp(latest_imu.timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                
                # IMU数据
                yaw = latest_imu.angles['yaw']
                yaw_str = f"Yaw:{yaw:7.2f}°"
                gyro_z = latest_imu.gyro['gyro_z']
                gyro_z_str = f"Z:{gyro_z:7.2f}°/s"
                
                # 压力传感器数据
                front_pressure = synced_pressures['front'].pressure
                left_pressure = synced_pressures['left'].pressure
                right_pressure = synced_pressures['right'].pressure
                
                # 基于压力变化预测角度
                angle_estimate = estimate_angle_from_pressure()

                # 在线标定：让压力推出来的角速度幅值尽量贴近IMU gyro_z（只在前进时更新）
                update_k_rate_from_imu(
                    dp_dt_mbar_s=angle_estimate.dp_dt_mbar_s,
                    imu_gyro_z_deg_s=gyro_z,
                    forward_dp_mbar=angle_estimate.forward_dp_mbar
                )

                predicted_yaw = angle_estimate.pred_yaw
                predicted_yaw_inst = angle_estimate.pred_yaw_inst
                predicted_yaw_str = (
                    f"pred_yaw:{predicted_yaw:7.2f}° "
                    f"pred_yaw_inst:{predicted_yaw_inst:7.2f}° "
                    f"(c={angle_estimate.confidence:.2f},k={_k_rate:.3f},fwd={angle_estimate.forward_dp_mbar:.2f}mbar)"
                )
                
                # 格式化输出：时间——yaw角度——z角速度——一号压力传感器——二号压力传感器——三号压力传感器——预测角度
                print(f"{timestamp_str}  {yaw_str:20s}  {gyro_z_str:20s}  {front_pressure:7.2f}mbar  {left_pressure:7.2f}mbar  {right_pressure:7.2f}mbar  {predicted_yaw_str:20s}")
                
                # 保存同步后的数据包
                synced_packet = SyncedDataPacket(
                    timestamp=latest_imu.timestamp,
                    yaw=yaw,
                    gyro_z=gyro_z,
                    pressure_front=front_pressure,
                    pressure_left=left_pressure,
                    pressure_right=right_pressure,
                    pressure_front_temp=synced_pressures['front'].temperature,
                    pressure_left_temp=synced_pressures['left'].temperature,
                    pressure_right_temp=synced_pressures['right'].temperature,
                    pred_yaw=predicted_yaw,
                    pred_confidence=angle_estimate.confidence,
                    pred_yaw_inst=predicted_yaw_inst
                )
                
                with packets_lock:
                    all_data_packets.append(synced_packet)
        
        time.sleep(SYNC_INTERVAL)

def save_data_to_file(output_file):
    """将所有数据包保存到文件"""
    try:
        with packets_lock:
            if len(all_data_packets) == 0:
                print("没有数据包需要保存")
                return
            
            print(f"\n正在保存 {len(all_data_packets)} 个数据包到文件: {output_file}")
            
            with open(output_file, 'w', encoding='utf-8') as f:
                # 写入文件头
                f.write("=" * 140 + "\n")
                f.write("合并传感器数据监测系统 - 数据日志\n")
                f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"数据包总数: {len(all_data_packets)}\n")
                f.write("=" * 140 + "\n\n")
                
                # 写入表头
                f.write("时间                    Yaw角度               Z轴角速度              一号压力传感器(前)    二号压力传感器(左)    三号压力传感器(右)    pred_yaw / pred_yaw_inst\n")
                f.write("-" * 140 + "\n")
                
                # 写入所有数据包
                for packet in all_data_packets:
                    timestamp_str = datetime.fromtimestamp(packet.timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    
                    yaw_str = f"Yaw:{packet.yaw:7.2f}°"
                    gyro_z_str = f"Z:{packet.gyro_z:7.2f}°/s"
                    predicted_yaw_str = f"pred_yaw:{packet.pred_yaw:7.2f}° pred_yaw_inst:{packet.pred_yaw_inst:7.2f}°"
                    
                    f.write(f"{timestamp_str}  {yaw_str:20s}  {gyro_z_str:20s}  {packet.pressure_front:7.2f}mbar  {packet.pressure_left:7.2f}mbar  {packet.pressure_right:7.2f}mbar  {predicted_yaw_str:20s}\n")
                
                # 写入详细数据
                f.write("\n" + "=" * 140 + "\n")
                f.write("详细数据（包含温度信息和预测置信度）\n")
                f.write("=" * 140 + "\n\n")
                
                for i, packet in enumerate(all_data_packets, 1):
                    timestamp_str = datetime.fromtimestamp(packet.timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    f.write(f"\n数据包 #{i} - {timestamp_str}\n")
                    f.write(f"  IMU Yaw角度: {packet.yaw:.2f}°\n")
                    f.write(f"  IMU Z轴角速度: {packet.gyro_z:.2f}°/s\n")
                    f.write(f"  压力传感器(前): {packet.pressure_front:.2f}mbar, 温度: {packet.pressure_front_temp:.2f}°C\n")
                    f.write(f"  压力传感器(左): {packet.pressure_left:.2f}mbar, 温度: {packet.pressure_left_temp:.2f}°C\n")
                    f.write(f"  压力传感器(右): {packet.pressure_right:.2f}mbar, 温度: {packet.pressure_right_temp:.2f}°C\n")
                    f.write(f"  pred_yaw(积分角): {packet.pred_yaw:.2f}°, pred_yaw_inst(瞬时映射): {packet.pred_yaw_inst:.2f}°, 置信度: {packet.pred_confidence:.3f}\n")
                    f.write("-" * 140 + "\n")
            
            print(f"数据已成功保存到文件: {output_file}")
            print(f"共保存 {len(all_data_packets)} 个数据包")
            
    except Exception as e:
        print(f"保存文件时发生错误: {e}")
        import traceback
        traceback.print_exc()

def main():
    """主程序"""
    # 生成输出文件名（格式：时间—datalog）
    timestamp_str = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    output_file = f"{timestamp_str}—datalog.txt"
    
    # 创建信号处理函数（使用闭包传递output_file）
    def signal_handler_wrapper(sig, frame):
        print("\n\n接收到退出信号，正在保存数据...")
        save_data_to_file(output_file)
        sys.exit(0)
    
    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler_wrapper)
    signal.signal(signal.SIGTERM, signal_handler_wrapper)
    
    # 启动IMU线程
    imu_thread = threading.Thread(target=read_imu_serial, daemon=True)
    imu_thread.start()
    
    # 启动压力传感器线程
    pressure_threads = []
    for name, port in PRESSURE_PORTS.items():
        thread = threading.Thread(target=read_pressure_serial, args=(name, port), daemon=True)
        pressure_threads.append(thread)
        thread.start()
    
    # 等待传感器初始化
    time.sleep(2)
    
    print("\n所有传感器已启动，开始数据同步...")
    print(f"数据将保存到文件: {output_file}")
    print("按 Ctrl+C 退出并保存数据\n")
    
    try:
        # 启动数据同步和显示线程
        synchronize_and_display()
    except KeyboardInterrupt:
        print("\n\n程序终止，正在保存数据...")
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        save_data_to_file(output_file)
        print("合并传感器监测系统退出")

if __name__ == "__main__":
    main()
