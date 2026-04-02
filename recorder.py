#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一传感器读取脚本
同时读取IMU和三个压力传感器的数据，并同步时间戳
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

# 串口配置
IMU_PORT = '/dev/ttyACM0'
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

# 全局数据存储（线程安全）
data_lock = threading.Lock()
imu_data = {
    'angles': None,      # 姿态角
    'gyro': None,        # 角速度
    'accel': None,       # 加速度
    'mag': None,         # 磁场
    'timestamp': None
}
pressure_data = {
    'front': None,
    'left': None,
    'right': None
}

# 数据队列用于时间同步
imu_queue = queue.Queue()
pressure_queues = {
    'front': queue.Queue(),
    'left': queue.Queue(),
    'right': queue.Queue()
}

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
class SyncedDataPacket:
    """同步后的完整数据包（仅xy平面转动，只保留yaw和z轴角速度）"""
    timestamp: float
    yaw: float  # 偏航角（度）
    gyro_z: float  # Z轴角速度（度/秒）
    pressure_front: float
    pressure_left: float
    pressure_right: float
    pressure_front_temp: float
    pressure_left_temp: float
    pressure_right_temp: float

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
                                # 队列满时，移除最旧的数据
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
    """同步数据并显示（仅xy平面转动）"""
    print("=" * 120)
    print("统一传感器数据监测系统（xy平面转动模式）")
    print("=" * 120)
    print("时间                    Yaw角度               Z轴角速度              一号压力传感器(前)    二号压力传感器(左)    三号压力传感器(右)")
    print("-" * 120)
    
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
                    # 找到时间差最小的压力数据
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
                
                # 只保留Yaw角度
                yaw = latest_imu.angles['yaw']
                yaw_str = f"Yaw:{yaw:7.2f}°"
                
                # 只保留Z轴角速度
                gyro_z = latest_imu.gyro['gyro_z']
                gyro_z_str = f"Z:{gyro_z:7.2f}°/s"
                
                # 压力传感器数据
                front_pressure = synced_pressures['front'].pressure
                left_pressure = synced_pressures['left'].pressure
                right_pressure = synced_pressures['right'].pressure
                
                # 格式化输出：时间——Yaw角度——Z轴角速度——一号压力传感器数据（前）——2号压力传感器数据——三号压力传感器数据
                print(f"{timestamp_str}  {yaw_str:20s}  {gyro_z_str:20s}  {front_pressure:7.2f}mbar  {left_pressure:7.2f}mbar  {right_pressure:7.2f}mbar")
                
                # 保存同步后的数据包（只保存yaw和gyro_z）
                synced_packet = SyncedDataPacket(
                    timestamp=latest_imu.timestamp,
                    yaw=yaw,
                    gyro_z=gyro_z,
                    pressure_front=front_pressure,
                    pressure_left=left_pressure,
                    pressure_right=right_pressure,
                    pressure_front_temp=synced_pressures['front'].temperature,
                    pressure_left_temp=synced_pressures['left'].temperature,
                    pressure_right_temp=synced_pressures['right'].temperature
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
                f.write("=" * 120 + "\n")
                f.write("统一传感器数据监测系统 - 数据日志\n")
                f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"数据包总数: {len(all_data_packets)}\n")
                f.write("=" * 120 + "\n\n")
                
                # 写入表头
                f.write("时间                    Yaw角度               Z轴角速度              一号压力传感器(前)    二号压力传感器(左)    三号压力传感器(右)\n")
                f.write("-" * 120 + "\n")
                
                # 写入所有数据包
                for packet in all_data_packets:
                    timestamp_str = datetime.fromtimestamp(packet.timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    
                    # Yaw角度
                    yaw_str = f"Yaw:{packet.yaw:7.2f}°"
                    
                    # Z轴角速度
                    gyro_z_str = f"Z:{packet.gyro_z:7.2f}°/s"
                    
                    # 压力传感器数据
                    front_pressure = packet.pressure_front
                    left_pressure = packet.pressure_left
                    right_pressure = packet.pressure_right
                    
                    # 写入数据行
                    f.write(f"{timestamp_str}  {yaw_str:20s}  {gyro_z_str:20s}  {front_pressure:7.2f}mbar  {left_pressure:7.2f}mbar  {right_pressure:7.2f}mbar\n")
                
                # 写入详细数据（包含温度信息）
                f.write("\n" + "=" * 120 + "\n")
                f.write("详细数据（包含温度信息）\n")
                f.write("=" * 120 + "\n\n")
                
                for i, packet in enumerate(all_data_packets, 1):
                    timestamp_str = datetime.fromtimestamp(packet.timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    f.write(f"\n数据包 #{i} - {timestamp_str}\n")
                    f.write(f"  Yaw角度: {packet.yaw:.2f}°\n")
                    f.write(f"  Z轴角速度: {packet.gyro_z:.2f}°/s\n")
                    f.write(f"  压力传感器(前): {packet.pressure_front:.2f}mbar, 温度: {packet.pressure_front_temp:.2f}°C\n")
                    f.write(f"  压力传感器(左): {packet.pressure_left:.2f}mbar, 温度: {packet.pressure_left_temp:.2f}°C\n")
                    f.write(f"  压力传感器(右): {packet.pressure_right:.2f}mbar, 温度: {packet.pressure_right_temp:.2f}°C\n")
                    f.write("-" * 120 + "\n")
            
            print(f"数据已成功保存到文件: {output_file}")
            print(f"共保存 {len(all_data_packets)} 个数据包")
            
    except Exception as e:
        print(f"保存文件时发生错误: {e}")
        import traceback
        traceback.print_exc()

def signal_handler(sig, frame, output_file):
    """信号处理函数，用于优雅退出"""
    print("\n\n接收到退出信号，正在保存数据...")
    save_data_to_file(output_file)
    sys.exit(0)

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
        print("统一传感器监测系统退出")

if __name__ == "__main__":
    main()
