import struct
import ctypes
from ctypes import *
import time
from time import sleep

# 定义常量
VCI_USBCAN2 = 4
STATUS_OK = 1

# 定义初始化配置结构体
class VCI_INIT_CONFIG(Structure):
    _fields_ = [
        ("AccCode", c_ulong),
        ("AccMask", c_ulong),
        ("Reserved", c_ulong),
        ("Filter", c_ubyte),
        ("Timing0", c_ubyte),
        ("Timing1", c_ubyte),
        ("Mode", c_ubyte)
    ]

# 定义CAN数据帧结构体（标准帧格式）
class VCI_CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_ulong),        # 11位帧ID
        ("TimeStamp", c_ulong),
        ("TimeFlag", c_ubyte),
        ("SendType", c_ubyte),
        ("RemoteFlag", c_ubyte),
        ("ExternFlag", c_ubyte), # 0=标准帧，1=扩展帧
        ("DataLen", c_ubyte),    # DLC数据长度（1-8）
        ("Data", c_ubyte * 8),   # 数据域
        ("Reserved", c_ubyte * 3)
    ]

# 初始化配置（1Mbps波特率）
vci_initconfig = VCI_INIT_CONFIG(
    0x80000008,  # 验收码
    0xFFFFFFFF,  # 屏蔽码
    0,           # 保留字段
    0,           # 滤波方式
    0x00,        # 波特率定时器0 (1Mbps)
    0x14,        # 波特率定时器1 (1Mbps)
    0            # 工作模式 (0=正常模式)
)

# Windows系统加载DLL
canDLL = windll.LoadLibrary('./ControlCAN.dll')

# 设备初始化
ret = canDLL.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
if ret == STATUS_OK:
    print('调用 VCI_OpenDevice成功\r\n')
elif ret == -1:
    print('设备掉线\r\n')
elif ret != 0:
    print('链接设备失败\r\n')

# 初始化通道0
ret = canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 0, byref(vci_initconfig))
if ret == STATUS_OK:
    print('调用 VCI_InitCAN1成功\r\n')
else:
    print('调用 VCI_InitCAN1出错\r\n')

# 启动通道0
ret = canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 0)
if ret == STATUS_OK:
    print('调用 VCI_StartCAN1成功\r\n')
else:
    print('调用 VCI_StartCAN1出错\r\n')

class VCI_CAN_OBJ_ARRAY(Structure):
    _fields_ = [
        ('SIZE', ctypes.c_uint16),  # 数组大小
        ('STRUCT_ARRAY', ctypes.POINTER(VCI_CAN_OBJ))  # 结构体数组指针
    ]

    def __init__(self, num_of_structs):
        # 创建指定数量的结构体数组
        self.STRUCT_ARRAY = ctypes.cast(
            (VCI_CAN_OBJ * num_of_structs)(),
            ctypes.POINTER(VCI_CAN_OBJ)
        )
        self.SIZE = num_of_structs  # 设置数组大小
        self.ADDR = self.STRUCT_ARRAY[0]  # 获取数组首地址
# 以上部分不用管

def join_frame_id(id_num):
    return id_num|0x100


# 数据转换函数
def int_to_signed_bytes(value, n_bytes):

    if n_bytes < 1 or n_bytes > 8:
        raise ValueError("字节数必须在1-8之间")
    
    # 根据字节数确定数据范围
    min_val = -(2 ** (n_bytes * 8 - 1))
    max_val = 2 ** (n_bytes * 8 - 1) - 1
    
    if value < min_val or value > max_val:
        raise ValueError(f"值超出{n_bytes}字节有符号整数范围 [{min_val}, {max_val}]")
    
    # 转换为小端序字节
    if value < 0:
        value = (1 << (n_bytes * 8)) + value  # 补码表示
    
    bytes_list = []
    for i in range(n_bytes):
        bytes_list.append((value >> (i * 8)) & 0xFF)
    
    return bytes_list

def int_to_unsigned_bytes(value, n_bytes):
    """
    将整数转换为n个无符号字节（小端序）
    
    参数:
        value (int): 要转换的整数
        n_bytes (int): 字节数（1-8）
    
    返回:
        list: n个字节的列表（小端序）
    """
    if n_bytes < 1 or n_bytes > 8:
        raise ValueError("字节数必须在1-8之间")
    
    # 根据字节数确定无符号整数范围
    min_val = 0
    max_val = (2 ** (n_bytes * 8)) - 1
    
    if value < min_val or value > max_val:
        raise ValueError(f"值超出{n_bytes}字节无符号整数范围 [{min_val}, {max_val}]")
    
    # 转换为小端序字节
    bytes_list = []
    for i in range(n_bytes):
        bytes_list.append((value >> (i * 8)) & 0xFF)
    
    return bytes_list

def float_to_4byte_little_endian(value):

    packed = struct.pack('<f', value)  # '<f' 表示小端序float32
    return [byte for byte in packed]

def send_can_frame(frame_id, data_bytes):
    """
    发送CAN标准帧
    
    参数:
        frame_id (int): 11位帧ID（0x000-0x7FF）
        data_bytes (list): 数据字节列表（1-8字节）
    
    返回:
        int: 发送结果（1=成功，0=失败）
    """
    if frame_id < 0 or frame_id > 0x7FF:
        raise ValueError("标准帧ID必须在0x000-0x7FF范围内")
    
    if len(data_bytes) < 1 or len(data_bytes) > 8:
        raise ValueError("数据长度必须在1-8字节之间")
    
    # 准备数据数组
    data_array = (c_ubyte * 8)()
    for i, byte_val in enumerate(data_bytes):
        data_array[i] = byte_val
    
    # 创建CAN帧对象
    vci_can_obj = VCI_CAN_OBJ(
        frame_id,           # 11位帧ID
        0,                  # 时间戳
        0,                  # 时间标志
        0,                  # 发送类型（0=正常发送）
        0,                  # 远程帧标志（0=数据帧）
        0,                  # 帧格式（0=标准帧）
        len(data_bytes),    # DLC（数据长度）
        data_array,         # 数据
        (c_ubyte * 3)(0, 0, 0)  # 保留字段
    )
    
    # 发送CAN帧
    ret = canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)
    
    print(f"帧ID: 0x{frame_id:03X}")
    print(f"DLC: {len(data_bytes)}")
    print(f"数据: {' '.join([f'{b:02X}' for b in data_bytes])}")
    print(f"发送结果: {ret}")
    print("-" * 40)
    # print(vci_can_obj)
    
    return ret


def reboot(id_num):
    """发送重启命令到指定ID的设备"""
    send_can_frame(join_frame_id(id_num), [0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF])

def set_maxspeed(id_num, speed):
    """
    发送并设置最大速度
    
    参数:
        id_num (int): 设备ID号
        speed (int): 速度值（32位有符号整数）
    """
    bytes_list = int_to_unsigned_bytes(speed, 4)  # 转换为4字节小端序
    data = [0xB2] + bytes_list  # 命令码 + 速度字节
    send_can_frame(join_frame_id(id_num), data)

def set_abs_angle(id_num, angle):
    """
    发送并设置绝对角度
    
    参数:
        id_num (int): 设备ID号
        angle (float): 角度值（浮点数）
    """
    bytes_list = int_to_signed_bytes(angle,4)  # 转换为4字节小端序
    data = [0xC2] + bytes_list  # 命令码 + 角度字节
    send_can_frame(join_frame_id(id_num), data)

def set_relative_angle(id_num, angle):
    """
    发送并设置相对角度
    
    参数:
        id_num (int): 设备ID号
        angle (float): 角度值（浮点数）
    """
    bytes_list = int_to_signed_bytes(angle,4)  # 转换为4字节小端序
    data = [0xC3] + bytes_list  # 命令码 + 角度字节
    send_can_frame(join_frame_id(id_num), data)

def set_original_point(id_num):
    """
    设置当前位置为原点
    断电不保存

    参数:
        id_num (int): 设备ID号
    """
    send_can_frame(join_frame_id(id_num), [0xB1])  # 命令码 + 无数据

def set_speed(id_num, speed):
    """
    发送并设置速度
    
    参数:
        id_num (int): 设备ID号
        speed (float): 速度值（浮点数）
    """
    bytes_list = int_to_signed_bytes(speed,4)  # 转换为4字节小端序
    data = [0xC1] + bytes_list  # 命令码 + 速度字节
    send_can_frame(join_frame_id(id_num), data)

def back_to_zero(id_num):
    """
    发送以最短路径并设置回到零点
    
    参数:
        id_num (int): 设备ID号
    """
    send_can_frame(join_frame_id(id_num), [0xC4])
                                    
def shutdowm(id_num):
    """
    发送并设置关机
    
    参数:
        id_num (int): 设备ID号
    """
    send_can_frame(join_frame_id(id_num), [0xCF])

def parse_version_response(data):
    """
    解析版本响应数据
    
    参数:
        data (list): 接收到的8字节数据 [0xA0, boot_low, boot_high, app_low, app_high, hw_low, hw_high, can_ver]
    
    返回:
        dict: 解析后的版本信息
    """
    if len(data) < 8:
        print("版本响应数据长度不足")
        return None
    
    # 解析Boot软件版本 (2字节无符号，小端序)
    boot_version_raw = data[1] | (data[2] << 8)
    
    # 解析应用软件版本 (2字节无符号，小端序)
    app_version_raw = data[3] | (data[4] << 8)
    
    # 解析硬件版本 (2字节无符号，小端序)
    hardware_version_raw = data[5] | (data[6] << 8)
    
    # 解析CAN自定义版本 (1字节无符号)
    can_version = data[7]
    
    # 格式化版本号显示（通常版本号会按某种格式显示，如主版本.次版本）
    boot_version = f"{(boot_version_raw >> 8) & 0xFF}.{boot_version_raw & 0xFF}"
    app_version = f"{(app_version_raw >> 8) & 0xFF}.{app_version_raw & 0xFF}"
    hardware_version = f"{(hardware_version_raw >> 8) & 0xFF}.{hardware_version_raw & 0xFF}"
    
    result = {
        'boot_version': boot_version,
        'app_version': app_version,
        'hardware_version': hardware_version,
        'can_custom_version': can_version,
        'raw_data': {
            'boot_version_raw': boot_version_raw,
            'app_version_raw': app_version_raw,
            'hardware_version_raw': hardware_version_raw,
            'can_version_raw': can_version
        }
    }
    
    print("版本信息解析结果:")
    print(f"  Boot软件版本: {boot_version} (原始值: 0x{boot_version_raw:04X})")
    print(f"  应用软件版本: {app_version} (原始值: 0x{app_version_raw:04X})")
    print(f"  硬件版本: {hardware_version} (原始值: 0x{hardware_version_raw:04X})")
    print(f"  CAN自定义版本: {can_version} (原始值: 0x{can_version:02X})")
    
    return result

def version_request(id_num, timeout=1.0):
    """
    发送版本请求命令到指定ID的设备
    
    参数:
        id_num (int): 设备ID号
        timeout (float): 接收超时时间（秒）
    
    返回:
        dict: 解析后的版本信息字典，如果接收失败返回None
    """
    send_can_frame(join_frame_id(id_num), [0xA0])  # 命令码 + 无数据

    # 准备接收数据
    expected_frame_id = join_frame_id(id_num)
    start_time = time.time()

    print(f"等待设备 {id_num} 的版本响应数据...")

    # 创建接收缓冲区
    rx_vci_can_obj = VCI_CAN_OBJ_ARRAY(100)

    while True:
        # 检查超时
        if time.time() - start_time > timeout:
            print(f"接收设备 {id_num} 数据超时")
            return None
        
        # 从通道0接收数据
        ret = canDLL.VCI_Receive(VCI_USBCAN2, 0, 0, byref(rx_vci_can_obj.ADDR), 100, 0)
        
        if ret > 0:  # 接收到数据
            for i in range(ret):
                frame = rx_vci_can_obj.STRUCT_ARRAY[i]
                frame_id = frame.ID
                data_len = frame.DataLen
                data = list(frame.Data[:data_len])
                
                print(f"接收到帧: ID=0x{frame_id:03X}, DLC={data_len}, Data={[hex(d) for d in data]}")
                
                # 检查是否是目标设备的响应
                if join_frame_id(frame_id) == expected_frame_id and data_len == 8 and len(data) >= 8:
                    if data[0] == 0xA0:  # 确认是版本响应
                        return parse_version_response(data)
                    else:
                        print(f"收到设备 {id_num} 的数据，但命令码不匹配: 0x{data[0]:02X}")
        
        # 短暂休眠避免CPU占用过高
        time.sleep(0.001)


"""MIT协议相关函数"""
def bytes_to_unsigned_int(low_byte, high_byte):
    """
    将两个字节组合成无符号16位整数（小端序）
    
    参数:
        low_byte (int): 低字节
        high_byte (int): 高字节
    
    返回:
        int: 组合后的16位无符号整数
    """
    return low_byte | (high_byte << 8)

def MIT_join_frame_id(id_num):
    """
    组合帧ID，先与0x100进行或操作，然后将第10位（最高位）置为1
    
    参数:
        id_num (int): 输入的ID号
    
    返回:
        int: 处理后的11位帧ID，第10位为1
    """
    # 先进行原来的或操作
    result = id_num | 0x100
    
    # 将第10位置为1 (0x400 = 1024 = 2^10)
    result = result | 0x400
    
    return result

def parse_mit_response(data):
    """
    解析MIT响应数据
    
    参数:
        data (list): 接收到的7字节数据 [0xF0, pos_max_low, pos_max_high, vel_max_low, vel_max_high, torque_max_low, torque_max_high]
    
    返回:
        dict: 解析后的数据
    """
    if len(data) < 7:
        print("MIT响应数据长度不足")
        return None
    
    # 解析位置最大值（2字节无符号，小端序，单位0.1rad）
    pos_max_raw = data[1] | (data[2] << 8)
    pos_max = pos_max_raw * 0.1  # 转换为rad
    
    # 解析速度最大值（2字节无符号，小端序，单位0.01rad/s）
    vel_max_raw = data[3] | (data[4] << 8)
    vel_max = vel_max_raw * 0.01  # 转换为rad/s
    
    # 解析力矩最大值（2字节无符号，小端序，单位0.1Nm）
    torque_max_raw = data[5] | (data[6] << 8)
    torque_max = torque_max_raw * 0.1  # 转换为Nm
    
    result = {
        'position_max': pos_max,
        'velocity_max': vel_max,
        'torque_max': torque_max,
        'raw_data': {
            'position_max_raw': pos_max_raw,
            'velocity_max_raw': vel_max_raw,
            'torque_max_raw': torque_max_raw
        }
    }
    
    print("MIT响应数据解析结果:")
    print(f"  位置最大值: {pos_max:.1f} rad (原始值: {pos_max_raw})")
    print(f"  速度最大值: {vel_max:.2f} rad/s (原始值: {vel_max_raw})")
    print(f"  力矩最大值: {torque_max:.1f} Nm (原始值: {torque_max_raw})")
    
    return result

def pack_control_data(target_pos, target_vel, kp, kd, target_torque):
    """
    将控制参数打包为8字节数据
    
    参数:
        target_pos (float): 目标位置 (-Pos_Max ~ Pos_Max), 0对应32767.5
        target_vel (float): 目标速度 (-Vel_Max ~ Vel_Max), 0对应2047.5
        kp (float): 位置增益 (0 ~ 500)
        kd (float): 速度增益 (0 ~ 5)
        target_torque (float): 目标力矩 (-Torque_Max ~ Torque_Max), 0对应2047.5
    
    返回:
        list: 8字节数据列表
    """
    
    # 目标位置: 16bit, 32767.5代表0, 0代表-Pos_Max, 65535代表+Pos_Max
    pos_normalized = int(target_pos + 32767.5)
    pos_normalized = max(0, min(65535, pos_normalized))
    
    # 目标速度: 12bit, 2047.5代表0, 0代表-Vel_Max, 4095代表+Vel_Max
    vel_normalized = int(target_vel + 2047.5)
    vel_normalized = max(0, min(4095, vel_normalized))
    
    # 位置增益: (0 ~ 500) -> (0 ~ 4095)
    kp_normalized = int(kp / 500 * 4095)
    kp_normalized = max(0, min(4095, kp_normalized))
    
    # 速度增益: (0 ~ 5) -> (0 ~ 4095)
    kd_normalized = int(kd / 5 * 4095)
    kd_normalized = max(0, min(4095, kd_normalized))
    
    # 目标力矩: 12bit, 2047.5代表0, 0代表-Torque_Max, 4095代表+Torque_Max
    torque_normalized = int(target_torque + 2047.5)
    torque_normalized = max(0, min(4095, torque_normalized))
    
    # 创建8字节数组
    data = [0] * 8
    
    # Byte[0:1] - 目标位置 16bit (小端格式)
    data[0] = (pos_normalized >> 8) & 0xFF  # 高8位
    data[1] = pos_normalized & 0xFF         # 低8位
    
    # Byte[2] - 目标速度高8位
    data[2] = (vel_normalized >> 4) & 0xFF
    
    # Byte[3] - 目标速度低4位(bit[7:4]) + Kp高4位(bit[3:0])
    data[3] = ((vel_normalized & 0x0F) << 4) | ((kp_normalized >> 8) & 0x0F)
    
    # Byte[4] - Kp低8位
    data[4] = kp_normalized & 0xFF
    
    # Byte[5] - Kd高8位
    data[5] = (kd_normalized >> 4) & 0xFF
    
    # Byte[6] - Kd低4位(bit[7:4]) + 目标力矩高4位(bit[3:0])
    data[6] = ((kd_normalized & 0x0F) << 4) | ((torque_normalized >> 8) & 0x0F)
    
    # Byte[7] - 目标力矩低8位
    data[7] = torque_normalized & 0xFF
    
    # 打印调试信息
    print(f"打包控制数据:")
    print(f"  目标位置: {target_pos} -> {pos_normalized} (0x{pos_normalized:04X})")
    print(f"  目标速度: {target_vel} -> {vel_normalized} (0x{vel_normalized:03X})")
    print(f"  位置增益Kp: {kp} -> {kp_normalized} (0x{kp_normalized:03X})")
    print(f"  速度增益Kd: {kd} -> {kd_normalized} (0x{kd_normalized:03X})")
    print(f"  目标力矩: {target_torque} -> {torque_normalized} (0x{torque_normalized:03X})")
    print(f"  打包数据: {' '.join([f'0x{b:02X}' for b in data])}")
    
    return data

def parse_mit_cmd_response(data):
    """
    解析MIT命令响应数据
    
    参数:
        data (list): 接收到的7字节数据 [0xF1, pos_high, pos_low, vel_high, vel_torque, torque_low, status]
    
    返回:
        dict: 解析后的数据
    """
    if len(data) < 7:
        print("MIT命令响应数据长度不足")
        return None
    
    # 解析机械位置 16bit (小端格式，中点代表0)
    # Byte[1]为高8位，Byte[2]为低8位
    pos_raw = (data[1] << 8) | data[2]
    mechanical_position = pos_raw - 32767.5  # 32767.5代表0位置
    
    # 解析机械速度 12bit (小端格式，中点代表0)
    # Byte[3]为高8位，Byte[4]-Bit[7:4]为低4位
    vel_raw = (data[3] << 4) | ((data[4] >> 4) & 0x0F)
    mechanical_velocity = vel_raw - 2047.5  # 2047.5代表0速度
    
    # 解析力矩 12bit (小端格式，中点代表0)
    # Byte[4]-Bit[3:0]为高4位，Byte[5]为低8位
    torque_raw = ((data[4] & 0x0F) << 8) | data[5]
    torque = torque_raw - 2047.5  # 2047.5代表0力矩
    
    # 解析状态信息 1byte
    status_byte = data[6]
    is_motion_mode = bool(status_byte & 0x01)  # Bit0: 运控模式
    has_fault = bool(status_byte & 0x02)       # Bit1: 系统故障
    
    result = {
        'mechanical_position': mechanical_position,
        'mechanical_velocity': mechanical_velocity,
        'torque': torque,
        'status': {
            'is_motion_mode': is_motion_mode,
            'has_fault': has_fault,
            'status_byte': status_byte
        },
        'raw_data': {
            'position_raw': pos_raw,
            'velocity_raw': vel_raw,
            'torque_raw': torque_raw
        }
    }
    
    print("MIT命令响应数据解析结果:")
    print(f"  机械位置: {mechanical_position:.1f} (原始值: {pos_raw})")
    print(f"  机械速度: {mechanical_velocity:.1f} (原始值: {vel_raw})")
    print(f"  力矩: {torque:.1f} (原始值: {torque_raw})")
    print(f"  运控模式: {'是' if is_motion_mode else '否'}")
    print(f"  系统故障: {'是' if has_fault else '否'}")
    print(f"  状态字节: 0x{status_byte:02X}")
    
    return result

"""MIT"""
def MIT_set_max(maxangle,maxspeed,maxTorque,id_num):
    send_can_frame(join_frame_id(id_num), 
                   [0xF0]+
                   int_to_unsigned_bytes(maxangle, 2) + #0.1rad
                   int_to_unsigned_bytes(maxspeed, 2) + #0.01rad/s
                   int_to_unsigned_bytes(maxTorque, 2)) #0.01Nm
    
def MIT_request_max(id_num, timeout=1.0):
    print(f"发送MIT请求到设备 {id_num}")
    send_can_frame(join_frame_id(id_num), [0xF0])

    # 准备接收数据
    expected_frame_id = join_frame_id(id_num)
    start_time = time.time()

    print(f"等待设备 {id_num} 的MIT响应数据...")

    # 创建接收缓冲区
    rx_vci_can_obj = VCI_CAN_OBJ_ARRAY(100)
    
    while True:
        # 检查超时
        if time.time() - start_time > timeout:
            print(f"接收设备 {id_num} 数据超时")
            return None
        
        # 从通道0接收数据
        ret = canDLL.VCI_Receive(VCI_USBCAN2, 0, 0, byref(rx_vci_can_obj.ADDR), 100, 0)
        
        if ret > 0:  # 接收到数据
            for i in range(ret):
                frame = rx_vci_can_obj.STRUCT_ARRAY[i]
                frame_id = frame.ID
                data_len = frame.DataLen
                data = list(frame.Data[:data_len])
                
                print(f"接收到帧: ID=0x{frame_id:03X}, DLC={data_len}, Data={[hex(d) for d in data]}")
                
                # 检查是否是目标设备的响应
                if join_frame_id(frame_id) == expected_frame_id and data_len == 7 and len(data) >= 7:
                    if data[0] == 0xF0:  # 确认是MIT响应
                        return parse_mit_response(data)
                    else:
                        print(f"收到设备 {id_num} 的数据，但命令码不匹配: 0x{data[0]:02X}")
        
        # 短暂休眠避免CPU占用过高
        time.sleep(0.001)
        
def MIT_request_cmd(id_num, timeout=1.0):
    send_can_frame(join_frame_id(id_num), [0xF1])
    
    # 准备接收数据
    expected_frame_id = join_frame_id(id_num)
    start_time = time.time()

    print(f"等待设备 {id_num} 的MIT响应数据...")
    # 创建接收缓冲区
    rx_vci_can_obj = VCI_CAN_OBJ_ARRAY(100)
    
    while True:
        # 检查超时
        if time.time() - start_time > timeout:
            print(f"接收设备 {id_num} 数据超时")
            return None
        
        # 从通道0接收数据
        ret = canDLL.VCI_Receive(VCI_USBCAN2, 0, 0, byref(rx_vci_can_obj.ADDR), 100, 0)
        
        if ret > 0:  # 接收到数据
            for i in range(ret):
                frame = rx_vci_can_obj.STRUCT_ARRAY[i]
                frame_id = frame.ID
                data_len = frame.DataLen
                data = list(frame.Data[:data_len])
                
                print(f"接收到帧: ID=0x{frame_id:03X}, DLC={data_len}, Data={[hex(d) for d in data]}")
                
                # 检查是否是目标设备的响应
                if join_frame_id(frame_id) == expected_frame_id and data_len == 7 and len(data) >= 7:
                    if data[0] == 0xF1:  # 确认是MIT命令响应
                        return parse_mit_cmd_response(data)
                    else:
                        print(f"收到设备 {id_num} 的数据，但命令码不匹配: 0x{data[0]:02X}")
        
        # 短暂休眠避免CPU占用过高
        time.sleep(0.001)


def MIT_command(id_num,target_pos=0.0, target_vel=0.0, kp=0.0, kd=0.0, target_torque=0.0,timeout=1):
    """
    发送MIT命令到设备
    """
    # 这里可以添加具体的MIT命令逻辑
    frame_id = MIT_join_frame_id(id_num)

    data=pack_control_data(
        target_pos,  # 目标位置
        target_vel,  # 目标速度
        kp,          # 位置增益
        kd,          # 速度增益
        target_torque # 目标力矩
    )

    send_can_frame(frame_id, data)

    # 准备接收数据
    expected_frame_id = join_frame_id(id_num)
    start_time = time.time()

    print(f"等待设备 {id_num} 的MIT响应数据...")
    # 创建接收缓冲区
    rx_vci_can_obj = VCI_CAN_OBJ_ARRAY(100)

    while True:
        # 检查超时
        if time.time() - start_time > timeout:
            print(f"接收设备 {id_num} 数据超时")
            return None

        # 从通道0接收数据
        ret = canDLL.VCI_Receive(VCI_USBCAN2, 0, 0, byref(rx_vci_can_obj.ADDR), 100, 0)

        if ret > 0:  # 接收到数据
            for i in range(ret):
                frame = rx_vci_can_obj.STRUCT_ARRAY[i]
                frame_id = frame.ID
                data_len = frame.DataLen
                data = list(frame.Data[:data_len])

                print(f"接收到帧: ID=0x{frame_id:03X}, DLC={data_len}, Data={[hex(d) for d in data]}")

                # 检查是否是目标设备的响应
                if join_frame_id(frame_id) == expected_frame_id and data_len == 7 and len(data) >= 7:
                    if data[0] == 0xF1:  # 确认是MIT命令响应
                        return parse_mit_cmd_response(data)
                    else:
                        print(f"收到设备 {id_num} 的数据，但命令码不匹配: 0x{data[0]:02X}")

        # 短暂休眠避免CPU占用过高
        time.sleep(0.001)

# 测试示例
if __name__ == "__main__":
    # 所有的测试代码都应该在这里正确缩进
    pass  # 如果没有要执行的代码，使用pass占位
    
    # 以下是您原来的测试代码，现在都正确缩进了：
    # MIT_set_max(maxangle=125,maxspeed=6500,maxTorque=5000,id_num=1)
    # sleep(1)  # 等待设备处理命令
    # MIT_request_max(id_num=1, timeout=5.0)
    # sleep(1)
    # MIT_set_max(maxangle=900,maxspeed=100,maxTorque=180,id_num=3)
    # sleep(1)
    # MIT_request_max(id_num=3)
    # sleep(1)
    # MIT_command(id_num=3,target_pos=1800, target_vel=5,  kp=10.0, kd=0.1,target_torque=5)
    # sleep(3)  # 等待设备处理命令
    # MIT_request_max(id_num=3)
    # sleep(1)
    # MIT_request_cmd(id_num=3, timeout=5.0)
    # sleep(1)
    # set_relative_angle(3,-1048)
    # sleep(3)
    # set_relative_angle(2,-2048)
    # sleep(3)
    # set_relative_angle(1,-2048)
    # set_relative_angle(3,16384)
    # sleep(5)
    # set_abs_angle(3,int(16384*2.5))
    # set_abs_angle(3,1000)
    # set_abs_angle(2,-4096)
    # sleep(3)
    # set_maxspeed(3,300)  # 设置设备1的最大速度为1000

    # set_speed(3,1000)
    # sleep(5)
    # reboot(3)
    # sleep(5)
    # set_original_point(1)
    # set_relative_angle(3,180)
    # sleep(15)
    # set_abs_angle(1,1000)
    # sleep(10)
    # back_to_zero(1)
    # sleep(0.1)
    # back_to_zero(2)
    # sleep(0.1)
    # back_to_zero(3)
    # sleep(0.1)

    reboot(1)
    sleep(0.1)
    reboot(2)
    sleep(0.1)
    reboot(3)
    sleep(2)

    set_abs_angle(1,0)
    sleep(0.1)
    set_abs_angle(2,0)
    sleep(0.1)
    set_abs_angle(3,0)
    sleep(1)

    # back_to_zero(1)
    # back_to_zero(1)
    # sleep(15)
    # set_speed(1,2000)
    # sleep(10)
    # reboot(3)
    # shutdowm(1)
    # sleep(5)
    # version_request(1,timeout=5)