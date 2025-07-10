"""
CAN电机控制脚本
整合8108电机（8位定长数据）和6048电机（可变长度数据）的控制功能
波特率: 1000kbps
"""

import ctypes
from ctypes import *
import time
from time import sleep
import struct

# ==============================
# CAN设备类型和状态定义
# ==============================
VCI_USBCAN2 = 4
STATUS_OK = 1

# ==============================
# CAN数据结构定义
# ==============================
class VCI_INIT_CONFIG(Structure):
    _fields_ = [("AccCode", c_ulong),
                ("AccMask", c_ulong),
                ("Reserved", c_ulong),
                ("Filter", c_ubyte),
                ("Timing0", c_ubyte),
                ("Timing1", c_ubyte),
                ("Mode", c_ubyte)]

class VCI_CAN_OBJ(Structure):
    _fields_ = [("ID", c_ulong),
                ("TimeStamp", c_ulong),
                ("TimeFlag", c_ubyte),
                ("SendType", c_ubyte),
                ("RemoteFlag", c_ubyte),
                ("ExternFlag", c_ubyte),
                ("DataLen", c_ubyte),
                ("Data", c_ubyte * 8),
                ("Reserved", c_ubyte * 3)]

class VCI_CAN_OBJ_ARRAY(Structure):
    _fields_ = [
        ('SIZE', ctypes.c_uint16),
        ('STRUCT_ARRAY', ctypes.POINTER(VCI_CAN_OBJ))
    ]

    def __init__(self, num_of_structs):
        self.STRUCT_ARRAY = ctypes.cast(
            (VCI_CAN_OBJ * num_of_structs)(),
            ctypes.POINTER(VCI_CAN_OBJ)
        )
        self.SIZE = num_of_structs
        self.ADDR = self.STRUCT_ARRAY[0]

# ==============================
# CAN配置 - 1000kbps波特率
# ==============================
# 1000kbps波特率配置: Timing0=0x00, Timing1=0x14
vci_initconfig = VCI_INIT_CONFIG(
    0x80000008,    # AccCode
    0xFFFFFFFF,    # AccMask
    0,             # Reserved
    0,             # Filter
    0x00,          # Timing0 (1000kbps)
    0x14,          # Timing1 (1000kbps)
    0              # Mode
)

# 加载CAN DLL
canDLL = windll.LoadLibrary('./ControlCAN.dll')

# ==============================
# 8108电机控制常量和数据
# ==============================
# 8108电机命令ID (ODrive协议)
Set_Controller_Mode_8108 = 0x0B  # 设置控制模式
Set_Axis_State_8108 = 0x07       # 设置轴状态
Set_Input_Pos_8108 = 0x0C        # 设置输入位置
Reboot_8108 = 0x16               # 重启

# 8108电机控制模式数据（8字节定长）
ubyte_array = c_ubyte * 8
mode_pos_trapezium_8108 = ubyte_array(3, 0, 0, 0, 5, 0, 0, 0)  # 位置梯形模式
closed_loop_8108 = ubyte_array(8, 0, 0, 0, 0, 0, 0, 0)         # 闭环控制
axis_stop_8108 = ubyte_array(1, 0, 0, 0, 0, 0, 0, 0)           # 停止
motor_calibration_8108 = ubyte_array(4, 0, 0, 0, 0, 0, 0, 0)   # 电机校准
encoder_calibration_8108 = ubyte_array(7, 0, 0, 0, 0, 0, 0, 0) # 编码器校准

# ==============================
# 6048电机控制常量
# ==============================
# 6048电机命令类型
CMD_SET_MAXSPEED_6048 = 0xB2     # 设置最大速度
CMD_SET_ABS_ANGLE_6048 = 0xC2    # 设置绝对角度
CMD_SET_REL_ANGLE_6048 = 0xC3    # 设置相对角度
CMD_SET_ORIGINAL_POINT_6048 = 0xB1 # 设置原点
CMD_SET_SPEED_6048 = 0xC1        # 设置速度
CMD_BACK_TO_ZERO_6048 = 0xC4     # 回到零点


# ==============================
# 通用数据转换函数
# ==============================
def float32_to_bytes(value):
    """将float32转换为4字节列表（小端序）"""
    byte_data = struct.pack('<f', value)
    return list(byte_data)

def int_to_signed_bytes(value, byte_count):
    """将有符号整数转换为指定字节数的列表"""
    if byte_count == 1:
        return [(value & 0xFF)]
    elif byte_count == 2:
        return [value & 0xFF, (value >> 8) & 0xFF]
    elif byte_count == 4:
        return [value & 0xFF, (value >> 8) & 0xFF, 
                (value >> 16) & 0xFF, (value >> 24) & 0xFF]
    else:
        raise ValueError("不支持的字节数")

def int_to_unsigned_bytes(value, n_bytes):
    """将整数转换为n个无符号字节（小端序）"""
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
# ==============================
# CAN帧发送和接收函数
# ==============================
def send_can_frame(frame_id, data_bytes):
    """发送CAN标准帧"""
    if frame_id < 0 or frame_id > 0x7FF:
        raise ValueError("标准帧ID必须在0x000-0x7FF范围内")
    
    if len(data_bytes) < 0 or len(data_bytes) > 8:
        raise ValueError("数据长度必须在0-8字节之间")
    
    data_len = len(data_bytes)
    data_array = (c_ubyte * 8)()
    
    for i in range(data_len):
        data_array[i] = data_bytes[i]
    
    vci_can_obj = VCI_CAN_OBJ(
        frame_id, 0, 0, 0, 0, 0, data_len, data_array, (c_ubyte * 3)(0, 0, 0)
    )
    
    ret = canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)
    
    print(f"发送CAN帧:")
    print(f"  帧ID: 0x{frame_id:03X}")
    print(f"  DLC: 0x{data_len:02X} ({data_len} bytes)")
    if data_len > 0:
        print(f"  数据: {' '.join([f'0x{b:02X}' for b in data_bytes])}")
    else:
        print("  数据: 无数据")
    print(f"  发送结果: {ret} ({'成功' if ret == 1 else '失败'})")
    print("-" * 50)
    
    return ret

def encode_can_id(target_id, command_id):
    """编码CAN ID"""
    if not (0 <= target_id <= 0x3F):
        raise ValueError("target_id 必须是6位值 (0-0x3F)")
    if not (0 <= command_id <= 0x1F):
        raise ValueError("command_id 必须是5位值 (0-0x1F)")
    
    return (target_id << 5) | command_id

def join_6048_frame_id(node_id):
    """生成6048电机的标准帧ID (修复后的函数)"""
    return node_id | 0x100

# ==============================
# 8108电机控制函数
# ==============================
def set_8108_controller_mode(target_id, input_mode):
    """设置8108电机控制模式和输入模式"""
    can_id = encode_can_id(target_id, Set_Controller_Mode_8108)
    print(f"8108电机 - 设置控制模式, CAN_ID: 0x{can_id:03X}")
    data_bytes = [input_mode[i] for i in range(8)]
    return send_can_frame(can_id, data_bytes)

def set_8108_closed_loop(target_id):
    """设置8108电机闭环控制模式"""
    can_id = encode_can_id(target_id, Set_Axis_State_8108)
    print(f"8108电机 - 设置闭环控制, CAN_ID: 0x{can_id:03X}")
    data_bytes = [closed_loop_8108[i] for i in range(8)]
    return send_can_frame(can_id, data_bytes)

def set_8108_position(target_id, input_pos):
    """设置8108电机位置（梯形曲线控制）"""
    can_id = encode_can_id(target_id, Set_Input_Pos_8108)
    print(f"8108电机 - 设置位置 {input_pos}, CAN_ID: 0x{can_id:03X}")
    float_bytes = float32_to_bytes(input_pos)
    data_bytes = float_bytes + [0, 0, 0, 0]
    return send_can_frame(can_id, data_bytes)

def set_8108_stop(target_id):
    """停止8108电机控制"""
    can_id = encode_can_id(target_id, Set_Axis_State_8108)
    print(f"8108电机 - 停止控制, CAN_ID: 0x{can_id:03X}")
    data_bytes = [axis_stop_8108[i] for i in range(8)]
    return send_can_frame(can_id, data_bytes)

def calibrate_8108(target_id):
    """校准8108电机编码器与电机"""
    can_id = encode_can_id(target_id, Set_Axis_State_8108)
    print(f"8108电机 - 校准, CAN_ID: 0x{can_id:03X}")
    
    # 校准电机
    motor_data_bytes = [motor_calibration_8108[i] for i in range(8)]
    ret1 = send_can_frame(can_id, motor_data_bytes)
    print("校准电机：", ret1)
    
    sleep(0.1)
    
    # 校准编码器
    encoder_data_bytes = [encoder_calibration_8108[i] for i in range(8)]
    ret2 = send_can_frame(can_id, encoder_data_bytes)
    print("校准编码器：", ret2)
    
    return ret1 and ret2

def reboot_8108(target_id):
    """重启8108电机"""
    can_id = encode_can_id(target_id, Reboot_8108)
    print(f"8108电机 - 重启, CAN_ID: 0x{can_id:03X}")
    data_bytes = [axis_stop_8108[i] for i in range(8)]
    return send_can_frame(can_id, data_bytes)

# ==============================
# 6048电机控制函数
# ==============================
def set_6048_max_speed(node_id, max_speed):
    """设置6048电机最大速度"""
    frame_id = join_6048_frame_id(node_id)
    print(f"6048电机 - 设置最大速度 {max_speed}, Frame_ID: 0x{frame_id:03X}")
    
    cmd_data = [CMD_SET_MAXSPEED_6048]
    speed_data = int_to_unsigned_bytes(max_speed, 4)
    data = cmd_data + speed_data
    
    return send_can_frame(frame_id, data)

def set_6048_reboot(frame_id):
    return send_can_frame(join_6048_frame_id(frame_id), [0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF])
    
def set_6048_abs_angle(node_id, angle):
    """设置6048电机绝对角度"""
    frame_id = join_6048_frame_id(node_id)
    print(f"6048电机 - 设置绝对角度 {angle}, Frame_ID: 0x{frame_id:03X}")
    
    cmd_data = [CMD_SET_ABS_ANGLE_6048]
    angle_data = int_to_signed_bytes(angle, 4)
    data = cmd_data + angle_data
    
    return send_can_frame(frame_id, data)

def set_6048_relative_angle(node_id, angle):
    """设置6048电机相对角度"""
    frame_id = join_6048_frame_id(node_id)
    print(f"6048电机 - 设置相对角度 {angle}, Frame_ID: 0x{frame_id:03X}")
    
    cmd_data = [CMD_SET_REL_ANGLE_6048]
    angle_data = int_to_signed_bytes(angle, 4)
    data = cmd_data + angle_data
    
    return send_can_frame(frame_id, data)

def back_to_zero_6048(id_num):
    """
    发送以最短路径并设置回到零点
    
    参数:
        id_num (int): 设备ID号
    """
    send_can_frame(join_6048_frame_id(id_num), [0xC4])

def set_original_point_6048(id_num):
    """
    设置当前位置为原点
    断电不保存

    参数:
        id_num (int): 设备ID号
    """
    send_can_frame(join_6048_frame_id(id_num), [0xB1])  # 命令码 + 无数据 

def set_speed_6048(id_num, speed):
    """
    发送并设置速度
    
    参数:
        id_num (int): 设备ID号
        speed (float): 速度值（浮点数）
    """
    bytes_list = int_to_signed_bytes(speed,4)  # 转换为4字节小端序
    data = [0xC1] + bytes_list  # 命令码 + 速度字节
    send_can_frame(join_6048_frame_id(id_num), data)


# ==============================
# CAN设备管理函数
# ==============================
def disable_can_interface():
    """失能CAN接口"""
    print("=" * 40)
    print("正在失能CAN接口...")
    print("=" * 40)
    
    try:
        ret_stop = canDLL.VCI_ResetCAN(VCI_USBCAN2, 0, 0)
        print(f"停止CAN通道: {'成功' if ret_stop == 1 else '失败'}")
        return ret_stop == 1
    except Exception as e:
        print(f"失能CAN接口时发生错误: {e}")
        return False

def enable_can_interface():
    """重新启用CAN接口"""
    print("=" * 40)
    print("正在重新启用CAN接口...")
    print("=" * 40)
    
    try:
        ret_init = canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 0, byref(vci_initconfig))
        if ret_init == STATUS_OK:
            print('✓ 重新初始化CAN成功')
        else:
            print('✗ 重新初始化CAN失败')
            return False
        
        ret_start = canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 0)
        if ret_start == STATUS_OK:
            print('✓ 重新启动CAN成功')
        else:
            print('✗ 重新启动CAN失败')
            return False
        
        print("CAN接口重新启用完成!")
        print("=" * 40)
        return True
        
    except Exception as e:
        print(f"启用CAN接口时发生错误: {e}")
        return False

def restart_can_interface():
    """重启CAN接口"""
    print("=" * 50)
    print("正在重启CAN接口...")
    print("=" * 50)
    
    try:
        canDLL.VCI_ResetCAN(VCI_USBCAN2, 0, 0)
        sleep(0.1)
        
        canDLL.VCI_CloseDevice(VCI_USBCAN2, 0)
        sleep(0.5)
        
        ret = canDLL.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
        if ret == STATUS_OK:
            print('✓ 重新打开设备成功')
        else:
            print('✗ 重新打开设备失败')
            return False
        
        ret = canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 0, byref(vci_initconfig))
        if ret == STATUS_OK:
            print('✓ 重新初始化CAN成功')
        else:
            print('✗ 重新初始化CAN失败')
            return False
        
        ret = canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 0)
        if ret == STATUS_OK:
            print('✓ 重新启动CAN成功')
        else:
            print('✗ 重新启动CAN失败')
            return False
        
        print("=" * 50)
        print("CAN接口重启完成!")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"重启过程中发生错误: {e}")
        return False

# ==============================
# 电机控制器类
# ==============================
class MotorController:
    """统一的电机控制器类"""
    
    def __init__(self):
        self.initialized = False
    
    def init(self):
        """初始化CAN接口"""
        print("=" * 60)
        print("初始化CAN电机控制系统")
        print("波特率: 1000kbps")
        print("=" * 60)
        
        # 打开设备
        ret = canDLL.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
        if ret == STATUS_OK:
            print('✓ 调用 VCI_OpenDevice成功')
        elif ret == -1:
            print('✗ 设备掉线')
            return False
        else:
            print('✗ 链接设备失败')
            return False
        
        # 初始化通道0
        ret = canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 0, byref(vci_initconfig))
        if ret == STATUS_OK:
            print('✓ 调用 VCI_InitCAN成功')
        else:
            print('✗ 调用 VCI_InitCAN出错')
            return False
        
        # 启动通道0
        ret = canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 0)
        if ret == STATUS_OK:
            print('✓ 调用 VCI_StartCAN成功')
        else:
            print('✗ 调用 VCI_StartCAN出错')
            return False
        
        self.initialized = True
        print("=" * 60)
        print("CAN电机控制系统初始化完成!")
        print("=" * 60)
        return True
    
    

# ==============================
# 初始化和测试函数
# ==============================
def initialize_8108_motors():
    reboot_8108(4)
    sleep(0.1)
    reboot_8108(5)
    sleep(0.1)
    reboot_8108(6)
    sleep(1)

    set_8108_controller_mode(4, mode_pos_trapezium_8108)
    sleep(0.1)
    set_8108_controller_mode(5, mode_pos_trapezium_8108)
    sleep(0.1)
    set_8108_controller_mode(6, mode_pos_trapezium_8108)
    sleep(0.5)

    set_8108_closed_loop(4)
    sleep(0.1)
    set_8108_closed_loop(5)
    sleep(0.1)
    set_8108_closed_loop(6)
    sleep(1)

    set_8108_position(4, 0)
    sleep(0.5)
    set_8108_position(5, 0)
    sleep(0.5)
    set_8108_position(6, 0)


    return 

def initialize_6048_motor():
    set_6048_reboot(1)
    sleep(0.1)
    set_6048_reboot(2)
    sleep(0.1)
    set_6048_reboot(3)
    sleep(1)

    # set_6048_abs_angle(1,0)
    # sleep(0.5)
    # set_6048_abs_angle(2,0)
    # sleep(0.5)
    # set_6048_abs_angle(3,0)
    # sleep(0.5)
    
    back_to_zero_6048(1)
    sleep(0.5)
    back_to_zero_6048(2)
    sleep(5)
    set_6048_abs_angle(3,0)

    return

# ==============================
# 主程序
# ==============================
if __name__ == "__main__":
    # 创建控制器实例
    controller = MotorController()
    # print("test")
    # controller=MotorController.init()
    if controller.init():
        print("CAN电机控制系统已成功初始化!")
    sleep(3)
    initialize_6048_motor()
    # initialize_8108_motors()


    

    
