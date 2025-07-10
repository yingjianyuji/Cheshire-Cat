from ctypes import *
import ctypes
import struct
from time import sleep
import os
# 定义常量
VCI_USBCAN2 = 4  # 设备类型，表示USBCAN-2设备
STATUS_OK = 1  # 操作成功状态码

# 定义CAN初始化配置结构体
class VCI_INIT_CONFIG(Structure):
    _fields_ = [
        ("AccCode", c_uint),  # 验收码
        ("AccMask", c_uint),  # 屏蔽码
        ("Reserved", c_uint),  # 保留字段
        ("Filter", c_ubyte),  # 滤波方式
        ("Timing0", c_ubyte),  # 波特率定时器0
        ("Timing1", c_ubyte),  # 波特率定时器1
        ("Mode", c_ubyte)  # 工作模式
    ]


# 定义CAN数据帧结构体
class VCI_CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint),  # 帧ID
        ("TimeStamp", c_uint),  # 时间戳
        ("TimeFlag", c_ubyte),  # 是否使用时间戳标志
        ("SendType", c_ubyte),  # 发送类型
        ("RemoteFlag", c_ubyte),  # 远程帧标志
        ("ExternFlag", c_ubyte),  # 扩展帧标志
        ("DataLen", c_ubyte),  # 数据长度
        ("Data", c_ubyte * 8),  # 数据内容（最多8字节）
        ("Reserved", c_ubyte * 3)  # 保留字段
    ]

vci_initconfig = VCI_INIT_CONFIG(
    0x80000008,  # 验收码
    0xFFFFFFFF,  # 屏蔽码
    0,  # 保留字段
    0,  # 滤波方式
    0x00,  # 波特率定时器0
    0x14,  # 波特率定时器1
    0  # 工作模式 (0=正常模式)
)

# Windows系统加载DLL
canDLL = windll.LoadLibrary('./ControlCAN.dll')

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
if ret != STATUS_OK:
    print('调用 VCI_InitCAN1出错\r\n')

# 启动通道0
ret = canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 0)
if ret == STATUS_OK:
    print('调用 VCI_StartCAN1成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_StartCAN1出错\r\n')




# ret = canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)
# if ret == STATUS_OK:
#     print('CAN1通道发送成功\r\n')
# if ret != STATUS_OK:
#     print('CAN1通道发送失败\r\n')
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

def Encode_can_id(target_id, command_id):
    """
    将目标ID和命令ID编码为11位CAN ID

    参数:
        target_id (int):  6位目标ID (0x00-0x3F)
        command_id (int): 5位命令ID (0x00-0x1F)

    返回:
        int: 11位CAN ID (高6位=target_id, 低5位=command_id)

    示例:
        #>>> Encode_can_id(0x05, 0x0C)
        0xAC  # (0x05 << 5) | 0x0C = 0xAC
    """
    # 检查输入范围
    if not (0 <= target_id <= 0x3F):
        raise ValueError("target_id 必须是6位值 (0-0x3F)")
    if not (0 <= command_id <= 0x1F):
        raise ValueError("command_id 必须是5位值 (0-0x1F)")

    # 组合ID并返回
    return (target_id << 5) | command_id


def float32_to_bytes(value: float) -> list:
    """
    将float32编码为4字节的小端序ubyte数组

    参数:
        value (float): 要编码的浮点数

    返回:
        list: 4字节的ubyte数组 (小端序)

    示例:
        #>>> float32_to_bytes(2.2)
        [0xCD, 0xCC, 0x0C, 0x40]  # 对应小端序 40 0C CC CD
    """
    # 使用struct.pack将float转为4字节小端序bytes
    packed = struct.pack('<f', value)  # '<f' 表示小端序float32

    # 转换为ubyte数组
    ubyte_array = [c_ubyte(byte).value for byte in packed]

    return ubyte_array
# 4. 发送CAN数据
# 创建8字节数据数组
ubyte_array = c_ubyte * 8
a = ubyte_array(1, 0, 0, 0, 0, 0, 0, 0)  # 测试数据

# 创建3字节保留字段数组
ubyte_3array = c_ubyte * 3
b = ubyte_3array(0, 0, 0)  # 保留字段初始化为0

#创建数据帧开头
#控制命令
Set_Controller_Mode=0x0B#设置控制模式
mode_pos_trapezium=ubyte_array(3,0,0,0,5,0,0,0)#梯形控制模式
Set_Axis_State=0x07#设置状态，8为闭环控制状态
closed_loop=ubyte_array(8,0,0,0,0,0,0,0)
Set_Input_Pos=0x0C#设置目标位置
motor_calibration=ubyte_array(4,0,0,0,0,0,0,0)
encoder_calibration=ubyte_array(7,0,0,0,0,0,0,0)
"""
位置：CD CC 0C 40 
力矩：00 00 
速度：00 00 

设置目标位置，速度前馈和力矩前
馈，其中目标位置为2.2（浮点数：
0x400CCCCD），力矩前馈和速度前
馈为0
"""
Reboot_id=0x16#重启
#电机序号
motor0=0x00
motor1=0x01
motor2=0x02

def set_axis_con_input_mode(target_id,input_mode):
    """设置控制模式和输入模式"""
    can_ID = Encode_can_id(target_id, Set_Controller_Mode)
    print("CAN_id:",can_ID)
    # 将ctypes数组转换为Python列表
    data_bytes = [input_mode[i] for i in range(8)]
    ret = send_can_frame(can_ID, data_bytes)
    return ret

def set_axis_sta_closed_loop(target_id):
    """设置闭环控制模式"""
    can_ID = Encode_can_id(target_id, Set_Axis_State)
    print("CAN_id:", can_ID)
    # 将ctypes数组转换为Python列表
    data_bytes = [closed_loop[i] for i in range(8)]
    ret = send_can_frame(can_ID, data_bytes)
    return ret

def set_axis_sta_stop(target_id):
    """关闭控制模式"""
    can_ID = Encode_can_id(target_id, Set_Axis_State)
    print("CAN_id:", can_ID)
    # 将ctypes数组转换为Python列表
    data_bytes = [a[i] for i in range(8)]
    ret = send_can_frame(can_ID, data_bytes)
    return ret

def set_axis_pos_trapezium(target_id, input_pos):
    """在梯形曲线控制模式下控制机械臂位置"""
    can_ID = Encode_can_id(target_id, Set_Input_Pos)
    print("CAN_id:", can_ID)
    float_bytes = float32_to_bytes(input_pos)  # 得到4字节列表
    print("float_bytes:", float_bytes)
    data_bytes = float_bytes + [0, 0, 0, 0]    # 拼成8字节
    print("data_bytes:", data_bytes)
    ret = send_can_frame(can_ID, data_bytes)
    return ret

def set_axis_calibration(target_id):
    """校准编码器与电机"""
    can_ID = Encode_can_id(target_id, Set_Axis_State)
    print("CAN_id:", can_ID)
    
    # 校准电机
    motor_data_bytes = [motor_calibration[i] for i in range(8)]
    ret1 = send_can_frame(can_ID, motor_data_bytes)
    print("校准电机：", ret1)
    
    sleep(0.1)
    
    # 校准编码器
    encoder_data_bytes = [encoder_calibration[i] for i in range(8)]
    ret2 = send_can_frame(can_ID, encoder_data_bytes)
    print("校准编码器：", ret2)
    
    return ret1 and ret2

def Reboot(target_id):
    """重启"""
    can_ID = Encode_can_id(target_id, Reboot_id)
    print("CAN_id:", can_ID)
    # 将ctypes数组转换为Python列表
    data_bytes = [a[i] for i in range(8)]
    ret = send_can_frame(can_ID, data_bytes)
    print("重启:", ret)
    return ret

def send_can_frame(frame_id, data_bytes):
    """
    发送CAN标准帧
    
    参数:
        frame_id (int): 11位帧ID（0x000-0x7FF）
        data_bytes (list): 数据字节列表（0-8字节）
    
    返回:
        int: 发送结果（1=成功，0=失败）
    """
    if frame_id < 0 or frame_id > 0x7FF:
        raise ValueError("标准帧ID必须在0x000-0x7FF范围内")
    
    if len(data_bytes) < 0 or len(data_bytes) > 8:
        raise ValueError("数据长度必须在0-8字节之间")
    
    # 准备数据数组，根据实际数据长度创建
    data_len = len(data_bytes)
    data_array = (c_ubyte * 8)()  # 结构体要求固定8字节
    
    # 只填充实际数据
    for i in range(data_len):
        data_array[i] = data_bytes[i]
    
    # 创建CAN帧对象
    vci_can_obj = VCI_CAN_OBJ(
        frame_id,           # 11位帧ID
        0,                  # 时间戳
        0,                  # 时间标志
        0,                  # 发送类型（0=正常发送）
        0,                  # 远程帧标志（0=数据帧）
        0,                  # 帧格式（0=标准帧）
        data_len,           # DLC（数据长度）
        data_array,         # 数据
        (c_ubyte * 3)(0, 0, 0)  # 保留字段
    )
    
    # 发送CAN帧
    ret = canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)
    
    # 打印发送的数据（16进制格式）
    print(f"发送CAN帧:")
    print(f"  帧ID: 0x{frame_id:03X}")
    print(f"  DLC: 0x{data_len:02X} ({data_len} bytes)")
    if data_len > 0:
        print(f"  数据: {' '.join([f'0x{b:02X}' for b in data_bytes])}")
    else:
        print(f"  数据: 无数据（远程帧或空帧）")
    print(f"  发送结果: {ret} ({'成功' if ret == 1 else '失败'})")
    print("-" * 50)
    
    return ret
# 创建CAN数据帧对象
# 参数：ID=0x1, 时间戳=0, 不使用时间戳, 发送类型=1(单次发送),
# 非远程帧, 标准帧, 数据长度=8, 数据内容, 保留字段
#vci_can_obj = VCI_CAN_OBJ(, 0, 0, 1, 0, 0, 8, a, b)
#
# ret = canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)
# if ret == STATUS_OK:
#     print('CAN1通道发送成功\r\n')
# if ret != STATUS_OK:
#     print('CAN1通道发送失败\r\n')

#print(Encode_can_id(0x05, 0x0C))
#print(float32_to_bytes(2.2))
#Encode_can_id(0x05, 0x0C)
#float32_to_bytes(2.2)
# set_axis_con_input_mode(6, mode_pos_trapezium)
# sleep(1)
# set_axis_sta_closed_loop(6)
# sleep(0.1)
# set_axis_sta_closed_loop(6)
# # sleep(1)
# # sleep(1)
# # set_axis_sta_closed_loop(6)
# sleep(3)
# set_axis_pos_trapezium(6,20)
#set_axis_calibration(1)
# Reboot(5)
#set_axis_sta_stop(1)
#初始化4-6号电机

# set_axis_sta_closed_loop(5)
# sleep(0.2)
# set_axis_sta_closed_loop(5)
# sleep(0.2)
# set_axis_sta_closed_loop(4)
# sleep(0.2)
# set_axis_sta_closed_loop(4)
# sleep(0.2)

Reboot(6)
sleep(0.2)
Reboot(5)
sleep(0.2)
Reboot(4)
sleep(0.2)

set_axis_con_input_mode(6, mode_pos_trapezium)
sleep(0.5)
set_axis_con_input_mode(5, mode_pos_trapezium)
sleep(0.5)
set_axis_con_input_mode(4, mode_pos_trapezium)
sleep(0.5)

set_axis_sta_closed_loop(6)
sleep(0.2)
set_axis_sta_closed_loop(5)
sleep(0.2)
set_axis_sta_closed_loop(4)
sleep(0.1)

set_axis_pos_trapezium(6,0)
sleep(0.5)
set_axis_pos_trapezium(5,0)
sleep(0.5)
set_axis_pos_trapezium(4,0)
sleep(0.5)



# set_axis_con_input_mode(5, mode_pos_trapezium)
# set_axis_con_input_mode(4, mode_pos_trapezium)
# Reboot(6)
# sleep(1)
# Reboot(5)
# sleep(1)
# Reboot(4)