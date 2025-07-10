# motor_can_receiver.py
# 功能：专用电机CAN数据接收程序（持续监听并打印电机发送的CAN帧）
# 环境：Python 3.8+，需安装ctypes库

from ctypes import *
import time

# 常量定义
VCI_USBCAN2 = 4  # 设备类型: USBCAN-2
STATUS_OK = 1  # 操作成功状态码
CAN_BAUD_500K = 0x001C  # 波特率500Kbps (电机常用波特率)


# CAN初始化配置结构体
class VCI_INIT_CONFIG(Structure):
    _fields_ = [
        ("AccCode", c_uint),  # 验收码 (默认接收所有帧)
        ("AccMask", c_uint),  # 屏蔽码 (不屏蔽任何位)
        ("Reserved", c_uint),  # 保留字段
        ("Filter", c_ubyte),  # 滤波模式 (0=接收所有帧)
        ("Timing0", c_ubyte),  # 波特率定时器0
        ("Timing1", c_ubyte),  # 波特率定时器1
        ("Mode", c_ubyte)  # 工作模式 (0=正常模式)
    ]


# CAN数据帧结构体
class VCI_CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint),  # 帧ID (11位或29位)
        ("TimeStamp", c_uint),  # 时间戳 (ms)
        ("TimeFlag", c_ubyte),  # 时间戳标志
        ("SendType", c_ubyte),  # 发送类型
        ("RemoteFlag", c_ubyte),  # 远程帧标志
        ("ExternFlag", c_ubyte),  # 扩展帧标志
        ("DataLen", c_ubyte),  # 数据长度 (0-8)
        ("Data", c_ubyte * 8),  # 数据内容
        ("Reserved", c_ubyte * 3)
    ]


def load_can_library():
    """加载CAN动态库 (自动适配Windows/Linux)"""
    try:
        # Windows
        return windll.LoadLibrary("./ControlCAN.dll")
    except:
        try:
            # Linux
            return cdll.LoadLibrary("./ControlCAN.so")
        except Exception as e:
            print(f"[错误] 加载CAN库失败: {e}")
            exit(1)


def initialize_can(can_dll, channel):
    """初始化指定CAN通道"""
    init_config = VCI_INIT_CONFIG(
        AccCode=0x00000000,
        AccMask=0xFFFFFFFF,
        Reserved=0,
        Filter=0,
        Timing0=0x00,  # 500Kbps配置
        Timing1=0x1C,
        Mode=0
    )

    if can_dll.VCI_InitCAN(VCI_USBCAN2, 0, channel, byref(init_config)) != STATUS_OK:
        print(f"[错误] CAN通道{channel}初始化失败!")
        return False

    if can_dll.VCI_StartCAN(VCI_USBCAN2, 0, channel)!= STATUS_OK:
        print(f"[错误] CAN通道{channel}启动失败!")
    return False

    print(f"CAN通道{channel}已就绪 (波特率500Kbps)")
    return True


def print_motor_data(frame):
    """格式化打印电机数据帧"""
    print(f"电机数据 -> ID: {hex(frame.ID)} | 数据: {list(frame.Data[:frame.DataLen])}")
    # 示例：解析电机特定数据（根据实际协议调整）
    if frame.ID == 0x201:  # 示例：假设0x201是电机状态帧
        rpm = (frame.Data[1] << 8) | frame.Data[0]  # 假设前2字节是转速
        print(f"  转速: {rpm} RPM | 温度: {frame.Data[2]}°C")


def monitor_motor_can(can_dll, channel):
    """持续监听电机CAN数据"""
    print(f"开始监听电机CAN数据 (通道{channel})...")
    try:
        while True:
            # 创建接收缓冲区
            recv_buf = (VCI_CAN_OBJ * 100)()  # 每次最多接收100帧
            ret = can_dll.VCI_Receive(VCI_USBCAN2, 0, channel, byref(recv_buf), 100, 0)

            if ret > 0:
                for i in range(ret):
                    print_motor_data(recv_buf[i])

            time.sleep(0.01)  # 避免CPU占用过高

    except KeyboardInterrupt:
        print("\n用户终止监听")


def main():
    # 1. 加载库
    can_dll = load_can_library()

    # 2. 打开设备
    if can_dll.VCI_OpenDevice(VCI_USBCAN2, 0, 0) != STATUS_OK:
        print("[错误] 设备连接失败！")


    # 3. 初始化CAN通道（默认使用通道0）
    if not initialize_can(can_dll, 0):
        can_dll.VCI_CloseDevice(VCI_USBCAN2, 0)


    # 4. 开始监听
    monitor_motor_can(can_dll, 0)
    print("can")
    # 5. 关闭设备
    can_dll.VCI_CloseDevice(VCI_USBCAN2, 0)
    print("设备已安全断开")
    print("can")

if __name__ == "__main__":
    main()