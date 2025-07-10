# python3.8.0 64位（python 32位要用32位的DLL/so(Linux)）
# 该程序用于通过USB-CAN适配器进行CAN通信

from ctypes import *
import ctypes
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


# 加载CAN库
# Windows系统使用ControlCAN.dll
# CanDLLName = './ControlCAN.dll' # 把DLL放到对应的目录下

# Linux系统使用ControlCAN.so
CanDLLName = './ControlCAN.so'  # 把SO放到对应的目录下,LINUX

# Windows系统加载DLL
canDLL = windll.LoadLibrary('./ControlCAN.dll')

# Linux系统加载SO库
#canDLL = cdll.LoadLibrary('./libcontrolcan.so')

# 打印当前使用的库名称
print(CanDLLName)

# 1. 打开设备
ret = canDLL.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
if ret == STATUS_OK:
    print('调用 VCI_OpenDevice成功\r\n')
elif ret == -1:
    print('设备掉线\r\n')
elif ret != 0:
    print('链接设备失败\r\n')

# 2. 初始化CAN通道0
# 配置参数：波特率500k，正常模式
vci_initconfig = VCI_INIT_CONFIG(
    0x80000008,  # 验收码
    0xFFFFFFFF,  # 屏蔽码
    0,  # 保留字段
    0,  # 滤波方式
    0x00,  # 波特率定时器0 (500kbps)
    0x1C,  # 波特率定时器1 (500kbps)
    0  # 工作模式 (0=正常模式)
)

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

# 3. 初始化CAN通道1 (与通道0相同的配置)
ret = canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 1, byref(vci_initconfig))
if ret == STATUS_OK:
    print('调用 VCI_InitCAN2 成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_InitCAN2 出错\r\n')

# 启动通道1
ret = canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 1)
if ret == STATUS_OK:
    print('调用 VCI_StartCAN2 成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_StartCAN2 出错\r\n')

# 4. 发送CAN数据
# 创建8字节数据数组
ubyte_array = c_ubyte * 8
a = ubyte_array(1, 2, 3, 4, 5, 6, 7, 8)  # 测试数据1-8

# 创建3字节保留字段数组
ubyte_3array = c_ubyte * 3
b = ubyte_3array(0, 0, 0)  # 保留字段初始化为0

# 创建CAN数据帧对象
# 参数：ID=0x1, 时间戳=0, 不使用时间戳, 发送类型=1(单次发送),
# 非远程帧, 标准帧, 数据长度=8, 数据内容, 保留字段
vci_can_obj = VCI_CAN_OBJ(0x1, 0, 0, 1, 0, 0, 8, a, b)

# 通过通道0发送数据
"""
DWORD __stdcall VCI_Transmit(DWORD DeviceType,DWORD DeviceInd,DWORD 
CANInd,PVCI_CAN_OBJ pSend,DWORD Length);
参数： 
DevType 
设备类型。对应不同的产品型号详见：适配器设备类型定义。 
DevIndex 
设备索引，比如当只有一个USB-CAN适配器时，索引号为0，这时再插入一个USB-CAN适
配器那么后面插入的这个设备索引号就是1，以此类推。 
CANIndex 
CAN通道索引。第几路 CAN。即对应卡的CAN通道号，CAN1为0，CAN2为1。 
pSend 
要发送的帧结构体 VCI_CAN_OBJ数组的首指针。 
Length 
要发送的帧结构体数组的长度（发送的帧数量）。最大为1000，建议设为1，每次发送
单帧，以提高发送效率。
"""
ret = canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)
if ret == STATUS_OK:
    print('CAN1通道发送成功\r\n')
if ret != STATUS_OK:
    print('CAN1通道发送失败\r\n')


# 5. 接收CAN数据
# 定义结构体数组类用于接收多个CAN帧
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


# 创建接收缓冲区，可容纳2500个CAN帧
rx_vci_can_obj = VCI_CAN_OBJ_ARRAY(2500)

# 进入无限循环接收数据
while True:
    # 从通道0接收数据
    ret = canDLL.VCI_Receive(VCI_USBCAN2, 0, 0, byref(rx_vci_can_obj.ADDR), 2500, 0)
    if ret > 0:  # 接收到数据
        for i in range(0, ret):  # 遍历所有接收到的帧
            print('CAN1通道接收成功', end=" ")
            print('ID：', end="")
            print(hex(rx_vci_can_obj.STRUCT_ARRAY[i].ID), end=" ")  # 打印帧ID
            print('DataLen：', end="")
            print(hex(rx_vci_can_obj.STRUCT_ARRAY[i].DataLen), end=" ")  # 打印数据长度
            print('Data：', end="")
            # 打印数据内容（转换为列表）
            print(list(rx_vci_can_obj.STRUCT_ARRAY[i].Data), end=" ")
            print('\r')

    # 从通道1接收数据
    ret = canDLL.VCI_Receive(VCI_USBCAN2, 0, 1, byref(rx_vci_can_obj.ADDR), 2500, 0)
    if ret > 0:  # 接收到数据
        for i in range(0, ret):
            print('CAN2通道接收成功', end=" ")
            print('ID：', end="")
            print(hex(rx_vci_can_obj.STRUCT_ARRAY[i].ID), end=" ")
            print('DataLen：', end="")
            print(hex(rx_vci_can_obj.STRUCT_ARRAY[i].DataLen), end=" ")
            print('Data：', end="")
            print(list(rx_vci_can_obj.STRUCT_ARRAY[i].Data), end=" ")
            print('\r')

# 6. 关闭设备（实际上前面的无限循环不会执行到这里）
canDLL.VCI_CloseDevice(VCI_USBCAN2, 0)