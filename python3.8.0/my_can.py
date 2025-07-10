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

vci_initconfig = VCI_INIT_CONFIG(
    0x80000008,  # 验收码
    0xFFFFFFFF,  # 屏蔽码
    0,  # 保留字段
    0,  # 滤波方式
    0x00,  # 波特率定时器0 (500kbps)
    0x14,  # 波特率定时器1 (500kbps)
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