import pygame
import serial
import math
import time

# 配置：串口参数
SERIAL_PORT = 'COM11'  # 根据实际设置，例如 'COM3' 或 '/dev/ttyUSB0'
BAUD_RATE = 115200  # 波特率
DEADZONE = 0.2  # 摇杆死区

# PWM 范围
PWM_MIN = 5
PWM_MAX = 80

# 数据帧常量
FRAME_HEADER = [0xA0, 0xA5]  # 帧头
FRAME_TAIL = [0xC0, 0xC5]    # 帧尾

# 电机方向控制字节
MOTOR_FORWARD = 0x0F     # 正转
MOTOR_REVERSE = 0xF0     # 反转
MOTOR_NO_CONTROL = 0x00  # 无控制

# 茶几电机控制字节
TABLE_NO_CONTROL = 0x00  # 无控制
TABLE_UP = 0xF0         # 升
TABLE_DOWN = 0x0F       # 降


def apply_deadzone(value, deadzone):
    """应用死区：若值在[-deadzone, deadzone]内，则视为0."""
    return 0.0 if abs(value) < deadzone else value


def joystick_to_speed(val):
    """
    将摇杆输入（[-1,1]）转换为速度（0-100）。
    使用平方映射实现非线性加速响应。
    """
    # 如果速度为0，返回0
    if abs(val) <= 0.01:
        return 0
    magnitude = abs(val)
    magnitude*=2.71828
    # 非线性映射：平方后线性缩放到 [PWM_MIN, PWM_MAX] 范围
    speed = PWM_MIN + (PWM_MAX - PWM_MIN) * (math.log(magnitude))
    return int(min(100, max(0, speed)))


def get_button_states(joystick):
    """
    获取手柄按键状态
    
    参数:
        joystick: pygame joystick对象
    
    返回:
        tuple: (x_pressed, y_pressed, a_pressed, b_pressed) 按键状态
    """
    # Xbox手柄按键映射：A键=0, B键=1, X键=2, Y键=3
    a_button = joystick.get_button(0)  # A键
    b_button = joystick.get_button(1)  # B键
    x_button = joystick.get_button(2)  # X键
    y_button = joystick.get_button(3)  # Y键
    
    return a_button, b_button, x_button, y_button


def calculate_motor_direction(speed_value):
    """
    根据速度值计算电机方向控制字节
    
    参数:
        speed_value: 速度值（-1到1之间）
    
    返回:
        int: 方向控制字节
    """
    if abs(speed_value) <= 0.01:
        return MOTOR_NO_CONTROL
    elif speed_value > 0:
        return MOTOR_FORWARD
    else:
        return MOTOR_REVERSE


def calculate_crc(data_bytes):
    """
    计算CRC校验码（对前9个字节进行异或操作）
    
    参数:
        data_bytes: 前9个字节的数据列表
    
    返回:
        int: CRC校验码
    """
    crc = 0
    for byte_val in data_bytes:
        crc ^= byte_val
    return crc


def create_control_frame(motor1_speed, motor2_speed, motor3_speed, motor4_speed, 
                        motor1_dir_val, motor2_dir_val, motor3_dir_val, motor4_dir_val,
                        table_control):
    """
    创建完整的控制数据帧
    
    参数:
        motor1_speed~motor4_speed: 4个电机的速度（0-100）
        motor1_dir_val~motor4_dir_val: 4个电机的方向值（-1到1）
        table_control: 茶几控制（0=无控制, 1=升, -1=降）
    
    返回:
        list: 12字节的完整数据帧
    """
    # 构建数据帧
    frame = []
    
    # 1. 帧头 (2字节)
    frame.extend(FRAME_HEADER)
    
    # 2. 4个电机控制 (8字节)
    motors = [
        (motor1_dir_val, motor1_speed),
        (motor2_dir_val, motor2_speed),
        (motor3_dir_val, motor3_speed),
        (motor4_dir_val, motor4_speed)
    ]
    
    for dir_val, speed in motors:
        direction_byte = calculate_motor_direction(dir_val)
        speed_byte = max(0, min(100, int(speed)))  # 确保速度在0-100范围内
        frame.append(direction_byte)
        frame.append(speed_byte)
    
    # 3. 茶几电机控制 (1字节)
    if table_control > 0:
        table_byte = TABLE_UP
    elif table_control < 0:
        table_byte = TABLE_DOWN
    else:
        table_byte = TABLE_NO_CONTROL
    frame.append(table_byte)
    
    # 4. CRC校验 (1字节) - 对帧内数据进行异或（不包含帧头）
    # 帧内数据：4个电机控制(8字节) + 茶几控制(1字节) = 9字节
    frame_data_only = frame[2:]  # 跳过帧头的2字节
    crc_byte = calculate_crc(frame_data_only)
    frame.append(crc_byte)
    
    # 5. 帧尾 (2字节)
    frame.extend(FRAME_TAIL)
    
    return frame


def send_control_frame(ser, frame_data):
    """
    发送控制数据帧
    
    参数:
        ser: 串口对象
        frame_data: 12字节的数据帧列表
    """
    # 将数据转换为字节并发送
    byte_data = bytes(frame_data)
    ser.write(byte_data)
    
    # 打印完整的数据帧（原始字节格式）
    print("=" * 60)
    print("发送数据帧:")
    
    # 十六进制格式显示
    hex_str = ' '.join([f'{b:02X}' for b in frame_data])
    print(f"HEX: {hex_str}")
    
    # 分组显示便于阅读
    print("分组显示:")
    print(f"  帧头:     {frame_data[0]:02X} {frame_data[1]:02X}")
    print(f"  电机1:    {frame_data[2]:02X} {frame_data[3]:02X}")
    print(f"  电机2:    {frame_data[4]:02X} {frame_data[5]:02X}")
    print(f"  电机3:    {frame_data[6]:02X} {frame_data[7]:02X}")
    print(f"  电机4:    {frame_data[8]:02X} {frame_data[9]:02X}")
    print(f"  茶几:     {frame_data[10]:02X}")
    print(f"  CRC:      {frame_data[11]:02X}")
    print(f"  帧尾:     {frame_data[12]:02X} {frame_data[13]:02X}")
    
    print("=" * 60)


def main():
    # 初始化 pygame 和 Joystick
    pygame.init()

    # 设置SDL环境变量以独占模式运行
    import os
    os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
    os.environ["SDL_VIDEODRIVER"] = "dummy"  # 不显示窗口

    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("未检测到手柄，请检查连接。")
        return
    else:
        print("已连接手柄")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # 打开串口
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # 等待串口稳定

    print("开始控制循环（Ctrl+C 退出）。")
    print("控制说明：")
    print("  左摇杆：控制移动和转向")
    print("  右摇杆Y轴：控制总体速度（油门）")
    print("  X键：茶几上升")
    print("  Y键：茶几下降")
    print("  A键、B键：保留功能")
    
    try:
        while True:
            # 处理 Pygame 事件（必要操作）
            for event in pygame.event.get():
                pass

            # 获取摇杆轴值（Xbox360：左X=0, 左Y=1, 右X=3, 右Y=4）
            left_x = joystick.get_axis(0)   # 左摇杆横向
            left_y = -joystick.get_axis(1)  # 左摇杆纵向（向前为正）
            right_x = joystick.get_axis(3)  # 右摇杆横向
            right_y = -joystick.get_axis(4) # 右摇杆纵向（向前为正）
            
            # 获取按键状态
            a_pressed, b_pressed, x_pressed, y_pressed = get_button_states(joystick)
            
            print(f"摇杆数据 - 左X: {left_x:.3f}, 左Y: {left_y:.3f}, 右X: {right_x:.3f}, 右Y: {right_y:.3f}")
            print(f"按键状态 - A: {'●' if a_pressed else '○'}, B: {'●' if b_pressed else '○'}, X: {'●' if x_pressed else '○'}, Y: {'●' if y_pressed else '○'}")

            # 应用死区过滤
            left_x = apply_deadzone(left_x, DEADZONE)
            left_y = apply_deadzone(left_y, DEADZONE)
            right_x = apply_deadzone(right_x, DEADZONE)
            right_y = apply_deadzone(right_y, DEADZONE)
            
            # 如果右摇杆Y轴接近0，设为绝对值
            if abs(right_y) < 0.1:
                right_y = abs(right_y)
                
            # 计算前进和转向分量
            throttle = right_y  # 前进/后退速度分量
            turn = left_x
            forward = left_y

            # 如果向后行驶，则转向方向反转
            if forward < 0:
                turn = -turn

            # 差速驱动：左右轮速度 (-1 到 1)
            left_speed = (forward + (turn * 0.5)) * 2
            right_speed = (forward - (turn * 0.5)) * 2

            left_speed *= throttle
            right_speed *= throttle
            print(f"计算速度 - 左轮: {left_speed:.3f}, 右轮: {right_speed:.3f}")
            
            # 归一化防止超过范围
            max_val = max(abs(left_speed), abs(right_speed))
            if max_val > 1:
                left_speed /= max_val
                right_speed /= max_val

            # 计算各电机的速度（0-100）
            motor1_speed = joystick_to_speed(right_speed)   # 电机1: 左轮
            motor2_speed = joystick_to_speed(left_speed)  # 电机2: 右轮
            motor3_speed = joystick_to_speed(left_speed)  # 电机3: 右轮（交叉）
            motor4_speed = joystick_to_speed(right_speed)   # 电机4: 左轮（交叉）
#lrrl lrlr
            # 茶几控制
            table_control = 0  # 默认无控制
            if x_pressed:
                table_control = 1   # 上升
            elif y_pressed:
                table_control = -1  # 下降

            # 创建并发送控制帧
            control_frame = create_control_frame(
                motor1_speed, motor2_speed, motor3_speed, motor4_speed,
                left_speed, right_speed, right_speed, left_speed,
                table_control
            )
            
            send_control_frame(ser, control_frame)
            
            time.sleep(0.05)  # 20Hz发送频率

    except KeyboardInterrupt:
        print("退出控制循环。")
    finally:
        ser.close()
        pygame.quit()


if __name__ == "__main__":
    main()
