from pymavlink import mavutil
import time

# 创建MAVLink连接
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')  # 或使用串口 'com3'，或 'tcp:127.0.0.1:5760'

# 等待接收到心跳包
print("等待飞控心跳...")
master.wait_heartbeat()
print("心跳包已接收!")


# 请求关键数据流
def setup_data_streams():
    # 请求姿态数据(30) - 50Hz
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,  # 消息ID
        20000,  # 间隔时间(微秒) = 20ms (50Hz)
        0, 0, 0, 0, 0
    )

    # 请求位置数据(32) - 10Hz
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        100000,  # 100ms (10Hz)
        0, 0, 0, 0, 0
    )

    # 请求电池状态(147) - 1Hz
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
        1000000,  # 1秒
        0, 0, 0, 0, 0
    )


# 设置数据流
setup_data_streams()

# 主循环处理接收到的消息
while True:
    try:
        # 获取消息
        msg = master.recv_match(blocking=True, timeout=1.0)
        if not msg:
            continue

        # 处理特定消息
        if msg.get_type() == 'ATTITUDE':
            print(f"姿态数据 - 滚转: {msg.roll:.2f} rad, 俯仰: {msg.pitch:.2f} rad, 偏航: {msg.yaw:.2f} rad")

        elif msg.get_type() == 'LOCAL_POSITION_NED':
            print(f"位置数据 - X: {msg.x:.2f} m, Y: {msg.y:.2f} m, Z: {msg.z:.2f} m")

        elif msg.get_type() == 'BATTERY_STATUS':
            print(f"电池状态 - 电压: {msg.voltages[0] / 1000:.2f} V, 剩余: {msg.battery_remaining}%")

    except KeyboardInterrupt:
        print("\n退出程序")
        break