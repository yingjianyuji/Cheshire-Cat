from pymavlink import mavutil
import time

# 创建MAVLink连接
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')  # 或使用串口 'com3'，或 'tcp:127.0.0.1:5760'

# 等待接收到心跳包+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
print("等待飞控心跳...")
master.wait_heartbeat()
print("心跳包已接收!")


# 请求关键数据流
def setup_data_streams():
    # master.mav.command_long_send(
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    #     0,
    #     mavutil.mavlink.MAV_DATA_STREAM_ALL,  # 消息ID
    #     1000000,  # 间隔时间(微秒) = 200ms (5Hz)
    #     0, 0, 0, 0, 0
    # )
    # master.mav.command_long_send(
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    #     0,
    #     mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,  # 消息ID
    #     20000,  # 间隔时间(微秒) = 20ms (50Hz)
    #     0, 0, 0, 0, 0
    # )

# def send_gps_raw_int(lat, lon, alt, hdop, vdop, speed, heading, fix_type=3):
#     master.mav.gps_raw_int_send(
#         time_usec=int(time.time() * 1e6),  # 时间戳（微秒）
#         fix_type=fix_type,  # 3=3D fix, 4=RTK fix
#         lat=int(lat * 1e7),  # 纬度（度 × 1e7）
#         lon=int(lon * 1e7),  # 经度（度 × 1e7）
#         alt=int(alt * 1000),  # 高度（毫米）
#         eph=int(hdop * 100),  # 水平精度（厘米）
#         epv=int(vdop * 100),  # 垂直精度（厘米）
#         vel=int(speed * 100),  # 地面速度（厘米/秒）
#         cog=int(heading * 100),  # 航向（0-360 × 100）
#         satellites_visible=12,  # 可见卫星数
#         alt_ellipsoid=int(alt * 1000),  # 椭球高度（毫米）
#         h_acc=int(hdop * 100),  # 水平精度（毫米）
#         v_acc=int(vdop * 100),  # 垂直精度（毫米）
#         vel_acc=0,  # 速度精度（毫米/秒）
#         hdg_acc=0,  # 航向精度（度 × 1e5）
#         yaw=0  # 偏航角（度 × 100）
#     )
    # # 请求姿态数据(30) - 50Hz
    # master.mav.command_long_send(
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    #     0,
    #     mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,  # 消息ID
    #     20000,  # 间隔时间(微秒) = 20ms (50Hz)
    #     0, 0, 0, 0, 0
    # )
    #
    # # 请求位置数据(32) - 10Hz
    # master.mav.command_long_send(
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    #     0,
    #     mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    #     100000,  # 100ms (10Hz)
    #     0, 0, 0, 0, 0
    # )
    #
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
        #print(msg)
        if not msg:
            continue

        # 处理特定消息
        if msg.get_type() == 'ATTITUDE':
            print(f"姿态数据 - 滚转: {msg.roll:.2f} rad, 俯仰: {msg.pitch:.2f} rad, 偏航: {msg.yaw:.2f} rad")

        elif msg.get_type() == 'LOCAL_POSITION_NED':
            print(f"位置数据 - X: {msg.x:.2f} m, Y: {msg.y:.2f} m, Z: {msg.z:.2f} m")

        elif msg.get_type() == 'BATTERY_STATUS':
            print(f"电池状态 - 电压: {msg.voltages[0] / 1000:.2f} V, 剩余: {msg.battery_remaining}%")

        elif msg.get_type() == 'VFR_HUD':
            print(f"[VFR_HUD] 地速: {msg.groundspeed:.2f} m/s, 空速: {msg.airspeed:.2f} m/s")

        elif msg.get_type()=='GLOBAL_POSITION_INT':
            print(f"[GLOBAL_POSITION_INT] 北向速度: {msg.vx / 100:.2f} m/s, 东向速度: {msg.vy / 100:.2f} m/s,下降速度:{msg.vz / 100:.2f} m/s,偏向角:{msg.hdg},纬度:{msg.lat},经度:{msg.lon},高度:{msg.alt}")

        # send_gps_raw_int(
        #     lat=47.3977419, lon=8.5455941, alt=488.0,
        #     hdop=0.8, vdop=1.2, speed=2.5, heading=180.0,
        #     fix_type=3  # 4=RTK Fixed
        # )

    except KeyboardInterrupt:
        print("\n退出程序")
        break