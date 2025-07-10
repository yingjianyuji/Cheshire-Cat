from pymavlink import mavutil
import time

# 连接飞控
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')  # 或使用串口 'com3'，或 'tcp:127.0.0.1:5760'

# 等待接收到心跳包
print("等待飞控心跳...")
master.wait_heartbeat()
print("心跳包已接收!")

def send_gps_raw_int(lat, lon, alt, hdop, vdop, speed, heading, fix_type=3):
    master.mav.gps_raw_int_send(
        time_usec=int(time.time() * 1e6),  # 时间戳（微秒）
        fix_type=fix_type,                # 3=3D fix, 4=RTK fix
        lat=int(lat * 1e7),               # 纬度（度 × 1e7）
        lon=int(lon * 1e7),               # 经度（度 × 1e7）
        alt=int(alt * 1000),              # 高度（毫米）
        eph=int(hdop * 100),             # 水平精度（厘米）
        epv=int(vdop * 100),             # 垂直精度（厘米）
        vel=int(speed * 100),            # 地面速度（厘米/秒）
        cog=int(heading * 100),          # 航向（0-360 × 100）
        satellites_visible=12,           # 可见卫星数
        alt_ellipsoid=int(alt * 1000),   # 椭球高度（毫米）
        h_acc=int(hdop * 100),           # 水平精度（毫米）
        v_acc=int(vdop * 100),           # 垂直精度（毫米）
        vel_acc=0,                       # 速度精度（毫米/秒）
        hdg_acc=0,                       # 航向精度（度 × 1e5）
        yaw=0                            # 偏航角（度 × 100）
    )

# 示例调用
send_gps_raw_int(
    lat=47.3977419, lon=8.5455941, alt=488.0,
    hdop=0.8, vdop=1.2, speed=2.5, heading=180.0,
    fix_type=3  # 4=RTK Fixed
)