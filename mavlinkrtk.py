import serial
from pymavlink import mavutil

# 连接飞控
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')  # 或使用串口 'com3'，或 'tcp:127.0.0.1:5760'

# 等待接收到心跳包
print("等待飞控心跳...")
master.wait_heartbeat()
print("心跳包已接收!")

# 连接 RTK GPS
rtk_gps = serial.Serial("/dev/ttyUSB1", baudrate=115200)

def parse_rtk_data(nmea_data):
    # 解析 NMEA 数据（示例：GGA 语句）
    if "$GNGGA" in nmea_data:
        parts = nmea_data.split(",")
        lat = float(parts[2][:2]) + float(parts[2][2:]) / 60  # 纬度（度）
        lon = float(parts[4][:3]) + float(parts[4][3:]) / 60  # 经度（度）
        alt = float(parts[9])  # 高度（米）
        fix_type = 4 if parts[6] == "4" else 3  # 4=RTK Fixed
        return lat, lon, alt, fix_type
    return None

while True:
    nmea_line = rtk_gps.readline().decode("ascii").strip()
    rtk_data = parse_rtk_data(nmea_line)
    if rtk_data:
        lat, lon, alt, fix_type = rtk_data
        send_gps_raw_int(lat, lon, alt, 0.5, 0.8, 0.0, 0.0, fix_type)
        send_global_position_int(lat, lon, alt, 0.0, 0.0, 0.0, 0.0)
    time.sleep(0.1)  # 10Hz 更新