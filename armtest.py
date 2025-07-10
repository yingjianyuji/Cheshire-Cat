from pymavlink import mavutil
import time

# 创建 UDP 连接
# 格式: 'udp:<IP>:<port>' (SITL 通常用 127.0.0.1:14550)
# 真实飞控使用飞控的 IP 地址
conn = mavutil.mavlink_connection('udp:0.0.0.0:14550')

# 等待心跳包确认连接
print("等待心跳包...")
heartbeat = conn.wait_heartbeat(timeout=10)
if not heartbeat:
    print("错误：未收到心跳包！检查连接和端口")
    exit(1)

print(f"已连接！系统ID: {conn.target_system}")

# 设置 GUIDED 模式（解锁前必需）
mode_id = conn.mode_mapping()['GUIDED']
conn.mav.set_mode_send(
    conn.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
print("已发送 GUIDED 模式请求")

# 等待模式切换确认
while True:
    msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg and msg.custom_mode == mode_id:
        print("已进入 GUIDED 模式")
        break

# 发送解锁指令
conn.mav.command_long_send(
    conn.target_system,
    conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,  # 确认位
    1,  # 1=解锁, 0=上锁
    0, 0, 0, 0, 0, 0  # 参数保留
)
print("已发送解锁指令")

# 等待解锁ACK
ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if ack:
    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("无人机已成功解锁！")
    else:
        print(f"解锁失败！错误码: {ack.result}")
else:
    print("未收到解锁确认")

# 持续监控状态
try:
    while True:
        # 检查状态信息
        status = conn.recv_match(type='SYS_STATUS', blocking=False)
        if status:
            print(f"电池电压: {status.voltage_battery / 1000:.1f}V")

        # 检查解锁状态
        heartbeat = conn.recv_match(type='HEARTBEAT', blocking=False)
        if heartbeat:
            armed = "已解锁" if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "已上锁"
            print(f"状态: {armed} | 模式: {conn.flightmode}")

        time.sleep(0.5)

except KeyboardInterrupt:
    # 安全上锁
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("\n已发送上锁指令，程序退出")