import struct

def pack_can_frame_6048(position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0, node_id=1):
    """
    打包6048电机的CAN数据帧
    
    参数:
        position (float): 位置，单位弧度(RAD)
        velocity (float): 速度，单位RAD/s
        kp (float): KP值
        kd (float): KD值  
        torque (float): 力矩，单位N.m
        node_id (int): 节点ID (0-63, 6位)
    
    输出:
        打印格式化的CAN数据帧
    """
    
    # 检查node_id范围
    if not (0 <= node_id <= 63):
        raise ValueError("node_id 必须在0-63范围内 (6位)")
    
    # 计算CAN ID：node_id (6位) << 5 | cmd_id (5位)
    cmd_id = 0x08  # 根据要求，command_id为0x08
    can_id = (node_id << 5) | cmd_id
    
    # DLC始终为8字节（6048电机使用固定8字节数据）
    dlc = 8
    
    # 位置转换：pos_int = (pos_double + 12.5) * 65535 / 25
    pos_int = int((position + 12.5) * 65535 / 25)
    pos_int = max(0, min(65535, pos_int))  # 限制在16位范围内
    
    # 速度转换：vel_int = (vel_double + 65) * 4095 / 130  
    vel_int = int((velocity + 65) * 4095 / 130)
    vel_int = max(0, min(4095, vel_int))  # 限制在12位范围内
    
    # KP值转换：kp_int = kp_double * 4095 / 500
    kp_int = int(kp * 4095 / 500)
    kp_int = max(0, min(4095, kp_int))  # 限制在12位范围内
    
    # KD值转换：kd_int = kd_double * 4095 / 5
    kd_int = int(kd * 4095 / 5)
    kd_int = max(0, min(4095, kd_int))  # 限制在12位范围内
    
    # 力矩转换：t_int = (t_double + 50) * 4095 / 100
    t_int = int((torque + 50) * 4095 / 100)
    t_int = max(0, min(4095, t_int))  # 限制在12位范围内
    
    # 按照图像格式打包数据
    # BYTE0: 位置低8位
    byte0 = pos_int & 0xFF
    
    # BYTE1: 位置高8位  
    byte1 = (pos_int >> 8) & 0xFF
    
    # BYTE2: 速度高8位
    byte2 = (vel_int >> 4) & 0xFF
    
    # BYTE3: 速度低4位(高4位) + KP值高4位(低4位)
    byte3 = ((vel_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F)
    
    # BYTE4: KP值低8位
    byte4 = kp_int & 0xFF
    
    # BYTE5: KD值高8位
    byte5 = (kd_int >> 4) & 0xFF
    
    # BYTE6: KD值低4位(高4位) + 力矩高4位(低4位)  
    byte6 = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F)
    
    # BYTE7: 力矩低8位
    byte7 = t_int & 0xFF
    
    # 创建数据数组
    data_bytes = [byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7]
    
    # 打印格式化输出
    print("=" * 70)
    print("6048电机CAN数据帧打包结果")
    print("=" * 70)
    print(f"CAN帧信息:")
    print(f"  节点ID: {node_id} (0x{node_id:02X})")
    print(f"  命令ID: {cmd_id} (0x{cmd_id:02X})")
    print(f"  CAN ID: {can_id} (0x{can_id:03X}) = (节点ID << 5) | 命令ID")
    print(f"  DLC:    {dlc} 字节")
    print("-" * 70)
    print(f"输入参数:")
    print(f"  位置: {position:.3f} RAD")
    print(f"  速度: {velocity:.3f} RAD/s") 
    print(f"  KP值: {kp:.3f}")
    print(f"  KD值: {kd:.3f}")
    print(f"  力矩: {torque:.3f} N.m")
    print("-" * 70)
    print(f"转换后的整数值:")
    print(f"  位置: {pos_int} (16位)")
    print(f"  速度: {vel_int} (12位)")
    print(f"  KP值: {kp_int} (12位)")
    print(f"  KD值: {kd_int} (12位)")
    print(f"  力矩: {t_int} (12位)")
    print("-" * 70)
    print(f"CAN数据帧 (8字节):")
    print(f"  BYTE0: 0x{byte0:02X} (位置低8位)")
    print(f"  BYTE1: 0x{byte1:02X} (位置高8位)")
    print(f"  BYTE2: 0x{byte2:02X} (速度高8位)")
    print(f"  BYTE3: 0x{byte3:02X} (速度低4位 + KP高4位)")
    print(f"  BYTE4: 0x{byte4:02X} (KP值低8位)")
    print(f"  BYTE5: 0x{byte5:02X} (KD值高8位)")
    print(f"  BYTE6: 0x{byte6:02X} (KD值低4位 + 力矩高4位)")
    print(f"  BYTE7: 0x{byte7:02X} (力矩低8位)")
    print("-" * 70)
    print(f"完整CAN帧:")
    print(f"  帧ID: 0x{can_id:03X}")
    print(f"  DLC:  0x{dlc:01X}")
    print(f"  数据: {' '.join([f'{b:02X}' for b in data_bytes])}")
    print(f"  十进制: {' '.join([f'{b:3d}' for b in data_bytes])}")
    print("=" * 70)
    
    return {'can_id': can_id, 'dlc': dlc, 'data': data_bytes}

# 测试函数
if __name__ == "__main__":
    
    # 测试用例2：极值测试 (节点ID=5)
    print("测试用例2：边界值测试 (节点ID=5)")
    result2 = pack_can_frame_6048(position=2.25, velocity=6.50, kp=1.00, kd=5.0, torque=10.0, node_id=7)
    
    