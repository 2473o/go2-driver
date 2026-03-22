# 时间戳 (纳秒，替代 Header 以去字符串化)
uint64 timestamp_ns

# 12个关节的位置 [FL_hip, FL_thigh, FL_calf, FR_hip, ..., RL..., RR...]
float32[12] q

# 12个关节的速度
float32[12] dq

# 12个关节的力矩
float32[12] tau

# 4个足端的接触力（有效范围：-32768~32767）
int16[4] foot_force

# --- 展开的 IMU 数据 (移除嵌套结构) ---
float32[4] imu_quaternion
float32[3] imu_gyroscope
float32[3] imu_accelerometer
