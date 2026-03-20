std_msgs/Header header

# 12个关节的位置 [FL_hip, FL_thigh, FL_calf, FR_hip, ..., RL..., RR...]
float32[12] q

# 12个关节的速度
float32[12] dq

# 12个关节的力矩
float32[12] tau

# 4个足端的接触力 [FL, FR, RL, RR]
float32[4] foot_force

unitree_go/IMUState imu_state