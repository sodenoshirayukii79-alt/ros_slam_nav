# 感知层输出消息，供决策层使用

# 激光检测到的最小距离（米）
float32 min_distance

# 各扇区阻挡标志（检测到障碍物时为 True）
bool front_blocked
bool left_blocked
bool right_blocked

# 危险等级 0.0~1.0（0=安全，1=紧急）
float32 danger_level

# 障碍物方向："front"、"left"、"right"、"clear"
string obstacle_direction

# 各扇区距离值，供决策层参考
float32 front_distance
float32 left_distance
float32 right_distance
float32 back_distance
