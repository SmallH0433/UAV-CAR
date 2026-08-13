# 无人机下发给车端的指令
std_msgs/Header header
uint8 cmd_type                   # 0=停止 1=前往目标点 2=速度直控 3=模式切换
geometry_msgs/Pose target_pose   # cmd_type=1 时有效
geometry_msgs/Twist velocity     # cmd_type=2 时有效
uint8 mode                       # cmd_type=3 时有效
