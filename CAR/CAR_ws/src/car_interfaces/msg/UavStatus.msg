# 车端上报给无人机的状态
std_msgs/Header header
geometry_msgs/Pose pose          # odom 系下位姿
float32 speed                    # 当前线速度 m/s
float32 battery_voltage
uint8 mode                       # 0=待机 1=自主避障 2=无人机协同
bool obstacle_ahead              # 前方是否有障碍
float32 min_obstacle_distance    # 最近障碍距离 m
