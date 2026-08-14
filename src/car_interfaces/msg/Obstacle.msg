# 单个障碍物（base_link 坐标系）
float32 angle        # 方位角 rad，车头方向为 0，左正右负
float32 distance     # 距离 m
float32 radius       # 等效半径 m
float32 velocity     # 接近速度 m/s，静止障碍为 0
uint8 source         # 0=激光雷达 1=摄像头 2=融合
