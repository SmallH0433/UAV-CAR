# 双向丝杆控制组指令（ESP8266 下位机：双 42 步进 + T8 双向丝杆）
std_msgs/Header header

uint8 CMD_STOP  = 0   # 立即停止，原地自锁（HOLD_MID）；忽略 group，两组同停
uint8 CMD_IN    = 1   # 螺母向内运动至行程终点（57mm）
uint8 CMD_OUT   = 2   # 螺母向外运动回外侧
uint8 CMD_RELAX = 3   # 释放使能（不再自锁，省电降温）
uint8 CMD_LOCK  = 4   # 恢复使能（自锁）

uint8 group      # 0=两组同时 1=组1 2=组2（CMD_STOP 忽略此字段）
uint8 command    # 见上方 CMD_* 常量
uint32 speed     # 脉冲频率 steps/s，0=不改变（1600 = 2mm/s，全程约 33.5s）
