# 双向丝杆控制组状态（数组下标 0=组1 1=组2）
std_msgs/Header header

uint8 STATE_AT_OUTER   = 0   # 外侧自锁
uint8 STATE_MOVING_IN  = 1
uint8 STATE_AT_INNER   = 2   # 内侧自锁
uint8 STATE_MOVING_OUT = 3
uint8 STATE_HOLD_MID   = 4   # 中途停止，原地自锁

uint8[2] state      # 见上方 STATE_* 常量
float32[2] pos_mm   # 相对外侧位置 mm（0=外侧，57=内侧）
bool[2] enabled     # EN 使能（自锁保持）
uint32 speed        # 当前脉冲频率 steps/s
