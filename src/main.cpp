// ============================================================
// 双 T8 双向丝杆步进驱动 (ESP8266 下位机)
//
// 机械: 42步进电机双出轴位于丝杆中点, 左端正牙右端反牙,
//       电机一转两侧螺母反向等速移动. 导程 2mm, 单边行程 67mm.
//
// 接线 (每电机 STEP/DIR/EN/GND 四线):
//   电机1: STEP=IO13 DIR=IO12 EN=IO14
//   电机2: STEP=IO15 DIR=IO4  EN=IO2
//   TXD/RXD 保留为树莓派4B (ROS 2 Humble) 串口命令链路
//   IO0 不使用(启动模式脚); IO5/IO16/ADC 备用
//
// 状态机 (每个控制组独立):
//   AT_OUTER(外侧自锁) --IN-->  MOVING_IN --到位--> AT_INNER(内侧自锁)
//   AT_INNER           --OUT--> MOVING_OUT --到位--> AT_OUTER
//   运动中 STOP -> HOLD_MID(原地自锁), 之后仍可继续 IN/OUT 走完剩余行程
//   自锁 = 保持 EN 使能, 电机带保持转矩 (T8 2mm 导程本身机械自锁,
//   保持使能是为了消除齿隙游动; 发热敏感可用 RELAX 释放)
//
// 串口协议 (115200 8N1, 每行一条命令, 大小写不敏感):
//   IN 1 | IN 2 | IN     组1/组2/两组同时 向内运动 67mm
//   OUT 1| OUT 2| OUT    向外运动 67mm (回到外侧)
//   STOP               两组立即停止, 保持自锁
//   RELAX [1|2]        释放使能(不再自锁, 省电降温)
//   LOCK  [1|2]        恢复使能(自锁)
//   SPEED <steps/s>    设置脉冲频率 (默认 1600)
//   POS                查询状态与位置
//   应答: OK ... / ERR ...; 运动到位主动上报: DONE G1 IN pos=67.00mm
//
// 注意: 上电默认假定螺母处于外侧(位置=0). 若不然请先手动归位再上电,
//       或后续加装限位开关做归零.
// ============================================================
#include <Arduino.h>

// ---------- 引脚 ----------
#define M1_STEP 13
#define M1_DIR  12
#define M1_EN   14
#define M2_STEP 15
#define M2_DIR  4
#define M2_EN   2

#define EN_ACTIVE_LOW 1   // TB6600/DM542/A4988 均为低电平使能

// ---------- 机械参数 ----------
const float LEAD_MM      = 2.0f;               // T8 导程 2mm
const float TRAVEL_MM    = 67.0f;              // 单边行程
const int   MICROSTEPS   = 8;                  // 按驱动板拨码修改
const long  STEPS_PER_REV = 200L * MICROSTEPS;
const long  TRAVEL_STEPS  = (long)(TRAVEL_MM / LEAD_MM * STEPS_PER_REV + 0.5f);

// DIR 电平与"向内"的对应关系, 实际相反时对调 HIGH/LOW
#define DIR_IN HIGH

const uint32_t DEFAULT_SPEED = 1600;           // steps/s

// ---------- 状态机 ----------
enum State : uint8_t { AT_OUTER, MOVING_IN, AT_INNER, MOVING_OUT, HOLD_MID };
const char* stateName(State s) {
  switch (s) {
    case AT_OUTER:   return "AT_OUTER";
    case MOVING_IN:  return "MOVING_IN";
    case AT_INNER:   return "AT_INNER";
    case MOVING_OUT: return "MOVING_OUT";
    case HOLD_MID:   return "HOLD_MID";
  }
  return "?";
}

struct Motor {
  uint8_t pinStep, pinDir, pinEn;
  State   state;
  long    pos;            // 相对外侧的位置, 单位步 (0=外侧, TRAVEL_STEPS=内侧)
  long    target;         // 运动目标位置(步)
  bool    stepLevel;      // STEP 当前电平
  uint32_t nextToggle;    // 下一次翻转 STEP 的 micros() 时刻
  bool    enabled;        // EN 是否使能(自锁)
};

Motor motors[2] = {
  { M1_STEP, M1_DIR, M1_EN, AT_OUTER, 0, 0, false, 0, false },
  { M2_STEP, M2_DIR, M2_EN, AT_OUTER, 0, 0, false, 0, false },
};

uint32_t speedSteps = DEFAULT_SPEED;           // steps/s
uint32_t halfPeriodUs = 1000000UL / DEFAULT_SPEED / 2;

// ---------- 底层 ----------
static void setEnable(Motor& m, bool on) {
  m.enabled = on;
#if EN_ACTIVE_LOW
  digitalWrite(m.pinEn, on ? LOW : HIGH);
#else
  digitalWrite(m.pinEn, on ? HIGH : LOW);
#endif
}

static void setDirInward(Motor& m, bool inward) {
  digitalWrite(m.pinDir, inward ? DIR_IN : !DIR_IN);
}

// 非阻塞脉冲发生: 两个电机可同时运动, 且运动期间串口照常响应
static void updateMotor(Motor& m) {
  if (m.state != MOVING_IN && m.state != MOVING_OUT) return;
  uint32_t now = micros();
  if ((int32_t)(now - m.nextToggle) < 0) return;

  m.stepLevel = !m.stepLevel;
  digitalWrite(m.pinStep, m.stepLevel ? HIGH : LOW);
  m.nextToggle = now + halfPeriodUs;
  if (m.stepLevel) return;                     // 只在完整脉冲(下降沿)计数

  m.pos += (m.state == MOVING_IN) ? 1 : -1;

  bool done = (m.state == MOVING_IN)  ? (m.pos >= m.target)
                                      : (m.pos <= m.target);
  if (done) {
    m.pos = constrain(m.pos, 0L, TRAVEL_STEPS);
    State arrive = (m.state == MOVING_IN) ? AT_INNER : AT_OUTER;
    const char* verb = (m.state == MOVING_IN) ? "IN" : "OUT";
    m.state = arrive;                          // 保持 EN 使能 = 自锁
    Serial.printf("DONE G%d %s pos=%.2fmm\n",
                  (int)(&m - motors) + 1, verb, m.pos * LEAD_MM / STEPS_PER_REV);
  }
}

static void startMove(int idx, bool inward) {
  Motor& m = motors[idx];
  if (m.state == MOVING_IN || m.state == MOVING_OUT) {
    Serial.printf("ERR G%d busy (%s)\n", idx + 1, stateName(m.state));
    return;
  }
  if (inward && m.pos >= TRAVEL_STEPS) { Serial.printf("ERR G%d already inner\n", idx + 1); return; }
  if (!inward && m.pos <= 0) { Serial.printf("ERR G%d already outer\n", idx + 1); return; }

  m.target   = inward ? TRAVEL_STEPS : 0;
  m.state    = inward ? MOVING_IN : MOVING_OUT;
  m.stepLevel = false;
  digitalWrite(m.pinStep, LOW);
  setDirInward(m, inward);
  setEnable(m, true);
  delayMicroseconds(20);                       // DIR 建立时间, 驱动器要求 DIR 先于 STEP
  m.nextToggle = micros();
  Serial.printf("OK G%d %s start, %ld steps (%.1fmm)\n",
                idx + 1, inward ? "IN" : "OUT",
                labs(m.target - m.pos), TRAVEL_MM);
}

static void stopAll() {
  for (auto& m : motors) {
    if (m.state == MOVING_IN || m.state == MOVING_OUT) {
      if (m.pos <= 0) m.state = AT_OUTER;
      else if (m.pos >= TRAVEL_STEPS) m.state = AT_INNER;
      else m.state = HOLD_MID;               // 中途停止: 原地自锁, 可继续 IN/OUT
    }
    digitalWrite(m.pinStep, LOW);
    m.stepLevel = false;
  }
  Serial.println("OK STOP (holding)");
}

static void printStatus() {
  for (int i = 0; i < 2; i++) {
    Motor& m = motors[i];
    Serial.printf("G%d state=%s pos=%.2fmm steps=%ld en=%d\n",
                  i + 1, stateName(m.state),
                  m.pos * LEAD_MM / STEPS_PER_REV, m.pos, m.enabled ? 1 : 0);
  }
  Serial.printf("SPEED %lu steps/s (%.2f mm/s, 全程约 %.1f s)\n",
                (unsigned long)speedSteps,
                speedSteps * LEAD_MM / STEPS_PER_REV,
                (double)TRAVEL_STEPS / speedSteps);
}

// ---------- 命令解析 ----------
static char lineBuf[48];
static uint8_t lineLen = 0;

static void handleCommand(char* cmd) {
  // 转大写
  for (char* p = cmd; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;

  if (!strcmp(cmd, "POS") || !strcmp(cmd, "STATUS")) { printStatus(); return; }
  if (!strcmp(cmd, "STOP")) { stopAll(); return; }

  if (!strncmp(cmd, "SPEED", 5)) {
    uint32_t v = atoi(cmd + 5);
    if (v >= 50 && v <= 20000) {
      speedSteps = v;
      halfPeriodUs = 1000000UL / v / 2;
      Serial.printf("OK SPEED %lu\n", (unsigned long)v);
    } else Serial.println("ERR SPEED range 50..20000");
    return;
  }

  if (!strncmp(cmd, "RELAX", 5) || !strncmp(cmd, "LOCK", 4)) {
    bool relax = (cmd[0] == 'R');
    int which = 0;
    const char* arg = cmd + (relax ? 5 : 4);
    while (*arg == ' ') arg++;
    if (*arg >= '1' && *arg <= '2') which = *arg - '0';
    for (int i = 0; i < 2; i++) {
      if (which && which != i + 1) continue;
      setEnable(motors[i], !relax);
    }
    Serial.printf("OK %s\n", relax ? "RELAX" : "LOCK");
    return;
  }

  if (!strncmp(cmd, "IN", 2) || !strncmp(cmd, "OUT", 3)) {
    bool inward = (cmd[0] == 'I');
    const char* arg = cmd + (inward ? 2 : 3);
    while (*arg == ' ') arg++;
    int which = 0;                             // 0 = 两组
    if (*arg >= '1' && *arg <= '2') which = *arg - '0';
    for (int i = 0; i < 2; i++) {
      if (which && which != i + 1) continue;
      startMove(i, inward);
    }
    return;
  }

  Serial.printf("ERR unknown cmd: %s\n", cmd);
}

static void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = 0;
      if (lineLen) handleCommand(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ---------- 主程序 ----------
void setup() {
  Serial.begin(115200);
  for (auto& m : motors) {
    pinMode(m.pinStep, OUTPUT);
    pinMode(m.pinDir, OUTPUT);
    pinMode(m.pinEn, OUTPUT);
    digitalWrite(m.pinStep, LOW);
    setEnable(m, false);                       // 上电先失能
  }
  Serial.println("\n=== dual T8 bidirectional leadscrew driver ===");
  Serial.println("G1: STEP=IO13 DIR=IO12 EN=IO14 | G2: STEP=IO15 DIR=IO4 EN=IO2");
  Serial.printf("lead=%.1fmm travel=%.1fmm microsteps=%d -> %ld steps/travel\n",
                LEAD_MM, TRAVEL_MM, MICROSTEPS, TRAVEL_STEPS);
  Serial.println("cmds: IN[1|2] OUT[1|2] STOP RELAX[1|2] LOCK[1|2] SPEED n POS");
  Serial.println("WARN: 上电假定螺母在外侧(pos=0)");
}

void loop() {
  pollSerial();
  updateMotor(motors[0]);
  updateMotor(motors[1]);
  yield();
}
