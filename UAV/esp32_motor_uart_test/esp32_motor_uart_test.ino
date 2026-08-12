#include <Arduino.h>

// GOOUUU ESP32-S3 N16R8 -> Yahboom 4-channel motor driver UART
// ESP32 G18 (RX) <- Yahboom TX2 (yellow wire)
// ESP32 G17 (TX) -> Yahboom RX2 (green wire)
constexpr int MOTOR_RX = 18;
constexpr int MOTOR_TX = 17;
constexpr int TEST_SPEED = 100;
constexpr uint32_t TEST_TIME_MS = 400;
constexpr int TEST_PWM = 600;
constexpr uint32_t PWM_TEST_TIME_MS = 300;

HardwareSerial MotorSerial(1);

void stopAll() {
  // First request zero closed-loop speed, then release PWM output.
  MotorSerial.print("$spd:0,0,0,0#");
  MotorSerial.flush();
  delay(10);
  MotorSerial.print("$pwm:0,0,0,0#");
  MotorSerial.flush();
}

void jogMotor(uint8_t index) {
  int speedValue[4] = {0, 0, 0, 0};
  speedValue[index] = TEST_SPEED;

  char command[48];
  snprintf(command, sizeof(command), "$spd:%d,%d,%d,%d#",
           speedValue[0], speedValue[1], speedValue[2], speedValue[3]);

  Serial.print("Send: ");
  Serial.println(command);
  MotorSerial.print(command);
  MotorSerial.flush();

  delay(TEST_TIME_MS);
  stopAll();
  Serial.println("Stopped");
}

void jogMotorPwm(uint8_t index) {
  int pwmValue[4] = {0, 0, 0, 0};
  pwmValue[index] = TEST_PWM;

  char command[48];
  snprintf(command, sizeof(command), "$pwm:%d,%d,%d,%d#",
           pwmValue[0], pwmValue[1], pwmValue[2], pwmValue[3]);

  Serial.print("Send direct PWM: ");
  Serial.println(command);
  MotorSerial.print(command);
  MotorSerial.flush();

  delay(PWM_TEST_TIME_MS);
  stopAll();
  Serial.println("Stopped");
}

void printHelp() {
  Serial.println();
  Serial.println("Motor test ready");
  Serial.println("1-4: jog corresponding motor for 0.4 seconds");
  Serial.println("A-D: direct-PWM jog corresponding motor for 0.3 seconds");
  Serial.println("S: emergency stop");
  Serial.println("V: request battery voltage");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Arduino ESP32 API order: baud, format, RX pin, TX pin.
  MotorSerial.begin(115200, SERIAL_8N1, MOTOR_RX, MOTOR_TX);
  stopAll();
  printHelp();
}

void loop() {
  while (MotorSerial.available()) {
    Serial.write(MotorSerial.read());
  }

  if (!Serial.available()) {
    return;
  }

  const char command = Serial.read();
  if (command >= '1' && command <= '4') {
    jogMotor(static_cast<uint8_t>(command - '1'));
  } else if (command >= 'a' && command <= 'd') {
    jogMotorPwm(static_cast<uint8_t>(command - 'a'));
  } else if (command >= 'A' && command <= 'D') {
    jogMotorPwm(static_cast<uint8_t>(command - 'A'));
  } else if (command == 's' || command == 'S') {
    stopAll();
    Serial.println("Emergency stop");
  } else if (command == 'v' || command == 'V') {
    MotorSerial.print("$read_vol#");
    MotorSerial.flush();
  } else if (command == 'h' || command == 'H' || command == '?') {
    printHelp();
  }
}
