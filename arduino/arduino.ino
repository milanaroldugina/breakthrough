// Финальная прошивка Arduino дня 2.
// Учебная архитектура: Arduino выполняет движение и измерения.
// Один и тот же текстовый протокол доступен одновременно:
// 1) через USB Serial для прямого управления из Python;
// 2) через SoftwareSerial для последующего подключения ESP32-C3.
// Перед запуском должны быть физически проверены направления моторов,
// знаки энкодеров, 358 тиков/оборот и питание ESP32-C3.
//
// Команды:
// VEL linear_mm_s angular_mrad_s
// SERVO angle_deg
// RESET_ODOM
// GET
// PING
// STOP
//
// Телеметрия:
// TEL x y theta_mrad v w_mrad encL encR lineL lineR ir_cm servo

#include <Servo.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const uint8_t LEFT_DIR_PIN = 4;
const uint8_t LEFT_PWM_PIN = 5;
const uint8_t RIGHT_PWM_PIN = 6;
const uint8_t RIGHT_DIR_PIN = 7;
const uint8_t LEFT_ENCODER_A_PIN = 2;
const uint8_t LEFT_ENCODER_B_PIN = 8;
const uint8_t RIGHT_ENCODER_A_PIN = 3;
const uint8_t RIGHT_ENCODER_B_PIN = 10;
const uint8_t SERVO_PIN = 9;
const uint8_t LEFT_LINE_PIN = A0;
const uint8_t RIGHT_LINE_PIN = A1;
const uint8_t IR_DISTANCE_PIN = A2;
const uint8_t ESP_RX_PIN = A4;
const uint8_t ESP_TX_PIN = A5;

// Геометрические величины хранятся в миллиметрах. Эти три значения
// определяют масштаб всей одометрии и меняются только после измерения робота.
const float WHEEL_DIAMETER_MM = 44.0f;
const float WHEEL_BASE_MM = 128.0f;
const float ENCODER_TICKS_PER_REV = 358.0f;
const float MM_PER_TICK =
    PI * WHEEL_DIAMETER_MM / ENCODER_TICKS_PER_REV;
const float IR_K = 3782.0f;
const float IR_C = 96.0f;

const unsigned long CONTROL_PERIOD_MS = 50;
const unsigned long TELEMETRY_PERIOD_MS = 200;
const unsigned long VELOCITY_WATCHDOG_MS = 500;
const unsigned long START_BOOST_MS = 250;
const unsigned long SERVO_MOVE_MS = 180;

// Мотор-редуктор не держит скорость ниже устойчивого минимума.
// Ноль всё равно остаётся полной остановкой.
const int MIN_LINEAR_MM_S = 150;
const int MAX_LINEAR_MM_S = 400;
const int MIN_ANGULAR_MRAD_S = 3000;
const int MAX_ANGULAR_MRAD_S = 6000;
const float MAX_WHEEL_SPEED_MM_S = 400.0f;

const int LEFT_START_FORWARD_PWM = 100;
const int LEFT_START_REVERSE_PWM = 100;
const int RIGHT_START_FORWARD_PWM = 90;
const int RIGHT_START_REVERSE_PWM = 110;
const int MIN_HOLD_PWM = 40;
const float SPEED_P_GAIN = 0.20f;
const float MAX_SPEED_CORRECTION_PWM = 60.0f;
const float STRAIGHT_P_GAIN = 1.5f;
const float MAX_STRAIGHT_CORRECTION_MM_S = 35.0f;

// Таблицы получены по измерениям конкретных мотор-редукторов.
// Первая строка — скорость, остальные — требуемый PWM.
const uint8_t MOTOR_TABLE_SIZE = 16;
const int SPEED_TABLE[MOTOR_TABLE_SIZE] = {
  0, 75, 100, 125, 150, 175, 200, 225,
  250, 275, 300, 325, 350, 375, 395, 400
};
const int LEFT_FORWARD_PWM_TABLE[MOTOR_TABLE_SIZE] = {
  0, 40, 45, 49, 54, 59, 66, 73,
  80, 91, 102, 119, 142, 175, 215, 220
};
const int LEFT_REVERSE_PWM_TABLE[MOTOR_TABLE_SIZE] = {
  0, 41, 46, 50, 55, 60, 66, 72,
  80, 90, 102, 117, 138, 171, 210, 220
};
const int RIGHT_FORWARD_PWM_TABLE[MOTOR_TABLE_SIZE] = {
  0, 44, 48, 53, 58, 63, 69, 77,
  85, 95, 108, 124, 147, 178, 220, 220
};
const int RIGHT_REVERSE_PWM_TABLE[MOTOR_TABLE_SIZE] = {
  0, 43, 48, 52, 58, 63, 69, 76,
  85, 95, 108, 125, 147, 181, 219, 220
};

const uint8_t COMMAND_BUFFER_SIZE = 96;

struct MotorPState {
  int8_t previousDirection;
  unsigned long boostUntilMs;
};

SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);
Servo scannerServo;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
long previousLeftTicks = 0;
long previousRightTicks = 0;
long straightStartLeft = 0;
long straightStartRight = 0;

float xMm = 0.0f;
float yMm = 0.0f;
float thetaRad = 0.0f;
float leftSpeedMmS = 0.0f;
float rightSpeedMmS = 0.0f;
float targetLeftMmS = 0.0f;
float targetRightMmS = 0.0f;

int requestedLinearMmS = 0;
int requestedAngularMradS = 0;
int servoAngleDeg = 90;
bool controlEnabled = false;
bool straightActive = false;
unsigned long servoDetachMs = 0;

MotorPState leftMotorState = {0, 0};
MotorPState rightMotorState = {0, 0};

char usbCommandBuffer[COMMAND_BUFFER_SIZE];
uint8_t usbCommandLength = 0;
char espCommandBuffer[COMMAND_BUFFER_SIZE];
uint8_t espCommandLength = 0;
unsigned long previousControlMs = 0;
unsigned long previousTelemetryMs = 0;
unsigned long lastVelocityCommandMs = 0;

void onLeftEncoder() {
  leftTicks += digitalRead(LEFT_ENCODER_B_PIN) ? 1 : -1;
}

void onRightEncoder() {
  rightTicks += digitalRead(RIGHT_ENCODER_B_PIN) ? 1 : -1;
}

void copyTicks(long &left, long &right) {
  noInterrupts();
  left = leftTicks;
  right = rightTicks;
  interrupts();
}

int8_t signOf(float value) {
  if (value > 0.5f) return 1;
  if (value < -0.5f) return -1;
  return 0;
}

float normalizeAngle(float angle) {
  while (angle > PI) angle -= 2.0f * PI;
  while (angle < -PI) angle += 2.0f * PI;
  return angle;
}

int applySignedMinimum(int value, int minimum, int maximum) {
  if (value == 0) return 0;
  const int magnitude = constrain(abs(value), minimum, maximum);
  return value > 0 ? magnitude : -magnitude;
}

void setMotor(uint8_t dirPin, uint8_t pwmPin, int pwm) {
  pwm = constrain(pwm, -255, 255);
  // Для этого шилда модуль PWM одинаков в обоих направлениях:
  // назад означает DIR=HIGH и analogWrite(abs(pwm)).
  digitalWrite(dirPin, pwm < 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(pwm));
}

void setMotors(int leftPwm, int rightPwm) {
  setMotor(LEFT_DIR_PIN, LEFT_PWM_PIN, leftPwm);
  setMotor(RIGHT_DIR_PIN, RIGHT_PWM_PIN, rightPwm);
}

void resetMotorState(MotorPState &state) {
  state.previousDirection = 0;
  state.boostUntilMs = 0;
}

void stopMotion() {
  requestedLinearMmS = 0;
  requestedAngularMradS = 0;
  targetLeftMmS = 0.0f;
  targetRightMmS = 0.0f;
  controlEnabled = false;
  straightActive = false;
  resetMotorState(leftMotorState);
  resetMotorState(rightMotorState);
  setMotors(0, 0);
}

void resetOdometry() {
  copyTicks(previousLeftTicks, previousRightTicks);
  straightStartLeft = previousLeftTicks;
  straightStartRight = previousRightTicks;
  xMm = 0.0f;
  yMm = 0.0f;
  thetaRad = 0.0f;
  leftSpeedMmS = 0.0f;
  rightSpeedMmS = 0.0f;
}

int readIrDistanceCm() {
  const int raw = analogRead(IR_DISTANCE_PIN);
  if (raw >= 700) return 5;
  if (raw < 160) return 60;

  const float distance = IR_K / (raw - IR_C);
  return constrain((int)(distance + 0.5f), 5, 60);
}

int servoPhysicalAngle(int logicalAngle) {
  // Логический угол растёт вправо в интерфейсе; механика сервы развёрнута.
  return 180 - logicalAngle;
}

void moveServo(int angle) {
  angle = constrain(angle, 20, 160);
  if (angle == servoAngleDeg) return;

  servoAngleDeg = angle;
  if (!scannerServo.attached()) {
    scannerServo.attach(SERVO_PIN);
  }
  scannerServo.write(servoPhysicalAngle(servoAngleDeg));
  servoDetachMs = millis() + SERVO_MOVE_MS;
}

void updateServo(unsigned long nowMs) {
  if (scannerServo.attached() &&
      (long)(nowMs - servoDetachMs) >= 0) {
    scannerServo.detach();
  }
}

void setVelocity(int linearMmS, int angularMradS) {
  requestedLinearMmS = applySignedMinimum(
      linearMmS, MIN_LINEAR_MM_S, MAX_LINEAR_MM_S);
  requestedAngularMradS = applySignedMinimum(
      angularMradS, MIN_ANGULAR_MRAD_S, MAX_ANGULAR_MRAD_S);

  const bool newStraight =
      requestedLinearMmS != 0 && requestedAngularMradS == 0;
  if (newStraight && !straightActive) {
    copyTicks(straightStartLeft, straightStartRight);
  }
  straightActive = newStraight;
  controlEnabled =
      requestedLinearMmS != 0 || requestedAngularMradS != 0;
  if (!controlEnabled) stopMotion();
}

void updateOdometry(long deltaLeftTicks, long deltaRightTicks) {
  const float leftDistance = deltaLeftTicks * MM_PER_TICK;
  const float rightDistance = deltaRightTicks * MM_PER_TICK;
  const float distance = 0.5f * (leftDistance + rightDistance);
  const float deltaTheta =
      (rightDistance - leftDistance) / WHEEL_BASE_MM;
  const float middleTheta = thetaRad + 0.5f * deltaTheta;

  xMm += distance * cos(middleTheta);
  yMm += distance * sin(middleTheta);
  thetaRad = normalizeAngle(thetaRad + deltaTheta);
}

int tablePwm(uint8_t index, bool leftMotor, bool forward) {
  if (leftMotor && forward) {
    return LEFT_FORWARD_PWM_TABLE[index];
  }
  if (leftMotor && !forward) {
    return LEFT_REVERSE_PWM_TABLE[index];
  }
  if (!leftMotor && forward) {
    return RIGHT_FORWARD_PWM_TABLE[index];
  }
  return RIGHT_REVERSE_PWM_TABLE[index];
}

int calibratedPwm(float target, bool leftMotor) {
  const bool forward = target > 0.0f;
  const float speed = fabs(target);
  if (speed < 0.5f) return 0;
  if (speed <= SPEED_TABLE[1]) return MIN_HOLD_PWM;

  for (uint8_t index = 1;
       index < MOTOR_TABLE_SIZE - 1;
       index++) {
    if (speed <= SPEED_TABLE[index + 1]) {
      const float speed0 = SPEED_TABLE[index];
      const float speed1 = SPEED_TABLE[index + 1];
      const float pwm0 = tablePwm(index, leftMotor, forward);
      const float pwm1 = tablePwm(index + 1, leftMotor, forward);
      // Линейная интерполяция заполняет промежутки между измеренными точками.
      const float part = (speed - speed0) / (speed1 - speed0);
      return (int)(pwm0 + part * (pwm1 - pwm0));
    }
  }

  return tablePwm(MOTOR_TABLE_SIZE - 1, leftMotor, forward);
}

int startPwm(bool leftMotor, int8_t direction) {
  if (leftMotor && direction > 0) return LEFT_START_FORWARD_PWM;
  if (leftMotor && direction < 0) return LEFT_START_REVERSE_PWM;
  if (!leftMotor && direction > 0) return RIGHT_START_FORWARD_PWM;
  return RIGHT_START_REVERSE_PWM;
}

int calculateMotorPwm(
    float target,
    float measured,
    unsigned long nowMs,
    bool leftMotor,
    MotorPState &state) {
  const int8_t direction = signOf(target);
  if (direction == 0) {
    resetMotorState(state);
    return 0;
  }

  if (direction != state.previousDirection) {
    state.previousDirection = direction;
    state.boostUntilMs = nowMs + START_BOOST_MS;
  }

  if ((long)(state.boostUntilMs - nowMs) > 0) {
    int launchPwm = calibratedPwm(target, leftMotor);
    const int minimumStartPwm = startPwm(leftMotor, direction);
    if (launchPwm < minimumStartPwm) {
      launchPwm = minimumStartPwm;
    }
    return direction * launchPwm;
  }

  const float basePwm = calibratedPwm(target, leftMotor);
  const float error = target - measured;
  const float correction = constrain(
      SPEED_P_GAIN * error,
      -MAX_SPEED_CORRECTION_PWM,
      MAX_SPEED_CORRECTION_PWM);
  const float output = direction * basePwm + correction;
  if (direction > 0) return constrain((int)output, 0, 255);
  return constrain((int)output, -255, 0);
}

void updateControl(unsigned long nowMs) {
  if (nowMs - previousControlMs < CONTROL_PERIOD_MS) return;
  const float dt = (nowMs - previousControlMs) * 0.001f;
  previousControlMs = nowMs;

  long left;
  long right;
  copyTicks(left, right);
  const long deltaLeft = left - previousLeftTicks;
  const long deltaRight = right - previousRightTicks;
  previousLeftTicks = left;
  previousRightTicks = right;

  leftSpeedMmS = deltaLeft * MM_PER_TICK / dt;
  rightSpeedMmS = deltaRight * MM_PER_TICK / dt;
  updateOdometry(deltaLeft, deltaRight);

  if (!controlEnabled) return;

  const float angularRadS = requestedAngularMradS * 0.001f;
  targetLeftMmS =
      requestedLinearMmS - angularRadS * WHEEL_BASE_MM * 0.5f;
  targetRightMmS =
      requestedLinearMmS + angularRadS * WHEEL_BASE_MM * 0.5f;

  if (straightActive) {
    // Сравниваем путь от начала прямого участка. Колесо, ушедшее вперёд,
    // получает меньшую цель, а отставшее — большую.
    const int direction = requestedLinearMmS > 0 ? 1 : -1;
    const float leftProgress =
        (left - straightStartLeft) * MM_PER_TICK * direction;
    const float rightProgress =
        (right - straightStartRight) * MM_PER_TICK * direction;
    const float pathError = leftProgress - rightProgress;
    const float correction = constrain(
        STRAIGHT_P_GAIN * pathError,
        -MAX_STRAIGHT_CORRECTION_MM_S,
        MAX_STRAIGHT_CORRECTION_MM_S);
    targetLeftMmS -= direction * correction;
    targetRightMmS += direction * correction;
  }

  targetLeftMmS = constrain(
      targetLeftMmS, -MAX_WHEEL_SPEED_MM_S, MAX_WHEEL_SPEED_MM_S);
  targetRightMmS = constrain(
      targetRightMmS, -MAX_WHEEL_SPEED_MM_S, MAX_WHEEL_SPEED_MM_S);

  const int leftPwm = calculateMotorPwm(
      targetLeftMmS,
      leftSpeedMmS,
      nowMs,
      true,
      leftMotorState);
  const int rightPwm = calculateMotorPwm(
      targetRightMmS,
      rightSpeedMmS,
      nowMs,
      false,
      rightMotorState);
  setMotors(leftPwm, rightPwm);
}

void sendLine(const char *line) {
  // Ответы и телеметрия одновременно доступны компьютеру по USB
  // и ESP32-C3 по отдельному UART.
  Serial.println(line);
  espSerial.println(line);
}

void sendTelemetry() {
  long left;
  long right;
  copyTicks(left, right);
  const int linearMmS =
      (int)(0.5f * (leftSpeedMmS + rightSpeedMmS));
  const int angularMradS =
      (int)(1000.0f *
            (rightSpeedMmS - leftSpeedMmS) / WHEEL_BASE_MM);

  char line[150];
  snprintf(
      line,
      sizeof(line),
      "TEL %ld %ld %ld %d %d %ld %ld %d %d %d %d",
      (long)xMm,
      (long)yMm,
      (long)(thetaRad * 1000.0f),
      linearMmS,
      angularMradS,
      left,
      right,
      analogRead(LEFT_LINE_PIN),
      analogRead(RIGHT_LINE_PIN),
      readIrDistanceCm(),
      servoAngleDeg);
  sendLine(line);
}

void processCommand(char *line) {
  char *save = NULL;
  char *command = strtok_r(line, " ", &save);
  if (command == NULL) return;

  if (strcmp(command, "VEL") == 0) {
    char *linearText = strtok_r(NULL, " ", &save);
    char *angularText = strtok_r(NULL, " ", &save);
    if (linearText == NULL || angularText == NULL) {
      sendLine("ERR VEL_NEEDS_LINEAR_ANGULAR");
      return;
    }
    setVelocity(atoi(linearText), atoi(angularText));
    lastVelocityCommandMs = millis();
  } else if (strcmp(command, "SERVO") == 0) {
    char *angleText = strtok_r(NULL, " ", &save);
    if (angleText == NULL) {
      sendLine("ERR SERVO_NEEDS_ANGLE");
      return;
    }
    moveServo(atoi(angleText));
  } else if (strcmp(command, "RESET_ODOM") == 0) {
    stopMotion();
    resetOdometry();
    sendLine("OK RESET_ODOM");
  } else if (strcmp(command, "STOP") == 0) {
    stopMotion();
  } else if (strcmp(command, "PING") == 0) {
    sendLine("PONG ARDUINO");
  } else if (strcmp(command, "GET") == 0) {
    sendTelemetry();
  } else {
    sendLine("ERR UNKNOWN_COMMAND");
  }
}

void readCommands(
    Stream &stream,
    char *buffer,
    uint8_t &length) {
  // USB, UART и TCP являются потоками. Команду разбираем только
  // после получения символа конца строки.
  while (stream.available()) {
    const char symbol = stream.read();
    if (symbol == '\n' || symbol == '\r') {
      if (length > 0) {
        buffer[length] = '\0';
        processCommand(buffer);
        length = 0;
      }
    } else if (length < COMMAND_BUFFER_SIZE - 1) {
      buffer[length++] = symbol;
    } else {
      length = 0;
      sendLine("ERR LINE_TOO_LONG");
    }
  }
}

void readUsbCommands() {
  readCommands(Serial, usbCommandBuffer, usbCommandLength);
}

void readEspCommands() {
  readCommands(espSerial, espCommandBuffer, espCommandLength);
}

void checkWatchdog(unsigned long nowMs) {
  if (controlEnabled &&
      nowMs - lastVelocityCommandMs > VELOCITY_WATCHDOG_MS) {
    stopMotion();
    sendLine("EVENT WATCHDOG_STOP");
  }
}

void setup() {
  Serial.begin(115200);
  espSerial.begin(38400);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(LEFT_ENCODER_A_PIN, INPUT);
  pinMode(LEFT_ENCODER_B_PIN, INPUT);
  pinMode(RIGHT_ENCODER_A_PIN, INPUT);
  pinMode(RIGHT_ENCODER_B_PIN, INPUT);
  pinMode(LEFT_LINE_PIN, INPUT);
  pinMode(RIGHT_LINE_PIN, INPUT);
  pinMode(IR_DISTANCE_PIN, INPUT);
  attachInterrupt(
      digitalPinToInterrupt(LEFT_ENCODER_A_PIN), onLeftEncoder, RISING);
  attachInterrupt(
      digitalPinToInterrupt(RIGHT_ENCODER_A_PIN), onRightEncoder, RISING);
  scannerServo.attach(SERVO_PIN);
  scannerServo.write(servoPhysicalAngle(servoAngleDeg));
  servoDetachMs = millis() + SERVO_MOVE_MS;
  stopMotion();
  resetOdometry();
  previousControlMs = millis();
  previousTelemetryMs = millis();
  sendLine("READY ARDUINO_DAY2");
}

void loop() {
  readUsbCommands();
  readEspCommands();
  const unsigned long nowMs = millis();
  checkWatchdog(nowMs);
  updateControl(nowMs);
  updateServo(nowMs);

  if (nowMs - previousTelemetryMs >= TELEMETRY_PERIOD_MS) {
    previousTelemetryMs = nowMs;
    sendTelemetry();
  }
}
