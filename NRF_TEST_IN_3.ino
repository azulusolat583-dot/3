#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// ----------------- НАСТРОЙКИ ПИНОВ -----------------

const uint8_t PIN_CE  = 9;
const uint8_t PIN_CSN = 3;

const uint8_t PIN_SERVO_TILT = 7;
const uint8_t PIN_SERVO_PAN  = 6;

const uint8_t PIN_LASER     = 5;
const uint8_t PIN_EMERGENCY = 4;

// ----------------- ОБЪЕКТЫ -----------------

RF24 radio(PIN_CE, PIN_CSN);
Servo servoTilt;
Servo servoPan;

// адреса
const byte addressBase[6] = "BASE1";  // база
const byte addressCube[6] = "CUBE1";  // кубсат (приём команд)

const uint8_t DEVICE_ID = 1;

// ----------------- НАСТРОЙКИ РЕЖИМОВ -----------------

const uint16_t laser_delay = 2500;  // мс
const uint16_t servo_delay = 400; // пауза между шагами (для ТЗ)

// ----------------- ТИПЫ ДАННЫХ -----------------

enum CommandType : uint8_t {
  CMD_NONE = 0,
  CMD_TEST_ALL,
  CMD_TEST_LASER,
  CMD_TEST_SERVOS,
  CMD_TEST_RADIO,
  CMD_TEST_MCU,
  CMD_SCAN_HORIZONTAL,
  CMD_SCAN_VERTICAL,
  CMD_SCAN_DIAG1,
  CMD_SCAN_DIAG2,
  CMD_RESET_TO_ZERO,
  CMD_SCAN_ALL          // полный цикл сканирования
};

enum ModeState : uint8_t {
  MODE_IDLE = 0,
  MODE_LISTEN,
  MODE_SCAN_H,
  MODE_SCAN_V,
  MODE_SCAN_D1,
  MODE_SCAN_D2,
  MODE_TEST
};

enum TestType : uint8_t {
  TEST_NONE = 0,
  TEST_LASER,
  TEST_SERVOS,
  TEST_RADIO,
  TEST_MCU,
  TEST_ALL
};

struct CommandPacket {
  uint8_t     deviceId;
  CommandType cmd;
};

struct TelemetryPacket {
  uint8_t   deviceId;
  int8_t    angleTilt;
  int8_t    anglePan;
  ModeState mode;
};

struct TestTelemetryPacket {
  uint8_t   deviceId;
  TestType  test;
  bool      started;
  bool      finished;
  bool      resultOK;
};

// ----------------- СОСТОЯНИЕ -----------------

int8_t    curTiltDeg = 0;
int8_t    curPanDeg  = 0;
ModeState curMode    = MODE_IDLE;

// ----------------- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ -----------------

int angleToServo(int8_t angleDeg) {
  if (angleDeg < -40) angleDeg = -40;
  if (angleDeg >  40) angleDeg =  40;
  return map(angleDeg, -40, 40, 50, 130);
}

bool emergencyPressed() {
  return digitalRead(PIN_EMERGENCY) == LOW;
}

void moveToAngles(int8_t tiltDeg, int8_t panDeg, uint16_t stepDelayMs = 20) {
  int targetTilt = constrain(tiltDeg, -40, 40);
  int targetPan  = constrain(panDeg, -40, 40);

  int startTilt = curTiltDeg;
  int startPan  = curPanDeg;

  int steps = max(abs(targetTilt - startTilt), abs(targetPan - startPan));
  if (steps == 0) {
    servoTilt.write(angleToServo(targetTilt));
    servoPan.write(angleToServo(targetPan));
    curTiltDeg = targetTilt;
    curPanDeg  = targetPan;
    return;
  }

  for (int i = 1; i <= steps; i++) {
    if (emergencyPressed()) break;
    int aTilt = startTilt + (targetTilt - startTilt) * i / steps;
    int aPan  = startPan  + (targetPan  - startPan)  * i / steps;
    servoTilt.write(angleToServo(aTilt));
    servoPan.write(angleToServo(aPan));
    delay(stepDelayMs);
  }
  curTiltDeg = targetTilt;
  curPanDeg  = targetPan;
}

// ----------------- ОТПРАВКА ТЕЛЕМЕТРИИ -----------------

void sendTelemetry() {
  TelemetryPacket pkt;
  pkt.deviceId  = DEVICE_ID;
  pkt.angleTilt = -curTiltDeg;
  pkt.anglePan  = curPanDeg;
  pkt.mode      = curMode;

  radio.stopListening();
  radio.openWritingPipe(addressBase);   // отправляем на базу
  radio.write(&pkt, sizeof(pkt));
  radio.startListening();
}

void sendTestTelemetry(TestType t, bool started, bool finished, bool resultOK) {
  TestTelemetryPacket pkt;
  pkt.deviceId = DEVICE_ID;
  pkt.test     = t;
  pkt.started  = started;
  pkt.finished = finished;
  pkt.resultOK = resultOK;

  radio.stopListening();
  radio.openWritingPipe(addressBase);   // отправляем на базу
  radio.write(&pkt, sizeof(pkt));
  radio.startListening();
}

// ----------------- БЛОК ТЕСТОВ -----------------

void testLaser() {
  curMode = MODE_TEST;
  sendTestTelemetry(TEST_LASER, true, false, true);

  for (int i = 0; i < 3; i++) {
    if (emergencyPressed()) {
      sendTestTelemetry(TEST_LASER, false, true, false);
      return;
    }
    digitalWrite(PIN_LASER, HIGH);
    delay(300);
    digitalWrite(PIN_LASER, LOW);
    delay(300);
  }

  sendTestTelemetry(TEST_LASER, false, true, true);
}

void testServos() {
  curMode = MODE_TEST;
  sendTestTelemetry(TEST_SERVOS, true, false, true);

  moveToAngles(0, 0, 10);
  delay(300);
  if (emergencyPressed()) { sendTestTelemetry(TEST_SERVOS, false, true, false); return; }

  moveToAngles(-40, -40, 10);
  delay(500);
  if (emergencyPressed()) { sendTestTelemetry(TEST_SERVOS, false, true, false); return; }

  moveToAngles(40, 40, 10);
  delay(500);
  if (emergencyPressed()) { sendTestTelemetry(TEST_SERVOS, false, true, false); return; }

  moveToAngles(0, 0, 10);
  delay(300);

  sendTestTelemetry(TEST_SERVOS, false, true, true);
}

void testRadio() {
  curMode = MODE_TEST;
  sendTestTelemetry(TEST_RADIO, true, false, true);

  for (int i = 0; i < 3; i++) {
    if (emergencyPressed()) { sendTestTelemetry(TEST_RADIO, false, true, false); return; }
    sendTelemetry();
    delay(500);
  }

  sendTestTelemetry(TEST_RADIO, false, true, true);
}

void testMCU() {
  curMode = MODE_TEST;
  sendTestTelemetry(TEST_MCU, true, false, true);

  for (int i = 0; i < 2; i++) {
    if (emergencyPressed()) { sendTestTelemetry(TEST_MCU, false, true, false); return; }
    digitalWrite(PIN_LASER, HIGH);
    delay(100);
    digitalWrite(PIN_LASER, LOW);
    delay(100);
  }

  sendTestTelemetry(TEST_MCU, false, true, true);
}

void testAll() {
  curMode = MODE_TEST;
  sendTestTelemetry(TEST_ALL, true, false, true);

  testLaser();
  testServos();
  testRadio();
  testMCU();

  sendTestTelemetry(TEST_ALL, false, true, true);
}

// ----------------- РЕЖИМЫ СКАНИРОВАНИЯ -----------------

void modeHorizontalScan() {
  curMode = MODE_SCAN_H;

  for (int tilt = 40; tilt >= -40; tilt -= 10) {
    if (emergencyPressed()) break;

    moveToAngles(tilt, 0);
    delay(100);

    digitalWrite(PIN_LASER, HIGH);
    sendTelemetry();
    delay(laser_delay);
    digitalWrite(PIN_LASER, LOW);

    delay(servo_delay);
  }
}

void modeVerticalScan() {
  curMode = MODE_SCAN_V;

  for (int pan = -40; pan <= 40; pan += 10) {
    if (emergencyPressed()) break;

    moveToAngles(0, pan);
    delay(100);

    digitalWrite(PIN_LASER, HIGH);
    sendTelemetry();
    delay(laser_delay);
    digitalWrite(PIN_LASER, LOW);

    delay(servo_delay);
  }
}

void modeDiagonalScan1() {
  curMode = MODE_SCAN_D1;

  for (int a = -40; a <= 40; a += 10) {
    if (emergencyPressed()) break;

    moveToAngles(-a, a);
    delay(100);

    digitalWrite(PIN_LASER, HIGH);
    sendTelemetry();
    delay(laser_delay);
    digitalWrite(PIN_LASER, LOW);

    delay(servo_delay);
  }
}

void modeDiagonalScan2() {
  curMode = MODE_SCAN_D2;

  for (int a = -40; a <= 40; a += 10) {
    if (emergencyPressed()) break;

    moveToAngles(-a, -a);
    delay(100);

    digitalWrite(PIN_LASER, HIGH);
    sendTelemetry();
    delay(laser_delay);
    digitalWrite(PIN_LASER, LOW);
    delay(servo_delay);
  }

}

void modeScanAll() {
  modeHorizontalScan();
  if (emergencyPressed()) return;
  modeVerticalScan();
  if (emergencyPressed()) return;
  modeDiagonalScan1();
  if (emergencyPressed()) return;
  modeDiagonalScan2();
}

void goToZero() {
  moveToAngles(0, 0, 10);
  curMode = MODE_IDLE;
  sendTelemetry();
}

// ----------------- ОБРАБОТКА КОМАНД -----------------

void handleCommand(const CommandPacket &cmd) {
  if (cmd.deviceId != DEVICE_ID && cmd.deviceId != 255) {
    return;
  }

  switch (cmd.cmd) {
    case CMD_TEST_ALL:
      testAll();
      goToZero();
      break;

    case CMD_TEST_LASER:
      testLaser();
      break;

    case CMD_TEST_SERVOS:
      testServos();
      break;

    case CMD_TEST_RADIO:
      testRadio();
      break;

    case CMD_TEST_MCU:
      testMCU();
      break;

    case CMD_SCAN_HORIZONTAL:
      modeHorizontalScan();
      goToZero();
      break;

    case CMD_SCAN_VERTICAL:
      modeVerticalScan();
      goToZero();
      break;

    case CMD_SCAN_DIAG1:
      modeDiagonalScan1();
      goToZero();
      break;

    case CMD_SCAN_DIAG2:
      modeDiagonalScan2();
      goToZero();
      break;

    case CMD_SCAN_ALL:
      modeScanAll();
      goToZero();
      break;

    case CMD_RESET_TO_ZERO:
      goToZero();
      break;

    default:
      break;
  }

  curMode = MODE_LISTEN;
}

// ----------------- SETUP / LOOP -----------------

void setup() {
  pinMode(PIN_LASER, OUTPUT);
  digitalWrite(PIN_LASER, LOW);

  pinMode(PIN_EMERGENCY, INPUT_PULLUP);

  servoTilt.attach(PIN_SERVO_TILT);
  servoPan.attach(PIN_SERVO_PAN);

  moveToAngles(0, 0, 10);

  radio.begin();
  radio.setChannel(90);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);

  // Кубсат принимает команды на свой адрес
  radio.openReadingPipe(1, addressCube);
  radio.startListening();

  curMode = MODE_LISTEN;

  // сразу отправим одну телеметрию после старта
  delay(500);
  sendTelemetry();
}

void loop() {
  if (emergencyPressed()) {
    digitalWrite(PIN_LASER, LOW);
    delay(100);
    return;
  }

  if (radio.available()) {
    CommandPacket cmd;
    radio.read(&cmd, sizeof(cmd));
    handleCommand(cmd);
  }

  static unsigned long lastPing = 0;
  if (millis() - lastPing > 5000) {
    lastPing = millis();
    if (curMode == MODE_LISTEN || curMode == MODE_IDLE) {
      sendTelemetry();
    }
  }
}
