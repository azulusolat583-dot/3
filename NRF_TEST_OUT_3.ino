#include <SPI.h>
#include <RF24.h>



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
  CMD_SCAN_ALL
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


const uint8_t PIN_CE  = 7;
const uint8_t PIN_CSN = 8;

RF24 radio(PIN_CE, PIN_CSN);

const byte addressBase[6] = "BASE1";  // база (приём)
const byte addressCube[6] = "CUBE1";  // кубсат (команды)

const uint8_t TARGET_DEVICE_ID = 1;


void sendCommand(CommandType cmd) {
  CommandPacket pkt;
  pkt.deviceId = TARGET_DEVICE_ID;
  pkt.cmd      = cmd;

  radio.stopListening();
  radio.openWritingPipe(addressCube);   // шлём на кубсат
  bool ok = radio.write(&pkt, sizeof(pkt));
  radio.startListening();

  Serial.print("CMD sent: ");
  Serial.print((int)cmd);
  Serial.print(" status=");
  Serial.println(ok ? "OK" : "FAIL");
}

const char* modeToStr(ModeState m) {
  switch (m) {
    case MODE_IDLE:   return "IDLE";
    case MODE_LISTEN: return "LISTEN";
    case MODE_SCAN_H: return "SCAN_H";
    case MODE_SCAN_V: return "SCAN_V";
    case MODE_SCAN_D1:return "SCAN_D1";
    case MODE_SCAN_D2:return "SCAN_D2";
    case MODE_TEST:   return "TEST";
    default:          return "UNKNOWN";
  }
}

const char* testToStr(TestType t) {
  switch (t) {
    case TEST_LASER:   return "LASER";
    case TEST_SERVOS:  return "SERVOS";
    case TEST_RADIO:   return "RADIO";
    case TEST_MCU:     return "MCU";
    case TEST_ALL:     return "ALL";
    default:           return "NONE";
  }
}

// ---------- SETUP / LOOP ----------

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Base station ready.");
  Serial.println("Commands:");
  Serial.println(" 1 - TEST_ALL");
  Serial.println(" 2 - TEST_LASER");
  Serial.println(" 3 - TEST_SERVOS");
  Serial.println(" 4 - TEST_RADIO");
  Serial.println(" 5 - TEST_MCU");
  Serial.println(" h - HORIZONTAL SCAN");
  Serial.println(" v - VERTICAL SCAN");
  Serial.println(" d - DIAGONAL 1");
  Serial.println(" f - DIAGONAL 2");
  Serial.println(" s - SCAN ALL (H, V, D1, D2)");
  Serial.println(" 0 - RESET TO ZERO");

  radio.begin();
  radio.setChannel(90);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);

  radio.openReadingPipe(1, addressBase);
  radio.startListening();
}

void loop() {
  // --- чтение команд с компьютера ---
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1': sendCommand(CMD_TEST_ALL);        break;
      case '2': sendCommand(CMD_TEST_LASER);      break;
      case '3': sendCommand(CMD_TEST_SERVOS);     break;
      case '4': sendCommand(CMD_TEST_RADIO);      break;
      case '5': sendCommand(CMD_TEST_MCU);        break;
      case 'h': sendCommand(CMD_SCAN_HORIZONTAL); break;
      case 'v': sendCommand(CMD_SCAN_VERTICAL);   break;
      case 'd': sendCommand(CMD_SCAN_DIAG1);      break;
      case 'f': sendCommand(CMD_SCAN_DIAG2);      break;
      case 's': sendCommand(CMD_SCAN_ALL);        break;
      case '0': sendCommand(CMD_RESET_TO_ZERO);   break;
      default: break;
    }
  }

  // --- приём телеметрии от кубсата ---
   if (radio.available()) {
    uint8_t buf[32];
    radio.read(&buf, sizeof(buf));

    // Пытаемся сначала как TestTelemetryPacket
    TestTelemetryPacket *tt = (TestTelemetryPacket*)buf;

    bool looksLikeTest =
      tt->deviceId == TARGET_DEVICE_ID &&
      tt->test >= TEST_LASER && tt->test <= TEST_ALL;

    if (looksLikeTest) {
      Serial.print("[TEST] ID=");
      Serial.print(tt->deviceId);
      Serial.print(" test=");
      Serial.print(testToStr(tt->test));
      Serial.print(" started=");
      Serial.print(tt->started ? "1" : "0");
      Serial.print(" finished=");
      Serial.print(tt->finished ? "1" : "0");
      Serial.print(" ok=");
      Serial.println(tt->resultOK ? "1" : "0");
    } else {
      TelemetryPacket *tp = (TelemetryPacket*)buf;

      Serial.print("[TEL] ID=");
      Serial.print(tp->deviceId);
      Serial.print(" mode=");
      Serial.print(modeToStr(tp->mode));
      Serial.print(" tilt=");
      Serial.print(tp->angleTilt);
      Serial.print(" pan=");
      Serial.println(tp->anglePan);
    }
  }
}
