// BarVBT firmware: Seeed XIAO nRF52840 Sense
// Streams the onboard LSM6DS3TR-C (accel + gyro) at 100 Hz over BLE notifications.
//
// Arduino setup:
//   Boards Manager URL: https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
//   Board package: "Seeed nRF52 Boards" (the non-mbed one, which includes Bluefruit)
//   Board: "Seeed XIAO nRF52840 Sense"
//   Library: "Seeed Arduino LSM6DS3"
//
// Packet (little-endian): uint16 seq (index of first sample, wraps at 65536),
// then N x { int16 ax, ay, az [mg]; int16 gx, gy, gz [0.1 deg/s] }.
// N is as many samples as fit in the negotiated MTU (max 5 → one packet per 50 ms).

#include <bluefruit.h>
#include <Wire.h>
#include <LSM6DS3.h>

// 128-bit UUIDs in little-endian byte order.
// Service b1a7e001-5c3b-4f0e-9a1e-7f2d0b6c0001, characteristic b1a7e002-5c3b-4f0e-9a1e-7f2d0b6c0001
const uint8_t SVC_UUID[16] = {0x01,0x00,0x6c,0x0b,0x2d,0x7f,0x1e,0x9a,0x0e,0x4f,0x3b,0x5c,0x01,0xe0,0xa7,0xb1};
const uint8_t CHR_UUID[16] = {0x01,0x00,0x6c,0x0b,0x2d,0x7f,0x1e,0x9a,0x0e,0x4f,0x3b,0x5c,0x02,0xe0,0xa7,0xb1};

BLEService        vbtService(SVC_UUID);
BLECharacteristic imuChar(CHR_UUID);
LSM6DS3           imu(I2C_MODE, 0x6A);

const uint32_t SAMPLE_US   = 10000;  // 100 Hz
const uint8_t  MAX_BATCH   = 5;
const uint8_t  SAMPLE_SIZE = 12;

uint16_t connHandle = BLE_CONN_HANDLE_INVALID;
uint16_t seq = 0;             // index of next sample
uint8_t  pkt[2 + MAX_BATCH * SAMPLE_SIZE];
uint8_t  inPkt = 0;           // samples currently in pkt
uint32_t nextSample = 0;

static int16_t clamp16(float x) {
  if (x > 32767.0f) return 32767;
  if (x < -32768.0f) return -32768;
  return (int16_t)lroundf(x);
}

void onConnect(uint16_t h) {
  connHandle = h;
  inPkt = 0;
  digitalWrite(LED_BLUE, LOW);   // LEDs are active-low
}

void onDisconnect(uint16_t h, uint8_t reason) {
  (void)h; (void)reason;
  connHandle = BLE_CONN_HANDLE_INVALID;
  digitalWrite(LED_BLUE, HIGH);
}

uint8_t batchSize() {
  if (connHandle == BLE_CONN_HANDLE_INVALID) return 1;
  BLEConnection* c = Bluefruit.Connection(connHandle);
  if (!c) return 1;
  int fit = ((int)c->getMtu() - 3 - 2) / SAMPLE_SIZE;   // ATT header 3, seq 2
  if (fit < 1) fit = 1;
  if (fit > MAX_BATCH) fit = MAX_BATCH;
  return (uint8_t)fit;
}

void setup() {
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);

#ifdef PIN_LSM6DS3TR_C_POWER
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);   // IMU power switch on the Sense
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(10);
#endif

  imu.settings.accelRange      = 16;    // ±16 g: Olympic lifts and drops spike hard
  imu.settings.accelSampleRate = 416;
  imu.settings.accelBandWidth  = 100;
  imu.settings.gyroRange       = 2000;  // ±2000 dps
  imu.settings.gyroSampleRate  = 416;
  if (imu.begin() != 0) {
    while (true) { digitalWrite(LED_RED, !digitalRead(LED_RED)); delay(150); }  // fast red blink = IMU not found
  }

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);   // allows MTU up to 247
  Bluefruit.begin();
  Bluefruit.setName("BarVBT");
  Bluefruit.setTxPower(4);
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);
  Bluefruit.Periph.setConnInterval(6, 12);   // 7.5–15 ms so 100 Hz data keeps up

  vbtService.begin();
  imuChar.setProperties(CHR_PROPS_NOTIFY);
  imuChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  imuChar.setMaxLen(sizeof(pkt));
  imuChar.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(vbtService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  nextSample = micros();
}

void loop() {
  if ((int32_t)(micros() - nextSample) < 0) return;
  nextSample += SAMPLE_US;

  int16_t s[6] = {
    clamp16(imu.readFloatAccelX() * 1000.0f),   // g → mg
    clamp16(imu.readFloatAccelY() * 1000.0f),
    clamp16(imu.readFloatAccelZ() * 1000.0f),
    clamp16(imu.readFloatGyroX() * 10.0f),      // dps → 0.1 dps
    clamp16(imu.readFloatGyroY() * 10.0f),
    clamp16(imu.readFloatGyroZ() * 10.0f),
  };

  if (connHandle == BLE_CONN_HANDLE_INVALID || !imuChar.notifyEnabled()) {
    seq++;          // keep counting so the app can see gaps
    inPkt = 0;
    return;
  }

  if (inPkt == 0) memcpy(pkt, &seq, 2);
  memcpy(pkt + 2 + inPkt * SAMPLE_SIZE, s, SAMPLE_SIZE);
  inPkt++;
  seq++;

  if (inPkt >= batchSize()) {
    imuChar.notify(pkt, 2 + inPkt * SAMPLE_SIZE);
    inPkt = 0;
  }
}
