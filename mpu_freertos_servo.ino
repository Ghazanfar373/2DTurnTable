/*
 * MPU9250 attitude + command-driven servos, Arduino Mega 2560 + FreeRTOS
 * Library: "FreeRTOS" by Phillip Stevens (Arduino_FreeRTOS)
 *
 * Tasks:
 *   TaskIMU    (prio 3) - sensor read + complementary filter, sends attitude at 10 Hz
 *   TaskSerial (prio 2) - reads binary servo commands from Serial
 *   TaskServo  (prio 2) - applies a pulse width (us) to the servo named in the command
 *
 * Every packet starts with 0xAA. The command byte selects the length.
 * Checksum is the XOR of every byte except the last.
 *
 * Servo command (6 bytes, little-endian pulse):
 *   [0] 0xAA          sync
 *   [1] 0x01          CMD_SET_US
 *   [2] servo id      1 or 2
 *   [3] us low byte
 *   [4] us high byte  pulse width in microseconds
 *   [5] checksum
 *
 * Servo ack uses the same 6-byte layout with command 0x81 and the clamped pulse.
 * Values outside SERVO_MIN_US..SERVO_MAX_US are clamped. Commands received during
 * gyro calibration are queued and applied once calibration finishes.
 *
 * Example: servo 1 -> 1500 us
 *   AA 01 01 DC 05 73
 *
 * Attitude packet (15 bytes, little-endian IEEE754 floats, degrees), sent every 100 ms
 * once samples are flowing:
 *   [0]     0xAA
 *   [1]     0x02       CMD_ATTITUDE
 *   [2..5]  pitch
 *   [6..9]  roll
 *   [10..13] yaw
 *   [14]    checksum
 * The GUI parses the 15 bytes starting at 0xAA 0x02.
 *
 * Stack sizes are in BYTES on AVR.
 */
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <Wire.h>
#include <Servo.h>

// ---------------- IMU configuration ----------------
const uint8_t  MPU_ADDR    = 0x68;
const uint16_t CAL_SAMPLES = 500;
#define IMU_PERIOD_MS    20

// ---------------- Servo configuration (adjust to your hardware) ----------------
#define SERVO1_PIN   9
#define SERVO2_PIN   10

#define SERVO_CENTER_US  1500  // position written at startup, before any command
#define SERVO_MIN_US     1000  // safety limits
#define SERVO_MAX_US     2000

// ---------------- Serial command protocol ----------------
#define PKT_SYNC       0xAA
#define PKT_LEN        6
#define ATT_PKT_LEN    15
#define CMD_SET_US     0x01
#define CMD_ATTITUDE   0x02
#define CMD_SET_ACK    0x81

// ---------------- Stacks ----------------
#define IMU_STACK_BYTES     1600
#define SERIAL_STACK_BYTES  320
#define SERVO_STACK_BYTES   320

// ---------------- Shared data ----------------
struct Attitude { float pitch, roll, yaw; };

static SemaphoreHandle_t imuReadySem;    // given when gyro calibration is finished
static SemaphoreHandle_t serialMtx;      // one task at a time writes Serial
static SemaphoreHandle_t servoTargetMtx; // guards requestedUs and requestedDirty
static SemaphoreHandle_t servoWakeSem;   // given when a newer pulse is stored
static uint16_t          requestedUs[2] = { SERVO_CENTER_US, SERVO_CENTER_US };
static uint8_t           requestedDirty = 0;

static Servo servo1, servo2;

// ---------------- I2C helpers (TaskIMU only) ----------------
static int16_t read16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

static void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

static bool mpuReadBytes(uint8_t reg, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, n, (uint8_t)true) != n) return false;
  return Wire.available() >= n;
}

static bool readAccelGyro(int16_t &ax, int16_t &ay, int16_t &az,
                          int16_t &gx, int16_t &gy, int16_t &gz) {
  if (!mpuReadBytes(0x3B, 6)) return false;
  ax = read16();
  ay = read16();
  az = read16();
  if (!mpuReadBytes(0x43, 6)) return false;
  gx = read16();
  gy = read16();
  gz = read16();
  return true;
}

// ---------------- Servo helpers ----------------
static int clampUs(int us) {
  if (us < SERVO_MIN_US) us = SERVO_MIN_US;
  if (us > SERVO_MAX_US) us = SERVO_MAX_US;
  return us;
}

static void writeServoUs(uint8_t id, int us) {
  if (id == 1)      servo1.writeMicroseconds(us);
  else if (id == 2) servo2.writeMicroseconds(us);
}

static uint8_t xorBytes(const uint8_t *p, uint8_t n) {
  uint8_t x = 0;
  for (uint8_t i = 0; i < n; i++) x ^= p[i];
  return x;
}

static void sendAck(uint8_t id, uint16_t us) {
  uint8_t pkt[PKT_LEN];
  pkt[0] = PKT_SYNC;
  pkt[1] = CMD_SET_ACK;
  pkt[2] = id;
  pkt[3] = (uint8_t)(us & 0xFF);
  pkt[4] = (uint8_t)(us >> 8);
  pkt[5] = xorBytes(pkt, PKT_LEN - 1);

  if (xSemaphoreTake(serialMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
    Serial.write(pkt, PKT_LEN);
    xSemaphoreGive(serialMtx);
  }
}

static void sendAttitude(const Attitude &a) {
  uint8_t pkt[ATT_PKT_LEN];
  pkt[0] = PKT_SYNC;
  pkt[1] = CMD_ATTITUDE;
  memcpy(pkt + 2,  &a.pitch, sizeof(float));
  memcpy(pkt + 6,  &a.roll,  sizeof(float));
  memcpy(pkt + 10, &a.yaw,   sizeof(float));
  pkt[ATT_PKT_LEN - 1] = xorBytes(pkt, ATT_PKT_LEN - 1);

  if (xSemaphoreTake(serialMtx, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.write(pkt, ATT_PKT_LEN);
    xSemaphoreGive(serialMtx);
  }
}

// ---------------- Tasks ----------------
void TaskIMU(void *pvParameters) {
  (void)pvParameters;

  mpuWrite(0x6B, 0x00);
  vTaskDelay(pdMS_TO_TICKS(100));

  if (xSemaphoreTake(serialMtx, portMAX_DELAY) == pdTRUE) {
    Serial.println(F("Calibrating gyro, keep sensor STILL..."));
    xSemaphoreGive(serialMtx);
  }
  vTaskDelay(pdMS_TO_TICKS(2000));

  long sumX = 0, sumY = 0, sumZ = 0;
  uint16_t got = 0;
  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    if (mpuReadBytes(0x43, 6)) {
      sumX += read16();
      sumY += read16();
      sumZ += read16();
      got++;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  float biasX = got ? (float)sumX / got : 0;
  float biasY = got ? (float)sumY / got : 0;
  float biasZ = got ? (float)sumZ / got : 0;

  if (xSemaphoreTake(serialMtx, portMAX_DELAY) == pdTRUE) {
    Serial.print(F("Calibration complete, samples: "));
    Serial.println(got);
    if (got == 0) Serial.println(F("IMU not responding"));
    xSemaphoreGive(serialMtx);
  }

  xSemaphoreGive(imuReadySem);   // servos may move now

  float pitch = 0, roll = 0, yaw = 0;
  uint32_t prevUs = micros();
  uint8_t reportDiv = 0;
  bool failReported = false;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(IMU_PERIOD_MS));

    uint32_t nowUs = micros();
    float dt = (nowUs - prevUs) * 1e-6f;
    prevUs = nowUs;

    int16_t ax, ay, az, gx, gy, gz;
    if (!readAccelGyro(ax, ay, az, gx, gy, gz)) {
      if (!failReported) {
        failReported = true;
        if (xSemaphoreTake(serialMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
          Serial.println(F("IMU read failed"));
          xSemaphoreGive(serialMtx);
        }
      }
      continue;
    }
    failReported = false;

    float accX = ax / 16384.0f, accY = ay / 16384.0f, accZ = az / 16384.0f;
    float gyrX = (gx - biasX) / 131.0f;
    float gyrY = (gy - biasY) / 131.0f;
    float gyrZ = (gz - biasZ) / 131.0f;

    float accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    float accRoll  = atan2(accY, accZ) * RAD_TO_DEG;

    pitch = 0.96f * (pitch + gyrY * dt) + 0.04f * accPitch;
    roll  = 0.96f * (roll  + gyrX * dt) + 0.04f * accRoll;
    yaw  += gyrZ * dt;

    if (++reportDiv < 5) continue;
    reportDiv = 0;
    Attitude a = { pitch, roll, yaw };
    sendAttitude(a);
  }
}

void TaskSerial(void *pvParameters) {
  (void)pvParameters;

  uint8_t buf[PKT_LEN];
  uint8_t n = 0;

  for (;;) {
    while (Serial.available() > 0) {
      uint8_t b = (uint8_t)Serial.read();

      if (n == 0) {
        if (b != PKT_SYNC) continue;
        buf[0] = b;
        n = 1;
        continue;
      }

      buf[n++] = b;
      if (n < PKT_LEN) continue;
      n = 0;

      uint8_t xorSum = 0;
      for (uint8_t i = 0; i < PKT_LEN - 1; i++) xorSum ^= buf[i];
      if (xorSum != buf[PKT_LEN - 1]) {
        if (b == PKT_SYNC) {
          buf[0] = b;
          n = 1;
        }
        continue;
      }

      if (buf[1] != CMD_SET_US) continue;

      uint8_t id = buf[2];
      if (id != 1 && id != 2) continue;

      uint16_t pulse = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
      if (xSemaphoreTake(servoTargetMtx, pdMS_TO_TICKS(10)) == pdTRUE) {
        requestedUs[id - 1] = pulse;
        requestedDirty |= (uint8_t)(1u << (id - 1));
        xSemaphoreGive(servoTargetMtx);
        xSemaphoreGive(servoWakeSem);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void TaskServo(void *pvParameters) {
  (void)pvParameters;

  // Servos were already commanded to center in setup().
  // Stay there until gyro calibration finishes.
  xSemaphoreTake(imuReadySem, portMAX_DELAY);

  for (;;) {
    xSemaphoreTake(servoWakeSem, portMAX_DELAY);

    uint16_t localUs[2];
    uint8_t mask;
    if (xSemaphoreTake(servoTargetMtx, pdMS_TO_TICKS(20)) != pdTRUE) continue;
    mask = requestedDirty;
    requestedDirty = 0;
    localUs[0] = requestedUs[0];
    localUs[1] = requestedUs[1];
    xSemaphoreGive(servoTargetMtx);
    if (mask == 0) continue;

    for (uint8_t i = 0; i < 2; i++) {
      if ((mask & (uint8_t)(1u << i)) == 0) continue;
      int pulse = clampUs(localUs[i]);
      writeServoUs(i + 1, pulse);
      sendAck(i + 1, (uint16_t)pulse);
    }
  }
}

// If a task runs out of stack, name it and keep the scheduler running.
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  (void)xTask;
  Serial.print(F("STACK "));
  Serial.println(pcTaskName);
  for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(25000, true);

  // Servos start at center (level) before the scheduler runs
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.writeMicroseconds(SERVO_CENTER_US);
  servo2.writeMicroseconds(SERVO_CENTER_US);

  imuReadySem     = xSemaphoreCreateBinary();
  serialMtx       = xSemaphoreCreateMutex();
  servoTargetMtx  = xSemaphoreCreateMutex();
  servoWakeSem    = xSemaphoreCreateBinary();

  xTaskCreate(TaskIMU,    "IMU",    IMU_STACK_BYTES,    NULL, 3, NULL);
  xTaskCreate(TaskSerial, "Serial", SERIAL_STACK_BYTES, NULL, 2, NULL);
  xTaskCreate(TaskServo,  "Servo",  SERVO_STACK_BYTES,  NULL, 2, NULL);
}

void loop() {}
