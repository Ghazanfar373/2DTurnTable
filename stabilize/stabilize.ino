/*
 * IMU stabilization for the Mega turntable. Separate from mpu_freertos_servo.ino.
 * Library: "FreeRTOS" by Phillip Stevens (Arduino_FreeRTOS)
 *
 * Pitch and roll come from the same complementary filter as the working sketch:
 * gyro for the fast motion, accelerometer to stop drift. Yaw is gyro only.
 * A PID loop then drives the two servos to cancel pitch and roll error.
 * The pulse slew limit keeps the worm gear from being stepped.
 *
 * Existing packets are unchanged:
 *   Servo trim, 6 bytes:  AA 01 id us_lo us_hi xor
 *   Servo ack, 6 bytes:   AA 81 id us_lo us_hi xor
 *   Attitude, 15 bytes:   AA 02 pitch roll yaw xor   (little-endian floats, degrees)
 *
 * Stabilizer config, 25 bytes, little-endian:
 *   [0]     0xAA
 *   [1]     0x03
 *   [2]     enable          0 hold trim, 1 level with the PID
 *   [3]     reserved
 *   [4..5]  Kp * 1000       int16
 *   [6..7]  Ki * 1000       int16
 *   [8..9]  Kd * 1000       int16
 *   [10..11] servo1 pitch mix, µs per degree   int16
 *   [12..13] servo2 pitch mix                  int16
 *   [14..15] servo1 roll mix                   int16
 *   [16..17] servo2 roll mix                   int16
 *   [18..19] target pitch, degrees * 100       int16
 *   [20..21] target roll, degrees * 100        int16
 *   [22..23] slew, µs per second               uint16
 *   [24]    xor of bytes 0..23
 *
 * With enable off, AA 01 still sets the servo pulse. The horn slews there
 * instead of jumping. With enable on, AA 01 sets the center the PID holds.
 *
 * Sign: error = target - measured. Positive pitch mix lowers both pulses
 * when the nose is above the target (the measured pitch-down direction).
 * Positive roll on servo 1 and negative roll on servo 2 commands the
 * measured roll-left direction when the right side is low.
 *
 * Stack sizes are in BYTES on AVR.
 */
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <Servo.h>

// ---------------- IMU ----------------
const uint8_t  MPU_ADDR    = 0x68;
const uint16_t CAL_SAMPLES = 500;
#define IMU_PERIOD_MS    20

// ---------------- Servos ----------------
#define SERVO1_PIN       9
#define SERVO2_PIN       10
#define SERVO_CENTER_US  1500
#define SERVO_MIN_US     1000
#define SERVO_MAX_US     2000

// ---------------- Packets ----------------
#define PKT_SYNC       0xAA
#define PKT_LEN        6
#define ATT_PKT_LEN    15
#define STAB_PKT_LEN   25
#define CMD_SET_US     0x01
#define CMD_ATTITUDE   0x02
#define CMD_STAB       0x03
#define CMD_SET_ACK    0x81

#define IMU_STACK_BYTES     2200
#define SERIAL_STACK_BYTES  512

struct StabConfig {
  uint8_t enable;
  float   kp, ki, kd;
  float   pitchUs[2];
  float   rollUs[2];
  float   targetPitch;
  float   targetRoll;
  float   center[2];
  float   slew;           // microseconds per second
};

static SemaphoreHandle_t serialMtx;
static SemaphoreHandle_t cfgMtx;
static StabConfig        cfg = {
  0, 1.0f, 0.0f, 0.05f,
  { 15.0f, 15.0f },
  { 15.0f, -15.0f },
  0.0f, 0.0f,
  { SERVO_CENTER_US, SERVO_CENTER_US },
  120.0f
};

static Servo servo1, servo2;

// PID and slew state, touched only by TaskIMU.
static float    integP = 0, integR = 0;
static float    prevP = 0, prevR = 0;
static float    dFiltP = 0, dFiltR = 0;
static float    outUs[2] = { SERVO_CENTER_US, SERVO_CENTER_US };
static uint16_t writtenUs[2] = { SERVO_CENTER_US, SERVO_CENTER_US };
static uint8_t  wasEnabled = 0;

// ---------------- I2C (TaskIMU only) ----------------
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

// ---------------- Packets and servos ----------------
static int clampUs(int us) {
  if (us < SERVO_MIN_US) return SERVO_MIN_US;
  if (us > SERVO_MAX_US) return SERVO_MAX_US;
  return us;
}

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int roundToInt(float v) {
  return (int)(v + (v >= 0.0f ? 0.5f : -0.5f));
}

static int16_t readI16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t readU16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint8_t xorBytes(const uint8_t *p, uint8_t n) {
  uint8_t x = 0;
  for (uint8_t i = 0; i < n; i++) x ^= p[i];
  return x;
}

static void writeServoUs(uint8_t id, int us) {
  if (id == 1)      servo1.writeMicroseconds(us);
  else if (id == 2) servo2.writeMicroseconds(us);
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

static void sendAttitude(float pitch, float roll, float yaw) {
  uint8_t pkt[ATT_PKT_LEN];
  pkt[0] = PKT_SYNC;
  pkt[1] = CMD_ATTITUDE;
  memcpy(pkt + 2,  &pitch, sizeof(float));
  memcpy(pkt + 6,  &roll,  sizeof(float));
  memcpy(pkt + 10, &yaw,   sizeof(float));
  pkt[ATT_PKT_LEN - 1] = xorBytes(pkt, ATT_PKT_LEN - 1);
  if (xSemaphoreTake(serialMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
    Serial.write(pkt, ATT_PKT_LEN);
    xSemaphoreGive(serialMtx);
  }
}

// Move the horn toward goal, but only transmit steps the TD8135MG will follow.
static void slewServo(uint8_t index, float goal, float maxStep) {
  float err = goal - outUs[index];
  if (fabsf(err) <= maxStep) outUs[index] = goal;
  else outUs[index] += (err > 0.0f ? maxStep : -maxStep);

  int pulse = clampUs(roundToInt(outUs[index]));
  int goalPulse = clampUs(roundToInt(goal));
  if (pulse == (int)writtenUs[index]) return;
  if (abs(pulse - (int)writtenUs[index]) < 8 && pulse != goalPulse) return;

  writtenUs[index] = (uint16_t)pulse;
  writeServoUs(index + 1, pulse);
  sendAck(index + 1, (uint16_t)pulse);
}

static void applyManual(uint8_t id, uint16_t pulse) {
  pulse = (uint16_t)clampUs(pulse);
  if (xSemaphoreTake(cfgMtx, pdMS_TO_TICKS(10)) != pdTRUE) return;
  cfg.center[id - 1] = pulse;
  xSemaphoreGive(cfgMtx);
}

static void applyStab(const uint8_t *pkt) {
  StabConfig next;
  if (xSemaphoreTake(cfgMtx, pdMS_TO_TICKS(10)) != pdTRUE) return;
  next = cfg;
  xSemaphoreGive(cfgMtx);

  next.enable = pkt[2] ? 1 : 0;
  next.kp = clampf(readI16(pkt + 4) / 1000.0f, 0.0f, 5.0f);
  next.ki = clampf(readI16(pkt + 6) / 1000.0f, 0.0f, 2.0f);
  next.kd = clampf(readI16(pkt + 8) / 1000.0f, 0.0f, 2.0f);
  next.pitchUs[0] = clampf(readI16(pkt + 10), -200.0f, 200.0f);
  next.pitchUs[1] = clampf(readI16(pkt + 12), -200.0f, 200.0f);
  next.rollUs[0]  = clampf(readI16(pkt + 14), -200.0f, 200.0f);
  next.rollUs[1]  = clampf(readI16(pkt + 16), -200.0f, 200.0f);
  next.targetPitch = clampf(readI16(pkt + 18) / 100.0f, -45.0f, 45.0f);
  next.targetRoll  = clampf(readI16(pkt + 20) / 100.0f, -45.0f, 45.0f);
  next.slew = clampf(readU16(pkt + 22), 20.0f, 800.0f);

  if (xSemaphoreTake(cfgMtx, pdMS_TO_TICKS(10)) == pdTRUE) {
    // Keep the trim centers last set by AA 01.
    next.center[0] = cfg.center[0];
    next.center[1] = cfg.center[1];
    cfg = next;
    xSemaphoreGive(cfgMtx);
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

  float pitch = 0, roll = 0, yaw = 0;
  uint32_t prevUs = micros();
  uint8_t reportDiv = 0;
  bool failReported = false;
  TickType_t lastWake = xTaskGetTickCount();
  StabConfig local = cfg;

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(IMU_PERIOD_MS));

    uint32_t nowUs = micros();
    float dt = (nowUs - prevUs) * 1e-6f;
    prevUs = nowUs;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.1f) dt = 0.1f;

    int16_t ax, ay, az, gx, gy, gz;
    bool imuOk = readAccelGyro(ax, ay, az, gx, gy, gz);
    if (!imuOk) {
      if (!failReported) {
        failReported = true;
        if (xSemaphoreTake(serialMtx, pdMS_TO_TICKS(20)) == pdTRUE) {
          Serial.println(F("IMU read failed"));
          xSemaphoreGive(serialMtx);
        }
      }
    } else {
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
    }

    if (xSemaphoreTake(cfgMtx, pdMS_TO_TICKS(2)) == pdTRUE) {
      local = cfg;
      xSemaphoreGive(cfgMtx);
    }

    if (local.enable && !wasEnabled) {
      integP = integR = 0;
      dFiltP = dFiltR = 0;
      prevP = pitch;
      prevR = roll;
      outUs[0] = writtenUs[0];
      outUs[1] = writtenUs[1];
    }
    if (!local.enable && wasEnabled) {
      // Leave the horns where they are. The next trim command is what moves them.
      local.center[0] = writtenUs[0];
      local.center[1] = writtenUs[1];
      if (xSemaphoreTake(cfgMtx, pdMS_TO_TICKS(2)) == pdTRUE) {
        cfg.center[0] = writtenUs[0];
        cfg.center[1] = writtenUs[1];
        xSemaphoreGive(cfgMtx);
      }
    }
    wasEnabled = local.enable;

    // A missed sample must not drag a live stabilizer back to the trim center.
    if (local.enable && !imuOk) continue;

    float goal1 = local.center[0];
    float goal2 = local.center[1];
    if (local.enable) {
      float pErr = local.targetPitch - pitch;
      float rErr = local.targetRoll - roll;
      integP = clampf(integP + pErr * dt, -20.0f, 20.0f);
      integR = clampf(integR + rErr * dt, -20.0f, 20.0f);

      // Derivative of the measurement, lightly filtered, so vibration
      // and a target change do not kick the worm gear.
      float dP = (pitch - prevP) / dt;
      float dR = (roll - prevR) / dt;
      prevP = pitch;
      prevR = roll;
      dFiltP = 0.8f * dFiltP + 0.2f * dP;
      dFiltR = 0.8f * dFiltR + 0.2f * dR;

      float uP = local.kp * pErr + local.ki * integP - local.kd * dFiltP;
      float uR = local.kp * rErr + local.ki * integR - local.kd * dFiltR;
      uP = clampf(uP, -30.0f, 30.0f);
      uR = clampf(uR, -30.0f, 30.0f);

      goal1 = local.center[0] + uP * local.pitchUs[0] + uR * local.rollUs[0];
      goal2 = local.center[1] + uP * local.pitchUs[1] + uR * local.rollUs[1];
    }

    float maxStep = local.slew * dt;
    slewServo(0, goal1, maxStep);
    slewServo(1, goal2, maxStep);

    if (!imuOk) continue;
    if (++reportDiv < 5) continue;
    reportDiv = 0;
    sendAttitude(pitch, roll, yaw);
  }
}

void TaskSerial(void *pvParameters) {
  (void)pvParameters;

  uint8_t buf[STAB_PKT_LEN];
  uint8_t n = 0;
  uint8_t need = 0;

  for (;;) {
    while (Serial.available() > 0) {
      uint8_t b = (uint8_t)Serial.read();

      if (n == 0) {
        if (b != PKT_SYNC) continue;
        buf[0] = b;
        n = 1;
        continue;
      }

      if (n == 1) {
        if (b == CMD_SET_US) need = PKT_LEN;
        else if (b == CMD_STAB) need = STAB_PKT_LEN;
        else need = 0;
        if (need == 0) {
          n = (b == PKT_SYNC) ? 1 : 0;
          if (n == 1) buf[0] = PKT_SYNC;
          continue;
        }
        buf[1] = b;
        n = 2;
        continue;
      }

      buf[n++] = b;
      if (n < need) continue;
      n = 0;

      if (xorBytes(buf, need - 1) != buf[need - 1]) {
        if (b == PKT_SYNC) {
          buf[0] = b;
          n = 1;
        }
        continue;
      }

      if (buf[1] == CMD_SET_US) {
        uint8_t id = buf[2];
        if (id == 1 || id == 2) {
          uint16_t pulse = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
          applyManual(id, pulse);
        }
      } else if (buf[1] == CMD_STAB) {
        applyStab(buf);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  (void)xTask;
  Serial.print(F("STACK "));
  Serial.println(pcTaskName);
  for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(25000, true);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.writeMicroseconds(SERVO_CENTER_US);
  servo2.writeMicroseconds(SERVO_CENTER_US);

  serialMtx = xSemaphoreCreateMutex();
  cfgMtx    = xSemaphoreCreateMutex();

  xTaskCreate(TaskIMU,    "IMU",    IMU_STACK_BYTES,    NULL, 3, NULL);
  xTaskCreate(TaskSerial, "Serial", SERIAL_STACK_BYTES, NULL, 2, NULL);
}

void loop() {}
