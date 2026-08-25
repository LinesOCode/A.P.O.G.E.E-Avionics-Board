#include <Arduino.h>
#include <SPI.h>
#include <hardware/watchdog.h>
#include <hardware/sync.h>
#include <hardware/clocks.h>
#include <pico/multicore.h>
#include <mbed.h>
#include <RadioLib.h>
#include <SD.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

// ========================== PIN ASSIGNMENTS ================================
// All four sensors share this SPI bus. Only one CS line may be low at a time.
// Keep board wiring changes in this block.
const uint8_t SENSOR_SPI_SCK_PIN = 18;
const uint8_t SENSOR_SPI_MOSI_PIN = 19;
const uint8_t SENSOR_SPI_MISO_PIN = 16;
const uint8_t BARO_SENSOR_CS = 5;  // Goertek SPL07-003 barometer
const uint8_t IMU_SENSOR_CS = 6;   // ST LSM6DS3TR accelerometer + gyro
const uint8_t ACCEL_SENSOR_CS = 7; // ST LIS2DH12TR accelerometer
const uint8_t GYRO_SENSOR_CS = 2;  // ST A3G4250DTR gyro
const uint8_t MOSFET_ARM_INPUT_PIN = 12;
const uint8_t DROGUE_PARACHUTE_GATE_PIN = 8;
const uint8_t MAIN_PARACHUTE_GATE_PIN = 9;
const uint8_t FIN_1_POWER_GATE_PIN = 13;
const uint8_t FIN_2_POWER_GATE_PIN = 14;
const uint8_t FIN_3_POWER_GATE_PIN = 15;
const uint8_t FIN_4_POWER_GATE_PIN = 20;
const uint8_t FIN_1_SERVO_SIGNAL_PIN = 21;
const uint8_t FIN_2_SERVO_SIGNAL_PIN = 22;
const uint8_t FIN_3_SERVO_SIGNAL_PIN = 23;
const uint8_t FIN_4_SERVO_SIGNAL_PIN = 24;
const uint8_t MISC_MOSFET_GATE_PIN = 25;
const uint8_t SD_CARD_CS_PIN = 27;       // MicroSD card chip select
const uint8_t LORA_RADIO_CS_PIN = 28;    // HopeRF RFM95W-915S2 NSS
const uint8_t LORA_RADIO_RESET_PIN = 1;
const uint8_t LORA_RADIO_DIO0_PIN = 0;
const uint8_t W25Q16_FLASH_CS_PIN = 3;   // Winbond W25Q16JVSSIQ, 16 Mbit
const float high_altitude_threshold_m = 1000.0f;
const float high_altitude_reset_m = 950.0f;
const uint8_t high_altitude_confirmations = 8;
const uint32_t sensor_health_period_ms = 250;
const uint8_t sensor_recovery_limit = 3;
const float max_fin_correction_deg = 12.0f;
const float maximum_control_angle_deg = 45.0f;
const bool FIN_CONTROL_ENABLED = false; // Explicitly enable only after restrained bench testing.
const float ATTITUDE_ACCEL_CORRECTION = 0.02f;
const uint32_t BAROMETER_SAMPLE_PERIOD_MS = 62;
const uint32_t TELEMETRY_PERIOD_MS = 50;
const uint32_t SD_FLUSH_PERIOD_MS = 1000;
const uint8_t INTER_RP_PACKET_MAGIC = 0xa7;

arduino::MbedSPI sensorSPI(SENSOR_SPI_MISO_PIN, SENSOR_SPI_MOSI_PIN, SENSOR_SPI_SCK_PIN);
SPISettings sensor_spi(8000000, MSBFIRST, SPI_MODE0);
File flightLog;
SX1276 loraRadio = new Module(LORA_RADIO_CS_PIN, LORA_RADIO_DIO0_PIN, LORA_RADIO_RESET_PIN, RADIOLIB_NC);
bool sdCardReady = false;
bool loraReady = false;
bool flashReady = false;
uint32_t flashJedecId = 0;
uint32_t lastSdFlushMs = 0;
uint16_t interRpSequence = 0;
spin_lock_t *telemetryLock;
spin_lock_t *peripheralLock;

struct FlightStatePacket {
  uint8_t magic;
  uint8_t version;
  uint16_t sequence;
  uint32_t timestampMs;
  float altitudeM;
  float velocityMps;
  float rollRad;
  float pitchRad;
  float pressurePa;
  uint8_t sensorHealthMask;
  uint8_t checksum;
};

volatile FlightStatePacket telemetrySnapshot;
volatile bool telemetrySnapshotReady = false;
static void publishRemoteTelemetry(const FlightStatePacket &packet);

static uint8_t packetChecksum(const FlightStatePacket &packet) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t checksum = 0;
  for (size_t index = 0; index < sizeof(packet) - 1; ++index) checksum ^= bytes[index];
  return checksum;
}

static uint32_t readW25q16JedecId() {
  uint8_t id[3];
  uint32_t irqState = spin_lock_blocking(peripheralLock);
  sensorSPI.beginTransaction(sensor_spi);
  digitalWrite(W25Q16_FLASH_CS_PIN, LOW);
  sensorSPI.transfer(0x9f);
  id[0] = sensorSPI.transfer(0);
  id[1] = sensorSPI.transfer(0);
  id[2] = sensorSPI.transfer(0);
  digitalWrite(W25Q16_FLASH_CS_PIN, HIGH);
  sensorSPI.endTransaction();
  spin_unlock(peripheralLock, irqState);
  return (static_cast<uint32_t>(id[0]) << 16) |
         (static_cast<uint32_t>(id[1]) << 8) | id[2];
}

static uint8_t spiRead8(uint8_t chipSelect, uint8_t reg) {
  uint32_t irqState = spin_lock_blocking(peripheralLock);
  sensorSPI.beginTransaction(sensor_spi);
  digitalWrite(chipSelect, LOW);
  sensorSPI.transfer(reg | 0x80);
  uint8_t value = sensorSPI.transfer(0);
  digitalWrite(chipSelect, HIGH);
  sensorSPI.endTransaction();
  spin_unlock(peripheralLock, irqState);
  return value;
}

static void spiWrite8(uint8_t chipSelect, uint8_t reg, uint8_t value) {
  uint32_t irqState = spin_lock_blocking(peripheralLock);
  sensorSPI.beginTransaction(sensor_spi);
  digitalWrite(chipSelect, LOW);
  sensorSPI.transfer(reg & 0x7f);
  sensorSPI.transfer(value);
  digitalWrite(chipSelect, HIGH);
  sensorSPI.endTransaction();
  spin_unlock(peripheralLock, irqState);
}

static void spiRead(uint8_t chipSelect, uint8_t reg, uint8_t *buffer, size_t length) {
  uint32_t irqState = spin_lock_blocking(peripheralLock);
  sensorSPI.beginTransaction(sensor_spi);
  digitalWrite(chipSelect, LOW);
  sensorSPI.transfer(reg | 0x80 | (length > 1 ? 0x40 : 0));
  memset(buffer, 0, length);
  sensorSPI.transfer(buffer, length);
  digitalWrite(chipSelect, HIGH);
  sensorSPI.endTransaction();
  spin_unlock(peripheralLock, irqState);
}

enum SensorId : uint8_t { SENSOR_SPL07, SENSOR_LSM6, SENSOR_LIS2DH, SENSOR_A3G4250 };
uint8_t sensorFailures[4] = {0, 0, 0, 0};
bool sensorHealthy[4] = {false, false, false, false};
uint32_t lastSensorHealthCheck = 0;

class Spl07 {
 public:
  explicit Spl07(uint8_t chipSelect) : chipSelect_(chipSelect) {}

  bool begin() {
    if (spiRead8(chipSelect_, 0x0d) != 0x10) return false;
    uint8_t calibration[18];
    spiRead(chipSelect_, 0x10, calibration, sizeof(calibration));
    c0_ = signExtend((calibration[0] << 4) | (calibration[1] >> 4), 12);
    c1_ = signExtend(((calibration[1] & 0x0f) << 8) | calibration[2], 12);
    c00_ = signExtend((calibration[3] << 12) | (calibration[4] << 4) | (calibration[5] >> 4), 20);
    c10_ = signExtend(((calibration[5] & 0x0f) << 16) | (calibration[6] << 8) | calibration[7], 20);
    c01_ = signExtend((calibration[8] << 8) | calibration[9], 16);
    c11_ = signExtend((calibration[10] << 8) | calibration[11], 16);
    c20_ = signExtend((calibration[12] << 8) | calibration[13], 16);
    c21_ = signExtend((calibration[14] << 8) | calibration[15], 16);
    c30_ = signExtend((calibration[16] << 8) | calibration[17], 16);
    // 16 measurements/s, 8x oversampling for both pressure and temperature.
    spiWrite8(chipSelect_, 0x06, 0x24);
    spiWrite8(chipSelect_, 0x07, 0xa7);
    spiWrite8(chipSelect_, 0x08, 0x07);
    spiWrite8(chipSelect_, 0x09, 0x00);
    return true;
  }

  float pressurePa() {
    uint8_t data[6];
    spiRead(chipSelect_, 0x00, data, sizeof(data));
    int32_t pressure = signExtend((data[0] << 16) | (data[1] << 8) | data[2], 24);
    int32_t temperature = signExtend((data[3] << 16) | (data[4] << 8) | data[5], 24);
    float pressureScale = 7864320.0f; // 8x oversampling scale factor
    float temperatureScale = 7864320.0f;
    float pressureScaled = pressure / pressureScale;
    float temperatureScaled = temperature / temperatureScale;
    float pressureCompensated = c00_ + pressureScaled * (c10_ + pressureScaled * (c20_ + pressureScaled * c30_));
    pressureCompensated += temperatureScaled * c01_ + pressureScaled * temperatureScaled * (c11_ + pressureScaled * c21_);
    return pressureCompensated * 100.0f;
  }

 private:
  static int32_t signExtend(uint32_t value, uint8_t bits) {
    uint32_t sign = 1UL << (bits - 1);
    return (value ^ sign) - sign;
  }
  uint8_t chipSelect_;
  int32_t c0_ = 0, c1_ = 0, c00_ = 0, c10_ = 0;
  int32_t c01_ = 0, c11_ = 0, c20_ = 0, c21_ = 0, c30_ = 0;
};

struct Vector3 { float x; float y; float z; };
struct ImuSample { Vector3 acceleration; Vector3 gyro; };

static ImuSample readLsm6(uint8_t chipSelect) {
  uint8_t data[12];
  spiRead(chipSelect, 0x22, data, sizeof(data));
  const float gyroScale = 8.75e-3f * DEG_TO_RAD;
  const float accelScale = 0.061f * 9.80665e-3f;
  ImuSample sample;
  sample.acceleration = {((int16_t)(data[6] | data[7] << 8)) * accelScale,
                         ((int16_t)(data[8] | data[9] << 8)) * accelScale,
                         ((int16_t)(data[10] | data[11] << 8)) * accelScale};
  sample.gyro = {((int16_t)(data[0] | data[1] << 8)) * gyroScale,
                 ((int16_t)(data[2] | data[3] << 8)) * gyroScale,
                 ((int16_t)(data[4] | data[5] << 8)) * gyroScale};
  return sample;
}

static Vector3 readLis2dh(uint8_t chipSelect) {
  uint8_t data[6];
  spiRead(chipSelect, 0x28, data, sizeof(data));
  const float scale = 0.001f * 9.80665f; // +/-2 g, high-resolution mode
  return {((int16_t)(data[0] | data[1] << 8) >> 4) * scale,
          ((int16_t)(data[2] | data[3] << 8) >> 4) * scale,
          ((int16_t)(data[4] | data[5] << 8) >> 4) * scale};
}

static Vector3 readA3g4250(uint8_t chipSelect) {
  uint8_t data[6];
  spiRead(chipSelect, 0x28, data, sizeof(data));
  const float scale = 8.75e-3f * DEG_TO_RAD;
  return {((int16_t)(data[0] | data[1] << 8)) * scale,
          ((int16_t)(data[2] | data[3] << 8)) * scale,
          ((int16_t)(data[4] | data[5] << 8)) * scale};
}

class AltitudeKalman {
 public:
  void reset(float altitude) {
    altitude_ = altitude;
    velocity_ = 0.0f;
    p00_ = 1.0f;
    p01_ = 0.0f;
    p11_ = 1.0f;
  }
  void predict(float acceleration, float dt) {
    if (!isfinite(acceleration) || !isfinite(dt) || dt <= 0.0f || dt > 0.1f) return;
    altitude_ += velocity_ * dt + 0.5f * acceleration * dt * dt;
    velocity_ += acceleration * dt;
    float p00 = p00_ + dt * (2.0f * p01_ + dt * p11_) + processAcceleration_ * dt * dt * dt * dt / 4.0f;
    float p01 = p01_ + dt * p11_ + processAcceleration_ * dt * dt * dt / 2.0f;
    float p11 = p11_ + processAcceleration_ * dt * dt;
    p00_ = p00;
    p01_ = p01;
    p11_ = p11;
    if (!isfinite(altitude_) || !isfinite(velocity_) || !isfinite(p00_) ||
        !isfinite(p01_) || !isfinite(p11_) || p00_ < 0.0f || p11_ < 0.0f) {
      reset(0.0f);
    }
  }
  void correct(float measuredAltitude) {
    if (!isfinite(measuredAltitude) || !isfinite(altitude_) || !isfinite(p00_) ||
        !isfinite(p01_) || !isfinite(p11_)) return;
    float innovation = measuredAltitude - altitude_;
    float covariance01 = p01_;
    float innovationVariance = p00_ + measurementVariance_;
    if (innovationVariance <= 0.0f || !isfinite(innovationVariance)) return;
    float gainAltitude = p00_ / innovationVariance;
    float gainVelocity = p01_ / innovationVariance;
    altitude_ += gainAltitude * innovation;
    velocity_ += gainVelocity * innovation;
    p00_ = (1.0f - gainAltitude) * p00_;
    p01_ = (1.0f - gainAltitude) * covariance01;
    p11_ = p11_ - gainVelocity * covariance01;
    if (!isfinite(altitude_) || !isfinite(velocity_) || p00_ < 0.0f || p11_ < 0.0f) reset(0.0f);
  }
  void update(float acceleration, float measuredAltitude, float dt) {
    predict(acceleration, dt);
    correct(measuredAltitude);
  }
  float altitude() const { return altitude_; }
  float velocity() const { return velocity_; }

 private:
  float altitude_ = 0.0f, velocity_ = 0.0f;
  float p00_ = 1.0f, p01_ = 0.0f, p11_ = 1.0f;
  const float processAcceleration_ = 0.8f;
  const float measurementVariance_ = 2.5f;
};

class FinController {
 public:
  FinController()
      : fin1Servo(digitalPinToPinName(FIN_1_SERVO_SIGNAL_PIN)),
        fin2Servo(digitalPinToPinName(FIN_2_SERVO_SIGNAL_PIN)),
        fin3Servo(digitalPinToPinName(FIN_3_SERVO_SIGNAL_PIN)),
        fin4Servo(digitalPinToPinName(FIN_4_SERVO_SIGNAL_PIN)) {}

  void begin() {
    for (uint8_t index = 0; index < 4; ++index) {
      servo(index).period_ms(20);
      servo(index).pulsewidth_us(1500);
    }
  }

  void disable() {
    setPower(false);
    setNeutral();
  }

  void correct(float roll, float pitch, bool armed) {
    if (!FIN_CONTROL_ENABLED || !armed || !isfinite(roll) || !isfinite(pitch) ||
        fabsf(roll) > maximum_control_angle_deg * DEG_TO_RAD ||
        fabsf(pitch) > maximum_control_angle_deg * DEG_TO_RAD) {
      disable();
      return;
    }
    setPower(true);
    float rollCorrection = constrain(-roll * RAD_TO_DEG, -max_fin_correction_deg, max_fin_correction_deg);
    float pitchCorrection = constrain(-pitch * RAD_TO_DEG, -max_fin_correction_deg, max_fin_correction_deg);
    writeAngle(0, pitchCorrection + rollCorrection);
    writeAngle(1, pitchCorrection - rollCorrection);
    writeAngle(2, -pitchCorrection - rollCorrection);
    writeAngle(3, -pitchCorrection + rollCorrection);
  }

 private:
  void setPower(bool enabled) {
    digitalWrite(FIN_1_POWER_GATE_PIN, enabled ? HIGH : LOW);
    digitalWrite(FIN_2_POWER_GATE_PIN, enabled ? HIGH : LOW);
    digitalWrite(FIN_3_POWER_GATE_PIN, enabled ? HIGH : LOW);
    digitalWrite(FIN_4_POWER_GATE_PIN, enabled ? HIGH : LOW);
  }

  void setNeutral() {
    for (uint8_t index = 0; index < 4; ++index) servo(index).pulsewidth_us(1500);
  }

  void writeAngle(uint8_t index, float correction) {
    correction = constrain(correction, -max_fin_correction_deg, max_fin_correction_deg);
    servo(index).pulsewidth_us(static_cast<int>(1500.0f + correction * (400.0f / 90.0f)));
  }

  mbed::PwmOut &servo(uint8_t index) {
    switch (index) {
      case 0: return fin1Servo;
      case 1: return fin2Servo;
      case 2: return fin3Servo;
      default: return fin4Servo;
    }
  }

  mbed::PwmOut fin1Servo;
  mbed::PwmOut fin2Servo;
  mbed::PwmOut fin3Servo;
  mbed::PwmOut fin4Servo;
};

extern Spl07 spl07;

static bool configureLsm6() {
  if (spiRead8(IMU_SENSOR_CS, 0x0f) != 0x6a) return false;
  spiWrite8(IMU_SENSOR_CS, 0x10, 0x70); // 833 Hz accelerometer, +/-2 g
  spiWrite8(IMU_SENSOR_CS, 0x11, 0x70); // 833 Hz gyro, 245 dps
  return true;
}

static bool configureLis2dh() {
  if (spiRead8(ACCEL_SENSOR_CS, 0x0f) != 0x33) return false;
  spiWrite8(ACCEL_SENSOR_CS, 0x20, 0x97); // 400 Hz, all axes
  spiWrite8(ACCEL_SENSOR_CS, 0x23, 0x08); // high-resolution, +/-2 g
  return true;
}

static bool configureA3g4250() {
  if (spiRead8(GYRO_SENSOR_CS, 0x0f) != 0xd3) return false;
  spiWrite8(GYRO_SENSOR_CS, 0x20, 0x8f); // 800 Hz, normal mode, all axes
  spiWrite8(GYRO_SENSOR_CS, 0x23, 0x00);
  return true;
}

static bool recoverSensor(SensorId sensor) {
  bool recovered = false;
  switch (sensor) {
    case SENSOR_SPL07: recovered = spl07.begin(); break;
    case SENSOR_LSM6: recovered = configureLsm6(); break;
    case SENSOR_LIS2DH: recovered = configureLis2dh(); break;
    case SENSOR_A3G4250: recovered = configureA3g4250(); break;
  }
  sensorFailures[sensor] = recovered ? 0 : sensorFailures[sensor];
  return recovered;
}

static void sensorHealthCheck(uint32_t now) {
  if (now - lastSensorHealthCheck < sensor_health_period_ms) return;
  lastSensorHealthCheck = now;
  const bool healthy[4] = {
      spiRead8(BARO_SENSOR_CS, 0x0d) == 0x10,
      spiRead8(IMU_SENSOR_CS, 0x0f) == 0x6a,
      spiRead8(ACCEL_SENSOR_CS, 0x0f) == 0x33,
      spiRead8(GYRO_SENSOR_CS, 0x0f) == 0xd3};
  for (uint8_t index = 0; index < 4; ++index) {
    if (healthy[index]) {
      sensorFailures[index] = 0;
      sensorHealthy[index] = true;
    } else if (++sensorFailures[index] >= sensor_recovery_limit) {
      sensorHealthy[index] = recoverSensor(static_cast<SensorId>(index));
    } else {
      sensorHealthy[index] = false;
    }
  }
}

Spl07 spl07(BARO_SENSOR_CS);
AltitudeKalman altitudeFilter;
FinController finController;
float rollEstimate = 0.0f;
float pitchEstimate = 0.0f;
uint32_t lastUpdateMicros;
uint32_t lastBarometerSampleMs = 0;
uint32_t lastTelemetryMs = 0;
float referencePressurePa;
float latestPressurePa = NAN;
uint8_t highAltitudeSamples = 0;
bool highAltitudeMosfetsActive = false;

static void publishRemoteTelemetry(const FlightStatePacket &packet);

static void sendFlightState(uint32_t nowMs, float pressure) {
  FlightStatePacket packet = {INTER_RP_PACKET_MAGIC, 1, interRpSequence++, nowMs,
                              altitudeFilter.altitude(), altitudeFilter.velocity(),
                              rollEstimate, pitchEstimate, pressure, 0, 0};
  if (sensorHealthy[SENSOR_SPL07]) packet.sensorHealthMask |= 1;
  if (sensorHealthy[SENSOR_LSM6]) packet.sensorHealthMask |= 2;
  if (sensorHealthy[SENSOR_LIS2DH]) packet.sensorHealthMask |= 4;
  if (sensorHealthy[SENSOR_A3G4250]) packet.sensorHealthMask |= 8;
  packet.checksum = packetChecksum(packet);
  uint32_t irqState = spin_lock_blocking(telemetryLock);
  memcpy(const_cast<FlightStatePacket *>(reinterpret_cast<volatile const FlightStatePacket *>(&telemetrySnapshot)), &packet, sizeof(packet));
  telemetrySnapshotReady = true;
  spin_unlock(telemetryLock, irqState);
}

static void initializeStorageAndTelemetry() {
  pinMode(SD_CARD_CS_PIN, OUTPUT);
  pinMode(LORA_RADIO_CS_PIN, OUTPUT);
  pinMode(W25Q16_FLASH_CS_PIN, OUTPUT);
  digitalWrite(SD_CARD_CS_PIN, HIGH);
  digitalWrite(LORA_RADIO_CS_PIN, HIGH);
  digitalWrite(W25Q16_FLASH_CS_PIN, HIGH);

  flashJedecId = readW25q16JedecId();
  flashReady = flashJedecId == 0xef4015UL;
  if (flashReady) {
    Serial.println("W25Q16 detected: 16 Mbit external SPI/QSPI-capable flash");
  } else {
    Serial.println("W25Q16 not detected; RP2040 onboard XIP flash is unchanged");
  }

  uint32_t irqState = spin_lock_blocking(peripheralLock);
  sdCardReady = SD.begin(SD_CARD_CS_PIN);
  if (sdCardReady) {
    flightLog = SD.open("flight.csv", FILE_WRITE);
    sdCardReady = static_cast<bool>(flightLog);
    if (sdCardReady && flightLog.size() == 0) {
      flightLog.println("time_ms,altitude_m,velocity_mps,roll_rad,pitch_rad,pressure_pa,flash_jedec");
    }
  }
  spin_unlock(peripheralLock, irqState);
  if (!sdCardReady) Serial.println("MicroSD logging unavailable");

  irqState = spin_lock_blocking(peripheralLock);
  int16_t radioState = loraRadio.begin(915.0, 125.0, 9, 5, 0x12, 17, 8, 0);
  spin_unlock(peripheralLock, irqState);
  loraReady = radioState == RADIOLIB_ERR_NONE;
  if (!loraReady) Serial.println("RFM95W telemetry unavailable");
}

static void telemetryCore1() {
  initializeStorageAndTelemetry();
  for (;;) {
    watchdog_update();
    FlightStatePacket packet;
    bool packetAvailable = false;
    uint32_t irqState = spin_lock_blocking(telemetryLock);
    if (telemetrySnapshotReady) {
      memcpy(&packet, const_cast<const FlightStatePacket *>(reinterpret_cast<volatile const FlightStatePacket *>(&telemetrySnapshot)), sizeof(packet));
      telemetrySnapshotReady = false;
      packetAvailable = true;
    }
    spin_unlock(telemetryLock, irqState);
    if (packetAvailable) publishRemoteTelemetry(packet);
    delay(1);
  }
}

static bool validVector(const Vector3 &value) {
  return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

static float vectorMagnitude(const Vector3 &value) {
  return sqrtf(value.x * value.x + value.y * value.y + value.z * value.z);
}

static bool accelerometerIsUsable(const Vector3 &value) {
  float magnitude = vectorMagnitude(value);
  return validVector(value) && magnitude > 4.0f && magnitude < 15.0f;
}

static Vector3 averageVector(const Vector3 &first, const Vector3 &second) {
  return {(first.x + second.x) * 0.5f,
          (first.y + second.y) * 0.5f,
          (first.z + second.z) * 0.5f};
}

static void updateHighAltitudeOutputs(float altitude, bool pressureConfirmed) {
  if (digitalRead(MOSFET_ARM_INPUT_PIN) == LOW) {
    highAltitudeMosfetsActive = false;
    highAltitudeSamples = 0;
    return;
  }
  if (highAltitudeMosfetsActive) {
    if (altitude < high_altitude_reset_m) {
      highAltitudeMosfetsActive = false;
      highAltitudeSamples = 0;
    }
    return;
  }
  if (pressureConfirmed && altitude >= high_altitude_threshold_m) {
    if (highAltitudeSamples < high_altitude_confirmations) ++highAltitudeSamples;
    if (highAltitudeSamples >= high_altitude_confirmations) {
      highAltitudeMosfetsActive = true;
      Serial.println("HIGH_ALTITUDE_MOSFETS_ACTIVE");
    }
  } else {
    highAltitudeSamples = 0;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BARO_SENSOR_CS, OUTPUT);
  pinMode(IMU_SENSOR_CS, OUTPUT);
  pinMode(ACCEL_SENSOR_CS, OUTPUT);
  pinMode(GYRO_SENSOR_CS, OUTPUT);
  pinMode(MOSFET_ARM_INPUT_PIN, INPUT_PULLDOWN);
  pinMode(DROGUE_PARACHUTE_GATE_PIN, OUTPUT);
  pinMode(MAIN_PARACHUTE_GATE_PIN, OUTPUT);
  pinMode(FIN_1_POWER_GATE_PIN, OUTPUT);
  pinMode(FIN_2_POWER_GATE_PIN, OUTPUT);
  pinMode(FIN_3_POWER_GATE_PIN, OUTPUT);
  pinMode(FIN_4_POWER_GATE_PIN, OUTPUT);
  pinMode(MISC_MOSFET_GATE_PIN, OUTPUT);
  digitalWrite(BARO_SENSOR_CS, HIGH);
  digitalWrite(IMU_SENSOR_CS, HIGH);
  digitalWrite(ACCEL_SENSOR_CS, HIGH);
  digitalWrite(GYRO_SENSOR_CS, HIGH);
  digitalWrite(DROGUE_PARACHUTE_GATE_PIN, LOW);
  digitalWrite(MAIN_PARACHUTE_GATE_PIN, LOW);
  digitalWrite(FIN_1_POWER_GATE_PIN, LOW);
  digitalWrite(FIN_2_POWER_GATE_PIN, LOW);
  digitalWrite(FIN_3_POWER_GATE_PIN, LOW);
  digitalWrite(FIN_4_POWER_GATE_PIN, LOW);
  digitalWrite(MISC_MOSFET_GATE_PIN, LOW);
  finController.begin();
  sensorSPI.begin();
  telemetryLock = spin_lock_init(spin_lock_claim_unused(true));
  peripheralLock = spin_lock_init(spin_lock_claim_unused(true));

  if (!spl07.begin() || !configureLsm6() || !configureLis2dh() || !configureA3g4250()) {
    Serial.println("Sensor identification failed");
    while (true) delay(1000);
  }
  sensorHealthy[SENSOR_SPL07] = true;
  sensorHealthy[SENSOR_LSM6] = true;
  sensorHealthy[SENSOR_LIS2DH] = true;
  sensorHealthy[SENSOR_A3G4250] = true;
  watchdog_enable(2000, true);
  referencePressurePa = spl07.pressurePa();
  latestPressurePa = referencePressurePa;
  if (!isfinite(referencePressurePa) || referencePressurePa < 30000.0f || referencePressurePa > 120000.0f) {
    Serial.println("Invalid barometer reference");
    while (true) delay(1000);
  }
  altitudeFilter.reset(0.0f);
  rollEstimate = 0.0f;
  pitchEstimate = 0.0f;
  lastUpdateMicros = micros();
  multicore_launch_core1(telemetryCore1);
}

void loop() {
  watchdog_update();
  uint32_t now = micros();
  float dt = (now - lastUpdateMicros) * 1.0e-6f;
  lastUpdateMicros = now;
  if (dt <= 0.0f || dt > 0.1f) {
    finController.disable();
    return;
  }
  uint32_t nowMillis = millis();
  sensorHealthCheck(nowMillis);

  Vector3 lisAcceleration = readLis2dh(ACCEL_SENSOR_CS);
  ImuSample lsm6 = readLsm6(IMU_SENSOR_CS);
  bool lisUsable = accelerometerIsUsable(lisAcceleration);
  bool lsmUsable = accelerometerIsUsable(lsm6.acceleration);
  Vector3 acceleration = lisUsable ? lisAcceleration : lsm6.acceleration;
  if (lisUsable && lsmUsable) {
    float accelerationDisagreement = vectorMagnitude({lisAcceleration.x - lsm6.acceleration.x,
                                                       lisAcceleration.y - lsm6.acceleration.y,
                                                       lisAcceleration.z - lsm6.acceleration.z});
    acceleration = accelerationDisagreement < 3.0f
                       ? averageVector(lisAcceleration, lsm6.acceleration)
                       : lisAcceleration;
  }
  if (!lisUsable && !lsmUsable) {
    acceleration = {0.0f, 0.0f, 9.80665f};
  }
  Vector3 externalGyro = readA3g4250(GYRO_SENSOR_CS);
  bool lsmGyroUsable = validVector(lsm6.gyro) && vectorMagnitude(lsm6.gyro) < 10.0f;
  bool externalGyroUsable = validVector(externalGyro) && vectorMagnitude(externalGyro) < 10.0f;
  Vector3 gyro = lsmGyroUsable && externalGyroUsable
                     ? averageVector(lsm6.gyro, externalGyro)
                     : (lsmGyroUsable ? lsm6.gyro : (externalGyroUsable ? externalGyro : Vector3{0.0f, 0.0f, 0.0f}));
  float measuredRoll = atan2f(acceleration.y, acceleration.z);
  float measuredPitch = atan2f(-acceleration.x, sqrtf(acceleration.y * acceleration.y + acceleration.z * acceleration.z));
  if (!isfinite(measuredRoll) || !isfinite(measuredPitch) || !isfinite(gyro.x) || !isfinite(gyro.y)) {
    finController.disable();
    return;
  }
  rollEstimate += gyro.x * dt;
  pitchEstimate += gyro.y * dt;
  rollEstimate += ATTITUDE_ACCEL_CORRECTION * (measuredRoll - rollEstimate);
  pitchEstimate += ATTITUDE_ACCEL_CORRECTION * (measuredPitch - pitchEstimate);
  if (!isfinite(rollEstimate) || !isfinite(pitchEstimate)) {
    rollEstimate = measuredRoll;
    pitchEstimate = measuredPitch;
  }
  float roll = rollEstimate;
  float pitch = pitchEstimate;
  bool motionSensorsHealthy = sensorHealthy[SENSOR_LSM6] &&
                              sensorHealthy[SENSOR_LIS2DH] &&
                              sensorHealthy[SENSOR_A3G4250];
  bool currentMotionSampleValid = lisUsable && lsmUsable &&
                                  lsmGyroUsable && externalGyroUsable;
  bool controlsArmed = FIN_CONTROL_ENABLED && motionSensorsHealthy &&
                       currentMotionSampleValid &&
                       digitalRead(MOSFET_ARM_INPUT_PIN) == HIGH;
  finController.correct(roll, pitch, controlsArmed);
  float sinRoll = sinf(roll);
  float cosRoll = cosf(roll);
  float sinPitch = sinf(pitch);
  float cosPitch = cosf(pitch);
  float verticalAcceleration = acceleration.x * sinPitch + acceleration.y * sinRoll * cosPitch + acceleration.z * cosRoll * cosPitch - 9.80665f;
  altitudeFilter.predict(verticalAcceleration, dt);
  bool pressureConfirmed = false;
  if (nowMillis - lastBarometerSampleMs >= BAROMETER_SAMPLE_PERIOD_MS) {
    lastBarometerSampleMs = nowMillis;
    float pressure = spl07.pressurePa();
    latestPressurePa = pressure;
    bool pressureUsable = isfinite(pressure) && pressure > 30000.0f && pressure < 120000.0f;
    if (pressureUsable) {
      float pressureAltitude = 44330.0f * (1.0f - powf(pressure / referencePressurePa, 0.19029495f));
      if (isfinite(pressureAltitude) && fabsf(pressureAltitude - altitudeFilter.altitude()) < 150.0f) {
        altitudeFilter.correct(pressureAltitude);
        pressureConfirmed = true;
      }
    }
  }
  updateHighAltitudeOutputs(altitudeFilter.altitude(), pressureConfirmed);
  sendFlightState(nowMillis, latestPressurePa);
  if (nowMillis - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = nowMillis;
    Serial.print(altitudeFilter.altitude(), 3);
    Serial.print(',');
    Serial.println(altitudeFilter.velocity(), 3);
  }
}
static void publishRemoteTelemetry(const FlightStatePacket &packet) {
  char record[160];
  int recordLength = snprintf(record, sizeof(record), "%lu,%.3f,%.3f,%.5f,%.5f,%.2f,%02X",
                              static_cast<unsigned long>(packet.timestampMs), packet.altitudeM,
                              packet.velocityMps, packet.rollRad, packet.pitchRad,
                              packet.pressurePa, packet.sensorHealthMask);
  if (recordLength <= 0 || recordLength >= static_cast<int>(sizeof(record))) return;
  if (sdCardReady) {
    uint32_t irqState = spin_lock_blocking(peripheralLock);
    flightLog.println(record);
    if (packet.timestampMs - lastSdFlushMs >= SD_FLUSH_PERIOD_MS) {
      flightLog.flush();
      lastSdFlushMs = packet.timestampMs;
    }
    spin_unlock(peripheralLock, irqState);
  }
  static uint32_t lastRadioTransmissionMs = 0;
  if (loraReady && packet.timestampMs - lastRadioTransmissionMs >= TELEMETRY_PERIOD_MS) {
    uint32_t irqState = spin_lock_blocking(peripheralLock);
    loraRadio.transmit(record);
    spin_unlock(peripheralLock, irqState);
    lastRadioTransmissionMs = packet.timestampMs;
  }
}
