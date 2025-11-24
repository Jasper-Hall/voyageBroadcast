#include <bluefruit.h>
#include <Wire.h>

// --- Sensor Libraries ---
#include "DFRobot_BloodOxygen_S.h"
#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <PDM.h>

// --- BLE Definitions ---
// Custom UUIDs (Changed to bypass browser cache)
const uint8_t SERVICE_UUID[] = {0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33};
const uint8_t CHAR_UUID[]    = {0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44};

BLEService        sensorService(SERVICE_UUID);
BLECharacteristic sensorChar(CHAR_UUID);

// --- Data Packet (Motion Sickness Detection Optimized - 18 bytes) ---
// Focused on sensors critical for motion sickness detection
struct __attribute__((packed)) Packet {
  float pressure_hpa; // 4 bytes - Inner ear pressure, altitude effects
  uint8_t heart_rate; // 1 byte - Autonomic response (increases with nausea)
  uint8_t spo2;       // 1 byte - Stress/nausea indicator
  int8_t accel_x;     // 1 byte - Head movement (CRITICAL for vestibular-visual mismatch)
  int8_t accel_y;     // 1 byte
  int8_t accel_z;     // 1 byte
  int8_t gyro_x;      // 1 byte - Rotational movement (CRITICAL)
  int8_t gyro_y;      // 1 byte
  int8_t gyro_z;      // 1 byte
  uint16_t timestamp; // 2 bytes - Packet counter for timing analysis (0-65535)
  // Removed: temp, humidity, skin_temp, light, mic (not relevant for MS detection)
  uint32_t reserved;  // 4 bytes - Future use (could add temp or other data later)
}; // Total: 18 bytes (well under 20-byte MTU limit!)

Packet dataPacket;

// --- Sensor Objects ---
#define I2C_ADDRESS    0x57
DFRobot_BloodOxygen_S_I2C MAX30102(&Wire ,I2C_ADDRESS);

Adafruit_APDS9960 apds9960; 
Adafruit_BMP280 bmp280;     
Adafruit_LIS3MDL lis3mdl;   
Adafruit_LSM6DS3TRC lsm6ds3trc; 
Adafruit_LSM6DS33 lsm6ds33;
Adafruit_SHT31 sht30;       

// --- Sensor Variables ---
bool new_rev = true; // Check for new sensor revision
long int accel_array[6];
long int check_array[6]={0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

// --- PDM Mic Variables ---
extern PDMClass PDM;
short sampleBuffer[256];  
volatile int samplesRead; 

void setup() {
  Serial.begin(115200);
  
  // 1. Setup Sensors first
  setupSensors();

  // 2. Setup Bluefruit
  Bluefruit.begin();
  Bluefruit.setName("Feather Sense Data");
  
  // Set connection interval for faster updates (9-16 * 1.25ms = 11.25-20ms)
  Bluefruit.Periph.setConnInterval(9, 16);
  
  // MTU will be negotiated automatically by BLE stack
  // Modern clients (including Web Bluetooth) will request appropriate MTU
  Serial.print("Ready for connection (packet size: ");
  Serial.print(sizeof(Packet));
  Serial.println(" bytes, will negotiate MTU)");
  
  sensorService.begin();

  // Enable both READ and NOTIFY properties
  // READ is needed to trigger MTU exchange in Web Bluetooth
  sensorChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  sensorChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  sensorChar.setFixedLen(sizeof(Packet));
  sensorChar.begin();

  // Debug: Print packet size
  Serial.print("Packet size: "); Serial.print(sizeof(Packet)); Serial.println(" bytes");
  Serial.print("Expected struct: 26 bytes, Actual: "); Serial.println(sizeof(Packet));

  // Set connection callback to increase MTU when client connects
  Bluefruit.Periph.setConnectCallback(ble_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(ble_disconnect_callback);

  startAdvertising();
  Serial.println("BLE Broadcasting started...");
}

void startAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(sensorService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); 
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

// Connection callback
void ble_connect_callback(uint16_t conn_handle) {
  Serial.println("Client connected!");
  Serial.print("Connection handle: "); Serial.println(conn_handle);
  
  // Check negotiated MTU
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  uint16_t mtu = connection->getMtu();
  Serial.print("Negotiated MTU: "); Serial.print(mtu);
  Serial.print(" bytes (need "); Serial.print(sizeof(Packet) + 3);
  Serial.println(" bytes minimum)");
  
  if (mtu < sizeof(Packet) + 3) {
    Serial.println("WARNING: MTU too small! Data may be truncated!");
  } else {
    Serial.println("✓ MTU sufficient for full packet transmission");
  }
}

void ble_disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.print("Client disconnected. Reason: "); Serial.println(reason);
}

void loop() {
  // Use non-blocking delay to keep BLE stack happy
  static unsigned long previousMillis = 0;
  const long interval = 200; // Send data every 200ms

  if (millis() - previousMillis >= interval) {
    previousMillis = millis();

    if (Bluefruit.connected()) {
      // 1. Read Sensors
      readAllSensors();
      
      // 2. Send over BLE
      // Debug: Verify we're sending the right size
      static int send_counter = 0;
      if (send_counter++ % 25 == 0) { // Print every 5 seconds (25 * 200ms)
        Serial.print("Sending packet, size: "); Serial.println(sizeof(dataPacket));
      }
      sensorChar.notify(&dataPacket, sizeof(dataPacket));
      
      // 3. Debug Print (Motion Sickness Sensors Only)
      Serial.print("MS Packet #"); Serial.print(dataPacket.timestamp);
      Serial.print(" | HR: "); Serial.print(dataPacket.heart_rate);
      Serial.print(" bpm | SPO2: "); Serial.print(dataPacket.spo2);
      Serial.print("% | Press: "); Serial.print(dataPacket.pressure_hpa);
      Serial.print(" Pa | Accel: ("); Serial.print(dataPacket.accel_x);
      Serial.print(","); Serial.print(dataPacket.accel_y);
      Serial.print(","); Serial.print(dataPacket.accel_z);
      Serial.print(") | Gyro: ("); Serial.print(dataPacket.gyro_x);
      Serial.print(","); Serial.print(dataPacket.gyro_y);
      Serial.print(","); Serial.print(dataPacket.gyro_z);
      Serial.println(")");
    }
  }
}

// --------------------------------------------------------
// --- Sensor Helper Functions (Ported from your snippet) ---
// --------------------------------------------------------

void setupSensors() {
  Serial.println("Initializing Sensors...");
  
  apds9960.begin();
  apds9960.enableProximity(true);
  apds9960.enableColor(true);

  // Initialize MAX30102
  Serial.print("Initializing MAX30102... ");
  if (!MAX30102.begin()) {
    Serial.println("FAIL");
  } else {
    Serial.println("SUCCESS");
    MAX30102.sensorStartCollect();
  }
  
  bmp280.begin();
  
  lis3mdl.begin_I2C();
  
  // Try to initialize LSM6DS3TRC first (newer revision)
  Serial.print("Attempting to initialize LSM6DS3TRC... ");
  if (lsm6ds3trc.begin_I2C()) {
    Serial.println("SUCCESS - Using LSM6DS3TRC");
    new_rev = true;
    // Test read to verify it's working
    sensors_event_t accel, gyro, temp;
    if (lsm6ds3trc.getEvent(&accel, &gyro, &temp)) {
      Serial.print("Test read - Accel: ["); Serial.print(accel.acceleration.x);
      Serial.print(", "); Serial.print(accel.acceleration.y);
      Serial.print(", "); Serial.print(accel.acceleration.z);
      Serial.print("] Gyro: ["); Serial.print(gyro.gyro.x);
      Serial.print(", "); Serial.print(gyro.gyro.y);
      Serial.print(", "); Serial.print(gyro.gyro.z);
      Serial.println("]");
    }
  } else {
    // Fall back to LSM6DS33
    Serial.print("Failed, trying LSM6DS33... ");
    if (lsm6ds33.begin_I2C()) {
      Serial.println("SUCCESS - Using LSM6DS33");
      new_rev = false;
      // Test read to verify it's working
      sensors_event_t accel, gyro, temp;
      if (lsm6ds33.getEvent(&accel, &gyro, &temp)) {
        Serial.print("Test read - Accel: ["); Serial.print(accel.acceleration.x);
        Serial.print(", "); Serial.print(accel.acceleration.y);
        Serial.print(", "); Serial.print(accel.acceleration.z);
        Serial.print("] Gyro: ["); Serial.print(gyro.gyro.x);
        Serial.print(", "); Serial.print(gyro.gyro.y);
        Serial.print(", "); Serial.print(gyro.gyro.z);
        Serial.println("]");
      }
    } else {
      Serial.println("ERROR: Both IMU sensors failed to initialize!");
    }
  }
  
  sht30.begin();
  
  PDM.onReceive(onPDMdata);
  PDM.begin(1, 16000);
}

void readAllSensors() {
  // Static counter for timestamp field
  static uint16_t packet_counter = 0;
  dataPacket.timestamp = packet_counter++;
  
  // 1. Pressure (BMP280) - Critical for motion sickness detection
  dataPacket.pressure_hpa = bmp280.readPressure();

  // 2. IMU Data (Accelerometer & Gyroscope) - CRITICAL for motion sickness
  sensors_event_t accel, gyro, temp;
  bool imu_read_success = false;
  
  if (new_rev) {
    imu_read_success = lsm6ds3trc.getEvent(&accel, &gyro, &temp);
  } else {
    imu_read_success = lsm6ds33.getEvent(&accel, &gyro, &temp);
  }

  // 3. MAX30102 (SPO2, Heartbeat) - Important for autonomic response detection
  MAX30102.getHeartbeatSPO2();
  
  // Validate and cap SPO2 (reasonable range: 70-100%)
  uint8_t spo2_raw = (uint8_t)MAX30102._sHeartbeatSPO2.SPO2;
  if (spo2_raw < 70 || spo2_raw > 100) {
    dataPacket.spo2 = 0; // Use 0 as invalid marker (dashboard will show "Calibrating")
  } else {
    dataPacket.spo2 = spo2_raw;
  }
  
  // Validate and cap Heart Rate (reasonable range: 40-220 bpm)
  uint8_t hr_raw = (uint8_t)MAX30102._sHeartbeatSPO2.Heartbeat;
  if (hr_raw < 40 || hr_raw > 220) {
    dataPacket.heart_rate = 0; // Use 0 as invalid marker (dashboard will show "Calibrating")
  } else {
    dataPacket.heart_rate = hr_raw;
  }
  
  // Reserved field for future expansion
  dataPacket.reserved = 0;
  
  // Debug: Print raw sensor values every 10 packets (every ~2 seconds at 200ms interval)
  static int debug_counter = 0;
  if (debug_counter++ % 10 == 0) {
    Serial.print("RAW IMU (rev="); Serial.print(new_rev ? "TRC" : "33");
    Serial.print(", success="); Serial.print(imu_read_success);
    Serial.print(") - Accel: [");
    Serial.print(accel.acceleration.x, 3); Serial.print(", ");
    Serial.print(accel.acceleration.y, 3); Serial.print(", ");
    Serial.print(accel.acceleration.z, 3);
    Serial.print("] Gyro: [");
    Serial.print(gyro.gyro.x, 3); Serial.print(", ");
    Serial.print(gyro.gyro.y, 3); Serial.print(", ");
    Serial.print(gyro.gyro.z, 3);
    Serial.println("]");
  }
  
  // --- ACCELEROMETER SCALING ---
  // Standard Gravity = 9.8 m/s^2. 
  // 9.8 * 10 = 98. This fits easily in int8 (-128 to 127).
  // Max range before clipping: +/- 12.7 m/s^2 (~1.3 G)
  float accel_x_scaled = accel.acceleration.x * 10.0;
  float accel_y_scaled = accel.acceleration.y * 10.0;
  float accel_z_scaled = accel.acceleration.z * 10.0;

  dataPacket.accel_x = (int8_t)constrain(accel_x_scaled, -127, 127);
  dataPacket.accel_y = (int8_t)constrain(accel_y_scaled, -127, 127);
  dataPacket.accel_z = (int8_t)constrain(accel_z_scaled, -127, 127);
  
  // --- GYROSCOPE SCALING ---
  // Adafruit Sensor Library returns Radians/Second.
  // Normal head movement is < 5 rad/s. 
  // 5 * 10 = 50. Fits easily in int8.
  float gyro_x_scaled = gyro.gyro.x * 10.0;
  float gyro_y_scaled = gyro.gyro.y * 10.0;
  float gyro_z_scaled = gyro.gyro.z * 10.0;

  dataPacket.gyro_x = (int8_t)constrain(gyro_x_scaled, -127, 127);
  dataPacket.gyro_y = (int8_t)constrain(gyro_y_scaled, -127, 127);
  dataPacket.gyro_z = (int8_t)constrain(gyro_z_scaled, -127, 127);
}

// --- PDM Functions ---
int32_t getPDMwave(int32_t samples) {
  short minwave = 30000;
  short maxwave = -30000;

  // We have to be careful not to block too long here in a BLE loop
  // Simple timeout protection
  unsigned long start = millis();
  
  while (samples > 0) {
    if (millis() - start > 50) break; // Break if taking too long (50ms)

    if (!samplesRead) {
      yield();
      continue;
    }
    for (int i = 0; i < samplesRead; i++) {
      minwave = min(sampleBuffer[i], minwave);
      maxwave = max(sampleBuffer[i], maxwave);
      samples--;
    }
    samplesRead = 0;
  }
  return maxwave - minwave;
}

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}