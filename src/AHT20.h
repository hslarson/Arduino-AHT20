#ifndef AHT20_H
#define AHT20_H

#include <Arduino.h>
#include <Wire.h>

#define AHT20_DEFAULT_ADDRESS 0x38

// Reset Command
#define AHT20_CMD_RESET 0xBA

// Initialization sequence 0xBE0800
#define AHT20_CMD_INITIALIZE_2 0xBE
#define AHT20_CMD_INITIALIZE_1 0x08
#define AHT20_CMD_INITIALIZE_0 0x00

// Measurment sequence 0xAC3300
#define AHT20_CMD_MEASURE_2 0xAC
#define AHT20_CMD_MEASURE_1 0x33
#define AHT20_CMD_MEASURE_0 0x00

// Class to handle cyclic redundancy checking
struct crc8 {
  uint8_t crc = 0xFF;

  void reset() { crc = 0xFF; }
  void update(uint8_t data) {
    crc = crc ^ data;
    for (int i = 0; i < 8; i++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }
};

class AHT20 {
 private:
  // I2C variables
  TwoWire* _Wire;
  uint8_t _deviceAddress;
  crc8 crc;

  // Measurement variables
  uint32_t _raw_temperature;
  uint32_t _raw_humidity;

  uint32_t _read_delay;
  uint32_t _last_read;
  bool _measurementStarted = false;

  // Status helpers
  uint8_t getStatus();  // Returns the status byte
  bool isCalibrated();  // Returns true if the cal bit is set, false otherwise
  bool isBusy();        // Returns true if the busy bit is set, false otherwise

  // Measurement helper functions
  bool initialize();          // Initialize for taking measurement
  bool triggerMeasurement();  // Trigger the AHT20 to take a measurement
  bool readData();  // Read and parse the 6 bytes of data into raw humidity and temp
  bool softReset();  // Restart the sensor system without turning power off and on

 public:
  explicit AHT20(const uint8_t deviceAddress = AHT20_DEFAULT_ADDRESS);  // Sets the address of the device

  // Sensor status/control functions
  bool begin(TwoWire* wire = &Wire, uint32_t read_delay_ms = 2000);  // Reset and calibrate the sensor
  bool isConnected(uint32_t timeout = 100);  // Checks if the AHT20 is connected to the I2C bus
  bool available();           // Returns true if new data is available

  // Data collection functions
  bool getReading(float* temperature, float* humidity, bool force_new = true);
  float getTemperature(bool force_new = true);
  float getHumidity(bool force_new = true);
};

#endif
