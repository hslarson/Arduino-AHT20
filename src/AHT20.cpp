#include "AHT20.h"

// Constructor
AHT20::AHT20(const uint8_t deviceAddress) { _deviceAddress = deviceAddress; }

// Initialize and calibrate the module
// Read delay enforces a minumum delay between measurements
// (2 seconds recommended)
bool AHT20::begin(TwoWire* wire, uint32_t read_delay_ms) {
  _Wire = wire;

  // Verify connection
  if (!isConnected()) return false;

  // Reset the device
  softReset();
  delay(40);

  // Initialize module
  initialize();

  // Start a measurement
  _read_delay = 0;
  readData();

  // Set read delay
  _read_delay = read_delay_ms;

  // Verify calibration
  return isCalibrated();
}

// Verify I2C communication
// Function will make up to 2 connection attempts in case
// the device is still powering up
bool AHT20::isConnected() {
  // Performing a null transaction messes up the sensor
  // Instead, read the status
  bool count = 0;
  while (!getStatus()) {
    if (count++) return false;
    delay(100);
  }

  return true;
}

// Returns the status byte
uint8_t AHT20::getStatus() {
  _Wire->requestFrom(_deviceAddress, (uint8_t)1);
  if (_Wire->available() == 1) return _Wire->read();
  return 0;
}

// Returns the state of the cal bit in the status byte
bool AHT20::isCalibrated() { return (getStatus() & (1 << 3)); }

// Returns the state of the busy bit in the status byte
bool AHT20::isBusy() { return (getStatus() & (1 << 7)); }

// Send command to initialize the sensor
bool AHT20::initialize() {
  _Wire->beginTransmission(_deviceAddress);
  _Wire->write(CMD_INITIALIZE_2);
  _Wire->write(CMD_INITIALIZE_1);
  _Wire->write(CMD_INITIALIZE_0);
  return _Wire->endTransmission() == 0;
}

// Reset the device
bool AHT20::softReset() {
  _Wire->beginTransmission(_deviceAddress);
  _Wire->write(CMD_RESET);
  return _Wire->endTransmission() == 0;
}

// Send command to start measurement
// Return true if a new measurement was started
bool AHT20::triggerMeasurement() {
  // Check for ongoing measurement and enforce cooldown
  if (_measurementStarted || (millis() - _last_read < _read_delay))
    return false;

  _Wire->beginTransmission(_deviceAddress);
  _Wire->write(CMD_MEASURE_2);
  _Wire->write(CMD_MEASURE_1);
  _Wire->write(CMD_MEASURE_0);

  _measurementStarted = _Wire->endTransmission() == 0;
  return _measurementStarted;
}

// Reads new data from sensor
// This function can be slow, make sure available() is true
// before calling to avoid excessive waiting
bool AHT20::readData() {
  // Start measurement
  if (triggerMeasurement()) {
    delay(75);
  } else if (!_measurementStarted) {
    return false;
  }

  // Wait for reading to complete
  // Time out after 100ms
  uint32_t start = millis();
  while (isBusy()) {
    if (millis() - start >= 100) {
      return false;
    }
    delay(1);
  }

  // Reset timer/flags
  _last_read = millis();
  _measurementStarted = false;

  // Fetch new data
  if (_Wire->requestFrom(_deviceAddress, (uint8_t)7) != 7) {
    return false;
  }

  uint8_t rx_byte;
  crc.reset();

  // Read status
  rx_byte = _Wire->read();
  crc.update(rx_byte);

  // Read humidity
  uint32_t humidity = 0;
  for (int i = 2; i >= 0; i--) {
    rx_byte = _Wire->read();
    crc.update(rx_byte);
    humidity |= (uint32_t)rx_byte << (8 * i);
  }
  humidity >>= 4;

  // Read temperature
  uint32_t temperature = (uint32_t)rx_byte << 16;
  for (int i = 1; i >= 0; i--) {
    rx_byte = _Wire->read();
    crc.update(rx_byte);
    temperature |= (uint32_t)rx_byte << (8 * i);
  }
  temperature &= ~0xFFF00000;

  // Read and check CRC
  crc.update(_Wire->read());
  if (crc.crc != 0) {
    return false;
  }

  // Save data only if it's valid
  _raw_temperature = temperature;
  _raw_humidity = humidity;

  return true;
}

// Return true if both datums are ready to be read
bool AHT20::available() {
  // Trigger a measurement if one has not been started,
  // then return false
  if (!_measurementStarted) {
    triggerMeasurement();
    return false;
  }

  // If measurement has been started, check to see if it's complete.
  return !isBusy();
}

// Get temperature value in celsius
float AHT20::getTemperature(bool force_new) {
  // If new data isn't readily available, use old data
  // (unless force_new is true)
  while (force_new || available()) {
    if (readData()) break;
    delay(1);
  }

  // Convert raw value to celsius
  return ((float)_raw_temperature / 1048576) * 200 - 50;
}

// Get humidity value as a percent [0-100]
float AHT20::getHumidity(bool force_new) {
  // If new data isn't readily available, use old data
  // (unless force_new is true)
  while (force_new || available()) {
    if (readData()) break;
    delay(1);
  }

  return ((float)_raw_humidity / 1048576) * 100;
}
