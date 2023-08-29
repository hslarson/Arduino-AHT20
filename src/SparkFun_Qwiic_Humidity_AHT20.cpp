#include <SparkFun_Qwiic_Humidity_AHT20.h>

// Constructor
AHT20::AHT20(const uint8_t deviceAddress) { _deviceAddress = deviceAddress; }

// Initialize and calibrate the module
// Read delay enforces a minumum delay between measurements (2 seconds recommended)
bool AHT20::begin(TwoWire* wire, uint32_t read_delay_ms) {
  _Wire = wire;

  if (!isConnected()) return false;

  // Wait 40 ms after power-on before reading temp or humidity.
  delay(40);

  // Check if the calibrated bit is set. If not, init the sensor.
  if (!isCalibrated()) {
    // Initialize module
    initialize();

    // Start a measurement
    _read_delay = 0;  // Force reading
    readData();

    // Verify calibration
    if (!isCalibrated()) {
      return false;
    }
  }

  // Set read delay
  _read_delay = read_delay_ms;

  // Mark all datums as old (need to be refreshed)
  sensorQueried.temperature = true;
  sensorQueried.humidity = true;

  return true;
}

// Verify I2C communication
bool AHT20::isConnected() {
  // Just performing a null transmission messes up the sensor
  // Instead, read the status
  uint8_t status = getStatus();
  if (status) {
    return true;
  }

  // If IC failed to respond, give it 20ms more for Power On Startup
  delay(100);
  return getStatus() != 0;
}

// Returns the status byte
uint8_t AHT20::getStatus() {
  _Wire->requestFrom(_deviceAddress, (uint8_t)1);
  if (_Wire->available()==1) return (_Wire->read());
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
bool AHT20::triggerMeasurement() {
  // Check for ongoing measurement
  if (_measurementStarted) return true;

  // Enforce read delay
  if (millis() - _last_read < _read_delay) {
    return false;
  }

  _Wire->beginTransmission(_deviceAddress);
  _Wire->write(CMD_MEASURE_2);
  _Wire->write(CMD_MEASURE_1);
  _Wire->write(CMD_MEASURE_0);
  if (_Wire->endTransmission() == 0) {
    _measurementStarted = true;
    return true;
  }

  return false;
}

// Reads new data from sensor
// Ignores sensorQueried flags
bool AHT20::readData() {
  // Start measurement
  // Return false if reading cooldown condition isn't met
  if (!_measurementStarted) {
    if (triggerMeasurement()) {
      delay(75);
    } else
      return false;
  }

  // Time out after 100ms
  int counter = 0;
  while (isBusy() && counter++ < 100) {
    delay(1);
  }
  if (counter >= 100) {
    return false;
  }

  // Reset timer/flags
  _last_read = millis();
  _measurementStarted = false;

  // Read new data
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
  sensorData.temperature = temperature;
  sensorData.humidity = humidity;

  // Mark data as fresh
  sensorQueried.temperature = false;
  sensorQueried.humidity = false;
  return true;
}

// Return true if both datums are fresh
// Return false otherwise
// If either datum is old, start a new reading
bool AHT20::available() {
  // Both datums are fresh
  if (!sensorQueried.humidity && !sensorQueried.temperature) {
    return true;
  }

  // Trigger a measurement if one has not been previously started, 
  // then return false
  if (!_measurementStarted) {
    if (triggerMeasurement()) delay(75);
    return false;
  }

  // If measurement has been started, check to see if it's complete.
  // If not complete, return false
  if (isBusy()) {
    return false;
  }

  // If complete, read data
  // Automatically restart measurement if read fails
  if (!readData()) {
    if (triggerMeasurement()) delay(75);
    return false;
  }
  return true;
}

// Get temperature value in celsius
float AHT20::getTemperature(bool force_new) {
  // If data is old, start new measurement
  // If new data isn't readily available, use old data (unless force_new is true)
  while (sensorQueried.temperature) {
    readData();
    if (!force_new) {
      break;
    }
    delay(1);
  }

  // Convert raw value to celsius
  float tempCelsius = ((float)sensorData.temperature / 1048576) * 200 - 50;

  // Mark data as old
  sensorQueried.temperature = true;

  return tempCelsius;
}

// Get humidity value as a percent [0-100]
float AHT20::getHumidity(bool force_new) {
  // If data is old, get new data
  // If new data isn't readily available, use old data (unless force_new is true)
  while (sensorQueried.humidity) {
    readData();
    if (!force_new) {
      break;
    }
    delay(1);
  }

  // Convert raw value to percent
  float relHumidity = ((float)sensorData.humidity / 1048576) * 100;

  // Mark data as old
  sensorQueried.humidity = true;

  return relHumidity;
}
