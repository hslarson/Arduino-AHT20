# AHT20 Arduino Library

## Improvements
- Implemented cyclic redundancy check (CRC)
- getTemperature() and getHumidity() can be called in a non-blocking manner
- Fixed bug where isConnected() prevents new measurements
- Improved readData() to be more fault tolerant
- Added an integrated reading cooldown

## Repository Contents
* **/examples** - Example code to show how the functions work.
* **/src** - Source files for the library (.cpp, .h).
* **library.properties** - General library properties for the Arduino package manager.
* **library.json** - General package properties.

## Documentation
* **[AHT20 Datasheet](https://files.seeedstudio.com/wiki/Grove-AHT20_I2C_Industrial_Grade_Temperature_and_Humidity_Sensor/AHT20-datasheet-2020-4-16.pdf)**
* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)**
