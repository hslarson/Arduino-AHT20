#include <Wire.h>
#include "AHT20.h"

AHT20 humiditySensor;

void setup() {
  // Init serial
  Serial.begin(115200);

  // Init I2C bus
  Wire.begin();

  // Init module
  if (!humiditySensor.begin(&Wire, 2000)) {
    Serial.println("AHT20 not detected. Please check wiring. Freezing.");
    while (1);
  }
  Serial.println("AHT20 acknowledged.");
}

void loop() {
  // Check f a new measurement is available
  if (humiditySensor.available() == true) {
    // Get the new temperature and humidity value
    float temperature = humiditySensor.getTemperature();
    float humidity = humiditySensor.getHumidity();

    // Print the results
    Serial.print("Temperature: ");
    Serial.print(temperature, 2);
    Serial.print(" C\t");
    Serial.print("Humidity: ");
    Serial.print(humidity, 2);
    Serial.print("% RH");

    Serial.println();
  }

  delay(10);
}
