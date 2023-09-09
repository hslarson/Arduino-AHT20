#include <Wire.h>
#include "AHT20.h"

AHT20 aht20;
float temperature, humidity;

void setup() {
  // Init serial
  Serial.begin(115200);

  // Init I2C bus
  Wire.begin();

  // Init module
  if (!aht20.begin(&Wire, 2000)) {
    Serial.println("AHT20 not detected. Please check wiring. Freezing.");
    while (1);
  }
  Serial.println("AHT20 acknowledged.");
}

void loop() {
  // Check if a new measurement is available
  if (aht20.available()) {
    // Get the new temperature and humidity value
    if (aht20.getReading(&temperature, &humidity)) {
      // Print the results
      Serial.print("Temperature: ");
      Serial.print(temperature, 2);
      Serial.print(" C\t");
      Serial.print("Humidity: ");
      Serial.print(humidity, 2);
      Serial.print("% RH");

      Serial.println();
    } else {
      Serial.println("Read Error");
    }
  }

  delay(10);
}
