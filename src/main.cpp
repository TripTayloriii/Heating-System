#include <Arduino.h>
#include <max6675.h>

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  MAX6675 thermocouple(10, 12, 13); // Initialize with CS, SO, SCK pins
  thermocouple.begin();

  Serial.println("MAX6675 Thermocouple Test");
}
void loop() {
  MAX6675 thermocouple(10, 12, 13); // Initialize with CS, SO, SCK pins
  float celsius = thermocouple.getCelsius();
  float fahrenheit = thermocouple.getFahrenheit();

  Serial.print("Temperature: ");
  Serial.print(celsius);
  Serial.print(" °C | ");
  Serial.print(fahrenheit);
  Serial.println(" °F");

  delay(1000); // Wait for 1 second before next reading
}