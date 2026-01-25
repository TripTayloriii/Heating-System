#include <Arduino.h>
#include <max6675.h>
// must use these pins on arduino Uno
int csPin = 10; //only pin subject to change
int soPin = 12;
int sckPin = 13;

MAX6675 thermocouple(csPin, soPin, sckPin);
void setup() {
  thermocouple.begin(); //must be called before readings
  Serial.begin(9600);
  Serial.println("MAX6675 Thermocouple Test");
  delay(100);
}

void loop() {
  Serial.print("Celsius: ");
  Serial.print(thermocouple.getCelsius());
  Serial.println(" °C");
  Serial.print("Fahrenheit: ");
  Serial.print(thermocouple.getFahrenheit());
  Serial.println(" °F");
  Serial.println("-----------------------\n");
  

  delay(225); //Takes 220 ms to ADC
}

