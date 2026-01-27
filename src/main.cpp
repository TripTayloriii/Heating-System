#include <Arduino.h>
#include <max6675.h>
#include <PID.h>
//Thermocouple -------------------------------------
// must use these pins on arduino Uno
int csPin = 10; //only pin subject to change
int soPin = 12;
int sckPin = 13;
int refreshRate = 225; //in ms; 220 bare minimum for MAX6675

MAX6675 thermocouple(csPin, soPin, sckPin);

//PID system -----------------------------------------
float Kp = 1;
float Kd = 0;
float Ki = 0;
int calibrator = A1; //potentiometer calibrator
float setpoint = 25.0; //celsius
unsigned long timer = 0;
PID thermoPID(Kp, Ki, Kd);

//Heating system -------------------------------------------
int HEATING_PIN = 3;
unsigned long windowSize = 2000; // 2 seconds
unsigned long windowStart;
float percentPower = 0.0;



void setup() {
  pinMode(HEATING_PIN, OUTPUT);
  thermocouple.begin(); //must be called before readings
  Serial.begin(9600);
  Serial.println("MAX6675 Thermocouple PID Test");
  delay(100);
  timer = millis();
  windowStart = timer;
}


void loop() {
  unsigned long currentTime = millis();
  unsigned long dt = (currentTime - timer); //in ms

  // long calibrationValue = (long)analogRead(calibrator);
  // Kp = (float) map(calibrationValue, 0, 1023, 0, 500) / 100.0; //map potentiometer reading from 0-5.0

  if(dt >= refreshRate){//only called once every refresh rate period
    //print current temp reading
    float celsiusMeasurement = thermocouple.getCelsius();
    Serial.print("Celsius: ");
    Serial.print(celsiusMeasurement);
    Serial.println(" °C");

    //collect measurement
    timer = currentTime; //update timer
    percentPower = thermoPID.update(setpoint, celsiusMeasurement, dt / 1000.0);
    percentPower = constrain(percentPower, 0, 100);

    //Sending PID output to python plotter (using binary protocol)
    Serial.write(0xAA); //reference byte
    Serial.write((uint8_t*)&setpoint, sizeof(float));
    Serial.write((uint8_t*)&celsiusMeasurement, sizeof(float));
    Serial.write((uint8_t*)&percentPower, sizeof(float));
  }
  


  //duty-cycle heating system
  if(currentTime - windowStart > windowSize){ //create new cycle window if exceeded current window
    windowStart = currentTime;
  }
  long timeHeatOn = windowSize * percentPower / 100.0;
  if(currentTime - windowStart < timeHeatOn){
    digitalWrite(HEATING_PIN, HIGH);
  }else{
    digitalWrite(HEATING_PIN, LOW);
  }
}