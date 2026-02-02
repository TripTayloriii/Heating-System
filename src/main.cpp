#include <Arduino.h>
#include <max6675.h>
#include <PID.h>
//Thermocouple -------------------------------------
// must use these pins on arduino Uno
int csPin = 10; //only pin subject to change
int soPin = 12;
int sckPin = 13;
int refreshRate = 225; //in ms; 220 bare minimum for MAX6675
float celsiusMeasurement = 0;
MAX6675 thermocouple(csPin, soPin, sckPin);

//PID system -----------------------------------------
float Kp = 3;
float Kd = 10.0;
float Ki = 0.1;
float Kpow = 2; //powerOutput = Kpow * setpoint - ambient
int calibrator = A1; //potentiometer calibrator
float setpoint = 0.0; //celsius
float ambientTemp = 22.0; //celsius
unsigned long timer = 0;
PID thermoPID(Kp, Ki, Kd);

//Heating system -------------------------------------------
int HEATING_PIN = 3;
unsigned long windowSize = 500; // 2 seconds
unsigned long windowStart;
float PIDcorrection = 0.0;
float totalPowerOutput = 0.0;
float testPowerValue = 0.0;



void setup() {
  pinMode(HEATING_PIN, OUTPUT);
  thermocouple.begin(); //must be called before readings
  Serial.begin(9600);
  Serial.println("MAX6675 Thermocouple PID Test");
  delay(100);
  celsiusMeasurement = thermocouple.getCelsius();
  PIDcorrection = thermoPID.update(setpoint, celsiusMeasurement, 1);
  timer = millis();
  windowStart = timer;
}


void loop() {
  unsigned long currentTime = millis();
  unsigned long dt = (currentTime - timer); //in ms

  long calibrationValue = (long)analogRead(calibrator);
  
  // if(calibrationValue >= 500){
  //   testPowerValue = 5;
  // }else{
  //   testPowerValue = 0;
  // }

  setpoint = (float) map(calibrationValue, 0, 1023, 0, 50); //map potentiometer reading to variable range
  

  if(dt >= refreshRate){//only called once every refresh rate period
    //collect measurement and calculate PID
    celsiusMeasurement = 0.9 * celsiusMeasurement + 0.1 * thermocouple.getCelsius();

    //print current temp reading
    // //Debugging (DO NOT USE IF serialPlotter is on)
    // Serial.print("Celsius: ");
    // Serial.print(celsiusMeasurement);
    // Serial.println(" °C");

    timer = currentTime; //update timer

    float predictivePower = Kpow * (setpoint - ambientTemp); 
    predictivePower = constrain(predictivePower, 0, 100);

    PIDcorrection = thermoPID.update(setpoint, celsiusMeasurement, dt / 1000.0);
    PIDcorrection = constrain(PIDcorrection, 0, 100);

    totalPowerOutput = PIDcorrection;
    totalPowerOutput = constrain(totalPowerOutput, 0, 100);
    // //for testing
    // totalPowerOutput = testPowerValue;

    //Sending PID output to python plotter (using binary protocol)
    Serial.write(0xAA); //reference byte
    Serial.write((uint8_t*)&setpoint, sizeof(float));
    Serial.write((uint8_t*)&celsiusMeasurement, sizeof(float));
    Serial.write((uint8_t*)&totalPowerOutput, sizeof(float));
    Serial.write((uint8_t*)&PIDcorrection, sizeof(float));
  }
  


  //duty-cycle heating system
  if(currentTime - windowStart > windowSize){ //create new cycle window if exceeded current window
    windowStart = currentTime;
  }
  long timeHeatOn = windowSize * totalPowerOutput / 100.0;
  if(currentTime - windowStart < timeHeatOn){
    digitalWrite(HEATING_PIN, HIGH);
  }else{
    digitalWrite(HEATING_PIN, LOW);
  }
}