#include <Arduino.h>
#include <max6675.h>
#include <PID.h>
//Thermocouple -------------------------------------
// must use these pins on arduino Uno
int csPin = 10; //only pin subject to change
int soPin = 12;
int sckPin = 13;
int refreshRate = 225; //in ms; 220 bare minimum for MAX6675
float celsiusMeasurement = NAN;
MAX6675 thermocouple(csPin, soPin, sckPin);

//PID system -----------------------------------------
float Kp = 2.5;
float Kd = 15.0;
float Ki = 0.05;
float setpoint = 0.0; //celsius
unsigned long timer = 0;
PID thermoPID(Kp, Ki, Kd);

//User input ------------------------------------------
String inputString = "";

//Heating system -------------------------------------------
int HEATING_PIN = 3;
unsigned long windowSize = 500; // .5 seconds
unsigned long windowStart;
float PIDcorrection = 0.0;
float totalPowerOutput = 0.0;



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
  //Look for new cmds from python
  while(Serial.available()){
    char c = Serial.read();

    if(c == '\n') { //end of cmd
      inputString.trim();
      
      if(inputString.startsWith("SP")){ //Setpoint command
        float newSetpoint = inputString.substring(2).toFloat();
        setpoint = constrain(newSetpoint,0,1000);
      }

      else if(inputString.startsWith("KP")){ //P gain
        float newP = inputString.substring(2).toFloat();
        Kp = newP;
        thermoPID.setK(Kp, Ki, Kd);
      }

      else if(inputString.startsWith("KI")){ //I gain
        float newI = inputString.substring(2).toFloat();
        Ki = newI;
        thermoPID.setK(Kp, Ki, Kd);
      }

      else if(inputString.startsWith("KD")){ //D gain
        float newD = inputString.substring(2).toFloat();
        Kd = newD;
        thermoPID.setK(Kp, Ki, Kd);
      }
      inputString = ""; //clear input
    }else{
      inputString += c;
    }
  }

//--------------------------------------------

  unsigned long currentTime = millis();
  unsigned long dt = (currentTime - timer); //in ms
  
  if(dt >= refreshRate){//only called once every refresh rate period
    //collect measurement and calculate PID
    float newReading = thermocouple.getCelsius();
    if(!isnan(newReading)){
      celsiusMeasurement = 0.9 * celsiusMeasurement + 0.1 * newReading;
    }

    //print current temp reading
    // //Debugging (DO NOT USE IF serialPlotter is on)
    // Serial.print("Celsius: ");
    // Serial.print(celsiusMeasurement);
    // Serial.println(" °C");

    timer = currentTime; //update timer

    PIDcorrection = thermoPID.update(setpoint, celsiusMeasurement, dt / 1000.0);
    PIDcorrection = constrain(PIDcorrection, 0, 100);

    totalPowerOutput = PIDcorrection;
    totalPowerOutput = constrain(totalPowerOutput, 0, 100);

    //Sending PID output to python plotter (using binary protocol)
    Serial.write(0xAA); //reference byte
    Serial.write((uint8_t*)&setpoint, sizeof(float));
    Serial.write((uint8_t*)&celsiusMeasurement, sizeof(float));
    Serial.write((uint8_t*)&totalPowerOutput, sizeof(float));
    Serial.write((uint8_t*)&PIDcorrection, sizeof(float));
    Serial.write((uint8_t*)&Kp, sizeof(float));
    Serial.write((uint8_t*)&Ki, sizeof(float));
    Serial.write((uint8_t*)&Kd, sizeof(float));
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