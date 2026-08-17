#include <Arduino.h>
// #include <max6675.h>
// int CS_PIN = 10; // must use these pins on arduino Uno (for MAX6675)
// int SO_PIN = 12;
// int SCK_PIN = 13;

#include <PID.h>
//Thermocouple -------------------------------------

int REFRESH_RATE = 25000; //in micros; 200000 micro seconds bare minimum for Phidget (225000 for MAX6675)
float celsiusMeasurement; //ambient

//PID system -----------------------------------------
float Kp = 0.15;
float Kd = 0.15;
float Ki = 0.1;
float setpoint = 0.0; //celsius
bool feedforwardOn = true;
unsigned long timer = 0;
PID thermoPID(Kp, Ki, Kd);

//User input ------------------------------------------
String inputString = "";

//Heating system -------------------------------------------
int HEATING_PIN = 3;
unsigned long WINDOW_SIZE = 1000000; // micro seconds
unsigned long windowStart;
float PIDcorrection = 0.0;
float totalPowerOutput = 0.0;

// Setup------------------------------------------------------------------------------

void setup() {
  pinMode(HEATING_PIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Phidget Thermocouple PID Test");
  delay(100);
  timer = micros();
  windowStart = timer;
}

// Main loop--------------------------------------------------------------------------------
void loop() {
  //Look for new cmds from python
  while(Serial.available()){
    char c = Serial.read();

    if(c == '\n') { //end of cmd
      inputString.trim();
      
      if(inputString.startsWith("TM")){ //temperature update
        celsiusMeasurement = inputString.substring(2).toFloat();
      }

      else if(inputString.startsWith("SP")){ //setpoint command
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

      else if(inputString.startsWith("FF")){ //toggle feedforward
        feedforwardOn = !feedforwardOn;
      }
      inputString = ""; //clear input
    }
    else{
      inputString += c;
    }
  }

//----------------------------------------------------

  unsigned long currentTime = micros();
  unsigned long dt = (currentTime - timer); //in micros
  
  if(dt >= REFRESH_RATE){//only called once every refresh rate period

    timer = currentTime; //update timer

    // Feedforward
    float feedforward = 0.308 * (setpoint - 83.545); //derived with trendline of hold temps at 5V

    // PID response
    PIDcorrection = 0.8*PIDcorrection + 0.2*thermoPID.update(setpoint, celsiusMeasurement, dt / 100000.0); //Dampened output
    PIDcorrection = constrain(PIDcorrection, -100, 100);

    // Combined response
    // totalPowerOutput = feedforwardOn*feedforward + PIDcorrection; 
    totalPowerOutput = setpoint;
    totalPowerOutput = constrain(totalPowerOutput, 0, 100);

    //Sending PID output to python plotter (using binary protocol)
    Serial.write(0xAA); //reference byte
    Serial.write((uint8_t*)&setpoint, sizeof(float));
    Serial.write((uint8_t*)&totalPowerOutput, sizeof(float));
    Serial.write((uint8_t*)&PIDcorrection, sizeof(float));
    Serial.write((uint8_t*)&Kp, sizeof(float));
    Serial.write((uint8_t*)&Ki, sizeof(float));
    Serial.write((uint8_t*)&Kd, sizeof(float));
  }
  
  //duty-cycle heating system
  if(currentTime - windowStart > WINDOW_SIZE){ //create new cycle window if exceeded current window
    windowStart = currentTime;
  }
  long timeHeatOn = WINDOW_SIZE * totalPowerOutput / 100.0;
  if(currentTime - windowStart < timeHeatOn){
    digitalWrite(HEATING_PIN, HIGH);
  }else{
    digitalWrite(HEATING_PIN, LOW);
  }
}