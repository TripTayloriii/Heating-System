//MAX6675 Datasheet https://www.analog.com/media/en/technical-documentation/data-sheets/max6675.pdf

#include "max6675.h"

// Constructor to initialize the MAX6675 with specified pins
MAX6675::MAX6675(int csPin, int soPin, int sckPin){
    this->csPin = csPin; //chip select
    this->soPin = soPin; //master input - slave output
    this->sckPin = sckPin; //synchronized clock
}

//Begins SPI and sets csPin to off (HIGH)
void MAX6675::begin(void){
    SPI.begin();
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH); //deselect MAX6675 by default
}

float MAX6675::getCelsius(void){
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0)); //set SPI settings
    digitalWrite(csPin, LOW); //select
    uint16_t value = SPI.transfer16(0); //read 16 bits
    digitalWrite(csPin, HIGH); //deselect
    SPI.endTransaction();

    if(value & 0x4){ //checks if bit2 is 1 (disconnected)
        Serial.println("MAX6675 disconnected");
        return NAN;
    }

    value >>= 3; //remove non-temperature bits
    return value * 0.25; //LSB increments 0.25 Celcius
}

float MAX6675::getFahrenheit(void){
    float celsius = getCelsius();
    if(isnan(celsius)){
        return NAN; //propagate disconnection
    }
    return celsius * 9.0 / 5.0 + 32.0; //convert to Fahrenheit
}