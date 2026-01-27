//MAX6675 Datasheet https://www.analog.com/media/en/technical-documentation/data-sheets/max6675.pdf

#ifndef max6675_H
#define max6675_H

#include <Arduino.h>
#include <SPI.h>

class MAX6675{
    public:
        MAX6675(int csPin, int soPin, int sckPin); //pin 10, 12, 13 respectively on Arduino Uno
        void begin(void);
        float getCelsius(void);
        float getFahrenheit(void);

    private:
        int csPin; //pin 10 (on Arduino Uno)
        int soPin; //pin 12
        int sckPin; //pin 13
};

#endif  // max6675_H