#include <Arduino.h>
#include <Wire.h>

#include "Source/Orientation.h"
#include "Source/MotorDriver.h"

bool initEverything();

void dead();

int main()
{
    if (!initEverything())
        dead();

    while (1)
    {
        //MotorDriver::direction(90);
        Serial.println("Loop");
        Wire.requestFrom(8, 6);
        
        
        while (Wire.available())
        {
            char c = Wire.read();
            Serial.print(c);
        }
        
        
        Serial.println();
        delay(1000);
    }

    return 0;
}

bool initEverything()
{
    init();

    Serial.begin(115200);
    Serial.println("Initializing");

    Wire.begin();

    //Orientation::init();
    MotorDriver::init();
    MotorDriver::setMaxSpeed(200);

    return true;
}

void dead()
{
    Serial.println(F("R.I.P Arduino"));

    while (1);
}
