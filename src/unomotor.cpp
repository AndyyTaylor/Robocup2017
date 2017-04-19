#include <Arduino.h>

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
        MotorDriver::direction(90);
    }

    return 0;
}

bool initEverything()
{
    init();

    Serial.begin(115200);
    Serial.println("Initializing");

    Orientation::init();
    MotorDriver::init();
    MotorDriver::setMaxSpeed(200);

    return true;
}

void dead()
{
    Serial.println(F("R.I.P Arduino"));

    while (1);
}
