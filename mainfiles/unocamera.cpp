#include <Arduino.h>

#include "Source/Orientation.h"

bool initEverything();

void dead();

int main()
{
    if (!initEverything())
        dead();
    
    while (1)
    {
        Orientation::update();
        // Orientation::getCompassHeading()
        // Orientation::getYaw()
        Serial.println(Orientation::getYaw());
    }
    
    return 0;
}

bool initEverything()
{
    init();
    
    Serial.begin(115200);
    Serial.println("Initializing");
    
    if (!Orientation::init())
        return false;
    
    return true;
}

void dead()
{
    Serial.println(F("R.I.P Arduino"));
    
    while (1);
}