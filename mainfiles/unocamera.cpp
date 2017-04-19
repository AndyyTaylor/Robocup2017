#include <Arduino.h>

#include "Source/Orientation.h"

#include <Wire.h>

bool initEverything();

void dead();

void sendData();

int main()
{
    if (!initEverything())
        dead();

    while (1)
    {
        Orientation::update();
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
    
    Wire.begin(8);                // join i2c bus with address #8
    Wire.onRequest(sendData);

    

    return true;
}

void sendData()
{
    Serial.println("Sending data");
    String s = String(Orientation::getYaw());
    while (s.length() < 6)
    {
        s += "0";
    }
    Serial.println(s);
    
    char data[6];
    for (int i = 0; i < 6; i++)
    {
        data[i] = (char) s[i];
    }
    Serial.println(data[0]);
    Wire.write(data);
}

void dead()
{
    Serial.println(F("R.I.P Arduino"));

    while (1);
}
