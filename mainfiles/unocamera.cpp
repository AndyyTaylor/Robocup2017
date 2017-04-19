#include <Arduino.h>

#include "Source/Orientation.h"

#include <Wire.h>
#include <EasyTransferI2C.h>

bool initEverything();

void dead();

void requestEvent();

EasyTransferI2C ET;

struct SEND_DATA_STRUCTURE {
  double sendHeading;
};

SEND_DATA_STRUCTURE myheading;

#define I2C_SLAVE_ADRESS 9

int main()
{
    if (!initEverything())
        dead();

    while (1)
    {
        myheading.sendHeading = Orientation::getYaw();
        ET.sendData(I2C_SLAVE_ADRESS);
    }

    return 0;
}

bool initEverything()
{
    init();

    myheading.sendHeading = 400;

    Wire.begin(8);                // join i2c bus with address #8
    ET.begin(details(myheading), &Wire);

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
