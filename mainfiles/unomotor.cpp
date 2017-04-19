#include <Arduino.h>
#include <Wire.h>
#include <EasyTransferI2C.h>

#include "Source/Orientation.h"
#include "Source/MotorDriver.h"

bool initEverything();

void dead();

void receive();

EasyTransferI2C ET;

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  double sendHeading;
};

RECEIVE_DATA_STRUCTURE myheading;

#define I2C_SLAVE_ADRESS 9

int main()
{
    if (!initEverything())
        dead();

    while (1)
    {
        //MotorDriver::direction(90);
        if(ET.receiveData()) {
          Serial.println(myheading.sendHeading);
        }
    }

    return 0;
}

bool initEverything()
{
    init();

    Serial.begin(115200);
    Serial.println("Initializing");

    Wire.begin(I2C_SLAVE_ADRESS);

    ET.begin(details(myheading), &Wire);

    Wire.onReceive(receive);

    Orientation::init();
    MotorDriver::init();
    MotorDriver::setMaxSpeed(200);

    return true;
}

void receive(int numBytes) {}

void dead()
{
    Serial.println(F("R.I.P Arduino"));

    while (1);
}
