#include <Arduino.h>

#include "Source/Orientation.h"

//#include <Wire.h>

#include <EasyTransferI2C.h>

//create object
EasyTransferI2C ET;

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  double number;
};

//give a name to the group of data
SEND_DATA_STRUCTURE mydata;

//define slave i2c address
#define I2C_SLAVE_ADDRESS 9

bool initEverything();

void dead();

void sendData();
void receiveData();

bool awaitingdata = false;

int main()
{
    if (!initEverything())
        dead();

    while (1)
    {
        Orientation::update();
        mydata.number = Orientation::getYaw();
        Serial.println(mydata.number);
        ET.sendData(I2C_SLAVE_ADDRESS);
        //Serial.println(Orientation::getCompassHeading());
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

    ET.begin(details(mydata), &Wire);

    mydata.number=0;

    return true;
}

void dead()
{
    Serial.println(F("R.I.P Arduino"));

    while (1);
}
