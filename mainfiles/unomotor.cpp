#include <Arduino.h>
//#include <Wire.h>

#include "Source/Orientation.h"
#include "Source/MotorDriver.h"

#include <EasyTransferI2C.h>

//create object
EasyTransferI2C ET;

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float number;
  float angle;
  bool foundball;
  bool stab;
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE mydata;

#define I2C_SLAVE_ADDRESS 9

void receive(int numBytes) {}

bool initEverything();

void dead();

int lastmillis = 0;

int main()
{
    if (!initEverything())
        dead();

    while (1)
    {
        if(ET.receiveData()){
          //this is how you access the variables. [name of the group].[variable name]
          //since we have data, we will blink it out.
          float testing = mydata.angle;
          bool doit = mydata.stab;
          Serial.println(testing);
          //MotorDriver::update(mydata.number);
        }
        //Serial.println("yay");
        MotorDriver::direction(100);
    }

    return 0;
}

bool initEverything()
{
    init();

    Serial.begin(115200);
    Serial.println("Initializing");

    Wire.begin(I2C_SLAVE_ADDRESS);
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
    ET.begin(details(mydata), &Wire);
  //define handler function on receiving data
    Wire.onReceive(receive);

    //Orientation::init();
    MotorDriver::init();
    MotorDriver::setMaxSpeed(100);

    return true;
}

void dead()
{
    Serial.println(F("R.I.P Arduino"));

    while (1);
}
