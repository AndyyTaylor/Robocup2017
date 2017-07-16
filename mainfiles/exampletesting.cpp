#include <Arduino.h>

#include "Source/Orientation.h"
#include "Source/MotorDriver.h"

#include "DualMC33926MotorShield.h"

#include <EasyTransfer.h>
#include <Wire.h>
#include <EasyTransferI2C.h>

//create object
EasyTransfer ETin;
EasyTransferI2C ET;

#define I2C_SLAVE_ADDRESS 9

void dead();

void receive(int numBytes) {}

bool radarValid();
bool radarTrash();
bool radarXValid();
bool radarYValid();
bool initEverything();

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float angle;
  bool foundball;
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE mydata;

struct RECEIVE_DATA_STRUCTURE_LIDAR{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int x;
  int y;
  int width;
  int height;
  float angle;
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE_LIDAR mylidar;

DualMC33926MotorShield tp(7, 11, A0, 8, 12, A1, 4, 0);
DualMC33926MotorShield bt(14, 5, A2, 15, 2, A3, 17, 16);

int main()
{
    if (!initEverything())
        dead();

    while (1)
    {
        if(ETin.receiveData()){
          //this is how you access the variables. [name of the group].[variable name]
          //since we have data, we will blink it out.
          if (!radarTrash()){
            if (radarXValid()) Serial.print(mylidar.x);
            else Serial.print("----");
            Serial.print(", ");
            if (radarYValid()) Serial.print(mylidar.y);
            else Serial.print("----");
            Serial.print(" | ");
            Serial.print(mylidar.width);
            Serial.print(" x ");
            Serial.print(mylidar.height);
            Serial.print(" -> ");
            Serial.println(mylidar.angle);
            //MotorDriver::update(mydata.number);
          } else {
            Serial.println("--------------------------------");
          }
        }
        if(ET.receiveData()){
          //this is how you access the variables. [name of the group].[variable name]
          //since we have data, we will blink it out.
          //Serial.println(mydata.angle);
          //MotorDriver::update(mydata.number);
        }
        //MotorDriver::direction(0);
        bt.setM1Speed(0);
        bt.setM2Speed(-100);
        tp.setM1Speed(0);
        tp.setM2Speed(100);
        // Serial.println("test");


    }

    return 0;
}

bool radarValid() {
  return abs(1840-mylidar.width) < 50 && abs(2450-mylidar.height) < 50;
}

bool radarTrash() {
  return abs(1840-mylidar.width) > 50 && abs(2450-mylidar.height) > 50;
}

bool radarXValid() {
  return abs(1840-mylidar.width) < 50;
}

bool radarYValid() {
  return abs(2450-mylidar.height) < 50;
}

bool initEverything()
{
    init();

    Serial.begin(115200);
    Serial3.begin(115200);
    Serial.println("Initializing");

      Wire.begin(I2C_SLAVE_ADDRESS);
      //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
      ET.begin(details(mydata), &Wire);
      //define handler function on receiving data
      Wire.onReceive(receive);

  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
    ETin.begin(details(mylidar), &Serial3);
  //define handler function on receiving data

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
