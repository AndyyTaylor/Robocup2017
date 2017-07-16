#include <Arduino.h>

#include "Source/Orientation.h"
#include "Source/Vision.h"

#include <Wire.h>

#include <EasyTransferI2C.h>

//create object
EasyTransferI2C ET;

struct SEND_DATA_STRUCTURE{
  float angle;
  bool visible;
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

int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

int main() {
    if (!initEverything())
        dead();

    while (1) {
        // Orientation::update();
        Vision::update();
        // Orientation::loadGyroData();
        Vision::updateMotor();
        
        // mydata.number = Orientation::getYaw();
        // mydata.number = 17;
        mydata.angle = Vision::getBallAngle();
        mydata.visible = Vision::isVisible();
        // mydata.stab = Orientation::isStabalized();
        Serial.println(mydata.angle);
        ET.sendData(I2C_SLAVE_ADDRESS);
        // Serial.println("Loop");
        // Serial.println(Vision::getBallAngle());
        
        
        // Serial.print(mydata.number);
        // Serial.print(" : ");
        // Serial.println(freeRam());
        /*Serial.println(Orientation::getCompassHeading());
        Serial.println(Orientation::getYaw());*/
        delay(25);
    }

    return 0;
}

bool initEverything() {
    init();

    Serial.begin(115200);
    Serial.println("Initializing");

    // if (!Orientation::init())
    //     return false;
    
    Serial.println("Init Vision");
    if (!Vision::init())
        return false;
        
    Wire.begin();
    ET.begin(details(mydata), &Wire);

    return true;
}

void dead() {
    Serial.println(F("R.I.P Arduino"));

    while (1) {}
}
