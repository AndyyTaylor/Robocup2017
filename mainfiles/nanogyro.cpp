#include <Arduino.h>

#include "Source/Orientation.h"

#include <EasyTransfer.h>

EasyTransfer ET;

struct SEND_DATA_STRUCTURE{
    float angle;
    bool stab;
};

SEND_DATA_STRUCTURE mydata;

void setup() {
    Orientation::init();
    
    Serial.begin(115200);
    ET.begin(details(mydata), &Serial);
}

void loop() {
    Orientation::update();
    
    mydata.angle = Orientation::getYaw();
    mydata.stab = Orientation::isStabalized();
    // Serial.println(mydata.angle);
    ET.sendData();
}

