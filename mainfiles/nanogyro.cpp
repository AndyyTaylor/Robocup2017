#include <Arduino.h>

#include "Source/Orientation.h"

#include <EasyTransfer.h>
#include <elapsedMillis.h>

EasyTransfer ET;
elapsedMillis timer;

struct SEND_DATA_STRUCTURE{
    float angle;
    bool stab;
};

SEND_DATA_STRUCTURE mydata;

void setup() {
    Serial.begin(115200);
    Orientation::init();
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    // Serial.println("Init finished");
    ET.begin(details(mydata), &Serial);
}

void loop() {
    Orientation::update();
    
    mydata.angle = Orientation::getYaw();
    mydata.stab = Orientation::isStabalized();
    if (mydata.stab)
        digitalWrite(LED_BUILTIN, HIGH);

    ET.sendData();
}

