#include <Arduino.h>

#include "Source/Orientation.h"
#include "Source/Vision.h"

#include <Wire.h>
#include <EasyTransferI2C.h>

struct SEND_DATA_STRUCTURE{
  float angle;
  bool visible;
};

const int I2C_SLAVE_ADDRESS = 9;

bool initEverything();
void dead();

void sendData();
void receiveData();

EasyTransferI2C ET;
SEND_DATA_STRUCTURE send_packet;
bool awaitingdata = false;

int main() {
    if (!initEverything())
        dead();

    while (1) {
        Vision::update();
        Vision::updateMotor();
        
        send_packet.angle = Vision::getBallAngle();
        send_packet.visible = Vision::isVisible();
        
        Serial.println(send_packet.angle);
        ET.sendData(I2C_SLAVE_ADDRESS);
        
        // delay(30);
    }

    return 0;
}

bool initEverything() {
    init();

    Serial.begin(115200);
    Serial.println("Initializing");
    
    if (!Vision::init())
        return false;
        
    Wire.begin();
    ET.begin(details(send_packet), &Wire);

    return true;
}

void dead() {
    Serial.println(F("R.I.P Arduino"));

    while (1) {}
}
