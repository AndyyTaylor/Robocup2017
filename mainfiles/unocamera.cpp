#include <Arduino.h>

#include "Source/Orientation.h"
#include "Source/Vision.h"

#include <Wire.h>
#include <EasyTransfer.h>

struct SEND_DATA_STRUCTURE {
  float angle;
  int bally;
  int visible;
};

struct RECEIVE_DATA_STRUCTURE {
    int target;
};

const int I2C_SLAVE_ADDRESS = 9;

bool initEverything();
void dead();

void sendData();
void receiveData();

EasyTransfer ETin;
EasyTransfer ETout;
SEND_DATA_STRUCTURE send_packet;
RECEIVE_DATA_STRUCTURE receive_packet;

bool awaitingdata = false;
int target = 1;

int main() {
    if (!initEverything())
        dead();

    while (1) {
        digitalWrite(LED_BUILTIN, LOW);
        Vision::update(target);
        Vision::updateMotor();
        
        send_packet.angle = Vision::getBallAngle();
        if (Vision::isVisible()) {
            send_packet.visible = target;
        } else {
            send_packet.visible = 0;
        }
        send_packet.bally = Vision::getBallY();

        ETout.sendData();
        
        if (ETin.receiveData()) {
            target = receive_packet.target;
        }
        
        // delay(30);
    }

    return 0;
}

bool initEverything() {
    init();

    Serial.begin(115200);
    
    if (!Vision::init())
        return false;
    

    Wire.begin();
    pinMode(LED_BUILTIN, OUTPUT);
    ETout.begin(details(send_packet), &Serial);
    ETin.begin(details(receive_packet), &Serial);

    return true;
}

void dead() {
    Serial.println(F("R.I.P Arduino"));

    while (1) {}
}
