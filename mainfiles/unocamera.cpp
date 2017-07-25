#include <Arduino.h>

#include "Source/Orientation.h"
#include "Source/Vision.h"

#include <Wire.h>
#include <EasyTransfer.h>
#include <elapsedMillis.h>

struct SEND_DATA_STRUCTURE {
  float angle;
  int bally;
  int visible;
  bool motorButton;
  bool gyroButton;
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
elapsedMillis easyTimer;
SEND_DATA_STRUCTURE send_packet;
RECEIVE_DATA_STRUCTURE receive_packet;

bool awaitingdata = false;
int target = 1;
bool lidarButtonOn = true;
bool lidarButtonReleased = true;
bool motorButtonOn = false;
bool gyroButtonOn = false;

int main() {
    if (!initEverything())
        dead();

    while (1) {
        digitalWrite(LED_BUILTIN, LOW);
        Vision::update(target);
        if (!lidarButtonOn) {
            Vision::updateMotor();
        }
        
        
        if (digitalRead(4) == LOW) {
            lidarButtonReleased = true;
        } else if (digitalRead(4) == HIGH && lidarButtonReleased) {
            lidarButtonReleased = false;
            if (lidarButtonOn)
                lidarButtonOn = false;
            else
                lidarButtonOn = true;
        }
        
        if (!lidarButtonOn)
            digitalWrite(7, HIGH);
        else
            digitalWrite(7, LOW);
        
        if (digitalRead(3) == LOW) {
            motorButtonOn = false;
        } else {
            motorButtonOn = true;
        }
        
        if (digitalRead(2) == LOW)
            gyroButtonOn = false;
        else
            gyroButtonOn = true;
        
        if (easyTimer > 30) {
            easyTimer = 0;
            send_packet.angle = Vision::getBallAngle();
            if (Vision::isVisible()) {
                send_packet.visible = target;
            } else {
                send_packet.visible = 0;
            }
            send_packet.bally = Vision::getBallY();
            
            send_packet.motorButton = motorButtonOn;
            send_packet.gyroButton = !gyroButtonOn;
            
            ETout.sendData();
        }
        
        
        if (ETin.receiveData()) {
            target = receive_packet.target;
        }
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
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    pinMode(4, INPUT);
    pinMode(7, OUTPUT);
    
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
    
    ETout.begin(details(send_packet), &Serial);
    ETin.begin(details(receive_packet), &Serial);

    return true;
}

void dead() {
    Serial.println(F("R.I.P Arduino"));

    while (1) {}
}
