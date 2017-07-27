#include <Arduino.h>

#include "Source/Orientation.h"

#include <EasyTransfer.h>
#include <elapsedMillis.h>

struct SEND_DATA_STRUCTURE{
    float angle;
    bool stab;
};

int A0MAX = 0;
int A1MAX = 0;
int A2MAX = 0;
int A3MAX = 0;

const int LINE_HANG = 50;

bool onLine();
void debugLightSensors();

EasyTransfer ET;
elapsedMillis stabTimer;
elapsedMillis lineTimer;

SEND_DATA_STRUCTURE mydata;

void setup() {
    Serial.begin(115200);
    Orientation::init();
    
    pinMode(13, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    
    A0MAX = analogRead(A0)+100;
    A1MAX = analogRead(A1)+100;
    A2MAX = analogRead(A2)+100;
    A3MAX = analogRead(A3)+100;
    
    mydata.stab = false;
    ET.begin(details(mydata), &Serial);
}

void loop() {
    Orientation::update();
    
    mydata.angle = Orientation::getYaw();
    mydata.stab = Orientation::isStabalized();
        
    if (onLine())
        lineTimer = 0;
        
    if (mydata.stab || stabTimer > 100) {
        ET.sendData();
        // debugLightSensors();
        stabTimer = 0;
    }
    
    if (lineTimer < LINE_HANG) {
        digitalWrite(13, HIGH);
        digitalWrite(12, HIGH);
    } else {
        digitalWrite(13, LOW);
        digitalWrite(12, LOW);
    }
}

bool onLine() {
    return A0MAX < analogRead(A0) || A1MAX < analogRead(A1) || A2MAX < analogRead(A2) || A3MAX < analogRead(A3);
    // return A2MAX < analogRead(A2);
}

void debugLightSensors() {
    if (onLine()) Serial.println("-----------");
    if (A0MAX == 0 || A0MAX < analogRead(A0)) {
        Serial.print("A0: ");
        Serial.println(analogRead(A0));
    }
    if (A1MAX == 0 || A1MAX < analogRead(A1)) {
        Serial.print("A1: ");
        Serial.println(analogRead(A1));
    }
    if (A2MAX == 0 || A2MAX < analogRead(A2)) {
        Serial.print("A2: ");
        Serial.println(analogRead(A2));
    }
    if (A3MAX == 0 || A3MAX < analogRead(A3)) {
        Serial.print("A3: ");
        Serial.println(analogRead(A3));
    }
}
