#include <Arduino.h>

#include "Source/MotorDriver.h"
#include "Source/Orientation.h"
#include "DualMC33926MotorShield.h"

#include <EasyTransfer.h>
#include <Wire.h>
#include <EasyTransferI2C.h>
#include <elapsedMillis.h>

struct RECEIVE_DATA_STRUCTURE_LIDAR{
    int x;
    int y;
};

struct RECEIVE_DATA_STRUCTURE_CAMERA{
    float angle;
    bool foundball;
};

struct RECEIVE_DATA_STRUCTURE_GYRO {
    float angle;
    bool stab;
};

const int I2C_SLAVE_ADDRESS = 9;

void receive(int numBytes) {}
void printSim();
void dead();

void receiveRadar();
void receiveCamera();
void receiveGyro();

bool radarValid();
bool radarTrash();
bool radarXValid();
bool radarYValid();

bool initEverything();


RECEIVE_DATA_STRUCTURE_LIDAR mylidar;
RECEIVE_DATA_STRUCTURE_CAMERA mydata;
RECEIVE_DATA_STRUCTURE_GYRO gyroData;

DualMC33926MotorShield tp(7, 11, A0, 8, 12, A1, 4, 0);
DualMC33926MotorShield bt(14, 5, A2, 15, 2, A3, 17, 16);

EasyTransfer radarIn;
EasyTransfer GyroIn;
EasyTransferI2C cameraIn;

elapsedMillis serialTimer;

int x, y, width, height, ball, gyro, vis, stab;

int main() {
    if (!initEverything())
        dead();
    
    while (1) {
        // receiveCamera();
        receiveRadar();
        // receiveGyro();
        
        
        if (serialTimer > 50) {
            // printSim();
            serialTimer = 0;
        }
    }

    return 0;
}

void receiveGyro() {
    if (GyroIn.receiveData()) {
        gyro = gyroData.angle;
        stab = gyroData.stab;
    }
}

void receiveCamera() {
    if (cameraIn.receiveData()) {
        ball = mydata.angle;
        vis = mydata.foundball;
    }
}

void receiveRadar() {
    if (radarIn.receiveData()) {
        Serial.print(mylidar.x);
        Serial.print(", ");
        Serial.println(mylidar.y);
    }
}

void printSim() {
    Serial.print(x);
    Serial.print(F(", "));
    Serial.print(y);
    Serial.print(F(", "));
    Serial.print(width);
    Serial.print(F(", "));
    Serial.print(height);
    Serial.print(F(", "));
    Serial.print(ball);
    Serial.print(F(", "));
    Serial.print(gyro);
    Serial.print(F(", "));
    Serial.print(vis);
    Serial.print(F(", "));
    Serial.println(stab);
}

bool initEverything() {
    init();

    Serial.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
    Serial.println(F("Initializing"));

    Wire.begin(I2C_SLAVE_ADDRESS);
    cameraIn.begin(details(mydata), &Wire);
    GyroIn.begin(details(gyroData), &Serial2);
    radarIn.begin(details(mylidar), &Serial3);

    Wire.onReceive(receive);
    
    MotorDriver::init();
    MotorDriver::setMaxSpeed(100);

    return true;
}

void dead() {
    Serial.println(F("R.I.P Arduino"));

    while (1) continue;
}
