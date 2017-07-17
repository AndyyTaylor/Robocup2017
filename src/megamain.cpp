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
    int width;
    int height;
    float angle;
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
int RADAR_ERROR = 100;

void receive(int numBytes) {}
void printSim();
void dead();

void receiveRadar();
void receiveCamera();
void receiveGyro();

float angleToTarget(int x1, int y1, int x2, int y2);
float distanceToTarget(int x1, int y1, int x2, int y2);

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
elapsedMillis visionTimer;
elapsedMillis yTimer;
elapsedMillis xTimer;

int x, y, width, height, ball, gyro, vis, stab;
float mode = 0;

int main() {
    if (!initEverything())
        dead();
    
    while (1) {
        receiveCamera();
        receiveRadar();
        receiveGyro();
        
        MotorDriver::update(gyro);
        // MotorDriver::direction(0);
        
        if (stab) {
            if ((vis || visionTimer < 100) && false) {
                mode = 0;
                if (vis) {
                    visionTimer = 0;
                }
                
                if (abs(ball) < 20) {
                    // MotorDriver::direction(0);
                } else {
                    // MotorDriver::direction(ball * 1.5);
                    // MotorDriver::direction(ball + (20 * (ball / abs(ball))));
                }
            } else if (yTimer < 700 && xTimer < 700 && (abs(y-2000) > 100 || abs(x-900) > 100)) {
                mode = distanceToTarget(x, y, 900, 2000) / 13 + 75;
                RADAR_ERROR = distanceToTarget(x, y, 900, 2000) / 4;
                mode = RADAR_ERROR;
                if (yTimer < 700 && xTimer < 700) {
                    MotorDriver::direction(angleToTarget(x, y, 900, 2000));
                    MotorDriver::setMaxSpeed(distanceToTarget(x, y, 900, 2000) / 13 + 75);
                } else if (yTimer < 1000 && abs(y-1800) > 100) {
                    // MotorDriver::direction(180);
                } else if (xTimer < 1000 && abs(x-900) > 100) {
                    // MotorDriver::direction(90);
                } else if (xTimer < 1000 && abs(x-900) > 100) {
                    // MotorDriver::direction(-90);
                }
            } else {
                MotorDriver::stop();
            }
        } else {
            MotorDriver::stop();
        }
        
        if (serialTimer > 30) {
            // printSim();
            Serial.print(mode);
            Serial.print(" -> ");
            Serial.print(x);
            Serial.print(", ");
            Serial.print(y);
            Serial.print(" | ");
            
            if (vis)
                Serial.print(ball);
            else
                Serial.print("---");
                
            Serial.print(" - ");
            
            if (stab)
                Serial.println(gyro);
            else
                Serial.println("---");
                
            serialTimer = 0;
        }
    }

    return 0;
}

float angleToTarget(int x1, int y1, int x2, int y2) {
    return atan2(y1-y2, x1-x2) * 180 / 3.14159265 - 90;
}

float distanceToTarget(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

bool radarValid() {
  return abs(1840-mylidar.width) < RADAR_ERROR && abs(2450-mylidar.height) < RADAR_ERROR;
}

bool radarTrash() {
  return abs(1840-mylidar.width) > RADAR_ERROR && abs(2450-mylidar.height) > RADAR_ERROR;
}

bool radarXValid() {
  return abs(1840-mylidar.width) < RADAR_ERROR;
}

bool radarYValid() {
  return abs(2450-mylidar.height) < RADAR_ERROR;
}

void receiveGyro() {
    if (GyroIn.receiveData()) {
        gyro = gyroData.angle;
        stab = gyroData.stab;
    }
}

void receiveCamera() {
    if (cameraIn.receiveData()) {
        vis = mydata.foundball;
        if (vis) ball = mydata.angle;
    }
}

void receiveRadar() {
    if (radarIn.receiveData()) {
        
        /*if (!radarTrash()) {
            if (radarXValid())
                Serial.print(mylidar.x);
            else
                Serial.print(F("----"));
                
            Serial.print(F(", "));
            
            if (radarYValid())
                Serial.print(mylidar.y);
            else
                Serial.print("----");
                
            Serial.print(F(" | "));
            Serial.print(mylidar.width);
            Serial.print(F(" x "));
            Serial.print(mylidar.height);
            Serial.print(F(" -> "));
            Serial.println(mylidar.angle);
        } else {
            // Serial.println(F("--------------------------------"));
            Serial.print(mylidar.width);
            Serial.print(" - ");
            Serial.println(mylidar.height);
        }*/
        
        
        if (!radarTrash()) {
            if (radarXValid()) {
                xTimer = 0;
                x = mylidar.x;
            }
            if (radarYValid()) {
                yTimer = 0;
                y = mylidar.y;
            }
            width = mylidar.width;
            height = mylidar.height;
        } else {
            // Serial.println("--------------------------------");
        }
        
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
    MotorDriver::setMaxSpeed(200);

    return true;
}

void dead() {
    Serial.println(F("R.I.P Arduino"));

    while (1) continue;
}
