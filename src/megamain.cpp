#include <Arduino.h>

#include <string>

#include "Source/MotorDriver.h"
#include "Source/Orientation.h"
#include "DualMC33926MotorShield.h"

#include <EasyTransfer.h>
#include <Wire.h>
#include <EasyTransferI2C.h>
#include <elapsedMillis.h>

struct RECEIVE_DATA_STRUCTURE_LIDAR {
    int x;
    int y;
    int width;
    int height;
    float angle;
};

struct RECEIVE_DATA_STRUCTURE_CAMERA {
    float angle;
    bool foundball;
};

struct RECEIVE_DATA_STRUCTURE_GYRO {
    float angle;
    bool stab;
};

const int I2C_SLAVE_ADDRESS = 9;
const int MIN_RADAR_ERROR = 50;
const int MIN_MOTOR = 100;
const int MAX_MOTOR = 400;
const int MAX_RADAR_ERROR = 800;
const int MIN_POS_TIMEOUT = 200;
const int MAX_POS_TIMEOUT = 1000;


int Radar_Error = 100;
int Pos_Timeout = 1000;

void receive(int numBytes) {}
void printSim();
void dead();

void receiveRadar();
void receiveCamera();
void receiveGyro();
void goToTarget(int tx, int ty, int maxRadar = MAX_RADAR_ERROR, int maxMotor = MAX_MOTOR, int minDistance = 100);

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

DualMC33926MotorShield tp(7, 11, A8, 8, 12, A1, 4, 0);
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
std::string debug;  // NOLINT

int main() {
    if (!initEverything())
        dead();
    
    while (1) {
        receiveCamera();
        receiveRadar();
        receiveGyro();
        
        if (visionTimer > 1000) vis = false;
        
        MotorDriver::update(gyro);
        
        debug = "";
        
        mode = -1;
        if (stab) {
            if ((vis || visionTimer < 100)) {
                mode = 0;
                if (vis) {
                    visionTimer = 0;
                }
                // MotorDriver::update(gyro - angleToTarget(x, y, 900, 300));
                MotorDriver::setMaxSpeed(200);  // HACK - hardcoded
                if (abs(ball) < 20) {
                    MotorDriver::direction(0);
                } else if (abs(ball) < 45) {
                    MotorDriver::direction(ball * 2.5);
                } else {
                    MotorDriver::direction(ball * 1.5);
                }
                
            } else if (yTimer < Pos_Timeout && xTimer < Pos_Timeout) {
                mode = 1;
                // MotorDriver::update(gyro);
                if (y > 1300) {
                    goToTarget(900, 2000, MAX_RADAR_ERROR, MAX_MOTOR / 1.5, 100);
                    // goToTarget(900, 2000);
                } else {
                    if (x > 900) {
                        goToTarget(1400, 1600);
                    } else {
                        goToTarget(600, 1600);
                    }
                }
            } else {
                MotorDriver::stop();
            }
        }
        
        if (xTimer < 300) {
            if (x < 400) {
                debug = "OOB Left";
            } else if (x > 1400) {
                debug = "OOB Right";
            }
        }
        if (yTimer < 300) {
            if (y < 400) {
                debug = "OOB Top";
            } else if (y > 2000) {
                debug = "OOB bottom";
            }
        }
        
        if (mode == -1) {
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
                Serial.print(gyro);
            else {
                Serial.print("---");
                Serial.print(gyro);
            }
                
            
            if (debug != "") {
                Serial.print(" : ");
                Serial.print(debug.c_str());
            }
            
            Serial.println();
                
            serialTimer = 0;
        }
    }

    return 0;
}

void goToTarget(int tx, int ty, int maxRadar, int maxMotor, int minDistance) {
    if (!(abs(y-ty) > minDistance || abs(x-tx) > minDistance)) {
        MotorDriver::stop();
        return;
    }
    
    if (maxRadar < MIN_RADAR_ERROR) maxRadar = MIN_RADAR_ERROR;
    if (maxRadar > MAX_RADAR_ERROR) maxRadar = MAX_RADAR_ERROR;
    if (maxMotor < MIN_MOTOR) maxMotor = MIN_MOTOR;
    if (maxMotor > MAX_MOTOR) maxMotor = MAX_MOTOR;
    
    float distance = distanceToTarget(x, y, tx, ty);
    
    float radarOutput = MIN_RADAR_ERROR + 1.0 * ((maxRadar - MIN_RADAR_ERROR)
                        / (1500.0f - minDistance)) * (distance - minDistance);
    float motorOutput = MIN_MOTOR + 1.0 * ((maxMotor - MIN_MOTOR)
                        / (1500.0f - minDistance)) * (distance - minDistance);
    Pos_Timeout = MIN_POS_TIMEOUT + 1.0 * ((MAX_POS_TIMEOUT - MIN_POS_TIMEOUT)
                        / (1500.0f - minDistance)) * (distance - minDistance);
    
    /*Serial.print("Radar: ");
    Serial.print(radarOutput);
    Serial.print(" Motor: ");
    Serial.println(motorOutput);*/
    Radar_Error = radarOutput;
    MotorDriver::direction(angleToTarget(x, y, tx, ty), false);
    MotorDriver::setMaxSpeed(motorOutput);
}

float angleToTarget(int x1, int y1, int x2, int y2) {
    return atan2(y1-y2, x1-x2) * 180 / 3.14159265 - 90;
}

float distanceToTarget(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

bool radarValid() {
  return abs(1840-mylidar.width) < Radar_Error && abs(2450-mylidar.height) < Radar_Error;
}

bool radarTrash() {
  return abs(1840-mylidar.width) > Radar_Error && abs(2450-mylidar.height) > Radar_Error;
}

bool radarXValid() {
  return abs(1840-mylidar.width) < Radar_Error;
}

bool radarYValid() {
  return abs(2450-mylidar.height) < Radar_Error;
}

void receiveGyro() {
    if (GyroIn.receiveData()) {
        gyro = gyroData.angle;
        stab = gyroData.stab;
    }
}

void receiveCamera() {
    if (cameraIn.receiveData()) {
        // Serial.println("Camera");
        
        vis = mydata.foundball;
        if (vis)  {
            visionTimer = 0;
            ball = mydata.angle;
        }
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
        
        mode = Pos_Timeout;
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
    
    pinMode(45, OUTPUT);
    pinMode(A8, INPUT);
    digitalWrite(45, HIGH);

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
