#include <Arduino.h>

#include <math.h>
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
    int bally;
    int foundball;
};

struct RECEIVE_DATA_STRUCTURE_GYRO {
    float angle;
    bool stab;
};

struct SEND_DATA_STRUCTURE_CAMERA {
    int target;
};

/*
SIG 36
ANA A7
LAS A8
LGT A9-A12
*/

const int I2C_SLAVE_ADDRESS = 9;
const int MIN_RADAR_ERROR = 300;
const int MIN_MOTOR = 100;
const int MAX_MOTOR = 400;
const int MAX_RADAR_ERROR = 800;
const int MIN_POS_TIMEOUT = 200;
const int MAX_POS_TIMEOUT = 1000;
const int BOUNDARY = 550;


int Radar_Error = 100;
int Pos_Timeout = 400;

void receive(int numBytes) {}
void printSim();
void dead();

void followBall();
void parkTheBus();

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
bool inField(int x1, int y1);
bool initEverything();


RECEIVE_DATA_STRUCTURE_LIDAR mylidar;
RECEIVE_DATA_STRUCTURE_CAMERA cameraData;
RECEIVE_DATA_STRUCTURE_GYRO gyroData;
SEND_DATA_STRUCTURE_CAMERA cameraOutData;

DualMC33926MotorShield tp(7, 11, A8, 8, 12, A1, 4, 0);
DualMC33926MotorShield bt(14, 5, A2, 15, 2, A3, 17, 16);

EasyTransfer radarIn;
EasyTransfer GyroIn;
EasyTransfer cameraIn;
EasyTransfer cameraOut;

elapsedMillis serialTimer;
elapsedMillis visionTimer;
elapsedMillis yTimer = 9999;
elapsedMillis xTimer = 9999;
elapsedMillis kickTimer;

int x, y, width, height, ball, gyro, vis, stab;
bool hasBall;
float mode = 0;
float desiredDirection;
int bally = 0;
bool stop = false;
bool smooth = true;
bool hitLine = false;
std::string debug;  // NOLINT

int main() {
    if (!initEverything())
        dead();
    
    while (1) {
        receiveCamera();
        receiveRadar();
        receiveGyro();
        
        if (visionTimer > 1000) vis = false;
        desiredDirection = 0;
        stop = false;
        smooth = true;
        
        if (analogRead(A8) < 50)
            hasBall = true;
        else
            hasBall = false;
        
        float toOppGoal = angleToTarget(x, y, 900, 300);
        MotorDriver::update(gyro - toOppGoal);
        
        debug = "";
        
        cameraOutData.target = 1;
        if (hasBall) {
            cameraOutData.target = 2;
        }
        
        if (kickTimer > 80) {
            digitalWrite(36, LOW);
        }
        // Serial.println(angleToTarget(x, y, 900, 300));
        mode = -1;
        if (stab) {
            if ((analogRead(A9) > 300 || analogRead(A10) > 200) && !hitLine) {
                hitLine = true;
                x = 0;
                y = 0;
                xTimer = 9999;
                yTimer = 9999;
            } else if (hitLine && (xTimer > 1000 || yTimer > 1000)) {
                MotorDriver::stop();
            } else if (!inField(x, y)) {
                hitLine = true;
                mode = 3;
                goToTarget(800, 1000, MAX_RADAR_ERROR, 200);
            } else {
                hitLine = false;
            }
            if (hitLine) {
            } else if (hasBall) {
                mode = 2;
                digitalWrite(36, HIGH);
                kickTimer = 0;
                if (vis == 2) {
                    MotorDriver::update(gyro - ball);
                    if (abs(gyro - ball) < 5) {
                        digitalWrite(36, HIGH);
                        kickTimer = 0;
                    }
                } else {
                    MotorDriver::stop();
                }
            } else if ((vis || visionTimer < 600)) {
                followBall();
            } else if (yTimer < Pos_Timeout && xTimer < Pos_Timeout) {
                parkTheBus();
            } else {
                stop = true;
                // MotorDriver::stop();
            }
        }
        if (mode == -1) {
            stop = true;
            // MotorDriver::stop();
        }
        
        if (stop) {
            MotorDriver::stop();
            // Serial.println(gyro);
        } else if (mode == 0) {
            // desiredDirection = ball;
            // serialTimer = 0;
            int direction = -1;
            if ((desiredDirection < 0 && desiredDirection > -90) || (desiredDirection > 90 && desiredDirection < 180)) {
                direction = 1;
            }
            float ngyro = gyro;
            if (ngyro > 180) {
                ngyro = -(360-gyro);
            }
            float goalDirection = desiredDirection+ngyro;
            
            int dist = 200;
            
            float newX = x + sin(radians(goalDirection)) * dist;
            float newY = y + cos(radians(goalDirection)) * dist;
            
            // Serial.print(x);
            // Serial.print(", ");
            // Serial.print(y);
            // Serial.print(" -> ");
            
            // Serial.print(newX);
            // Serial.print(", ");
            // Serial.print(newY);
            // Serial.print(" | ");
            // Serial.print(goalDirection);
            
            int runs = 0;
            while (!inField(newX, 1000) && runs < 300) {
                runs += 1;
                goalDirection += 1*direction;
                newX = x + sin(radians(goalDirection)) * dist;
                newY = y + cos(radians(goalDirection)) * dist;
            }
            
            // Serial.print(desiredDirection);
            // Serial.print(" -> ");
            // Serial.println(goalDirection - ngyro);
            
            
            if (stop) {
                MotorDriver::stop();
            } else {
                MotorDriver::direction(goalDirection - ngyro, smooth);
            }
            
        }
        
        
        
        if (serialTimer > 50) {
            Serial.print(hitLine);
            Serial.print(" -> ");
            Serial.print(xTimer);
            Serial.print(", ");
            Serial.print(yTimer);
            
            // printSim();
            /*Serial.print(bally);
            Serial.print(" -> ");
            Serial.print(x);
            Serial.print(", ");
            Serial.print(y);
            Serial.print(" | ");
            
            Serial.print(vis);
            
            Serial.print(" | ");
            
            if (vis)
                Serial.print(ball);
            else
                Serial.print("---");
                
            Serial.print(" - ");
            
            if (stab) {
                Serial.print(gyro);
            } else {
                Serial.print("---");
                Serial.print(gyro);
            }
            
            if (hasBall)
                Serial.print(" ball");
            else
                Serial.print(" ----");
            
            Serial.print("  ");
            Serial.print(analogRead(A7));
            
            if (debug != "") {
                Serial.print(" : ");
                Serial.print(debug.c_str());
            }*/
            
            /*Serial.print("A9: ");
            Serial.println(analogRead(A9));
            Serial.print("A10: ");
            Serial.println(analogRead(A10));
            Serial.print("A11: ");
            Serial.println(analogRead(A11));
            Serial.print("A12: ");
            Serial.println(analogRead(A12));*/
            
            Serial.println();
                
            serialTimer = 0;
        }
    }

    return 0;
}

void parkTheBus() {
    mode = 1;
    // MotorDriver::update(gyro);
    if (y > 1300) {
        goToTarget(900, 2000, MAX_RADAR_ERROR, MAX_MOTOR / 1.5, 200);
        // goToTarget(900, 2000);
    } else {
        if (x > 900) {
            goToTarget(1400, 1600);
        } else {
            goToTarget(600, 1600);
        }
    }
}

void followBall() {
    mode = 0;
    if (vis) {
        visionTimer = 0;
    }
    // MotorDriver::update(gyro - angleToTarget(x, y, 900, 300));
    Radar_Error = 400;
    float motorOutput = MIN_MOTOR + 1.0 * ((200 - MIN_MOTOR)
                        / (50 - 20)) * (bally - 20);
    if (motorOutput < MIN_MOTOR || cameraOutData.target == 2) motorOutput = MIN_MOTOR;

    MotorDriver::setMaxSpeed(motorOutput);  
    if (abs(ball) < 20) {
        // MotorDriver::direction(0);
        desiredDirection = 0;
    } else if (abs(ball) < 45) {
        // MotorDriver::direction(ball * 2.5);
        desiredDirection = ball * 2.5;
    } else {
        // MotorDriver::direction(ball * 1.5);
        desiredDirection = ball * 1.5;
    }
}

bool inField(int x1, int y1) {
    return (x1 > BOUNDARY && x1 < 1840-BOUNDARY && y1 > BOUNDARY && y1 < 2450-BOUNDARY);
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
    // desiredDirection = angleToTarget(x, y, tx, ty);
    smooth = false;
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
        vis = cameraData.foundball;
        if (vis)  {
            visionTimer = 0;
            ball = cameraData.angle;
            bally = cameraData.bally;
        }
        cameraOut.sendData();
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
            if (radarXValid() && (abs(x-mylidar.x) < 300 || xTimer > 500)) {
                xTimer = 0;
                x = mylidar.x;
            }
            if (radarYValid() && (abs(y-mylidar.y) < 300 || yTimer > 300)) {
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
    pinMode(A7, INPUT);
    pinMode(A9, INPUT);
    pinMode(A10, INPUT);
    pinMode(A11, INPUT);
    pinMode(A12, INPUT);
    digitalWrite(45, HIGH);
    pinMode(36, OUTPUT);

    Serial.begin(115200);
    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
    Serial.println(F("Initializing"));

    Wire.begin(I2C_SLAVE_ADDRESS);
    cameraOut.begin(details(cameraOutData), &Serial1);
    cameraIn.begin(details(cameraData), &Serial1);
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
