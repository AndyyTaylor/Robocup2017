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
    int visible;
    bool motorButton;
    bool gyroButton;
};

struct RECEIVE_DATA_STRUCTURE_GYRO {
    float angle;
    bool stab;
};

struct SEND_DATA_STRUCTURE_CAMERA {
    int target;
};

const int I2C_SLAVE_ADDRESS = 9;
const int MIN_RADAR_ERROR = 200;
const int MIN_MOTOR = 130;
const int MAX_MOTOR = 250;
const int MAX_RADAR_ERROR = 800;
const int MIN_POS_TIMEOUT = 300;
const int MAX_POS_TIMEOUT = 500;
const int FIELD_HEIGHT = 2200;
const int FIELD_WIDTH = 1550;
int BOUNDARY = 400;


int Radar_Error = 100;
int Pos_Timeout = 50;

void receive(int numBytes) {}
void receiveRadar();
void receiveCamera();
void receiveGyro();
void lineInterrupt();
void setupVars();
void followBall();
void debug();
void goToTarget(int tx, int ty, int maxRadar = MAX_RADAR_ERROR, int maxMotor = 400, int minDistance = 100);
void parkTheBus();

float angleToTarget(int x1, int y1, int x2, int y2);
float distanceToTarget(int x1, int y1, int x2, int y2);

bool radarValid();
bool radarTrash();
bool radarXValid();
bool radarYValid();
bool inField(int x1, int y1);
bool initEverything();

void dead();

RECEIVE_DATA_STRUCTURE_LIDAR mylidar;
RECEIVE_DATA_STRUCTURE_CAMERA cameraData;
RECEIVE_DATA_STRUCTURE_GYRO gyroData;
SEND_DATA_STRUCTURE_CAMERA cameraOutData;

EasyTransfer radarIn;
EasyTransfer GyroIn;
EasyTransfer cameraIn;
EasyTransfer cameraOut;

elapsedMillis serialTimer;
elapsedMillis visionTimer;
elapsedMillis yTimer = 9999;
elapsedMillis xTimer = 9999;
elapsedMillis kickTimer;
elapsedMillis lineTimer;
elapsedMillis hangTimer;
elapsedMillis hasBallTimer = 9999;

int x, y, width, height, ball, bally, gyro, vis, stab;
bool hasBall = false;
bool hitLine = false;
bool prevHitLine = false;
bool motorsStopped = false;
int gyroOffset = 0;

bool stop = false;
int desiredDirection = 0;
bool smooth = true;
int mode = -1;
bool prevVis = false;

int main() {
    if (!initEverything()) {
        dead();
    }
    
    while (1) {
        setupVars();
        
        if (!stab) {
            mode = 0;
            debug();
            MotorDriver::stop();
            continue;
        }
        
        MotorDriver::update(gyro);
        
        digitalWrite(LED_BUILTIN, HIGH);
        
        if (hitLine && !prevHitLine) {
            prevHitLine = true;
            xTimer = 9999;
            yTimer = 9999;
            stop = true;
            mode = 4;
            hitLine = false;
        } else if ((hitLine || lineTimer < 1000) && (xTimer > 400 || yTimer > 400)) {
            stop = true;
            mode = 4;
        } else if (!inField(x, y) && (xTimer < 100 && yTimer < 100)) {
            mode = 4;
            goToTarget(800, 1500, MAX_RADAR_ERROR, 150);
            hitLine = true;
            prevHitLine = true;
            mode = 4;
        } else if (inField(x, y) && (xTimer < 400 && yTimer < 400)) {
            hitLine = false;
            prevHitLine = false;
            mode = 4;
        }
        
        if (!hitLine && lineTimer > 300) {
            if (hasBall && vis == 0) {
                mode = 7;
                if (y > 800 || yTimer > 1000) {
                    cameraOutData.target = 2;
                } else {
                    cameraOutData.target = 3;
                }
                // MotorDriver::update(gyro - angleToTarget(x, y, FIELD_WIDTH / 2, 300));
                MotorDriver::setMaxSpeed(200);
                desiredDirection = 0;
            } else if ((vis > 0 || visionTimer < 800)) {
                mode = 1;
                followBall();
                Pos_Timeout = 500;
            } else if (xTimer < 1000 && yTimer < 1000) {
                mode = 2;
                parkTheBus();
            } else {
                mode = -1;
                stop = true;
            }
        }
        
        desiredDirection = 90;
        if (!cameraData.motorButton) {
            if (stop)
                mode = -2;
            else
                mode = -3;
                
            MotorDriver::stop();
        } else {
            MotorDriver::direction(desiredDirection, smooth);
        }
        
        debug();
    }
}

void followBall() {
    MotorDriver::setMaxSpeed(200);

    if (abs(ball) < 20) {
        desiredDirection = 0;
    } else if (abs(ball) < 75) {
        desiredDirection = ball * 2.5;
    } else {
        desiredDirection = ball * 1.3;
    }
    
    stop = false;
}

void parkTheBus() {
    /*int bound = 600;
    int num = (millis() / 1000) % 100 / 10;
    while (num > 2) { num -= 3; }
    if (num == 0) {
        goToTarget(FIELD_WIDTH/2, FIELD_HEIGHT/2, MAX_RADAR_ERROR, 400, 150);
    } else if (num == 1) {
        goToTarget(bound, bound, MAX_RADAR_ERROR, 400, 150);
    } else if (num == 2) {
        goToTarget(FIELD_WIDTH - bound, bound, MAX_RADAR_ERROR, 400, 150);
    } else {
        goToTarget(FIELD_WIDTH - bound, bound, MAX_RADAR_ERROR, 400, 150);
    }*/
    // goToTarget(FIELD_WIDTH/2, FIELD_HEIGHT/2, MAX_RADAR_ERROR, 400, 150);
    // goToTarget(800, 1300, MAX_RADAR_ERROR, 400, 150);
    if (y > FIELD_HEIGHT / 5 * 3) {
        goToTarget(800, 1300, MAX_RADAR_ERROR, 400, 150);
    } else {
        if (x > FIELD_WIDTH / 3 * 2) {
            goToTarget(FIELD_WIDTH - 500, FIELD_HEIGHT - 200, MAX_RADAR_ERROR, 400);
        } else {
            goToTarget(500, FIELD_HEIGHT - 200, MAX_RADAR_ERROR, 400);
        }
    }
}

void goToTarget(int tx, int ty, int maxRadar, int maxMotor, int minDistance) {
    if (!(abs(y-ty) > minDistance || abs(x-tx) > minDistance)) {
        stop = true;
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
    desiredDirection = angleToTarget(x, y, tx, ty);
    smooth = false;
    // MotorDriver::direction(angleToTarget(x, y, tx, ty), false);
    MotorDriver::setMaxSpeed(motorOutput);
}

bool inField(int x1, int y1) {
    return (x1 > BOUNDARY && x1 < FIELD_WIDTH-BOUNDARY && y1 > BOUNDARY && y1 < FIELD_HEIGHT-BOUNDARY);
}

float angleToTarget(int x1, int y1, int x2, int y2) {
    return atan2(y1-y2, x1-x2) * 180 / 3.14159265 - 90;
}

float distanceToTarget(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

bool radarValid() {
  return abs(FIELD_WIDTH-mylidar.width) < Radar_Error && abs(FIELD_HEIGHT-mylidar.height) < Radar_Error;
}

bool radarTrash() {
  return abs(FIELD_WIDTH-mylidar.width) > Radar_Error && abs(FIELD_HEIGHT-mylidar.height) > Radar_Error;
}

bool radarXValid() {
  return abs(FIELD_WIDTH-mylidar.width) < Radar_Error;
}

bool radarYValid() {
  return abs(FIELD_HEIGHT-mylidar.height) < Radar_Error;
}

void lineInterrupt() {
    hitLine = true;
    lineTimer = 0;
    Serial.println("Hit a fucking line");
}

void receiveGyro() {
    if (GyroIn.receiveData()) {
        gyro = gyroData.angle - gyroOffset;
        while (gyro < -360) gyro += 360;
        stab = gyroData.stab;
    }
}

void setupVars() {
    receiveCamera();
    receiveRadar();
    receiveGyro();
    
    cameraOutData.target = 1;
    
    if (analogRead(A6) < 100) {
        hasBall = true;
        hasBallTimer = 0;
    } else {
        hasBall = false;
    }
    
    if (hitLine || lineTimer < 300) {
        BOUNDARY = 400;
    } else {
        BOUNDARY = 300;
    }
    
    stop = false;
    smooth = true;
}

void receiveCamera() {
    if (cameraIn.receiveData()) {
        vis = cameraData.visible;
        motorsStopped = cameraData.motorButton;
        if (cameraData.gyroButton) {
            gyroOffset = gyro + gyroOffset;
        }
        if (vis)  {
            visionTimer -= 100;
            if (visionTimer > 800) visionTimer = 800;
            ball = cameraData.angle;
            bally = cameraData.bally;
        }
        cameraOut.sendData();
    }
}

void receiveRadar() {
    if (radarIn.receiveData()) {
        Serial.print(mylidar.width);
        Serial.print(", ");
        Serial.println(mylidar.height);
        if (!radarTrash()) {
            if (radarXValid() && (abs(x-mylidar.x) < 300 || xTimer > 800)) {
                xTimer = 0;
                x = mylidar.x;
            }
            if (radarYValid() && (abs(y-mylidar.y) < 300 || yTimer > 800)) {
                yTimer = 0;
                y = mylidar.y;
            }
            width = mylidar.width;
            height = mylidar.height;
        }
    }
}

bool initEverything() {
    init();
    
    pinMode(45, OUTPUT);
    pinMode(A6, INPUT);
    pinMode(A7, INPUT);
    pinMode(A9, INPUT);
    pinMode(A10, INPUT);
    pinMode(A11, INPUT);
    pinMode(A12, INPUT);
    digitalWrite(45, HIGH);
    pinMode(51, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    
    digitalWrite(LED_BUILTIN, LOW);
    attachInterrupt(digitalPinToInterrupt(20), lineInterrupt, RISING);
    
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

void debug() {
    if (serialTimer > 30) {
        /*Serial.print(hitLine);
        Serial.print(" -> ");
        Serial.print(xTimer);
        Serial.print(", ");
        Serial.print(yTimer);*/
        
        // printSim();
        Serial.print(mode);
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
            Serial.print(" ball ");
        else
            Serial.print(" ---- ");
        
        Serial.print(MotorDriver::getMaxSpeed());
        Serial.print(" --- ");
        
        int num = (millis() / 1000) % 100 / 10;
        while (num > 3) { num -= 3; }
        Serial.print(desiredDirection);
        
        Serial.println();
            
        serialTimer = 0;
    }
}
