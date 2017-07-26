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

/*
SIG 51
ANA A7
LAS A6
LGT A9-A12
*/

const int I2C_SLAVE_ADDRESS = 9;
const int MIN_RADAR_ERROR = 300;
const int MIN_MOTOR = 130;
const int MAX_MOTOR = 250;
const int MAX_RADAR_ERROR = 800;
const int MIN_POS_TIMEOUT = 100;
const int MAX_POS_TIMEOUT = 1000;
const int FIELD_HEIGHT = 2250;
const int FIELD_WIDTH = 1550;
int BOUNDARY = 400;


int Radar_Error = 100;
int Pos_Timeout = 50;

void receive(int numBytes) {}
void printSim();
void dead();
void debug();

void followBall();
void followGib();
void parkTheBus();

void receiveRadar();
void receiveCamera();
void receiveGyro();
void lineInterrupt();
void goToTarget(int tx, int ty, int maxRadar = MAX_RADAR_ERROR, int maxMotor = 400, int minDistance = 100);

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
    if (!initEverything())
        dead();
    
    while (1) {
        receiveCamera();
        receiveRadar();
        receiveGyro();
        
        // MotorDriver::direction(0);
        
        if (analogRead(A6) < 100) {
            hasBall = true;
            hasBallTimer = 0;
        } else {
            hasBall = false;
        }
        
        if (hasBall) {
            if (hasBallTimer < 300 || y < 700) {
                cameraOutData.target = 3;
            } else {
                cameraOutData.target = 2;
            }
        } else {
            cameraOutData.target = 1;
        }

        if (!stab)  {
            mode = 0;
            debug();
            MotorDriver::stop();
            continue;
        }
        debug();
        // MotorDriver::getMaxSpeed();
        digitalWrite(LED_BUILTIN, HIGH);
        
        if (visionTimer > 1000)
            vis = false;
            
        stop = false;
        
        MotorDriver::update(gyro);
        
        if (kickTimer > 250) {
            digitalWrite(51, LOW);
        }
        /*if (kickTimer > 10000) {
            digitalWrite(51, HIGH);
            if (kickTimer > 10250) {
                kickTimer = 0;
            }
        } else {
            digitalWrite(51, LOW);
        }*/


        if (hitLine)
            BOUNDARY = 450;
        else
            BOUNDARY = 250;
        
        if (!prevHitLine && hitLine) {
            x = 0;
            y = 0;
            xTimer = 9999;
            yTimer = 9999;
            prevHitLine = hitLine;
            Serial.println("HIT LINE");
        }
        
        if (hitLine && (xTimer > 400 || yTimer > 400)) {
            mode = 0;
            stop = true;
        } else if (!inField(x, y) && (xTimer < 400 && yTimer < 400)) {
            mode = 1;
            hitLine = true;
            prevHitLine = true;
            goToTarget(FIELD_WIDTH/2, FIELD_HEIGHT / 2, MAX_RADAR_ERROR, 100);
        } else {
            hitLine = false;
            prevHitLine = false;
        }
        
        if (!hitLine && false) {
            if (hasBall || hasBallTimer < 3000) {
                mode = 2;
                // visionTimer = 0;
                float toOppGoal = 0;
                bool stopped = false;
                if (vis == 2 && false) {
                    toOppGoal = gyro - ball;
                    if (toOppGoal > 180) toOppGoal = toOppGoal-360;
                } else if (xTimer < 300 && yTimer < 300) {
                    toOppGoal = gyro - angleToTarget(x, y, FIELD_WIDTH / 2, 300);
                } else {
                    stopped = true;
                }
                // Serial.println(toOppGoal);
                MotorDriver::update(toOppGoal);
                MotorDriver::setMaxSpeed(175);
                if (!stopped) {
                    if (y < 700 && fabs(toOppGoal) > 30) {
                        desiredDirection = 180;
                        cameraOutData.target = 3;
                    } else {
                        desiredDirection = 0;
                        cameraOutData.target = 2;
                    }
                } else {
                    stop = true;
                }
                
                if (analogRead(A7) > 600) {
                    Serial.println("KICK");
                    digitalWrite(51, HIGH);
                    kickTimer = 0;
                }
            } else if ((vis > 0 || visionTimer < 600)) {
                float toOppGoal = gyro - angleToTarget(x, y, FIELD_WIDTH / 2, 0);
                if (toOppGoal > 180) toOppGoal = toOppGoal-360;
                MotorDriver::update(toOppGoal);
                // Serial.print(angleToTarget(x, y, 900, 0));
                // Serial.print(" - ");
                // Serial.println(toOppGoal);
                Radar_Error = 400;
                if (xTimer < 400 && yTimer < 400 && false) {
                    mode = 3;
                    followGib();
                } else {
                    mode = 4;
                    followBall();
                }
            } else if (yTimer < Pos_Timeout && xTimer < Pos_Timeout) {
                parkTheBus();
            } else {
                stop = true;
                // MotorDriver::stop();
            }
        }
        
        parkTheBus();
        if (stop || !motorsStopped) {
            MotorDriver::stop();
        } else {
            int prevBoundary = BOUNDARY;
            BOUNDARY = 550;
            if (!inField(x, y)) {
                // MotorDriver::setMaxSpeed(MIN_MOTOR);
            }
            BOUNDARY = prevBoundary;
            // MotorDriver::setMaxSpeed(100);
            MotorDriver::direction(desiredDirection, smooth);
        }
        // MotorDriver::direction(0);
    }

    return 0;
}

void parkTheBus() {
    if (y > FIELD_HEIGHT / 3 * 2) {
        goToTarget(FIELD_WIDTH / 2, FIELD_HEIGHT - BOUNDARY - 100, MAX_RADAR_ERROR, 200, 200);
    } else {
        if (x > FIELD_WIDTH / 3 * 2) {
            goToTarget(FIELD_WIDTH - 500, FIELD_HEIGHT - BOUNDARY - 500, MAX_RADAR_ERROR, 400);
        } else {
            goToTarget(500, FIELD_HEIGHT - BOUNDARY - 500, MAX_RADAR_ERROR, 400);
        }
    }
}

void followGib() {
    float angleToGoal = angleToTarget(x, y, 1000, 300) - (ball + gyro);
    if (angleToGoal < -360) angleToGoal += 360;
    if (angleToGoal < -180) angleToGoal = angleToGoal+360;
    
    float motorOutput = MIN_MOTOR + 1.0 * ((MAX_MOTOR - MIN_MOTOR)
                        / (50 - 20)) * (bally - 20);
    if (motorOutput < MIN_MOTOR || cameraOutData.target == 2) motorOutput = MIN_MOTOR;
    else if (motorOutput > MAX_MOTOR) motorOutput = MAX_MOTOR;

    MotorDriver::setMaxSpeed(motorOutput);
    
    // Serial.println(ball - angleToGoal);
    // MotorDriver::direction(ball - angleToGoal);
    desiredDirection = ball - angleToGoal;
}

void followBall() {
    if (vis) {
        visionTimer = 0;
    }
    // MotorDriver::update(gyro - angleToTarget(x, y, 900, 300));
    Radar_Error = 400;
    float motorOutput = MIN_MOTOR + 1.0 * ((MAX_MOTOR - MIN_MOTOR)
                        / (180 - 60)) * (bally - 60);
    if (motorOutput < MIN_MOTOR || cameraOutData.target == 2) motorOutput = MIN_MOTOR;
    else if (motorOutput > MAX_MOTOR) motorOutput = MAX_MOTOR;

    if (abs(ball) < 36) {
        // MotorDriver::direction(0);
        desiredDirection = ball * 1.3;
    } else if (abs(ball) < 90) {
        // MotorDriver::direction(ball * 2.5);
        desiredDirection = ball + 54;
    } else {
        // MotorDriver::direction(ball * 1.5);
        desiredDirection = ball * 1.3;
    }
    /*MotorDriver::setMaxSpeed(motorOutput);
    if (abs(ball) < 20) {
        desiredDirection = 0;
    } else {
        desiredDirection = ball * 1.3;
    }
    
    float distance;
    if (bally < 100) {
        distance = 600 + 1.0 * ((1000)
                            / (100 - 60)) * (bally - 60);
    } else if (bally < 130) {
        distance = 1000 + 1.0 * ((2200 - 1000)
                            / (130 - 100)) * (bally - 100);
    } else {
        distance = 2200 + 1.0 * ((5000 - 2200)
                            / (160 - 130)) * (bally - 130);
    }*/
    /*if (abs(ball) > 36 && abs(ball) < 90) {
        desiredDirection = ball + 54 * (ball / abs(ball));
    } else {
        desiredDirection = ball * 1.3;
    }*/
    
    /*float mod = (160.0f / distance);
    desiredDirection = desiredDirection * fmax(mod, 1.5);*/

    
    // float ballyang = ((static_cast<float>(bally)/200.0f) - 0.5f) * 47;
    // desiredDirection = ballyang;
    // desiredDirection = 12.0f / tan(radians(30 + (90 - ballyang)));
}

void lineInterrupt() {
    hitLine = true;
    lineTimer = 0;
}

bool inField(int x1, int y1) {
    return (x1 > BOUNDARY && x1 < FIELD_WIDTH-BOUNDARY && y1 > BOUNDARY && y1 < FIELD_HEIGHT-BOUNDARY);
}

void goToTarget(int tx, int ty, int maxRadar, int maxMotor, int minDistance) {
    Serial.print(abs(x-tx));
    Serial.print(", ");
    Serial.print(abs(y-ty));
    Serial.print(" : ");
    Serial.println(!(abs(y-ty) > minDistance || abs(x-tx) > minDistance));
    if (!(abs(y-ty) > minDistance || abs(x-tx) > minDistance)) {
        stop = true;
        return;
    }
    
    if (maxRadar < MIN_RADAR_ERROR) maxRadar = MIN_RADAR_ERROR;
    if (maxRadar > MAX_RADAR_ERROR) maxRadar = MAX_RADAR_ERROR;
    if (maxMotor < MIN_MOTOR) maxMotor = MIN_MOTOR;
    // if (maxMotor > MAX_MOTOR) maxMotor = MAX_MOTOR;
    
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

void receiveGyro() {
    if (GyroIn.receiveData()) {
        gyro = gyroData.angle - gyroOffset;
        while (gyro < -360) gyro += 360;
        stab = gyroData.stab;
    }
}

void receiveCamera() {
    if (cameraIn.receiveData()) {
        vis = cameraData.visible;
        motorsStopped = cameraData.motorButton;
        if (cameraData.gyroButton) {
            gyroOffset = gyro + gyroOffset;
        }
        if (vis)  {
            if (!prevVis) {
                prevVis = true;
            } else {
                visionTimer -= 100;
                if (visionTimer > 600) visionTimer = 600;
                ball = cameraData.angle;
                bally = cameraData.bally;
            }
        } else {
            prevVis = false;
        }
        cameraOutData.target = 4;  // TODO(andy): remove the shit out of this
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
        Serial.print(mylidar.width);
        Serial.print(", ");
        Serial.println(mylidar.height);
        if (!radarTrash()) {
            // Serial.print(xTimer);
            // Serial.print(", ");
            // Serial.println(yTimer);
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
        } else {
            /*Serial.println("--------------------------------");
            Serial.print(mylidar.width);
            Serial.print(", ");
            Serial.println(mylidar.height);
            Serial.println("--------------------------------");*/
        }
    }
}

void debug() {
    return;
    if (serialTimer > 30) {
        /*Serial.print(hitLine);
        Serial.print(" -> ");
        Serial.print(xTimer);
        Serial.print(", ");
        Serial.print(yTimer);*/
        
        // printSim();
        Serial.print(bally);
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
        Serial.print(desiredDirection);
        Serial.print(" @ ");
        Serial.print(MotorDriver::getMaxSpeed());
        
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
