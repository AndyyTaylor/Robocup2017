#include "Vision.h"

#include <Arduino.h>
#include <SPI.h>
#include <Pixy.h>

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

namespace Vision {
    struct Ball {
        int x, y, w, h;
        float angle;

        void setAngle() {
            angle = (((float) x/319.0f) - 0.5f) * 75;
        }

        float getAngle(){
            return angle;
        }
    };

    #define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)
    #define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

    Adafruit_MotorShield AFMS = Adafruit_MotorShield();

    Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2);

    Pixy pixy;
    Ball ball;
    long lastSeenBall;
    
    int dirmod = 1;
    int limit = 20;
    float totalstep = 0;
    bool canFlipDir = true;
    bool ballFound = false;
    bool searching = false;

    bool init() {
        pixy.init();
        
        AFMS.begin();
        
        myMotor->setSpeed(10);
        
        return true;
    }

    void update() {
        // if (totalstep != 0) totalstep = fmin(fabs(totalstep), limit) * (totalstep/fabs(totalstep));
        updateView();
        //return;
        //myMotor->step(50, FORWARD, DOUBLE);
        //delay(100);
        //myMotor->step(50, BACKWARD, DOUBLE);
    }
    
    void updateMotor() {
      if (millis() - lastSeenBall > 50) {
            findBall();
        } else if (ballFound && !(fabs(totalstep) > limit-1)) {
            lookAtBall();
            ballFound = false;
        }
    }

    void updateView() {
        uint16_t blocks;
        char buf[32];
        // Serial.println(ball.x);
        blocks = pixy.getBlocks();
        
        if (blocks) {
            for (int i = 0; i < blocks; i++) {
                // Serial.println(pixy.blocks[i].signature);
                if (pixy.blocks[i].signature == 1) {
                    ball.x = pixy.blocks[i].x;
                    ball.y = pixy.blocks[i].y;
                    ball.w = pixy.blocks[i].width;
                    ball.h = pixy.blocks[i].height;
                    ball.setAngle();
                    // lookAtBall();
                    ballFound = true;
                    lastSeenBall = millis();
                }
            }
        }
    }
    
    void findBall() {
        Serial.print("Finding: ");
        Serial.print(getBallAngle());
        Serial.print(", ");
        Serial.println(dirmod * ball.getAngle());
        /*if (fabs(totalstep) > limit-1 && canFlipDir) {
            dirmod *= -1;
            canFlipDir = false;
        } else if (fabs(totalstep) < limit-4) {
            canFlipDir = true;
        }*/
        if (totalstep > limit-1) {
            searching = true;
            dirmod = -1;
        } else if (totalstep < -(limit-1)) {
            searching = true;
            dirmod = 1;
        } else if (!searching && ball.getAngle() > 0) {
            myMotor->step(1, BACKWARD, SINGLE);
            totalstep -= 0.5*2;
        } else if (!searching) {
            myMotor->step(1, FORWARD, SINGLE);
            totalstep += 0.5*2;
        }
        
        if (searching && dirmod == 1) {
            myMotor->step(1, FORWARD, SINGLE);
            totalstep += 0.5*2;
        } else if (searching) {
            myMotor->step(1, BACKWARD, SINGLE);
            totalstep -= 0.5*2;
        }
    }

    void lookAtBall() {
        Serial.print("Looking: ");
        Serial.println(getBallAngle());
        dirmod = 1;
        searching = false;
        int type = INTERLEAVE;
        if (fabs(ball.getAngle()) < 12) return;
        if (fabs(ball.getAngle()) < 25) type = INTERLEAVE;
        if (ball.getAngle() > 0) {
            myMotor->step(1, BACKWARD, type);
            if (type == INTERLEAVE) totalstep -= 0.5;
            else
            totalstep -= 0.5;
        } else {
            myMotor->step(1, FORWARD, type);
            if (type == INTERLEAVE) totalstep += 0.5;
            else
            totalstep += 0.5;
        }
    }

    float getBallAngle() {
        // Serial.println(totalstep);
        // Serial.print(" : ");
        return -ball.getAngle() + totalstep * (180/22);
    }
    
    bool isVisible() {
        return millis() - lastSeenBall < 50;
    }
}   // namespace Vision
