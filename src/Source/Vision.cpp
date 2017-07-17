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
            angle = ((static_cast<float>(x)/319.0f) - 0.5f) * 75;
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
        // pixy.setBrightness(255);
        // pixy.setLED(0, 0, 0);
        
        AFMS.begin();
        
        myMotor->setSpeed(4);
        
        return true;
    }

    void update() {
        // if (totalstep != 0) totalstep = fmin(fabs(totalstep), limit) * (totalstep/fabs(totalstep));
        updateView();
    }
    
    void updateMotor() {
      if (millis() - lastSeenBall > 50) {
            findBall();
        } else if (ballFound && !(fabs(totalstep) > limit-1)) {
            lookAtBall();
            ballFound = false;
        }
        // Serial.println(totalstep);
    }

    void updateView() {
        uint16_t blocks;

        blocks = pixy.getBlocks();
        
        if (blocks) {
            for (int i = 0; i < blocks; i++) {
                if (pixy.blocks[i].signature == 1) {
                    // if (pixy.blocks[i].width * pixy.blocks[i].height < 100) continue;
                    ball.x = pixy.blocks[i].x;
                    ball.y = pixy.blocks[i].y;
                    ball.w = pixy.blocks[i].width;
                    ball.h = pixy.blocks[i].height;
                    ball.setAngle();

                    ballFound = true;
                    lastSeenBall = millis();
                }
            }
        }
    }
    
    void findBall() {
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
        dirmod = 1;
        searching = false;
        int type;
        if (fabs(ball.getAngle()) < 10) return;
        if (fabs(ball.getAngle()) < 18)
            type = INTERLEAVE;
        else
            type = SINGLE;
        
        if (ball.getAngle() > 0) {
            myMotor->step(1, BACKWARD, type);
            if (type == INTERLEAVE) totalstep -= 0.5;
            else
            totalstep -= 1;
        } else {
            myMotor->step(1, FORWARD, type);
            if (type == INTERLEAVE) totalstep += 0.5;
            else
            totalstep += 1;
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
