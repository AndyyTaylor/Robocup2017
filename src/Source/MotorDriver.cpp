#include "MotorDriver.h"

#include <Arduino.h>
#include <elapsedMillis.h>

#include "DualMC33926MotorShield.h"

namespace MotorDriver {
    DualMC33926MotorShield tp(7, 11, A0, 8, 12, A1, 4, 0);
    DualMC33926MotorShield bt(33, 5, A2, 32, 2, A3, 30, 31);
    
    elapsedMillis speedTimer;

    int maxspeed = 400;
    int curSpeed = 0;
    int prevSpeed = 0;
    double curOrientation = 0;
    
    double curAngle = 0;
    double lastChange = 0;

    void update(double orientation) {
        curOrientation = orientation;
    }

    void init() {
        tp.init();
        bt.init();
    }

    void setMaxSpeed(int speed) {
        maxspeed = speed;
    }

    void stop() {
        bt.setM1Speed(0);
        bt.setM2Speed(0);
        tp.setM1Speed(0);
        tp.setM2Speed(0);
    }

    void direction(double inangle, bool smooth) {
        if (smooth) {
            double angledifference = inangle - curAngle;
            if (abs(angledifference) > 0 && millis() - lastChange > 3) {
                curAngle += abs(angledifference)/angledifference*2;
                lastChange = millis();
            }
        } else {
            curAngle = inangle;
        }
        
        
        /*Serial.print("Current Angle: ");
        Serial.print(curAngle);
        Serial.print(" | Input Angle: ");
        Serial.println(inangle);*/
        inangle = curAngle/180.0f*PI;
        double m2 = cos(inangle+PI/4) * maxspeed;
        double m1 = cos(inangle-PI/4) * maxspeed;

        bt.setM1Speed(correct(-m1));
        bt.setM2Speed(correct(-m2));
        tp.setM1Speed(correct(m1));
        tp.setM2Speed(correct(m2));
    }
    
    int getMaxSpeed() {
        /*Serial.print(speedTimer);
        Serial.print(" : ");
        Serial.print(curSpeed);
        Serial.print(" -> ");
        Serial.print(maxspeed);
        Serial.print(" (");
        Serial.print(maxspeed-prevSpeed);
        Serial.println(")");*/
        return maxspeed;
    }

    double correct(double speed) {
      double newOrientation = curOrientation;
      if (curOrientation > 180) {newOrientation -=360;}
      return max(-400, min(400, speed - newOrientation));
    }
    
    double relativeAngle(double input) {
        while (input > 180) {input -= 360;}
        while (input < -180) {input +=360;}
        return input;
    }
}   // namespace MotorDriver
