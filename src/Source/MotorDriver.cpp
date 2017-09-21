#include "MotorDriver.h"

#include <Arduino.h>
#include <elapsedMillis.h>

#include <Teensy_MotorShield.h>

namespace MotorDriver {
    //(7, 9, A0, 8, 10, A1, 4, 12)
    Teensy_MotorShield tp(8, 11, 12, 9, 10, 24);
    Teensy_MotorShield bt(7, 6, 5, 4, 3, 2);

    elapsedMillis speedTimer;

    double motorSpeed = 250;
    int maxSpeed = 300;
    int curSpeed = 0;
    int prevSpeed = 0;
    double curOrientation = 0;

    bool triggeredLines[] = {false, false, false, false};

    double curAngle = 0;
    double lastChange = 0;

    void update(double orientation, bool lineTest[4]) {
        curOrientation = orientation;

        for (int i = 0; i < 4; i ++) {
          triggeredLines[i] = lineTest[i];
        }
    }

    void init() {
        tp.init();
        bt.init();
    }

    void setMaxSpeed(int speed) {
        maxSpeed = speed;
    }

    void stop(bool fix) {
        int speed = 0;
        if (fix) {
          speed = correct(speed)*2;
        }
        bt.setM1Speed(speed);
        bt.setM2Speed(speed);
        tp.setM1Speed(speed);
        tp.setM2Speed(speed);
    }

    void direction(double inangle, bool out, bool smooth) {
        if (smooth) {
            double angledifference = inangle - curAngle;
            if (abs(angledifference) > 6 && millis() - lastChange > 1) {
                curAngle += abs(angledifference)/angledifference*6;
                lastChange = millis();
                motorSpeed = 200;
            }
            else if (motorSpeed < maxSpeed) {
              motorSpeed += 5;
            }
        } else {
            curAngle = inangle;
        }

        if (out) {
          motorSpeed = 200;
        }

        inangle = curAngle/180.0f*PI;

        double vectorX = motorSpeed*sin(inangle);
        double vectorY = motorSpeed*cos(inangle);

        if (triggeredLines[0] && vectorY > 0) { vectorY = -50;}
        if (triggeredLines[1] && vectorX > 0) { vectorX = -50;}
        if (triggeredLines[2] && vectorY < 0) { vectorY = 50;}
        if (triggeredLines[3] && vectorX < 0) { vectorX = 50;}

        inangle = atan2(vectorX, vectorY);
        double correctMotorSpeed = sqrt(vectorX*vectorX + vectorY*vectorY);

        Serial.print(inangle / PI * 180);
        Serial.print(", ");
        Serial.println(motorSpeed);

        /*Serial.print("Current Angle: ");
        Serial.print(curAngle);
        Serial.print(" | Input Angle: ");
        Serial.println(inangle);*/
        double m2 = cos(inangle+PI/4) * correctMotorSpeed;
        double m1 = cos(inangle-PI/4) * correctMotorSpeed;
        //Serial.println(motorSpeed);
        bt.setM1Speed(correct(-m1));
        bt.setM2Speed(correct(-m2));
        tp.setM1Speed(correct(m1));
        tp.setM2Speed(correct(m2));


        // bt.setM1Speed(-200);
        // bt.setM2Speed(-200);
        // tp.setM1Speed(200);
        // tp.setM2Speed(200);

    }

    int getmotorSpeed() {
        /*Serial.print(speedTimer);
        Serial.print(" : ");
        Serial.print(curSpeed);
        Serial.print(" -> ");
        Serial.print(motorSpeed);
        Serial.print(" (");
        Serial.print(motorSpeed-prevSpeed);
        Serial.println(")");*/
        return motorSpeed;
    }

    double correct(double speed) {
      double newOrientation = curOrientation;
      if (curOrientation > 180) {newOrientation -=360;}
      return max(-400, min(400, speed - newOrientation*2));
    }

    double relativeAngle(double input) {
        while (input > 180) {input -= 360;}
        while (input < -180) {input +=360;}
        return input;
    }
}   // namespace MotorDriver
