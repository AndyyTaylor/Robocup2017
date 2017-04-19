#include "MotorDriver.h"

#include <Arduino.h>

#include <Wire.h>
#include "DualMC33926MotorShield.h"

namespace MotorDriver
{
    DualMC33926MotorShield tp(3, 9, A2, 2, 10, A3, 0, 11);
    DualMC33926MotorShield bt;

    int maxspeed = 400;

    void init() {
      tp.init();
      bt.init();
    }

    void setMaxSpeed(int speed) {
      maxspeed = speed;
    }

    void direction(double inangle) {
      inangle = inangle/180*PI;
      double m2 = cos(inangle-PI/4) * maxspeed;
      double m1 = cos(inangle+PI/4) * maxspeed;

      bt.setM1Speed(-m1);
      bt.setM2Speed(m2);
      tp.setM1Speed(m1);
      tp.setM2Speed(-m2);
    }
}
