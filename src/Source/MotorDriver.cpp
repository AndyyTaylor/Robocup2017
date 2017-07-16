#include "MotorDriver.h"

#include <Arduino.h>

#include "DualMC33926MotorShield.h"

namespace MotorDriver
{
    DualMC33926MotorShield tp(7, 11, A0, 8, 12, A1, 4, 0);
    DualMC33926MotorShield bt(33, 5, A2, 32, 2, A3, 30, 31);

    int maxspeed = 400;

    double curOrientation = 0;

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

    void direction(double inangle) {
      //bt2 bt1  new tp1 bt2
      //tp1 tp2      tp2 bt1

      inangle = inangle/180*PI;
      double m2 = cos(inangle+PI/4) * maxspeed;
      double m1 = cos(inangle-PI/4) * maxspeed;

      bt.setM1Speed(correct(-m1));
      bt.setM2Speed(correct(-m2));
      tp.setM1Speed(correct(m1));
      tp.setM2Speed(correct(m2));
    }

    double correct(double speed) {
      double newOrientation = curOrientation;
      if (curOrientation > 180) {newOrientation -=360;}
      return max(-400, min(400, speed - newOrientation));
    }
}
