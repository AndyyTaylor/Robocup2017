#ifndef MOTORDRIVER_H_INCLUDED
#define MOTORDRIVER_H_INCLUDED

namespace MotorDriver
{
    void init();

    void update(double orientation);

    void direction(double inangle);

    void setMaxSpeed(int speed);

    void stop();

    double correct(double speed);
}


#endif
