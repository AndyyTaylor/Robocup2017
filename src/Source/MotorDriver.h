#ifndef MOTORDRIVER_H_INCLUDED
#define MOTORDRIVER_H_INCLUDED

namespace MotorDriver {
    void init();

    void update(double orientation);
    void direction(double inangle, bool smooth = true);
    void setMaxSpeed(int speed);
    void stop();

    double correct(double speed);
}   // namespace MotorDriver


#endif
