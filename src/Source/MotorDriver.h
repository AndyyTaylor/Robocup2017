#ifndef MOTORDRIVER_H_INCLUDED
#define MOTORDRIVER_H_INCLUDED

namespace MotorDriver {
    void init();

    void update(double orientation, bool lineTest[4]);
    void direction(double inangle, bool out = false, bool smooth = true);
    void setMaxSpeed(int speed);
    void stop(bool fix = true);

    int getMaxSpeed();

    double correct(double speed);
}   // namespace MotorDriver


#endif
