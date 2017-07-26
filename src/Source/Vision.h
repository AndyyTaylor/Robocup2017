#ifndef CAMERA_H_INCLUDED
#define CAMERA_H_INCLUDED

namespace Vision {
    bool init();
    bool isVisible();

    void update(int target);
    void updateView(int target);
    void updateMotor();

    float getBallAngle();
    int getBallY();
    
    void setDribblerSpeed(int speed);
    void setLimit(int _limit);

    void runDribblerMotor(int dir);
    
    
    void lookAtBall();
    void findBall();
}   // namespace Vision

#endif
