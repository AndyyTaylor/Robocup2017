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
    
    void lookAtBall();
    void findBall();
}   // namespace Vision

#endif
