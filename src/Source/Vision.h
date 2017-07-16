#ifndef CAMERA_H_INCLUDED
#define CAMERA_H_INCLUDED

namespace Vision
{
    bool init();
    bool isVisible();

    void update();
    void updateView();
    void updateMotor();

    float getBallAngle();
    void lookAtBall();
    void findBall();
}

#endif
