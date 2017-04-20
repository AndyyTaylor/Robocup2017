#ifndef CAMERA_H_INCLUDED
#define CAMERA_H_INCLUDED

namespace Vision
{
    bool init();
    
    void update();
    void updateView();
    
    float getBallAngle();
    void lookAtBall();
}

#endif