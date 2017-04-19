#ifndef CAMERA_H_INCLUDED
#define CAMERA_H_INCLUDED

namespace Camera
{
    bool init();
    
    void update();
    void updateView();
    
    double getBallAngle();
}

#endif