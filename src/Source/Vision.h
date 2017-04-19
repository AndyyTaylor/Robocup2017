#ifndef CAMERA_H_INCLUDED
#define CAMERA_H_INCLUDED

class Camera
{
public:
    Camera();
    bool init();
    
    void update();
    void updateView();
    
    double getBallAngle();
};


#endif