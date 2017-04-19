#ifndef ORIENTATION_H_INCLUDED
#define ORIENTATION_H_INCLUDED

namespace Orientation
{
    bool init();
    void update();
    
    bool setupCompass();
    bool setupGyro();

    void calcCompassHeading();
    void loadGyroData();
    void load_ypr();
    void checkGyroStabalized();
    void setOffsets();
    
    double getCompassHeading();
    double getYaw();
    bool isStabalized();

    void outputAllData();
}


#endif