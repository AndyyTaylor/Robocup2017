#ifndef ORIENTATION_H_INCLUDED
#define ORIENTATION_H_INCLUDED

namespace Orientation
{
    void init();
    void update();
    
    void setupCompass();
    void setupGyro();

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