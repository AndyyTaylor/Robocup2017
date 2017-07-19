#ifndef ORIENTATION_H_INCLUDED
#define ORIENTATION_H_INCLUDED

namespace Orientation {
    bool init();
    void update();
    
    bool setupGyro();

    void loadGyroData();
    void load_ypr();
    void checkGyroStabalized();
    void setOffsets();
    
    float normalise(float angle);
    float getYaw();
    bool isStabalized();

    void outputAllData();
}   // namespace Orientation


#endif
