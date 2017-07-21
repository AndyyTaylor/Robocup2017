#include "Source/Radar.h"

#include <Arduino.h>
#include <EasyTransfer.h>
#include <RPLidar.h>

struct SEND_STRUCT{
    int x;
    int y;
    int width;
    int height;
    float angle;
};

const int RPLIDAR_MOTOR = 3;
const int I2C_SLAVE_ADDRESS = 9;

RPLidar lidar;
SEND_STRUCT mydata;
EasyTransfer Sender;
std::vector<Radar::Point> field;

void setup() {
    lidar.begin(Serial3);
    Serial1.begin(115200);

    Sender.begin(details(mydata), &Serial1);
    pinMode(RPLIDAR_MOTOR, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    
    delay(18000);
}

void loop() {
    if (IS_OK(lidar.waitPoint())) {
        float distance = lidar.getCurrentPoint().distance;
        float angle    = lidar.getCurrentPoint().angle;
        // bool  startBit = lidar.getCurrentPoint().startBit;
        // byte  quality  = lidar.getCurrentPoint().quality;
        
        if (distance > 3000) return;
        
        float x = cosf(radians(static_cast<float>(angle))) * static_cast<float>(distance);
        float y = sinf(radians(static_cast<float>(angle))) * static_cast<float>(distance);
        
        field.push_back({static_cast<int>(x), static_cast<int>(y)});
        
        digitalWrite(LED_BUILTIN, LOW);
        if (field.size() > 100) {
            digitalWrite(LED_BUILTIN, HIGH);
            
            float pos[5];
            calcBoundRect(field, pos);
            
            mydata.x = pos[0];
            mydata.y = pos[1];
            mydata.width = pos[2];
            mydata.height = pos[3];
            mydata.angle = pos[4];
            Sender.sendData();
            
            field.clear();
        }
    } else {
        analogWrite(RPLIDAR_MOTOR, 0);
        rplidar_response_device_info_t info;
        if (IS_OK(lidar.getDeviceInfo(info, 100))) {
            lidar.startScan();
            
            analogWrite(RPLIDAR_MOTOR, 200);
            delay(1000);
        }
    }
}