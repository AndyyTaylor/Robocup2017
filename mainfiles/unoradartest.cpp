#include "Source/Radar.h"

#include <Arduino.h>
#include <EasyTransfer.h>
#include <RPLidar.h>

#define RPLIDAR_MOTOR 3

struct SEND_STRUCT{
    int x;
    int y;
};

RPLidar lidar;
EasyTransfer Sender;

SEND_STRUCT mydata;
std::vector<Radar::Point> field;

#define I2C_SLAVE_ADDRESS 9
int data = 0;
            
void setup() {
    // Serial.begin(9600);
    lidar.begin(Serial3);
    Serial1.begin(115200);
    // Serial.println("Initializing...");

    Sender.begin(details(mydata), &Serial1);
    pinMode(RPLIDAR_MOTOR, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
}

int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void loop() {
    if (IS_OK(lidar.waitPoint())) {
        float distance = lidar.getCurrentPoint().distance;
        float angle    = lidar.getCurrentPoint().angle;
        bool  startBit = lidar.getCurrentPoint().startBit;
        byte  quality  = lidar.getCurrentPoint().quality;
        
        if (distance > 3000) return;
        
        float x = cosf(radians(static_cast<float>(angle))) * static_cast<float>(distance);
        float y = sinf(radians(static_cast<float>(angle))) * static_cast<float>(distance);
        
        field.push_back({(int) x, (int) y});
        
        if (field.size() % 10 == 0) {
            mydata.x = x;
            mydata.y = y;
            Sender.sendData();
        }
        
        // Serial.println(field.size());
        digitalWrite(LED_BUILTIN, LOW);
        if (field.size() > 300) {
            digitalWrite(LED_BUILTIN, HIGH);
            
            float pos[5];
            calcBoundRect(field, pos);
            
            field.clear();
        }
    
        
    } else {
        analogWrite(RPLIDAR_MOTOR, 0);
        // Serial.println("Connecting...");
        // try to detect RPLIDAR...
        rplidar_response_device_info_t info;
        if (IS_OK(lidar.getDeviceInfo(info, 100))) {
            // detected...
            lidar.startScan();
            
            // start motor rotating at max allowed speed
            analogWrite(RPLIDAR_MOTOR, 200);
            delay(1000);
        }
    }
    // Serial.println("End Loop.");
}
