// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <Arduino.h>

#include "Source/Radar.h"

#include <RPLidar.h>

RPLidar lidar;

#define RPLIDAR_MOTOR 3
#include <EasyTransfer.h>

EasyTransfer ET;

struct SEND_DATA_STRUCTURE{
    int x;
    int y;
    int width;
    int height;
    float angle;
};

SEND_DATA_STRUCTURE mydata;
std::vector<Radar::Point> field;

#define I2C_SLAVE_ADDRESS 9
int data = 0;
            
void setup() {
    // Serial.begin(9600);
    lidar.begin(Serial3);
    Serial1.begin(115200);
    // Serial.println("Initializing...");

    ET.begin(details(mydata), &Serial1);
    pinMode(RPLIDAR_MOTOR, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
}

int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void loop() {
    // digitalWrite(LED_BUILTIN, LOW);
    // Serial.println("Looping...");
    if (IS_OK(lidar.waitPoint())) {
        float distance = lidar.getCurrentPoint().distance;
        float angle    = lidar.getCurrentPoint().angle;
        bool  startBit = lidar.getCurrentPoint().startBit;
        byte  quality  = lidar.getCurrentPoint().quality;
        
        if (startBit) data+=1;
        // delete &distance;
        // delete &angle;
        if (distance > 3000) return;
        
        
        float x = cosf(radians(static_cast<float>(angle))) * static_cast<float>(distance);
        float y = sinf(radians(static_cast<float>(angle))) * static_cast<float>(distance);
        
        field.push_back({(int) x, (int) y});
        
        // Serial.println(field.size());
        digitalWrite(LED_BUILTIN, LOW);
        if (field.size() > 100) {
            digitalWrite(LED_BUILTIN, HIGH);
            
            float pos[5];
            calcBoundRect(field, pos);
            
            // mydata.x = *pos;
            // mydata.y = *(pos + 1);
            /* mydata.y = field.size();
            mydata.w = *(pos + 2);
            mydata.h = *(pos + 3);
            mydata.angle = *(pos + 4); */
            // mydata.angle = sizeof(Radar::Point);  // + (sizeof(Radar::Point) * field.size());
            
            // mydata.angle = freeRam();
            mydata.x = pos[0];
            mydata.y = pos[1];
            mydata.width = pos[2];
            mydata.height = pos[3];
            mydata.angle = pos[4];
            ET.sendData();
            // Serial.println(angle);
            
            field.clear();
            
            data = 0;
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
