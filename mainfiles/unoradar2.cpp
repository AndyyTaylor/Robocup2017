// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <Arduino.h>

#include "Source/Radar.h"

#include <RPLidar.h>

RPLidar lidar;

#define RPLIDAR_MOTOR 3
#include <EasyTransferI2C.h>
#include <EasyTransfer.h>

EasyTransfer ETin, ETout;

struct SEND_DATA_STRUCTURE{
  float number;
  float angle;
  bool visible;
  bool stab;
};

struct RECEIVE_DATA_STRUCTURE{
    
};

RECEIVE_DATA_STRUCTURE rxdata;
SEND_DATA_STRUCTURE txdata;

std::vector<Radar::Point> field;

int data = 0;
            
void setup() {
    // Serial.begin(9600);
    // lidar.begin(Serial1);
    // Serial.println("Initializing...");
    Serial1.begin(115200);
    Wire.begin();
    ETin.begin(details(rxdata), &Serial1);
    ETout.begin(details(txdata), &Serial1);
    pinMode(RPLIDAR_MOTOR, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
}

int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void loop() {
    // mydata.number = -1;
    txdata.angle = -1;
    txdata.stab = false;
    txdata.visible = false;
    Serial.println("Sendy send");
    ETout.sendData();
}

void loop2() {
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
        
        /*mydata.x = x;
        mydata.y = y;
        mydata.q = quality;*/
        // if (field.size() % 10 == 0) ET.sendData(I2C_SLAVE_ADDRESS);
        
        field.push_back({(int) x, (int) y});
        
        // Serial.println(field.size());
        digitalWrite(LED_BUILTIN, LOW);
        if (field.size() > 200) {
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
