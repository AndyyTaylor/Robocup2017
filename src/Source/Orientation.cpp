#include "Orientation.h"

#include <Arduino.h>

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <elapsedMillis.h>

namespace Orientation
{
    Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
    MPU6050 mpu;

    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    bool gyroStablized = false; // set true once the gryo has stabalized
    float prevYaw = 0.0;
    elapsedMillis stabElapsed;
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    float compassHeading;
    float yaw;
    float pitch;
    float roll;

    float comp_off;
    float yaw_off;
    float pitch_off;
    float roll_off;

    volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
    void dmpDataReady() {
        mpuInterrupt = true;
    }

    void init() {
      //Serial.begin(115200);

      setupCompass();
      setupGyro();
      
      Serial.println("Waiting for gyro to stabalize...");
    }

    void update() {
        while (!mpuInterrupt && fifoCount < packetSize) {
            calcCompassHeading();
        }

        loadGyroData();
        
        if (!gyroStablized) 
        {
            checkGyroStabalized();
            if (gyroStablized) setOffsets();
        }
        
        if (gyroStablized)
        {
            //outputAllData();
        }    
    }

    void setOffsets()
    {
        comp_off = compassHeading;
        yaw_off = yaw;
        pitch_off = pitch;
        roll_off = roll;
    }

    void checkGyroStabalized()
    {
        if (abs(prevYaw-yaw)>1)
        {
            stabElapsed = 0;
            prevYaw = yaw;
        }
        
        if (stabElapsed > 3000)
        {
            gyroStablized = true;
        }
    }

    void calcCompassHeading()
    {
      sensors_event_t event;
      mag.getEvent(&event);

      float heading = atan2(event.magnetic.y, event.magnetic.x);
      heading += 0.22; // 13 degrees declination angle

      if(heading < 0)
        heading += 2*PI;

      if(heading > 2*PI)
        heading -= 2*PI;

      compassHeading = heading * 180/M_PI - comp_off;
    }

    void loadGyroData()
    {
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            mpu.resetFIFO();
            Serial.println(F("FIFO overflow!"));
        } else if (mpuIntStatus & 0x02) {
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            mpu.getFIFOBytes(fifoBuffer, packetSize);

            fifoCount -= packetSize;
            load_ypr();
        }
    }

    void load_ypr()
    {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        yaw = ypr[0]*180/M_PI - yaw_off;
        pitch = ypr[1]*180/M_PI - pitch_off;
        roll = ypr[2]*180/M_PI - roll_off;
        
        // normalise(yaw)
        if(yaw < 0)
            yaw += 360;
        if(yaw > 360)
            yaw -= 360;
        
        if(pitch < 0)
            pitch += 360;
        if(pitch > 360)
            pitch -= 360;
            
        if(roll < 0)
            roll += 360;
        if(roll > 360)
            roll -= 360;
    }

    void outputAllData()
    {
        Serial.println("----------------");
        Serial.print("Compass Heading: ");
        Serial.println(compassHeading);
        
        Serial.println("Gryo");
        Serial.print("\tyaw: ");
        Serial.println(yaw);
        //Serial.print("\tpitch: ");
        //Serial.println(pitch);
        //Serial.print("\troll: ");
        //Serial.println(roll);
        
        /*Serial.println("Offsets");
        Serial.print("\tcomp: ");
        Serial.println(comp_off);
        Serial.print("\tyaw: ");
        Serial.println(yaw_off);
        Serial.print("\tpitch: ");
        Serial.println(pitch_off);
        Serial.print("\troll: ");
        Serial.println(roll_off);*/
    }

    void setupCompass()
    {
        if(!mag.begin())
        {
          /* There was a problem detecting the HMC5883 ... check your connections */
          Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
          while(1);
        }
    }

    void setupGyro()
    {
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
              Wire.begin();
              TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
        #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
              Fastwire::setup(400, true);
        #endif

        mpu.initialize();

        devStatus = mpu.dmpInitialize();

        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788); 

        if (devStatus == 0) {
          Serial.println(F("Enabling DMP..."));
          mpu.setDMPEnabled(true);

          Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
          attachInterrupt(0, dmpDataReady, RISING);
          mpuIntStatus = mpu.getIntStatus();

          Serial.println(F("DMP ready! Waiting for first interrupt..."));
          dmpReady = true;

          packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
          Serial.print(F("DMP Initialization failed (code "));
          Serial.print(devStatus);
          Serial.println(F(")"));
        }
    }
    
    double getCompassHeading()
    {
        return compassHeading;
    }
    
    double getYaw()
    {
        return yaw;
    }
    
    bool isStabalized()
    {
        return gyroStablized;
    }
}
