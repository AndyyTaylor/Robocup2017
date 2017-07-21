#include "Orientation.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <elapsedMillis.h>
#include <Wire.h>

namespace Orientation {
    MPU6050 mpu;

    bool dmpReady = false;
    bool gyroStablized = false;  // set true once the gryo has stabalized
    float prevYaw = 0.0;
    elapsedMillis stabElapsed;
    uint8_t mpuIntStatus;
    uint8_t devStatus;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];

    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

    float yaw;
    float pitch;
    float roll;

    float yaw_off;
    float pitch_off;
    float roll_off;

    volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
    void dmpDataReady() {
        mpuInterrupt = true;
    }

    bool init() {
        return setupGyro();
    }

    void update() {
        loadGyroData();

        if (!gyroStablized) {
            checkGyroStabalized();
            if (gyroStablized) setOffsets();
        }
    }

    void setOffsets() {
        yaw_off = yaw;
        pitch_off = pitch;
        roll_off = roll;
    }

    void checkGyroStabalized() {
        if (abs(prevYaw-yaw) > 1) {
            stabElapsed = 0;
            prevYaw = yaw;
        }

        if (stabElapsed > 3000) {
            gyroStablized = true;
        }
    }

    void loadGyroData() {
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            mpu.resetFIFO();
            Serial.println(F("Gyro overflow!"));
        } else if (mpuIntStatus & 0x02) {
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            mpu.getFIFOBytes(fifoBuffer, packetSize);

            fifoCount -= packetSize;
            load_ypr();
        }
    }

    void load_ypr() {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        yaw = ypr[0]*180/M_PI - yaw_off;
        pitch = ypr[1]*180/M_PI - pitch_off;
        roll = ypr[2]*180/M_PI - roll_off;

        yaw = normalise(yaw);
        pitch = normalise(pitch);
        roll = normalise(roll);
    }

    bool setupGyro() {
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
              Wire.begin();
              TWBR = 24;
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
            mpu.setDMPEnabled(true);

            attachInterrupt(0, dmpDataReady, RISING);
            mpuIntStatus = mpu.getIntStatus();

            dmpReady = true;

            packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
            return false;
        }
        return true;
    }
    
    float normalise(float angle) {
        if (angle < 0) angle += 360;
        if (angle > 360) angle -= 360;
        
        return angle;
    }

    float getYaw() {
        return yaw;
    }

    bool isStabalized() {
        return gyroStablized;
    }
}   // namespace Orientation
