#ifndef MPU6050_H
#define MPU6050_H

#include "mbed.h"

class MPU6050 {
public:
    MPU6050(I2C &i2c, uint8_t addr = 0x68);
    bool initialize();
    void calibrate(int samples = 1000);

    void updateMahony();
    void getOrientation(float *pitch, float *roll, float *yaw);

private:
    I2C &_i2c;
    uint8_t _addr;

    void writeReg(uint8_t reg, uint8_t data);
    void readRegs(uint8_t reg, uint8_t *data, uint8_t length);
    int16_t read16(uint8_t reg);

    Timer _timer;

    float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
    float gyro_bias[3] = {0};
    float acc_bias[3] = {0};

    float twoKp = 2.0f * 0.5f;  
    float twoKi = 2.0f * 0.0f;  
    float integralFBx = 0, integralFBy = 0, integralFBz = 0;

    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  
};

#endif
