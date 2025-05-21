#include "mpu6050.h"
#include <cmath>

#define M_PI 3.14159265358979323846f

MPU6050::MPU6050(I2C &i2c, uint8_t addr) : _i2c(i2c), _addr(addr << 1) {
    _timer.start();
}

bool MPU6050::initialize() {
    writeReg(0x6B, 0x00); 
    uint8_t whoami;
    readRegs(0x75, &whoami, 1);
    return whoami == 0x68;
}

void MPU6050::calibrate(int samples) {
    int32_t acc_sum[3] = {0}, gyro_sum[3] = {0};

    for (int i = 0; i < samples; ++i) {
        acc_sum[0] += read16(0x3B);
        acc_sum[1] += read16(0x3D);
        acc_sum[2] += read16(0x3F);
        gyro_sum[0] += read16(0x43);
        gyro_sum[1] += read16(0x45);
        gyro_sum[2] += read16(0x47);
        ThisThread::sleep_for(5ms);
    }

    for (int i = 0; i < 3; ++i) {
        acc_bias[i] = acc_sum[i] / (float)samples;
        gyro_bias[i] = gyro_sum[i] / (float)samples;
    }

    acc_bias[2] -= 16384.0f; 
}

void MPU6050::updateMahony() {
    int16_t raw_acc[3], raw_gyro[3];
    raw_acc[0] = read16(0x3B);
    raw_acc[1] = read16(0x3D);
    raw_acc[2] = read16(0x3F);
    raw_gyro[0] = read16(0x43);
    raw_gyro[1] = read16(0x45);
    raw_gyro[2] = read16(0x47);

    float acc[3], gyro[3];
    for (int i = 0; i < 3; ++i) {
        acc[i] = (raw_acc[i] - acc_bias[i]) / 16384.0f;
        gyro[i] = (raw_gyro[i] - gyro_bias[i]) / 131.0f * M_PI / 180.0f;
    }

    float dt = std::chrono::duration<float>(_timer.elapsed_time()).count();
    _timer.reset();

    float norm = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    if (norm == 0.0f) return;
    for (int i = 0; i < 3; ++i) acc[i] /= norm;

    
    float vx = 2*(q1*q3 - q0*q2);
    float vy = 2*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float ex = (acc[1]*vz - acc[2]*vy);
    float ey = (acc[2]*vx - acc[0]*vz);
    float ez = (acc[0]*vy - acc[1]*vx);

    integralFBx += twoKi * ex * dt;
    integralFBy += twoKi * ey * dt;
    integralFBz += twoKi * ez * dt;

    gyro[0] += twoKp * ex + integralFBx;
    gyro[1] += twoKp * ey + integralFBy;
    gyro[2] += twoKp * ez + integralFBz;

    
    float dq0 = 0.5f * (-q1*gyro[0] - q2*gyro[1] - q3*gyro[2]);
    float dq1 = 0.5f * ( q0*gyro[0] + q2*gyro[2] - q3*gyro[1]);
    float dq2 = 0.5f * ( q0*gyro[1] - q1*gyro[2] + q3*gyro[0]);
    float dq3 = 0.5f * ( q0*gyro[2] + q1*gyro[1] - q2*gyro[0]);

    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;

    
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

    
    pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * 180.0f / M_PI;
    roll  = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * 180.0f / M_PI;
    yaw   = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 180.0f / M_PI;

    
    float gyro_z_thresh = 0.02f;
    float acc_z_thresh = 0.05f;

    bool is_stationary = fabsf(gyro[2]) < gyro_z_thresh && fabsf(acc[2] - 1.0f) < acc_z_thresh;
    if (is_stationary) {
        yaw *= 0.99985f;  
    }
}

void MPU6050::getOrientation(float *pitchOut, float *rollOut, float *yawOut) {
    *pitchOut = pitch;
    *rollOut = roll;
    *yawOut = yaw;
}

void MPU6050::writeReg(uint8_t reg, uint8_t data) {
    char buf[2] = {static_cast<char>(reg), static_cast<char>(data)};
    _i2c.write(_addr, buf, 2);
}

void MPU6050::readRegs(uint8_t reg, uint8_t *data, uint8_t length) {
    char r = reg;
    _i2c.write(_addr, &r, 1, true);
    _i2c.read(_addr, reinterpret_cast<char*>(data), length);
}

int16_t MPU6050::read16(uint8_t reg) {
    uint8_t data[2];
    readRegs(reg, data, 2);
    return (int16_t)((data[0] << 8) | data[1]);
}
