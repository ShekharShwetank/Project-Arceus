#include "pid.h"

PID::PID(float kp, float ki, float kd, float dt)
    : _kp(kp), _ki(ki), _kd(kd), _dt(dt), _prev_error(0.0f), _integral(0.0f) {}

float PID::compute(float setpoint, float measured) {
    float error = setpoint - measured;
    _integral += error * _dt;
    float derivative = (error - _prev_error) / _dt;
    _prev_error = error;

    float output = _kp * error + _ki * _integral + _kd * derivative;

    if (output > _max_output) output = _max_output;
    if (output < _min_output) output = _min_output;

    return output;
}

void PID::reset() {
    _prev_error = 0.0f;
    _integral = 0.0f;
}

void PID::setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::setOutputLimits(float min, float max) {
    _min_output = min;
    _max_output = max;
}

float PID::getKp() const { return _kp; }
float PID::getKi() const { return _ki; }
float PID::getKd() const { return _kd; }
