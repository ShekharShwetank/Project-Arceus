#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, float ki, float kd, float dt);

    float compute(float setpoint, float measured);
    void reset();

    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);

    float getKp() const;
    float getKi() const;
    float getKd() const;

private:
    float _kp, _ki, _kd;
    float _dt;
    float _prev_error;
    float _integral;

    float _min_output;
    float _max_output;
};

#endif 
