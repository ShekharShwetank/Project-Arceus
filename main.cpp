#include "mbed.h"
#include "mpu6050.h"
#include "pid.h"
#include "ppm.h"
#include "buzzer.h"  

using namespace ThisThread;

BufferedSerial usb(PA_2, PA_3, 115200);
I2C i2c(PB_9, PB_8);
MPU6050 mpu(i2c);
DigitalOut Led(PC_13);
char buffer[128];


PwmOut motor1(PB_13); // Front Left
PwmOut motor2(PA_10); // Front Right
PwmOut motor3(PB_6);  // Rear Right
PwmOut motor4(PB_5);  // Rear Left

Buzzer buzzer(PA_15); 

float pitch, roll, yaw;
float base_throttle = 0.0f;
float desired_pitch = 0.0f, desired_roll = 0.0f, desired_yaw = 0.0f;

float dt = 0.01f;
PID pid_pitch(0.6f, 0.003f, 0.15f, dt);
PID pid_roll(0.6f, 0.003f, 0.15f, dt);
PID pid_yaw(0.4f, 0.001f, 0.1f, dt);


float apply_deadband(float input, float deadband = 0.025f) {
    return (fabs(input) < deadband) ? 0.0f : input;
}

int main() {
    i2c.frequency(400000);
    usb.write("Initializing MPU6050...\n", 26);
    Led = 1;

    if (!mpu.initialize()) {
        usb.write("MPU6050 initialization failed!\n", 33);
        Led = 0;
        return 1;
    }

    usb.write("Calibrating sensors...\n", 24);
    mpu.calibrate(1000);
    usb.write("Calibration done.\n", 19);

    ppm_init();

    motor1.period_us(2000);
    motor2.period_us(2000);
    motor3.period_us(2000);
    motor4.period_us(2000);
    
    
    motor1.pulsewidth_us(1000);
    motor2.pulsewidth_us(1000);
    motor3.pulsewidth_us(1000);
    motor4.pulsewidth_us(1000);
    sleep_for(2s);

    pid_pitch.reset();
    pid_roll.reset();
    pid_yaw.reset();

    Timer loop_timer;
    loop_timer.start();


    pid_pitch.setTunings(0.33f, 0.003f, 0.06f);
    pid_roll.setTunings(0.31f, 0.003f, 0.07f);    
    pid_yaw.setTunings(0.3f, 0.001f, 0.08f);      

    pid_pitch.setOutputLimits(-0.10f, 0.10f);
    pid_roll.setOutputLimits(-0.9f, 0.9f);
    pid_yaw.setOutputLimits(-0.04f, 0.06f);
    float pitch_trim = -0.08f;

    while (true) {
        mpu.updateMahony();
        mpu.getOrientation(&pitch, &roll, &yaw);

        int roll_raw     = ppm_read(0); // CH1
        int pitch_raw    = ppm_read(1); // CH2
        int throttle_raw = ppm_read(2); // CH3
        int yaw_raw      = ppm_read(3); // CH4
        int arm_raw      = ppm_read(4); // CH5

        bool armed = arm_raw > 1500;

        if (!armed || throttle_raw < 900 || throttle_raw > 2100) {
            motor1.pulsewidth_us(1000);
            motor2.pulsewidth_us(1000);
            motor3.pulsewidth_us(1000);
            motor4.pulsewidth_us(1000);
            buzzer.play_disarm_tune(); 
            sleep_for(100ms);
            continue;
        }

        base_throttle = (float)(throttle_raw - 1000) / 1000.0f;
        base_throttle = fmaxf(base_throttle, 0.15f); 

        desired_roll  = (float)(roll_raw  - 1500) / 500.0f;
        desired_pitch = (float)(pitch_raw - 1500) / 500.0f;
        desired_yaw   = (float)(yaw_raw   - 1500) / 500.0f;

        desired_roll  = fminf(fmaxf(desired_roll, -1.0f), 1.0f);
        desired_pitch = fminf(fmaxf(desired_pitch, -1.0f), 1.0f);
        desired_yaw   = fminf(fmaxf(desired_yaw, -1.0f), 1.0f);

        desired_roll  = apply_deadband(desired_roll);
        desired_pitch = apply_deadband(desired_pitch);
        desired_yaw   = apply_deadband(desired_yaw);

        bool stick_override = (pitch_raw > 1900 && roll_raw > 1900);

        float m1, m2, m3, m4;

        if (stick_override) {
            m1 = m2 = m3 = m4 = 0.25f;
            pid_pitch.reset();
            pid_roll.reset();
            pid_yaw.reset();
        }
        else {
            float pitch_out = pid_pitch.compute(desired_pitch + pitch_trim, pitch);
            float roll_out  = pid_roll.compute(desired_roll, roll);
            float yaw_out   = pid_yaw.compute(desired_yaw, yaw);

            m1 = base_throttle + pitch_out - roll_out + yaw_out;
            m2 = base_throttle + pitch_out + roll_out - yaw_out;
            m3 = base_throttle - pitch_out + roll_out + yaw_out;
            m4 = base_throttle - pitch_out - roll_out - yaw_out;

            m1 = fminf(fmaxf(m1, 0.15f), 1.0f);
            m2 = fminf(fmaxf(m2, 0.15f), 1.0f);
            m3 = fminf(fmaxf(m3, 0.15f), 1.0f);
            m4 = fminf(fmaxf(m4, 0.15f), 1.0f);
        }

        motor1.pulsewidth_us((int)(1010 + (m1 - 0.150f) * 1000));
        motor2.pulsewidth_us((int)(1010 + (m2 - 0.150f) * 1000));
        motor3.pulsewidth_us((int)(1010 + (m3 - 0.150f) * 1000));
        motor4.pulsewidth_us((int)(1010 + (m4 - 0.150f) * 1000));

        int len = snprintf(buffer, sizeof(buffer),"M1: %d, M2: %d, M3: %d, M4: %d\n\n",(int)(m1 * 100), (int)(m2 * 100), (int)(m3 * 100), (int)(m4 * 100));
        usb.write(buffer, len);

        sleep_for(10ms);
    }
}
