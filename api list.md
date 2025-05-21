Mbed OS APIs:

I2C Communication:
I2C class for I2C protocol communication
I2C::frequency()
I2C::write()
I2C::read()

GPIO and PWM:
DigitalOut for LED control
PwmOut for motor and buzzer control
period_us()
pulsewidth_us()
period()
write()

Serial Communication:
BufferedSerial for USB communication
write()

Timer and Timing:

Timer class for timing operations
start()
reset()
elapsed_time()
ThisThread::sleep_for() for delays

Interrupt Handling
InterruptIn for PPM signal processing
rise() for interrupt attachment

Custom Class APIs

MPU6050 Class:
class MPU6050 {
    MPU6050(I2C &i2c, uint8_t addr = 0x68);
    bool initialize();
    void calibrate(int samples = 1000);
    void updateMahony();
    void getOrientation(float *pitch, float *roll, float *yaw);
}

PID Class:
class PID {
    PID(float kp, float ki, float kd, float dt);
    float compute(float setpoint, float measured);
    void reset();
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    float getKp();
    float getKi();
    float getKd();
}

Buzzer Class:
class Buzzer {
    Buzzer(PinName pin);
    void play_disarm_tune();
    void play_note(float frequency, int duration_ms);
}

PPM Module Functions:
void ppm_init();
int ppm_read(uint8_t channel);

Standard C++ APIs

Math Functions
fabs()
fmaxf()
fminf()
sqrtf()
asinf()
atan2f()

Standard I/O
snprintf() for string formatting

Chrono Library
std::chrono::duration
Time literals: ms (milliseconds)

Constants
M_PI for mathematical calculations.