#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdlib>
#include <mutex>
#include <iostream>

#define I2C_DEVICE "/dev/i2c-1"
#define PCA9685_ADDR 0x40

// PCA9685 Registers
#define MODE1      0x00
#define PRESCALE   0xFE
#define LED0_ON_L  0x06

// Utility function to write to a register
void writeRegister(int fd, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    write(fd, buffer, 2);
}

// Function to set PWM pulse on one channel
void setPWM(int fd, int channel, int on, int off) {
    int reg_base = LED0_ON_L + 4 * channel;
    writeRegister(fd, reg_base + 0, on & 0xFF);        // LEDn_ON_L
    writeRegister(fd, reg_base + 1, (on >> 8) & 0xFF); // LEDn_ON_H
    writeRegister(fd, reg_base + 2, off & 0xFF);       // LEDn_OFF_L
    writeRegister(fd, reg_base + 3, (off >> 8) & 0xFF);// LEDn_OFF_H
}

int angleToPulse(double angle_deg)
{
    constexpr float min_pulse = 150;
    constexpr float max_pulse = 465;
    if(angle_deg > 0 and angle_deg <= 180)
    {
        float pulse = min_pulse + (angle_deg / 180.0f) * (max_pulse - min_pulse);
        return pulse;
    }
    else
    {
        std::cout << "invalid deg\n";
    }
    return min_pulse;
}

class I2CPWMController
{
public:

    void setServoAngle(uint16_t channel, float angle_deg);

    I2CPWMController();

    ~I2CPWMController();
private:
    bool fd_initialized = false;
    int fd;
    std::mutex fd_mtx_;

    void initPCA9685();
};

I2CPWMController::I2CPWMController()
{
    fd_initialized = false;

    fd = open(I2C_DEVICE, O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open I2C device\n";
        std::exit(EXIT_FAILURE);
    }
    fd_initialized = true;

    initPCA9685();
}

I2CPWMController::~I2CPWMController()
{
    if(fd_initialized)
    {
        close(fd);
    }
}

void I2CPWMController::setServoAngle(uint16_t channel, float angle_deg)
{
    // make fd mutex lock
    auto pulse = angleToPulse(angle_deg);
    setPWM(fd, channel, 0, pulse);
}

void I2CPWMController::initPCA9685() // make private
{
    if (ioctl(fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        std::cerr << "Failed to connect to PCA9685\n";
        close(fd);
        std::exit(EXIT_FAILURE);
    }

    // Step 1: Enter sleep mode to set frequency
    writeRegister(fd, MODE1, 0x10); // Sleep

    // Step 2: Set prescale to get ~50Hz
    // prescale = round(25MHz / (4096 * freq)) - 1
    // 50 Hz => prescale = 121 (0x79)
    writeRegister(fd, PRESCALE, 121);

    // Step 3: Wake up
    writeRegister(fd, MODE1, 0x00);
    usleep(5000); // Wait for oscillator
}