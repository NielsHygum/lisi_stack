#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <cstdint>

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

int main() {
    int fd = open(I2C_DEVICE, O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open I2C device\n";
        return 1;
    }

    if (ioctl(fd, I2C_SLAVE, PCA9685_ADDR) < 0) {
        std::cerr << "Failed to connect to PCA9685\n";
        close(fd);
        return 1;
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

    // Step 4: Set PWM pulse for servo on channel 0
    // 0° = ~150, 90° = ~307, 180° = ~460
    float angle = 0.0f;
    std::string input = "";
    int pulse = 150;
    std::cout << "type q to quit\n";
    while(input != "q")
    {
//          int pulse = 307; // 1.5 ms pulse
//          pulse = 480;
            uint channel = 13;
            //std::cout << "current angle: " << angle << '\n';
            /*if (input == 'w')
                pulse += 5;

            if (input == 'z')
                pulse = 150;
            if (input == 'm')
               pulse = 460;
            if (input == 's')
                pulse -= 5;*/
            std::cin >> input;
            std::cout << "input was: " << input << "\n";
            pulse = angleToPulse(std::stof(input));
            std::cout << "pulse = " << pulse << '\n';
            setPWM(fd, channel, 0, pulse);
    }
    close(fd);
    std::cout << "Servo should now be at ~90 degrees\n";
    return 0;
}
