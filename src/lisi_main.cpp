#include <iostream>
#include "../include/i2c_pwm_controller.hpp"

int main() {

    I2CPWMController i2c_pwm_controller;

    std::cout << "type q to quit\n";
    std::string input = "";
    while(input != "q")
    {
//          int pulse = 307; // 1.5 ms pulse
//          pulse = 480;
           // uint channel = 13;
            //std::cout << "current angle: " << angle << '\n';
            /*if (input == 'w')
                pulse += 5;

            if (input == 'z')
                pulse = 150;
            if (input == 'm')
               pulse = 460;
            if (input == 's')
                pulse -= 5;*/
            std::cout << "set channel" << std::endl;
            std::cin >> input;
            int channel = std::stoi(input);
            std::cout << "set angle" << std::endl;
        std::cin >> input;
            float angle = std::stof(input);
            std::cout << "input was: " << input << "\n";
       // i2c_pwm_controller.setServoAngle(13, input) //steering //channel 13 steering channel 15 throttle
        i2c_pwm_controller.setServoAngle(channel, std::stof(input)); //throttle
    }

    return 0;
}
