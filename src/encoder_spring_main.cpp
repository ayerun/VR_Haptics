#include <nuhal/uart.h>
#include <nuhal/uart_linux.h>
#include <motor_communication.hpp>
#include <iostream>
#include <signal.h>

void signal_callback(int signum) {
   std::cout << "Caught signal " << signum << std::endl;
   std::cout << "Terminating Program" << std::endl;
   exit(signum);
}

int main(int argc, char* argv[]) {
    // Register signal and signal callback
    signal(SIGINT, signal_callback);

    Odrive odrive("/dev/ttyACM1", 115200);
    odrive.zeroEncoderPosition(0);
    odrive.setClosedLoopControl(0);

    double k = 6;
    double torque = 0;
    while(true) {
        odrive.updateEncoderReadings(0);
        double theta = odrive.getEncoderPosition();

        //engage spring
        if (theta > 1) {

            //calculate and clamp torque
            theta -= 1;
            torque = k*theta;
            torque = std::max(0.0,std::min(torque,0.5));

            //command motor
            odrive.sendTorqueCommand(0,-torque);

            //get motor current
            odrive.updateMotorCurrent(0);
            double current = odrive.getCurrent();

            //check if current limit has been reached
            if (current >= 1.5) {
                std::cout << "Input Motor Torque: " << torque << std::endl;
                std::cout << "Motor Current: " << current << std::endl << std::endl;
            }
        }

        //deactivate spring
        else {
            if(odrive.getInputTorque() != 0) {
                bool safe = odrive.sendTorqueCommand(0,0);
                if(safe) {
                    //set state to closed loop control in case of motor error
                    odrive.setClosedLoopControl(0);
                }
            }
        }
    }
}