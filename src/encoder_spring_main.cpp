#include <nuhal/uart.h>
#include <nuhal/uart_linux.h>
#include <motor_communication.hpp>
#include <iostream>
#include <signal.h>
#include <chrono>
#include <unistd.h>
#include <fstream>

void signal_callback(int signum) {
   std::cout << "Caught signal " << signum << std::endl;
   std::cout << "Terminating Program" << std::endl;
   exit(signum);
}

int main(int argc, char* argv[]) {
    // Register signal and signal callback
    signal(SIGINT, signal_callback);

    std::string filename;
    std::string portname;
    if (argc == 2) {
        filename = argv[1];
        portname = "/dev/ttyACM0";
    } 
    else if (argc == 3) {
        filename = argv[1];
        portname = argv[2];
    }
    else {
        std::cout << "whats the output file name?" << std::endl;
        return 0;
    }

    
    std::ofstream datafile;
    datafile.open(filename);

    Odrive odrive("/dev/ttyACM0", 115200);
    odrive.zeroEncoderPosition(0);
    // odrive.setClosedLoopControl(0);

    double k = 6;
    double torque = 0;
    double loop_rate = 0.1;
    std::chrono::steady_clock::time_point program_start = std::chrono::steady_clock::now();
    while(true) {
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        odrive.updateEncoderReadings(0);
        const double theta = odrive.getEncoderPosition();
        double displacement = 0;

        //get motor current
        odrive.updateMotorCurrent(0);
        double current = odrive.getCurrent();

        //engage spring
        if (theta > 1) {

            //calculate and clamp torque
            displacement = theta-1;
            torque = k*displacement;
            torque = std::max(0.0,std::min(torque,0.5));

            //command motor
            odrive.sendTorqueCommand(0,0);
        }
        //deactivate spring
        else {
            if(odrive.getInputTorque() != 0) {
                bool safe = odrive.sendTorqueCommand(0,0);
                // if(safe) {
                //     //set state to closed loop control in case of motor error
                //     odrive.setClosedLoopControl(0);
                // }
            }
        }
        if (current >= 1.5) {
            std::cout << "Input Motor Torque: " << torque << std::endl;
            std::cout << "Motor Current: " << current << std::endl << std::endl;
        }

        //track time
        std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(stop-start);
        double time_stamp = std::chrono::duration_cast<std::chrono::duration<double>>(stop-program_start).count();

        //write to csv
        datafile << time_stamp << "," << current << "," << torque << "," << theta << "\n";

        //enforce loop rate
        if (time_span.count() < loop_rate) {
            double sleeptime = loop_rate-time_span.count();
            sleep(sleeptime);
        }
    }
    datafile.close();
    return 1;
}