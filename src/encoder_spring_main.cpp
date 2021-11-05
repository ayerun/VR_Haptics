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

    //logging
    std::string filename;
    std::ofstream datafile;
    bool loggingEnabled = false;

    //Odrive port
    std::string portname;
    std::string default_port = "/dev/ttyACM0";
    
    if (argc == 1) {
        portname = default_port;
    }
    else if (argc == 2) {
        filename = argv[1];
        portname = default_port;
        loggingEnabled = true;
    } 
    else if (argc == 3) {
        filename = argv[1];
        portname = argv[2];
        loggingEnabled = true;
    }
    else {
        std::cout << "Invalid number of command line arguements" << std::endl;
        return 0;
    }

    if (loggingEnabled) {    
        datafile.open(filename);
    }

    //Odrive setup
    Odrive odrive(portname, 115200);
    odrive.zeroEncoderPosition(0);

    //constants
    double k = 60;
    double torque = 0;
    double loop_rate = 0.001;

    //start timer
    std::chrono::steady_clock::time_point program_start = std::chrono::steady_clock::now();

    while(true) {

        //start loop timer
        std::chrono::steady_clock::time_point loop_start = std::chrono::steady_clock::now();

        //get encoder data
        odrive.updateEncoderReadings(0);
        const double theta = odrive.getEncoderPosition();

        //spring displacement
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
            odrive.sendTorqueCommand(0,-torque);
        }

        //deactivate spring
        else {
            if(odrive.getInputTorque() != 0) {
                odrive.sendTorqueCommand(0,0);
            }
        }

        //track time
        std::chrono::steady_clock::time_point loop_stop = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(loop_stop-loop_start);
        double time_stamp = std::chrono::duration_cast<std::chrono::duration<double>>(loop_stop-program_start).count();

        //write to csv
        if (loggingEnabled) {
            datafile << time_stamp << "," << current << "," << torque << "," << theta << "\n";
        }

        //enforce loop rate
        if (time_span.count() < loop_rate) {
            double sleeptime = loop_rate-time_span.count();
            sleep(sleeptime);
        }
    }

    //close file
    datafile.close();
    return 1;
}