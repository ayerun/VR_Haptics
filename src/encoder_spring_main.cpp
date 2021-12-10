#include <nuhal/uart.h>
#include <nuhal/uart_linux.h>
#include <motor_communication.hpp>
#include <iostream>
#include <signal.h>
#include <chrono>
#include <unistd.h>
#include <fstream>

int main(int argc, char* argv[]) {

    //logging
    std::string filename;
    std::ofstream datafile;
    bool loggingEnabled = false;

    //Odrive port
    std::string portname;
    std::string default_port = "/dev/ttyACM1";
    
    //Parse command line arguements
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
        return 1;
    }

    //Odrive setup
    Odrive odrive(portname, 115200);
    odrive.zeroEncoderPosition(0);

    //constants
    double k = 0.1666667;   //[Nm/deg]
    double torque = 0;

    if (loggingEnabled) {    
        datafile.open(filename);
        datafile << "Time (s)" << "," << " Current (A)" << "," << " Torque (Nm)" << "," << " Angle (degrees)" << "," << " K = " << k  << " (N/deg)" <<"\n";
    }

    //start timer
    std::chrono::steady_clock::time_point program_start = std::chrono::steady_clock::now();

    while(true) {

        //start loop timer
        std::chrono::steady_clock::time_point loop_start = std::chrono::steady_clock::now();

        //get encoder data
        odrive.updateEncoderReadings(0);
        const double theta = odrive.getEncoderPosition()*360;

        //spring displacement
        double displacement = 0;

        //get motor current
        odrive.updateMotorCurrent(0);
        double current = odrive.getCurrent();

        //engage spring
        if (theta > 360) {

            //calculate and clamp torque
            displacement = theta-360;
            torque = k*displacement;
            torque = std::max(0.0,std::min(torque,0.5));

            //command motor
            odrive.sendTorqueCommand(0,-torque);
        }

        //deactivate spring
        else odrive.sendTorqueCommand(0,0);

        //track time
        std::chrono::steady_clock::time_point loop_stop = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(loop_stop-loop_start);
        double time_stamp = std::chrono::duration_cast<std::chrono::duration<double>>(loop_stop-program_start).count();

        //write to csv
        if (loggingEnabled) {
            datafile << time_stamp << "," << current << "," << torque << "," << theta-360 << "\n";
        }
    }

    //close file
    datafile.close();
    return 0;
}