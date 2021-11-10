#include "pch.h"
#include "common.h"
#include "options.h"
#include "platformplugin.h"
#include "graphicsplugin.h"
#include "openxr_program.h"
#include "motor_communication.hpp"
#include <fstream>
#include <Eigen/Geometry>

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
        return 0;
    }

    //Odrive setup
    Odrive odrive(portname, 115200);
    odrive.zeroEncoderPosition(0);


    // Set graphics plugin, VR form factor, and VR view configuration
    std::shared_ptr<Options> options = std::make_shared<Options>();
    options->GraphicsPlugin = "Vulkan2";
    options->FormFactor = "Hmd";
    options->ViewConfiguration = "Stereo";

    bool requestRestart = false;
    do {
        // Create platform-specific implementation.
        std::shared_ptr<IPlatformPlugin> platformPlugin = CreatePlatformPlugin_Xlib(options);

        // Create graphics API implementation.
        std::shared_ptr<IGraphicsPlugin> graphicsPlugin = CreateGraphicsPlugin_Vulkan(options, platformPlugin);

        // Initialize the OpenXR program.
        std::shared_ptr<IOpenXrProgram> program = CreateOpenXrProgram(options, platformPlugin, graphicsPlugin);

        program->CreateInstance();
        program->InitializeSystem();
        program->InitializeSession();
        program->CreateSwapchains();

        //constants
        double k = 0.1666667;   //[Nm/deg]
        double torque = 0;

        if (loggingEnabled) {    
            datafile.open(filename);
            datafile << "Time (s)" << "," << " Current (A)" << "," << " Torque (Nm)" << "," << " Angle (degrees)" << "," << " K = " << k  << " (N/deg)" <<"\n";
        }

        //start timer
        std::chrono::steady_clock::time_point program_start = std::chrono::steady_clock::now();

        bool exitRenderLoop = false;
        while (!exitRenderLoop) {
            program->PollEvents(&exitRenderLoop, &requestRestart);
            if (exitRenderLoop) {
                break;
            }

            if (program->IsSessionRunning()) {
                program->PollActions();


                XrTime displayTime = program->RenderFrame();
                XrSpaceLocation pos = program->getControllerSpace(displayTime);

                //convert openXR types to Eigen
                Eigen::Quaternion<float,Eigen::AutoAlign> controller_orientation(pos.pose.orientation.w,pos.pose.orientation.x,pos.pose.orientation.y,pos.pose.orientation.z);
                Eigen::Vector3f controller_position;
                controller_position << pos.pose.position.x, pos.pose.position.y, pos.pose.position.z;

                //create Rotate identity matrix by quaternion
                Eigen::Transform<float,3,Eigen::Affine> Twc;
                Twc.setIdentity();
                Twc.rotate(controller_orientation);

                //compute rotation about y axis in degrees
                float ang = atan2(Twc.rotation()(0,0),Twc.rotation()(2,0))*(180/PI);
                std::cout << ang << std::endl;

                //translate controller
                // Twc.translate(controller_position);
                

                //get encoder data
                odrive.updateEncoderReadings(0);
                const double theta = odrive.getEncoderPosition();

                //get motor current
                odrive.updateMotorCurrent(0);
                double current = odrive.getCurrent();

                //spring displacement
                double displacement = 0;

                //engage spring
                if (ang > 0 && ang < 90) {

                    //calculate and clamp torque
                    torque = k*ang;
                    torque = std::max(0.0,std::min(torque,0.5));

                    //command motor
                    odrive.sendTorqueCommand(0,-torque);
                }

                //deactivate spring
                else odrive.sendTorqueCommand(0,0);

                //track time
                std::chrono::steady_clock::time_point loop_stop = std::chrono::steady_clock::now();
                double time_stamp = std::chrono::duration_cast<std::chrono::duration<double>>(loop_stop-program_start).count();

                 //write to csv
                if (loggingEnabled) {
                    datafile << time_stamp << "," << current << "," << torque << "," << ang << "\n";
                }
            }
            // Throttle loop since xrWaitFrame won't be called.
            else std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }

    } while (requestRestart);

    

    
    while(true) {
        
    }
    
    return 1;
}
