#include "pch.h"
#include "common.h"
#include "options.h"
#include "platformplugin.h"
#include "graphicsplugin.h"
#include "openxr_program.h"
#include "motor_communication.hpp"
#include <fstream>

int main(int argc, char* argv[]) {

    //logging
    std::string filename;
    std::ofstream datafile;
    bool loggingEnabled = false;

    //Odrive port
    std::string portname;
    std::string default_port = "/dev/ttyACM2";
    
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

    if (loggingEnabled) {    
        datafile.open(filename);
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
        double k = 0.005;
        double torque = 0;

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
                auto rpy = quaternion2rpy(pos.pose.orientation);

                double roll = std::get<0>(rpy)*(180/PI);
                double pitch = std::get<1>(rpy)*(180/PI);   //this is it
                double yaw = std::get<2>(rpy)*(180/PI);
                // double theta = yaw/(4.0*PI)+0.5;
                // Log::Write(Log::Level::Error, "Roll: " + std::to_string(roll));
                // Log::Write(Log::Level::Error, "Pitch: " + std::to_string(pitch));
                // Log::Write(Log::Level::Error, "Yaw: " + std::to_string(yaw));

                //spring displacement
                double displacement = 0;

                //engage spring
                if (pitch > 0) {
                    
                    // Log::Write(Log::Level::Error, "Pitch: " + std::to_string(pitch));

                    //calculate and clamp torque
                    torque = k*pitch;
                    torque = std::max(0.0,std::min(torque,0.5));

                    //command motor
                    odrive.sendTorqueCommand(0,-torque);

                    //get motor current
                    odrive.updateMotorCurrent(0);
                    double current = odrive.getCurrent();
                }

                //deactivate spring
                else {
                    if(odrive.getInputTorque() != 0) {
                        odrive.sendTorqueCommand(0,0);
                    }
                }
            }
            else {
                // Throttle loop since xrWaitFrame won't be called.
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }
        }

    } while (requestRestart);

    

    
    while(true) {
        
    }
    
    return 1;
}
