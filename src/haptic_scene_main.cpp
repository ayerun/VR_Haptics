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
        double k = 0.1666667;
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

                //convert openXR types to Eigne
                Eigen::Quaternion<float,Eigen::AutoAlign> controller_orientation(pos.pose.orientation.w,pos.pose.orientation.x,pos.pose.orientation.y,pos.pose.orientation.z);
                Eigen::Vector3f controller_position;
                controller_position << pos.pose.position.x, pos.pose.position.y, pos.pose.position.z;

                //create transformation matrix
                Eigen::Transform<float,3,Eigen::Affine> Twc;
                Twc.setIdentity();
                Twc.rotate(controller_orientation);
                Twc.translate(controller_position);
                std::cout << Twc.translation() << std::endl << std::endl;
                

                //get encoder data
                odrive.updateEncoderReadings(0);
                const double theta = odrive.getEncoderPosition();

                //spring displacement
                double displacement = 0;

                //engage spring
                // if (pitch < 0 && pitch > -45) {
                    
                //     Log::Write(Log::Level::Error, "Pitch: " + std::to_string(pitch));

                //     //calculate and clamp torque
                //     torque = k*pitch;
                //     torque = std::min(0.0,std::max(torque,-0.5));

                //     //command motor
                //     odrive.sendTorqueCommand(0,torque);

                //     //get motor current
                //     odrive.updateMotorCurrent(0);
                //     double current = odrive.getCurrent();
                // }

                // //deactivate spring
                // else odrive.sendTorqueCommand(0,0);
            }
            // Throttle loop since xrWaitFrame won't be called.
            else std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }

    } while (requestRestart);

    

    
    while(true) {
        
    }
    
    return 1;
}
