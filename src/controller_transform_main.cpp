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

    //Odrive port
    std::string portname;
    std::string default_port = "/dev/ttyACM1";

    //Controller
    int hand = Side::RIGHT;

    //Logging
    std::string filename;
    std::ofstream datafile;
    bool loggingEnabled = false;
    
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
        double k = 0.2;   //[Nm/deg]

        double torque = 0;

        if (loggingEnabled) {    
            datafile.open(filename);
            datafile << "Time (s)" << "," << " Current (A)" << "," << " Torque (Nm)" << "," << " Angle (degrees)" << "," << " K = " << k  << " (N/deg)" <<"\n";
        }

        //start timer
        std::chrono::steady_clock::time_point program_start = std::chrono::steady_clock::now();

        Eigen::Transform<float,3,Eigen::Affine> Tww_;
        Tww_.setIdentity();
        bool originSet = false;

        bool exitRenderLoop = false;
        while (!exitRenderLoop) {
            program->PollEvents(&exitRenderLoop, &requestRestart);
            if (exitRenderLoop) {
                break;
            }

            if (program->IsSessionRunning()) {
                program->PollActions();


                XrTime displayTime = program->RenderFrame();
                XrSpaceLocation pos = program->getControllerSpace(displayTime, hand);

                //convert openXR types to Eigen
                Eigen::Quaternion<float,Eigen::AutoAlign> controller_orientation(pos.pose.orientation.w,pos.pose.orientation.x,pos.pose.orientation.y,pos.pose.orientation.z);
                Eigen::Vector3f controller_position;
                controller_position << pos.pose.position.x, pos.pose.position.y, pos.pose.position.z;

                //create Rotate identity matrix by quaternion
                Eigen::Transform<float,3,Eigen::Affine> Twc;
                Twc.setIdentity();
                Twc.rotate(controller_orientation);

                //translate controller
                Twc.translate(controller_position);

                //Define w_ frame at controller start position
                if (!originSet && program->isHandActive(hand)) {
                    Tww_ = Twc;
                    originSet = true;
                }
                else if (originSet) {
                    auto Tw_c = Tww_.inverse()*Twc;
                    std::cout << Tw_c.translation() << std::endl << std::endl;
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
