#include "pch.h"
#include "common.h"
#include "options.h"
#include "platformplugin.h"
#include "graphicsplugin.h"
#include "openxr_program.h"
#include "motor_communication.hpp"
#include <fstream>
#include <computational_geometry.hpp>
#include <Eigen/Geometry>

int main(int argc, char* argv[]) {

    //Constants
    int hand = Side::LEFT;  //controller
    double pointer_length = 0.25;
    double ceiling_height = 0.45;
    double k = 100;

    double torque = 0;

    //Odrive port
    std::string portname;
    std::string default_port = "/dev/ttyACM1";

    //logging
    std::string filename;
    std::ofstream datafile;
    bool loggingEnabled = false;
    
    //Parse command line arguements
    if (argc == 1) {
        portname = default_port;
    }
    else if (argc > 1) {
        std::cout << "Invalid number of command line arguements" << std::endl;
        return 0;
    }

    //Odrive setup
    Odrive odrive(portname, 115200);
    odrive.zeroEncoderPosition(0,0.25);

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

        //start timer
        std::chrono::steady_clock::time_point program_start = std::chrono::steady_clock::now();

        //world to world prime
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

                //get encoder data
                odrive.updateEncoderReadings(0);
                const double theta = geometry::normalize_angle(geometry::rev2rad(odrive.getEncoderPosition()));

                //convert openXR types to Eigen
                Eigen::Quaternion<float,Eigen::AutoAlign> controller_orientation(pos.pose.orientation.w,pos.pose.orientation.x,pos.pose.orientation.y,pos.pose.orientation.z);
                Eigen::Vector3f controller_position(pos.pose.position.x, pos.pose.position.y, pos.pose.position.z);

                //create identity matrix
                Eigen::Transform<float,3,Eigen::Affine> Twc;
                Twc.setIdentity();

                //translate then rotate to create Twc
                Twc.translate(controller_position);
                Twc.rotate(controller_orientation);

                //Define w_ frame at controller start position
                if (!originSet && program->isHandActive(hand)) {
                    //rotate controller to make +Z up
                    Eigen::Matrix3f rot;
                    rot <<  1,0,0,
                            0,-1,0,
                            0,0,-1;
                    Tww_ = Twc.rotate(rot);
                    originSet = true;
                }

                else if (originSet) {

                    //calculate controller position in w_ frame
                    auto Tw_c = Tww_.inverse()*Twc;

                    //calculate displacement
                    double controller_height = Tw_c.translation()(2);
                    double displacement = controller_height+pointer_length*sin(theta)-ceiling_height;
                    std::cout << displacement << std::endl;

                    if (displacement > 0) {

                        //calculate toque
                        torque = abs(k*displacement);

                        //clamp torque
                        torque = std::min(torque,0.5);

                        //reverse direction based on controller angle
                        if (theta < geometry::PI/2) {
                            torque *= -1;
                        }

                        //activate spring
                        odrive.sendTorqueCommand(0,torque);
                    }

                    //deactivate spring
                    else odrive.sendTorqueCommand(0,0);
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
