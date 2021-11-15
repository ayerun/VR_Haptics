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

    //Controller
    int hand = Side::LEFT;

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

                //calculate controller position in w_ frame
                else if (originSet) {
                    auto Tw_c = Tww_.inverse()*Twc;
                    std::cout << Tw_c.translation()(2) << std::endl << std::endl;
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
