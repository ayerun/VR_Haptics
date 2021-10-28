#include "pch.h"
#include "common.h"
#include "options.h"
#include "platformplugin.h"
#include "graphicsplugin.h"
#include "openxr_program.h"
#include "motor_communication.hpp"
#include <signal.h>

void signal_callback(int signum) {
   std::cout << "Caught signal " << signum << std::endl;
   std::cout << "Terminating Program" << std::endl;
//    exit(signum);
}

void renderLoop(double* yaw) {
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
                *yaw = std::get<2>(rpy);
                // Log::Write(Log::Level::Error, "Roll: " + std::to_string(std::get<0>(rpy)));
                // Log::Write(Log::Level::Error, "Pitch: " + std::to_string(std::get<1>(rpy)));
                Log::Write(Log::Level::Error, "Yaw: " + std::to_string(*yaw));
            } else {
                // Throttle loop since xrWaitFrame won't be called.
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }
        }

    } while (requestRestart);
}

int main(int argc, char* argv[]) {
    // Register signal and signal callback
    signal(SIGINT, signal_callback);

    double yaw = -100.0;

    //Render in separate thread
    auto renderThread = std::thread(renderLoop,&yaw);
    renderThread.detach();

    //Encoder Spring
    Odrive odrive("/dev/ttyACM1", 115200);
    odrive.zeroEncoderPosition(0);
    odrive.setClosedLoopControl(0);

    double k = 0.5;
    double torque = 0;
    while(true) {
        double theta = yaw/(4.0*PI)+0.5;

        //engage spring
        if (theta > 0.4) {

            //calculate and clamp torque
            theta -= 0.4;
            torque = k*theta;
            torque = std::max(0.0,std::min(torque,0.5));

            //command motor
            odrive.sendTorqueCommand(0,-torque);

            //get motor current
            // odrive.updateMotorCurrent(0);
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
    
    return 1;
}
