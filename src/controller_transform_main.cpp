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

double calculateTorque(double displacement, double theta, double k) {
    if (displacement < 0) {

        //calculate toque
        double torque = abs(k*displacement);

        //clamp torque
        torque = std::min(torque,0.5);

        //reverse direction based on controller angle
        if (theta > geometry::PI/2) {
            torque *= -1;
        }
        return torque;
    }
    else return 0;
}

Eigen::Transform<float,3,Eigen::Affine> createTransform(XrPosef pose) {
    //convert openXR types to Eigen
    Eigen::Quaternion<float,Eigen::AutoAlign> controller_orientation(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
    Eigen::Vector3f controller_position(pose.position.x, pose.position.y, pose.position.z);

    //create identity matrix
    Eigen::Transform<float,3,Eigen::Affine> Twc;
    Twc.setIdentity();

    //translate then rotate to create Twc
    Twc.translate(controller_position);
    Twc.rotate(controller_orientation);
    
    return Twc;
}

int main(int argc, char* argv[]) {

    //Constants
    int hand = Side::LEFT;  //controller
    double pointer_length = 0.18;
    double floor_height = 0.1;
    double k = 100;

    //Odrive port
    std::string portname = "/dev/ttyACM1";

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

                //get encoder data
                odrive.updateEncoderReadings(0);
                const double theta = geometry::normalize_angle(geometry::rev2rad(odrive.getEncoderPosition()));

                //Render and get controller data
                XrTime displayTime = program->RenderFrame();
                XrSpaceLocation pos = program->getControllerSpace(displayTime, hand);

                //Create controller tranformation matrix
                auto Twc = createTransform(pos.pose);

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
                    double displacement = controller_height+pointer_length*sin(theta)-floor_height;
                    std::cout << displacement << std::endl;

                    double torque = calculateTorque(displacement,theta,k);
                    odrive.sendTorqueCommand(0,torque);
                }
                
            }
            // Throttle loop since xrWaitFrame won't be called.
            else std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }

    } while (requestRestart);
    
    return 1;
}
