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
#include <float.h>

double calculateTorque(double controller_height, double controller_angle) {
    double pointer_length = 0.18;   //controller contact point
    double floor_height = 0.1;      //contact suface location
    double k = 500;                 //spring constant
    double torque_lim = DBL_MAX;    //torque limit

    double displacement = controller_height+pointer_length*sin(controller_angle)-floor_height;
    std::cout << displacement << std::endl;

    if (displacement < 0) {

        //square displacement to make K units N/m
        displacement = pow(displacement,2);

        //calculate toque
        double torque = abs(k*displacement);

        //clamp torque
        torque = std::min(torque,torque_lim);

        //reverse direction based on controller angle
        if (controller_angle > geometry::PI/2) {
            torque *= -1;
        }
        return torque;
    }
    else return 0;
}

Eigen::Transform<float,3,Eigen::Affine> toTransform(XrPosef& pose) {
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

std::shared_ptr<IOpenXrProgram> initializeProgram() {
    // Set graphics plugin, VR form factor, and VR view configuration
    std::shared_ptr<Options> options = std::make_shared<Options>();
    options->GraphicsPlugin = "Vulkan2";
    options->FormFactor = "Hmd";
    options->ViewConfiguration = "Stereo";
    
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

    return program;
}
 
int main(int argc, char* argv[]) {

    //Constants
    int hand = Side::LEFT;
    double alpha = 0.5;
    bool drum_contact = false;

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

    //Write first line of log
    if (loggingEnabled) {
        datafile.open(filename);
        datafile << "Time (s)" << "," << " Controller Height (m)" << "\n";
    }

    //Odrive setup
    Odrive odrive(portname, 115200);
    odrive.zeroEncoderPosition(0,0.25);

    //initialize openXR program
    auto program = initializeProgram();

    //world to world prime
    Eigen::Transform<float,3,Eigen::Affine> Tww_;
    Tww_.setIdentity();
    bool originSet = false;

    //start timer
    std::chrono::steady_clock::time_point program_start = std::chrono::steady_clock::now();
    double last_time = 0;
    double last_height = 0;
    int count =0;

    //initialize exponential filter
    ExponentialFilter ef = ExponentialFilter(alpha);

    bool exitRenderLoop = false;
    bool requestRestart = false;
    while (!exitRenderLoop) {
        program->PollEvents(&exitRenderLoop, &requestRestart);
        if (exitRenderLoop || requestRestart) {
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
            auto Twc = toTransform(pos.pose);

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
                double controller_height = Tw_c.translation()(2);

                //track time
                std::chrono::steady_clock::time_point loop_stop = std::chrono::steady_clock::now();
                double time_stamp = std::chrono::duration_cast<std::chrono::duration<double>>(loop_stop-program_start).count();

                //exponential smoothing
                controller_height = ef.filterData(controller_height);

                //calculate torque and command motor
                double torque = calculateTorque(controller_height,theta);
                odrive.sendTorqueCommand(0,torque);

                 //write to csv
                if (loggingEnabled) {
                    datafile << time_stamp << "," << controller_height << "\n";
                }
            }
            
        }
        // Throttle loop since xrWaitFrame won't be called.
        else std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    
    return 1;
}
