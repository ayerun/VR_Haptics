#include "pch.h"
#include "common.h"
#include "options.h"
#include "platformplugin.h"
#include "graphicsplugin.h"
#include "openxr_program.h"
#include <motor_communication.hpp>
#include <fstream>
#include <haptics.hpp>
#include <Eigen/Geometry>
#include <float.h>

double calculateVelocity(double z_pos) {
    static double z_i = 0;
    static bool initialized = false;
    static std::chrono::steady_clock::time_point time_start;

    if (!initialized) {
        z_i = z_pos;
        time_start = std::chrono::steady_clock::now();
        initialized = true;
        return 0;
    }

    //calculate change in pose
    double dz = abs(z_pos-z_i);
    z_i = z_pos;

    //calculate change in time
    std::chrono::steady_clock::time_point time_stop = std::chrono::steady_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::duration<double>>(time_stop-time_start).count();
    time_start = time_stop;
    
    return dz/dt;
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
    int hand = Side::RIGHT;
    double alpha = 0.5;                         //exponential filter alpha
    float pointer_length = 0.15;                //end of drum stick

    //Drum Constants
    double snare_length = 0.4;                  //length of drum
    double snare_width = 0.4;                   //width of drum
    double snare_k = 600;
    Eigen::Vector3f snare_center;               //center coordinates of drum
    snare_center << 0, 0, 0.1;
    std::pair<int,int> snare_sustain_limits{0,500};
    std::pair<int,int> snare_level_limits{0,3};
    bool playDrum = true;

    Drum snare(0,snare_center,snare_length,snare_width,snare_k,snare_sustain_limits,snare_level_limits);

    //Odrive port
    std::string portname;
    std::string default_port = "/dev/ttyACM1";

    //Parse command line arguements
    if (argc == 1) {
        portname = default_port;
    }
    else if (argc == 2) {
        portname = argv[1];
    } 
    else {
        std::cout << "Invalid number of command line arguements" << std::endl;
        return 0;
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

    //initialize exponential filter
    ExponentialFilter ef = ExponentialFilter(3,alpha);

    bool exitRenderLoop = false;
    bool requestRestart = false;
    while (!exitRenderLoop) {
        program->PollEvents(&exitRenderLoop, &requestRestart);
        if (exitRenderLoop || requestRestart) {
            break;
        }

        if (program->IsSessionRunning()) {
            program->PollActions();

            //Render and get controller data
            XrTime displayTime = program->RenderFrame();
            XrSpaceLocation pos = program->getControllerSpace(displayTime, hand);

            //Create controller tranformation matrix
            auto Twc = toTransform(pos.pose);

            //rotate controller to make +Z up
            Eigen::Matrix3f rot;
            rot <<  1,0,0,
                    0,-1,0,
                    0,0,-1;
            Twc.rotate(rot);

            //Define w_ frame at controller start position
            if (!originSet && program->isHandActive(hand)) {
                Tww_ = Twc;
                originSet = true;
            }

            else if (originSet) {

                //calculate controller position in w_ frame
                auto Tw_c = Tww_.inverse()*Twc;
                double controller_height = Tw_c.translation()(2);

                //calculate drumstick position in w_ frame
                Eigen::Transform<float,3,Eigen::Affine> Tcp;
                Tcp.setIdentity();
                Eigen::Vector3f translation;
                translation << 0.0, 0.0, pointer_length;
                Tcp.translate(translation);
                auto Tw_p = Tw_c*Tcp;

                //get drumstick position
                std::vector<double> drumstick_pos;
                for (int i=0; i<3; i++) drumstick_pos.push_back(Tw_p.translation()(i));

                //exponential smoothing
                ef.filterData(drumstick_pos);
                auto filtered_drumstick_pos = ef.getForcastFloat();

                //get drumstick velocity
                double vel = calculateVelocity(filtered_drumstick_pos[2]);

                //calculate torque and command motor
                double torque = snare.update(filtered_drumstick_pos,vel);
                odrive.sendTorqueCommand(0,torque);

            }
            
        }
        // Throttle loop since xrWaitFrame won't be called.
        else std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    
    return 1;
}
