// Copyright (c) 2017-2021, The Khronos Group Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include "pch.h"
#include "common.h"
#include "options.h"
#include "platformplugin.h"
#include "graphicsplugin.h"
#include "openxr_program.h"
#include "motor_communication.hpp"
#include <QCoreApplication>
#include <QSerialPort>
#include <QStringList>
#include <QTextStream>

int main(int argc, char* argv[]) {
    //get command line args (port and buad rate)
    QCoreApplication coreApplication(argc, argv);
    const int argumentCount = QCoreApplication::arguments().size();
    const QStringList argumentList = QCoreApplication::arguments();

    //expection: incorrect number of command line args
    QTextStream standardOutput(stdout);
    if (argumentCount == 1) {
        standardOutput << QObject::tr("Usage: %1 <serialportname>")
                        .arg(argumentList.first()) << endl;
        return 1;
    }

    //create serial port object then set name & baud rate
    QSerialPort serialPort;
    const QString serialPortName = argumentList.at(1);
    serialPort.setPortName(serialPortName);
    const int serialPortBaudRate = QSerialPort::Baud115200;
    serialPort.setBaudRate(serialPortBaudRate);

    //open port as read write
    bool result = serialPort.open(QIODevice::ReadWrite);

    //Create odrive object
    odrive board(&serialPort);

    //prepare odrive for torque control
    board.setClosedLoopControl(0);
    board.setTorqueControlMode(0);
    board.sendTorqueCommand(0,0.1);

    //Render in separate thread
    auto renderThread = std::thread{[] {
        
        // Set graphics plugin, VR form factor, and VR view configuration
        std::shared_ptr<Options> options = std::make_shared<Options>();
        options->GraphicsPlugin = "Vulkan2";
        options->FormFactor = "Hmd";
        options->ViewConfiguration = "Stereo";

        bool requestRestart = false;
        do {
            // Create platform-specific implementation.
            std::shared_ptr<IPlatformPlugin> platformPlugin = CreatePlatformPlugin(options);

            // Create graphics API implementation.
            std::shared_ptr<IGraphicsPlugin> graphicsPlugin = CreateGraphicsPlugin(options, platformPlugin);

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
                    program->RenderFrame();
                } else {
                    // Throttle loop since xrWaitFrame won't be called.
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                }
            }

        } while (requestRestart);
    }};
    renderThread.detach();
    

    //exit application
    return coreApplication.exec();
}
