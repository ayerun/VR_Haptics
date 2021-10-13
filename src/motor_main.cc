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

namespace {

<<<<<<< HEAD
    void ShowHelp() {
        // TODO: Improve/update when things are more settled.
        Log::Write(Log::Level::Info,
                "scene --graphics|-g <Graphics API> [--formfactor|-ff <Form factor>] [--viewconfig|-vc <View config>] "
                "[--blendmode|-bm <Blend mode>] [--space|-s <Space>] [--verbose|-v]");
        Log::Write(Log::Level::Info, "Graphics APIs:            Vulkan2");
        Log::Write(Log::Level::Info, "Form factors:             Hmd");
        Log::Write(Log::Level::Info, "View configurations:      Stereo");
        Log::Write(Log::Level::Info, "Environment blend modes:  Opaque, Additive, AlphaBlend");
        Log::Write(Log::Level::Info, "Spaces:                   View, Local, Stage");
=======
void ShowHelp() {
    // TODO: Improve/update when things are more settled.
    Log::Write(Log::Level::Info,
               "scene --graphics|-g <Graphics API> [--formfactor|-ff <Form factor>] [--viewconfig|-vc <View config>] "
               "[--blendmode|-bm <Blend mode>] [--space|-s <Space>] [--verbose|-v]");
    Log::Write(Log::Level::Info, "Graphics APIs:            Vulkan2");
    Log::Write(Log::Level::Info, "Form factors:             Hmd");
    Log::Write(Log::Level::Info, "View configurations:      Stereo");
    Log::Write(Log::Level::Info, "Environment blend modes:  Opaque, Additive, AlphaBlend");
    Log::Write(Log::Level::Info, "Spaces:                   View, Local, Stage");
}

bool UpdateOptionsFromCommandLine(Options& options) {
    // Set graphics plugin to Vulkan2 by default
    if (options.GraphicsPlugin.empty()) {
        options.GraphicsPlugin = "Vulkan2";
    }

    // Set form factor to hmd by default
    if (options.FormFactor.empty()) {
        options.FormFactor = "Hmd";
    }

    // Set view configuration to stereo by defaul
    if (options.ViewConfiguration.empty()) {
        options.ViewConfiguration = "Stereo";
>>>>>>> origin/dev
    }
}  // namespace

int main(int argc, char* argv[]) {
    //get command line args (port and buad rate)
    QCoreApplication coreApplication(argc, argv);
    const int argumentCount = QCoreApplication::arguments().size();
    const QStringList argumentList = QCoreApplication::arguments();

    //expection: incorrect number of command line args
    QTextStream standardOutput(stdout);
    if (argumentCount == 1) {
        standardOutput << QObject::tr("Usage: %1 <serialportname> [baudrate]")
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
    board.sendTorqueCommand(0,0);

    // Spawn a thread to wait for a keypress
    static bool quitKeyPressed = false;
    auto exitPollingThread = std::thread{[] {
        Log::Write(Log::Level::Info, "Press any key to shutdown...");
        (void)getchar();
        quitKeyPressed = true;
    }};
    exitPollingThread.detach();

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

            while (!quitKeyPressed) {
                bool exitRenderLoop = false;
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

        } while (!quitKeyPressed && requestRestart);
    }};
    renderThread.detach();
    

    //exit application
    return coreApplication.exec();
}
