# VR Haptic Drum Kit
Author: Arun Kumar

## Overview
This repository contains the software for a VR Haptic drum kit. 
<a href="https://ayerun.github.io/Portfolio/haptics.html" target="_blank">Click Here For An In-Depth Project Summary</a>

## Hardware Dependencies
This software is meant for use with the Valve Index, SteamVR Base Station 2.0, and ODrive.

## Software Dependencies
* OpenXR
* Vulkan
* Eigen3
* nuhal

## File Structure
* data
    * logged data from haptic experiments
    * each folder within data has a python script that is used to plot the csv files
* drum
    * Pure Data drum synthesizer
    * samples used in synthesizer
* include
    * graphics
        * header files for OpenXR program
        * these files are stripped back versions of the <a href="https://github.com/KhronosGroup/OpenXR-SDK-Source/tree/master/src/tests/hello_xr" target="_blank">hello_xr OpenXR example</a>
    * haptics
        * header files for motor communication and haptic modeling
* src
    * graphics
        * source files for OpenXR program
        * these files are stripped back versions of the <a href="https://github.com/KhronosGroup/OpenXR-SDK-Source/tree/master/src/tests/hello_xr" target="_blank">hello_xr OpenXR example</a>
    * haptics
        * source files for motor communication and haptic modeling
* vulkan_shaders
    * shaders for OpenXR program

## Usage Instructions
1. compile
    1. make build directory `mkdir build`
    1. change directory to build `cd build`
    1. build project `cmake ..`
    1. compile project `make`
1. setup Odrive
    1. power ODrive and connect it to computer
    1. launch odrivetool in terminal `odrivetool`
    1. calibrate motor `odrv0.axis0.requested_state=AXIS_STATE_FULL_CALIBRATION_SEQUENCE`
    1. set motor in closed loop control `odrv0.axis0.requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL`
        1. There is a bug in the ODrive ASCII protocol that prevents you from setting the state. These steps cannot be done through the motor communication library.
1. launch SteamVR
1. launch drum_kit.pd `pd {project}/drum/drum_kit.pd`
1. put on haptice device
1. place controller vertical in the location you want the snare drum
1. run executable and pipe output to Pure Data `{project}/build/drumkit | pdsend 8080`

## Executables
* drumkit - haptic drum kit
    * arguement 1 - ODrive portname
    * if no arguements given, script uses the default portname
    * output must be piped to Pure Data through port 8080
* encoder_spring - 1 degree of freedom spring using encoder feedback, <a href="https://ayerun.github.io/Portfolio/haptics.html" target="_blank">see this for more details</a>
    * argurement 1 - logfile name
    * arguement 2 - portname
    * if no arguements given, script does not log data and uses the default portname
* vr_spring - 1 degree of freedom spring using vr tracking feedback, <a href="https://ayerun.github.io/Portfolio/haptics.html" target="_blank">see this for more details</a>
    * argurement 1 - logfile name
    * arguement 2 - portname
    * if no arguements given, script does not log data and uses the default portname