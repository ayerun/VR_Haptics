#include <nuhal/uart.h>
#include <nuhal/uart_linux.h>
#include <motor_communication.hpp>
#include <iostream>

int main(int argc, char* argv[]) {
    Odrive odrive("/dev/ttyACM1", 115200);
    odrive.sendTorqueCommand(0,0);
    return 0;
}