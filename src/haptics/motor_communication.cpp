#include <motor_communication.hpp>
#include <iostream>

Odrive::Odrive(const std::string &name, unsigned int baud) {
    board = uart_open(name.c_str(), baud, UART_FLOW_NONE, UART_PARITY_NONE);
}

bool Odrive::updateVoltage() {
    std::string str = "r vbus_voltage";
    bool response = writeToBoard(str,100);
    
    if (response) {
        char readData[11];
        bool dataReady = uart_wait_for_data(board,100);
        if (dataReady) {
            int readResponse = uart_read_block(board,&readData,sizeof(readData),1000,UART_TERM_LF);
            if (isdigit(readData[0])) {
                voltage = std::stod(readData);
                return true;
            }
            else {
                // std::cout << "Voltage read write transaction failed" << std::endl;
                return false;
            }
        }
        else {
            std::cout << "No voltage data available" << std::endl;
            return false;
        }
    }
    else {
        std::cout << "Failed to write to odrive" << std::endl;
        return false;
    }
}

bool Odrive::zeroEncoderPosition(int motor) {
    std::string str = "f " + std::to_string(motor);
    bool response = writeToBoard(str,100);
    if (response) {
        char readData[30];
        bool dataReady = uart_wait_for_data(board,100);
        if (dataReady) {
            int readResponse = uart_read_block(board,&readData,sizeof(readData),1000,UART_TERM_LF);
            if (isdigit(readData[0])) {
                encoder_initial = std::stod(readData);
                encoder_position = 0;    
                return true;
            }
            else {
                // std::cout << "Zero encoder read write transaction failed" << std::endl;
                return false;
            }
        }
        else {
            std::cout << "No encoder data available" << std::endl;
            return false;
        }
    }
    else {
        std::cout << "Failed to write to odrive" << std::endl;
        return false;
    }
}

bool Odrive::updateEncoderReadings(int motor) {
    std::string str = "f " + std::to_string(motor);
    bool response = writeToBoard(str,100);
    if (response) {
        char readData[30];
        bool dataReady = uart_wait_for_data(board,100);
        if (dataReady) {
            int readResponse = uart_read_block(board,&readData,sizeof(readData),1000,UART_TERM_LF);
            std::string encoderData(readData);
            int space = encoderData.find(" ");
            std::string encoderVelocity = encoderData.substr(space+1,7);
            if (isdigit(readData[0]) && encoderVelocity.size() > 1) {
                encoder_position = std::stod(encoderData)-encoder_initial;
                encoder_velocity = std::stod(encoderVelocity);
                return true;
            }
            else {
                // std::cout << "Encoder read write transaction failed" << std::endl;
                return false;
            }
        }
        else {
            std::cout << "No encoder data available" << std::endl;
            return false;
        }
    }
    else {
        std::cout << "Failed to write to odrive" << std::endl;
        return false;
    }
}

bool Odrive::sendTorqueCommand(int motor, double torque) {
    std::string str = "c " + std::to_string(motor) + " " + std::to_string(torque);
    bool response = writeToBoard(str,100);
    if (response) {
        input_torque = torque;
    }
    return response;
}

bool Odrive::setClosedLoopControl(int motor) {
    std::string str = "w axis" + std::to_string(motor) + ".requested_state=8";
    bool response = writeToBoard(str,100);
    return response;
}

bool Odrive::setTorqueControlMode(int motor) {
    std::string str = "w axis" + std::to_string(motor) + ".controller.config.control_mode=1";
    bool response = writeToBoard(str,100);
    return response;
}

bool Odrive::runFullCalibration(int motor) {
    std::string str = "w axis" + std::to_string(motor) + ".requested_state=3";
    bool response = writeToBoard(str,100);
    return response;
}

bool Odrive::runEncoderOffsetCalibration(int motor) {
    std::string str = "w axis" + std::to_string(motor) + ".requested_state=7";
    bool response = writeToBoard(str,100);
    return response;
}

bool Odrive::updateMotorCurrent(int motor) {
    std::string str = "r axis" + std::to_string(motor) + ".motor.current_control.Iq_measured";
    bool response = writeToBoard(str,100);

    if (response) {
        char readData[11];
        bool dataReady = uart_wait_for_data(board,100);
        if (dataReady) {
            int readResponse = uart_read_block(board,&readData,sizeof(readData),1000,UART_TERM_LF);
            if(isdigit(readData[0])) {
                current = std::stod(readData);
                return true;
            }
            else {
                // std::cout << "Current read write transaction failed" << std::endl;
                return false;
            }
        }
        else {
            std::cout << "No current data available" << std::endl;
            return false;
        }
    }
    else {
        std::cout << "Failed to write to odrive" << std::endl;
        return false;
    }
}

double Odrive::getCurrent() {
    return current;
}

double Odrive::getVoltage() {
    return voltage;
}

double Odrive::getInputTorque() {
    return input_torque;
}

double Odrive::getEncoderInitial() {
    return encoder_initial;
}

double Odrive::getEncoderPosition() {
    return encoder_position;
}

double Odrive::getEncoderVelocity() {
    return encoder_velocity;
}

//use string view
bool Odrive::writeToBoard(const std::string& str, uint32_t timeout) {

    const auto strn = str+"\n";

    int response = uart_write_block(board,strn.c_str(),strn.size(),timeout);

    if (response == strn.size()) {
        return true;
    }
    else {
        return false;
    }
}