#include <motor_communication.hpp>
#include <iostream>

Odrive::Odrive(const char name[], unsigned int baud) {
    board = uart_open(name, baud, UART_FLOW_NONE, UART_PARITY_NONE);
}

bool Odrive::updateVoltage() {
    std::string str = "r vbus_voltage";
    bool response = writeToBoard(&str,100);
    
    if (response) {
        char readData[11];
        bool dataReady = uart_wait_for_data(board,100);
        if (dataReady) {
            int readResponse = uart_read_block(board,&readData,sizeof(readData),1000,UART_TERM_LF);
            try {
                voltage = std::stod(readData);
            }
            catch (...) {
                std::cout << "Failed to read voltage" << std::endl;   
                return false;     
            }
            return true;
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
    bool response = writeToBoard(&str,100);
    if (response) {
        char readData[30];
        bool dataReady = uart_wait_for_data(board,100);
        if (dataReady) {
            int readResponse = uart_read_block(board,&readData,sizeof(readData),1000,UART_TERM_LF);
            try {
                encoder_initial = std::stod(readData);
            }
            catch(...) {
                std::cout << "Failed read encoder for zeroing" << std::endl;
                return false;
            }
            encoder_position = 0;
            return true;
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
    bool response = writeToBoard(&str,100);
    if (response) {
        char readData[30];
        bool dataReady = uart_wait_for_data(board,100);
        if (dataReady) {
            int readResponse = uart_read_block(board,&readData,sizeof(readData),1000,UART_TERM_LF);
            std::string encoderData(readData);
            int space = encoderData.find(" ");
            std::string encoderVelocity = encoderData.substr(space+1,7);
            try {
                double newPos = std::stod(encoderData)-encoder_initial;
                double newVel = std::stod(encoderVelocity);
                encoder_position = newPos;
                encoder_velocity = newVel;
            }
            catch (...) {
                std::cout << "Failed to read encoder values" << std::endl;
                return false;
            }

            return true;
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
    bool response = writeToBoard(&str,100);
    if (response) {
        input_torque = torque;
    }
    return response;
}

bool Odrive::setClosedLoopControl(int motor) {
    std::string str = "w axis" + std::to_string(motor) + ".requested_state=8";
    bool response = writeToBoard(&str,100);
    return response;
}

bool Odrive::setTorqueControlMode(int motor) {
    std::string str = "w axis" + std::to_string(motor) + ".controller.config.control_mode=1";
    bool response = writeToBoard(&str,100);
    return response;
}

bool Odrive::runFullCalibration(int motor) {
    std::string str = "w axis" + std::to_string(motor) + ".requested_state=3";
    bool response = writeToBoard(&str,100);
    return response;
}

bool Odrive::runEncoderOffsetCalibration(int motor) {
    std::string str = "w axis" + std::to_string(motor) + ".requested_state=7";
    bool response = writeToBoard(&str,100);
    return response;
}

bool Odrive::updateMotorCurrent(int motor) {
    std::string str = "r axis" + std::to_string(motor) + ".motor.current_control.Iq_measured";
    bool response = writeToBoard(&str,100);

    if (response) {
        char readData[11];
        bool dataReady = uart_wait_for_data(board,100);
        if (dataReady) {
            int readResponse = uart_read_block(board,&readData,sizeof(readData),1000,UART_TERM_LF);
            current = std::stod(readData);
            return true;
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

bool Odrive::writeToBoard(std::string* str, uint32_t timeout) {

    //Convert string to char pointer
    char* writable = new char[str->size()+1];
    std::copy(str->begin(), str->end(), writable);

    //Append '\n'
    writable[str->size()] = '\n';

    //Write to borad
    size_t len = str->size()+1;
    int response = uart_write_block(board,writable,len,timeout);

    //Deallocate memory
    delete[] writable;

    if (response == len) {
        return true;
    }
    else {
        return false;
    }
}