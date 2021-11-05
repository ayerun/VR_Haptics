#ifndef MOTOR_COMMUNICATION_GUARD
#define MOTOR_COMMUNICATION_GUARD

#include <nuhal/uart.h>
#include <string>

class Odrive {
    public:

    Odrive(const std::string &name, unsigned int baud) ;

    bool updateVoltage();
    bool updateEncoderReadings(int motor);
    bool zeroEncoderPosition(int motor);
    bool sendTorqueCommand(int motor, double torque);
    bool setClosedLoopControl(int motor);
    bool setTorqueControlMode(int motor);
    bool runFullCalibration(int motor);
    bool runEncoderOffsetCalibration(int motor);
    bool updateMotorCurrent(int motor);
    
    double getCurrent();
    double getEncoderInitial();
    double getEncoderPosition();
    double getEncoderVelocity();
    double getVoltage();
    double getInputTorque();

    private:

    bool writeToBoard(const std::string& str, uint32_t timeout);

    const uart_port* board;
    double voltage = 0;
    double input_torque = 0;
    double current = 0;
    double encoder_position;
    double encoder_initial;
    double encoder_velocity;
};

#endif