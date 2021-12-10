#ifndef MOTOR_COMMUNICATION_GUARD
#define MOTOR_COMMUNICATION_GUARD

/// \file
/// \brief Library to communicate with ODrive

#include <nuhal/uart.h>
#include <string>


/// \brief ODrive device object
class Odrive {

    public:

    /// \brief creates an ODrive object
    /// \param name - portname
    /// \param baud - baudrate
    Odrive(const std::string &name, unsigned int baud) ;

    /// \brief update voltage member
    /// \returns true if successful
    bool updateVoltage();

    /// \brief update encoder position member
    /// \param motor - integer corresponding to motor (0 or 1)
    /// \returns true if successful
    bool updateEncoderReadings(int motor);

    /// \brief reset encoder initial member
    /// \param init_pos - value to reset to in revolutions
    /// \param motor - integer corresponding to motor (0 or 1)
    /// \returns true if successful
    bool zeroEncoderPosition(int motor, double init_pos=0);

    /// \brief send torque command to motor
    /// \param motor - integer corresponding to motor (0 or 1)
    /// \param torque - torque command
    /// \returns true if successful
    bool sendTorqueCommand(int motor, double torque);

    /// \brief set motor to torque control mode
    /// \param motor - integer corresponding to motor (0 or 1)
    /// \returns true if successful
    bool setTorqueControlMode(int motor);

    /// \brief update current member
    /// \param motor - integer corresponding to motor (0 or 1)
    /// \returns true if successful
    bool updateMotorCurrent(int motor);
    
    /// \brief current getter function
    double getCurrent();

    /// \brief encoder initial getter function
    double getEncoderInitial();

    /// \brief encoder position getter function
    double getEncoderPosition();

    /// \brief encoder velocity getter function
    double getEncoderVelocity();
    
    /// \brief voltage getter function
    double getVoltage();

    /// \brief torque getter function
    double getInputTorque();

    private:

    /// \brief serially send information to ODrive
    /// \param str - data to send
    /// \param timeout - max time limit to send data
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