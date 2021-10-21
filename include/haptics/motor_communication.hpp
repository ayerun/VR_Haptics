#ifndef motor_communication
#define motor_communication

#include <QSerialPort>
#include <QTextStream>
#include <QTimer>
#include <QByteArray>
#include <QObject>

/// \brief A class for ODrive communication
class odrive : public QObject
{
    Q_OBJECT

    public:

        /// \brief constructor
        /// \param serialPort - serial port for communication
        /// \param parent - parent of object
        explicit odrive(QSerialPort* serialPort, QObject* parent = nullptr);

        /// \brief writes data to serial port
        /// \param new_writeData - data to write
        void write(const QByteArray &new_writeData);

        /// \brief set state of specifiedmotor to closed loop control
        /// \param motor - motor number (0 or 1)
        void setClosedLoopControl(int motor);

        /// \brief reads the current state of the specified motor
        /// \param motor - motor number (0 or 1)
        /// \returns integer corresponding to current state
        int readState(int motor);

        /// \brief sets the controller mode of the specified motor to torque control
        /// \param motor - motor number (0 or 1)
        void setTorqueControlMode(int motor);

        /// \brief reads the controller mode of the specified motor
        /// \param motor - motor number (0 or 1)
        /// \returns integer corresponding to controller mode
        int readControlMode(int motor);

        /// \brief command a motor to move to specified position
        /// \param motor - motor number (0 or 1)
        /// \param destination - desired position (revolutions)
        void sendMotorTrajectory(int motor, double destination);

        /// \brief used when sending one motor position setpoint at a time
        /// \param motor - motor number (0 or 1)
        /// \param position - desired position (revolutions)
        /// \param velocity_lim - velocity limit (revolutions/second), set to 0 if no limit
        /// \param torque_lim - torque limit (Nm), set to 0 if no limit
        void sendMotorPosition(int motor, double position, double velocity_lim = 0, double torque_lim = 0);

        /// \brief used when streaming motor positions and tracking trajectory
        /// \param motor - motor number (0 or 1)
        /// \param position - desired position (revolutions)
        /// \param velocity_ff - velocity feed-forward term (revolutions/second), set to 0 to omit
        /// \param torque_ff - torque feed-forward term (Nm), set to 0 to omit
        void streamMotorPosition(int motor, double position, double velocity_ff = 0, double torque_ff = 0);

        /// \brief sends a velocity command to motor
        /// \param motor - motor number (0 or 1)
        /// \param velocity - desired velocity (revolutions/second)
        /// \param torque_ff - torque feed-forward term (Nm), set to 0 to omit
        void sendVelocityCommand(int motor, double velocity, double torque_ff = 0);

        /// \brief sends a torque command to motor
        /// \param motor - motor number (0 or 1)
        /// \param torque - desired torque (Nm)
        void sendTorqueCommand(int motor, double torque);

        /// \brief requests position and velocity information from motor
        /// \param motor - motor number (0 or 1)
        /// \returns - list of doubles in format [position, velocity]
        void requestFeedback();

        /// \brief read odrive voltage
        /// \returns voltage (V)
        double readVoltage();

        /// \brief accessor function for readData
        /// \returns readData
        QByteArray get_readData();

    private slots:

        /// \brief stops w_timer after writing the serial port
        void handleBytesWritten();

        /// \brief callback when w_timer reaches time limit
        void writeTimeout();

        /// \brief exception handling for serial port communications
        /// \param error - error message from QSerialPort object
        void handleError(QSerialPort::SerialPortError error);

    private:
        QSerialPort* serial = nullptr;      //serial port
        QTextStream standardOutput;         //qt's cout
        QByteArray writeData;               //data transmitted through serial port
        QByteArray readData;                //data recieved through serial port
        qint64 bytesWritten = 0;            //number of bytes written to serial port
        QTimer w_timer;                     //timer for writing
        QTimer feedback_timer;              //timer for encoder feedback
        double encoderPos = 0;              //encoder position
        double encoderVel = 0;              //encoder velocity
};

/// \brief calculate the xor of an ascii string
/// \param str - input string
/// \param len - string length
/// \returns xor of the input string
int xorAscii(std::string str, int len);

#endif