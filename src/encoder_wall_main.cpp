#include "motor_communication.hpp"
#include <QCoreApplication>
#include <QSerialPort>
#include <QStringList>
#include <QTextStream>
#include <QTimer>
#include <iostream>

int main(int argc, char *argv[])
{
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
    
    //exit application
    return coreApplication.exec();
}