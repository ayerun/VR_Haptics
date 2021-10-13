#include <motor_communication.hpp>
#include <QList>
#include <QCoreApplication>

odrive::odrive(QSerialPort* serialPort, QObject* parent) :
    QObject(parent),
    serial(serialPort),
    standardOutput(stdout)
{
    connect(serialPort, &QSerialPort::bytesWritten,this, &odrive::handleBytesWritten);
    connect(serialPort, &QSerialPort::errorOccurred, this, &odrive::handleError);
    connect(&w_timer, &QTimer::timeout, this, &odrive::writeTimeout);
}

void odrive::handleError(QSerialPort::SerialPortError serialPortError)
{
    if (serialPortError == QSerialPort::WriteError)
    {
        standardOutput << QObject::tr("An I/O error occurred while writing"
                                        " the data to port %1, error: %2")
                            .arg(serial->portName())
                            .arg(serial->errorString())
                         << endl;
        QCoreApplication::exit(1);
    }
    
    else if (serialPortError == QSerialPort::ReadError)
    {
        standardOutput << QObject::tr("An I/O error occurred while reading "
                                        "the data from port %1, error: %2")
                            .arg(serial->portName())
                            .arg(serial->errorString())
                         << "\n";
        QCoreApplication::exit(1);
    }
}

void odrive::write(const QByteArray &new_writeData)
{
    writeData = new_writeData;

    const qint64 bytesWritten = serial->write(writeData);

    if (bytesWritten == -1) {
        standardOutput << QObject::tr("Failed to write the data to port %1, error: %2")
                            .arg(serial->portName())
                            .arg(serial->errorString())
                         << endl;
        QCoreApplication::exit(1);
    } else if (bytesWritten != writeData.size()) {
        standardOutput << QObject::tr("Failed to write all the data to port %1, error: %2")
                            .arg(serial->portName())
                            .arg(serial->errorString())
                         << endl;
        QCoreApplication::exit(1);
    }
    w_timer.start(10);
}

void odrive::handleBytesWritten()
{
    w_timer.stop();
}

void odrive::writeTimeout()
{
    standardOutput << QObject::tr("Operation timed out for port %1, error: %2")
                        .arg(serial->portName())
                        .arg(serial->errorString())
                     << endl;
    QCoreApplication::exit(1);
}


double odrive::readVoltage()
{
    //initialize command
    QString str("r vbus_voltage ");

    //append checksum
    int checksum = xorAscii(str.toStdString(),str.length());
    const QString checksum_str = QString("*%1").arg(checksum);
    str.append(checksum_str);

    //append enter
    const QChar enter(13);
    str.append(enter);

    //convert and write
    const QByteArray command = str.toUtf8();
    write(command);

    //read
    if(serial->waitForReadyRead(5))
    {
        readData = serial->readAll();
        double voltage = readData.toDouble();
        if(serial->waitForReadyRead(5))
        {
            readData = serial->readAll();
        }
        return voltage;
    }
    else
    {
        standardOutput << QObject::tr("No data was currently available for reading from port %1")
                        .arg(serial->portName())
                         << "\n";
        return -1;
    }
}

void odrive::setClosedLoopControl(int motor)
{
    QString str = QString("w axis%1.requested_state 8").arg(motor);
    const QChar enter(13);
    str.append(enter);
    const QByteArray command = str.toUtf8();
    write(command);
}

int odrive::readState(int motor)
{
    QString str = QString("r axis%1.current_state").arg(motor);
    const QChar enter(13);
    str.append(enter);
    const QByteArray command = str.toUtf8();
    write(command);

    //read
    if(serial->waitForReadyRead(5))
    {
        readData = serial->readAll();
        int state = readData.toInt();
        return state;
    }
    else
    {
        standardOutput << QObject::tr("No data was currently available for reading from port %1")
                        .arg(serial->portName())
                         << "\n";
        return -1;
    }
}

void odrive::setTorqueControlMode(int motor)
{
    QString str = QString("w axis%1.controller.config.control_mode 1").arg(motor);
    const QChar enter(13);
    str.append(enter);
    const QByteArray command = str.toUtf8();
    write(command);
}

int odrive::readControlMode(int motor)
{
    QString str = QString("r axis%1.controller.config.control_mode").arg(motor);
    const QChar enter(13);
    str.append(enter);
    const QByteArray command = str.toUtf8();
    write(command);

    //read
    if(serial->waitForReadyRead(5))
    {
        readData = serial->readAll();
        int mode = readData.toInt();
        return mode;
    }
    else
    {
        standardOutput << QObject::tr("No data was currently available for reading from port %1")
                        .arg(serial->portName())
                         << "\n";
        return -1;
    }
}

void odrive::sendMotorTrajectory(int motor, double destination)
{
    QString str = QString("t %1 %2").arg(motor).arg(destination);
    const QChar enter(13);
    str.append(enter);
    const QByteArray command = str.toUtf8();
    write(command);
}

void odrive::sendMotorPosition(int motor, double position, double velocity_lim, double torque_lim)
{
    QString str = QString("q %1 %2 %3 %4").arg(motor).arg(position).arg(velocity_lim).arg(torque_lim);
    const QChar enter(13);
    str.append(enter);
    const QByteArray command = str.toUtf8();
    write(command);
}

void odrive::streamMotorPosition(int motor, double position, double velocity_ff, double torque_ff)
{
    QString str = QString("p %1 %2 %3 %4").arg(motor).arg(position).arg(velocity_ff).arg(torque_ff);
    const QChar enter(13);
    str.append(enter);
    const QByteArray command = str.toUtf8();
    write(command);
}

void odrive::sendVelocityCommand(int motor, double velocity, double torque_ff)
{
    //initialize command
    QString str = QString("v %1 %2 %3 ").arg(motor).arg(velocity).arg(torque_ff);

    //append enter
    const QChar enter(13);
    str.append(enter);

    //convert and write
    const QByteArray command = str.toUtf8();
    write(command);
}

void odrive::sendTorqueCommand(int motor, double torque)
{
    QString str = QString("c %1 %2").arg(motor).arg(torque);
    const QChar enter(13);
    str.append(enter);
    const QByteArray command = str.toUtf8();
    write(command);
}

double* odrive::requestFeedback(int motor)
{
    QString str = QString("f %1").arg(motor);
    const QChar enter(13);
    str.append(enter);
    const QByteArray command = str.toUtf8();
    write(command);
    static double response[2];
    if(serial->waitForReadyRead(5))
    {
        readData = serial->readAll();

        char separator = 32;
        const QList<QByteArray> feedback = readData.split(separator);

        response[0] = feedback[0].toDouble();
        response[1] = feedback[1].toDouble();

        return response;
    }
    else
    {
        standardOutput << QObject::tr("No data was currently available for reading from port %1")
                        .arg(serial->portName())
                         << "\n";

        response[0] = -1;
        response[1] = -1;
        return response;
    }
}

QByteArray odrive::get_readData()
{
    return readData;
}

int xorAscii(std::string str, int len)
{
    //store value of first character
    int ans = int(str[0]);
    
    //calculate XOR
    for (int i = 1; i < len; i++)
    {
 
        ans = (ans ^ (int(str[i])));
    }

    return ans;
}