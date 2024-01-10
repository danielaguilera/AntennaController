#include "fiuracontroller.h"

//////////////////////////////////////////////////
/// \brief FIURAController::FIURAController
/// \param parent
///
FIURAController::FIURAController(QObject *parent)
    : QObject{parent}
{
    m_tpo = new QElapsedTimer();
    m_trate = new QElapsedTimer();
    m_elevationPosition = 0;
    m_elevationPositionmm = 0;
    m_elevationTarget = 0;
    m_elevationMaxForce = 0;
    m_elevationKpGain = 0;
    m_elevationKiGain = 0;
    m_elevationKdGain = 0;
    m_elevationInMotion = false;
    m_elevationConnectedFlag = false;
    m_trate->start();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Métodos del motor de ronza /////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////
/// \brief FIURAController::connectRonza
/// \param ipAddress
/// \param port
///
void FIURAController::connectRonza(QString ipAddress, int port)
{
    m_ronzaConn = new QTcpSocket(this);
    m_ronzaConn->connectToHost(ipAddress, port);
    if (m_ronzaConn->state() == QAbstractSocket::ConnectedState)
    {
        qDebug() << "Ronza conectada";
        this->rebootElevationActuator();
    }
}

///////////////////////////////////////////////////////////////////
/// \brief FIURAController::ronzaConnected
/// \return
///
bool FIURAController::ronzaConnected()
{
    if (m_ronzaConn != nullptr)
    {
        return m_ronzaConn->state() == QAbstractSocket::ConnectedState;
    }
    else
    {
        return false;
    }
}

///////////////////////////////////////////////////////////////////
/// \brief FIURAController::enableRonzaDriver
///
void FIURAController::enableRonzaDriver()
{
    if (!this->ronzaConnected())
    {
        return;
    }
    m_ronzaConn->write("DRV.EN\r\n");
    m_ronzaConn->waitForBytesWritten();
}

///////////////////////////////////////////////////////////////////
/// \brief FIURAController::disableRonzaDriver
///
void FIURAController::disableRonzaDriver()
{
    if (!this->ronzaConnected())
    {
        return;
    }
    m_ronzaConn->write("DRV.DIS\r\n");
    m_ronzaConn->waitForBytesWritten();
}

//////////////////////////////////////////////
/// \brief FIURAController::stopRonza
///
void FIURAController::stopRonza()
{
    if (!this->ronzaConnected())
    {
        return;
    }
    m_ronzaConn->write("DRV.STOP\r\n");
    m_ronzaConn->waitForBytesWritten();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief FIURAController::setRonzaPositionTarget
///
void FIURAController::setRonzaPositionTarget(double targetPosition)
{
    if (!this->ronzaConnected())
    {
        return;
    }
    targetPosition = std::round(targetPosition * 100.0) / 100.0;
    m_ronzaTarget = targetPosition;
    const char* command;
    command = QString("MT.P %1\r\nMT.SET\r\nMT.MOVE 0\r\n").arg(targetPosition).toUtf8().constData();
    m_ronzaConn->write(command);
    m_ronzaConn->waitForBytesWritten();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief FIURAController::setRonzaVelocityTarget
///
void FIURAController::setRonzaVelocityTarget(double targetVelocity)
{
    if (!this->ronzaConnected())
    {
        return;
    }
    targetVelocity = std::round(targetVelocity * 100.0) / 100.0;
    const char* command;
    if (targetVelocity >= 0)
    {
        command = QString("JOG.V %1\r\nJOG.MOVEP\r\n").arg(targetVelocity).toUtf8().constData();
    }
    else
    {
        command = QString("JOG.V %1\r\nJOG.MOVEN\r\n").arg(abs(targetVelocity)).toUtf8().constData();
    }
    m_ronzaConn->write(command);
    m_ronzaConn->waitForBytesWritten();
}

////////////////////////////////////////////////////////////////////////
/// \brief FIURAController::startHomingSequence
///
void FIURAController::startRonzaHomingSequence() //EN ESTA PARTE PONER UN FLAG QUE PREGUNTE SI LLEGÓ AL HOME, Y EN CASO QUE LO HAGA, RECIÉN ALLÍ SE PUEDA PARTIR
{
    if (!this->ronzaConnected())
    {
        return;
    }
    m_ronzaConn->write("HOME.MODE 4\r\nHOME.V 1\r\nHOME.MOVE\r\n");
    m_ronzaConn->waitForBytesWritten();
}

////////////////////////////////////////////////////////////////////////
/// \brief FIURAController::presetRonzaParams
///
void FIURAController::presetRonzaParams()
{
    if (!this->ronzaConnected())
    {
        return;
    }
    m_ronzaConn->write("DRV.STOP\r\nDRV.DIS\r\nDRV.ACC 10000\r\nDRV.DEC 10000\r\nIL.LIMITP 5\r\nIL.LIMITN 5\r\nDRV.DBILIMIT 1\r\nMT.CLEAR 0\r\nMT.CLEAR 1\r\nMT.NUM 0\r\nMT.CNTL 1029\r\nMT.ACC 10000\r\nMT.DEC 10000\r\nMT.V 50\r\nMT.TNUM 0\r\nMT.SET\r\nHOME.AUTOMOVE 1\r\n");
    m_ronzaConn->waitForBytesWritten();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Métodos del motor de elevación //////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////
/// \brief FIURAController::connectElevationMotor
/// \param port
///
void FIURAController::connectElevationMotor(QString port)
{
    foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
    {
        m_elevationConn = new QSerialPort(info);
        if (m_elevationConn->open(QIODevice::ReadWrite)) {
            m_elevationConn->setPortName(port);
            m_elevationConn->setBaudRate(QSerialPort::Baud115200);
            m_elevationConn->setDataBits(QSerialPort::Data8);
            m_elevationConn->setParity(QSerialPort::NoParity);
            m_elevationConn->setStopBits(QSerialPort::OneStop);
            m_elevationConn->setFlowControl(QSerialPort::NoFlowControl);
            m_elevationConn->setReadBufferSize(32);
            m_elevationPort = port;
            connect(m_elevationConn,SIGNAL(readyRead()),this,SLOT(slot_elevationReceiveMsg()));
        } else {
            qDebug() << "Unable to open port, error code" << m_elevationConn->error();
        }
    }
    if (m_elevationConn->portName() == ELEVACION_COM_PORT)
    {
        qDebug() << "Elevación conectada";
    }
}

//////////////////////////////////////////
/// \brief FIURAController::elevationConnected
/// \return
///
bool FIURAController::elevationConnected()
{
    foreach (const QSerialPortInfo &portInfo, QSerialPortInfo::availablePorts()) {
        if (portInfo.portName() == m_elevationPort) {
            return true;
        }
    }
    return false;
}

/////////////////////////////////////////////////////
/// \brief FIURAController::setElevationKpGain
/// \param Kp
///
void FIURAController::setElevationKpGain(uint64_t Kp)
{
    QByteArray command = "kp" + QByteArray::number(Kp) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

///////////////////////////////////////////////////////
/// \brief FIURAController::setElevationKdGain
/// \param Kd
///
void FIURAController::setElevationKdGain(uint64_t Kd)
{
    QByteArray command = "kd" + QByteArray::number(Kd) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

//////////////////////////////////////////////////////
/// \brief FIURAController::setElevationKiGain
/// \param Ki
///
void FIURAController::setElevationKiGain(uint64_t Ki)
{
    QByteArray command = "ki" + QByteArray::number(Ki) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

/////////////////////////////////////////////////////
/// \brief FIURAController::requestElevationKpGain
/// \param Kp
///
void FIURAController::requestElevationKpGain()
{
    QByteArray command = "kp\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

///////////////////////////////////////////////////////
/// \brief FIURAController::requestElevationKdGain
/// \param Kd
///
void FIURAController::requestElevationKdGain()
{
    QByteArray command = "kd\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

//////////////////////////////////////////////////////
/// \brief FIURAController::requestElevationKiGain
/// \param Ki
///
void FIURAController::requestElevationKiGain()
{
    QByteArray command = "ki\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

/////////////////////////////////////////////////////
/// \brief FIURAController::requestElevationKpGain
/// \param Kp
///
int FIURAController::getElevationKpGain()
{
    return m_elevationKpGain;
}

///////////////////////////////////////////////////////
/// \brief FIURAController::requestElevationKdGain
/// \param Kd
///
int FIURAController::getElevationKdGain()
{
    return m_elevationKdGain;
}

//////////////////////////////////////////////////////
/// \brief FIURAController::requestElevationKiGain
/// \param Ki
///
int FIURAController::getElevationKiGain()
{
    return m_elevationKiGain;
}

/////////////////////////////////////////////////////
/// \brief FIURAController::setElevationSIMode
/// \param mode
///
void FIURAController::setElevationSIMode(int mode)
{
    QByteArray command = "im" + QByteArray::number(mode) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

///////////////////////////////////////////////////////
/// \brief FIURAController::disableElevationMotorCoast
///
void FIURAController::disableElevationMotorCoast()
{
    m_elevationConn->write(QByteArray("cs321\r"));
    m_elevationConn->waitForBytesWritten();
    m_elevationInMotion = true;
}

//////////////////////////////////////////////////////
/// \brief FIURAController::enableElevationMotorCoast
///
void FIURAController::enableElevationMotorCoast()
{
    m_elevationConn->write(QByteArray("cg321\r"));
    m_elevationConn->waitForBytesWritten();
    m_elevationInMotion = false;
}

///////////////////////////////////////////////////////////////////
/// \brief FIURAController::setElevationMaximumForce
/// \param maxForcekgf
///
void FIURAController::setElevationMaximumForce(double maxForcekgf)
{
    int maxTorque = (int)((1/0.0135) * (15 + maxForcekgf/0.453592));
    maxTorque = (maxTorque < 2000) ? 2000 : ((maxTorque > 10000) ? 10000 : maxTorque);
    m_elevationMaxForce = 0.453592*(0.0135*maxTorque - 15);
    QByteArray command = "mt" + QByteArray::number(maxTorque) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

///////////////////////////////////////////////////////////////////
/// \brief FIURAController::requestElevationMaximumForce
///
void FIURAController::requestElevationMaximumForce()
{
    m_elevationConn->write("mt\r");
    m_elevationConn->waitForBytesWritten();
}

///////////////////////////////////////////////////////////////////
/// \brief FIURAController::getElevationMaximumForce
///
double FIURAController::getElevationMaximumForce()
{
    return m_elevationMaxForce;
}

///////////////////////////////////////////////////////////////////////
/// \brief FIURAController::setElevationPositionTargetmm
/// \param posTargetmm
///
void FIURAController::setElevationPositionTargetmm(double posTargetmm)
{
    m_elevationTargetmm = (double)posTargetmm;
    m_elevationTarget = FIURAController::elevationMMToSteps(posTargetmm);
    QByteArray command = "pa" + QByteArray::number(m_elevationTarget) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

////////////////////////////////////////////////////////////////
/// \brief FIURAController::setElevationPositionTarget
/// \param posTarget
///
void FIURAController::setElevationPositionTarget(int posTarget)
{
    m_elevationTarget = posTarget;
    m_elevationTargetmm = FIURAController::elevationStepsToMM(posTarget);
    QByteArray command = "pa" + QByteArray::number(posTarget) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

void FIURAController::requestElevationPositionmm()
{
    m_elevationConn->write("ap\r");
    m_elevationConn->waitForBytesWritten();
}

double FIURAController::getElevationPositionmm()
{
    return m_elevationPositionmm;
}

/////////////////////////////////////////////////////////////
/// \brief FIURAController::setElevationTargetToCurrentPosition
///
void FIURAController::setElevationTargetToCurrentPosition()
{
    m_elevationTargetmm = (int)m_elevationPositionmm;
    m_elevationTarget = m_elevationPosition;
    m_elevationConn->write(QByteArray("pc\r"));
    m_elevationConn->waitForBytesWritten();
}

////////////////////////////////////////////////////////////////////
/// \brief FIURAController::enableElevationStreaming
/// \param streamFreqHz
///
void FIURAController::enableElevationStreaming(double streamFreqHz)
{
    uint64_t xt = (uint64_t)(STREAM_RATE_CONSTANT/streamFreqHz);
    QByteArray command = "x11\rxt" + QByteArray::number(xt) + "\rxg\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

/////////////////////////////////////////////////
/// \brief FIURAController::disableElevationStreaming
///
void FIURAController::disableElevationStreaming()
{
    m_elevationConn->write(QByteArray("xs\r"));
    m_elevationConn->waitForBytesWritten();
}

/////////////////////////////////////////////////
/// \brief FIURAController::rebootElevationActuator
///
void FIURAController::rebootElevationActuator()
{
    m_elevationConn->write(QByteArray("zr321\r"));
    m_elevationConn->waitForBytesWritten();
}

/////////////////////////////////////////////////
/// \brief FIURAController::slot_elevationReceiveMsg
///
void FIURAController::slot_elevationReceiveMsg()
{
    QByteArray data = m_elevationConn->readAll();
    QByteArray mode6String("A6 97");
    QByteArray mode0String("A0 91");

    qDebug() << data;

    if (data.contains(mode6String)) {
        qDebug() << "Modo 6";
    }

    if (data.contains(mode0String)) {
        qDebug() << "Modo 0";
    }


        static QRegularExpression KpRegex("Proportional Gain: (\\d+)\\r");
        QRegularExpressionMatch matchKp = KpRegex.match(data);
        if (matchKp.hasMatch()) {
            QString gainValue = matchKp.captured(1);
            m_elevationKpGain = gainValue.toInt();
        }

        static QRegularExpression KiRegex("Integral Gain: (\\d+)\\r");
        QRegularExpressionMatch matchKi = KiRegex.match(data);
        if (matchKi.hasMatch()) {
            QString gainValue = matchKi.captured(1);
            m_elevationKiGain = gainValue.toInt();
        }

        static QRegularExpression KdRegex("Derivative Gain: (\\d+)\\r");
        QRegularExpressionMatch matchKd = KdRegex.match(data);
        if (matchKd.hasMatch()) {
            QString gainValue = matchKd.captured(1);
            m_elevationKdGain = gainValue.toInt();
        }

        static QRegularExpression MTRegex("Maximum Torque: (\\d+)\\r");
        QRegularExpressionMatch matchMT = MTRegex.match(data);
        if (matchMT.hasMatch()) {
            QString gainValue = matchMT.captured(1);
            m_elevationMaxForce = 0.453592*(0.0135*gainValue.toInt() - 15);
            m_elevationMaxForce = round(m_elevationMaxForce);
        }

        static QRegularExpression PosRegex("Position: (\\d+)\\r");
        QRegularExpressionMatch matchPos = PosRegex.match(data);
        if (matchPos.hasMatch()) {
            QString gainValue = matchPos.captured(1);
            m_elevationPosition = gainValue.toInt();
            m_elevationPositionmm = FIURAController::elevationStepsToMM(m_elevationPosition);
        }

        static QRegularExpression spMinRegex("spMin: (\\d+)\\r");
        QRegularExpressionMatch matchspMin = spMinRegex.match(data);
        if (matchspMin.hasMatch()) {
            QString gainValue = matchspMin.captured(1);
            m_elevationSPMin = gainValue.toInt();
        }

        static QRegularExpression spMaxRegex("spMax: (\\d+)\\r");
        QRegularExpressionMatch matchspMax = spMaxRegex.match(data);
        if (matchspMax.hasMatch()) {
            QString gainValue = matchspMax.captured(1);
            m_elevationSPMax = gainValue.toInt();
        }

        static QRegularExpression tpRegex("Target position: (\\d+)\\r");
        QRegularExpressionMatch matchtp = tpRegex.match(data);
        if (matchtp.hasMatch()) {
            QString gainValue = matchtp.captured(1);
            m_elevationPosition = gainValue.toInt();
            m_elevationPositionmm = FIURAController::elevationStepsToMM(m_elevationPosition);
        }

    data.replace("\r", "");
    QList<QByteArray> chunks = data.split('\n');
    foreach (QByteArray chunk, chunks)
    {
        if (chunk.size() < 7)
        {
            continue;
        }
        QList<QByteArray> tokens = chunk.split(' ');
        if (tokens.size() != 2)
        {
            continue;
        }
        QByteArray valueToken = tokens.at(0);
        QByteArray checksumToken = tokens.at(1);
        qsizetype valueTokenSize = valueToken.size();
        if (checksumToken.size() != 2)
        {
            continue;
        }
        int msgChecksum = 0;
        if (sscanf(checksumToken.constData(), "%x", &msgChecksum) != 1) {
            continue;
        }
        int asciiSum = 32;
        for (int i = 0; i< valueTokenSize; i++)
        {
            int asciiValue = valueToken.at(i);
            asciiSum += asciiValue;
        }
        int computedChecksum = asciiSum % 256;
        if (computedChecksum != msgChecksum)
        {
            continue;
        }
        int value = valueToken.toInt();
        m_elevationPosition = value;
        m_elevationPositionmm = FIURAController::elevationStepsToMM(value);
        //qDebug() << "T:" << m_tpo->elapsed() << "ms - N:" << m_elevationPosition << " - P:" << m_elevationPositionmm << "mm";
        //qDebug() << "T:" << m_tpo->elapsed() << " Msg:" << data;
        //m_tpo->start();
        return;
    }
}

/////////////////////////////////////////////////////
/// \brief FIURAController::elevationStepsToMM
/// \param steps
/// \return
///
double FIURAController::elevationStepsToMM(int steps)
{
    return (steps - 1024) * L_MM/1024.0;
}

//////////////////////////////////////////////////////////
/// \brief FIURAController::elevationMMToSteps
/// \param lengthMM
/// \return
///
int FIURAController::elevationMMToSteps(double lengthMM)
{
    return (int)(1024 * lengthMM / L_MM + 1024);
}

//////////////////////////////////////////////////////////
/// \brief FIURAController::clearElevationPIDFilter
/// \return
///
void FIURAController::clearElevationPIDFilter()
{
    m_elevationConn->write("zi\r");
    m_elevationConn->waitForBytesWritten();
}

void FIURAController::requestElevationSPMin()
{
    m_elevationConn->write("ln\r");
    m_elevationConn->waitForBytesWritten();
}

void FIURAController::requestElevationSPMax()
{
    m_elevationConn->write("lx\r");
    m_elevationConn->waitForBytesWritten();
}

void FIURAController::setElevationSPMinmm(double spMinmm)
{
    int steps = FIURAController::elevationMMToSteps(spMinmm);
    QByteArray command = "ln" + QByteArray::number(steps) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
    command = "mn" + QByteArray::number(steps) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

void FIURAController::setElevationSPMaxmm(double spMaxmm)
{
    int steps = FIURAController::elevationMMToSteps(spMaxmm);
    QByteArray command = "lx" + QByteArray::number(steps) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
    command = "mx" + QByteArray::number(steps) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

void FIURAController::setElevationSPMinSteps(int spMin)
{
    QByteArray command = "ln" + QByteArray::number(spMin) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
    command = "mn" + QByteArray::number(spMin) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

void FIURAController::setElevationSPMaxSteps(int spMax)
{
    QByteArray command = "lx" + QByteArray::number(spMax) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
    command = "mx" + QByteArray::number(spMax) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

double FIURAController::getElevationSPMinmm()
{
    return FIURAController::elevationStepsToMM(m_elevationSPMin);
}

double FIURAController::getElevationSPMaxmm()
{
    return FIURAController::elevationStepsToMM(m_elevationSPMax);
}

void FIURAController::disableElevationMessageParsing()
{
    disconnect(m_elevationConn, SIGNAL(readyRead()), this, SLOT(slot_elevationReceiveMsg()));
}

void FIURAController::setElevationAnalogOutputMode(int mode)
{
    QByteArray command = "ms" + QByteArray::number(mode) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

void FIURAController::setElevationAnalogOutput(int val)
{
    QByteArray command = "co" + QByteArray::number(val) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

void FIURAController::requestElevationAnalogOutputMode()
{
    m_elevationConn->write("ms\r");
    m_elevationConn->waitForBytesWritten();
}

void FIURAController::setElevationAnalogOutputMinSteps(int val)
{
    QByteArray command = "yn" + QByteArray::number(val) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}

void FIURAController::setElevationAnalogOutputMaxSteps(int val)
{
    QByteArray command = "yx" + QByteArray::number(val) + "\r";
    m_elevationConn->write(command);
    m_elevationConn->waitForBytesWritten();
}


