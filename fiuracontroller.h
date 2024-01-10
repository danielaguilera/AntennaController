#ifndef FIURACONTROLLER_H
#define FIURACONTROLLER_H

#include <QElapsedTimer>
#include <QTcpSocket>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QRegularExpression>
#include <QRegularExpressionMatch>
#include <QTimer>
#include <QThread>
#include <QObject>
#include <cctype>
#include <iostream>

// Parámetros ronza:
#define RONZA_IP "192.168.0.99"
#define RONZA_PORT 23

// Parámetros elevación:
#define ELEVACION_COM_PORT "COM3"
#define STREAM_RATE_CONSTANT 234375
#define L_MM 3.175

class FIURAController : public QObject
{
    Q_OBJECT
public:
    explicit FIURAController(QObject *parent = nullptr);

    // Ronza
    void connectRonza(QString ipAddress, int port);
    bool ronzaConnected();
    void enableRonzaDriver();
    void disableRonzaDriver();
    void setRonzaPositionTarget(double targetPosition);
    void setRonzaVelocityTarget(double targetVelocity);
    void stopRonza();
    void startRonzaHomingSequence();
    void presetRonzaParams();

    // Elevación:
    static double elevationStepsToMM(int steps);
    static int elevationMMToSteps(double lengthMM);

    bool elevationConnected();
    bool m_elevationConnectedFlag;
    void connectElevationMotor(QString port);
    void setElevationSIMode(int mode);
    void setElevationKpGain(uint64_t Kp);
    void setElevationKiGain(uint64_t Ki);
    void setElevationKdGain(uint64_t Kd);
    void requestElevationKpGain();
    void requestElevationKiGain();
    void requestElevationKdGain();
    int getElevationKpGain();
    int getElevationKiGain();
    int getElevationKdGain();
    void disableElevationMotorCoast();
    void enableElevationMotorCoast();
    void setElevationMaximumForce(double maxForcekgf);
    void requestElevationMaximumForce();
    double getElevationMaximumForce();
    void setElevationPositionTargetmm(double posTargetmm);
    void requestElevationPositionmm();
    double getElevationPositionmm();
    void setElevationPositionTarget(int posTarget);
    void setElevationTargetToCurrentPosition();
    void enableElevationStreaming(double streamFreqHz);
    void disableElevationStreaming();
    void rebootElevationActuator();
    void clearElevationPIDFilter();
    void disableElevationMessageParsing();
    void setElevationAnalogOutputMode(int mode);
    void setElevationAnalogOutput(int val);
    void setElevationAnalogOutputMinSteps(int val);
    void setElevationAnalogOutputMaxSteps(int val);
    void requestElevationAnalogOutputMode();




    void requestElevationSPMin();
    void requestElevationSPMax();
    void setElevationSPMinmm(double spMinmm);
    void setElevationSPMaxmm(double spMaxmm);
    void setElevationSPMinSteps(int spMin);
    void setElevationSPMaxSteps(int spMax);
    double getElevationSPMinmm();
    double getElevationSPMaxmm();

private:
    QElapsedTimer *m_tpo;
    QElapsedTimer *m_trate;

    // Ronza
    QTcpSocket *m_ronzaConn;
    double m_ronzaPosition;
    double m_ronzaTarget;

    // Elevación
    QSerialPort *m_elevationConn;
    int m_elevationPosition;
    int m_elevationTarget;
    int m_elevationPositionmm;
    int m_elevationTargetmm;
    double m_elevationMaxForce;
    int m_elevationKpGain;
    int m_elevationKdGain;
    int m_elevationKiGain;
    int m_elevationSPMin = 2048;
    int m_elevationSPMax = 40982;
    QString m_elevationPort;
    bool m_elevationInMotion;

private slots:
    void slot_elevationReceiveMsg();

signals:

};

#endif
