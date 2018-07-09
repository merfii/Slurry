#pragma once

#include <QObject>
#include <QtSerialPort/QSerialPort>

class LaserProjectorAndSensor: public QObject
{
	Q_OBJECT
public:
	virtual ~LaserProjectorAndSensor();
	static LaserProjectorAndSensor *GetInstance();
	static bool isConstructed();

	void LaserOn();
	void LaserOff();

	void Trigger1On();
	void Trigger1Off();
	
	void Trigger2On();
	void Trigger2Off();

	void TrigbotOn();
	void TrigbotOff();
	
	void FlickerRunFast();
	void FlickerRunSlow();
	void FlickerStop();

	void MeterRun();
	void MeterStop();
	void MeterStatus();

	void MeterMark();	//Meter测量一次大概需要1秒  使用该函数标记测量时间
	bool MeterChanged();	//使用该函数判断有没有新的测量数据到达

//	double GetDistance();
private:
	LaserProjectorAndSensor();
	LaserProjectorAndSensor(LaserProjectorAndSensor &);
	void operator=(LaserProjectorAndSensor &);
	void write(const QByteArray &writeData);

	void resolve(QByteArray &ba);
	int m_bytesLeft;
	QByteArray  m_readData;
	char flickerSlowFreq, flickerFastFreq, flckerIntervalFast, flckerIntervalSlow;

	static LaserProjectorAndSensor *pInst;

//	用Qt的串口
private slots:
	void handleReadyRead();
	void handleBytesWritten(qint64 bytes);
	void handleTimeout();
	void handleError(QSerialPort::SerialPortError serialPortError);

};
