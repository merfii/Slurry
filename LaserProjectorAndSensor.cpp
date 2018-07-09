  #pragma comment(lib, "Qt5SerialPort.lib")


#include <QTimer>
#include <QByteArray>
#include <QRegularExpression>
#include <atomic>
//#include <QFile>
#include "GlobalShared.h"
#include "Slurry.h"
#include "Logger.h"
#include "SystemParameter.h"

#include "LaserProjectorAndSensor.h"

#define COM_BAUD (QSerialPort::Baud115200)
#define COM_TRAIL_TIME 20	//ms 之后视为接收到一条数据

#define MAGIC_HEAD 0xC9
#define CMD_CLOSE_ALL 0x00
#define CMD_LED2_ON 0x10
#define CMD_LED2_OFF 0x11
#define CMD_LED3_ON 0x12
#define CMD_LED3_OFF 0x13
#define CMD_LASER_ON 0x20
#define CMD_LASER_OFF 0x21
#define CMD_TRIGGER1_ON 0x30
#define CMD_TRIGGER1_OFF 0x31
#define CMD_TRIGGER2_ON 0x32
#define CMD_TRIGGER2_OFF 0x33
#define CMD_TRIGBOT_ON 0x34
#define CMD_TRIGBOT_OFF 0x35
#define CMD_FLICKER_RUN 0x40
#define CMD_FLICKER_STOP 0x41
#define CMD_METER_RUN 0x50
#define CMD_METER_STOP 0x51
#define CMD_METER_STATUS 0x52

static QSerialPort *m_serialPort;
static QTimer      *m_timer;
static QRegularExpression *cmdPattern;
static std::atomic_bool *meter_changed;
static std::atomic<double> *m_distance;
LaserProjectorAndSensor* LaserProjectorAndSensor::pInst;

static void resolve(QByteArray &ba);

static inline uchar check(uchar a, uchar b)
{
	return a^b;
}

LaserProjectorAndSensor::LaserProjectorAndSensor():
QObject(GlobalShared::slurry),m_bytesLeft(0)
{
	m_readData.clear();
	meter_changed = false;
	m_serialPort = new QSerialPort;
	m_serialPort->setPortName(SystemParameter::GetInstance()->GetSettingString("Settings/Hardware/COM"));
	m_serialPort->setDataBits(QSerialPort::Data8);
	m_serialPort->setStopBits(QSerialPort::OneStop);
	m_serialPort->setParity(QSerialPort::EvenParity);
	m_serialPort->setBaudRate(COM_BAUD);

	flickerSlowFreq = SystemParameter::GetInstance()->GetSettingInt("Settings/CameraControl/FlickerSlowFreq");
	flickerFastFreq = SystemParameter::GetInstance()->GetSettingInt("Settings/CameraControl/FlickerFastFreq");
	flckerIntervalFast = SystemParameter::GetInstance()->GetSettingInt("Settings/CameraControl/FlickerIntervalFast");
	flckerIntervalSlow = SystemParameter::GetInstance()->GetSettingInt("Settings/CameraControl/FlickerIntervalSlow");


	m_timer = new QTimer(this);
	cmdPattern = new QRegularExpression("D:\\s*(\\d+.\\d+)m");
	meter_changed = new std::atomic_bool;
	m_distance = new std::atomic < double > ;
	m_distance = 0;

	if (!m_serialPort->open(QIODevice::ReadWrite)) {
		qDebug() << QString("Failed to open port %1,  %2").arg(SystemParameter::GetInstance()->GetSettingString("Settings/Hardware/COM"))
			.arg(m_serialPort->errorString());
		return;
	}
	connect(m_serialPort, SIGNAL(readyRead()),SLOT(handleReadyRead()));
	connect(m_serialPort, SIGNAL(error(QSerialPort::SerialPortError)), SLOT(handleError(QSerialPort::SerialPortError)));
	connect(m_timer, SIGNAL(timeout()), SLOT(handleTimeout()));
	connect(m_serialPort, SIGNAL(bytesWritten(qint64)), SLOT(handleBytesWritten(qint64)));

	m_timer->start(COM_TRAIL_TIME);
}


LaserProjectorAndSensor::~LaserProjectorAndSensor()
{
	m_serialPort->close();
	delete m_serialPort;
	delete m_timer;
	delete cmdPattern;
	delete meter_changed;
	delete m_distance;
}

LaserProjectorAndSensor* LaserProjectorAndSensor::GetInstance()
{
	if (pInst == nullptr)
	{
		pInst = new LaserProjectorAndSensor;	//由Qt自动析构
	}
	return pInst;
}

bool LaserProjectorAndSensor::isConstructed()
{
	return pInst != nullptr;
}

void LaserProjectorAndSensor::handleReadyRead()
{
	m_readData.append(m_serialPort->readAll());
	m_timer->start(COM_TRAIL_TIME);
}

void LaserProjectorAndSensor::handleTimeout()
{
	if (m_readData.isEmpty() || !m_readData.endsWith('\n'))
	{
		return;
	}else
	{
		resolve(m_readData);
		m_readData.clear();
	}
}

void LaserProjectorAndSensor::handleError(QSerialPort::SerialPortError serialPortError)
{
	if (serialPortError == QSerialPort::ReadError) {
		qCritical() << QObject::tr("An I/O error occurred: %2").arg(m_serialPort->errorString()) << endl;
	}
}


void LaserProjectorAndSensor::handleBytesWritten(qint64 bytes)
{
	m_bytesLeft -= bytes;
	if (m_bytesLeft == 0) {
		//Ok
		;
	}
}


void LaserProjectorAndSensor::write(const QByteArray &writeData)
{
	if (! m_serialPort->isOpen())
	{
		return;
	}

	m_bytesLeft = writeData.size();

	qint64 bytesWritten = m_serialPort->write(writeData);

	if (bytesWritten != writeData.size()) {
		qWarning() << QObject::tr("Failed to write data: %1").arg(m_serialPort->errorString()) << endl;
	}
}

void LaserProjectorAndSensor::resolve(QByteArray &ba)
{
	QRegularExpressionMatch match = cmdPattern->match(ba);
	if (match.hasMatch())
	{
		m_distance->store( match.captured(1).toDouble() );
		GlobalShared::slurry->ui.lcdDist->display(m_distance->load());
		meter_changed->store(true);
	}
	else if (ba[0] == 'S')
	{
		Logger::PrintLog(ba);
	}
}


void LaserProjectorAndSensor::LaserOn()
{
	QByteArray ba(6,0);

	ba[0] = MAGIC_HEAD;
	ba[1] = ba[4] = CMD_LASER_ON;
	ba[2] = ba[3] = 0x00;
	ba[5] = check(ba[1], ba[2]);
	write(ba);
}



void LaserProjectorAndSensor::LaserOff()
{
	QByteArray ba(6, 0);
	ba[0] = MAGIC_HEAD;
	ba[1] = ba[4] = CMD_LASER_OFF;
	ba[2] = ba[3] = 0x00;
	ba[5] = check(ba[1], ba[2]);
	write(ba);
}

void LaserProjectorAndSensor::Trigger1On()
{
	QByteArray ba(6, 0);
	ba[0] = MAGIC_HEAD;
	ba[1] = ba[4] = CMD_TRIGGER1_ON;
	ba[2] = ba[3] = 0x00;
	ba[5] = check(ba[1], ba[2]);
	write(ba);
}

void LaserProjectorAndSensor::Trigger1Off()
{
	QByteArray ba(6, 0);
	ba[0] = MAGIC_HEAD;
	ba[1] = ba[4] = CMD_TRIGGER1_OFF;
	ba[2] = ba[3] = 0x00;
	ba[5] = check(ba[1], ba[2]);
	write(ba);
}

void LaserProjectorAndSensor::Trigger2On()
{
	QByteArray ba(6, 0);
	ba[0] = MAGIC_HEAD;
	ba[1] = ba[4] = CMD_TRIGGER2_ON;
	ba[2] = ba[3] = 0x00;
	ba[5] = check(ba[1], ba[2]);
	write(ba);
}

void LaserProjectorAndSensor::Trigger2Off()
{
	QByteArray ba(6, 0);
	ba[0] = MAGIC_HEAD;
	ba[1] = ba[4] = CMD_TRIGGER2_OFF;
	ba[2] = ba[3] = 0x00;
	ba[5] = check(ba[1], ba[2]);
	write(ba);
}


void LaserProjectorAndSensor::TrigbotOn()
{
	QByteArray ba(6, 0);
	ba[0] = MAGIC_HEAD;
	ba[1] = ba[4] = CMD_TRIGBOT_ON;
	ba[2] = ba[3] = 0x00;
	ba[5] = check(ba[1], ba[2]);
	write(ba);
}

void LaserProjectorAndSensor::TrigbotOff()
{
	QByteArray ba(6, 0);
	ba[0] = MAGIC_HEAD;
	ba[1] = ba[4] = CMD_TRIGBOT_OFF;
	ba[2] = ba[3] = 0x00;
	ba[5] = check(ba[1], ba[2]);
	write(ba);
}


void LaserProjectorAndSensor::FlickerRunSlow()
{
	QByteArray ba(6, 0);
	ba[0] = MAGIC_HEAD;
	ba[1] = ba[4] = CMD_FLICKER_RUN;
	ba[2] = flickerSlowFreq;
	ba[3] = flckerIntervalSlow;
	ba[5] = check(ba[1], ba[2]);
	write(ba);
}


void LaserProjectorAndSensor::FlickerRunFast()
{
	QByteArray ba(6, 0);
	ba[0] = MAGIC_HEAD;
	ba[1] = ba[4] = CMD_FLICKER_RUN;
	ba[2] = flickerFastFreq;
	ba[3] = flckerIntervalFast;
	ba[5] = check(ba[1], ba[2]);
	write(ba);
}

void LaserProjectorAndSensor::FlickerStop()
{
	QByteArray ba(6, 0);
	ba[0] = MAGIC_HEAD;
	ba[1] = ba[4] = CMD_FLICKER_STOP;
	ba[2] = ba[3] = 0x00;
	ba[5] = check(ba[1], ba[2]);
	write(ba);
}

void LaserProjectorAndSensor::MeterRun()
{
	QByteArray ba(6, 0);
	ba[0] = MAGIC_HEAD;
	ba[1] = ba[4] = CMD_METER_RUN;
	ba[2] = ba[3] = 0x00;
	ba[5] = check(ba[1], ba[2]);
	write(ba);
}

void LaserProjectorAndSensor::MeterStop()
{
	QByteArray ba(6, 0);
	ba[0] = MAGIC_HEAD;
	ba[1] = ba[4] = CMD_METER_STOP;
	ba[2] = ba[3] = 0x00;
	ba[5] = check(ba[1], ba[2]);
	write(ba);
}

void LaserProjectorAndSensor::MeterStatus()
{
	QByteArray ba(6, 0);
	ba[0] = MAGIC_HEAD;
	ba[1] = ba[4] = CMD_METER_STATUS;
	ba[2] = ba[3] = 0x00;
	ba[5] = check(ba[1], ba[2]);
	write(ba);
}

bool LaserProjectorAndSensor::MeterChanged()
{
	return meter_changed;
}

//double LaserProjectorAndSensor::GetDistance()
//{
//	return m_distance->load();
//}

void LaserProjectorAndSensor::MeterMark()
{
	meter_changed = false;
}