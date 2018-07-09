#pragma execution_character_set("utf-8")
/*
这是利用Qt事件循环的写法 看上去比较方便
由于其信号机制走内部的Eventloop
若界面线程繁忙且数据量大 则QTcpSocket有bug
在接收几次数据之后 再也收不到数据且无任何提示
连接也看起来正常

UR机械臂的数据量约每秒125Hz次发送 每次数据包约1KB
从Qt5.4升级到Qt5.9后情况更严重了 遂抛弃
*/


#include <QByteArray>
#include "Logger.h"
#include "GlobalShared.h"
#include "Slurry.h"
#include "FpsCounter.h"
#include "UrRealtimeCommunication.h"

UrRealtimeCommunication::UrRealtimeCommunication(QString ipAddress)
{
	robotIpAddr = ipAddress;
	robot_state = new RobotStateRT;
	robot_state->setVersion(3.3);
	clientConnection = new QTcpSocket;
	fpsCounter = new FpsCounter(NULL);
	//clientConnection->setSocketOption(QAbstractSocket::LowDelayOption, 1);
	//clientConnection->setSocketOption(QAbstractSocket::KeepAliveOption, 1);
	//clientConnection->setReadBufferSize(4 * 1024);

	connect(clientConnection, SIGNAL(connected()), this, SLOT(connected()));
	connect(clientConnection, SIGNAL(disconnected()), this, SLOT(disconnected()));
	connect(clientConnection, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(connectError(QAbstractSocket::SocketError)));
	connect(clientConnection, SIGNAL(readyRead()), this, SLOT(dataRecv()));

//	connect(&retryTimer, SIGNAL(timeout()), this, SLOT(tryConnect()));
//	retryTimer.start(1000 * CONNECT_RETRY_TIME);
	QTimer::singleShot(500, this, SLOT(tryConnect()));
}

UrRealtimeCommunication::~UrRealtimeCommunication()
{
	clientConnection->abort();
	if (clientConnection->state() == QAbstractSocket::UnconnectedState ||
		clientConnection->waitForDisconnected(500)){
		qDebug("Disconnected!");
	}
	delete clientConnection;
	delete robot_state;
	delete fpsCounter;
}


void UrRealtimeCommunication::connected()
{
//	m_connected = true;
	Logger::PrintLog("已连接到机械臂....");
}

void UrRealtimeCommunication::disconnected()
{

}

void UrRealtimeCommunication::connectError(QAbstractSocket::SocketError socketError)
{
	Logger::PrintLog(clientConnection->errorString() + socketError);
	//m_connected = false;
}

void UrRealtimeCommunication::tryConnect()
{
	if ( clientConnection->state() == QAbstractSocket::UnconnectedState)
	{
		clientConnection->connectToHost(robotIpAddr, 30003, QIODevice::ReadWrite);
	}
	/*
	查找所有ip地址
	QString ipAddress;
	QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();
	// use the first non-localhost IPv4 address
	for (int i = 0; i < ipAddressesList.size(); ++i) {
	if (ipAddressesList.at(i) != QHostAddress::LocalHost &&
	ipAddressesList.at(i).toIPv4Address()) {
	ipAddress = ipAddressesList.at(i).toString();
	break;
	}
	}
	*/
}

void UrRealtimeCommunication::ReConnect()
{
	clientConnection->disconnectFromHost();
	if (clientConnection->state() != QAbstractSocket::UnconnectedState)
		clientConnection->waitForDisconnected(500);
	tryConnect();
}

void UrRealtimeCommunication::dataRecv()
{
	if (!clientConnection->isOpen())
	{
		return;
	}
	QByteArray recv = clientConnection->readAll();
	qDebug() <<QString("Robot data recv %1 bytes").arg(QString::number(recv.length()));
	//robot_state->unpack(reinterpret_cast<uint8_t*>(recv.data()));
	fpsCounter->tick();
}

float UrRealtimeCommunication::getNetworkRate() const
{
	return fpsCounter->getFPSf();
}

void UrRealtimeCommunication::addCommandToQueue(QString cmd) {
	int bytes_written;

	if (!cmd.endsWith('\n')) {
		cmd.append("\n");
	}
	if (clientConnection->isOpen())
	{
		bytes_written = clientConnection->write(cmd.toLatin1());
		clientConnection->flush();
		qDebug() << QString("Robot command: %1").arg(cmd);
	}
	else
	{
		qDebug() << QString("Can't send command \" %1 \".").arg(cmd);
	}
}


void UrRealtimeCommunication::addCommandToQueue(std::string &cmd) {
	QString c = QString::fromStdString(cmd);
	addCommandToQueue(c);
}


QString UrRealtimeCommunication::getLocalIp() {
	return clientConnection->localAddress().toString();
}
QString UrRealtimeCommunication::getDestIp() {
	return clientConnection->peerAddress().toString();
}
