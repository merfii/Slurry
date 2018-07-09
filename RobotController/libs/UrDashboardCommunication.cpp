#pragma execution_character_set("utf-8")
#include <QByteArray>
#include "Logger.h"
#include "GlobalShared.h"
#include "Slurry.h"

#include "UrDashboardCommunication.h"

#define CONNECT_RETRY_TIME 8		//sec

UrDashboardCommunication::UrDashboardCommunication(QString ipAddress)
{
	robotIpAddr = ipAddress;
	clientConnection = new QTcpSocket;
	connect(clientConnection, SIGNAL(connected()), this, SLOT(connected()));
	connect(clientConnection, SIGNAL(disconnected()), this, SLOT(disconnected()));
	connect(clientConnection, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(connectError(QAbstractSocket::SocketError)));
	connect(clientConnection, SIGNAL(readyRead()), this, SLOT(dataRecv()));
	clientConnection->setSocketOption(QAbstractSocket::LowDelayOption, 1);
	clientConnection->connectToHost(robotIpAddr, 29999, QIODevice::ReadWrite);

	connect(&retryTimer, SIGNAL(timeout()), this, SLOT(reConnect()));
	retryTimer.start(1000 * CONNECT_RETRY_TIME);
}

UrDashboardCommunication::~UrDashboardCommunication()
{
	clientConnection->disconnectFromHost();
	delete clientConnection;
}


void UrDashboardCommunication::connected()
{
}

void UrDashboardCommunication::disconnected()
{
}

void UrDashboardCommunication::connectError(QAbstractSocket::SocketError socketError)
{
	//Logger::PrintLog(clientConnection->errorString() + socketError);
}

void UrDashboardCommunication::reConnect()
{
	if ( clientConnection->state() == QAbstractSocket::UnconnectedState)
	{
		clientConnection->connectToHost(robotIpAddr, 29999, QIODevice::ReadWrite);
	}
}

void UrDashboardCommunication::dataRecv()
{
	if (!clientConnection->isOpen())
	{
		return;
	}
	QByteArray recv = clientConnection->readAll();
//	qDebug() <<QString("Dashboard: ") + QString(recv);
}

void UrDashboardCommunication::addCommandToQueue(QString cmd) {
	int bytes_written;

	if (!cmd.endsWith('\n')) {
		cmd.append("\n");
	}
	if (clientConnection->isOpen())
	{
		bytes_written = clientConnection->write(cmd.toLatin1());
	//	qDebug() << QString("Write data %1 bytes").arg(bytes_written);
	}
	else
	{
		qDebug() << QString("Can't send command \" %1 \".").arg(cmd);
	}
}


void UrDashboardCommunication::addCommandToQueue(std::string &cmd) {
	QString c = QString::fromStdString(cmd);
	addCommandToQueue(c);
}


QString UrDashboardCommunication::getLocalIp() {
	return clientConnection->localAddress().toString();
}
QString UrDashboardCommunication::getDestIp() {
	return clientConnection->peerAddress().toString();
}
