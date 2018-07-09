#pragma execution_character_set("utf-8")

#include <winsock2.h>
#include <ws2tcpip.h>
#include <cstdlib>
#include <cstdio>
#include <QByteArray>
#include "Logger.h"
#include "GlobalShared.h"
#include "Slurry.h"
#include "FpsCounter.h"
#include "RobotStateHistory.h"
#include "UrRealtimeCommunication.h"

static const int CONNECT_RETRY_TIME = 3;	//sec
static const int TIMEOUT = 1000/125+2;	//ms
static const int PACK_SIZE = 1060;	//bytes

static SOCKET itsSocket;
static FD_SET readSet;

UrRealtimeCommunication::UrRealtimeCommunication(QString ipAddress, RobotStateRT *state, RobotStateHistory *history)
	:robotIpAddr(ipAddress), robot_state(state), robot_history(history)
{
	quit = false;
	fpsCounter = new FpsCounter(NULL);
//	clientConnection->setSocketOption(QAbstractSocket::LowDelayOption, 1);
//	clientConnection->setSocketOption(QAbstractSocket::KeepAliveOption, 1);
//	clientConnection->setReadBufferSize(4 * 1024);
	qDebug() << robotIpAddr;
	assert(nullptr != robot_state);

	start(QThread::HighestPriority);
}

UrRealtimeCommunication::~UrRealtimeCommunication()
{
	quit = true;
	wait();
	delete fpsCounter;
}

void UrRealtimeCommunication::waitQuit()
{

}

void UrRealtimeCommunication::_initConnect(const char * ip)
{
	struct addrinfo hints;
	struct addrinfo *result = NULL;
	struct addrinfo *ptr = NULL;

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	int iResult;
	// Resolve the server address and port
	iResult = getaddrinfo(ip, "30003", &hints, &result);
	if (iResult != 0) {
		//resolve failed
		printf("getaddrinfo failed with error: %d\n", iResult);
		return;
	}

	SOCKET socketObj = INVALID_SOCKET;
	ptr = result;

	// Create a SOCKET for connecting to server
	socketObj = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);

	if (socketObj == INVALID_SOCKET) {
		// try to use another addrinfo in list
		while (ptr = ptr->ai_next){
			// Create a SOCKET for connecting to server
			socketObj = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
			if (socketObj == INVALID_SOCKET) {
				continue;
			}
			else {
				break;
			}
		}
	}

	if (socketObj == INVALID_SOCKET) {
		freeaddrinfo(result);
		printf("Cannot instantiate SOCKET object (error %d)\n", WSAGetLastError());
		return;
	}
	/*
	unsigned long ulMode = 1;
	ioctlsocket(socketObj, FIONBIO, &ulMode);
*/
	// Connect unconnected socketObj to host.
	int ret =  ::connect(socketObj, ptr->ai_addr, (int)ptr->ai_addrlen);
	if (ret == SOCKET_ERROR) {
		closesocket(socketObj);
		printf("Cannot connect to remote host %s\n", ip);
		return;
	}

	int timeout = TIMEOUT;
	//超时模式
	//ret = ::setsockopt(socketObj, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));
	ret = ::setsockopt(socketObj, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));

	//TCP_NODELAY
	const char chOpt = 1;
	setsockopt(socketObj, IPPROTO_TCP, TCP_NODELAY, &chOpt, sizeof(chOpt));

	// if we here the socketObj is good
	itsSocket = socketObj;
}

void UrRealtimeCommunication::run()
{
	char buf[PACK_SIZE]; 
	while (!quit) {
		if (!isValid())
		{
			static int _nn;
			if (_nn++ % (CONNECT_RETRY_TIME * 1000 / TIMEOUT) == 0)
			{
				_initConnect(robotIpAddr.toStdString().c_str());
			}
			msleep(TIMEOUT);
			continue;
		}
		qint64 rdsize;
		do{
			buf[0] = 0;
			rdsize = recv(itsSocket, buf, PACK_SIZE,0);
			if (rdsize == PACK_SIZE)
			{
				//qDebug() << QString("Robot data recv %1 bytes").arg(QString::number(rdsize));
				robot_state->unpack((uint8_t*)buf);
				robot_history->add(robot_state);
				fpsCounter->tick();
			}
		}while (rdsize > 0 && !quit);
		yieldCurrentThread();
	}
	closesocket(itsSocket);
}

void UrRealtimeCommunication::makeNonBlocking()
{
	// make async socket
	unsigned long ulMode = 1;
	ioctlsocket(itsSocket, FIONBIO, &ulMode);
}

bool UrRealtimeCommunication::isValid()
{
	if (itsSocket == INVALID_SOCKET)
		return false;

	int ret;
	bool ok;
	char buffer[1];
	
	/*MSG_PEEK表示从输入队列中读数据但并不将数据从输入队列中移除*/
	ret = recv(itsSocket, buffer, 1, MSG_PEEK);
	ok = (WSAECONNRESET != WSAGetLastError()) && ret>=0;
	return ok;
//	Logger::PrintLog("已连接到机械臂....");
}


/*
void UrRealtimeCommunication::ReConnect()
{
	clientConnection->disconnectFromHost();
	if (clientConnection->state() != QAbstractSocket::UnconnectedState)
		clientConnection->waitForDisconnected(500);
	tryConnect();
}
*/

float UrRealtimeCommunication::getNetworkRate() const
{
	return fpsCounter->getFPSf();
}

void UrRealtimeCommunication::addCommandToQueue(QString cmd) {
	int bytes_written;
	QByteArray cmdByte = cmd.toLatin1();

	//行末尾的回车是必须的 没有回车则机械臂不执行指令
	if (! cmdByte.endsWith('\n')) {
		cmdByte.append('\n');
	}

	if (isValid())
	{
//		bytes_written = clientConnection->write(cmd.toLatin1());
		bytes_written = send(itsSocket, cmdByte.data(), cmdByte.size(), 0);
//		clientConnection->flush();
		qDebug() << cmd;
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
	char host[256];
	gethostname(host, sizeof(host));
	struct hostent *p = gethostbyname(host);
	if (p == 0)
	{
		return QString();
	}
	else
	{
		struct in_addr in;
		memcpy(&in, p->h_addr_list[0], sizeof(struct in_addr));
		return QString(inet_ntoa(in));
	}
//	QString s = clientConnection->localAddress().toString();
}

QString UrRealtimeCommunication::getDestIp() {
	//QString s = clientConnection->peerAddress().toString();
	return robotIpAddr;
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