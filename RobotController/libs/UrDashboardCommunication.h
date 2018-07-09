#pragma once

#include <QObject>
#include <QString>
#include <QtNetwork>

#include "RobotStateRT.h"
class UrDashboardCommunication: QObject {
	Q_OBJECT
public:
	UrDashboardCommunication(QString ipAddress);
	~UrDashboardCommunication();

	void addCommandToQueue(QString cmd);
	void addCommandToQueue(std::string &cmd);

	QString getLocalIp();
	QString getDestIp();

	/*
	UrDashboardCommunication(std::condition_variable& msg_cond, std::string host,
	unsigned int safety_count_max = 12);
	*/

private:
	QTcpSocket *clientConnection;
	QTimer retryTimer;
	QString robotIpAddr;

private slots:
	void reConnect();
	void connected();
	void disconnected();
	void connectError(QAbstractSocket::SocketError socketError);
	void dataRecv();



};

/*
class UrDashboardCommunication {
private:
	unsigned int safety_count_max_;
	int sockfd_;
	struct sockaddr_in serv_addr_;
	struct hostent *server_;
	std::string local_ip_;
	bool keepalive_;
	std::thread comThread_;
	int flag_;
	std::recursive_mutex command_string_lock_;
	std::string command_;
	unsigned int safety_count_;
	void run();


public:
	bool connected_;
	RobotStateRT* robot_state_;

	UrDashboardCommunication(std::condition_variable& msg_cond, std::string host,
			unsigned int safety_count_max = 12);
	bool start();
	void halt();
	void setSpeed(double q0, double q1, double q2, double q3, double q4,
			double q5, double acc = 100.);
	void addCommandToQueue(std::string inp);
	void setSafetyCountMax(uint inp);
	std::string getLocalIp();

};
*/