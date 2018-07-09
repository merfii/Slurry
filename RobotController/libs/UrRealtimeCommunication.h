#pragma once

#include <QObject>
#include <QString>
#include <QThread>
#include "RobotStateRT.h"

class FpsCounter;
class RobotStateHistory;
class QTimer;
class UrRealtimeCommunication : public QThread
{
	Q_OBJECT
public:
	UrRealtimeCommunication(QString ipAddress, RobotStateRT *state, RobotStateHistory *history);
	~UrRealtimeCommunication();

	void addCommandToQueue(QString cmd);
	void addCommandToQueue(std::string &cmd);

	bool isValid();
	void makeNonBlocking();

	QString getLocalIp();
	QString getDestIp();
	float getNetworkRate() const;
	void waitQuit();

protected:
	void run() override;

private:
	void _initConnect(const char *ip);

	RobotStateRT *robot_state;
	RobotStateHistory *robot_history;
	FpsCounter *fpsCounter;
	QString robotIpAddr;

	QMutex mutex;
	volatile bool quit;
};

