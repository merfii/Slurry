#pragma once

#include <QObject>
#include "RobotFrame.h"
#include "RobotStateRT.h"

class QElapsedTimer;
class UrRealtimeCommunication;
class UrDashboardCommunication;
class QTcpServer;
class QTcpSocket;
class RobotStateHistory;

#define QT_IMPL

class RobotController: public QObject
{
	Q_OBJECT

public:
	virtual ~RobotController();
	static RobotController* GetInstance();
	static RobotStateRT* GetRobotState();
	static RobotFrame GetRobotFrame();
	static RobotStateHistory* GetRobotStateHistory();

	void RawCommand(QString cmd);
	void DashboardCommand(QString cmd);
		
	void PowerOn();
	void PowerOff();
	void Shutdown();
	void BreakRelease();
	void Reconnect();

	void Stop(double acc = 1);
	//以frm中的笛卡尔坐标为移动目标
	void MoveP2P(RobotFrame& frm, double speed = 0.05, double blend = 0);
	//以frm中的六轴角为移动目标
	void MoveP2P_J(RobotFrame& frm, double speed = 0.05, double blend = 0);

	//以下均为笛卡尔坐标
	void MoveLine(RobotFrame& frm, double speed = 0.05);
	void MovePath(std::vector<RobotFrame> &path, double speed = 0.05, double blend = 0, bool reverse = false);
	void MoveSpeed(std::vector<double> speeds, double acc = 1.0);
	void ServoP2P(RobotFrame &frm, double gain = 300);

	void TeachMode();
	void FreeDriveMode();
	void FreeDriveModeEnd();
	void ForceMode();
	void ForceModeEnd();

	void setToolVoltage(unsigned int v);
	void setFlag(unsigned int n, bool b);
	void setDigitalOut(unsigned int n, bool b);
	void setAnalogOut(unsigned int n, double f);
	void setPayload(double m);

	QString GetDestIp();
	int GetNetworkRate();
	void ReConnect();
	/*

	机械臂的任务调度关键问题在于：
	机械臂是一个慢速系统，如何与其他软件执行流协作？当机械臂运动时,系统应该返回GUI主界面
	方案一(QT_IMPL)：短时间调用Qt::processEvents处理其它界面事件;长时间调用QObject::startTimer做成状态机

	方案二:使用多线程 虽然简化了状态机编程,但线程同步十分复杂,与GUI交流和与Camera交流都需要加锁。不采用。

	方案三(COROUTINE_IMPL): 使用协程(coroutine) 工程中的Coroutine.h库即为此作用
	控制代码向机械臂发送运动指令后即调用yeild返回
	主执行函数由GUI空闲调用(或者做成定时器)

	以上方案均利用抛出异常，跳出函数返回
	并需处理回调超时及运行错误,外部停止命令
	*/
	void yeild(int timeout);
	void waitMovePass(RobotFrame &tar, int timeout);	//ms
	void waitMoveStop(RobotFrame &tar, int timeout);	//ms
	void waitMoveCancel();


private:
	bool m_moveCancel;
	bool withinTolerance(RobotFrame &tar);
	bool isStopped();
	bool hasStopSignal();
	double tolerance;
	QElapsedTimer *elapsedTimer;
	RobotStateRT *state;
	RobotStateHistory *robotHistory;
	UrRealtimeCommunication* robotcom;
	UrDashboardCommunication* dashboard;

	//For servo network
	//QTcpServer *tcpServer;
	//QTcpSocket *socketAccepted;
	//void initServer();
//private slots:
//	void newConnection();
//	void serverDataRecv();
//
	RobotController();
	static QSharedPointer<RobotController> pInst;
	RobotController(const RobotController &);	//Not implemented
	void operator=(const RobotController &);	//Not implemented



private:	//for servo
	bool uploadProg();
	void StopServo();
	bool Servo(std::vector<double> inp_timestamps,
		std::vector<std::vector<double> > inp_positions,
		std::vector<std::vector<double> > inp_velocities);
	void servoj(std::vector<double> positions, int keepalive = 1);
	std::vector<double> interp_cubic(double t, double T,
		std::vector<double> p0_pos, std::vector<double> p1_pos,
		std::vector<double> p0_vel, std::vector<double> p1_vel);

	bool executing_traj;
	double servoj_time;

	const int MULT_JOINTSTATE_ = 1000000;
	const int MULT_TIME_ = 1000000;


};