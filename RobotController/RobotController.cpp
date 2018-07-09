#pragma execution_character_set("utf-8")
//#pragma comment(lib, "Ws2_32.lib")

#include <QtNetwork>
#include <QTest>
#include <winsock2.h>
#include <QByteArray>
#include <Thread>
#include "RobotStateHistory.h"
#include "UrRealtimeCommunication.h"
#include "UrDashboardCommunication.h"
#include "SystemParameter.h"
#include "RobotException.h"
#include "GlobalShared.h"

#include "RobotController.h"

#define SERVER_PORT 50007 

QSharedPointer<RobotController> RobotController::pInst;


RobotController::RobotController()
{
	elapsedTimer = new QElapsedTimer;
	state = new RobotStateRT;
	robotHistory = new RobotStateHistory;
	state->setVersion(3.3);
	robotcom = new UrRealtimeCommunication(
		SystemParameter::GetInstance()->GetSettingString("Settings/Hardware/RobotIp"),
		state,
		robotHistory);
	dashboard = new UrDashboardCommunication(SystemParameter::GetInstance()->GetSettingString("Settings/Hardware/RobotIp"));
	tolerance = SystemParameter::GetInstance()->GetSettingDouble(QString("Settings/Robot/MoveTolerance"));
	executing_traj = false;
	servoj_time = 0.008;
}

RobotController::~RobotController()
{
	delete elapsedTimer;
	delete robotcom;
	delete dashboard;
	delete state;
	delete robotHistory;
}

RobotController* RobotController::GetInstance()
{
	if (pInst.isNull())
	{
		pInst = QSharedPointer<RobotController>(new RobotController);
	}
	return pInst.data();
}
//
//UrRealtimeCommunication* RobotController::GetCommunication()
//{
//	return GetInstance()->robot;
//}

RobotStateRT* RobotController::GetRobotState()
{
	return GetInstance()->state;
}

RobotFrame RobotController::GetRobotFrame()
{
	if (GetInstance()->state->isValid())
	{
		return GetInstance()->state->getFrame();
	}
	else
	{
		return RobotFrame();
	}

}


RobotStateHistory* RobotController::GetRobotStateHistory()
{
	return GetInstance()->robotHistory;
}

QString RobotController::GetDestIp()
{
	return robotcom->getDestIp();
}

int RobotController::GetNetworkRate()
{
	return robotcom->getNetworkRate();
}

void RobotController::RawCommand(QString cmd)
{
	robotcom->addCommandToQueue(cmd);
}

void RobotController::DashboardCommand(QString cmd)
{
	dashboard->addCommandToQueue(cmd);
}

void RobotController::PowerOn()
{
	dashboard->addCommandToQueue("unlock protective stop");
//	QTest::qWait(300);
	dashboard->addCommandToQueue("close safety popup");
//	QTest::qWait(300);
	dashboard->addCommandToQueue("power on");
}

void RobotController::PowerOff()
{
	dashboard->addCommandToQueue("power off");
}

void RobotController::Shutdown()
{
	dashboard->addCommandToQueue("shutdown");
}

void RobotController::BreakRelease()
{
	dashboard->addCommandToQueue("brake release");
}

void RobotController::Stop(double acc)
{
	robotcom->addCommandToQueue(QString("stopj(%1)").arg(QString::number(acc, 'f', 6)));
} 

void RobotController::Reconnect()
{
	delete robotcom;
	robotcom = new UrRealtimeCommunication(
		SystemParameter::GetInstance()->GetSettingString("Settings/Hardware/RobotIp"),
		state,
		robotHistory);
}

void RobotController::MoveP2P(RobotFrame &frm, double speed, double blend)
{
	double acc = 1;
	QString tcp = QString::fromStdString(frm.tcpToString());
	QString cmd;
	cmd += QString("movej(%1, a=%2, v=%3, r=%4)") //q a v t r
		.arg(tcp)
		.arg(QString::number(acc, 'f', 5))	// rad /s^2
		.arg(QString::number(speed, 'f', 5)) // rad /s
		.arg(QString::number(blend, 'f', 5));	//blend radius, m
	robotcom->addCommandToQueue(cmd);
}

void RobotController::MoveP2P_J(RobotFrame& frm, double speed, double blend)
{
	double acc = 1;
	QString tcp = QString::fromStdString(frm.jointsToString());
	QString cmd;
	cmd += QString("movej(%1, a=%2, v=%3, r=%4)") //q a v t r
		.arg(tcp)
		.arg(QString::number(acc, 'f', 5))	// rad /s^2
		.arg(QString::number(speed, 'f', 5)) // rad /s
		.arg(QString::number(blend, 'f', 5));	//blend radius, m
	robotcom->addCommandToQueue(cmd);
}


void RobotController::MoveLine(RobotFrame &frm, double speed)
{
	QString tcp = QString::fromStdString(frm.tcpToString());
	QString cmd;
	cmd += QString("movel(%1, v=%2)") //q a v t r
		.arg(tcp)
		.arg(QString::number(speed, 'f', 5)); // rad /s
	robotcom->addCommandToQueue(cmd);
}

void RobotController::MovePath(std::vector<RobotFrame> &path, double speed, double blend, bool reverse)
{
	if (speed > 1.5)
		speed = 0.2;
	double acc = 1;
	QString cmd;
	cmd = QString("def path():\n");
	auto ptsize = path.size();
	for (int i = 0; i < ptsize; i++)
	{
		int idx = reverse ? (ptsize - i - 1) : (i);
		QString tcp = QString::fromStdString(path[idx].tcpToString());
		cmd += QString("movej(%1, a=%2, v=%3, r=%4)\n") //q a v t r
			.arg(tcp)
			.arg(QString::number(acc, 'f', 5))	// rad /s^2
			.arg(QString::number(speed, 'f', 5)) // rad /s
			.arg(QString::number(blend, 
			'f', 5));	//blend radius, m
	}
	cmd += "end\n";
	robotcom->addCommandToQueue(cmd);
}

void RobotController::MoveSpeed(std::vector<double> speeds, double acc)
{
	QString cmd;
	cmd = QString("speedj([%1, %2, %3, %4, %5, %6], a=%7,)\n")
		.arg(QString::number(speeds[0], 'f', 5)).arg(QString::number(speeds[1], 'f', 5)).arg(QString::number(speeds[2], 'f', 5))
		.arg(QString::number(speeds[3], 'f', 5)).arg(QString::number(speeds[4], 'f', 5)).arg(QString::number(speeds[5], 'f', 5))
		.arg(acc);
	robotcom->addCommandToQueue(cmd);
}

void RobotController::ServoP2P(RobotFrame &frm, double gain)
{
	QString tcp = QString::fromStdString(frm.tcpToString());
	QString cmd;
	cmd += QString("servoj(%1, gain=%2)\n")
		.arg(tcp)
		.arg(QString::number(gain, 'f', 5));
	robotcom->addCommandToQueue(cmd);
}

void RobotController::TeachMode()
{
	robotcom->addCommandToQueue(QString("teach_mode()"));
}

//经实验 freedrive模式必须加在函数里的循环里才能使用 且之前需要是移动指令
void RobotController::FreeDriveMode()
{
	QString cmd;
	cmd = QString("def freedr():\n");
	cmd += "    while 1:\n";
	cmd += "    speedj([0,0,0,0,0,0],1,0.01)\n";
	cmd += "    freedrive_mode()\n";
	cmd += "    end\n";
	cmd += "end\n";
	robotcom->addCommandToQueue(cmd);
}

void RobotController::FreeDriveModeEnd()
{
	robotcom->addCommandToQueue(QString("end_freedrive_mode()"));
}

void RobotController::ForceMode()
{
	robotcom->addCommandToQueue(QString("force_mode()"));
}

void RobotController::ForceModeEnd()
{
	robotcom->addCommandToQueue(QString("end_force_mode()"));
}

void RobotController::setToolVoltage(unsigned int v) {
	char buf[256];
	sprintf(buf, "sec setOut():\n\tset_tool_voltage(%d)\nend\n", v);
	robotcom->addCommandToQueue(buf);
	qDebug() << buf;
}
void RobotController::setFlag(unsigned int n, bool b) {
	char buf[256];
	sprintf(buf, "sec setOut():\n\tset_flag(%d, %s)\nend\n", n,
		b ? "True" : "False");
	robotcom->addCommandToQueue(buf);
	qDebug() << buf;
}

void RobotController::setDigitalOut(unsigned int n, bool b) {
	char buf[256];
	if (n > 9) {
		sprintf(buf,
			"sec setOut():\n\tset_configurable_digital_out(%d, %s)\nend\n",
			n - 10, b ? "True" : "False");
	}
	else if (n > 7) {
		sprintf(buf, "sec setOut():\n\tset_tool_digital_out(%d, %s)\nend\n",
			n - 8, b ? "True" : "False");

	}
	else {
		sprintf(buf, "sec setOut():\n\tset_standard_digital_out(%d, %s)\nend\n",
			n, b ? "True" : "False");

	}
	robotcom->addCommandToQueue(buf);
}
void RobotController::setAnalogOut(unsigned int n, double f) {
	char buf[256];
	sprintf(buf, "sec setOut():\n\tset_standard_analog_out(%d, %1.4f)\nend\n", n, f);

	robotcom->addCommandToQueue(buf);
}

void RobotController::setPayload(double m) {
	char buf[256];
	sprintf(buf, "sec setOut():\n\tset_payload(%1.3f)\nend\n", m);
	robotcom->addCommandToQueue(buf);
}


std::vector<double> RobotController::interp_cubic(double t, double T,
	std::vector<double> p0_pos, std::vector<double> p1_pos,
	std::vector<double> p0_vel, std::vector<double> p1_vel) {
	/*Returns positions of the joints at time 't' */
	std::vector<double> positions;
	for (unsigned int i = 0; i < p0_pos.size(); i++) {
		double a = p0_pos[i];
		double b = p0_vel[i];
		double c = (-3 * p0_pos[i] + 3 * p1_pos[i] - 2 * T * p0_vel[i]
			- T * p1_vel[i]) / pow(T, 2);
		double d = (2 * p0_pos[i] - 2 * p1_pos[i] + T * p0_vel[i]
			+ T * p1_vel[i]) / pow(T, 3);
		positions.push_back(a + b * t + c * pow(t, 2) + d * pow(t, 3));
	}
	return positions;
}

bool RobotController::Servo(std::vector<double> inp_timestamps,
	std::vector<std::vector<double> > inp_positions,
	std::vector<std::vector<double> > inp_velocities) {
	std::chrono::high_resolution_clock::time_point t0, t;
	std::vector<double> positions;
	unsigned int j;

	if (!uploadProg()) {
		return false;
	}

	executing_traj = true;
	t0 = std::chrono::high_resolution_clock::now();
	t = t0;
	j = 0;
	while ((inp_timestamps[inp_timestamps.size() - 1]
		>= std::chrono::duration_cast<std::chrono::duration<double>>(t - t0).count()) && executing_traj) {
		while (inp_timestamps[j]
			<= std::chrono::duration_cast<std::chrono::duration<double>>(
			t - t0).count() && j < inp_timestamps.size() - 1) {
			j += 1;
		}
		positions = interp_cubic(
			std::chrono::duration_cast<std::chrono::duration<double>>(
			t - t0).count() - inp_timestamps[j - 1],
			inp_timestamps[j] - inp_timestamps[j - 1], inp_positions[j - 1],
			inp_positions[j], inp_velocities[j - 1], inp_velocities[j]);
		servoj(positions);

		// oversample with 4 * sample_time
		std::this_thread::sleep_for(
			std::chrono::milliseconds((int)((servoj_time * 1000) / 4.)));
		t = std::chrono::high_resolution_clock::now();
	}
	executing_traj = false;
	return true;
}

void RobotController::servoj(std::vector<double> positions, int keepalive) {
	/*
	if (socketAccepted && socketAccepted->isOpen())
	{
		unsigned int bytes_written;
		int tmp;
		char buf[28];
		for (int i = 0; i < 6; i++) {
			tmp = htonl((int)(positions[i] * MULT_JOINTSTATE_));
			buf[i * 4] = tmp & 0xff;
			buf[i * 4 + 1] = (tmp >> 8) & 0xff;
			buf[i * 4 + 2] = (tmp >> 16) & 0xff;
			buf[i * 4 + 3] = (tmp >> 24) & 0xff;
		}
		tmp = htonl((int)keepalive);
		buf[6 * 4] = tmp & 0xff;
		buf[6 * 4 + 1] = (tmp >> 8) & 0xff;
		buf[6 * 4 + 2] = (tmp >> 16) & 0xff;
		buf[6 * 4 + 3] = (tmp >> 24) & 0xff;
		socketAccepted->write(buf, 28);
	}
	else{
		qDebug() << QString("UrDriver::servoj called without a reverse connection ");
		return;
	}
	*/
}

void RobotController::StopServo() {
	executing_traj= false;
	Stop();
}

bool RobotController::uploadProg() {

	/*
	if (!(socketAccepted && socketAccepted->isOpen()))
	{
		return false;
	}*/
	std::string cmd_str;
	char buf[128];
	cmd_str = "def driverProg():\n";

	sprintf(buf, "\tMULT_jointstate = %i\n", MULT_JOINTSTATE_);
	cmd_str += buf;

	cmd_str += "\tSERVO_IDLE = 0\n";
	cmd_str += "\tSERVO_RUNNING = 1\n";
	cmd_str += "\tcmd_servo_state = SERVO_IDLE\n";
	cmd_str += "\tcmd_servo_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n";
	cmd_str += "\tdef set_servo_setpoint(q):\n";
	cmd_str += "\t\tenter_critical\n";
	cmd_str += "\t\tcmd_servo_state = SERVO_RUNNING\n";
	cmd_str += "\t\tcmd_servo_q = q\n";
	cmd_str += "\t\texit_critical\n";
	cmd_str += "\tend\n";
	cmd_str += "\tthread servoThread():\n";
	cmd_str += "\t\tstate = SERVO_IDLE\n";
	cmd_str += "\t\twhile True:\n";
	cmd_str += "\t\t\tenter_critical\n";
	cmd_str += "\t\t\tq = cmd_servo_q\n";
	cmd_str += "\t\t\tdo_brake = False\n";
	cmd_str += "\t\t\tif (state == SERVO_RUNNING) and ";
	cmd_str += "(cmd_servo_state == SERVO_IDLE):\n";
	cmd_str += "\t\t\t\tdo_brake = True\n";
	cmd_str += "\t\t\tend\n";
	cmd_str += "\t\t\tstate = cmd_servo_state\n";
	cmd_str += "\t\t\tcmd_servo_state = SERVO_IDLE\n";
	cmd_str += "\t\t\texit_critical\n";
	cmd_str += "\t\t\tif do_brake:\n";
	cmd_str += "\t\t\t\tstopj(1.0)\n";
	cmd_str += "\t\t\t\tsync()\n";
	cmd_str += "\t\t\telif state == SERVO_RUNNING:\n";

	//	if (sec_interface_->robot_state_->getVersion() >= 3.1)
	sprintf(buf, "\t\t\t\tservoj(q, t=%.4f, lookahead_time=0.03)\n",
		servoj_time);
	//	else
	//		sprintf(buf, "\t\t\t\tservoj(q, t=%.4f)\n", servoj_time_);
	cmd_str += buf;

	cmd_str += "\t\t\telse:\n";
	cmd_str += "\t\t\t\tsync()\n";
	cmd_str += "\t\t\tend\n";
	cmd_str += "\t\tend\n";
	cmd_str += "\tend\n";

	sprintf(buf, "\tsocket_open(\"%s\", %i)\n", robotcom->getLocalIp().toStdString().c_str(), SERVER_PORT);
	cmd_str += buf;

	cmd_str += "\tthread_servo = run servoThread()\n";
	cmd_str += "\tkeepalive = 1\n";
	cmd_str += "\twhile keepalive > 0:\n";
	cmd_str += "\t\tparams_mult = socket_read_binary_integer(6+1)\n";
	cmd_str += "\t\tif params_mult[0] > 0:\n";
	cmd_str += "\t\t\tq = [params_mult[1] / MULT_jointstate, ";
	cmd_str += "params_mult[2] / MULT_jointstate, ";
	cmd_str += "params_mult[3] / MULT_jointstate, ";
	cmd_str += "params_mult[4] / MULT_jointstate, ";
	cmd_str += "params_mult[5] / MULT_jointstate, ";
	cmd_str += "params_mult[6] / MULT_jointstate]\n";
	cmd_str += "\t\t\tkeepalive = params_mult[7]\n";
	cmd_str += "\t\t\tset_servo_setpoint(q)\n";
	cmd_str += "\t\tend\n";
	cmd_str += "\tend\n";
	cmd_str += "\tsleep(.1)\n";
	cmd_str += "\tsocket_close()\n";
	cmd_str += "\tkill thread_servo\n";
	cmd_str += "end\n";

	robotcom->addCommandToQueue(cmd_str); 
	return true;
}

void RobotController::waitMoveCancel()
{
	m_moveCancel = true;
}

//经过某点
void RobotController::waitMovePass(RobotFrame &tar, int timeout)	//ms
{
	m_moveCancel = false;
	elapsedTimer->start();
	while (!elapsedTimer->hasExpired(timeout))
	{
		if (m_moveCancel)
		{
			break;
		}
		GlobalShared::app->processEvents(QEventLoop::WaitForMoreEvents, 10);
		if (withinTolerance(tar))
		{
			return;
		}
	}
	throw RobotMoveException();
}

//停在某点
void RobotController::waitMoveStop(RobotFrame &tar, int timeout)	//ms
{
	m_moveCancel = false;
	elapsedTimer->start();
	while (!elapsedTimer->hasExpired(timeout))
	{
		if (m_moveCancel)
		{
			break;
		}
		GlobalShared::app->processEvents(QEventLoop::WaitForMoreEvents, 10);
		if (withinTolerance(tar) && (hasStopSignal() || isStopped()))
		{
			return;
		}
	}
	throw RobotMoveException();
}

void RobotController::yeild(int timeout)
{
	m_moveCancel = false;
	elapsedTimer->start();
#ifdef QT_IMPL
	while (!elapsedTimer->hasExpired(timeout))
	{
		if (m_moveCancel)
		{
			throw RobotMoveException();
		}
		GlobalShared::app->processEvents(QEventLoop::WaitForMoreEvents, 10);
		
	}
#endif

#ifdef COROUTINE_IMPL
	QTest::qSleep(timeout);
#endif
}

bool RobotController::withinTolerance(RobotFrame &tar)
{
	RobotFrame cur = RobotController::GetRobotFrame();
	double dxdydz = 0, angle = 0, vol = 0;
	dxdydz += (tar.P[0] - cur.P[0]) * (tar.P[0] - cur.P[0]);
	dxdydz += (tar.P[1] - cur.P[1]) * (tar.P[1] - cur.P[1]);
	dxdydz += (tar.P[2] - cur.P[2]) * (tar.P[2] - cur.P[2]);
	
	angle += (tar.P[3] - cur.P[3]) * (tar.P[3] - cur.P[3]);
	angle += (tar.P[4] - cur.P[4]) * (tar.P[4] - cur.P[4]);
	angle += (tar.P[5] - cur.P[5]) * (tar.P[5] - cur.P[5]);

	//qDebug() << "dxdydz:" << b1 << " angle:" << b2 << "vol" << b3;
	return  (dxdydz < tolerance * tolerance) && (angle < 0.08*0.08 /* rad */);
}


bool RobotController::isStopped()
{
	//判断速度
	std::vector<double> v = RobotController::GetRobotState()->getTcpSpeedActual();
	double vol = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	return (vol < 0.001*0.001 /* m/s */);

}

bool RobotController::hasStopSignal()
{
	return false;
}


/*
void RobotController::initServer()
{
	tcpServer = new QTcpServer(this);
	if (!tcpServer->listen(QHostAddress::Any, SERVER_PORT))
	{
		qCritical() << QString("Unable to start the server: %1.")
			.arg(tcpServer->errorString());
		return;
	}

	QObject::connect(tcpServer, SIGNAL(newConnection()), this, SLOT(newConnection()));

	socketAccepted = nullptr;
	

}
*/


/*
void RobotController::newConnection()
{
	socketAccepted = tcpServer->nextPendingConnection();
	connect(socketAccepted, SIGNAL(readyRead()), this, SLOT(serverDataRecv()));
	connect(socketAccepted, SIGNAL(disconnected()), socketAccepted, SLOT(deleteLater()));
}


void RobotController::serverDataRecv()
{

	QString recv = socketAccepted->readAll().trimmed();
	int length = recv.section(',', 0, 0).toInt();
	qDebug()<<QString("网络数据:" + recv + QString().setNum(length) + QString().setNum(recv.length()));
	
	if (length == recv.length())
	{	
		//resolve
		QStringList strlist = recv.split(',');
	}
}
*/



/*
QDataStream 使用的是Qt自身的串行化格式  必须配套使用 >>反串行化
QDataStream out(&bytearray, QIODevice::WriteOnly);
out.device()->seek(0);
out << QString("%1").arg((bytearray.size() - sizeof(quint16),2));
*/

/*查找所有ip地址
QString ipAddress;
QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();
// use the first non-localhost IPv4 address
for (int i = 0; i < ipAddressesList.size(); ++i) {
if (ipAddressesList.at(i) != QHostAddress::LocalHost &&
ipAddressesList.at(i).toIPv4Address()) {
ipAddress = ipAddressesList.at(i).toString();
break;
}
}*/
