#include <iostream>
#include <winsock2.h>
#include "Logger.h"
#include "RobotStateRT.h"
#include <QDebug>

//#define ntohd(x) ((double)ntohll(x))
#define be64toh(x) (ntohll(x))

/*
static double inline ntohd(uint64_t nf)
{
	//Network endian to little endian
	return (double)ntohll(nf)	//big endian 64 to host
}
*/

const char* SAFETY_MODE_STRING[] =
{
	"SAFETY_MODE_NORMAL",
	"SAFETY_MODE_NORMAL",
	"SAFETY_MODE_REDUCED",
	"SAFETY_MODE_PROTECTIVE_STOP",
	"SAFETY_MODE_RECOVERY",
	"SAFETY_MODE_SAFEGUARD_STOP",
	"SAFETY_MODE_SYSTEM_EMERGENCY_STOP",
	"SAFETY_MODE_ROBOT_EMERGENCY_STOP",
	"SAFETY_MODE_VIOLATION",
	"SAFETY_MODE_FAULT",
};

const char* ROBOT_MODE_STRING[] =
{
	"ROBOT_MODE_DISCONNECTED",
	"ROBOT_MODE_CONFIRM_SAFETY",
	"ROBOT_MODE_BOOTING",
	"ROBOT_MODE_POWER_OFF",
	"ROBOT_MODE_POWER_ON",
	"ROBOT_MODE_IDLE",
	"ROBOT_MODE_BACKDRIVE",
	"ROBOT_MODE_RUNNING",
	"ROBOT_MODE_UPDATING_FIRMWARE"
};

RobotStateRT::RobotStateRT(){
	version_ = 0.0;
	time_ = 0.0;
	q_target_.assign(6, 0.0);
	qd_target_.assign(6, 0.0);
	qdd_target_.assign(6, 0.0);
	i_target_.assign(6, 0.0);
	m_target_.assign(6, 0.0);
	q_actual_.assign(6, 0.0);
	qd_actual_.assign(6, 0.0);
	i_actual_.assign(6, 0.0);
	i_control_.assign(6, 0.0);
	tool_vector_actual_.assign(6, 0.0);
	tcp_speed_actual_.assign(6, 0.0);
	tcp_force_.assign(6, 0.0);
	tool_vector_target_.assign(6, 0.0);
	tcp_speed_target_.assign(6, 0.0);
	digital_input_bits_.assign(64, false);
	digital_input_int64 = 0;
	motor_temperatures_.assign(6, 0.0);
	controller_timer_ = 0.0;
	robot_mode_ = 0.0;
	joint_modes_.assign(6, 0.0);
	safety_mode_ = 0.0;
	tool_accelerometer_values_.assign(3, 0.0);
	speed_scaling_ = 0.0;
	linear_momentum_norm_ = 0.0;
	v_main_ = 0.0;
	v_robot_ = 0.0;
	i_robot_ = 0.0;
	v_actual_.assign(6, 0.0);
	digital_output_bits_.assign(20, false);
	digital_output_int64 = 0;

	data_published_ = false;
	controller_updated_ = false;
	pMsg_cond_ = nullptr;
	prevStateTimestamp = std::chrono::high_resolution_clock::time_point(std::chrono::duration<int>(1));
}


RobotStateRT::RobotStateRT(std::condition_variable* msg_cond):
RobotStateRT()
{
	pMsg_cond_ = msg_cond;
}

RobotStateRT::~RobotStateRT() {
	/* Make sure nobody is waiting after this thread is destroyed */
	data_published_ = true;
	controller_updated_ = true;
	if (pMsg_cond_){
		pMsg_cond_->notify_all();
	}
}

RobotStateRT::RobotStateRT(const RobotStateRT& state)
{
	__copy(state);
}

RobotStateRT& RobotStateRT::operator=(const RobotStateRT& state)
{
	if (this == &state)
	{
		return *this;
	}
	__copy(state);
	return *this;
}

void RobotStateRT::__copy(const RobotStateRT& state)
{
	const_cast<RobotStateRT&>(state).val_lock_.lock();
	this->version_ = state.version_;
	this->time_ = state.time_;
	this->q_target_ = state.q_target_;
	this->qd_target_ = state.qd_target_;
	this->qdd_target_ = state.qdd_target_;
	this->i_target_ = state.i_target_;
	this->m_target_ = state.m_target_;
	this->q_actual_ = state.q_actual_;
	this->qd_actual_ = state.qd_actual_;
	this->i_actual_ = state.i_actual_;
	this->i_control_ = state.i_control_;
	this->tool_vector_actual_ = state.tool_vector_actual_;
	this->tcp_speed_actual_ = state.tcp_speed_actual_;
	this->tcp_force_ = state.tcp_force_;
	this->tool_vector_target_ = state.tool_vector_target_;
	this->tcp_speed_target_ = state.tcp_speed_target_;
	this->digital_input_bits_ = state.digital_input_bits_;
	this->digital_input_int64 = state.digital_input_int64;
	this->motor_temperatures_ = state.motor_temperatures_;
	this->controller_timer_ = state.controller_timer_;
	this->robot_mode_ = state.robot_mode_;
	this->joint_modes_ = state.joint_modes_;
	this->safety_mode_ = state.safety_mode_;
	this->tool_accelerometer_values_ = state.tool_accelerometer_values_;
	this->speed_scaling_ = state.speed_scaling_;
	this->linear_momentum_norm_ = state.linear_momentum_norm_;
	this->v_main_ = state.v_main_;
	this->v_robot_ = state.v_robot_;
	this->i_robot_ = state.i_robot_;
	this->v_actual_ = state.v_actual_;
	this->data_published_ = state.data_published_;
	this->controller_updated_ = state.controller_updated_;
	this->prevStateTimestamp = state.prevStateTimestamp;

	this->digital_output_bits_ = state.digital_output_bits_;
	this->digital_output_int64 = state.digital_output_int64;

	pMsg_cond_ = nullptr;

	const_cast<RobotStateRT&>(state).val_lock_.unlock();

}



void RobotStateRT::setDataPublished() {
	data_published_ = false;
}

bool RobotStateRT::getDataPublished() {
	return data_published_;
}

void RobotStateRT::setControllerUpdated() {
	controller_updated_ = false;
}
bool RobotStateRT::getControllerUpdated() {
	return controller_updated_;
}

void RobotStateRT::setVersion(double ver) {
	//val_lock_.lock();
	version_ = ver;
	//val_lock_.unlock();
}

double RobotStateRT::getVersion() {
	double ret;
	//val_lock_.lock();
	ret = version_;
	//val_lock_.unlock();
	return ret;
}

double RobotStateRT::getTime() {
	double ret;
	//val_lock_.lock();
	ret = time_;
	//val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getQTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = q_target_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getQdTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = qd_target_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getQddTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = qdd_target_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getITarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = i_target_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getMTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = m_target_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getQActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = q_actual_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getQdActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = qd_actual_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getIActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = i_actual_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getIControl() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = i_control_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getToolVectorActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tool_vector_actual_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getTcpSpeedActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tcp_speed_actual_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getTcpForce() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tcp_force_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getToolVectorTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tool_vector_target_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getTcpSpeedTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tcp_speed_target_;
	val_lock_.unlock();
	return ret;
}

std::vector<bool> RobotStateRT::getDigitalInputBits() {
	std::vector<bool> ret;
	val_lock_.lock();
	ret = digital_input_bits_;
	val_lock_.unlock();
	return ret;
}

std::vector<bool> RobotStateRT::getDigitalOutputBits() {
	std::vector<bool> ret;
	val_lock_.lock();
	ret = digital_output_bits_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getMotorTemperatures() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = motor_temperatures_;
	val_lock_.unlock();
	return ret;
}
double RobotStateRT::getControllerTimer() {
	double ret;
	//val_lock_.lock();
	ret = controller_timer_;
	//val_lock_.unlock();
	return ret;
}
int RobotStateRT::getRobotMode() {
	int ret;
	//val_lock_.lock();
	ret = (int)robot_mode_;
	//val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getJointModes() {
	std::vector<double> ret;
	val_lock_.lock();
	//std::cout << "Jointmode: " << safety_mode_ << std::endl;
	ret = joint_modes_;
	val_lock_.unlock();
	return ret;
}

std::string RobotStateRT::getJointModeString() {
	int mode = (int)getJointModes()[0];
	std::string str;

	switch (mode)
	{ 
	case JOINT_SHUTTING_DOWN_MODE:
		str = std::string("JOINT_SHUTTING_DOWN_MODE");
		break;

	case JOINT_PART_D_CALIBRATION_MODE:
		str = std::string("JOINT_PART_D_CALIBRATION_MODE");
		break;

	case JOINT_BACKDRIVE_MODE:
		str = std::string("JOINT_BACKDRIVE_MODE");
		break;

	case JOINT_POWER_OFF_MODE:
		str = std::string("JOINT_POWER_OFF_MODE");
		break;

	case JOINT_NOT_RESPONDING_MODE:
		str = std::string("JOINT_NOT_RESPONDING_MODE");
		break;

	case JOINT_MOTOR_INITIALISATION_MODE:
		str = std::string("JOINT_MOTOR_INITIALISATION_MODE");
		break;

	case JOINT_BOOTING_MODE:
		str = std::string("JOINT_BOOTING_MODE");
		break;

	case JOINT_PART_D_CALIBRATION_ERROR_MODE:
		str = std::string("JOINT_PART_D_CALIBRATION_ERROR_MODE");
		break;


	case JOINT_BOOTLOADER_MODE:
		str = std::string("JOINT_BOOTLOADER_MODE");
		break;


	case JOINT_CALIBRATION_MODE:
		str = std::string("JOINT_CALIBRATION_MODE");
		break;


	case JOINT_FAULT_MODE:
		str = std::string("JOINT_FAULT_MODE");
		break;


	case JOINT_RUNNING_MODE:
		str = std::string("JOINT_RUNNING_MODE");
		break;

	case JOINT_IDLE_MODE:
		str = std::string("JOINT_IDLE_MODE");
		break;

	default:
		str = std::string();
	}
	return str;
}

int RobotStateRT::getSafety_mode() {
	int ret;
	//val_lock_.lock();
	ret = (int)safety_mode_;
	//val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getToolAccelerometerValues() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tool_accelerometer_values_;
	val_lock_.unlock();
	return ret;
}
double RobotStateRT::getSpeedScaling() {
	double ret;
	//val_lock_.lock();
	ret = speed_scaling_;
	//val_lock_.unlock();
	return ret;
}
double RobotStateRT::getLinearMomentumNorm() {
	double ret;
	//val_lock_.lock();
	ret = linear_momentum_norm_;
	//val_lock_.unlock();
	return ret;
}
double RobotStateRT::getVMain() {
	double ret;
	//val_lock_.lock();
	ret = v_main_;
	//val_lock_.unlock();
	return ret;
}
double RobotStateRT::getVRobot() {
	double ret;
	//val_lock_.lock();
	ret = v_robot_;
	//val_lock_.unlock();
	return ret;
}
double RobotStateRT::getIRobot() {
	double ret;
	//val_lock_.lock();
	ret = i_robot_;
	//val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getVActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = v_actual_;
	val_lock_.unlock();
	return ret;
}

RobotFrame RobotStateRT::getFrame()
{
	val_lock_.lock();
	RobotFrame frm = RobotFrame::fromTCP(tool_vector_actual_);
	for (int i = 0; i < 6; i++)
		frm.J[i] = q_actual_[i];
	val_lock_.unlock();
	return frm;
}

std::string RobotStateRT::getStatesString()
{
	char str1[128], str2[128], str3[128];

	std::vector<double> frmJ = getQActual();
	std::vector<double> tcp = getToolVectorActual();
	std::vector<double> tcpV = getTcpSpeedActual();
	sprintf_s(str1, 128, "[%6.5f, %6.5f, %6.5f, %6.5f, %6.5f, %6.5f]\n", frmJ[0], frmJ[1], frmJ[2], frmJ[3], frmJ[4], frmJ[5]);
	sprintf_s(str2, 128, "p[%6.5f, %6.5f, %6.5f, %6.5f, %6.5f, %6.5f]\n", tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5]);
	sprintf_s(str3, 128, "TCP_V:\t%6.4f  %6.4f  %6.4f  %6.4f  %6.4f  %6.4f\n", tcpV[0], tcpV[1], tcpV[2], tcpV[3], tcpV[4], tcpV[5]);	std::string out;
	out.append(str1);
	out.append(str2);
	out.append(str3);

//	std::cout << "States:" << out << std::endl;
	return out;
}

std::string RobotStateRT::getStatesStringWtihLabel()
{
	char str1[128], str2[128], str3[128], str4[64], str5[64], str6[64], str7[65], str8[65], strtime[64];
	str1[0] = str2[0] = str3[0] = str4[0] = str5[0] = str6[0] = str7[0] = str8[0] = strtime[0] = '\0';

	std::vector<double> frmJ = getQActual();
	std::vector<double> tcp = getToolVectorActual();
	std::vector<double> tcpV = getTcpSpeedActual();
	sprintf_s(str1, 128, "Joint:\t%6.4f  %6.4f  %6.4f  %6.4f  %6.4f  %6.4f\n", frmJ[0], frmJ[1], frmJ[2], frmJ[3], frmJ[4], frmJ[5]);
	sprintf_s(str2, 128, "TCP:  \t%6.4f  %6.4f  %6.4f  %6.4f  %6.4f  %6.4f\n", tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5]);
	sprintf_s(str3, 128, "TCP_V:\t%6.4f  %6.4f  %6.4f  %6.4f  %6.4f  %6.4f\n", tcpV[0], tcpV[1], tcpV[2], tcpV[3], tcpV[4], tcpV[5]);
	//Robot mode
	int enumid;
	enumid = getRobotMode();
	sprintf_s(str4, 64, "RobotMode: %d %s\n", enumid, ROBOT_MODE_STRING[enumid]);
	//Joint mode
	sprintf_s(str5, 64, "JointMode: %s\n", getJointModeString().c_str());
	//safety mode
	enumid = getSafety_mode();
	sprintf_s(str6, 64, "SafetyMode: %d %s\n", enumid, SAFETY_MODE_STRING[enumid]);
	//digital_input
	std::string inputstr = IObitsToString(digital_input_bits_);
	memcpy_s(str7, 64, inputstr.c_str(), 64);
	str7[64] = '\0';
	//digital_output
	std::string outputstr = IObitsToString(digital_output_bits_);

	memcpy_s(str8, 64, outputstr.c_str(), 64);
	str8[64] = '\0';
	//time
	sprintf_s(strtime, 64, "Controller Timer: %8.6f\n",getTime());

	std::string out;

	out.append(str1);
	out.append(str2);
	out.append(str3);
	out.append(str4);
	out.append(str5);
	out.append(str6);
	out.append("DigitalInputs:\t");
	out.append(str7);
	out.append("\n");
	out.append("DigitalOutputs:\t");
//	std::cout << std::hex << digital_output_int64 << '\t';
	out.append(str8);
	out.append("\n");
	out.append(strtime);

	return out;

}

bool RobotStateRT::isValid()
{
	bool valid = false;
	std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();
	val_lock_.lock();
	//auto ns = std::chrono::duration_cast<std::chrono::milliseconds>(t - prevStateTimestamp);
	std::chrono::duration<double, std::milli> dt = t - prevStateTimestamp;
	if (dt.count() < 100)
	{
		valid = true;
	}
	val_lock_.unlock();
	return valid;
}

std::vector<double> RobotStateRT::unpackVector(uint8_t * buf, int start_index,
	int nr_of_vals) {
	uint64_t q;
	std::vector<double> ret;
	for (int i = 0; i < nr_of_vals; i++) {
		memcpy(&q, &buf[start_index + i * sizeof(q)], sizeof(q));
		ret.push_back(ntohd(q));
	}
	return ret;
}

std::vector<bool> RobotStateRT::unpackDigitalBits(int64_t data) {
	std::vector<bool> ret;
	for (int i = 0; i < 64; i++) {
		ret.push_back(data & (1 << i));
	}
	return ret;
}

std::string RobotStateRT::IObitsToString(std::vector<bool> bits)
{
	std::string s;
	for (int i = 0; i < 20; i++)
	{
		if (i %4 == 0)
			s.push_back(' ');
		if (bits[i])
			s.push_back('1');
		else
			s.push_back('0');
	}
	return s;
}

void RobotStateRT::unpack(uint8_t * buf) {
	uint64_t unpack_to;
	uint16_t offset = 0;
	int len;
	memcpy(&len, &buf[offset], sizeof(len));
	offset += sizeof(len);
	len = ntohl(len);

	//Check the correct message length is received
	bool len_good = true;
	if (version_ >= 1.6 && version_ < 1.7) { //v1.6
		if (len != 756)
			len_good = false;
	} else if (version_ >= 1.7 && version_ < 1.8) { //v1.7
		if (len != 764)
			len_good = false;
	} else if (version_ >= 1.8 && version_ < 1.9) { //v1.8
		if (len != 812)
			len_good = false;
	} else if (version_ >= 3.0 && version_ < 3.2) { //v3.0 & v3.1
		if (len != 1044)
			len_good = false;
	} else if (version_ >= 3.2 && version_ < 3.4) { //v3.2 & v3.3
		if (len != 1060 && len != 2120)
			len_good = false;
	}

	if (!len_good) {
		printf("Wrong message length %i\n", len);
		return;
	}

	val_lock_.lock();
	memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
	time_ = ntohd(unpack_to);
	offset += sizeof(double);

	q_target_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	
	qd_target_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	
	qdd_target_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	
	i_target_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	
	m_target_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	
	q_actual_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	
	qd_actual_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	
	i_actual_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	if (version_ <= 1.8) {
		if (version_ != 1.6)
			tool_accelerometer_values_ = unpackVector(buf, offset, 3);
		offset += sizeof(double) * (3 + 15);
		tcp_force_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;
		tool_vector_actual_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;
		tcp_speed_actual_ = unpackVector(buf, offset, 6);
	} else {
		i_control_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;
		
		tool_vector_actual_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;
		
		tcp_speed_actual_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;
		
		tcp_force_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;
		
		tool_vector_target_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;
		
		tcp_speed_target_ = unpackVector(buf, offset, 6);
	}
	offset += sizeof(double) * 6;

	memcpy(&digital_input_int64, &buf[offset], sizeof(digital_input_int64));
	digital_input_bits_ = unpackDigitalBits(ntohd(digital_input_int64));
	offset += sizeof(double);
	
	motor_temperatures_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	
	memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
	controller_timer_ = ntohd(unpack_to);
	if (version_ > 1.6) {
		offset += sizeof(double) * 2;
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		robot_mode_ = ntohd(unpack_to);
		if (version_ > 1.7) {
			offset += sizeof(double);
			std::vector<double> jmodes = unpackVector(buf, offset, 6);
			for (int jjj = 0; jjj < 6; jjj++){
				joint_modes_[jjj] = (int)jmodes[jjj];
			}
		offset += sizeof(double) * 6;
		}
	}
	if (version_ > 1.8) {
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		safety_mode_ = ntohd(unpack_to);
		offset += sizeof(double);

		//Used by Universal Robots software only
		offset += sizeof(double) * 6;

		tool_accelerometer_values_ = unpackVector(buf, offset, 3);
		offset += sizeof(double) * 3;

		//Used by Universal Robots software only
		offset += sizeof(double) * 6;

		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		speed_scaling_ = ntohd(unpack_to);
		offset += sizeof(double);
		
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		linear_momentum_norm_ = ntohd(unpack_to);
		offset += sizeof(double);

		//Used by Universal Robots software only
		offset += sizeof(double) * 2;

		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		v_main_ = ntohd(unpack_to);
		offset += sizeof(double);
		
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		v_robot_ = ntohd(unpack_to);
		offset += sizeof(double);
		
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		i_robot_ = ntohd(unpack_to);
		offset += sizeof(double);
		
		v_actual_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;

		memcpy(&digital_output_int64, &buf[offset], sizeof(digital_output_int64));
		digital_output_bits_ = unpackDigitalBits(ntohd(digital_output_int64));
		offset += sizeof(double);

		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		program_state_ = ntohd(unpack_to);
//		offset += sizeof(double);
	}

	prevStateTimestamp = std::chrono::high_resolution_clock::now();
	controller_updated_ = true;
	data_published_ = true;
	val_lock_.unlock();
	if (pMsg_cond_){
		pMsg_cond_->notify_all();
	}
}

