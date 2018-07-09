#pragma once


/*
解析Realtime communication interface(30003)的数据
Only robot state and Version messages
不含网络连接
*/

#include <cstdint>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include "RobotFrame.h"

enum SAFETY_MODE
{
	SAFETY_MODE_NORMAL	=1,
	SAFETY_MODE_REDUCED	=2,
	SAFETY_MODE_PROTECTIVE_STOP	=3,
	SAFETY_MODE_RECOVERY	=4,
	SAFETY_MODE_SAFEGUARD_STOP	=5,
	SAFETY_MODE_SYSTEM_EMERGENCY_STOP	=6,
	SAFETY_MODE_ROBOT_EMERGENCY_STOP	=7,
	SAFETY_MODE_VIOLATION	=8,
	SAFETY_MODE_FAULT	=9
};


enum ROBOT_MODE{
	ROBOT_MODE_DISCONNECTED,
	ROBOT_MODE_CONFIRM_SAFETY,
	ROBOT_MODE_BOOTING,
	ROBOT_MODE_POWER_OFF,
	ROBOT_MODE_POWER_ON,
	ROBOT_MODE_IDLE,
	ROBOT_MODE_BACKDRIVE,
	ROBOT_MODE_RUNNING,
	ROBOT_MODE_UPDATING_FIRMWARE,
};

enum JOINT_MODE{
	JOINT_SHUTTING_DOWN_MODE	= 236,
	JOINT_PART_D_CALIBRATION_MODE	=237,
	JOINT_BACKDRIVE_MODE	=238,
	JOINT_POWER_OFF_MODE	=239,
	JOINT_NOT_RESPONDING_MODE	=245,
	JOINT_MOTOR_INITIALISATION_MODE	= 246,
	JOINT_BOOTING_MODE	= 247,
	JOINT_PART_D_CALIBRATION_ERROR_MODE	=248,
	JOINT_BOOTLOADER_MODE	= 249,	
	JOINT_CALIBRATION_MODE	=250,
	JOINT_FAULT_MODE	=252,
	JOINT_RUNNING_MODE	=253,
	JOINT_IDLE_MODE	=255
};


class RobotStateRT {

public:
	RobotStateRT();
	RobotStateRT(std::condition_variable* msg_cond);
	~RobotStateRT();
	RobotStateRT(const RobotStateRT& state);
	RobotStateRT& operator=(const RobotStateRT& state);

	RobotFrame getFrame();
	std::vector<double> getQActual();
	std::vector<double> getToolVectorActual();
	std::string getStatesString();
	std::string getStatesStringWtihLabel();


	double getVersion();
	double getTime();
	std::vector<double> getQTarget();
	std::vector<double> getQdTarget();
	std::vector<double> getQddTarget();
	std::vector<double> getITarget();
	std::vector<double> getMTarget();
	std::vector<double> getQdActual();
	std::vector<double> getIActual();
	std::vector<double> getIControl();
	std::vector<double> getTcpSpeedActual();
	std::vector<double> getTcpForce();
	std::vector<double> getToolVectorTarget();
	std::vector<double> getTcpSpeedTarget();
	std::vector<bool> getDigitalInputBits();
	std::vector<bool> getDigitalOutputBits();
	std::vector<double> getMotorTemperatures();
	double getControllerTimer();
	int getRobotMode();
	std::vector<double> getJointModes();
	std::string getJointModeString();
	int getSafety_mode();
	std::vector<double> getToolAccelerometerValues();
	double getSpeedScaling();
	double getLinearMomentumNorm();
	double getVMain();
	double getVRobot();
	double getIRobot();

	void setVersion(double ver);

	void setDataPublished();
	bool getDataPublished();
	bool getControllerUpdated();
	void setControllerUpdated();
	std::vector<double> getVActual();
	bool isValid();
	void unpack(uint8_t * buf);

private:
	double version_; //protocol version

	double time_; //Time elapsed since the controller was started
	std::vector<double> q_target_; //Target joint positions
	std::vector<double> qd_target_; //Target joint velocities
	std::vector<double> qdd_target_; //Target joint accelerations
	std::vector<double> i_target_; //Target joint currents
	std::vector<double> m_target_; //Target joint moments (torques)
	std::vector<double> q_actual_; //Actual joint positions
	std::vector<double> qd_actual_; //Actual joint velocities
	std::vector<double> i_actual_; //Actual joint currents
	std::vector<double> i_control_; //Joint control currents
	std::vector<double> tool_vector_actual_; //Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
	std::vector<double> tcp_speed_actual_; //Actual speed of the tool given in Cartesian coordinates
	std::vector<double> tcp_force_; //Generalised forces in the TCP
	std::vector<double> tool_vector_target_; //Target Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
	std::vector<double> tcp_speed_target_; //Target speed of the tool given in Cartesian coordinates
	std::vector<bool> digital_input_bits_; //Current state of the digital inputs. NOTE: these are bits encoded as int64_t, e.g. a value of 5 corresponds to bit 0 and bit 2 set high
	int64_t digital_input_int64;
	std::vector<double> motor_temperatures_; //Temperature of each joint in degrees celsius
	double controller_timer_; //Controller realtime thread execution time
	double robot_mode_; //Robot mode
	std::vector<double> joint_modes_; //Joint control modes
	double safety_mode_; //Safety mode
	std::vector<double> tool_accelerometer_values_; //Tool x,y and z accelerometer values (software version 1.7)
	double speed_scaling_; //Speed scaling of the trajectory limiter
	double linear_momentum_norm_; //Norm of Cartesian linear momentum
	double v_main_; //Masterboard: Main voltage
	double v_robot_; //Matorborad: Robot voltage (48V)
	double i_robot_; //Masterboard: Robot current
	std::vector<double> v_actual_; //Actual joint voltages
	std::vector<bool> digital_output_bits_;
	int64_t digital_output_int64;

	double program_state_;
	std::chrono::high_resolution_clock::time_point prevStateTimestamp;
	std::mutex val_lock_; // Locks the variables while unpack parses data;

	std::condition_variable* pMsg_cond_; //Signals that new vars are available
	bool data_published_; //to avoid spurious wakes
	bool controller_updated_; //to avoid spurious wakes

	std::vector<double> unpackVector(uint8_t * buf, int start_index, int nr_of_vals);
	std::vector<bool> unpackDigitalBits(int64_t data);
	static std::string IObitsToString(std::vector<bool> bits);
	void __copy(const RobotStateRT& state);
};
