#pragma once
#include <vector>

class RobotFrame
{
public:
	RobotFrame();
	static RobotFrame fromJointsRad(const double joints[6]);
	static RobotFrame fromJointsRad(const std::vector<double> &joints);
	static RobotFrame fromTCP(const double tcp[6]);
	static RobotFrame fromTCP(const std::vector<double> &tcp);
	static RobotFrame fromString(const char* pstr);
	static double rad2deg(double rad);
	static double deg2rad(double deg);

	double J[6];
	double P[6];

	void toRad();
	void toDeg();
	void forwardKinematics();

	std::string tcpToString();
	std::string jointsToString();
	void print();

	void save(const std::string &filename);
	void load(const std::string &filename);
	bool inline isValid() const	{
		return m_valid;
	}

	friend std::ostream &operator<<(std::ostream &s, const RobotFrame &A);

protected:
	double *pOut;
	bool m_valid;
	
	bool inverseKinematics(); //unfinished
};
