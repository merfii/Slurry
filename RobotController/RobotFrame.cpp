#pragma execution_character_set("utf-8")
#define _USE_MATH_DEFINES
#include <visp3/core/vpHomogeneousMatrix.h>
#include "RobotController.h"
#include "RobotStateRT.h"
#include "RobotFrame.h"

static std::vector<double> forward(double Q[6]);
static std::vector<double> forward_to_joint(double Q[6], int joint_idx);
static std::vector<double> inverse(const double T[4][4], double NaN);
static std::vector<double> findNearestSolution(std::vector<double> &solutions, double currentPose[6]);

RobotFrame::RobotFrame()
{
	for (int i = 0; i < 6; i++)
		J[i] = NAN;

	for (int i = 0; i < 6; i++)
		P[i] = NAN;

	pOut = nullptr;
	m_valid = false;
}

RobotFrame RobotFrame::fromJointsRad(const double joints[6])
{
	RobotFrame rf;
	for (int i = 0; i < 6; i++)
		rf.J[i] = joints[i];
	rf.m_valid = true;

	return rf;
}

RobotFrame RobotFrame::fromJointsRad(const std::vector<double> &joints)
{
	RobotFrame rf;
	for (int i = 0; i < 6; i++)
		rf.J[i] = joints[i];
	rf.m_valid = true;
	return rf;
}


RobotFrame RobotFrame::fromTCP(const double tcp[6])
{
	RobotFrame rf;
	for (int i = 0; i < 6; i++)
	{
		rf.P[i] = tcp[i];
	}
	rf.m_valid = true;
	return rf;
}

RobotFrame RobotFrame::fromTCP(const std::vector<double> &tcp)
{
	RobotFrame rf;
	for (int i = 0; i < 6; i++)
		rf.P[i] = tcp[i];
	rf.m_valid = true;
	return rf;
}

RobotFrame RobotFrame::fromString(const char* pstr)
{
	std::stringstream stream;
	double dat[6];

	while (*pstr != '[' && *pstr != 'p')
	{
		pstr ++;
		if (*pstr == '\0'){
			return RobotFrame();
		}
	}
	if (*pstr == 'p')
	{
		// TCP
		pstr++;
		if (6 == sscanf_s(pstr, "[%lf,%lf,%lf,%lf,%lf,%lf]", dat, dat + 1, dat + 2, dat + 3, dat + 4, dat + 5))
			return RobotFrame::fromTCP(dat);
		return RobotFrame();
	}
	else
	{
		// Joints
		if (6 == sscanf_s(pstr, "[%lf,%lf,%lf,%lf,%lf,%lf]", dat, dat + 1, dat + 2, dat + 3, dat + 4, dat + 5))
			return RobotFrame::fromJointsRad(dat);
		return RobotFrame();
	}
	return RobotFrame();
}

double RobotFrame::rad2deg(double rad)
{
	return rad * M_1_PI * 180;
}

double RobotFrame::deg2rad(double deg)
{
	return deg / 180 * M_PI;
}

void RobotFrame::toRad()
{
	for (int i = 0; i < 6; i++)
		J[i] = deg2rad(J[i]);
}

void RobotFrame::toDeg()
{
	for (int i = 0; i < 6; i++)
		J[i] = rad2deg(J[i]);
}

//证运动学也是有误差的 约+-0.5mm
void RobotFrame::forwardKinematics()
{
	vpHomogeneousMatrix H(forward_to_joint(J,6));
	//std::cout << H;
	vpTranslationVector trans = H.getTranslationVector();
	vpThetaUVector theta = H.getThetaUVector();
	
	P[0] = trans[0]; P[1] = trans[1]; P[2] = trans[2];
	P[3] = theta[0]; P[4] = theta[1]; P[5] = theta[2];
}

//!!!!!!!!!!!!!!!!!这个函数有问题 有bug 别用!!!!!!!!!!!!!!!!!
bool RobotFrame::inverseKinematics()
{

	double T[4][4];
	vpHomogeneousMatrix H(P[0], P[1], P[2], P[3], P[4], P[5]);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			T[i][j] = H[i][j];
		}
	}
	std::vector<double> sol = inverse(T, NAN);
	if (sol.size() < 6)
		return false;
	else
	{
		std::vector<double> jj;
		jj = findNearestSolution(sol, RobotController::GetRobotFrame().J);
		for (int i = 0; i < 6; i++)
			J[i] = jj[i];
		return true;
	}
}


void RobotFrame::print()
{
	std::cout << "Robot Frame: " << std::endl;
	pOut = J;
	std::cout << *this << std::endl;
	pOut = P;
	std::cout << *this << std::endl;
}

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>


std::string RobotFrame::tcpToString()
{
	std::ostringstream strm;
	pOut = P;
	strm << "p[" << *this << "]";
	return strm.str();
}

std::string RobotFrame::jointsToString()
{
	std::ostringstream strm;
	pOut = J;
	strm << "[" << *this << "]";
	return strm.str();
}

std::ostream &operator<<(std::ostream &s, const RobotFrame &A)
{
	std::ios_base::fmtflags original_flags = s.flags();

	s.precision(6);
	int i;
	for (i = 0; i<5; i++) {
		s << A.pOut[i] << ", ";
	}
	// We don't add "  " after the last row element
	s << A.pOut[i];

	s.flags(original_flags); // restore s to standard state
	return s;
}

void RobotFrame::save(const std::string &filename)
{
	std::fstream file;
	file.open(filename.c_str(), std::fstream::out | std::fstream::app);		//std::fstream::binary);
	if (!file) {
		file.close();
		return;
	}

	unsigned int i = 0;
	const char* header = "RobotFrame";
	// 写入注释
	file << "# ";
	while (header[i] != '\0') {
		file << header[i];
		if (header[i] == '\n')
			file << "# ";
		i++;
	}
	file << std::endl;
	file << jointsToString() << std::endl;
	file << tcpToString() << std::endl;
	file.close();
}


void RobotFrame::load(const std::string &filename)
{
	std::fstream file;
	file.open(filename.c_str(), std::fstream::in);
	if (!file) {
		file.close();
		return;
	}

	std::string h;
	bool headerIsDecoded = false;
	char line[128];
	do {
		std::streampos pos = file.tellg();
		file.getline(line, 128);
		std::string prefix("# ");
		std::string line_(line);
		if (line_.compare(0, 2, "# ") == 0) {
			// Line is a comment
			// If we are not on the first line, we should add "\n" to the end of the previous line
			if (pos)
				h += "\n";
			h += line_.substr(2); // Remove "# "
		}
		else {
			// rewind before the line
			file.seekg(pos, file.beg);
			headerIsDecoded = true;
		}
	} while (!headerIsDecoded);

	if (!h.empty()) {
		std::cout << "Load: " << h << std::endl;
	}
	line[0] = '\0';
	file.getline(line, 128);
	if (line[0])
	{
		RobotFrame frm = RobotFrame::fromString(line);
		for (int i = 0; i < 6; i++)
		{
			J[i] = frm.J[i];
		}
	}
	line[0] = '\0';
	file.getline(line, 128);
	if (line[0] != '\0')
	{
		RobotFrame frm = RobotFrame::fromString(line);
		for (int i = 0; i < 6; i++)
		{
			P[i] = frm.P[i];
		}
	}
	file.close();
}

/*
static bool loadYAML(const std::string &filename)
{
	std::fstream file;
	file.open(filename.c_str(), std::fstream::in);
	if (!file) {
		file.close();
		return false;
	}

	unsigned int rows = 0, cols = 0;
	std::string h;
	std::string line, subs;
	bool inheader = true;
	unsigned int i = 0, j;
	unsigned int lineStart = 0;

	while (getline(file, line)) {
		if (inheader) {
			if (rows == 0 && line.compare(0, 5, "rows:") == 0) {
				std::stringstream ss(line);
				ss >> subs;
				ss >> rows;
			}
			else if (cols == 0 && line.compare(0, 5, "cols:") == 0) {
				std::stringstream ss(line);
				ss >> subs;
				ss >> cols;
			}
			else if (line.compare(0, 5, "data:") == 0)
				inheader = false;
			else
				h += line + "\n";
		}
		else {
			// if i == 0, we just got out of the header: initialize matrix dimensions
			if (i == 0) {
				if (rows == 0 || cols == 0) {
					file.close();
					return false;
				}
				A.resize(rows, cols);
				// get indentation level which is common to all lines
				lineStart = (unsigned int)line.find("[") + 1;
			}
			std::stringstream ss(line.substr(lineStart, line.find("]") - lineStart));
			j = 0;
			while (getline(ss, subs, ','))
				A[i][j++] = atof(subs.c_str());
			i++;
		}
	}

	if (! h.empty()) {
		std::cout << h << std::endl;
	}

	file.close();
	return true;
}
*/


#define UR5_PARAMS
#ifdef UR5_PARAMS
static const double d1 = 0.089159;
static const double a2 = -0.42500;
static const double a3 = -0.39225;
static const double d4 = 0.10915;
static const double d5 = 0.09465;
static const double d6 = 0.0823;
#endif

// 这个函数的输入输出各轴的顺序是乱的，别用这个。用forward_to_joint(Q,6)
// Transform from base link to 0th link
// -1,  0,  0,  0
//  0, -1,  0,  0
//  0,  0,  1,  0
//  0,  0,  0,  1

// Transform from 6th link to end effector
//  0, -1,  0,  0
//  0,  0, -1,  0
//  1,  0,  0,  0
//  0,  0,  0,  1
// @param q       The 6 joint values
// @return T       The 4x4 end effector pose in row-major ordering
static std::vector<double> forward(double Q[6])
{
	// s1~s6 对应 Q0~Q5
	double s1 = sin(Q[0]), c1 = cos(Q[0]);
	double s2 = sin(Q[1]), c2 = cos(Q[1]);
	double s3 = sin(Q[2]), c3 = cos(Q[2]); 
	double s5 = sin(Q[4]), c5 = cos(Q[4]);
	double s6 = sin(Q[5]), c6 = cos(Q[5]);
	double q234 = Q[1] + Q[2] + Q[3];
	double s234 = sin(q234), c234 = cos(q234);
	
	std::vector<double> T;
	T.resize(16);
	T[0] = ((c1*c234 - s1*s234)*s5) / 2.0 - c5*s1 + ((c1*c234 + s1*s234)*s5) / 2.0;	//-Ax

	T[1] = (c6*(s1*s5 + ((c1*c234 - s1*s234)*c5) / 2.0 + ((c1*c234 + s1*s234)*c5) / 2.0) -	//Nx
		(s6*((s1*c234 + c1*s234) - (s1*c234 - c1*s234))) / 2.0);

	T[2] = (-(c6*((s1*c234 + c1*s234) - (s1*c234 - c1*s234))) / 2.0 -	//Ox
		s6*(s1*s5 + ((c1*c234 - s1*s234)*c5) / 2.0 + ((c1*c234 + s1*s234)*c5) / 2.0));

	T[3] = ((d5*(s1*c234 - c1*s234)) / 2.0 - (d5*(s1*c234 + c1*s234)) / 2.0 -	// -Px
		d4*s1 + (d6*(c1*c234 - s1*s234)*s5) / 2.0 + (d6*(c1*c234 + s1*s234)*s5) / 2.0 -
		a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3);

	T[4] = c1*c5 + ((s1*c234 + c1*s234)*s5) / 2.0 + ((s1*c234 - c1*s234)*s5) / 2.0;	//-Ay

	T[5] = (c6*(((s1*c234 + c1*s234)*c5) / 2.0 - c1*s5 + ((s1*c234 - c1*s234)*c5) / 2.0) +	//Ny
		s6*((c1*c234 - s1*s234) / 2.0 - (c1*c234 + s1*s234) / 2.0));

	T[6] = (c6*((c1*c234 - s1*s234) / 2.0 - (c1*c234 + s1*s234) / 2.0) -	//Oy
		s6*(((s1*c234 + c1*s234)*c5) / 2.0 - c1*s5 + ((s1*c234 - c1*s234)*c5) / 2.0));

	T[7] = ((d5*(c1*c234 - s1*s234)) / 2.0 - (d5*(c1*c234 + s1*s234)) / 2.0 + d4*c1 +
		(d6*(s1*c234 + c1*s234)*s5) / 2.0 + (d6*(s1*c234 - c1*s234)*s5) / 2.0 + d6*c1*c5 -
		a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3); 	//-Py

	T[8] = ((c234*c5 - s234*s5) / 2.0 - (c234*c5 + s234*s5) / 2.0);	//Az

	T[9] = ((s234*c6 - c234*s6) / 2.0 - (s234*c6 + c234*s6) / 2.0 - s234*c5*c6); //-Nz

	T[10] = (s234*c5*s6 - (c234*c6 + s234*s6) / 2.0 - (c234*c6 - s234*s6) / 2.0);	//-Oz

	T[11] = (d1 + (d6*(c234*c5 - s234*s5)) / 2.0 + a3*(s2*c3 + c2*s3) + a2*s2 -	//Pz
		(d6*(c234*c5 + s234*s5)) / 2.0 - d5*c234);

	T[12] = T[13] = T[14] = 0.0;
	T[15] = 1.0;
	return T;
}

// @param q       The 6 joint values 
// @param Ti      The 4x4 pose of No.(joint_idx) joint in row-major ordering
static std::vector<double> forward_to_joint(double Q[6], int joint_idx)
{
	double s1 = sin(Q[0]), c1 = cos(Q[0]); // q1
	double s2 = sin(Q[1]), c2 = cos(Q[1]); // q2
	double s3 = sin(Q[2]), c3 = cos(Q[2]);
	double s5 = sin(Q[4]), c5 = cos(Q[4]); // q5
	double s6 = sin(Q[5]), c6 = cos(Q[5]); // q6
	double s23 = sin(Q[1] + Q[2]), c23 = cos(Q[1] + Q[2]);
	double s234 = sin(Q[1] + Q[2] + Q[3]), c234 = cos(Q[1] + Q[2] + Q[3]);

	std::vector<double> T;
	T.resize(16);

	switch (joint_idx)
	{
	case 1:
		T[0] = c1;
		T[1] = 0;
		T[2] = s1;
		T[3] = 0;
		T[4] = s1;
		T[5] = 0;
		T[6] = -c1;
		T[7] = 0;
		T[8] = 0;
		T[9] = 1;
		T[10] = 0;
		T[11] = d1;
		T[12] = 0;
		T[13] = 0;
		T[14] = 0;
		T[15] = 1;
		break;

	case 2:
		T[0] = c1*c2;
		T[1] = -c1*s2;
		T[2] = s1;
		T[3] = a2*c1*c2;
		T[4] = c2*s1;
		T[5] = -s1*s2;
		T[6] = -c1;
		T[7] = a2*c2*s1; 
		T[8] = s2;
		T[9] = c2;
		T[10] = 0;
		T[11] = d1 + a2*s2; 
		T[12] = 0;
		T[13] = 0;
		T[14] = 0;
		T[15] = 1;
		break;

	case 3:
		T[0] = c23*c1; 
		T[1] = -s23*c1; 
		T[2] = s1; 
		T[3] = c1*(a3*c23 + a2*c2); 
		T[4] = c23*s1; 
		T[5] = -s23*s1; 
		T[6] = -c1; 
		T[7] = s1*(a3*c23 + a2*c2); 
		T[8] = s23; 
		T[9] = c23; 
		T[10] = 0; 
		T[11] = d1 + a3*s23 + a2*s2; 
		T[12] = 0; 
		T[13] = 0; 
		T[14] = 0; 
		T[15] = 1;
		break;

	case 4:
		T[0] = c234*c1; 
		T[1] = s1; 
		T[2] = s234*c1; 
		T[3] = c1*(a3*c23 + a2*c2) + d4*s1; 
		T[4] = c234*s1; 
		T[5] = -c1; 
		T[6] = s234*s1; 
		T[7] = s1*(a3*c23 + a2*c2) - d4*c1; 
		T[8] = s234; 
		T[9] = 0; 
		T[10] = -c234; 
		T[11] = d1 + a3*s23 + a2*s2; 
		T[12] = 0; 
		T[13] = 0; 
		T[14] = 0; 
		T[15] = 1;
		break;

	case 5:
		T[0] = s1*s5 + c234*c1*c5; 
		T[1] = -s234*c1; 
		T[2] = c5*s1 - c234*c1*s5; 
		T[3] = c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; 
		T[4] = c234*c5*s1 - c1*s5; 
		T[5] = -s234*s1; 
		T[6] = -c1*c5 - c234*s1*s5; 
		T[7] = s1*(a3*c23 + a2*c2) - d4*c1 + d5*s234*s1; 
		T[8] = s234*c5; 
		T[9] = c234; 
		T[10] = -s234*s5; 
		T[11] = d1 + a3*s23 + a2*s2 - d5*c234; 
		T[12] = 0; 
		T[13] = 0; 
		T[14] = 0; 
		T[15] = 1;
		break;

	case 6:
		T[0] = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; 
		T[1] = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; 
		T[2] = c5*s1 - c234*c1*s5; 
		T[3] = d6*(c5*s1 - c234*c1*s5) + c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; 
		T[4] = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; 
		T[5] = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; 
		T[6] = -c1*c5 - c234*s1*s5; 
		T[7] = s1*(a3*c23 + a2*c2) - d4*c1 - d6*(c1*c5 + c234*s1*s5) + d5*s234*s1; 
		T[8] = c234*s6 + s234*c5*c6; 
		T[9] = c234*c6 - s234*c5*s6; 
		T[10] = -s234*s5; 
		T[11] = d1 + a3*s23 + a2*s2 - d5*c234 - d6*s234*s5; 
		T[12] = 0; 
		T[13] = 0; 
		T[14] = 0; 
		T[15] = 1;
		break;
	}
	return T;
}


const double ZERO_THRESH = 0.00000001;
static int SIGN(double x) {
	return (x > 0) - (x < 0);
}

//！！！！！！！！！！！！！！！这个函数有问题 有bug  别用！！！！！！！！！！！！！！！！！！！
// @param T       The 4x4 end effector pose in row-major ordering
// @param NaN  An optional parameter which designates what the q6 value should take in case of an infinite solution on that joint.
// @return        An 8x6 array of doubles returned, all angles should be in [0,2*PI), maximum number of solutions found is 8
static std::vector<double> inverse(const double T[4][4], double NaN)
{
	std::vector<double> q_sols;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
		std::cout << T[i][j] << " ";
		}
	}
	////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
	double q1[2];
	{
		double A = d6*T[1][2] - T[1][3];
		double B = d6*T[0][2] - T[0][3];
		double R = A*A + B*B;
		if (fabs(A) < ZERO_THRESH) {
			double div;
			if (fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
				div = -SIGN(d4)*SIGN(B);
			else
				div = -d4 / B;
			double arcsin = asin(div);
			if (fabs(arcsin) < ZERO_THRESH)
				arcsin = 0.0;
			if (arcsin < 0.0)
				q1[0] = arcsin + 2.0 * M_PI;
			else
				q1[0] = arcsin;
			q1[1] = M_PI - arcsin;
		}
		else if (fabs(B) < ZERO_THRESH) {
			double div;
			if (fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
				div = SIGN(d4)*SIGN(A);
			else
				div = d4 / A;
			double arccos = acos(div);
			q1[0] = arccos;
			q1[1] = 2.0 * M_PI - arccos;
		}
		else if (d4*d4 > R) {
			return std::vector<double>();
		}
		else {
			double arccos = acos(d4 / sqrt(R));
			double arctan = atan2(-B, A);
			double pos = arccos + arctan;
			double neg = -arccos + arctan;
			if (fabs(pos) < ZERO_THRESH)
				pos = 0.0;
			if (fabs(neg) < ZERO_THRESH)
				neg = 0.0;
			if (neg >= 0.0)
				q1[1] = neg;
			else
				q1[1] = 2.0 * M_PI + neg;
			if (pos >= 0.0)
				q1[0] = pos;
			else
				q1[0] = 2.0 * M_PI + pos;

		}
	}

	////////////////////////////// wrist 2 joint (q5) //////////////////////////////
	double q5[2][2];
	{
		for (int i = 0; i<2; i++) {
			double numer = (T[0][3]*sin(q1[i]) - T[1][3]*cos(q1[i]) - d4);
			double div;
			if (fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
				div = SIGN(numer) * SIGN(d6);
			else
				div = numer / d6;
			double arccos = acos(div);
			q5[i][0] = arccos;
			q5[i][1] = 2.0 * M_PI - arccos;
		}
	}
	////////////////////////////////////////////////////////////////////////////////

	{
		for (int i = 0; i<2; i++) {
			for (int j = 0; j<2; j++) {
				double c1 = cos(q1[i]), s1 = sin(q1[i]);
				double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
				double q6;
				////////////////////////////// wrist 3 joint (q6) //////////////////////////////
				if (fabs(s5) < ZERO_THRESH)
					q6 = NaN;
				else {
					q6 = atan2(SIGN(s5)*-(T[0][1]*s1 - T[1][1]*c1),
						SIGN(s5)*(T[0][0]*s1 - T[1][0]*c1));
					if (fabs(q6) < ZERO_THRESH)
						q6 = 0.0;
					if (q6 < 0.0)
						q6 += 2.0 * M_PI;
				}

				///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
				double q2[2], q3[2], q4[2];
				double c6 = cos(q6), s6 = sin(q6);
				double x04x = -s5*(T[0][2]*c1 + T[1][2]*s1) - c5*(s6*(T[0][1]*c1 + T[1][1]*s1) - c6*(T[0][0]*c1 + T[1][0]*s1));
				double x04y = c5*(T[2][0]*c6 - T[2][1]*s6) - T[2][2]*s5;
				double p13x = d5*(s6*(T[0][0]*c1 + T[1][0]*s1) + c6*(T[0][1]*c1 + T[1][1]*s1)) - d6*(T[0][2]*c1 + T[1][2]*s1) +
					T[0][3]*c1 + T[1][3]*s1;
				double p13y = T[2][3] - d1 - d6*T[2][2] + d5*(T[2][1]*c6 + T[2][0]*s6);

				double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
				if (fabs(fabs(c3) - 1.0) < ZERO_THRESH)
					c3 = SIGN(c3);
				else if (fabs(c3) > 1.0) {
					// TODO NO SOLUTION
					continue;
				}
				double arccos = acos(c3);
				q3[0] = arccos;
				q3[1] = 2.0 * M_PI - arccos;
				double denom = a2*a2 + a3*a3 + 2 * a2*a3*c3;
				double s3 = sin(arccos);
				double A = (a2 + a3*c3), B = a3*s3;
				q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
				q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
				double c23_0 = cos(q2[0] + q3[0]);
				double s23_0 = sin(q2[0] + q3[0]);
				double c23_1 = cos(q2[1] + q3[1]);
				double s23_1 = sin(q2[1] + q3[1]);
				q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
				q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
				////////////////////////////////////////////////////////////////////////////////
				for (int k = 0; k<2; k++) {
					if (fabs(q2[k]) < ZERO_THRESH)
						q2[k] = 0.0;
					else if (q2[k] < 0.0) q2[k] += 2.0 * M_PI;
					if (fabs(q4[k]) < ZERO_THRESH)
						q4[k] = 0.0;
					else if (q4[k] < 0.0) q4[k] += 2.0 * M_PI;
					q_sols.push_back(q1[i]); 
					q_sols.push_back(q2[k]);
					q_sols.push_back(q3[k]);
					q_sols.push_back(q4[k]);
					q_sols.push_back(q5[i][j]);
					q_sols.push_back(q6);
				}

			}
		}
	}
	return q_sols;
}

static std::vector<double> findNearestSolution(std::vector<double> &solutions, double currentPose[6])
{
	//一共6个轴发生了移动。对于每个轴，都有最多六个方案，选择其中移动最少的。
	//计算哪个方案赢得了最多轴的“最少”
	
		std::vector<std::vector<double> > diffs;
		diffs.resize(solutions.size()/6);
		
		int nearestSolution[6];
		int minDistances[6];

		for (int i = 0; i < solutions.size()/6; i++){
			diffs[i].resize(6);
			for (int j = 0; j < 6; j++){
				diffs[i][j] = abs(solutions[i*6+j] - currentPose[j]);
			}
		}

		for (int i = 0; i < diffs.size(); i++){
			minDistances[i] = DBL_MAX;
			for (int j = 0; j < 6; j++){
				if (diffs[i][j] < minDistances[i]){
					nearestSolution[j] = i;
					minDistances[i] = diffs[i][j];
					//distance = minDistances[i];
				}
			}
		}

		std::vector<int> count;
		count.resize(solutions.size(),0);
		for (int i = 0; i < solutions.size(); i++){
			count[nearestSolution[i]]++;
		}

		int nearest = INT_MIN;
		int max = INT_MIN;
		for (int i = 0; i < count.size(); i++){
			if (count[i] > max){
				nearest = i;
				max = count[i];
			}
		}
		
		//QDebug() << "nearest " << nearest;
		std::vector<double> retSolution;
		retSolution.resize(6);
		for (int i = 0; i < 6; i++)
		{
			retSolution[0] = solutions[6 * nearest + i];
		}
		
		return retSolution;
}
