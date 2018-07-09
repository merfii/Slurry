#pragma execution_character_set("utf-8")
#include <QMessageBox>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD> 
#include <visp3/core/vpColVector.h>
#include "RobotController.h"
#include "RobotFrame.h"

#include "ToolCalibrationDialog.h"

using std::cout;
using std::endl;

#define RES_THRESH 5e-4
ToolCalibrationDialog::ToolCalibrationDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	m_ok = false;
	for (int i = 0; i < 5; i++)
		frm[i] = new RobotFrame;
}

ToolCalibrationDialog::~ToolCalibrationDialog()
{
	for (int i = 0; i < 5; i++)
		delete frm[i];
}

void ToolCalibrationDialog::captureFrm0()
{
	if (RobotController::GetInstance()->GetNetworkRate() > 0)
	{
		*frm[0] = RobotController::GetRobotFrame();
		ui.label0->setText(frm[0]->tcpToString().c_str());
	}
}

void ToolCalibrationDialog::captureFrm1()
{
	if (RobotController::GetInstance()->GetNetworkRate() > 0)
	{
		*frm[1] = RobotController::GetRobotFrame();
		ui.label1->setText(frm[1]->tcpToString().c_str());
	}
}

void ToolCalibrationDialog::captureFrm2()
{
	if (RobotController::GetInstance()->GetNetworkRate() > 0)
	{
		*frm[2] = RobotController::GetRobotFrame();
		ui.label2->setText(frm[2]->tcpToString().c_str());
	}
}

void ToolCalibrationDialog::captureFrm3()
{
	if (RobotController::GetInstance()->GetNetworkRate() > 0)
	{
		*frm[3] = RobotController::GetRobotFrame();
		ui.label3->setText(frm[3]->tcpToString().c_str());
	}
}

void ToolCalibrationDialog::captureFrm4()
{
	if (RobotController::GetInstance()->GetNetworkRate() > 0)
	{
		*frm[4] = RobotController::GetRobotFrame();
		ui.label4->setText(frm[4]->tcpToString().c_str());
	}
}


#include <sstream>
//这个函数包含大量的矩阵元素操作 你可能需要参阅我文档中的公式推导才能看懂
void ToolCalibrationDialog::solvePoint()
{
	vpHomogeneousMatrix Mri[4];
	for (int i = 0; i < 3; i++)
	{
		if (!frm[i]->isValid())
		{
			QMessageBox::warning(this, "错误", "输入数据不足, 无法标定");
			return;
		}

		Mri[i].buildFrom(
			frm[i]->P[0],
			frm[i]->P[1],
			frm[i]->P[2],
			frm[i]->P[3],
			frm[i]->P[4],
			frm[i]->P[5]);	//unit: m m m rad rad rad
	}
	//构造矩阵 求解三个点的方程 
	Eigen::MatrixXd A(9, 6);
	for (int i = 0; i < 3; i++)
	{
		vpHomogeneousMatrix &Mr = Mri[i];
		A(i * 3 + 0, 0) = Mr[0][0];	A(i * 3 + 0, 1) = Mr[0][1];	A(i * 3 + 0, 2) = Mr[0][2];
		A(i * 3 + 0, 3) = -1;		A(i * 3 + 0, 4) = 0;		A(i * 3 + 0, 5) = 0;

		A(i * 3 + 1, 0) = Mr[1][0];	A(i * 3 + 1, 1) = Mr[1][1];	A(i * 3 + 1, 2) = Mr[1][2];
		A(i * 3 + 1, 3) = 0;		A(i * 3 + 1, 4) = -1;		A(i * 3 + 1, 5) = 0;


		A(i * 3 + 2, 0) = Mr[2][0];	A(i * 3 + 2, 1) = Mr[2][1];	A(i * 3 + 2, 2) = Mr[2][2];
		A(i * 3 + 2, 3) = 0;		A(i * 3 + 2, 4) = 0;		A(i * 3 + 2, 5) = -1;
	}
	cout << "A:\n" << A << endl;
	
	Eigen::VectorXd b(9);
	for (int i = 0; i < 3; i++)
	{
		vpHomogeneousMatrix &Mr = Mri[i];
		b(i * 3 + 0) = -Mr[0][3];
		b(i * 3 + 1) = -Mr[1][3];
		b(i * 3 + 2) = -Mr[2][3];
	}
	cout << "b:\n" << b << endl;

	Eigen::VectorXd X = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
	cout << "X is:\n"<< X << endl;

	Mxyz.resize(4);
	x0.resize(4);
	Mxyz[0] = X(0); Mxyz[1] = X(1); Mxyz[2] = X(2); Mxyz[3] = 1;
	x0[0] = X(3); x0[1] = X(4); x0[2] = X(5); x0[3] = 1;
	if (frm[3]->isValid())
	{
		Mri[3].buildFrom(
			frm[3]->P[0],frm[3]->P[1],frm[3]->P[2],
			frm[3]->P[3],frm[3]->P[4],frm[3]->P[5]);	//unit: m m m rad rad rad
		//验证第四个点
		vpColVector res(4);
		
		res = Mri[3] * Mxyz -x0;
		m_res = res.sumSquare();
		if (m_res > RES_THRESH)
		{
			ui.textEdit->setPlainText(QString("验证误差: %1 \n 误差过大 标定失败").arg(m_res));
		}
	}
}


void ToolCalibrationDialog::solveMtool()
{
	if (Mxyz.size() == 0){
		return;
	}
	vpHomogeneousMatrix Mri;
	if (frm[4]->isValid())
	{
		Mri.buildFrom(
			frm[4]->P[0], frm[4]->P[1], frm[4]->P[2],
			frm[4]->P[3], frm[4]->P[4], frm[4]->P[5]);	//unit: m m m rad rad rad
		Eigen::Vector3d R1, R2, R3, T;
		vpColVector Zdir(4);
		//使用第5个点确定z方向 这里隐含了伸长为z正方向
		Zdir = Mri.inverse()*x0 - Mxyz;
		if (Zdir[2] < 0){
			Zdir = -Zdir;
		}
		R3 << Zdir[0], Zdir[1], Zdir[2];
		R3.normalize();

		R2 << 0, 1, 0;
		R1 = R2.cross(R3);

		T << Mxyz[0], Mxyz[1], Mxyz[2];

		//保存变换矩阵
		Mtool[0][0] = R1[0]; Mtool[1][0] = R1[1]; Mtool[2][0] = R1[2]; Mtool[3][0] = 0;
		Mtool[0][1] = R2[0]; Mtool[1][1] = R2[1]; Mtool[2][1] = R2[2]; Mtool[3][1] = 0;
		Mtool[0][2] = R3[0]; Mtool[1][2] = R3[1]; Mtool[2][2] = R3[2]; Mtool[3][2] = 0;
		Mtool[0][3] = T[0];  Mtool[1][3] = T[1];  Mtool[2][3] = T[2];  Mtool[3][3] = 1;

		std::ostringstream strm;
		strm << Mtool;
		ui.textEdit->setPlainText(QString("验证误差: %1 \n 标定成功\n").arg(m_res) +
			QString(strm.str().c_str()));
		m_ok = true;
	}
}

#include "SystemParameter.h"


void ToolCalibrationDialog::save()
{
	std::ofstream ofile(TOOL_PARAM_FILE, std::ios::out | std::ios::trunc);
	Mtool.save(ofile);
	ofile.close();
	SystemParameter::GetInstance()->Reload();
}

void ToolCalibrationDialog::moveForwardTest()
{
	if (!m_ok){
		std::vector<double> m = SystemParameter::GetInstance()->GetToolMatrix();
		Mtool.buildFrom(m);
	}
	
	RobotController::GetInstance()->Stop();
	RobotFrame frm = RobotController::GetRobotFrame();
	if (!frm.isValid())
	{
		return;
	}
	
	vpColVector dz(4);
	dz[0] = 0;
	dz[1] = 0;
	dz[2] = 0.02;	
	dz[3] = 0;

	vpHomogeneousMatrix Mrobot;
	Mrobot.buildFrom(frm.P[0], frm.P[1], frm.P[2], frm.P[3], frm.P[4], frm.P[5]);

	dz = Mrobot * Mtool * dz;

	cout << dz << endl;
	frm.P[0] += dz[0];
	frm.P[1] += dz[1];
	frm.P[2] += dz[2];

	RobotController::GetInstance()->MoveLine(frm,0.02);
}

void ToolCalibrationDialog::moveBackwardTest()
{
	if (!m_ok){
		//QMessageBox::warning(this, "错误", "尚未标定, 无法测试");
		std::vector<double> m = SystemParameter::GetInstance()->GetToolMatrix();
		Mtool.buildFrom(m);
	}

	//if(RobotController::GetInstance()->GetNetworkRate() > 0)
	RobotController::GetInstance()->Stop();
	RobotFrame frm = RobotController::GetRobotFrame();
	if (!frm.isValid())
	{
		return;
	}

	vpColVector dz(4);
	dz[0] = 0;
	dz[1] = 0;
	dz[2] = -0.02;	//向后退
	dz[3] = 0;

	vpHomogeneousMatrix Mrobot;
	Mrobot.buildFrom(frm.P[0], frm.P[1], frm.P[2], frm.P[3], frm.P[4], frm.P[5]);

	dz = Mrobot * Mtool * dz;

	cout << dz << endl;
	frm.P[0] += dz[0];
	frm.P[1] += dz[1];
	frm.P[2] += dz[2];

	RobotController::GetInstance()->MoveLine(frm, 0.04);
}
