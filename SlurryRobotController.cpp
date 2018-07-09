#pragma execution_character_set("utf-8")
#include "GlobalShared.h"
#include "Logger.h"
#include "SystemParameter.h"
#include "ScanVisualizer.h"
#include "CameraController.h"
#include "RobotController.h"
#include "FrmProcessorDisp.h"
#include "FrmProcessorScan.h"
#include "RobotPath.h"
#include "PathRecorder.h"
#include "AudioPlayer.h"
#include "LaserProjectorAndSensor.h"

#include "Slurry.h"

void Slurry::scanTeach()
{
	if (ui.buttonTeach->isChecked())
	{
		LaserProjectorAndSensor::GetInstance()->LaserOn();
		ui.buttonTeachPlay->setEnabled(false);
		AudioPlayer::GetInstance()->playAlarm();
		AudioPlayer::GetInstance()->playSlowAndSteady(true);
		//RobotController::GetInstance()->FreeDriveMode();
		recorder->record();
		ui.buttonTeach->setText("结束示教");
		ui.buttonTeachPlay->setChecked(false);

	}
	else
	{
		AudioPlayer::GetInstance()->playSlowAndSteady(false);
		recorder->stop();
		RobotController::GetInstance()->Stop();
		ui.buttonTeach->setText("开始示教");
		ui.buttonTeachPlay->setEnabled(true);
		ui.buttonTeachPlay->setChecked(false);
		LaserProjectorAndSensor::GetInstance()->LaserOff();
	}
}


void Slurry::scanTeachTest()
{
	if (ui.buttonTeachPlay->isChecked())
	{
//		qDebug() << "checked";
	}
	if (recorder && !recorder->isEmpty())
	{
		if (ui.buttonTeachPlay->isChecked())
		{
			LaserProjectorAndSensor::GetInstance()->LaserOn();
			recorder->playReversed();
			
		}
		else
		{
			recorder->play();
		}
	}
}

void Slurry::scanStart()
{
	CameraController *controller = CameraController::GetInstance();
	RobotController::GetInstance();
	if (ui.buttonScan->isChecked())
	{
		ui.buttonScan->setText("结束扫描");
		std::shared_ptr<FrmProcessorScan> processorScan = std::make_shared<FrmProcessorScan>();
		connect(processorScan.get(), SIGNAL(DispInMainWindowA(cv::Mat)), GlobalShared::slurry->ui.viewer1, SLOT(setImage(const cv::Mat)), Qt::DirectConnection);
		connect(processorScan.get(), SIGNAL(DispInMainWindowB(cv::Mat)), GlobalShared::slurry->ui.viewer2, SLOT(setImage(const cv::Mat)), Qt::DirectConnection);

		if (nullptr != controller)
		{
			controller->SetProcessor(processorScan);
			scanVisualizer->ScanTaskInit();
			processorScan->StartScan();
			AudioPlayer::GetInstance()->playScanWarning();
			AudioPlayer::GetInstance()->playCameraCapture();

			if (recorder == nullptr || recorder->isEmpty())
			{
				;
			}
			else
			{
				if (ui.buttonTeachPlay->isChecked())
				{
					recorder->playReversed(0.12);
				}
			}
		}
	}
	else
	{
		ui.buttonScan->setText("开始扫描");
		dynamic_cast<FrmProcessorScan*>(controller->GetProcessor())->StopScan();
		std::shared_ptr<FrmProcessorDisp> disp = std::make_shared<FrmProcessorDisp>();
		controller->SetProcessor(disp);
		scanVisualizer->ScanTaskEnd();
	}
}

void Slurry::scanClear()
{
	scanVisualizer->ScanTaskClear();
}


void Slurry::scanSaveData()
{
	scanVisualizer->ScanSaveData();
	/*
	保存debug点云数据
	laserPoints3D.release();
	cv::FileStorage fs("SystemParameters/laserPoints3D.yml", cv::FileStorage::READ);
	fs["Points"] >> laserPoints3D;
	fs.release();
	*/

}


void Slurry::scanLoadData()
{
	scanVisualizer->ScanLoadData(this);

	/*
	装载debug点云数据
	laserPoints3D.release();
	cv::FileStorage fs("SystemParameters/laserPoints3D.yml", cv::FileStorage::READ);
	fs["Points"] >> laserPoints3D;
	fs.release();
	*/
}

void Slurry::toolDispWidget(bool tooggled)
{
	if (tooggled)
	{
		ui.vtkWindow->SetLineWidgetDisp(true);
		ui.ctWindow->SetLineWidgetDisp(true);
	}
	else
	{
		ui.vtkWindow->SetLineWidgetDisp(false);
		ui.ctWindow->SetLineWidgetDisp(false);
	}
}

#include <QMessageBox>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include "RobotFrame.h"

void Slurry::toolIntoPosition()
{
	RobotController::GetInstance()->Stop();

	RobotFrame frm = RobotController::GetRobotFrame();
	if (!frm.isValid())
		return;

	if (!ui.buttonToolWidget->isChecked())
		return;

	vpHomogeneousMatrix Mr, Mtool, Mrtool;
	Mr.buildFrom(frm.P[0], frm.P[1], frm.P[2], frm.P[3],frm.P[4],frm.P[5]);	//unit: m m m rad rad rad
	Mtool.buildFrom(SystemParameter::GetInstance()->GetToolMatrix());
	Mrtool = Mr * Mtool;

	{
		//判断距离是否过远
		vpColVector dxdydz(3);
		dxdydz[0] = targetP1[0] - Mrtool[0][3];
		dxdydz[1] = targetP1[1] - Mrtool[1][3];
		dxdydz[2] = targetP1[2] - Mrtool[2][3];

		if (dxdydz.sumSquare() > 0.2*0.2)
		{
			QMessageBox::critical(this, "安全提示", "当导引筒距离目标小于20cm时方可使用该功能");
			return;
		}
	}
	//判断方向是否过偏
	vpColVector z_axis(3);	//目标z轴方向
	z_axis[0] = targetP1[0] - targetP2[0];
	z_axis[1] = targetP1[1] - targetP2[1];
	z_axis[2] = targetP1[2] - targetP2[2];
	z_axis.normalize();

	vpColVector toolZ_axis(3);	//当前工具z轴方向
	toolZ_axis[0] = Mrtool[0][2];
	toolZ_axis[1] = Mrtool[1][2];
	toolZ_axis[2] = Mrtool[2][2];

	//依据当前位置 自动掉转方向
	if (vpColVector::dotProd(z_axis, toolZ_axis) <0 )
	{
		z_axis = -z_axis;
	}

	/*if (vpColVector::dotProd(z_axis, toolZ_axis) < 0.7)
	{
		QMessageBox::critical(this, "安全提示", "导引筒朝向需与目标方向接近");
		return;
	}*/

	QMessageBox::information(this, "安全提示", "即将移动  请注意");

	vpColVector toolX_axis(3);
	toolX_axis[0] = Mrtool[0][0];
	toolX_axis[1] = Mrtool[1][0];
	toolX_axis[2] = Mrtool[2][0];

	vpColVector y_axis(3);
	y_axis = vpColVector::cross(z_axis, toolX_axis);
	y_axis.normalize();

	vpColVector x_axis(3);
	x_axis = vpColVector::cross(y_axis, z_axis);
	x_axis.normalize();

	for (int i = 0; i < 3; i++)
	{
		Mrtool[i][0] = x_axis[i];
		Mrtool[i][1] = y_axis[i];
		Mrtool[i][2] = z_axis[i];
	}

	Mrtool[0][3] = targetP1[0];	Mrtool[1][3] = targetP1[1];	Mrtool[2][3] = targetP1[2];
	Mrtool[3][0] = Mrtool[3][1] = Mrtool[3][2] = 0;
	Mrtool[3][3] = 1;
	
	//cout << "Mrtool_new" << Mrtool << endl << endl;

	vpThetaUVector thetaUV;
	vpTranslationVector T;
	Mr = Mrtool * Mtool.inverse();
	Mr.extract(thetaUV);
	Mr.extract(T);

	//cout << "Mr_new" << Mr << endl << endl;

	frm.P[0] = T[0]; frm.P[1] = T[1]; frm.P[2] = T[2];
	frm.P[3] = thetaUV[0]; frm.P[4] = thetaUV[1]; frm.P[5] = thetaUV[2];
	RobotController::GetInstance()->MoveP2P(frm,0.02,0);
}

void Slurry::toolMoveForward1cm()
{
	RobotController::GetInstance()->Stop();

	vpHomogeneousMatrix Mtool;
	Mtool.buildFrom(SystemParameter::GetInstance()->GetToolMatrix());

	RobotFrame frm = RobotController::GetRobotFrame();
	if (!frm.isValid())
	{
		return;
	}

	vpColVector dz(4);
	dz[0] = 0;
	dz[1] = 0;
	dz[2] = 0.01;
	dz[3] = 0;

	vpHomogeneousMatrix Mrobot;
	Mrobot.buildFrom(frm.P[0], frm.P[1], frm.P[2], frm.P[3], frm.P[4], frm.P[5]);

	dz = Mrobot * Mtool * dz;

	cout << dz << endl;
	frm.P[0] += dz[0];
	frm.P[1] += dz[1];
	frm.P[2] += dz[2];

	RobotController::GetInstance()->MoveLine(frm, 0.01);
}

void Slurry::toolMoveBackward1cm()
{
	RobotController::GetInstance()->Stop();

	vpHomogeneousMatrix Mtool;
	Mtool.buildFrom(SystemParameter::GetInstance()->GetToolMatrix());

	RobotFrame frm = RobotController::GetRobotFrame();
	if (!frm.isValid())
	{
		return;
	}

	vpColVector dz(4);
	dz[0] = 0;
	dz[1] = 0;
	dz[2] = -0.01;
	dz[3] = 0;

	vpHomogeneousMatrix Mrobot;
	Mrobot.buildFrom(frm.P[0], frm.P[1], frm.P[2], frm.P[3], frm.P[4], frm.P[5]);

	dz = Mrobot * Mtool * dz;

	cout << dz << endl;
	frm.P[0] += dz[0];
	frm.P[1] += dz[1];
	frm.P[2] += dz[2];

	RobotController::GetInstance()->MoveLine(frm, 0.01);
}

/*
开始扫描并回调存储数据+
各状态均能回到初始化并重新开始
******************************
状态：
#1 初始化 播放语音提示
#2 等待按钮 如果已按下则发送运动
#3 监视运动情况  如果出错则停止 松开则停止
#4 等待进入下一录制阶段
#5 等待按钮 按下即发送指令 并开始录制路径
#6 每个数据包录制路径 判断是否结束
#7 录制结束 若有按钮则沿原路径返回
#8 等待按下按钮 即自动播放指令 注意监测异常
在定时器startTimer(0)空闲事件中使用协程更方便编写逻辑 但需要引入第三方库 暂不采用
*/
