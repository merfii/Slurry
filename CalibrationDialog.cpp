#pragma execution_character_set("utf-8")

#include <opencv2/core.hpp>
#include <QMessageBox>
#include "GlobalShared.h"
#include "Slurry.h"
#include "Logger.h"
#include "FrmProcessorDisp.h"
#include "AudioPlayer.h"
#include "CameraController.h"
#include "RobotController.h"
#include "RobotException.h"
#include "LaserProjectorAndSensor.h"
#include "SystemParameter.h"

#include "CalibrationDialog.h"

using namespace std;
using namespace cv;

bool CalibrationDialog::reentrant;
CalibrationDialog* CalibrationDialog::pInst = nullptr;

void CalibrationDialog::Show(QWidget *parent)
{
	if (pInst == nullptr)
	{
		pInst = new CalibrationDialog(parent);
		//setWindowFlags(Qt::WindowStaysOnTopHint);会导致窗口隐藏 不明白怎么回事
	}
	pInst->show();
}

CalibrationDialog::CalibrationDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	//setModal(true);
	//setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);	//用于QWidget
	//setWindowModality(Qt::WindowModal);

	processorCamera = make_shared<FrmProcessorCameraCalibration>();
	connect(processorCamera.get(), SIGNAL(DispInDialogBig(cv::Mat)), ui.viewer1, SLOT(setImage(const cv::Mat)), Qt::DirectConnection);
	connect(processorCamera.get(), SIGNAL(DispInDialogSmall(cv::Mat)), ui.viewer2, SLOT(setImage(const cv::Mat)), Qt::DirectConnection);

	processorLaser = make_shared<FrmProcessorLaserCalibration>();
	connect(processorLaser.get(), SIGNAL(DispInDialogBig(cv::Mat)), ui.viewer1, SLOT(setImage(const cv::Mat)), Qt::DirectConnection);
	connect(processorLaser.get(), SIGNAL(DispInDialogSmall(cv::Mat)), ui.viewer2, SLOT(setImage(const cv::Mat)), Qt::DirectConnection);

	processorDisp = make_shared<FrmProcessorDisp>();
	reentrant = false;
}

CalibrationDialog::~CalibrationDialog()
{
	reentrant = false;
}

void CalibrationDialog::showEvent(QShowEvent * event)
{
	CameraController::GetInstance()->SetProcessor(processorCamera);
	processorCamera->Preview();
}

void CalibrationDialog::closeEvent(QCloseEvent *event)
{
	RobotController::GetInstance()->Stop();
	CameraController::GetInstance()->SetProcessor(processorDisp);
}

//相机参数标定
void CalibrationDialog::AutoCalibrateStart()
{
	RobotController *robot = RobotController::GetInstance();
	AudioPlayer *audio = AudioPlayer::GetInstance();

	if (ui.button1Auto->isChecked())
	{
		ui.button1Auto->setText("中止");
		ui.button1Start->setEnabled(false);
		ui.button1Capt->setEnabled(false);
		ui.button1Solve->setEnabled(false);

		if (ui.radioButton1->isChecked())
		{
			if (!path.load(PATH_A_FILE))
			{
				Logger::Debug(QString("找不到路径文件") + PATH_A_FILE);
				return;
			}
		}
		else
		{
			if (!path.load(PATH_B_FILE))
			{
				Logger::Debug(QString("找不到路径文件") + PATH_B_FILE);
				return;
			}
		}
		CameraController::GetInstance()->SetProcessor(processorCamera);
		try{
			audio->playCalibrationWarning();
			robot->yeild(500);
			robot->MoveP2P(path[0], 0.05, 0);
			if (ui.radioButton1->isChecked())
			{
				processorCamera->StartCalibration(CAMERA_COLOR_A);
			}
			else
			{
				processorCamera->StartCalibration(CAMERA_COLOR_B);
			}
			robot->yeild(2000);
			robot->MoveP2P(path[0], 0.3,0);
			robot->waitMoveStop(path[0], 10 * 1000);
			robot->yeild(500);
			audio->playCameraCapture();
			processorCamera->CapturePoints();
			ui.label->setText(QString("已捕获 %1 帧").arg(processorCamera->framesCaptured()));

			for (int i = 1; i < path.size(); i++)
			{
				robot->MoveP2P(path[i], 0.4, 0);
				robot->waitMoveStop(path[i], 10 * 1000);
				robot->yeild(500);
				audio->playCameraCapture();
				processorCamera->CapturePoints();
				ui.label->setText(QString("已捕获 %1 帧").arg(processorCamera->framesCaptured()));
			}
		}
		catch (RobotMoveException &e){
			robot->waitMoveCancel();
			ui.button1Auto->setText("自动标定");
			ui.button1Start->setEnabled(true);
			ui.button1Capt->setEnabled(false);
			ui.button1Solve->setEnabled(false); 
			QMessageBox::warning(this, "超时","\n移动超时！任务已终止       \n",QMessageBox::Abort);
			return;
		}

		CameraParameter param;
		if (processorCamera->Solve(param))
		{
			//解算成功 显示结果	
			QString text = QString("标定结果：\n") + QString::fromStdString(param.toString()) + QStringLiteral("\n您要保存吗？");
			//您要保存吗？
			if (QMessageBox::Yes == QMessageBox::question(this, "相机参数标定", text))
			{
				if (ui.radioButton1->isChecked())
				{
					param.save(CAMERA_COLOR_A_PARAM_FILE);
				}
				else
				{
					param.save(CAMERA_COLOR_B_PARAM_FILE);
				}
				//重新装载参数
				SystemParameter::Reload();
			}
		}
		else
		{
			QMessageBox::information(this, "相机参数标定", "求解失败", QMessageBox::Ok);
		}
		ui.button1Auto->setText("自动标定");
		ui.button1Start->setEnabled(true);
		ui.button1Capt->setEnabled(false);
		ui.button1Solve->setEnabled(false);
	}
	else
	{
		robot->Stop();
		robot->waitMoveCancel();
		processorCamera->StopCalibration();
		ui.button1Auto->setText("自动标定");
		ui.button1Start->setEnabled(true);
		ui.button1Capt->setEnabled(false);
		ui.button1Solve->setEnabled(false);
	}
}

void CalibrationDialog::ManualCalibrateStart()
{
	if (ui.button1Start->text() == "手动标定")
	{
		ui.button1Start->setText("中止");
		ui.button1Auto->setEnabled(false);
		ui.button1Capt->setEnabled(true);
		ui.button1Solve->setEnabled(false);
		
		path.clear();
		CameraController::GetInstance()->SetProcessor(processorCamera);
		if (ui.radioButton1->isChecked())
		{
			processorCamera->StartCalibration(CAMERA_COLOR_A);
		}
		else
		{
			processorCamera->StartCalibration(CAMERA_COLOR_B);
		}
		AudioPlayer::GetInstance()->playCalibrationWarning();
	}else
	{
		RobotController::GetInstance()->Stop();
		processorCamera->StopCalibration();
		ui.button1Start->setText("手动标定");
		ui.button1Solve->setText("解算参数");
		ui.button1Auto->setEnabled(true);
		ui.button1Capt->setEnabled(false);
		ui.button1Solve->setEnabled(false);
	}
}


void CalibrationDialog::ManualCapture()
{
	path.push_back(RobotController::GetRobotFrame());
	AudioPlayer::GetInstance()->playCameraCapture();
	processorCamera->CapturePoints();
	ui.label->setText(QString("已捕获 %1 帧").arg(processorCamera->framesCaptured()));
	ui.button1Solve->setEnabled(true);
}

void CalibrationDialog::ManualSolve()
{
	//path.save(PATH_NEW_FILE);

	CameraParameter param;
	ui.button1Capt->setEnabled(false);
	if (processorCamera->Solve(param))
	{
		//解算成功 显示结果	
		QString text = QString("标定结果：\n") + QString::fromStdString(param.toString()) + QStringLiteral("\n您要保存吗？");
		//您要保存吗？
		if (QMessageBox::Yes == QMessageBox::question(this, "相机参数标定", text))
		{
			if (ui.radioButton1->isChecked())
			{
				param.save(CAMERA_COLOR_A_PARAM_FILE);
			}
			else
			{
				param.save(CAMERA_COLOR_B_PARAM_FILE);
			}
			//重新装载参数
			SystemParameter::Reload();
		}
	}
	else
	{
		QMessageBox::information(this, "相机参数标定", "求解失败", QMessageBox::Ok);
	}
}

//激光平面标定
 
void CalibrationDialog::ManualLaserStart()
{
	if (ui.button2Start->text() == "手动标定")
	{
		ui.button2Start->setText("中止");
		ui.button2Auto->setEnabled(false);
		ui.button2Capt->setEnabled(true);
		ui.button2Solve->setEnabled(false);

		CameraController::GetInstance()->SetProcessor(processorLaser);
		processorLaser->StartCalibration();

//		AudioPlayer::GetInstance()->playCalibrationWarning();
	}
	else
	{
		//RobotController::GetInstance()->Stop();
		processorLaser->StopCalibration();
		ui.button2Start->setText("手动标定");
		ui.button2Solve->setText("解算参数");
		ui.button2Auto->setEnabled(true);
		ui.button2Capt->setEnabled(false);
		ui.button2Solve->setEnabled(false);
	}
}

void CalibrationDialog::AutoLaserStart()
{
	RobotController *robot = RobotController::GetInstance();
	AudioPlayer *audio = AudioPlayer::GetInstance();

	if (ui.button2Auto->text() == "自动标定")
	{
		ui.button2Auto->setText("中止");
		ui.button2Start->setEnabled(false);
		ui.button2Capt->setEnabled(false);
		ui.button2Solve->setEnabled(false);

		CameraController::GetInstance()->SetProcessor(processorLaser);
		processorLaser->StartCalibration();

		RobotPath pathLaser;
		if (!pathLaser.load(PATH_LASER_FILE))
		{
			Logger::Debug(QString("找不到路径文件") + PATH_LASER_FILE);
			return;
		}

		try{
			audio->playCalibrationWarning();
			robot->yeild(500);
			robot->MoveP2P(pathLaser[0], 0.05, 0);
			robot->yeild(2000);
			robot->MoveP2P(pathLaser[0], 0.2);
			robot->waitMoveStop(pathLaser[0], 5 * 1000);
			robot->yeild(2000);
			audio->playCameraCapture();
			processorLaser->CapturePoints();
			ui.label->setText(QString("已捕获 %1 帧").arg(processorLaser->framesCaptured()));

			for (int i = 1; i < pathLaser.size(); i++)
			{
				robot->MoveP2P(pathLaser[i], 0.2);
				robot->waitMoveStop(pathLaser[i], 10 * 1000);
				robot->yeild(3500);
				audio->playCameraCapture();
				processorLaser->CapturePoints();
				ui.label->setText(QString("已捕获 %1 帧").arg(processorLaser->framesCaptured()));
			}
		}
		catch (RobotMoveException &e){
			robot->waitMoveCancel();
			ui.button2Auto->setText("自动标定");
			ui.button2Start->setEnabled(true);
			ui.button2Capt->setEnabled(false);
			ui.button2Solve->setEnabled(false);
			QMessageBox::warning(this, "超时", "\n移动超时！任务已终止       \n", QMessageBox::Abort);
			return;
		}

		if(processorLaser->SolveLaserPlane())
		{
			QMessageBox::information(this, "激光平面标定成功", "成功", QMessageBox::Ok);
		}
		else
		{
			QMessageBox::information(this, "激光平面标定失败", "失败", QMessageBox::Ok);
		}

		ui.button2Auto->setText("自动标定");
		ui.button2Start->setEnabled(true);
		ui.button2Capt->setEnabled(false);
		ui.button2Solve->setEnabled(false);
	}
	else
	{
		robot->Stop();
		robot->waitMoveCancel();
		processorCamera->StopCalibration();
		ui.button2Auto->setText("自动标定");
		ui.button2Start->setEnabled(true);
		ui.button2Capt->setEnabled(false);
		ui.button2Solve->setEnabled(false);
	}
}

void CalibrationDialog::ManualLaserCapture()
{
	if (processorLaser->CapturePoints())
	{
		AudioPlayer::GetInstance()->playCameraCapture();
		ui.label->setText(QString("已捕获 %1 帧").arg(processorLaser->framesCaptured()));
		ui.button2Solve->setEnabled(true);
	}
}


void CalibrationDialog::ManualLaserSolve()
{
	processorLaser->StopCalibration();
	processorLaser->SolveLaserPlane();
}


void CalibrationDialog::keyPressEvent(QKeyEvent * event)
{

	switch (event->key())
	{
	case Qt::Key_D:
		processorCamera->debugUndistort();
		break;

	case Qt::Key_0:

		break;
	
	default:
		QDialog::keyPressEvent(event);
		break;
	}
}

static bool loadStrings( const string& filename, vector<string>& l )
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
}

