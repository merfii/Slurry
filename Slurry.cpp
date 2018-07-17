#pragma execution_character_set("utf-8")

#include <fstream>
#include <memory>

#include <QScreen>
#include <QGuiApplication>
#include <QMessageBox>
#include <QTimer>
#include <QMutex>

#include <opencv2/core.hpp>

#include "GlobalShared.h"
#include "Logger.h"
#include "SystemParameter.h"
#include "CameraController.h"
#include "RobotController.h"
#include "RobotControllerDialog.h"
#include "CalibrationDialog.h"
#include "LaserDialog.h"
#include "FrmProcessorDisp.h"
#include "Viewer2D.h"
#include "LaserProjectorAndSensor.h"
#include "AudioPlayer.h"
#include "PathRecorder.h"
#include "ScanVisualizer.h"
#include "ToolCalibrationDialog.h"
#include "TestPointsGenerator.h"

#include "Slurry.h"

Slurry::Slurry(QWidget *parent)
	: QMainWindow(parent), recorder(nullptr), m_ctMatched(false), m_ctData(nullptr)
{
	GlobalShared::slurry = this;

	recorder = new PathRecorder;
	scanVisualizer = new ScanVisualizer;
	pointCloudMutex = new QMutex;
	generator = nullptr;

	slurryInit();
	ctInit();
	scanVisualizer->SetVtkWindow(ui.vtkWindow);
	scanVisualizer->MainInit();
	//scanVisualizer->ScanTaskInit();
	scanClear();

	cameraInit();
	Logger::PrintLog(QStringLiteral("初始化完成"));
	//SystemParameter::GetInstance();	//make it construct
	//createPointsGenerator();
}

void Slurry::slurryInit()
{
	ui.setupUi(this);
	resize(QGuiApplication::primaryScreen()->availableSize() * 5 / 7);

	ui.logText->setCmdColor(Qt::darkBlue);
	ui.logText->setOutColor(Qt::black);
	ui.logText->setPrompt(QString("IN: "));
	ui.pilotRobot->setText(QStringLiteral("Robot"));
	ui.pilotMatch->setText(QStringLiteral("Match"));

	connect(ui.ctWindow, SIGNAL(LineWidgetMoved(double*, double*)), this, SLOT(lineWidgetTransform(double*, double*)));
	connect(this, SIGNAL(lineWidgetMove(double*, double*)), ui.vtkWindow, SLOT(SetLineWidgetPoints(double*, double*)));
	connect(ui.vtkWindow, SIGNAL(LineWidgetMoved(double*, double*)), this, SLOT(lineWidgetNoTransform(double*, double*)));

	QPalette pale;
	pale.setColor(QPalette::ButtonText, QColor(250, 10, 10));
	ui.buttonStop->setPalette(pale);

	m_regMatrix[0][0] = 1; m_regMatrix[0][1] = 0; m_regMatrix[0][2] = 0; m_regMatrix[0][3] = 0;
	m_regMatrix[1][0] = 0; m_regMatrix[1][1] = 1; m_regMatrix[1][2] = 0; m_regMatrix[1][3] = 0;
	m_regMatrix[2][0] = 0; m_regMatrix[2][1] = 0; m_regMatrix[2][2] = 1; m_regMatrix[2][3] = 0;
	
	startTimer(1000);	//for Robot State & Match State pilot display 
}

Slurry::~Slurry()                                                                                                                                                                                                                        
{
	delete recorder;
	delete scanVisualizer;
	delete pointCloudMutex;
}

void Slurry::closeEvent(QCloseEvent *event)
{
	if (LaserProjectorAndSensor::isConstructed())
	{
		LaserProjectorAndSensor::GetInstance()->MeterStop();
		LaserProjectorAndSensor::GetInstance()->FlickerStop();
		//LaserProjectorAndSensor::GetInstance()->LaserOff();
	}

	//可能还有图像被处理之后显示 而此时GUI控件已被销毁
	if (CameraController::isConstructed())
	{
		CameraController::NotifyExit();
		CameraController::GetInstance()->StopGrabbing();
		CameraController::Delete();
	}
	//GlobalShared::app->processEvents(QEventLoop::WaitForMoreEvents, 500);
	//QTest::qSleep(500);
	if (generator){
		generator->closeGenerator();
		delete generator;
	}
	QMainWindow::closeEvent(event);
}

void Slurry::lineWidgetTransform(double *P1, double *P2)
{
	for (int i = 0; i < 3; i++)
	{
		targetP1[i] = P1[0] * m_regMatrix[i][0] + P1[1] * m_regMatrix[i][1] + P1[2] * m_regMatrix[i][2] + m_regMatrix[i][3];
		targetP2[i] = P2[0] * m_regMatrix[i][0] + P2[1] * m_regMatrix[i][1] + P2[2] * m_regMatrix[i][2] + m_regMatrix[i][3];
	}
	emit lineWidgetMove(targetP1, targetP2);
}

void Slurry::lineWidgetNoTransform(double *P1, double *P2)
{
	for (int i = 0; i < 3; i++)
	{
		targetP1[i] = P1[i];
		targetP2[i] = P2[i];
	}
}

/******************* Slot functions *******************/

void Slurry::cameraInit()
{
	CameraController *camController = CameraController::GetInstance();

	int ret_n = camController->PollingDevices(true);

	if (0 == ret_n)
	{
		Logger::PrintLog(QString("未找到相机"));
		camController->CloseCameras();
		return;
	}
	else
	{
		Logger::PrintLog(QString("已找到相机 %1 个").arg(ret_n));
	}

	if (!camController->OpenCamerasAndLoadConfiguration())
	{
		Logger::PrintLog("打开相机失败~~");
		return;
	}

}

void Slurry::openCalibDialog()
{
	RobotController::GetInstance();
	GlobalShared::app->processEvents(QEventLoop::AllEvents);	//为了防止网络接收出错
	CalibrationDialog::Show(this);
}

void Slurry::openRobotControlDialog()
{
	RobotController::GetInstance();
	RobotControllerDialog::Show(this);
}


void Slurry::openLaserDialog()
{
	LaserDialog::Show(this);
}


void Slurry::openToolDialog()
{
	ToolCalibrationDialog dialog;
	if (dialog.exec() == QDialog::Accepted)
	{
		//保存结果
		vpHomogeneousMatrix Mtool = dialog.getResult();
		ofstream ss(TOOL_PARAM_FILE,std::ios::trunc);
		Mtool.save(ss);
		ss.flush();
		ss.close();
		SystemParameter::Reload();
	}
}

void Slurry::cameraDisplay()
{
	CameraController *controller = CameraController::GetInstance();
	if (nullptr != controller)
	{
		controller->SetModeContinue();
		//controller->SetModeSoftTrig();
		controller->SetProcessor(std::make_shared<FrmProcessorDisp>());
		controller->StartGrabbing();
	}
}

//
//void Slurry::cameraFrameShoot()
//{
//	CameraController *controller = CameraController::GetInstance();
//	if (nullptr != controller)
//	{
//		controller->SoftTrigAll();
//	}
//}

void Slurry::stop()
{
	RobotController::GetInstance()->Stop();
	recorder->stop();
	AudioPlayer::GetInstance()->playSlowAndSteady(false);
	ui.buttonTeach->setText("开始示教");
	ui.buttonTeachPlay->setEnabled(true);
	ui.buttonTeachPlay->setChecked(false);
}

void Slurry::about()
{
	statusBar()->showMessage("显示10秒状态栏....", 10 * 1000);
	QMessageBox::about(this, "关于Slurry",
		"<p>&nbsp;&nbsp;&nbsp;&nbsp;<b>Slurry工程</b>由<a href=\"http://www.sa.buaa.edu.cn/xysy.htm\">北京航空航天大学宇航学院</a>周付根教授资助，" \
		"作为本人攻读硕士学位研究课题。" \
		"题目《基于结构光的目标三维成像装置》</p>");
}
 
void Slurry::aboutQt()
{
	QApplication::aboutQt();
}

#include "LaserProjectorAndSensor.h"

void Slurry::timerEvent(QTimerEvent *event)
{
	if (RobotController::GetInstance()->GetNetworkRate() > 100)
	{
		ui.pilotRobot->setText(QStringLiteral("ROBOT\nOK"));
		ui.pilotRobot->setGreen();
		RobotFrame frm = RobotController::GetRobotFrame();
		scanVisualizer->SetRobot(frm);
		ui.vtkWindow->update();
	}
	else
	{
		ui.pilotRobot->setText(QStringLiteral("ROBOT\nNA"));
		ui.pilotRobot->setRedFlicker();
	}

	if (m_ctMatched)
	{
		ui.pilotMatch->setText(QStringLiteral("MATCHED"));
		ui.pilotMatch->setGreen();
	}
	else
	{
		ui.pilotMatch->setText(QStringLiteral("NOT\nMATCHED"));
		ui.pilotMatch->setYellow();
	}
	//static int ticktock;
	//if (ticktock %2 == 0){
	//	qDebug() << "Tick";
	//}
	//else{
	//	qDebug() << "Tock";
	//}
	//ticktock++;
}
