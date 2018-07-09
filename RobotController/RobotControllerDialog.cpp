#pragma execution_character_set("utf-8")

#include <QFileDialog>
#include <QDebug>
#include "RobotController.h"
#include "RobotPath.h"
#include "UrRealtimeCommunication.h"
#include "AudioPlayer.h"

#include "RobotControllerDialog.h"

RobotControllerDialog* RobotControllerDialog::pInst = nullptr;

void RobotControllerDialog::Show(QWidget *parent)
{
	if (pInst == nullptr)
	{
		pInst = new RobotControllerDialog(parent);	//Qt会自动析构这个
	}
	pInst->show();
}


RobotControllerDialog::RobotControllerDialog(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	setWindowFlags(Qt::Dialog);

	QPalette pal = ui.stopButton->palette();
	pal.setColor(QPalette::ButtonText, QColor(250, 10, 10));
	ui.stopButton->setPalette(pal);

	startTimer(2000);
}

RobotControllerDialog::~RobotControllerDialog()
{

}

void RobotControllerDialog::closeEvent(QCloseEvent *event)
{
	RobotController::GetInstance()->Stop();
	event->accept();
}

void RobotControllerDialog::freeDrive()
{
	RobotController::GetInstance()->FreeDriveMode();
}

void RobotControllerDialog::reconnect()
{
	RobotController::GetInstance()->Reconnect();
}
void RobotControllerDialog::test()
{
	//AudioPlayer::GetInstance()->play1();
	//RobotFrame frm = RobotController::GetRobotFrame();
	//qDebug() <<"Actual: "<< QString::fromStdString(frm.jointsToString()) << QString::fromStdString(frm.tcpToString());
	//frm.forwardKinematics();
	//qDebug() << "From forward:" << QString::fromStdString(frm.tcpToString());
}


void RobotControllerDialog::saveFrame()
{
	RobotFrame frm = RobotController::GetRobotFrame();
//	qDebug() << "Save: " << QString::fromStdString(frm.jointsToString()) << QString::fromStdString(frm.tcpToString());
	QString path = QFileDialog::getSaveFileName(this, "保存文件", QString(), "Text files (*.txt)");
	frm.save(path.toStdString());
}

void RobotControllerDialog::loadFrame()
{
	RobotFrame frm;
	QString path = QFileDialog::getOpenFileName(this, "打开文件", QString(), "Text files (*.txt)");
	frm.load(path.toStdString());
	frm.print();	
}

void RobotControllerDialog::stop()
{
	//RobotController::GetInstance()->FreeDriveModeEnd();
	RobotController::GetInstance()->Stop();
}

void RobotControllerDialog::poweron()
{
	RobotController::GetInstance()->PowerOn();
}


void RobotControllerDialog::releaseBreak()
{
	RobotController::GetInstance()->BreakRelease();
}


void RobotControllerDialog::rawcommand_send()
{
	RobotController::GetInstance()->RawCommand(ui.commandEdit->toPlainText());
	RobotController::GetInstance()->DashboardCommand(ui.dashboardEdit->toPlainText());
}

void RobotControllerDialog::timerEvent(QTimerEvent *event)
{
	ui.fpsLabel->setText(QString("fps: ") + QString::number(RobotController::GetInstance()->GetNetworkRate()));
	ui.IPEdit->setText(RobotController::GetInstance()->GetDestIp());

	std::string stateStr = RobotController::GetRobotState()->getStatesStringWtihLabel();
	QString statesStrLab = QString::fromStdString(stateStr);
	ui.statesDisp->setPlainText(statesStrLab);
	QString statesStr = QString::fromStdString(RobotController::GetRobotState()->getStatesString());
	ui.statesDispSingle->setPlainText(statesStr);
}