#include <memory>
#include "LaserProjectorAndSensor.h"
#include "CameraController.h"
#include "FrmProcessorDisp.h"
#include "Slurry.h"
#include "GlobalShared.h"

#include "LaserDialog.h"

LaserDialog* LaserDialog::pInst = nullptr;


void LaserDialog::Show(QWidget *parent)
{
	if (pInst == nullptr)
	{
		pInst = new LaserDialog(parent);
	}
	pInst->show();
}

LaserDialog::LaserDialog(QWidget *parent):
QDialog(parent)
{
	ui.setupUi(this);
	//setAttribute(Qt::WA_DeleteOnClose);
}

LaserDialog::~LaserDialog()
{

}

void LaserDialog::LaserOn()
{
	LaserProjectorAndSensor::GetInstance()->LaserOn();
}

void  LaserDialog::LaserOff()
{
	LaserProjectorAndSensor::GetInstance()->LaserOff();
}


void  LaserDialog::FlickerRun()
{
	std::shared_ptr<FrmProcessorDisp> processorDisp = std::make_shared<FrmProcessorDisp>();
	CameraController::GetInstance()->SetProcessor(processorDisp);
	CameraController::GetInstance()->SetModeExtTrig();
	CameraController::GetInstance()->StartGrabbing();
	LaserProjectorAndSensor::GetInstance()->FlickerRunSlow();
}


void  LaserDialog::FlickerStop()
{
	LaserProjectorAndSensor::GetInstance()->FlickerStop();

}

void  LaserDialog::MeterRun()
{
	LaserProjectorAndSensor::GetInstance()->Trigger2On();
	qDebug() << "On";
}


void  LaserDialog::MeterStop()
{
	LaserProjectorAndSensor::GetInstance()->Trigger2Off();
	qDebug() << "Off";
}


void  LaserDialog::MeterStatus()
{

}

//该硬件未安装
double  LaserDialog::GetDistance()	//mm
{
	//return LaserProjectorAndSensor::GetInstance()->GetDistance();
	return 0;
}
