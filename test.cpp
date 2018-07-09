#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <QFileDialog>

#include "RobotController.h"
#include "ScanVisualizer.h"
#include "GlobalShared.h"
#include "Logger.h"
#include "SystemParameter.h"
#include "CameraController.h"

#include "ToolCalibrationDialog.h"
#include "Slurry.h"

#include <visp3/core/vpHomogeneousMatrix.h>

//调试函数 用于将当前相机位置保存到文件
void Slurry::saveCameraPose()
{
	QString path = QFileDialog::getSaveFileName(this, "保存文件", QString(), "Text files (*.txt)");
	if (path.isEmpty())
		return;

	vpHomogeneousMatrix tcpMeyeA, tcpMeyeB, wA, wB;
	tcpMeyeA.buildFrom(SystemParameter::GetCameraColorAParam()->hand2eye);
	tcpMeyeB.buildFrom(SystemParameter::GetCameraColorBParam()->hand2eye);

	vpHomogeneousMatrix wMt;
	RobotFrame frm = RobotController::GetInstance()->GetRobotFrame();
	wMt.buildFrom(frm.P[0], frm.P[1], frm.P[2], frm.P[3], frm.P[4], frm.P[5]);
	wA = wMt * tcpMeyeA;
	wB = wMt * tcpMeyeB;
	std::ofstream ff;
	ff.open(path.toLocal8Bit().toStdString());
	wMt.save(ff);
	ff << endl << endl;
	wA.save(ff);
	ff << endl << endl;
	wB.save(ff);
	ff.close();
}


void Slurry::test1()
{
	m_ctMatched = true;

}

void Slurry::test2()
{
	ToolCalibrationDialog *dialog = new ToolCalibrationDialog;
	//dialog->show();
	dialog->exec();
}