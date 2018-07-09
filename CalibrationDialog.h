                                                  #pragma once

#include <memory>
#include <QDialog>
#include "FrmProcessorDisp.h"
#include "FrmProcessorCameraCalibration.h"
#include "FrmProcessorLaserCalibration.h"
#include "RobotPath.h"
#include "ui_CalibrationDialog.h"

#define DETECTION_TIMEOUT 1000 //ms
#define PATH_A_FILE "SystemParameters/PathA.txt"
#define PATH_B_FILE "SystemParameters/PathB.txt"
#define PATH_LASER_FILE "SystemParameters/PathLaser.txt"
#define PATH_NEW_FILE "SystemParameters/PathNew.txt"

class CameraParameter;

class CalibrationDialog : public QDialog
{
	Q_OBJECT
public:
	static void Show(QWidget *parent);
	CalibrationDialog(QWidget *parent = 0);
	~CalibrationDialog();

public slots:
	void showEvent(QShowEvent * event) override;
	void closeEvent(QCloseEvent *event) override;
	
	void AutoCalibrateStart();
	void ManualCalibrateStart();
	void ManualCapture();
	void ManualSolve();

	void AutoLaserStart();
	void ManualLaserStart();
	void ManualLaserCapture();
	void ManualLaserSolve();

protected:
	void keyPressEvent(QKeyEvent * event) override;

private:
	Ui::CalibrationDialog ui;
	std::shared_ptr<FrmProcessorCameraCalibration> processorCamera;
	std::shared_ptr<FrmProcessorLaserCalibration> processorLaser;
	std::shared_ptr<FrmProcessorDisp> processorDisp;
	static CalibrationDialog* pInst;

	static int count;
	RobotPath path;
	static bool reentrant;
};

