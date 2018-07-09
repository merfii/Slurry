#pragma once

#include <QDialog>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpColVector.h>
#include "ui_ToolCalibrationDialog.h"

class RobotFrame;
class ToolCalibrationDialog : public QDialog
{
	Q_OBJECT

public:
	ToolCalibrationDialog(QWidget *parent = Q_NULLPTR);
	~ToolCalibrationDialog();
	inline bool isSuccess() const
	{
		return m_ok;
	}

	inline vpHomogeneousMatrix getResult() const
	{
		return Mtool;
	}

public slots:
	void captureFrm0();
	void captureFrm1();
	void captureFrm2();
	void captureFrm3();
	void captureFrm4();
	void solvePoint();
	void solveMtool();
	void save();

	void moveForwardTest();
	void moveBackwardTest();


private:
	RobotFrame *frm[5];
	vpColVector Mxyz, x0;
	vpHomogeneousMatrix Mtool;
	Ui::ToolCalibrationDialog ui;
	double m_res;
	bool m_ok;
};
 