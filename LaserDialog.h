#ifndef LASERDIALOG_H
#define LASERDIALOG_H

#include <QDialog>
#include "ui_LaserDialog.h"

class LaserDialog : public QDialog
{
	Q_OBJECT

public:
	static void Show(QWidget *parent);
	LaserDialog(QWidget *parent = 0);
	~LaserDialog();

public slots:
	void LaserOn();
	void LaserOff();
	void FlickerRun();
	void FlickerStop();
	void MeterRun();
	void MeterStop();
	void MeterStatus();
	double GetDistance();	//mm


private:
	static LaserDialog *pInst;
	Ui::LaserDialog ui;
};

#endif // LASERDIALOG_H
