#pragma once

#include <QWidget>
#include "ui_RobotControllerDialog.h"

class QCloseEvent;
class RobotControllerDialog : public QWidget
{
	Q_OBJECT
public:
	static void Show(QWidget *parent);
	~RobotControllerDialog();

public slots:
	void stop();
	void freeDrive();
	void reconnect();
	void test();
	void saveFrame();
	void loadFrame();

	void poweron();
	void releaseBreak();

	void rawcommand_send();

protected:
	void timerEvent(QTimerEvent *event) override;
	void closeEvent(QCloseEvent *event) override;
	
private:
	Ui::RobotControllerDialog ui;
	RobotControllerDialog(QWidget *parent = 0);
	static RobotControllerDialog *pInst;
};
