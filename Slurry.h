#pragma once

#include <QtWidgets/QMainWindow>

#include "ui_Slurry.h"

/*
ViSP 解决手眼标定
Eigen解决矩阵运算 多项式插值 拟合 spline

位姿
自己定义 RobotPose3D
#include <visp3/core/vpPoseVector.h>	vpPoseVector
#include <mrpt/poses/CPose3D.h>			mrpt::poses::CPose3D
#include <mrpt/math/lightweight_geom_data.h>  mrpt::math::TPose3D

数学库使用<visp3/core/vpMath.h>
MRPT未使用 前向运动学 PnP测量 spline和拟合(部分)
*/

class RobotControllerDialog;
class SystemParameter;
class LaserDialog;
class PathRecorder;
class RobotPath;
class ScanVisualizer;
using cv::Mat;

class Slurry : public QMainWindow
{
	Q_OBJECT
public:
	friend class CTImageVisualization;
	Slurry(QWidget *parent = 0);
	~Slurry();
	
	void initCalibSelect();
	Ui::SlurryClass ui;

public slots:
	void closeEvent(QCloseEvent *event) override;
	void about();
	void aboutQt();

	void cameraInit();
	void cameraDisplay();
	void openCalibDialog();
	void openRobotControlDialog();
	void openToolDialog();
	void openLaserDialog();

	//调试函数 用于将当前相机位置保存到文件
	void saveCameraPose();
	void test1();
	void test2();

private:
	void slurryInit();
	void createPointsGenerator();

//Scan &tool slots
public slots:
	void stop();
	void scanTeach();
	void scanTeachTest();
	void scanStart();
	void scanClear();
	void scanSaveData();
	void scanLoadData();
	void toolDispWidget(bool tooggled);
	void toolIntoPosition();
	void toolMoveForward1cm();
	void toolMoveBackward1cm();

private:
	PathRecorder *recorder;
	ScanVisualizer *scanVisualizer;
	friend class FrmProcessorScan;
	
//CT Image Visualization.cpp
public slots:
	void ctInit();
	void ctDisplay();
	void ctLoadDICOM();
	void ctLoadImage3D();
	void ctSaveImage3D();
	void ctReconstruct();
	void ctRegistration();
	void ctShowSkin(bool show);
	void ctDebug();

signals:
	void lineWidgetMove(double *P1, double *P2);

public slots:
	void lineWidgetTransform(double *P1, double *P2);	//这个函数用于将CT坐标系的导引手柄变换到机械臂坐标系，以m_regMatrix作为变换矩阵
	void lineWidgetNoTransform(double *P1, double *P2);	//直接设置targetP坐标系

private:
	QString ctFileName;
	bool m_ctMatched;
	void *m_ctData;
	double m_regMatrix[3][4];
	double targetP1[3], targetP2[3];
protected:
	void timerEvent(QTimerEvent *event);
};

