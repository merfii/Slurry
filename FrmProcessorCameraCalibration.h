#pragma once

#include <QAtomicInt>
#include <QMutex>
#include <QElapsedTimer>
#include <opencv2/core.hpp>
#include "RobotFrame.h"

#include "FrameProcessor.h"


/*
对相机进行内参数标定和手眼标定 一次标定一个相机
该类包含帧回调函数 相机采集帧、提取特征点、解算标定  然后是另一个相机

Dialog控制Processor: 启动 捕获 装载数据
Processor控制Dialog: 显示
当对象被删除后signal-slot之间的连接自动断开

*/

class FrmProcessorCameraCalibration :public QObject, public FrameProcessor
{
	Q_OBJECT
public:
	FrmProcessorCameraCalibration();
	~FrmProcessorCameraCalibration();

	void ProcessColorA(FramePacket *fp);
	void ProcessColorB(FramePacket *fp);
	
	void StartCalibration(int camSel);
	void Preview();
	void StopCalibration();
	void StopAndShow(cv::Mat img);
	bool CapturePoints();
	bool Solve(CameraParameter &param);

	int framesCaptured() const;
	void debugUndistort();

signals:
	void DispInDialogBig(cv::Mat img);
	void DispInDialogSmall(cv::Mat img);

	void DispInMainWindowA(cv::Mat img);
	void DispInMainWindowB(cv::Mat img);

private:
	//当前关键点是否与上一次所采集不同
	bool framesAreDifferent();
	void resetAll();


	QAtomicInt stage;
	enum STAGE{ STAGE_STOP, STAGE_PREVIEW, STAGE_CAPTURE_A, STAGE_CAPTURE_B };
	CalibrationSetting calibSets;

	cv::Size imageSize;
	QElapsedTimer timestamper;
	QMutex mutex;
	qint64 prevTimestamp;

	std::vector<std::vector<cv::Point2f>>  imagePoints;
	std::vector<cv::Point2f> pointBuf;
	std::vector<RobotFrame>  robotMtcp;
	//Used in preview after undistort
	cv::Mat prevImg;
	
};
