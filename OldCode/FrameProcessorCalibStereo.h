#pragma once

#include <opencv2/core.hpp>
#include <QAtomicInt>
#include <QMutex>
#include <QElapsedTimer>
#include "FrameProcessor.h"


/*

该类包含帧回调函数 两相机同时采集 
提取棋盘格计算双目参数
提取激光特征点计算激光平面
与CalibrationDialog耦合紧密

Dialog->Processor: 捕获 解算 保存
Processor->Dialog: 解算误差 标定结果
当对象被删除后signal-slot之间的连接自动断开
*/

class FrameProcessorCalibStereo : public QObject, public FrameProcessor
{
	Q_OBJECT

public:
	FrameProcessorCalibStereo();
	~FrameProcessorCalibStereo();

	void ProcessGrayA(FramePacket *fp);
	void ProcessColorA(FramePacket *fp);

	void StartCalibration();
	bool CapturePoints();
	bool SolveCameraMatrix();
	void SaveCameraMatrix();

	std::string GetResultString();
	bool isEnoughFrames();

signals:
	void DispInDialogGray(cv::Mat img);
	void DispInDialogColor(cv::Mat img);

	void DispInMainWindowGray(cv::Mat img);
	void DispInMainWindowColor(cv::Mat img);

private:
	void resetAll();


	enum STAGE{ STAGE_STOP, STAGE_DETECT };
	CameraParameter camGrayParam, camColorParam;
	StereoParameter stereoParam;

	cv::Size boardSize;
	int nrFrames;
	QAtomicInt stage;
	cv::Size imageSize;
	QMutex mutex;
	qint64 prevTimestamp;
	QElapsedTimer timestamper;

	std::vector<std::vector<cv::Point2f>>  imagePointsGray, imagePointsColor;
	std::vector<cv::Point2f> pointBufGray, pointBufColor;
	std::vector<std::vector<cv::Point3f> > objectPoints;

	//Used in preview after undistort
	cv::Mat prevImgGray, prevImgColor;



};
