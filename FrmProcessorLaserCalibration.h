#pragma once

#include <opencv2/core.hpp>
#include <QAtomicInt>
#include <QMutex>
#include <QElapsedTimer>
#include "FrameProcessor.h"


/*

该类包含帧回调函数 两相机同时采集 
使用PnP方法解算棋盘格平面 提取激光特征点计算激光平面

与CalibrationDialog耦合紧密
Dialog->Processor: 捕获 解算 保存
Processor->Dialog: 解算误差 标定结果
当对象被删除后signal-slot之间的连接自动断开
*/




class FrmProcessorLaserCalibration : public QObject, public FrameProcessor
{
	Q_OBJECT

public:
	FrmProcessorLaserCalibration();
	~FrmProcessorLaserCalibration();

	void ProcessColorA(FramePacket *fp);
	void ProcessColorB(FramePacket *fp);

	void Preview();
	void StartCalibration();
	void StopCalibration();
	bool CapturePoints();
	bool SolveLaserPlane();

	int framesCaptured() const;
	std::string GetResultString();

signals:
	void DispInDialogBig(cv::Mat img);
	void DispInDialogSmall(cv::Mat img);

	void DispInMainWindowA(cv::Mat img);
	void DispInMainWindowB(cv::Mat img);

private:
	void resetAll();


	QAtomicInt stage;
	const cv::Size boardSize;
	enum STAGE{ STAGE_STOP, STAGE_PREVIEW, STAGE_CAPTURE };
	qint64 prevTimestamp;
	QElapsedTimer timestamper;

	struct _DATA{
		Mat cameraMatrix, distCoeffs;	  // vector of distortion coefficients

		QMutex mutex;
		//所有tmp变量是双缓冲 以在奇偶帧维持变量的一致性
		Mat PnpImg, PnpImgTmp;
		Mat diff;
		Mat chessPointsBuf, chessPointsBufTmp;
		Mat laserPoints3D;
		Mat laserPlane;
		int nCapture;

		_DATA();
		~_DATA();
		void _procEven(cv::Mat& src);
		void _procOdd(cv::Mat& src, cv::Mat& div2, const cv::Size& boardSize);
		bool _capture();
		bool _solve();
		void _clear();
	};

	struct _DATA m_A, m_B;
};
