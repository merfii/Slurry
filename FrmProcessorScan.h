#pragma once

#include <pcl/common/common.h>
#include <opencv2/core.hpp>
#include <QAtomicInt>
#include <QMutex>

#include "RobotFrame.h"
#include "FrameProcessor.h"

#define ROI_OFFSET 700
#define PADDING_WIDTH 100

class vpHomogeneousMatrix;
class QElapsedTimer;
class FrmProcessorScan : public QObject, public FrameProcessor
{
	Q_OBJECT
public:
	FrmProcessorScan();
	~FrmProcessorScan();

	void ProcessColorA(FramePacket *fp);
	void ProcessColorB(FramePacket *fp);

	void StartScan();
	void StopScan();

signals:
	void DispInDialogBig(cv::Mat img);
	void DispInDialogSmall(cv::Mat img);

	void DispInMainWindowA(cv::Mat img);
	void DispInMainWindowB(cv::Mat img);

private:

	QElapsedTimer *timestamper;
	QAtomicInt stage;
	const cv::Size boardSize;
	enum STAGE{ STAGE_STOP, STAGE_PREVIEW, STAGE_SCAN };

	struct _TIMESTAMP
	{
		qint64 fromComputerTimerOdd, fromComputerTimerEven;
		double fromRobotOdd, fromRobotEven;
		bool ok_odd, ok_even;
		double cameraExposureTime;
	};

	struct _DATA{
		_DATA();
		~_DATA();
		Mat cameraMatrix, distCoeffs;	  // vector of distortion coefficients
		Mat laserPlane;
		//所有tmp变量是双缓冲 以在奇偶帧维持变量的一致性
		Mat oddImg, oddImgTmp;
		RobotFrame oddFrm, evenFrm;
		Mat diff;
		Mat laserPoints3D;
		QMutex mutex;
		vpHomogeneousMatrix *tcpMeye;	// hand (end-effector) to eye (camera) transformation
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud;

		struct _TIMESTAMP TimeStamp;
		void _procEven(cv::Mat& src, struct _TIMESTAMP& stamp);
		void _procOdd(cv::Mat& src);
	};

	struct _DATA m_A, m_B;
};

