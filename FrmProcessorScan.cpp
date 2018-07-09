#pragma execution_character_set("utf-8")

#include <QElapsedTimer>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpPoint.h>
#undef M_PI_2
#undef M_PI_4

#include "slurry.h"
#include "GlobalShared.h"
#include "Slurry.h"
#include "ScanVisualizer.h" 
#include "Logger.h"
#include "CameraController.h"
#include "LaserProjectorAndSensor.h"
#include "RobotController.h"
#include "RobotStateHistory.h"
#include "SystemParameter.h"

#include "FrmProcessorScan.h"

using std::cout;
using std::endl;

static double DOUBLE_EPS = 1e-4;

FrmProcessorScan::FrmProcessorScan() :
stage(STAGE_STOP), boardSize(SystemParameter::GetCalibSetting()->boardSize)
{
	m_A.cameraMatrix = SystemParameter::GetCameraColorAParam()->cameraMatrix;
	m_A.distCoeffs = SystemParameter::GetCameraColorAParam()->distCoeffs;
	m_A.laserPlane = SystemParameter::GetCameraColorAParam()->laserPlane;
	m_A.tcpMeye = new vpHomogeneousMatrix;
	m_A.tcpMeye->buildFrom(SystemParameter::GetCameraColorAParam()->hand2eye);

	m_B.cameraMatrix = SystemParameter::GetCameraColorBParam()->cameraMatrix;
	m_B.distCoeffs = SystemParameter::GetCameraColorBParam()->distCoeffs;
	m_B.laserPlane = SystemParameter::GetCameraColorBParam()->laserPlane;
	m_B.tcpMeye = new vpHomogeneousMatrix;
	m_B.tcpMeye->buildFrom(SystemParameter::GetCameraColorBParam()->hand2eye);

	m_A.TimeStamp.cameraExposureTime = SystemParameter::GetInstance()->GetSettingInt(QStringLiteral("Settings/CameraControl/ExposureTime"));
	m_B.TimeStamp.cameraExposureTime = SystemParameter::GetInstance()->GetSettingInt(QStringLiteral("Settings/CameraControl/ExposureTime"));
	timestamper = new QElapsedTimer;
	timestamper->start();
}

FrmProcessorScan::~FrmProcessorScan()
{
	delete timestamper;
}
                     

void FrmProcessorScan::StartScan()
{
	stage = STAGE_SCAN;
	CameraController *camera = CameraController::GetInstance();
	LaserProjectorAndSensor::GetInstance()->FlickerStop();
	camera->StopGrabbing();

	camera->SetPixelModeColor();
	camera->SetAOIenable(true);
	camera->SetModeExtTrig();
	camera->ResetFrameCount();
	camera->StartGrabbing();
	LaserProjectorAndSensor::GetInstance()->FlickerRunFast();
}

void FrmProcessorScan::StopScan()
{
	LaserProjectorAndSensor::GetInstance()->FlickerStop();
	stage = STAGE_STOP;
}


void FrmProcessorScan::ProcessColorA(FramePacket *fp)
{
	//获取对应机械臂时间戳
	RobotStateHistory *robotHistory = RobotController::GetRobotStateHistory();

	//Preview 显示图像
	Mat src = fp->img;
	Mat div4;
	cv::resize(src, div4, cv::Size(), 0.25, 0.25);
	//cout << fp->framecount << endl;
	switch (stage)
	{
	case STAGE_STOP:
		break;

	case STAGE_PREVIEW:
		break;

	case STAGE_SCAN:
		if (fp->framecount % 2 == 0)
		{
			m_A.TimeStamp.ok_even = robotHistory->getSyncSignal(m_A.TimeStamp.fromRobotEven);
			m_A.TimeStamp.fromComputerTimerEven = timestamper->elapsed();
			
			//偶数帧 有激光线
			if (!m_A.oddImgTmp.empty())
			{
				m_A._procEven(src, m_A.TimeStamp);
				//显示差分图像
				//emit DispInMainWindowB(m_A.diff);
			}
		}
		else
		{
			m_A.TimeStamp.ok_odd = robotHistory->getSyncSignal(m_A.TimeStamp.fromRobotOdd);
			m_A.TimeStamp.fromComputerTimerOdd = timestamper->elapsed();

			//奇数帧 无激光线
			m_A._procOdd(src);
			emit DispInMainWindowA(div4);
		}
		break;
	}
}

void FrmProcessorScan::ProcessColorB(FramePacket *fp)
{
	//获取对应机械臂时间戳
	RobotStateHistory *robotHistory = RobotController::GetRobotStateHistory();

	//Preview 显示图像
	Mat src = fp->img;
	Mat div4;
	cv::resize(src, div4, cv::Size(), 0.25, 0.25);
	emit DispInMainWindowB(div4);

	switch (stage)
	{
	case STAGE_STOP:
		break;

	case STAGE_PREVIEW:
		break;

	case STAGE_SCAN:
		if (fp->framecount % 2 == 0)
		{

			m_B.TimeStamp.ok_even = robotHistory->getSyncSignal(m_B.TimeStamp.fromRobotEven);
			m_B.TimeStamp.fromComputerTimerEven = timestamper->elapsed();
			//偶数帧 有激光线
			if (!m_B.oddImgTmp.empty())
			{
				m_B._procEven(src, m_B.TimeStamp);
				//显示差分图像
				//emit DispInMainWindowB(m_A.diff);
			}
		}
		else
		{
			m_B.TimeStamp.ok_odd = robotHistory->getSyncSignal(m_B.TimeStamp.fromRobotOdd);
			m_B.TimeStamp.fromComputerTimerOdd = timestamper->elapsed();

			//奇数帧 无激光线
			m_B._procOdd(src);
			emit DispInMainWindowA(div4);
		}
		break;
	}
}

FrmProcessorScan::_DATA::_DATA()
{
	tcpMeye = nullptr;
	point_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}


FrmProcessorScan::_DATA::~_DATA()
{
	mutex.lock();
	mutex.unlock();
	delete tcpMeye;
}

static std::vector<vpColor> colorLineDetect(Mat& oddImg, Mat& laserPoints2D);
static void point2Dto3D(Mat& plane, Mat& p2d, std::vector<vpColor>& colorIn, std::vector<vpColVector>& p3d, std::vector<vpColor>& colorOut);
void FrmProcessorScan::_DATA::_procEven(Mat& src, struct _TIMESTAMP& stamp)
{
	RobotFrame frame;
	if (stamp.ok_even && stamp.ok_odd)
	{
		RobotStateRT state;
		if (!RobotController::GetRobotStateHistory()->findStateInTimeStamp(
			(stamp.fromRobotEven + stamp.fromRobotOdd) / 2 + 0, //(double)stamp.cameraExposureTime / 2/1000,
			state))
		{
			qDebug() << stamp.fromRobotEven << " " << stamp.fromRobotOdd;
			return;
		}else{
			frame = state.getFrame();
			if (!frame.isValid()){
				oddImgTmp = Mat();
				return;
			}
		}
	}
	else
	{
		frame = RobotController::GetRobotFrame();
	}

	absdiff(src, oddImgTmp, diff);
	oddImg = oddImgTmp;	//这里设置tmp变量作为双缓冲 以在Capture时维持各变量的一致性
	oddImgTmp = Mat();
	
	//提取激光线的上的点 (使用原始图像 对提取后的点进行畸变矫正)
	Mat laserPoints2D = laserLineDetect(diff);

	if (laserPoints2D.empty())	
		return;

	std::vector<vpColor> colorPoints2D = colorLineDetect(oddImg, laserPoints2D);

	for (int i = 0; i < laserPoints2D.rows; i++)
	{
		laserPoints2D.ptr<double>(i)[1] += ROI_OFFSET;
	}
	
	undistortPoints(laserPoints2D, laserPoints2D, cameraMatrix, distCoeffs);
	//std::cout << laserPoints2D.size() << endl;
	
	//激光点2D to 3D
	std::vector<vpColVector> laserPoints3D;		//vpPoint的结构太复杂了我不想用...
	std::vector<vpColor> colorPoints3D;
//int64_t t1 = cv::getTickCount();

	point2Dto3D(laserPlane, laserPoints2D, colorPoints2D, laserPoints3D, colorPoints3D);
//qDebug() << "2D_3D dt" <<(cv::getTickCount() - t1) / cv::getTickFrequency() * 1000 << endl;;	//ms

	//转换到世界坐标
	vpHomogeneousMatrix worldMtcp, worldMeye;
	worldMtcp.buildFrom(
		frame.P[0],
		frame.P[1],
		frame.P[2],
		frame.P[3],
		frame.P[4],
		frame.P[5]);	//unit: m m m rad rad rad

	worldMeye = worldMtcp * (*tcpMeye);
	//cout << worldMeye << endl << endl;

	for (int i = 0; i < laserPoints3D.size(); i++)
	{
		laserPoints3D[i] = worldMeye * laserPoints3D[i];
	}

	//生成PointCloud
	CV_Assert(laserPoints3D.size() == colorPoints3D.size());
	for (int i = 0; i < laserPoints3D.size(); i++)
	{
		vpColVector& p = laserPoints3D[i];
		vpColor& c = colorPoints3D[i];
		pcl::PointXYZRGB point;
		point.x = p[0];
		point.y = p[1];
		point.z = p[2];
		point.r = c.R;
		point.g = c.G;
		point.b = c.B;

		/*if (i % 100 == 0)
		{
			cout << point.x << " " << point.y << " " << point.z << " " << point.r << endl;
		}*/
		//uint32_t rgb = (static_cast<uint32_t>(r) << 16 |static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
		//point.rgb = *reinterpret_cast<float*>(&rgb);
		point_cloud->push_back(point);
	}

	//显示点云数据
	GlobalShared::slurry->scanVisualizer->ScanTaskShotPoints(point_cloud);
	point_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

}

void FrmProcessorScan::_DATA::_procOdd(Mat& src)
{
	//奇数帧 无激光线
	oddImgTmp = src;
}

static std::vector<vpColor> colorLineDetect(Mat& oddImg, Mat& laserPoints2D)
{
	std::vector<vpColor> colors;
	for (int i = 0; i < laserPoints2D.rows; i++)
	{
		int x = (int)laserPoints2D.ptr<double>(i)[0];
		int y = (int)laserPoints2D.ptr<double>(i)[1];
		colors.push_back(vpColor(
			oddImg.at<cv::Vec3b>(y, x)[0],
			oddImg.at<cv::Vec3b>(y, x)[1],
			oddImg.at<cv::Vec3b>(y, x)[2]));
	}
	return colors;
}

//单位为m
static void point2Dto3D(Mat& plane, Mat& p2d, std::vector<vpColor>& colorIn, std::vector<vpColVector>& p3d, std::vector<vpColor>& colorOut)
{
	CV_Assert(p2d.rows == colorIn.size());
	CV_Assert(p2d.cols == 1);	//channel = 2
	p3d.clear();
	colorOut.clear();
	vpColVector vpp(4);

	for (int i = 0; i < p2d.rows; i++)
	{
		double *p_p2d = p2d.ptr<double>(i);

		double u = p_p2d[0];
		double v = p_p2d[1];
		double *ple = plane.ptr<double>();
		double AuBvC = ple[0] * u + ple[1] * v + ple[2];
		if (abs(AuBvC) < DOUBLE_EPS || -ple[3] / AuBvC <= 0)
		{
			qDebug() << "One bad 3D laser point!";
			continue;
		}
		double x, y, z;
		z = -ple[3] / AuBvC;
		x = z * u;
		y = z * v;
		vpp[0] = x; vpp[1] = y; vpp[2] = z; vpp[3] = 1;

		if (x > 0.5 || x < -0.5)
			continue;
		if (y > 0.5 || y < -0.5)
			continue;
		if (z > 0.6 || z <0)
			continue;

		p3d.push_back(vpp);
		colorOut.push_back(colorIn[i]);
	}
}