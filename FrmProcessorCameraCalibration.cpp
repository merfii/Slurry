#pragma execution_character_set("utf-8")

#include <iostream>
#include <QMutexLocker>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/vision/vpCalibration.h>

#include "GlobalShared.h"
#include "Logger.h"
#include "slurry.h"
#include "CameraController.h"
#include "SystemParameter.h"
#include "RobotController.h"
#include "LaserProjectorAndSensor.h"

#include "FrmProcessorCameraCalibration.h"
#include <QTest>

using cv::Mat;
using std::cout;
using std::endl;

const int DETECTION_TIMEOUT = 1000;

FrmProcessorCameraCalibration::FrmProcessorCameraCalibration() :
stage(STAGE_STOP)
{
	timestamper.start();
	stage = STAGE_PREVIEW;

	connect(this, SIGNAL(DispInMainWindowA(cv::Mat)), GlobalShared::slurry->ui.viewer1, SLOT(setImage(const cv::Mat)), Qt::DirectConnection);
	connect(this, SIGNAL(DispInMainWindowB(cv::Mat)), GlobalShared::slurry->ui.viewer2, SLOT(setImage(const cv::Mat)), Qt::DirectConnection);

}

FrmProcessorCameraCalibration::~FrmProcessorCameraCalibration()
{

}


void FrmProcessorCameraCalibration::StartCalibration(int camSel)
{
	stage = STAGE_STOP;
	resetAll();
	calibSets = *(SystemParameter::GetCalibSetting());

	//CameraController::GetInstance()->SetPixelModeGray();
	CameraController::GetInstance()->SetModeContinue();
	LaserProjectorAndSensor::GetInstance()->FlickerStop();
	LaserProjectorAndSensor::GetInstance()->LaserOff();
	CameraController::GetInstance()->StartGrabbing();
	
	if (camSel == CAMERA_COLOR_A )
	{
		stage = STAGE_CAPTURE_A;
	}else if (camSel == CAMERA_COLOR_B)
	{
		stage = STAGE_CAPTURE_B;
	}
}

void FrmProcessorCameraCalibration::Preview()
{
	CameraController::GetInstance()->SetModeContinue();
	LaserProjectorAndSensor::GetInstance()->FlickerStop();
	CameraController::GetInstance()->StartGrabbing();
	stage = STAGE_PREVIEW;
}

void FrmProcessorCameraCalibration::StopAndShow(Mat img)
{
	stage = STAGE_STOP;
	if (!img.empty())
		emit DispInDialogBig(img);
}

void FrmProcessorCameraCalibration::StopCalibration()
{
	stage = STAGE_STOP;
}


void FrmProcessorCameraCalibration::resetAll()
{
	QMutexLocker locker(&mutex);

	imageSize = cv::Size();
	imagePoints.clear();
	pointBuf.clear();
}

void FrmProcessorCameraCalibration::ProcessColorA(FramePacket *fp)
{
	//Preview 显示图像
	Mat src = fp->img;
	Mat div4;
	cv::resize(src, div4, cv::Size(), 0.25, 0.25);

	emit DispInMainWindowA(div4);
	imageSize = src.size();
	Mat srcGray;
	bool found = false;
	switch (stage)
	{
	case STAGE_STOP:
		break;

	case STAGE_PREVIEW:
		emit DispInDialogBig(div4);
		break;

	case STAGE_CAPTURE_A:
		//提取特征点并等待保存
		mutex.lock();
		pointBuf.clear();
		found = findChessboardCorners(div4, calibSets.boardSize, pointBuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
		if (found)
		{
			prevImg = src;
			prevTimestamp = timestamper.elapsed();

			// Draw the corners.
			drawChessboardCorners(div4, calibSets.boardSize, Mat(pointBuf), found);
		}
		mutex.unlock();
		//在对话框中显示
		emit DispInDialogBig(div4);
		break;

	case STAGE_CAPTURE_B:
		emit DispInDialogSmall(div4);
		break;

	}
}

void FrmProcessorCameraCalibration::ProcessColorB(FramePacket *fp)
{
	//Preview 显示图像
	Mat src = fp->img;
	Mat div4;
	cv::resize(src, div4, cv::Size(), 0.25, 0.25);
	
	emit DispInMainWindowB(div4);
	imageSize = src.size();
	Mat srcGray;
	bool found = false;
	switch (stage)
	{
	case STAGE_STOP:
		break;

	case STAGE_PREVIEW:
		emit DispInDialogSmall(div4);
		break;

	case STAGE_CAPTURE_B:
		//提取特征点并等待保存
		mutex.lock();
		pointBuf.clear();
		found = findChessboardCorners(div4, calibSets.boardSize, pointBuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
		if (found)
		{
			prevImg = src;
			prevTimestamp = timestamper.elapsed();

			// Draw the corners.
			drawChessboardCorners(div4, calibSets.boardSize, Mat(pointBuf), found);
		}
		mutex.unlock();
		//在对话框中显示
		emit DispInDialogBig(div4);
		break;

	case STAGE_CAPTURE_A:
		emit DispInDialogSmall(div4);
		break;
	}
}

//若相邻两帧相同 则不采集 尚未完成
bool FrmProcessorCameraCalibration::framesAreDifferent()
{
	return true;
}

bool FrmProcessorCameraCalibration::CapturePoints()
{
	Mat greyImg;
	QMutexLocker locker(&mutex);
	robotMtcp.push_back(RobotController::GetInstance()->GetRobotFrame());
	bool s = pointBuf.empty() || timestamper.elapsed() - prevTimestamp >= DETECTION_TIMEOUT || !framesAreDifferent();
	if (!s)
	{
		pointsMultiply<cv::Point2f>(pointBuf, 4);		//原来角点的提取是在1/4缩略图上进行的，现在要放大4倍

		if (prevImg.channels() != 1)
		{
			cvtColor(prevImg, greyImg, cv::COLOR_BGR2GRAY);
		}
		else
		{
			greyImg = prevImg;
		}
		cornerSubPix(greyImg, pointBuf, cv::Size(15, 15),
			cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.01));
		
		imagePoints.push_back(pointBuf);
		Logger::PrintLog(QString("捕获角点 %1 个").arg(pointBuf.size()));
	}
	return !s;
}


static double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
	const std::vector<std::vector<cv::Point2f> >& imagePoints,
	const std::vector<Mat>& rvecs, const std::vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	std::vector<float>& perViewErrors);

bool FrmProcessorCameraCalibration::Solve(CameraParameter &param)
{
	int select = stage;
	stage = STAGE_STOP;

	param.boardSize = calibSets.boardSize;
	param.imageSize = imageSize;
	param.aspectRatio = calibSets.aspectRatio;
	
	QMutexLocker locker(&mutex);
	/************内参数标定*************/
	if (calibSets.flag & cv::CALIB_FIX_ASPECT_RATIO)
	{
		param.cameraMatrix.at<double>(0, 0) = calibSets.aspectRatio;
	}
	if (imagePoints.size() == 0)
	{
		return false;
	}

	std::vector<std::vector<cv::Point3f> > chessboardPoints(1);
	generateChessboardCorners(calibSets.boardSize, calibSets.squareSize, chessboardPoints[0]);
//	cout << chessboardPoints[0] << endl;
	chessboardPoints.resize(imagePoints.size(), chessboardPoints[0]);
	//Find intrinsic and extrinsic camera parameters
	param.rms = calibrateCamera(chessboardPoints, imagePoints, imageSize, param.cameraMatrix, param.distCoeffs, param.rvecs, param.tvecs, calibSets.flag);
	param.ok = checkRange(param.cameraMatrix) && checkRange(param.distCoeffs);
	param.totalAvgErr = computeReprojectionErrors(chessboardPoints, imagePoints, param.rvecs, param.tvecs, param.cameraMatrix,
			param.distCoeffs, param.reprojErrs);

	//显示结果
	cout << param.cameraMatrix << endl << endl;
	cout << param.distCoeffs << endl << endl;
	for (int i = 0; i < param.tvecs.size(); i++)
		cout << param.tvecs[i] << endl;

	Mat preview;
	undistort(prevImg, preview, param.cameraMatrix, param.distCoeffs);
	emit DispInDialogBig(preview);
	emit DispInDialogSmall(prevImg);

	cout << (param.ok ? "Instrinsac calibration succeeded" : "Calibration failed")
		<< ". average reprojection error = " << param.totalAvgErr << endl;
	
	/************手眼标定*************/
	//使用上一步中的外参数rvecs tvecs作为初始值 精确计算变换
	cout << "Type:" << param.rvecs[0].type() << " size:" << param.rvecs[0].size() << endl;
	std::vector<vpHomogeneousMatrix> eyeMobject, worldMtcp;
	vpHomogeneousMatrix tcpMeye;
	for (int i = 0; i < chessboardPoints.size(); i++)
	{
		if (!solvePnP(chessboardPoints[i], imagePoints[i], param.cameraMatrix, param.distCoeffs, param.rvecs[i], param.tvecs[i], true))
		{
			return false;
		}
		vpHomogeneousMatrix eMo;
		double *pt = param.tvecs[i].ptr<double>();
		double *pr = param.rvecs[i].ptr<double>();
		//注意要以m为单位
		eMo.buildFrom(pt[0], pt[1], pt[2], pr[0], pr[1], pr[2]);
		eyeMobject.push_back(eMo);
	}

	for (int i = 0; i < robotMtcp.size(); i++)
	{
		double *P = robotMtcp[i].P;
		vpHomogeneousMatrix wMt;
		wMt.buildFrom(P[0], P[1], P[2], P[3], P[4], P[5]);
		worldMtcp.push_back(wMt);
	}

	vpCalibration::calibrationTsai(eyeMobject, worldMtcp, tcpMeye);
	cout <<  "tcpMeye estimated " << endl << tcpMeye << endl;
	//tcpMeye.extract(erc);
	//std::cout << "Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2]) << std::endl;

	tcpMeye.convert(param.hand2eye);
	return true;
}


void FrmProcessorCameraCalibration::debugUndistort()
{
	CameraParameter *cp = new CameraParameter;
	cp->load(CAMERA_COLOR_A_PARAM_FILE);
	cout << cp->cameraMatrix << endl << endl;
	      
	Mat src,dst;
	src = cv::imread("img05.png");

	undistort(src, dst, cp->cameraMatrix, cp->distCoeffs);
	imshow("Src", src);
	imshow("Dst", dst);
	cv::waitKey(-1);
}


int FrmProcessorCameraCalibration::framesCaptured() const
{
	return imagePoints.size();
}


static double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
	const std::vector<std::vector<cv::Point2f> >& imagePoints,
	const std::vector<Mat>& rvecs, const std::vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	std::vector<float>& perViewErrors)
{
	std::vector<cv::Point2f> imagePoints2;
	size_t totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (size_t i = 0; i < objectPoints.size(); ++i)
	{
		projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
		err = norm(imagePoints[i], imagePoints2, cv::NORM_L2);

		size_t n = objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}
	return std::sqrt(totalErr / totalPoints);
}

