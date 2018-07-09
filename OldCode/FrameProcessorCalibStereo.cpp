#include <cstring>
#include <QMutexLocker>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "GlobalShared.h"
#include "slurry.h"
#include "CameraController.h"
#include "SystemParameter.h"

#include "FrameProcessorCalibStereo.h"

using namespace cv;

const int DETECTION_TIMEOUT = 1000;

FrameProcessorCalibStereo::FrameProcessorCalibStereo() :
stage(STAGE_STOP)
{

}

FrameProcessorCalibStereo::~FrameProcessorCalibStereo()
{
}


void FrameProcessorCalibStereo::StartCalibration()
{
	stage = STAGE_DETECT;

	boardSize = SystemParameter::GetCalibSetting()->boardSize;
	nrFrames = SystemParameter::GetCalibSetting()->nrFrames;
	camGrayParam = *(SystemParameter::GetCameraGrayParam());
	camColorParam = *(SystemParameter::GetCameraColorParam());

	resetAll();
}

void FrameProcessorCalibStereo::resetAll()
{
	QMutexLocker locker(&mutex);

	imageSize = Size();
	imagePointsGray.clear();
	imagePointsColor.clear();
	pointBufGray.clear();
	pointBufColor.clear();

}

void FrameProcessorCalibStereo::ProcessGrayA(FramePacket *fp)
{
	//Preview 显示图像
	cv::Mat src = BaslerMono2Mat(fp->buffer, fp->width, fp->height);
	cv::Mat img05;
	pyrDown(src, img05);
	emit DispInMainWindowGray(img05);

	imageSize = src.size();
	std::vector<cv::Point2f> pointBufTmp;
	Mat dst;
	bool found = false;
	switch (stage)
	{
	case STAGE_DETECT:
		//提取特征点并等待保存
		found = findChessboardCorners(src, boardSize, pointBufTmp, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		prevImgGray = img05;
		if (found)
		{
			//cvtColor(view, viewGray, COLOR_BGR2GRAY);
			cornerSubPix(src, pointBufTmp, Size(11, 11),
				Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 20, 0.05));

			mutex.lock();
			prevTimestamp = timestamper.elapsed();
			pointBufGray = pointBufTmp;
			mutex.unlock();

			// Draw the corners.
			dst = src;		//dst = src.clone();
			drawChessboardCorners(dst, boardSize, Mat(pointBufTmp), found);

			//在对话框中显示
			pyrDown(dst, dst);
			emit DispInDialogGray(dst);
		}
		break;

	case STAGE_STOP:
	default:
		break;
	}
}

void FrameProcessorCalibStereo::ProcessColorA(FramePacket *fp)
{
	//Preview 显示图像
	cv::Mat src = BaslerBG2Mat(fp->buffer, fp->width, fp->height);
	cv::Mat img05;
	pyrDown(src, img05);
	emit DispInMainWindowColor(img05);

	imageSize = src.size();
	std::vector<cv::Point2f> pointBufTmp;
	Mat srcGray;
	bool found = false;
	switch (stage)
	{
	case STAGE_DETECT:
		//提取特征点并等待保存

		found = findChessboardCorners(src, boardSize, pointBufTmp, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		prevImgColor = img05;
		if (found)
		{
			cvtColor(src, srcGray, COLOR_BGR2GRAY);
			cornerSubPix(srcGray, pointBufTmp, Size(11, 11),
				Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 20, 0.1));

			mutex.lock();
			prevTimestamp = timestamper.elapsed();
			pointBufColor = pointBufTmp;
			mutex.unlock();

			// Draw the corners.
			Mat dst = src;	//dst = src.clone();
			drawChessboardCorners(dst, boardSize, Mat(pointBufTmp), found);

			//在对话框中显示
			pyrDown(dst, dst);
			emit DispInDialogColor(dst);
		}
		break;

	case STAGE_STOP:
	default:
		break;
	}
}


bool FrameProcessorCalibStereo::CapturePoints()
{
	QMutexLocker locker(&mutex);
	bool s = pointBufGray.empty() || timestamper.elapsed() - prevTimestamp >= DETECTION_TIMEOUT;
	if (!s)
	{
		imagePointsGray.push_back(pointBufGray);
		imagePointsColor.push_back(pointBufColor);
	}
	return !s;
}


static void calcBoardCornerPositions(Size boardSize, float squareSize, std::vector<Point3f>& corners);

static double computeReprojectionErrors(
	const std::vector<std::vector<Point2f> >& imagePointsGray,
	const std::vector<std::vector<Point2f> >& imagePointsColor,
	const Mat& cameraMatrixGray,
	const Mat& cameraMatrixColor,
	const Mat& distCoeffsGray,
	const Mat& distCoeffsColor,
	const Mat& F);


static Mat stereoPreview(
	const Mat& srcImgGray, const Mat& srcImgColor,
	const Mat& cameraMatrixGray, const Mat& cameraMatrixColor,
	const Mat& distCoeffsGray, const Mat& distCoeffsColor,
	Mat R1, Mat P1,
	Mat R2, Mat P2,
	Size imageSize);

bool FrameProcessorCalibStereo::SolveCameraMatrix()
{
	return true;
}


void FrameProcessorCalibStereo::SaveCameraMatrix()
{
	QMutexLocker locker(&mutex);

	stereoParam.save();
}

bool FrameProcessorCalibStereo::isEnoughFrames()
{
	QMutexLocker locker(&mutex);
	return imagePointsGray.size() >= nrFrames;
}

std::string FrameProcessorCalibStereo::GetResultString()
{
	std::ostringstream o;
	o << "R" << stereoParam.R;
	o << "T" << stereoParam.T;
	o << "R1" << stereoParam.R1;
	o << "R2" << stereoParam.R2;
	o << "P1" << stereoParam.P1;
	o << "P2" << stereoParam.P2;
	o << "Q" << stereoParam.Q;
	o << "Avg Reprj Error" << stereoParam.totalAvgErr;
//	o << "Camera Matrix\n" << camCur->cameraMatrix;
//	o << "Distortion Coefficients" << camCur->distCoeffs;
	return o.str();
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, std::vector<Point3f>& corners)
{
	corners.clear();
	//	case Settings::CHESSBOARD:
	//	case Settings::CIRCLES_GRID:
	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners.push_back(Point3f(j*squareSize, i*squareSize, 0));

}


static double computeReprojectionErrors(
	const std::vector<std::vector<Point2f> >& imagePointsGray,
	const std::vector<std::vector<Point2f> >& imagePointsColor,
	const Mat& cameraMatrixGray,
	const Mat& cameraMatrixColor,
	const Mat& distCoeffsGray,
	const Mat& distCoeffsColor,
	const Mat& F)
{
	double err = 0;
	int npoints = 0;
	std::vector<Vec3f> linesForColor, linesForGray;
	for (int i = 0; i < imagePointsGray.size(); i++)
	{
		int npt = (int)imagePointsGray[i].size();
		Mat imgpt[2];
		imgpt[0] = Mat(imagePointsGray[i]);
		imgpt[1] = Mat(imagePointsColor[i]);

		undistortPoints(imgpt[0], imgpt[0], cameraMatrixGray, distCoeffsGray, Mat(), cameraMatrixGray);
		undistortPoints(imgpt[1], imgpt[1], cameraMatrixGray, distCoeffsGray, Mat(), cameraMatrixGray);

		computeCorrespondEpilines(imgpt[0], 1, F, linesForColor);	//Calculate epipolar lines corresponding to the points in the other image
		computeCorrespondEpilines(imgpt[1], 2, F, linesForGray);

		for (int j = 0; j < npt; j++)
		{
			double errij = fabs(imagePointsGray[i][j].x*linesForGray[j][0] + imagePointsGray[i][j].y*linesForGray[j][1] + linesForGray[j][2]) +
				fabs(imagePointsColor[i][j].x*linesForColor[j][0] + imagePointsColor[i][j].y*linesForColor[j][1] + linesForColor[j][2]);
			err += errij;
		}
		npoints += npt;
	}
	double avgEpipErr = err / npoints;
	cout << "average epipolar err = " << avgEpipErr << endl;
	return avgEpipErr;
}

static Mat stereoPreview(
	const Mat& srcImgGray, const Mat& srcImgColor,
	const Mat& cameraMatrixGray, const Mat& cameraMatrixColor,
	const Mat& distCoeffsGray, const Mat& distCoeffsColor, 
	Mat R1, Mat P1,
	Mat R2, Mat P2,
	Size imageSize)
{
	Mat rmapGray[2], rmapColor[2];
	initUndistortRectifyMap(cameraMatrixGray, distCoeffsGray, R1, P1, imageSize, CV_16SC2, rmapGray[0], rmapGray[1]);
	initUndistortRectifyMap(cameraMatrixColor, distCoeffsColor, R2, P2, imageSize, CV_16SC2, rmapColor[0], rmapColor[1]);

	Mat canvas;
	double sf;
	int w, h;


	sf = 400. / MAX(imageSize.width, imageSize.height);
	w = cvRound(imageSize.width*sf);
	h = cvRound(imageSize.height*sf);
	canvas.create(h * 2, w, CV_8UC3);

	Mat dstimg, canvasPart;

	remap(srcImgGray, dstimg, rmapGray[0], rmapGray[1], INTER_LINEAR);
	canvasPart = canvas(Rect(0, 0, w, h));
	resize(dstimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
	remap(srcImgColor, dstimg, rmapColor[0], rmapColor[1], INTER_LINEAR);
	canvasPart = canvas(Rect(w, 0, w, h));
	resize(dstimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
	
	for (int j = 0; j < canvas.cols; j += 16)
	{
		line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
	}
	return canvas;
}