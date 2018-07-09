                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       #pragma execution_character_set("utf-8")

#include <iostream>
#include <QMutexLocker>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include "GlobalShared.h"
#include "Logger.h"
#include "slurry.h"
#include "CameraController.h"
#include "LaserProjectorAndSensor.h"
#include "SystemParameter.h"

#include "FrmProcessorLaserCalibration.h"
//#include <QTest>

using std::cout;
using std::endl;
using cv::Point3d;

static double DOUBLE_EPS = 1e-3;
static bool isJitteredTooMuch(Mat& a, Mat& b);

FrmProcessorLaserCalibration::FrmProcessorLaserCalibration() :
stage(STAGE_STOP), boardSize(SystemParameter::GetCalibSetting()->boardSize)
{
	timestamper.start();
	m_A.cameraMatrix = SystemParameter::GetCameraColorAParam()->cameraMatrix;
	m_A.distCoeffs = SystemParameter::GetCameraColorAParam()->distCoeffs;
	m_A.laserPlane = SystemParameter::GetCameraColorAParam()->laserPlane;

	m_B.cameraMatrix = SystemParameter::GetCameraColorBParam()->cameraMatrix;
	m_B.distCoeffs = SystemParameter::GetCameraColorBParam()->distCoeffs;
	m_B.laserPlane = SystemParameter::GetCameraColorBParam()->laserPlane;
	
	connect(this, SIGNAL(DispInMainWindowA(cv::Mat)), GlobalShared::slurry->ui.viewer1, SLOT(setImage(const cv::Mat)), Qt::DirectConnection);
	connect(this, SIGNAL(DispInMainWindowB(cv::Mat)), GlobalShared::slurry->ui.viewer2, SLOT(setImage(const cv::Mat)), Qt::DirectConnection);
}

FrmProcessorLaserCalibration::~FrmProcessorLaserCalibration()
{
}


void FrmProcessorLaserCalibration::StartCalibration()
{
	m_A._clear();
	m_B._clear();
	stage = STAGE_CAPTURE;
	CameraController *camera = CameraController::GetInstance();
	LaserProjectorAndSensor::GetInstance()->FlickerStop();
	LaserProjectorAndSensor::GetInstance()->LaserOff();
	camera->StopGrabbing();
	camera->SetPixelModeColor();
	camera->SetAOIenable(false);
	camera->SetModeExtTrig();
	camera->ResetFrameCount();
	camera->StartGrabbing();
	LaserProjectorAndSensor::GetInstance()->FlickerRunSlow();

}

void FrmProcessorLaserCalibration::StopCalibration()
{
	LaserProjectorAndSensor::GetInstance()->FlickerStop();
	stage = STAGE_STOP;
}


void FrmProcessorLaserCalibration::Preview()
{
	CameraController::GetInstance()->SetModeContinue();
	LaserProjectorAndSensor::GetInstance()->FlickerStop();
	CameraController::GetInstance()->StartGrabbing();

	stage = STAGE_PREVIEW;
}

bool FrmProcessorLaserCalibration::CapturePoints()
{
	
	return m_A._capture() && m_B._capture();

}


void FrmProcessorLaserCalibration::resetAll()
{
	m_A._clear();
	m_B._clear();
	//imageSize = Size();
}

void FrmProcessorLaserCalibration::ProcessColorA(FramePacket *fp)
{
	//Preview 显示图像
	Mat src = fp->img;
	Mat div4;
	
	cv::resize(src, div4, cv::Size(), 0.25, 0.25);
	
	emit DispInMainWindowA(div4);

	switch (stage)
	{
	case STAGE_STOP:
		break;

	case STAGE_PREVIEW:
		emit DispInDialogBig(div4);
		break;

	case STAGE_CAPTURE:
		if (fp->framecount % 2 == 0)
		{
			//偶数帧 有激光线
			if (m_A.PnpImgTmp.empty())
				return;

			m_A._procEven(src);
			prevTimestamp = timestamper.elapsed();

			//显示差分图像
			emit DispInDialogSmall(m_A.diff);
		}else
		{
			//奇数帧 无激光线
			m_A._procOdd(src, div4, boardSize);
			prevTimestamp = timestamper.elapsed();

			//在对话框中显示
			emit DispInDialogBig(div4);
		}
		break;
	}
}

void FrmProcessorLaserCalibration::ProcessColorB(FramePacket *fp)
{
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
	//	emit DispInDialogBig(div4);
		break;

	case STAGE_CAPTURE:
		if (fp->framecount % 2 == 0)
		{
			//偶数帧 有激光线
			if (m_B.PnpImgTmp.empty())
				return;

			m_B._procEven(src);

			//显示差分图像
			//emit DispInDialogSmall(m_B.diff);
		}
		else
		{
			//奇数帧 无激光线
			m_B._procOdd(src, div4, boardSize);
		}
		break;
	}
}

FrmProcessorLaserCalibration::_DATA::_DATA()
{

}

FrmProcessorLaserCalibration::_DATA::~_DATA()
{
	mutex.lock();
	mutex.unlock();
}

void FrmProcessorLaserCalibration::_DATA::_procOdd(Mat& src, Mat& div4, const cv::Size& boardSize)
{
	//奇数帧 无激光线
	PnpImgTmp = src;
	bool found = findChessboardCorners(div4, boardSize, chessPointsBufTmp, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
	if (found)
	{
		drawChessboardCorners(div4, boardSize, chessPointsBufTmp, found);
	}
}

void FrmProcessorLaserCalibration::_DATA::_procEven(Mat& src)
{
	//偶数帧 有激光线
	mutex.lock();
	absdiff(src, PnpImgTmp, diff);
	PnpImg = PnpImgTmp;	//这里设置tmp变量作为双缓冲 以在Capture时维持各变量的一致性
	PnpImgTmp = Mat();
	chessPointsBuf = chessPointsBufTmp;
	chessPointsBufTmp = Mat();
	mutex.unlock();
}

void FrmProcessorLaserCalibration::_DATA::_clear()
{
	mutex.lock();
	chessPointsBuf = Mat();
	chessPointsBufTmp = Mat();
	laserPoints3D = Mat();
	nCapture = 0;
	mutex.unlock();
}

static int laserPointTo3D(Mat& R, Mat& T, Mat& p2d, Mat& p3d);
static void drawLaserLine(Mat& diff, Mat& points);
bool FrmProcessorLaserCalibration::_DATA::_capture()
{
	QMutexLocker locker(&mutex);

	//提取激光线的上的点 (使用原始图像 对提取后的点进行畸变矫正)
	Mat laserPoints2D = laserLineDetect(diff);

	if (laserPoints2D.empty())	return false;
	//cv::imwrite("diff.jpg",diff);
	//imshow("Diff BR", diff);
	//drawLaserLine(diff, laserPoints2D);

	diff = Mat();

	//提取棋盘格点
	if (chessPointsBuf.empty() || PnpImg.empty())
		return false;

	Mat greyImg;
	if (PnpImg.channels() != 1)
	{
		cvtColor(PnpImg, greyImg, cv::COLOR_BGR2GRAY);
	}
	else
	{
		greyImg = PnpImg;
	}
	PnpImg = Mat();
	chessPointsBuf = chessPointsBuf * 4;		//原来角点的提取是在1/4缩略图上进行的，现在要放大4倍
	cornerSubPix(greyImg, chessPointsBuf, cv::Size(15, 15),
		cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.01));
	
	//cout << "laserPoints2D" << endl << laserPoints2D << endl << endl;

	undistortPoints(laserPoints2D, laserPoints2D, cameraMatrix, distCoeffs);
	//cv::FileStorage fs("tttt2D.yml", cv::FileStorage::WRITE);
	//fs << "Points2D" << laserPoints2D;
	//fs.release();


	//SolvePnP 计算棋盘格变换  (包含畸变矫正)
	std::vector<Point3d> chessPoints3d;
	generateChessboardCorners(SystemParameter::GetCalibSetting()->boardSize, SystemParameter::GetCalibSetting()->squareSize, chessPoints3d);
	Mat rvec = Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
	Mat tvec = Mat::zeros(3, 1, CV_64FC1);    // output translation vector
	bool useExtrinsicGuess = false;   // if true the function uses the provided rvec and tvec values as initial approximations of the rotation and translation vectors
	const int iterationsCount = 1000;      // max number of Ransac iterations.
	const float reprojectionError = 2.0;  // maximum allowed distance to consider it an inlier.
	const double confidence = 0.98;        // ransac successful confidence.
	Mat inliers;
	const int flags = cv::SOLVEPNP_ITERATIVE; //SOLVEPNP_EPNP 快

	if (chessPointsBuf.rows != chessPoints3d.size())
	{
		qDebug() << "点数太少" << chessPointsBuf.rows << "<" << chessPoints3d.size();
		return false;
	}

	if(! solvePnPRansac(chessPoints3d, chessPointsBuf, cameraMatrix, distCoeffs, rvec, tvec,
		useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
		inliers, flags) )
	{
		return false;
	}

	Mat R, T;
	Rodrigues(rvec, R);      // converts Rotation Vector to Matrix
	T = tvec;       // set translation matrix
	cout << "R: " << R << endl;
	cout << "T: " << T << endl;

	//计算棋盘格上激光点的3D坐标  棋盘格外的点予以剔除
	Mat points;
	//qDebug() << "Capture points: " << laserPoints2D.rows << endl;
	laserPointTo3D(R, T, laserPoints2D, points);
	//qDebug() << "Capture " << points.rows;
	//所有点存在一起
	if (laserPoints3D.empty())
	{
		laserPoints3D = points;
	}
	else
	{
		laserPoints3D.resize(laserPoints3D.rows + points.rows);
		points.copyTo(laserPoints3D.rowRange(laserPoints3D.rows - points.rows, laserPoints3D.rows));
	}
//	cout << "Laser Points: " << endl << laserPoints3D.rowRange(laserPoints3D.rows - 10, laserPoints3D.rows) << endl << endl;
	nCapture ++;
	return true;
}

//计算棋盘格平面上激光点的3D坐标
//已知棋盘格平面和激光点在相机上的投影线。可用R*(m,n,0)' + T = z*(u v 1)'求解方程 
static int laserPointTo3D(Mat& R, Mat& T, Mat& p2d, Mat& p3d)
{
	CV_Assert(R.type() == CV_64FC1);
	Mat _R = R.clone();
	Mat Rm = R.clone();
	Mat Rn = R.clone(); 
	double chesswidth = SystemParameter::GetCalibSetting()->boardSize.width;
	double chessheight = SystemParameter::GetCalibSetting()->boardSize.height;
	double squaresize = SystemParameter::GetCalibSetting()->squareSize;

	CV_Assert(!p2d.empty());
	CV_Assert(p2d.type() == CV_64FC2);
	p3d.create(p2d.rows, 1, CV_64FC3);

	_R.at<double>(0, 2) = -T.at<double>(0, 0);
	_R.at<double>(1, 2) = -T.at<double>(1, 0);
	_R.at<double>(2, 2) = -T.at<double>(2, 0);
	double Dz = determinant(_R);

	Rm.at<double>(0, 0) = -T.at<double>(0, 0);
	Rm.at<double>(1, 0) = -T.at<double>(1, 0);
	Rm.at<double>(2, 0) = -T.at<double>(2, 0);

	Rn.at<double>(0, 1) = -T.at<double>(0, 0);
	Rn.at<double>(1, 1) = -T.at<double>(1, 0);
	Rn.at<double>(2, 1) = -T.at<double>(2, 0);

	Point3d p; 
	int count = 0;
	for (int i = 0; i < p2d.rows; i++)
	{
		double u = p2d.at<double>(i, 0);
		double v = p2d.at<double>(i, 1);

		_R.at<double>(0, 2) = -u;
		_R.at<double>(1, 2) = -v;
		_R.at<double>(2, 2) = -1;
		
		Rm.at<double>(0, 2) =  -u;
		Rm.at<double>(1, 2) =  -v;
		Rm.at<double>(2, 2) =  -1;

		Rn.at<double>(0, 2) = -u;
		Rn.at<double>(1, 2) = -v;
		Rn.at<double>(2, 2) = -1;
		
		double D = determinant(_R);
		if (abs(D) < DOUBLE_EPS)
		{
			continue;
		}

		p.z = Dz / D;
		p.x = u * p.z;
		p.y = v * p.z;
		//解出m n 裁剪掉棋盘格以外的杂点
		double Dm = determinant(Rm);
		double Dn = determinant(Rn);
		double m = Dm / D;
		double n = Dn / D; 

		if (m < -1 * squaresize || m > chesswidth*squaresize ||
			 n < -1 * squaresize || n > chessheight*squaresize)
		{
			continue;
		}
		
		p3d.ptr<double>(count)[0] = p.x;
		p3d.ptr<double>(count)[1] = p.y;
		p3d.ptr<double>(count)[2] = p.z;
		count++;
	}
	p3d.resize(count);
	return count;
}

static void drawLaserLine(Mat& diff, Mat& points)
{
	for (int i = 0; i < points.rows; i++)
	{
		double *p = points.ptr<double>(i);
		cv::drawMarker(diff, cv::Point(p[0], p[1]), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 4);
	}
	cv::imshow("Line", diff(cv::Rect(0,600,diff.cols,800)));
	cv::waitKey(1);
}


static const int ITERATE_N = 8;
static const double LOSS_RATE = 0.05;
static const double OUTLIER_THRESH = 0.05;	//m
static Mat removeOutliers(Mat &points, Mat &plane, double loss_rate, double outlier_thresh);
bool FrmProcessorLaserCalibration::_DATA::_solve()
{
	/* 伪造数据 */
	//laserPoints3D.create(6 * 1000, 1, CV_64FC3);

	//for (int i = 0; i < 6; i++)
	//{
	//	for (int j = 0; j < 1000; j++)
	//	{
	//		laserPoints3D.ptr<double>(i * 1000 + j)[0] = (j/2.0 + (rand()%10)/20.0)/1000;
	//		laserPoints3D.ptr<double>(i * 1000 + j)[1] = (rand() % 100) / 100.0*0.004+0.3;
	//		laserPoints3D.ptr<double>(i * 1000 + j)[2] = i*0.05;
	//	}
	//}

	int nPoints = laserPoints3D.rows;
	cout << "Total points: " << nPoints << endl;
	if (laserPoints3D.rows < 10)
		return false;

	/*
	保存数据
	laserPoints3D.release();
	cv::FileStorage fs("SystemParameters/laserPoints3D.yml", cv::FileStorage::READ);
	fs["Points"] >> laserPoints3D;
	fs.release();
	*/
	
	Mat _mask, _plane;
	Mat mX, mA, mB;
	mB.create(1, 1, CV_64FC1);
	mX.create(3, 1, CV_64FC1);
	_plane.create(1, 4, CV_64FC1);

	if (nPoints > 50)
	{
		//均匀选择其中50个点作为拟合初始值
		mA.create(50, 3, CV_64FC1);
		for (int i = 0 ; i < 50; i++)
		{
			double *pts = laserPoints3D.ptr<double>((int)(i/50.0*nPoints));
			mA.ptr<double>(i)[0] = pts[0];
			mA.ptr<double>(i)[1] = pts[1];
			mA.ptr<double>(i)[2] = pts[2];
//			laserPoints3D.row((int)(i / 50.0*nPoints)).copyTo(mA.row(i));
		}
		mB.at<double>(0, 0) = -1;mB.resize(mA.rows, -1);
	}
	else
	{
		//点的数量太少  所有点参与拟合
		mA.create(nPoints, 3, CV_64FC1);
		for (int i = 0; i < nPoints; i++)
		{
			double *pts = laserPoints3D.ptr<double>(i);
			mA.ptr<double>(i)[0] = pts[0];
			mA.ptr<double>(i)[1] = pts[1];
			mA.ptr<double>(i)[2] = pts[2];
		}
		mB.at<double>(0, 0) = -1; mB.resize(mA.rows, -1);
	}
	cv::solve(mA, mB, mX, cv::DECOMP_SVD);// cv::DECOMP_NORMAL);
	_plane.at<double>(0) = mX.at<double>(0, 0);
	_plane.at<double>(1) = mX.at<double>(1, 0);
	_plane.at<double>(2) = mX.at<double>(2, 0);
	_plane.at<double>(3) = 1;
	//cout << "Initial plane: " << _plane << endl << endl;

	Mat pts = laserPoints3D;
	for (int itn = 0; itn < ITERATE_N; itn++)
	{
		//计算每个坐标点到当前平面的距离 剔除距离超过最远的LOSS_RATE% 
		pts = removeOutliers(pts, _plane, LOSS_RATE, OUTLIER_THRESH);
		mA.resize(pts.rows);
		mB.at<double>(0, 0) = -1; mB.resize(mA.rows, -1);
		//剔除后的数据再进行拟合
		for (int i = 0; i < pts.rows; i++)
		{
			double *psrc = pts.ptr<double>(i);
			double *pdst = mA.ptr<double>(i);
			pdst[0] = psrc[0];
			pdst[1] = psrc[1];
			pdst[2] = psrc[2];
		}
		cv::solve(mA, mB, mX, cv::DECOMP_SVD);// cv::DECOMP_NORMAL);
		_plane.at<double>(0) = mX.at<double>(0, 0);
		_plane.at<double>(1) = mX.at<double>(1, 0);
		_plane.at<double>(2) = mX.at<double>(2, 0);
		_plane.at<double>(3) = 1;
	}
	laserPlane = _plane;
	cout << "plane: " << laserPlane << endl << endl;
//	debugDispLaserPlane(laserPoints3D, laserPlane);

	return true;
}



static Mat removeOutliers(Mat &points, Mat &plane, double loss_rate, double outlier_thresh)
{
	CV_Assert(points.rows > 0);
	Mat ret_pts;
	ret_pts.create(points.size(), points.type());

	std::vector<double> dists, dist_sample;
	dists.reserve(points.rows);
	dist_sample.reserve(200);

	double A = plane.at<double>(0);
	double B = plane.at<double>(1);
	double C = plane.at<double>(2);
	double D = plane.at<double>(3);

	double sqr_ABC = sqrt(A*A + B*B + C*C);
	CV_Assert(sqr_ABC > 0.001);

	double sample_ratio = 200.0 / points.rows;
	for (int i = 0; i < points.rows; i++)
	{
		double *pt = points.ptr<double>(i);
		dists.push_back(abs(pt[0] * A + pt[1] * B + pt[2] * C + D) / sqr_ABC);
		if (i == 0 || (double)dist_sample.size()/i < sample_ratio)
		{
			dist_sample.push_back(*dists.crbegin());
		}
	}
	CV_Assert(dists.size() == points.rows);
	std::sort(dist_sample.begin(), dist_sample.end());
	double thresh = std::min(dist_sample[(int)dist_sample.size()* (1 - loss_rate)],outlier_thresh);
	int j = 0;
	for (int i = 0; i < points.rows; i++)
	{
		if (dists[i] < thresh)
		{
			points.row(i).copyTo(ret_pts.row(j));
			j++;
		}
	}
	ret_pts.resize(j);
	return ret_pts;
}

bool FrmProcessorLaserCalibration::SolveLaserPlane()
{
	if (m_A._solve())
	{
		SystemParameter::GetCameraColorAParam()->laserPlane = m_A.laserPlane;
		SystemParameter::GetCameraColorAParam()->save(CAMERA_COLOR_A_PARAM_FILE);
	}
	else
	{
		return false;
	}

	if (m_B._solve())
	{
		SystemParameter::GetCameraColorBParam()->laserPlane = m_B.laserPlane;
		SystemParameter::GetCameraColorBParam()->save(CAMERA_COLOR_B_PARAM_FILE);
	}
	else
	{
		return false;
	}
	return true;
}

int FrmProcessorLaserCalibration::framesCaptured() const
{
	return m_A.nCapture;
}


static bool isJitteredTooMuch(Mat& a, Mat& b)
{
	return false;
}


/*
#include "PointCloudViewer.h"
#include <visp3/core/vpColor.h>
#include <visp3/core/vpPoint.h>

static void debugDispLaserPlane(Mat& laserPoints3D, Mat& plane)
{
pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
for (int i = 0; i < laserPoints3D.rows; i++)
{
pcl::PointXYZRGB point;
point.x = laserPoints3D.ptr<double>(i)[0];
point.y = laserPoints3D.ptr<double>(i)[1];
point.z = laserPoints3D.ptr<double>(i)[2];
point.r = 30;
point.g = 20;
point.b = 10;
//uint32_t rgb = (static_cast<uint32_t>(r) << 16 |static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
//point.rgb = *reinterpret_cast<float*>(&rgb);
point_cloud->push_back(point);
}
GlobalShared::slurry->pclViewer->addPointCloud(point_cloud);
if (!plane.empty())
{
pcl::ModelCoefficients plane_coeff;
plane_coeff.values.resize(4);    // We need 4 values
plane_coeff.values[0] = plane.ptr<double>()[0];
plane_coeff.values[1] = plane.ptr<double>()[1];
plane_coeff.values[2] = plane.ptr<double>()[2];
plane_coeff.values[3] = plane.ptr<double>()[3];
GlobalShared::slurry->pclViewer->viewer->addPlane(plane_coeff);
}
}
*/

