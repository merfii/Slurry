#pragma execution_character_set("utf-8")

#include <opencv2\opencv.hpp>

#include "Slurry.h"
#include "SystemParameter.h"
#include "CameraController.h"
#include "FrameProcessor.h"

#include "FrmProcessorDisp.h"

cv::Mat FrameProcessor::rmapA[2];
cv::Mat FrameProcessor::rmapB[2];

FrameProcessor::FrameProcessor():
useless(false)
{
}


FrameProcessor::~FrameProcessor()
{

}

void FrameProcessor::Process(FramePacket *fp)
{
	switch (fp->channIdx)
	{
	case CAMERA_GRAY_A:
		ProcessGrayA(fp);
		break;

	case CAMERA_GRAY_B:
		ProcessGrayB(fp);
		break;

	case CAMERA_COLOR_A:
		ProcessColorA(fp);
		break;

	case CAMERA_COLOR_B:
		ProcessColorB(fp);
		break;
	}

}


void FrameProcessor::ProcessGrayA(FramePacket *fp)
{
	;
}

void FrameProcessor::ProcessGrayB(FramePacket *fp)
{
	;
}

void FrameProcessor::ProcessColorA(FramePacket *fp)
{
	;
}

void FrameProcessor::ProcessColorB(FramePacket *fp)
{
	;
}


/*
void FrameProcessor::LoadCameraParameters()
{
	CameraParameter *paramA = SystemParameter::GetCameraColorAParam();
	cv::Mat cameraMatrixA = paramA->cameraMatrix;
	cv::Mat distCoeffsA = paramA->distCoeffs;
	
	try{
		initUndistortRectifyMap(cameraMatrixA, distCoeffsA, Mat(), cameraMatrixA, paramA->imageSize, CV_32FC1, rmapA[0], rmapA[1]);
	//	initUndistortRectifyMap(cameraMatrixB, distCoeffsB, Mat(), cameraMatrixB, paramB->imageSize, CV_32FC1, rmapB[0], rmapB[1]);
	}
	catch (const cv::Exception& e)
	{
		std::cout << e.msg;
	}
}
*/

bool FrameProcessor::isUseless() const
{
	return useless;
}

void FrameProcessor::setUseless(bool noUse)
{
	useless = noUse;
}


#include <iostream>
#include <opencv2/highgui.hpp>
static void maxRow(Mat& dat, Mat& maxId, Mat& maxVal);
static Mat findCentroid(Mat& diff, Mat& initId);

//求两帧图像之间的激光点 返回Nx1 channel=2 矩阵 每行为x y坐标
Mat FrameProcessor::laserLineDetect(cv::Mat diff)
{
	if (diff.empty())
		return Mat();
	CV_Assert(diff.channels() == 3);
	Mat BGR_Planes[3], div2, maxId, maxRowVal;
	int thresh;
	split(diff, BGR_Planes);
	//addWeighted(BGR_Planes[0], 0.5, BGR_Planes[2], 0.5, 0, diff);	//饱和加 20ms
	diff = BGR_Planes[0] / 2 + BGR_Planes[2]/3; /// 2 + BGR_Planes[2] / 2;	//16ms
	/*
	cv::resize(diff, div2, cv::Size(), 1, 0.5);
	cv::reduce(diff, maxRow, 0, cv::ReduceTypes::REDUCE_MAX);	//每列计算一个最大值
	cv::Scalar avg = cv::sum(maxRow) / maxRow.size().width;
	thresh = avg(0) * 4 /5;	// x0.8
	cout << "Avg: " <<  thresh << endl;
	*/

	//为了加快速度，先纵向寻找最大点，然后精细计算灰度重心
	//int64_t t1 = cv::getTickCount();
	maxRow(diff, maxId, maxRowVal);
	//qDebug() << "maxRow dt" <<(cv::getTickCount() - t1) / cv::getTickFrequency() * 1000 << endl;;	//ms
	//cout << "Max ID: " << maxId.colRange(cv::Range(0,10)) << endl;

	return findCentroid(diff, maxId);
}


static void maxRow(Mat& dat, Mat& maxId, Mat& maxVal)	//求每一列最大值及行号
{
	CV_Assert(dat.channels() == 1 && dat.type() == CV_8UC1);
	
	maxId = Mat::ones(1, dat.cols, CV_32SC1);
	maxId = -maxId;
	maxVal = Mat::zeros(1, dat.cols, CV_8UC1);
	uint8_t *pmax = maxVal.ptr<uint8_t>(0);
	int32_t *pid = maxId.ptr<int32_t>(0);

	for (int row = 0; row < dat.rows; row++)
	{
		uint8_t *pdat = dat.ptr<uint8_t>(row);
		for (int j = 0; j < dat.cols; j++)
		{
			if (pdat[j] > pmax[j])
			{
				pmax[j] = pdat[j];
				pid[j] = row;
			}
		}
	}
}

//Gaussian kernel 系数 sigma=2
#define GAUSSIAN_SUM(r,c) \
diff.at<uint8_t>(r - 6, j) * 3 + \
diff.at<uint8_t>(r - 5, j) * 10 + \
diff.at<uint8_t>(r - 4, j) * 30 + \
diff.at<uint8_t>(r - 3, j) * 70 + \
diff.at<uint8_t>(r - 2, j) * 120 + \
diff.at<uint8_t>(r - 1, j) * 170 + \
diff.at<uint8_t>(r,     j) * 200 + \
diff.at<uint8_t>(r + 1, j) * 170 + \
diff.at<uint8_t>(r + 2, j) * 120 + \
diff.at<uint8_t>(r + 3, j) * 70 + \
diff.at<uint8_t>(r + 4, j) * 30 + \
diff.at<uint8_t>(r + 5, j) * 10 + \
diff.at<uint8_t>(r + 6, j) * 3

bool sort_comp(cv::Point3d a, cv::Point3d b) {
	return (a.z < b.z);
}

#define FIND_CENTROID_ALPHA	0.1	//删去最小的10%数据
#define FIND_CENTROID_MINIMUM 10000	//最小亮度过滤
static Mat findCentroid(Mat& diff, Mat& initId)
{
	Mat retval;
	std::vector<cv::Point3d> points;
	cv::Point3d point;
	double thresh;
	for (int j = 0; j < diff.cols; j++)
	{
		int cidx = initId.at<int>(j);
		if ( cidx < 16 || cidx > diff.rows - 16)
		{
			continue;
		}

		double maxnum = 0; int maxId =-1;
		for (int delt = -6; delt < 6; delt++)
		{
			double num = GAUSSIAN_SUM(cidx + delt, j);
			if (num > maxnum)
			{
				maxnum = num;
				maxId = cidx + delt;
			}
		}
		//高斯加权后的灰度重心 亚像素级
		double w1, w2, w3;
		w1 = GAUSSIAN_SUM(maxId - 1, j);
		w2 = maxnum;
		w3 = GAUSSIAN_SUM(maxId + 1, j);
		
		point.x = j;
		point.y = (w1*(maxId - 1) + w2*maxId + w3*(maxId + 1)) / (w1 + w2 + w3);
		point.z = w2;
		points.push_back(point);
	}

	if (points.size() < 8)
	{
		return retval;
	}
	else
	{
		std::vector<cv::Point3d> points_sort;
		points_sort.assign(points.begin(), points.end());
		std::sort(points_sort.begin(), points_sort.end(), sort_comp);
		thresh = points_sort[(int)(points_sort.size()*FIND_CENTROID_ALPHA)].z;
		thresh = thresh > FIND_CENTROID_MINIMUM ? thresh : FIND_CENTROID_MINIMUM;
	//	cout << thresh << endl;
	}

	retval.create(points.size(), 1, CV_64FC2);
	int count = 0;
	for (const cv::Point3d &p : points)
	{
		if (p.z < thresh)
			continue;
		retval.ptr<double>(count)[0] = p.x;
		retval.ptr<double>(count)[1] = p.y;
		count++;
	}
	retval.resize(count);
	return retval;
}