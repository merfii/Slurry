                                 #pragma once

#include <QObject>
#include <opencv2\core.hpp>
#include <opencv2\imgproc.hpp>
#include "SystemParameter.h"

#define N_FRAME_PROCESSORS 4

using cv::Mat;
class FramePacket;

class FrameProcessor
{
public:
	FrameProcessor();
	virtual ~FrameProcessor();

	enum ProcessorName{PROC_NONE, PROC_DISPLAY, PROC_CALIB, PROC_TEST};

	virtual void ProcessGrayA(FramePacket *fp);
	virtual void ProcessGrayB(FramePacket *fp);
	virtual void ProcessColorA(FramePacket *fp);
	virtual void ProcessColorB(FramePacket *fp);
	
	virtual void Process(FramePacket *fp);

	bool isUseless() const; 
	void setUseless(bool noUse);	//Already not used as camera frame callback, delete

	static Mat laserLineDetect(Mat diff);

protected:
	
	static Mat UndistortA(Mat &raw)
	{
		Mat dst;
		cv::remap(raw, dst, rmapA[0], rmapA[1], cv::INTER_LINEAR);
		return dst;
	}

private:
	//static QHash<QString, Algorithm*> algorithms;
	static cv::Mat rmapA[2], rmapB[2];
	bool useless;

};

template <typename T>
static void pointsMultiply(std::vector<T>& points, float factor)
{
	for (std::vector<T>::iterator it = points.begin();
		it != points.end(); it++)
	{
		it->x *= factor;
		it->y *= factor;
	}
}

template <typename T>
static void generateChessboardCorners(cv::Size boardSize, float squareSize, std::vector<T>& corners)
{
	corners.clear();
	//	case Settings::CHESSBOARD:
	//	case Settings::CIRCLES_GRID:
	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			corners.push_back(T(j*squareSize, i*squareSize, 0));
}