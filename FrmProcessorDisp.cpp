#include "GlobalShared.h"
#include "Slurry.h"
#include "CameraController.h"
#include "FrmProcessorDisp.h"


FrmProcessorDisp::FrmProcessorDisp()
{
	connect(this, SIGNAL(DispInMainWindowColorA(cv::Mat)), GlobalShared::slurry->ui.viewer1, SLOT(setImage(const cv::Mat)), Qt::DirectConnection);
	connect(this, SIGNAL(DispInMainWindowColorB(cv::Mat)), GlobalShared::slurry->ui.viewer2, SLOT(setImage(const cv::Mat)), Qt::DirectConnection);
}


FrmProcessorDisp::~FrmProcessorDisp()
{
}

void FrmProcessorDisp::ProcessColorA(FramePacket *fp)
{
	cv::Mat img = fp->img;
	emit DispInMainWindowColorA(img);
}

void FrmProcessorDisp::ProcessColorB(FramePacket *fp)
{
	cv::Mat img = fp->img;
	emit DispInMainWindowColorB(img);
}
//
//void FrmProcessorDisp::ProcessGrayA(FramePacket *fp)
//{
//	cv::Mat img = fp->img;
//}
