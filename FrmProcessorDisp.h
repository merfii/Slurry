#pragma once

#include "FrameProcessor.h"


class FrmProcessorDisp :public QObject, public FrameProcessor 
{
	Q_OBJECT
public:
	FrmProcessorDisp();
	~FrmProcessorDisp();

	void ProcessColorA(FramePacket *fp);
	void ProcessColorB(FramePacket *fp);

	//void ProcessGrayA(FramePacket *fp);
	//void Process(FramePacket *fp);

	//利用signal slot的特性 当目标控件销毁时自动断开连接
signals:
	void DispInMainWindowColorA(cv::Mat img);
	void DispInMainWindowColorB(cv::Mat img);

private:

};

