                        #pragma once

#include <memory>
#include <mutex>
#include <opencv2/core.hpp>

enum{CAMERA_GRAY_A, CAMERA_GRAY_B, CAMERA_COLOR_A, CAMERA_COLOR_B};

class FrameProcessor;

class FramePacket
{
public:
//	uchar *buffer;
//	uint32_t height, width;
//	size_t size = 0;
//	size_t stride = 0;
	cv::Mat img;
	uint32_t framecount = 0;
	uint64_t timestamp = 0;
	int channIdx;
	//CameraController *controller;

	void Process();
};

class CameraController
{

public:
	//单例类 由主界面的closeEvent调用Delete手动析构
	static CameraController* GetInstance();
	static bool isConstructed();
	static void Delete();

	virtual ~CameraController();
	friend class FramePacket;
	static void NotifyExit();

	virtual int  PollingDevices(bool printout) = 0;
	virtual bool OpenCamerasAndLoadConfiguration() = 0;
	virtual void StartGrabbing() = 0;
	virtual void StopGrabbing() = 0;
	virtual void CloseCameras() = 0;
	virtual void SetModeContinue() = 0;
	virtual void SetModeSoftTrig() = 0;
	virtual void SetModeExtTrig() = 0;
	virtual void SoftTrigAll() = 0;
	virtual void GetFramePacket(FramePacket &fp) = 0;
	virtual int  GetCameraOpened() const = 0;
	virtual void SetPixelModeGray() = 0;
	virtual void SetPixelModeColor() = 0;
	virtual void SetAOIenable(bool enable) = 0;
	virtual void ResetFrameCount() = 0;

	void SetProcessor(std::shared_ptr<FrameProcessor> proc);
	FrameProcessor* GetProcessor();

protected:
	std::mutex processorLock;
	static std::mutex exit_semaphore;
	static bool exit_sign;
	std::shared_ptr<FrameProcessor> processor;
	CameraController();

private:
	void Process(FramePacket *fp);
	CameraController(const CameraController&);	//Not implemented.
	void operator =(const CameraController&);	//Not implemented.
	static CameraController* pInst;

};

