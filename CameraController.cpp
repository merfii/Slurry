#pragma once
#pragma execution_character_set("utf-8")

//Settings for using Basler GigE cameras.
#include <pylon/PylonIncludes.h>
//#include <pylon/PylonGUI.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/gige/BaslerGigEDeviceInfo.h>
#include "PylonConfigurationEventPrinter.h"

#include "Slurry.h"
#include "Logger.h"
#include "SystemParameter.h"
#include "CameraController.h"
#include "FrameProcessor.h"

typedef Pylon::CBaslerGigEInstantCamera Camera_t;
typedef Pylon::CBaslerGigEImageEventHandler ImageEventHandler_t; // Or use Camera_t::ImageEventHandler_t
typedef Pylon::CBaslerGigEGrabResultPtr GrabResultPtr_t; // Or use Camera_t::GrabResultPtr_t

using namespace Basler_GigECameraParams;
using namespace Pylon;

#define N_CAMERAS 4
bool CameraController::exit_sign = false;
std::mutex CameraController::exit_semaphore;
CameraController *CameraController::pInst;

class CameraControllerImpl final: public CameraController 
{
public:
	CameraControllerImpl();
	~CameraControllerImpl();

	virtual int  PollingDevices(bool printout) override;
	virtual bool OpenCamerasAndLoadConfiguration() override;
	virtual void StartGrabbing() override;
	virtual void StopGrabbing() override;
	virtual void CloseCameras() override;
	virtual void SetModeContinue() override;
	virtual void SetModeSoftTrig() override;
	virtual void SetModeExtTrig() override; 
	virtual void SoftTrigAll() override;
	virtual void GetFramePacket(FramePacket &fp) override;
	virtual int  GetCameraOpened() const override;
	virtual void SetPixelModeGray() override;
	virtual void SetPixelModeColor() override;
	virtual void SetAOIenable(bool enable) override;
	virtual void ResetFrameCount() override;

private:
	bool doOpenCamera(QString &idStr, Camera_t *&camera);
	void setModeContinue(Camera_t *&camptr);
	void setModeSoftTrig(Camera_t *&camptr);
	void setModeExtTrig(Camera_t *&camptr);
	void configChunk(Camera_t *&camptr);
	void configExposure(Camera_t *camptr, int expoTime);
	void execSoftTrig(Camera_t *&camptr);

	QVector<String_t> idlist;
	CBaslerGigEInstantCamera *camGrayA, *camGrayB, *camColorA, *camColorB;
	CBaslerGigEInstantCamera *cameras[N_CAMERAS];
//	CGrabResultPtr ptrGrabResults[4];
	int nCameraOpened;

};

class ImageEventHandler : public CImageEventHandler
{
public:
	ImageEventHandler(int chlIdx, CameraController *pCont)
	{
		fp.channIdx = chlIdx;
	//	fp.controller = pCont;
	}

	virtual void OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
	{
		EPixelType type = ptrGrabResult->GetPixelType();
		uchar* buffer = static_cast<uchar*>(ptrGrabResult->GetBuffer());
		
		switch (type)
		{
		case PixelType_Mono8:
			fp.img = BaslerMono8ToMat(buffer, ptrGrabResult->GetWidth(), ptrGrabResult->GetHeight());
			break;

		case PixelType_BayerBG8:
			fp.img = BaslerBG8ToMat(ptrGrabResult);
			break;

		case PixelType_BayerBG10:
			fp.img = BaslerBG10ToMat(ptrGrabResult);
			break;
		}

		size_t size, stride;
		size = ptrGrabResult->GetImageSize();
		ptrGrabResult->GetStride(stride);

		if (PayloadType_ChunkData == ptrGrabResult->GetPayloadType())
		{
			fp.timestamp = ptrGrabResult->GetTimeStamp();
			fp.framecount = ptrGrabResult->GetFrameNumber();
		}else
		{
			fp.timestamp = 0;
			fp.framecount = 0;
		}
		fp.Process();


	}

private:
	FramePacket fp;

	cv::Mat BaslerBG8ToMat(const CGrabResultPtr &src)
	{
		CPylonImage targetImage;
		// Create the converter and set parameters.
		CImageFormatConverter converter;
		converter.OutputPixelFormat = PixelType_RGB8packed;
		converter.Convert(targetImage, src);

		cv::Mat dst;
		int h, w;
		h = targetImage.GetHeight();
		w = targetImage.GetWidth();
		dst.create(h, w, CV_8UC3);
		memcpy(dst.ptr(), static_cast<uchar*>(targetImage.GetBuffer()), h*w * 3);

		/*
		Q_ASSERT(w % 2 == 0 && h % 2 == 0);
		cv::Mat dst;
		dst.create(h / 2, w / 2, CV_8UC3);
		for (int row = 0; row < h / 2; row++)
		{
			uchar *pdst = dst.ptr<uchar>(row);
			uchar *psrc1 = buf + row * 2 * w;
			uchar *psrc2 = buf + row * 2 * w + w;

			for (int col = 0; col < w / 2; col++)
			{
				int c2 = col * 2;
				int c3 = col * 3;
				OpenCV默认按照BGR构造显示 但其他软件用RGB方便
				这里仅用OpenCV做算法处理 故使用RGB
				相机BayerBG排列顺序为:
				B G B G
				G R G R
				
				// R
				pdst[c3 + 0] = psrc2[c2 + 1];
				// G
				pdst[c3 + 1] = psrc1[c2 + 1] / 2 + psrc2[c2 + 0] / 2;
				// B
				pdst[c3 + 2] = psrc1[c2 + 0];
			}
		}
		*/
		return dst;
	}

	cv::Mat BaslerBG10ToMat(const CGrabResultPtr &src)
	{
		CPylonImage targetImage;

		int64_t t1 = cv::getTickCount();

		// Converter raw data to OpenCV (Mat) format
		CImageFormatConverter converter;
		converter.OutputPixelFormat = PixelType_RGB8packed;
		converter.Convert(targetImage, src);

		int64_t t2 = cv::getTickCount();

		cv::Mat dst;
		int h, w;
		h = targetImage.GetHeight();
		w = targetImage.GetWidth();
		dst.create(h, w,  CV_8UC3);
		memcpy(dst.ptr(), static_cast<uchar*>(targetImage.GetBuffer()), h*w * 3);
		int64_t t3 = cv::getTickCount();

		//cout << "dT1: " << (t2 - t1) / cv::getTickFrequency()*1000 << "dT2: " << (t3 - t2) / cv::getTickFrequency()*1000 << endl;
		return dst;
	}


	cv::Mat BaslerMono8ToMat(uchar *buf, int w, int h)
	{
		cv::Mat raw(cv::Size(w, h), CV_8UC1, buf);
		return raw;
	}

	cv::Mat BaslerMono12ToMat(uchar *buf, int w, int h)
	{

	}

};


CameraControllerImpl::CameraControllerImpl()
{
	Pylon::PylonInitialize();
	idlist.resize(4);
	camColorA = camColorB = nullptr;
	camGrayA = camGrayB = nullptr;
	cameras[0] = cameras[1] = cameras[2] = cameras[3] = nullptr;
	nCameraOpened = 0;
	processor = nullptr;
}

CameraControllerImpl::~CameraControllerImpl()
{
	CloseCameras();
//	Pylon::PylonTerminate();
	std::cout << "Camera Deconstructed" << std::endl;
}


void CameraControllerImpl::CloseCameras()
{
	if (nCameraOpened == 0)
	{
		return;
	}

	for (int i = 0; i < N_CAMERAS; i++)
	{
		if (cameras[i] && cameras[i]->IsOpen())
		{
			cameras[i]->Close();
			cameras[i]->DestroyDevice();
			cameras[i] = nullptr;
		}
	}
	camColorA = camColorB = nullptr;
	camGrayA = camGrayB = nullptr;
	nCameraOpened = 0;
}


int  CameraControllerImpl::PollingDevices(bool printout)
{
	DeviceInfoList_t lstDevices;
	DeviceInfoList_t filter;
	
	CTlFactory& TlFactory = CTlFactory::GetInstance();

	idlist.clear();

	//filter.push_back( CDeviceInfo().SetDeviceClass());
	TlFactory.EnumerateDevices(lstDevices);

	DeviceInfoList_t::const_iterator it;
	Logger::PrintLog(QStringLiteral("正在查找相机..."));

	for (it = lstDevices.begin(); it != lstDevices.end(); ++it)
	{
		if (printout)
		{
			qDebug() << "DeviceClass:\t" << it->GetDeviceClass();
			qDebug() << "DeviceFactory:\t" << it->GetDeviceFactory();
			qDebug() << "DeviceVersion:\t" << it->GetDeviceVersion();
			qDebug() << "FullName:\t" << it->GetFullName();
			qDebug() << "FriendlyName:\t" << it->GetFriendlyName();
			qDebug() << "ModelName:\t" << it->GetModelName();
			qDebug() << "VendorName:\t" << it->GetVendorName();
			qDebug() << "SerialNumber:\t" << it->GetSerialNumber();
			qDebug() << "UserDefinedName:\t" << it->GetUserDefinedName();
		}
		idlist.push_back(it->GetSerialNumber());
	}
	return idlist.size();
}


bool CameraControllerImpl::doOpenCamera(QString &idStr, Camera_t *&ptrCam)
{
	CDeviceInfo dinfo;
	foreach(const String_t &id, idlist)
	{
		if (id == idStr)
		{
			dinfo.SetDeviceClass(BaslerGigEDeviceClass);
			dinfo.SetSerialNumber(id);
			//dinfo.SetFullName(name);
			ptrCam = new CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateDevice(dinfo));
		//	cameras[idx] = static_cast<CInstantCamera*>(camGrayA);
			nCameraOpened++;
			qDebug() << "Using device " << ptrCam->GetDeviceInfo().GetModelName() << id;
			return true;
		}
	}
	return false;
}

bool CameraControllerImpl::OpenCamerasAndLoadConfiguration()
{
	QString camstr;
	try
	{
		camstr = SystemParameter::GetInstance()->GetSettingString(QString("Settings/CameraControl/ID/") + "GrayA");
		doOpenCamera(camstr, camGrayA);
		camstr = SystemParameter::GetInstance()->GetSettingString(QString("Settings/CameraControl/ID/") + "GrayB");
		doOpenCamera(camstr, camGrayB);
		camstr = SystemParameter::GetInstance()->GetSettingString(QString("Settings/CameraControl/ID/") + "ColorA");
		doOpenCamera(camstr, camColorA);
		camstr = SystemParameter::GetInstance()->GetSettingString(QString("Settings/CameraControl/ID/") + "ColorB");
		doOpenCamera(camstr, camColorB);

		cameras[CAMERA_GRAY_A] = camGrayA;
		cameras[CAMERA_GRAY_B] = camGrayB;
		cameras[CAMERA_COLOR_A] = camColorA;
		cameras[CAMERA_COLOR_B] = camColorB;


		for (int i = 0; i < 4; i++)
		{
			if (nullptr == cameras[i])
			{
				continue;
			}
			cameras[i]->Open();
			
			cameras[i]->RegisterConfiguration( new CConfigurationEventPrinter, RegistrationMode_Append, Cleanup_Delete);
			cameras[i]->RegisterImageEventHandler(new ImageEventHandler(i, this), RegistrationMode_ReplaceAll, Cleanup_Delete);
			
			int t = SystemParameter::GetInstance()->GetSettingInt(QStringLiteral("Settings/CameraControl/ExposureTime"));
			configExposure(cameras[i],t);
			configChunk(cameras[i]);

		}
		SetPixelModeColor();
		SetAOIenable(false);

	}
	catch (const GenericException &e)
	{
		// Error handling.
		qDebug() << e.GetDescription() << endl;
		return false;
	}
	return true;
}



void CameraControllerImpl::StartGrabbing()
{
	try
	{
		for (int i = 0; i < 4; i++)
		{
			if (cameras[i] && cameras[i]->IsOpen())
			{
				cameras[i]->StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByInstantCamera);
				cameras[i]->ExecuteSoftwareTrigger();
			}
		}
	}catch (const GenericException &e)
	{
		// Error handling.
		qDebug() << e.GetDescription() << endl;
	}
	//(*it)->StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByUser);
	//camera.WaitForFrameTriggerReady( 500, TimeoutHandling_ThrowException)
}


void CameraControllerImpl::StopGrabbing()
{
	for (int i = 0; i < 4; i++)
	{
		if (cameras[i] && cameras[i]->IsOpen() && cameras[i]->IsGrabbing())
			cameras[i]->StopGrabbing();
	}
}

void CameraControllerImpl::SoftTrigAll()
{
	for (int i = 0; i < N_CAMERAS; i++)
	{
		execSoftTrig(cameras[i]);
	}
}


void CameraControllerImpl::execSoftTrig(Camera_t *&camptr)
{
	if (camptr && camptr->IsOpen())
	{
		try{ camptr->ExecuteSoftwareTrigger(); }
		catch (const GenericException &e)
		{
			// Error handling.
			qDebug() << e.GetDescription() << endl;
		}
	}
}


void CameraControllerImpl::setModeContinue(Camera_t *&camptr)
{
	camptr->TriggerSelector.SetValue(TriggerSelector_FrameStart);
	camptr->TriggerMode.SetValue(TriggerMode_Off);
}


void CameraControllerImpl::setModeSoftTrig(Camera_t *&camptr)
{
	camptr->TriggerSelector.SetValue(TriggerSelector_FrameStart);
	camptr->TriggerMode.SetValue(TriggerMode_On);
	camptr->TriggerSource.SetValue(TriggerSource_Software);
}

void CameraControllerImpl::setModeExtTrig(Camera_t *&camptr)
{
	camptr->TriggerSelector.SetValue(TriggerSelector_FrameStart);
	camptr->TriggerMode.SetValue(TriggerMode_On);
	camptr->TriggerSource.SetValue(TriggerSource_Line1);
	camptr->TriggerActivation.SetValue(TriggerActivation_RisingEdge);
	camptr->AcquisitionFrameRateEnable.SetValue(false);
}

void CameraControllerImpl::SetPixelModeGray()
{
	if (cameras[CAMERA_COLOR_A] != nullptr)
	{
		if (cameras[CAMERA_COLOR_A]->IsGrabbing())
		{
			StopGrabbing();
			cameras[CAMERA_COLOR_A]->PixelFormat.SetValue(PixelFormat_Mono8);
			StartGrabbing();
		}
		else
		{
			cameras[CAMERA_COLOR_A]->PixelFormat.SetValue(PixelFormat_Mono8);
		}
	}

	if (cameras[CAMERA_COLOR_B] != nullptr)
	{
		if (cameras[CAMERA_COLOR_B]->IsGrabbing())
		{
			StopGrabbing();
			cameras[CAMERA_COLOR_B]->PixelFormat.SetValue(PixelFormat_Mono8);
			StartGrabbing();
		}
		else
		{
			cameras[CAMERA_COLOR_B]->PixelFormat.SetValue(PixelFormat_Mono8);
		}
	}
}

void CameraControllerImpl::SetModeContinue()
{
	for (int i = 0; i < 4; i++)
	{
		try{
			if (cameras[i] != NULL)
			{
				setModeContinue(cameras[i]);
			}
		}
		catch (const GenericException &e)
		{
			qDebug() << e.GetDescription() << endl;
		}
	}
}

void CameraControllerImpl::SetModeSoftTrig()
{
	for (int i = 0; i < 4; i++)
	{
		if (cameras[i] != NULL)
		{
			setModeSoftTrig(cameras[i]);
		}
	}
}

void CameraControllerImpl::SetModeExtTrig()
{
	for (int i = 0; i < 4; i++)
	{
		if (cameras[i] != NULL)
		{
			setModeExtTrig(cameras[i]);
		}
	}
}

void CameraControllerImpl::SetPixelModeColor()
{
	if (cameras[CAMERA_COLOR_A] != nullptr)
	{
		if (cameras[CAMERA_COLOR_A]->IsGrabbing())
		{
			StopGrabbing();
			cameras[CAMERA_COLOR_A]->PixelFormat.SetValue(PixelFormat_BayerBG10);
			StartGrabbing();
		}
		else
		{
			cameras[CAMERA_COLOR_A]->PixelFormat.SetValue(PixelFormat_BayerBG10);
		}
	}

	if (cameras[CAMERA_COLOR_B] != nullptr)
	{
		if (cameras[CAMERA_COLOR_B]->IsGrabbing())
		{
			StopGrabbing();
			cameras[CAMERA_COLOR_B]->PixelFormat.SetValue(PixelFormat_BayerBG10);
			StartGrabbing();
		}
		else
		{
			cameras[CAMERA_COLOR_B]->PixelFormat.SetValue(PixelFormat_BayerBG10);
		}
	}
}


void CameraControllerImpl::SetAOIenable(bool enable)
{
	int height = SystemParameter::GetInstance()->GetSettingInt(QStringLiteral("Settings/CameraControl/Height"));
	int offsetY = SystemParameter::GetInstance()->GetSettingInt(QStringLiteral("Settings/CameraControl/OffsetY"));

	if (enable){
		if (cameras[CAMERA_COLOR_A] != nullptr)
		{
			cameras[CAMERA_COLOR_A]->Height.SetValue(height);
			cameras[CAMERA_COLOR_A]->OffsetY.SetValue(offsetY);
		}\

		if (cameras[CAMERA_COLOR_B] != nullptr)
		{
			cameras[CAMERA_COLOR_B]->Height.SetValue(height);
			cameras[CAMERA_COLOR_B]->OffsetY.SetValue(offsetY);
		}
	}
	else{
		if (cameras[CAMERA_COLOR_A] != nullptr)
		{
			cameras[CAMERA_COLOR_A]->OffsetY.SetValue(0);
			cameras[CAMERA_COLOR_A]->Height.SetValue(2048);
		}

		if (cameras[CAMERA_COLOR_B] != nullptr)
		{
			cameras[CAMERA_COLOR_B]->OffsetY.SetValue(0);
			cameras[CAMERA_COLOR_B]->Height.SetValue(2048);
		}
	}
}

void CameraControllerImpl::ResetFrameCount()
{
	for (int i = 0; i < 4; i++)
	{
		if (cameras[i] != NULL)
		{
			// Configure reset of frame counter
			cameras[i]->CounterSelector.SetValue(CounterSelector_Counter2);
			cameras[i]->CounterEventSource.SetValue(CounterEventSource_FrameStart);
			// Select reset by software
			cameras[i]->CounterResetSource.SetValue(CounterResetSource_Software);
			// execute reset by software
			cameras[i]->CounterReset.Execute();
		}
	}
}

void CameraControllerImpl::configChunk(Camera_t *&camptr)
{
	try{
		camptr->ChunkModeActive.SetValue(true);
		camptr->ChunkSelector.SetValue(ChunkSelector_Timestamp);
		camptr->ChunkEnable.SetValue(true);

		camptr->ChunkSelector.SetValue(ChunkSelector_Framecounter);
		camptr->ChunkEnable.SetValue(true);

	}
	catch (const GenericException &e)
	{
		// Error handling.
		qDebug() << e.GetDescription() << endl;
	}
	
}

void CameraControllerImpl::configExposure(Camera_t *camptr, int expoTime)
{
	// Set for the timed exposure mode
	camptr->ExposureMode.SetValue(ExposureMode_Timed);
	// Set the exposure time
	camptr->ExposureTimeAbs.SetValue(expoTime);
	
//	camptr->AcquisitionFrameRateEnable.SetValue(true);
//	camptr->AcquisitionFrameRateAbs.SetValue(10);

}

void CameraControllerImpl::GetFramePacket(FramePacket &fp)
{

/*
//已经使用线程callback的情况下不能再用该函数
RetrieveResult(500, ptrGrabResult, TimeoutHandling_Return))

#ifdef PYLON_WIN_BUILD
	// Display the grabbed image.
	Pylon::DisplayImage(1, ptrGrabResults[fp.channIdx]);
#endif
	fp.eCode = 1;
*/
}


int  CameraControllerImpl::GetCameraOpened() const
{
	return nCameraOpened;
}

CameraController::CameraController()
{

}

CameraController::~CameraController()
{

}

CameraController* CameraController::GetInstance()
{
	if (nullptr == pInst)
	{
		pInst = new CameraControllerImpl;
	}
	return pInst;
}

void CameraController::Delete()
{
	delete pInst;
	pInst = nullptr;
}

bool CameraController::isConstructed()
{
	return pInst != nullptr;
}



void CameraController::Process(FramePacket *fp)
{
//	std::<std::mutex> lck(processorLock);
	processorLock.lock();
	std::shared_ptr<FrameProcessor> p = processor;
	processorLock.unlock();

	if (p.get() != nullptr)
	{
		processor->Process(fp);
	}
}

void CameraController::SetProcessor(std::shared_ptr<FrameProcessor> p)
{
	processorLock.lock();
	processor = p;
	processorLock.unlock();
}


FrameProcessor* CameraController::GetProcessor()
{
	return processor.get();
}

void CameraController::NotifyExit()
{
	CameraController::exit_sign = true;
	CameraController::exit_semaphore.lock();
	CameraController::exit_semaphore.unlock();
}

void FramePacket::Process()
{
	if (CameraController::exit_sign)
		return;
	CameraController::exit_semaphore.lock();

	if (CameraController::exit_sign)
	{
		CameraController::exit_semaphore.unlock();
		return;
	}

	CameraController::GetInstance()->Process(this);
	CameraController::exit_semaphore.unlock();
}
