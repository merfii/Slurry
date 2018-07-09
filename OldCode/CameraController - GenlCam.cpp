#pragma execution_character_set("utf-8")


#include "GlobalShared.h"
#include "CameraController.h"


//Settings for using Basler GigE cameras.
#include <pylon/PylonIncludes.h>
//#include <pylon/PylonGUI.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/gige/BaslerGigEDeviceInfo.h>
typedef Pylon::CBaslerGigEInstantCamera Camera_t;
typedef Pylon::CBaslerGigEImageEventHandler ImageEventHandler_t; // Or use Camera_t::ImageEventHandler_t
typedef Pylon::CBaslerGigEGrabResultPtr GrabResultPtr_t; // Or use Camera_t::GrabResultPtr_t
using namespace Basler_GigECameraParams;

using namespace Pylon;
//using namespace GenApi;

class CameraControllerImpl final: public CameraController 
{
public:

	~CameraControllerImpl();
	static CameraController*  getInstance();

	virtual int  PollingDevices(bool printout) override;
	virtual bool LoadConfiguration() override;
	virtual void StartGrabbing() override;
	virtual void CloseCameras() override;
	virtual void SetModeContinue() override;
	virtual void SetModeSoftTrig() override;
	virtual void ExecuteSoftTrig() override;
	virtual void GetFramePacket(FramePacket &fp) override;
	virtual int  GetCameraOpened() const override;

protected:
	CameraControllerImpl();
	
private:

	void doLoadGigECamera();
	void doLoadUsbCamera();

	void configChunk();
	void configExposure();
	
	QVector<String_t> idlist;
	CBaslerGigEInstantCamera *camGrayA, *camGrayB, *camColorA, *camColorB;
//	CInstantCamera *cameras[4];
//	CGrabResultPtr ptrGrabResults[4];
	int nCameraOpened;
};


class ImageEventHandler : public CImageEventHandler
{
public:
	ImageEventHandler(int chlIdx)
	{
		m_channelIdx = chlIdx;
	}
	
	virtual void OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
	{
		CameraController::FramePacket fp;
		fp.buffer = static_cast<uchar*>(ptrGrabResult->GetBuffer());
		fp.height = ptrGrabResult->GetHeight();
		fp.width = ptrGrabResult->GetWidth();
		fp.size = ptrGrabResult->GetImageSize();
		ptrGrabResult->GetStride(fp.stride);
		
		if (PayloadType_ChunkData == ptrGrabResult->GetPayloadType())
		{
			fp.timestamp = ptrGrabResult->GetTimeStamp();
			fp.framecount = ptrGrabResult->GetFrameNumber();
		}else
		{
			fp.timestamp = 0;
			fp.framecount = 0;
		}
		fp.channIdx = m_channelIdx;
/**************************************************/
		//跨线程?
		emit process(fp);
//		GlobalShared::algorithm->process(fp);

/**************************************************/
	}

private:
	int m_channelIdx;
};


CameraControllerImpl::CameraControllerImpl()
{
	PylonInitialize();
	idlist.resize(4);
	camColorA = camColorB = nullptr;
	camGrayA = camGrayB = nullptr;
	nCameraOpened = 0;
}

CameraControllerImpl::~CameraControllerImpl()
{
	CloseCameras();
	PylonTerminate();
}


void CameraControllerImpl::CloseCameras()
{
	if (nCameraOpened == 0)
		return;

	if (camColorA&& camColorA->IsOpen())
	{
		camColorA->Close();
		camColorA->DestroyDevice();
	}
	if (camColorB&& camColorB->IsOpen())
	{
		camColorB->Close();
		camColorB->DestroyDevice();
	}

	if (camGrayA&& camGrayA->IsOpen())
	{
		camGrayA->Close();
		camGrayA->DestroyDevice();
	}
	if (camGrayB&& camGrayB->IsOpen())
	{
		camGrayB->Close();
		camGrayB->DestroyDevice();
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

	CloseCameras();
	idlist.clear();

	//filter.push_back( CDeviceInfo().SetDeviceClass());
	TlFactory.EnumerateDevices(lstDevices);

	DeviceInfoList_t::const_iterator it;
	GlobalShared::PrintLog(QStringLiteral("正在查找相机..."));
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
			qDebug() << endl;
		}
		idlist.push_back(it->GetSerialNumber());
	}
	GlobalShared::PrintLog(QString("已找到相机 %1 个").arg(idlist.size()));
	return idlist.size();
}


void CameraControllerImpl::doLoadGigECamera()
{
	QString camId;
	CDeviceInfo dinfo;
	int idx;
	camId = GlobalShared::Setting::GetString(QString("CameraControl/ID/") + "GrayA");
	idx = 0;
	foreach(const String_t &id, idlist)
	{
		if (id == camId)
		{
			//dinfo.SetDeviceClass(BaslerGigEDeviceClass);
			dinfo.SetSerialNumber(id);
			//dinfo.SetFullName(name);
			camGrayA = new CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateDevice(dinfo));
			cameras[idx] = static_cast<CInstantCamera*>(camGrayA);
			cameraOpened++;
			qDebug() << "Using device " << cameras[idx]->GetDeviceInfo().GetModelName() << id;
			break;
		}
	}

	camId = GlobalShared::Setting::GetString(QString("CameraControl/ID/") + "GrayB");
	idx = 1;
	foreach(const String_t &id, idlist)
	{
		if (id == camId)
		{
			//dinfo.SetDeviceClass(BaslerGigEDeviceClass);
			dinfo.SetSerialNumber(id);
			//dinfo.SetFullName(name);
			camGrayB = new CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateDevice(dinfo));
			cameras[idx] = static_cast<CInstantCamera*>(camGrayB);
			cameraOpened++;
			qDebug() << "Using device " << cameras[idx]->GetDeviceInfo().GetModelName() << id;
			break;
		}
	}

	//camera.RegisterConfiguration( new CConfigurationEventPrinter, RegistrationMode_Append, Cleanup_Delete); // Camera use.

}


void CameraControllerImpl::doLoadUsbCamera()
{
	QString camId;
	CDeviceInfo dinfo;
	int idx;
	camId = GlobalShared::Setting::GetString(QString("CameraControl/ID/") + "ColorA");
	idx = 2;
	foreach(const String_t &id, idlist)
	{
		if (id == camId)
		{
			//dinfo.SetDeviceClass(BaslerGigEDeviceClass);
			dinfo.SetSerialNumber(id);
			//dinfo.SetFullName(name);
			camColorA = new CBaslerUsbInstantCamera(CTlFactory::GetInstance().CreateDevice(dinfo));
			cameras[idx] = static_cast<CInstantCamera*>(camColorA);
			cameraOpened++;
			qDebug() << "Using device " << cameras[idx]->GetDeviceInfo().GetModelName() << id;
			break;
		}
	}

	camId = GlobalShared::Setting::GetString(QString("CameraControl/ID/") + "ColorB");
	idx = 3;
	foreach(const String_t &id, idlist)
	{
		if (id == camId)
		{
			//dinfo.SetDeviceClass(BaslerGigEDeviceClass);
			dinfo.SetSerialNumber(id);
			//dinfo.SetUserDefinedName("GrayA");
			camColorB = new CBaslerUsbInstantCamera(CTlFactory::GetInstance().CreateDevice(dinfo));
			cameras[idx] = static_cast<CInstantCamera*>(camColorB);
			cameraOpened++;
			qDebug() << "Using device " << cameras[idx]->GetDeviceInfo().GetModelName() << id;
			break;
		}
	}
}

bool CameraControllerImpl::LoadConfiguration()
{
	try
	{
		doLoadGigECamera();
		doLoadUsbCamera();
	
		for (int i = 0; i < 4; i++)
		{
			if (nullptr == cameras[i])
				continue;
			cameras[i]->Open();
			cameras[i]->RegisterImageEventHandler(new ImageEventHandler(i), RegistrationMode_Append, Cleanup_Delete);
		}
		SetModeSoftTrig();
		configChunk();
		configExposure();
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

/*
void CameraControllerImpl::StopGrabbing()
{
	for (int i = 0; i < 4; i++)
	{
		if (cameras[i] && cameras[i]->IsOpen() && cameras[i]->IsGrabbing())
			cameras[i]->StopGrabbing();
	}
}
*/

void CameraControllerImpl::SetModeContinue()
{
	for (int i = 0; i < 4; i++)
	{
		if (cameras[i] && cameras[i]->IsOpen())
		{
			INodeMap& nodemap = cameras[i]->GetNodeMap();
			CEnumerationPtr triggerSelector(nodemap.GetNode("TriggerSelector"));
			CEnumerationPtr triggerMode(nodemap.GetNode("TriggerMode"));
			triggerSelector->FromString("FrameStart");
			triggerMode->FromString("Off");
		}
	}
}


void CameraControllerImpl::SetModeSoftTrig()
{
	for (int i = 0; i < 4; i++)
	{
		if (cameras[i] && cameras[i]->IsOpen())
		{
			INodeMap& nodemap = cameras[i]->GetNodeMap();
			CEnumerationPtr triggerSelector(nodemap.GetNode("TriggerSelector"));
			CEnumerationPtr triggerMode(nodemap.GetNode("TriggerMode"));
			CEnumerationPtr triggerSource(nodemap.GetNode("TriggerSource"));
			triggerSelector->FromString("FrameStart");
			triggerMode->FromString("On");
			triggerSource->FromString("Software");
		}
	}
}

void CameraControllerImpl::ExecuteSoftTrig()
{
	try
	{
		for (int i = 0; i < 4; i++)
		{
			if (cameras[i] && cameras[i]->IsOpen())
			{
				cameras[i]->ExecuteSoftwareTrigger();
			}
		}
	}catch (const GenericException &e)
	{
		// Error handling.
		qDebug() << e.GetDescription() << endl;
	}
}

void CameraControllerImpl::configChunk()
{
	for (int i = 0; i < 4; i++)
	{
		if (cameras[i] && cameras[i]->IsOpen())
		{
			INodeMap& nodemap = cameras[i]->GetNodeMap();
			try{
				CBooleanPtr chunkModeActive(nodemap.GetNode("ChunkModeActive"));
				if (IsWritable(chunkModeActive))
				{
					chunkModeActive->SetValue(true);
				}
				CEnumerationPtr chunkSelector(nodemap.GetNode("ChunkSelector"));
				if (IsWritable(chunkSelector))
				{
					chunkSelector->FromString("Timestamp");
				}
				CBooleanPtr chunkEnable(nodemap.GetNode("ChunkEnable"));
				if (IsWritable(chunkEnable))
				{
					chunkEnable->SetValue(true);
				}

				if (IsWritable(chunkSelector))
				{
					chunkSelector->FromString("Framecounter");
				}
				if (IsWritable(chunkEnable))
				{
					chunkEnable->SetValue(true);
				}

			}
			catch (const GenericException &e)
			{
				// Error handling.
				qDebug() << e.GetDescription() << endl;
			}
		}
	}
	
	/*
	if (GenApi::IsWritable(camera->ChunkModeActive))
	{
		camera->ChunkModeActive.SetValue(true);

		camera->ChunkSelector.SetValue(ChunkSelector_Timestamp);
		camera->ChunkEnable.SetValue(true);

		camera->ChunkSelector.SetValue(ChunkSelector_Framecounter);
		camera->ChunkEnable.SetValue(true);
	}
	*/
}


static void doConfigExposure(CInstantCamera *camera, int expoTime)
{
	if (camera && camera->IsOpen())
	{
		INodeMap& nodemap = camera->GetNodeMap();
		CEnumerationPtr exposureMode(nodemap.GetNode("ExposureMode"));
		exposureMode->FromString("Timed");
		CFloatPtr exposureTime(nodemap.GetNode("ExposureTime"));
		if(IsWritable(exposureTime))
			exposureTime->SetValue(expoTime);

		CFloatPtr exposureTimeAbs(nodemap.GetNode("ExposureTimeAbs"));
		if (IsWritable(exposureTimeAbs))
			exposureTimeAbs->SetValue(expoTime);
	}

	/*

		// Set for the timed exposure mode
		camera->ExposureMode.SetValue(ExposureMode_Timed);
		// Set the exposure time
		camera->ExposureTimeAbs.SetValue(30000);
	*/
}

void CameraControllerImpl::configExposure()
{
	int t = GlobalShared::Setting::GetInt(QStringLiteral("CameraControl/ExposureTime"));
	doConfigExposure(cameras[0], t);
	doConfigExposure(cameras[1], t);
	doConfigExposure(cameras[2], t);
	doConfigExposure(cameras[3], t);
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
	return cameraOpened;
}

CameraController* CameraControllerImpl::getInstance()
{
	static CameraControllerImpl m_pinstance;
	return &m_pinstance;
}

CameraController::CameraController()
{

}

CameraController::~CameraController()
{

}

CameraController* CameraController::GetInstance()
{
	return dynamic_cast<CameraController*>(CameraControllerImpl::getInstance());
}
