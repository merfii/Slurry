#pragma once

#include <QMutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "RobotFrame.h"
#include "vtkRobotModel.h"

class QWidget;
typedef void* PDATA_M;

class ScanVisualizer
{
public:
	ScanVisualizer();
	virtual ~ScanVisualizer();

	void inline SetVtkWindow(QWidget *vtkWin){
		mVtkWindow = vtkWin;
	}

	//初始化主界面 添加必要的坐标系 物体等
	void MainInit();
	void SetRobot(RobotFrame &frm);
	
	void ScanTaskInit();
	//以点的形式直接显示 每次最多显示Nmax个点，超过则降采样
	void ScanTaskDispPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, int Nmax = 400);
	//本函数隐含假定两次扫描之间的点云基本平行
	void ScanTaskDispSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliceA, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliceB);
	void ScanTaskClear();
	void ScanTaskEnd();	//停止扫描  去除杂散点

private:
	void addSphere(float radius, float posXYZ[3]);
	void addCoordinateSystem();
	void addGround();
	void addRobot();

	QWidget *mVtkWindow;
	vtkRobotModel mRobot;
	PDATA_M mScanTaskData;
	QMutex mutex;	//这里加锁是为了防止两个相机线程直接互相干扰
	bool mScanning; 

};

//	int RegistrationAndEstimate();
//	int MoveToCTCoordinate(double bulletXYZ[3], double tailXYZ[3]);

//Eigen::Matrix4d GetRegTransformation() const;	//Mt should be used as:   Coordinate(Robot) = Mt * Coordinate(CT)
