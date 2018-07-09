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
	void ScanMainInit();
	void SetRobot(RobotFrame &frm);
	
	void ScanTaskInit();
	//调用该函数添入一次扫描的数据 每10次累积 连接三角面可视化 eyenorm给出法线方向 由物体表面指向扫描头
	void ScanTaskShotPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::Vector3f eyeNorm);	//单位m
	void ScanTaskShotPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);
	void ScanTaskDisplayPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);	//以点的形式直接显示 速度慢 仅供Debug使用
	void ScanTaskClear();
	void ScanTaskEnd();	//停止扫描  可能进行一些数据的后处理
	void ScanSaveData();
	void ScanLoadData(QWidget *window);

	int RegistrationAndEstimate();
	int MoveToCTCoordinate(double bulletXYZ[3], double tailXYZ[3]);

	Eigen::Matrix4d GetRegTransformation() const;	//Mt should be used as:   Coordinate(Robot) = Mt * Coordinate(CT)

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

