#pragma once

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <QObject>
#include <opencv2/core.hpp>

#include <QVTKWidget.h>

class QTimer;
class QMutex;
class vpPoint;
class vpColor;

using cv::Mat;

class PointCloudViewer : public QObject
{
	Q_OBJECT

public:
	PointCloudViewer(QObject *vtkWidget_);
	~PointCloudViewer();

	void addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);
	void clear();
	void close();

	pcl::visualization::PCLVisualizer *viewer;

private slots:
	void uiSpin();

private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud;
	QTimer *timer;
	QMutex *mutex;
	QVTKWidget *vtkWidget;

};

