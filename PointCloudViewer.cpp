#include <QTimer>
#include <QMutex>
#include <QMutexLocker>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Slurry.h"

#include "PointCloudViewer.h"

static unsigned int text_id = 0;
static void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
	void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer*> (viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
	{
		std::cout << "r was pressed" << std::endl;
		// removing all text
		//char str[512];
		//for (unsigned int i = 0; i < text_id; ++i)
		//{
		//	sprintf(str, "text#%03d", i);
		//	viewer->removeShape(str);
		//}
		//text_id = 0;
	}
}

static void mouseEventOccurred(const pcl::visualization::MouseEvent &event,
	void* viewer_void)
{
	//鼠标似乎有一些线程安全问题 暂时不用
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer*> (viewer_void);
	if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
		event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		std::cout << "L mouse released at (" << event.getX() << ", " << event.getY() << ")" << std::endl;
		//char str[512];
		//sprintf(str, "text#%03d", text_id++);
		//viewer->addText("clicked here", event.getX(), event.getY(), str);
	}
}

PointCloudViewer::PointCloudViewer(QObject *vtkWidget_)
{

	vtkWidget = dynamic_cast<QVTKWidget*>(vtkWidget_);
	
	point_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
	viewer->setBackgroundColor(0.94, 0.96, 0.96);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud, rgb, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	pcl::visualization::Camera camera;
	viewer->getCameraParameters(camera);
	//camera.focal[0] = 40; camera.focal[1] = 10; camera.focal[2] = 40;
	camera.pos[0] = 40;	camera.pos[1] = 40; camera.pos[2] = 60;
	camera.view[0] = 0; camera.view[1] = 0; camera.view[2] = 1;
	//cout << "near " << camera.clip[0] << " far " << camera.clip[1] << endl;
	viewer->setCameraParameters(camera);

//	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer);
//	viewer->registerMouseCallback(mouseEventOccurred, (void*)viewer);

//	viewer->addSphere(pcl::PointXYZ(0,0,0), 0.5, 0.5, 0.5, 0.0, "sphere");
	pcl::ModelCoefficients plane_coeff;
	plane_coeff.values.resize(4);    // We need 4 values
	plane_coeff.values[0] = 0;
	plane_coeff.values[1] = 0;
	plane_coeff.values[2] = 1;
	plane_coeff.values[3] = 0;

//	viewer->addPlane(plane_coeff,"p");

	mutex = new QMutex;

	timer = new QTimer;
	connect(timer, SIGNAL(timeout()), this, SLOT(uiSpin()));
	timer->start(200);
}

PointCloudViewer::~PointCloudViewer()
{
	mutex->lock();
	mutex->unlock();
	delete timer;
	delete viewer;
	delete mutex;
}


void PointCloudViewer::addPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
	mutex->lock();
	char cloudid[512];
	sprintf(cloudid, "cloud#%03d", text_id++);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(pointCloud, rgb, cloudid);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudid);
	/*pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud);
	viewer->updatePointCloud<pcl::PointXYZRGB>(point_cloud, rgb, "cloud");*/
	mutex->unlock();
}

void PointCloudViewer::clear()
{
	QMutexLocker locker(mutex);
	viewer->removeAllPointClouds();
}


void PointCloudViewer::uiSpin()
{
	QMutexLocker locker(mutex);
	if (!viewer->wasStopped())
	{
		viewer->spinOnce(1);
	}
	else
	{
		viewer->close();
	}
}


void PointCloudViewer::close()
{
	viewer->close();
}

/*

Shapes
{

viewer->addLine<pcl::PointXYZRGB>(cloud->points[0],	cloud->points[cloud->size() - 1], "line");
viewer->addSphere(cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");


pcl::ModelCoefficients coeffs;
coeffs.values.push_back(0.0);
coeffs.values.push_back(0.0);
coeffs.values.push_back(1.0);
coeffs.values.push_back(0.0);
viewer->addPlane(coeffs, "plane");

coeffs.values.clear();
coeffs.values.push_back(0.3);
coeffs.values.push_back(0.3);
coeffs.values.push_back(0.0);
coeffs.values.push_back(0.0);
coeffs.values.push_back(1.0);
coeffs.values.push_back(0.0);
coeffs.values.push_back(5.0);
viewer->addCone(coeffs, "cone");
}
*/

/*
void CreatePoints()
{
	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;


	// ----------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.05-----
	// ----------------------------------------------------------------
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(point_cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.05);
	ne.compute(*cloud_normals1);

	// ---------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.1-----
	// ---------------------------------------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.1);
	ne.compute(*cloud_normals2);
}

*/
