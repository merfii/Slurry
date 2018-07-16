#include "TestPointsGenerator.h"
#include "Slurry.h"

void TestPointsGenerator::run()
{
	count = 0;
	for (int i = 0; i < 1000; i++)
	{
		creatPoints();
		msleep(200);
		if (should_exit)
			return;
	}
}


void TestPointsGenerator::creatPoints()
{
	static double height;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB point;
	std::normal_distribution<> n(0, 1); //正态分布，大部分生成的随机数落在-2~-2之间 
	height = 80 * sin(count / 55.0) + 100 * cos(count / 37.0) + 80 * sin(count / 27.0 + n(rengine)*0.1 - 0.1);

	for (int i = -500; i < 500; i++)
	{
		point.x = (i + n(rengine)*2) / 1000.0;
		point.y = (count * 2 + 0.5*n(rengine)) / 800.0;
		point.z = (height + n(rengine) + 70) / 1500.0;
		point.r = (count * 2) % 255;
		point.g = (count * 3) % 255;
		point.b = (count * 4) % 255;
		points->push_back(point);
	}
	count++;
	GlobalShared::slurry->scanShotFrame(points);
}

void TestPointsGenerator::closeGenerator()
{
	should_exit = true;
	terminate();
	wait();
}

void Slurry::createPointsGenerator()
{
	generator = new TestPointsGenerator;
	scanVisualizer->ScanTaskInit();
	generator->should_exit = false;
	generator->start();
}

