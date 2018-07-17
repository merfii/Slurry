#include <QMutexLocker>


#include <vtkSmartPointer.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkAxes.h>
#include <vtkPolyData.h>
#include <vtkDoubleArray.h>
#include <vtkCell.h>
#include <vtkPolyDataMapper.h>
#include <vtkTubeFilter.h>
#include <vtkActor.h>
#include <vtkLODActor.h>
#include <vtkAssembly.h>
#include <vtkSphereSource.h>
#include <vtkPlaneSource.h>
#include <vtkDataSetMapper.h>
#include <vtkTransformPolyDataFilter.h>

#include <pcl/common/geometry.h>
#include <pcl/common/centroid.h>
#include <pcl/octree/octree.h>
#include <pcl/features/feature.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//#include <pcl/visualization/pcl_visualizer.h>


#include "VtkWindow.h"
#include "Utils.h"
#include "ScanVisualizer.h"

using std::cout;
using std::endl;

#define POINT_NEIGHBOR_DIST 0.008
#define QWIDGET2VTKWINDOW(widget) static_cast<VtkWindow*>(widget)
#define PDATA2SCANTASKDATA(dat) static_cast<ScanTaskData*>(dat)
//#define SQUARED_DIST(VecA, VecB) ((VecA[0]-VecB[0])*(VecA[0]-VecB[0])+(VecA[1]-VecB[1])*(VecA[1]-VecB[1])+(VecA[2]-VecB[2])*(VecA[2]-VecB[2]))
//pcl::geometry::distance(p1, p2)


class ScanTaskData{
public:
	ScanTaskData() :
		shotCount(0)
	{
	}

	~ScanTaskData()
	{

	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rawCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr deciCloud;

//	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;
	int shotCount;
};



ScanVisualizer::ScanVisualizer()
{
	mScanTaskData = nullptr;
	mScanning = false;
}


ScanVisualizer::~ScanVisualizer()
{

}


void ScanVisualizer::MainInit()
{
	if (dynamic_cast<VtkWindow*>(mVtkWindow) == nullptr)
	{
		return;
	}
	addGround();
	addCoordinateSystem();
	addRobot();
//	float xyz[3] = { 0, 0, 1.5 };
//	addSphere(0.01, xyz);
}


void ScanVisualizer::ScanTaskInit()
{
	QMutexLocker locker(&mutex);
	mScanning = true;
	delete PDATA2SCANTASKDATA(mScanTaskData);
	mScanTaskData = new ScanTaskData;
	if (mVtkWindow)
		QWIDGET2VTKWINDOW(mVtkWindow)->RemoveActorTS(QStringLiteral("Preview"));
}

static void createActorFromVTKDataSet(const vtkSmartPointer<vtkDataSet> &data, vtkActor *actor);
void ScanVisualizer::ScanTaskDispPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, int Nmax)
{
	QMutexLocker locker(&mutex);

	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkUnsignedCharArray> polyColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	polyColors->SetNumberOfComponents(3);
	polyColors->SetName("Colors");

	vtkIdType nr_points = pointCloud->size()>Nmax ? Nmax : pointCloud->size();
	// Create the supporting structures
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	vertices->Allocate(vertices->EstimateSize(1, nr_points));
	vertices->InsertNextCell(nr_points);
	unsigned char color[3];

	//对数据进行降采样
	float idxA = 0, idxB = 0;
	float deltaB = Nmax / (pointCloud->size()+0.5);	//这个0.5是为了避免浮点误差造成的不确定性

	for (auto it = pointCloud->begin(); it != pointCloud->end(); it++)
	{
		idxB += deltaB;
		if (idxA > idxB){
			continue;
		}
		vertices->InsertCellPoint(points->InsertNextPoint(it->x, it->y, it->z));
		color[0] = it->r; color[1] = it->g; color[2] = it->b;
		polyColors->InsertNextTypedTuple(color);
		idxA++;
	}
	polydata->SetPoints(points);
	polydata->GetPointData()->SetScalars(polyColors);
	polydata->SetVerts(vertices);
	//	cout << polydata->GetNumberOfPoints() << endl;
	//	cout << polydata->GetNumberOfCells() << endl;
	//	cout << "debug:" << debug << endl;

	// Create an Actor
	vtkActor* actor = vtkActor::New();
	createActorFromVTKDataSet(polydata, actor);
	//actor->SetPosition(0.5, -0.5, 0);
	QWIDGET2VTKWINDOW(mVtkWindow)->AddActorTS(actor, QStringLiteral("Points"));
}


static void createActorFromVTKDataSet(const vtkSmartPointer<vtkDataSet> &data, vtkActor *actor)
{
	bool use_scalars = true;
	vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
	mapper->SetInputData(data);

	if (use_scalars)
	{
		vtkSmartPointer<vtkDataArray> scalars = data->GetPointData()->GetScalars();
		double minmax[2];
		if (scalars)
		{
			scalars->GetRange(minmax);
			mapper->SetScalarRange(minmax);
			mapper->SetScalarModeToUsePointData();
			mapper->SetInterpolateScalarsBeforeMapping(false);	//for only vertices, set false
			mapper->ScalarVisibilityOn();
		}
	}
	mapper->ImmediateModeRenderingOff();
	actor->GetProperty()->SetInterpolationToFlat();

	/// FIXME disabling backface culling due to known VTK bug: vtkTextActors are not
	/// shown when there is a vtkActor with backface culling on present in the scene
	/// Please see VTK bug tracker for more details: http://www.vtk.org/Bug/view.php?id=12588
	//actor->GetProperty ()->BackfaceCullingOn ();

	actor->SetMapper(mapper);
}

static void estimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliceA, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliceB, float normals[3][3]);
static void sortPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliceA, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliceB);
static inline float _dist_point(const pcl::PointXYZRGB &A, const pcl::PointXYZRGB &B);
#define _INSERT_POINT(Points, Norms, Colors, id, it) \
	id = Points->InsertNextPoint(it->x, it->y, it->z);	\
	Norms->InsertNextTuple(vtkPtNorm);	\
	vtkPtColor[0] = it->r; vtkPtColor[1] = it->g; vtkPtColor[2] = it->b;	\
	Colors->InsertNextTypedTuple(vtkPtColor);

void ScanVisualizer::ScanTaskDispSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliceA, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliceB)
{
	//为了不阻塞后续过程 应考虑使用线程池
	QMutexLocker locker(&mutex);
	sortPoints(sliceA, sliceB);

	//初始化vtk对象
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> polyPoints = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkUnsignedCharArray> polyColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	polyColors->SetNumberOfComponents(3);
	polyColors->SetName("Colors");
	vtkSmartPointer<vtkDoubleArray> polyNorms = vtkSmartPointer<vtkDoubleArray>::New();
	polyNorms->SetNumberOfComponents(3); //3d normals (x,y,z)
	//ptsNorm->SetNumberOfTuples(polydata->GetNumberOfPoints());
	vtkSmartPointer<vtkCellArray> polyTriangles = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

	//vertices->InsertNextCell(nr_points);	//这里要先知道点的数量  不知道咋弄

	vtkIdType vtkPtId[3];
	unsigned char vtkPtColor[3];
	double vtkPtNorm[3] = { 0, 0, 1 };

	auto itA = sliceA->begin();
	auto itB = sliceB->begin();

	int prevIdA = -1, prevIdB = -1;
	int prevValid = 0;
	pcl::PointXYZRGB prevA, prevB;
	while (itA != sliceA->end() && itB != sliceB->end())
	{
		if (prevIdA < 0 && prevIdB < 0){
			prevA = *itA;
			prevB = *itB;
			_INSERT_POINT(polyPoints, polyNorms, polyColors, prevIdA, itA);
			_INSERT_POINT(polyPoints, polyNorms, polyColors, prevIdB, itB);
			itA++;
			itB++;
			prevValid = 1;
		}
		else if (prevIdA < 0){
			if (_dist_point(*itA, prevB) < POINT_NEIGHBOR_DIST*POINT_NEIGHBOR_DIST){
				prevA = *itA;
				_INSERT_POINT(polyPoints, polyNorms, polyColors, prevIdA, itA);
				itA++;
				prevValid = 1;
			}
			else if (_dist_point(*itA, prevB) > _dist_point(*itA, *itB)){
				prevB = *itB;
				_INSERT_POINT(polyPoints, polyNorms, polyColors, prevIdB, itB);
				itB++;
				prevValid = 1;
			}
			else{
				itA++;
				prevValid = 0;
			}
		}
		else if (prevIdB < 0){
			if (_dist_point(*itB, prevA) < POINT_NEIGHBOR_DIST*POINT_NEIGHBOR_DIST){
				prevB = *itB;
				_INSERT_POINT(polyPoints, polyNorms, polyColors, prevIdB, itB);
				itB++;
				prevValid = 1;
			}
			else if (_dist_point(*itB, prevA) > _dist_point(*itB, *itA)){
				prevA = *itA;
				_INSERT_POINT(polyPoints, polyNorms, polyColors, prevIdA, itA);
				itA++; prevValid = 1;
			}
			else{
				itB++;
				prevValid = 0;
			}
		}
		else // prevIdA >0 && prevIdB >0
		{
			if (_dist_point(*itA, prevB) < _dist_point(*itB, prevA)){
				if (_dist_point(*itA, prevB) < POINT_NEIGHBOR_DIST*POINT_NEIGHBOR_DIST &&
					_dist_point(*itA, prevA) < POINT_NEIGHBOR_DIST*POINT_NEIGHBOR_DIST)
				{
					vtkPtId[0] = prevIdA;
					vtkPtId[1] = prevIdB;
					_INSERT_POINT(polyPoints, polyNorms, polyColors, prevIdA, itA);
					vtkPtId[2] = prevIdA;
					polyTriangles->InsertNextCell(3, vtkPtId);
					prevA = *itA;
					itA++;
					prevValid = 1;
				}
				else{
					if (prevValid == 0){
						prevIdA = prevIdB = -1;
					}
					else{
						prevValid = 0;
						itA++;
					}
				}
			}
			else{
				if (_dist_point(*itB, prevB) < POINT_NEIGHBOR_DIST*POINT_NEIGHBOR_DIST &&
					_dist_point(*itB, prevA) < POINT_NEIGHBOR_DIST*POINT_NEIGHBOR_DIST)
				{
					vtkPtId[0] = prevIdA;
					vtkPtId[1] = prevIdB;
					_INSERT_POINT(polyPoints, polyNorms, polyColors, prevIdB, itB);
					vtkPtId[2] = prevIdB;
					polyTriangles->InsertNextCell(3, vtkPtId);
					prevB = *itB;
					itB++;
				}
				else{
					if (prevValid == 0){
						prevIdA = prevIdB = -1;
					}
					else{
						prevValid = 0;
						itB++;
					}
				}
			}
		}
	}

	polyData->SetPoints(polyPoints);
	polyData->GetPointData()->SetScalars(polyColors);
	polyData->GetPointData()->SetNormals(polyNorms);
	polyData->SetPolys(polyTriangles);
	//polyData->SetVerts(vertices);
	
	vtkActor* actor = vtkActor::New();
	createActorFromVTKDataSet(polyData, actor);
	QWIDGET2VTKWINDOW(mVtkWindow)->AddActorTS(actor, QStringLiteral("Surface"));
}

// u轴与slice方向基本一致 v轴与slice垂直 指向两个slice的交错方向 w轴一般来说指向z轴正方向
static void estimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliceA, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliceB, float normals[3][3])
{
}

static void sortPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliceA, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliceB)
{
	assert(sliceA->size() > 0);
	assert(sliceB->size() > 0);

	//判断是否需要逆转顺序
	bool reverse = false;
	if (sliceA->size() < 20 || sliceB->size() < 20)
	{
		pcl::PointXYZRGB pA0 = sliceA->at(0);
		pcl::PointXYZRGB pA1 = sliceA->at(sliceA->size()-1);

		pcl::PointXYZRGB pB0 = sliceB->at(0);
		pcl::PointXYZRGB pB1 = sliceB->at(sliceB->size()-1);
		 
		if ((pA1.x - pA0.x)*(pB1.x - pB0.x) + (pA1.y - pA0.y) * (pB1.y - pB0.y) + (pA1.z - pA0.z)*(pB1.z - pB0.z) < 0)
		{
			reverse = true;
		}
	}
	else
	{
		int vote[10] = { 0 };
		float da = (float)sliceA->size() / 20;
		float db = (float)sliceB->size() / 20;

		for (int i = 0; i < 10; i++)
		{
			int idxA0 = (int)(da*i);
			int idxA1 = (int)(da*i + sliceA->size() / 2);

			int idxB0 = (int)(db*i);
			int idxB1 = (int)(db*i + sliceB->size() / 2);

			float dot = (sliceA->at(idxA1).x - sliceA->at(idxA0).x)*(sliceB->at(idxB1).x - sliceB->at(idxB0).x) +
				(sliceA->at(idxA1).y - sliceA->at(idxA0).y)*(sliceB->at(idxB1).y - sliceB->at(idxB0).y) +
				(sliceA->at(idxA1).z - sliceA->at(idxA0).z)*(sliceB->at(idxB1).z - sliceB->at(idxB0).z);
			if (dot < 0){
				vote[i] = 1;
			}
		}
		int sum = 0;
		for (int i = 0; i < 10; i++){
			sum += vote[i];
		}
		if (sum >= 5){
			reverse = true;
		}
	}

	//逆转B
	if(reverse)
		std::reverse(sliceB->begin(), sliceB->end());
}

static float _dist_point(const pcl::PointXYZRGB &A, const pcl::PointXYZRGB &B)
{
	return (A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y) + (A.z - B.z)*(A.z - B.z);
}


void ScanVisualizer::ScanTaskClear()
{
	QMutexLocker locker(&mutex);

	delete PDATA2SCANTASKDATA(mScanTaskData);
	mScanTaskData = new ScanTaskData;
	if (mVtkWindow){
		QWIDGET2VTKWINDOW(mVtkWindow)->RemoveActorTS(QStringLiteral("Preview"));
		QWIDGET2VTKWINDOW(mVtkWindow)->RemoveActorTS(QStringLiteral("Points"));
	}

}


void ScanVisualizer::ScanTaskEnd()
{
	mScanning = false;

	/*

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliner(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliner(new pcl::PointCloud<pcl::PointXYZ>);



    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);//50个临近点
    sor.setStddevMulThresh(1.0);//距离大于1倍标准方差

    sor.filter(*cloud_inliner);

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_inliner, false);

    sor.setNegative(true);
    sor.filter(*cloud_outliner);

	*/
	//点云合并 全部显示
}

void ScanVisualizer::SetRobot(RobotFrame &frm)
{
	mRobot.SetJoints(frm.J);
	mRobot.SetGuider(frm.P);
}

void ScanVisualizer::addCoordinateSystem()
{
	vtkSmartPointer<vtkAxes> axes = vtkSmartPointer<vtkAxes>::New();
	axes->SetOrigin(0, 0, 0);	//设置坐标原点
	axes->SetScaleFactor(1.0);
	axes->Update();

	vtkSmartPointer<vtkFloatArray> axes_colors = vtkSmartPointer<vtkFloatArray>::New();
	axes_colors->Allocate(6);
	axes_colors->InsertNextValue(0.0);
	axes_colors->InsertNextValue(0.0);
	axes_colors->InsertNextValue(0.5);
	axes_colors->InsertNextValue(0.5);
	axes_colors->InsertNextValue(1.0);
	axes_colors->InsertNextValue(1.0);

	vtkSmartPointer<vtkPolyData> axes_data = axes->GetOutput();
	axes_data->GetPointData()->SetScalars(axes_colors);

	vtkSmartPointer<vtkTubeFilter> axes_tubes = vtkSmartPointer<vtkTubeFilter>::New();
	axes_tubes->SetInputData(axes_data);
	axes_tubes->SetRadius(0.01);
	axes_tubes->SetNumberOfSides(6);

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetScalarModeToUsePointData();
	mapper->SetInputConnection(axes_tubes->GetOutputPort());

	//	vtkLODActor *axes_actor = vtkLODActor::New();
	vtkActor *actor = vtkActor::New();
	actor->SetMapper(mapper);
	actor->SetOrigin(0, 0, 0);		//设置在世界坐标系中的位置

	QWIDGET2VTKWINDOW(mVtkWindow)->AddActorTS(actor, QStringLiteral("CoordinateSystem"));
}


void ScanVisualizer::addGround()	//背景和网格
{
	vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();
	plane->SetXResolution(20);
	plane->SetYResolution(20);
	plane->SetCenter(0.0, 0.0, 0.0);
	plane->SetNormal(0.0, 0.0, 1.0);

	vtkSmartPointer<vtkPolyDataMapper> planeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	planeMapper->SetInputConnection(plane->GetOutputPort());

	vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New();
	trans->Scale(4, 4, 1);

	vtkActor *planeActor = vtkActor::New();
	planeActor->SetMapper(planeMapper);
	planeActor->GetProperty()->SetRepresentationToWireframe();
	planeActor->GetProperty()->SetDiffuse(0.0);
	planeActor->GetProperty()->SetAmbientColor(0.6, 0.6, 0.6);
	planeActor->GetProperty()->SetAmbient(1.0);
	planeActor->SetUserTransform(trans);
	QWIDGET2VTKWINDOW(mVtkWindow)->AddActorTS(planeActor, QStringLiteral("BasePlane"));
}

void ScanVisualizer::addRobot()		//机械臂
{
	QWIDGET2VTKWINDOW(mVtkWindow)->AddActorTS(mRobot.GetVtkActor(), QStringLiteral("Robot"));
}

void ScanVisualizer::addSphere(float radius, float posXYZ[3])
{
	vtkSmartPointer<vtkSphereSource> s_sphere = vtkSmartPointer<vtkSphereSource>::New();
	s_sphere->SetRadius(radius);
	s_sphere->SetPhiResolution(24);
	s_sphere->SetThetaResolution(24);
	s_sphere->LatLongTessellationOff();
	s_sphere->Update();

	vtkPolyData *polyData = s_sphere->GetOutput();
	unsigned char red[3] = { 200, 30, 30 };
	vtkSmartPointer<vtkUnsignedCharArray> pointsColor = vtkSmartPointer<vtkUnsignedCharArray>::New();
	pointsColor->SetNumberOfComponents(3);
	for (int i = 0; i < polyData->GetNumberOfPoints(); i++)
	{
		pointsColor->InsertNextTypedTuple(red);
	}

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	//mapper->SetScalarModeToUsePointData();
	mapper->SetInputConnection(s_sphere->GetOutputPort());

	vtkLODActor *actor = vtkLODActor::New();
	actor->SetPosition(posXYZ[0], posXYZ[1], posXYZ[2]);
	actor->SetMapper(mapper);
	actor->GetProperty()->SetRepresentationToSurface();
	actor->GetProperty()->SetDiffuseColor(0.9, 0.3, 0.3);

	QWIDGET2VTKWINDOW(mVtkWindow)->AddActorTS(actor, QStringLiteral("Sphere"));
}

/*
pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer("3D Viewer");
viewer->setBackgroundColor(0.94, 0.96, 0.96);
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(PDATA2SCANTASKDATA(mScanTaskData)->rawCloud);
viewer->addPointCloud<pcl::PointXYZRGB>(PDATA2SCANTASKDATA(mScanTaskData)->rawCloud, rgb, "raw cloud");
//	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
viewer->addCoordinateSystem(1.0);
viewer->initCameraParameters();
viewer->spin();
*/


/*
//K nearest search
std::vector<int> pointIdxNKNSearch;
std::vector<float> pointNKNSquaredDistance;
int K = 5;

searchPoint.x = (bound[0] + bound[1]) / 2;
searchPoint.y = (bound[2] + bound[3]) / 2;
searchPoint.z = (bound[4] + bound[5]) / 2;

if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
{
for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
cout << "    " << pts2D->at(pointIdxNKNSearch[i]).x
<< " " << pts2D->at(pointIdxNKNSearch[i]).y
<< " " << pts2D->at(pointIdxNKNSearch[i]).z
<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << endl;
}
*/


// Neighbors within voxel search
/*
std::vector<int> pointIdxVec;
if (octree.voxelSearch(searchPoint, pointIdxVec))
{
cout << "Neighbors within voxel search at (" << searchPoint.x
<< " " << searchPoint.y
<< " " << searchPoint.z << ")"
<< endl;

for (size_t i = 0; i < pointIdxVec.size(); ++i)
cout << "    " << pts2D->at(pointIdxVec[i]).x
<< " " << pts2D->at(pointIdxVec[i]).y
<< " " << pts2D->at(pointIdxVec[i]).z << endl;
}
*/

/* transform
Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
float theta = M_PI/4;   //旋转的度数，这里是45度
transform_1 (0,0) = cos (theta);  //这里是绕的Z轴旋转
transform_1 (0,1) = -sin(theta);
transform_1 (1,0) = sin (theta);
transform_1 (1,1) = cos (theta);
//   transform_1 (0,2) = 0.3;   //这样会产生缩放效果
//   transform_1 (1,2) = 0.6;
//    transform_1 (2,2) = 1;
transform_1 (0,3) = 25; //这里沿X轴平移
transform_1 (1,3) = 30;
transform_1 (2,3) = 380;
pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);

*/
