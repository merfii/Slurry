#include <QMutexLocker>
#include <QFileDialog>
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
//#include <pcl/io/vtk_io.h>
#include <pcl/visualization/pcl_visualizer.h>


#include "VtkWindow.h"
#include "Utils.h"
#include "ScanVisualizer.h"

using std::cout;
using std::endl;


#define QWIDGET2VTKWINDOW(widget) static_cast<VtkWindow*>(widget)
#define PDATA2SCANTASKDATA(dat) static_cast<ScanTaskData*>(dat)
#define SQUARED_DIST(VecA, VecB) ((VecA[0]-VecB[0])*(VecA[0]-VecB[0])+(VecA[1]-VecB[1])*(VecA[1]-VecB[1])+(VecA[2]-VecB[2])*(VecA[2]-VecB[2]))
//pcl::geometry::distance(p1, p2);

#define DECIMATE_RATIO 10	//只取1/10的点用于显示
#define DISPLAY_EVERY_SHOT 1	//每帧更新显示
#define DISPLAY_GRID_SIZE 0.005	//规则化重采样格点每隔5mm

class ScanTaskData{
public:
	ScanTaskData() :
		shotCount(0)
	{
		rawCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		rawCloud->clear();
		deciCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		deciCloud_pre = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		norm(0) = 0; norm(1) = 0; norm(2) = 1;
	}

	~ScanTaskData()
	{

	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rawCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr deciCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr deciCloud_pre;	//用来储存上一轮点云以避免间隙

	Eigen::Vector3f norm;

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


void ScanVisualizer::ScanMainInit()
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

static vtkSmartPointer<vtkPolyData> scanPointsTriangulation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr srcPts, Eigen::Vector3f norm);
static void _calculate_bound_projected_rect(double bound[6], Eigen::Vector3f &axis_u, Eigen::Vector3f &axis_v, double out_rect[4]);
static pcl::PointXYZRGB _surface_point_normal(std::vector<int> &ptsIdx, pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> &octree,
	Eigen::Vector3f &norm_out);

void ScanVisualizer::ScanTaskShotPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::Vector3f eyeNorm)
{
	QMutexLocker locker(&mutex);
	if (!mScanning)
		return;

	ScanTaskData *mScanData = PDATA2SCANTASKDATA(mScanTaskData);
	Q_ASSERT(mScanData);
	//*mScanData->rawCloud += *pointCloud;
	mScanData->norm = (mScanData->norm * 2 + eyeNorm) / 3;	//求平均

	mScanData->shotCount++;

	if (mScanData->shotCount % DISPLAY_EVERY_SHOT == 0)
	{


		mScanData->deciCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int i = 0; i < pointCloud->size(); i += DECIMATE_RATIO)
		{
			mScanData->deciCloud->push_back(pointCloud->at(i));
			//mScanData->deciCloud_pre->push_back(pointCloud->at(i));
		}
		*mScanData->rawCloud += *mScanData->deciCloud;
		ScanTaskDisplayPoints(mScanData->deciCloud);
		return;


		if (mScanData->deciCloud->size() > 50)
		{
			//已经收集了DISPLAY_AFTER_SHOT次数据，将其三角化并显示出来
			vtkSmartPointer<vtkPolyData> polyData = scanPointsTriangulation(mScanData->deciCloud, mScanData->norm);
			// Setup actor and mapper
			vtkSmartPointer<vtkPolyDataMapper> mapper =	vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper->SetInputData(polyData);
			mapper->SetScalarModeToUsePointData();
			vtkActor *actor = vtkActor::New();
			actor->SetMapper(mapper);
//			actor->GetProperty()->SetDiffuse(1);
			actor->GetProperty()->SetSpecular(.3);
			actor->GetProperty()->SetSpecularPower(15);
			
//	QString label = QString("Preview%1").arg(mScanData->shotCount, 3, 10, QLatin1Char('0'));
			QWIDGET2VTKWINDOW(mVtkWindow)->AddActorTS(actor, QStringLiteral("Preview"));
		}
		mScanData->deciCloud = mScanData->deciCloud_pre;
		mScanData->deciCloud_pre = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	}
	else if (mScanData->shotCount % DISPLAY_EVERY_SHOT > DISPLAY_EVERY_SHOT - 6)
	{
		//前后两次曲面应少量重叠 以使两次显示表面之间没有间隙 
		for (int i = 0; i < pointCloud->size(); i += DECIMATE_RATIO)
		{
			mScanData->deciCloud_pre->push_back(pointCloud->at(i));
			mScanData->deciCloud->push_back(pointCloud->at(i));
		}
	}
	else{
		for (int i = 0; i < pointCloud->size(); i += DECIMATE_RATIO)
		{
			mScanData->deciCloud->push_back(pointCloud->at(i));
		}
	}

}


void ScanVisualizer::ScanTaskShotPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
	Eigen::Vector3f normZ;
	normZ << 0, 0, 1;
	ScanTaskShotPoints(pointCloud, normZ);
}


//将三维点沿法线投影到二维平面上  然后按行列扫描粗略生成三角片
static vtkSmartPointer<vtkPolyData> scanPointsTriangulation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr srcPts, Eigen::Vector3f norm)
{
	Q_ASSERT(srcPts->size() > 50);
	
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

//	vertices->InsertNextCell(nr_points);	//这里要先知道点的数量  不知道咋弄
	
	vtkIdType vtkPtId[3];
	unsigned char vtkPtColor[3];
	double vtkPtNorm[3];
/*
	polyPoints->InsertNextPoint(0.0, 0.0, 0.0);	//插入点
	polyColors->InsertNextTupleValue(vtkPtColor);	//插入颜色
	polyNorms->InsertNextTuple(vtkPtNorm);	//插入法线
	polyTriangles->InsertNextCell(3, vtkPtId);	//插入三角片
*/

	//生成投影的单位向量
	norm.normalize();
	Eigen::Vector3f axis_u, axis_v;
	{
		int u_start = srcPts->size() / (DISPLAY_EVERY_SHOT + 1);
		axis_u(0) = srcPts->at(u_start).x - srcPts->at(0).x;
		axis_u(1) = srcPts->at(u_start).y - srcPts->at(0).y;
		axis_u(2) = srcPts->at(u_start).z - srcPts->at(0).z;
		axis_u = axis_u - axis_u.dot(norm) * axis_u;
		axis_u.normalize();
		axis_v = norm.cross(axis_u);
		axis_v.normalize();
	}

//DebugTimeElapse(false);
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(0.005);	//最小分辨率0.005m=5mm
	octree.setEpsilon(0.0001);
	//octree.setTreeDepth(16);
	octree.setInputCloud(srcPts);
	octree.addPointsFromInputCloud();
//DebugTimeElapse(true);

	double bound[6], rect[4];
	octree.getBoundingBox(bound[0], bound[1], bound[2], bound[3], bound[4], bound[5]);
	_calculate_bound_projected_rect(bound, axis_u, axis_v, rect);
	int gridW = (int)((rect[2] - rect[0]) / DISPLAY_GRID_SIZE)+1;
	int gridH = (int)((rect[3] - rect[1]) / DISPLAY_GRID_SIZE)+1;

	std::vector< std::vector<std::vector<int> > > ptsIdx2dGrid;
	ptsIdx2dGrid.resize(gridH);
	for (auto it = ptsIdx2dGrid.begin(); it != ptsIdx2dGrid.end(); it++)
	{
		it->resize(gridW);
	}

	for (int srcidx = 0; srcidx < srcPts->size(); srcidx++)
	{
		double u = axis_u(0)* srcPts->at(srcidx).x + axis_u(1)* srcPts->at(srcidx).y + axis_u(2)* srcPts->at(srcidx).z;
		double v = axis_v(0)* srcPts->at(srcidx).x + axis_v(1)* srcPts->at(srcidx).y + axis_v(2)* srcPts->at(srcidx).z;
		const int u_idx = (int)((u - rect[0]) / DISPLAY_GRID_SIZE);
		const int v_idx = (int)((v - rect[1]) / DISPLAY_GRID_SIZE);
	//	cout << u_idx << " " << v_idx << endl;
	//	cout << u << " " << v << endl;
		ptsIdx2dGrid[u_idx][v_idx].push_back(srcidx);
	}

	//const double offsetU = rect[0] + DISPLAY_GRID_SIZE / 2;
	//const double offsetV = rect[1] + DISPLAY_GRID_SIZE / 2;
//DebugTimeElapse(false);
	for (int idxu = 0; idxu < gridH; idxu++)
	{
	//	cout << vtkPtNorm[0] << " " << vtkPtNorm[1] << " " << vtkPtNorm[2] << endl;
		for (int idxv = 0; idxv < gridW; idxv++)
		{
			if (ptsIdx2dGrid[idxu][idxv].size() > 0)
			{
				Eigen::Vector3f norm_out = norm;
				pcl::PointXYZRGB cpt = _surface_point_normal(ptsIdx2dGrid[idxu][idxv], octree, norm_out);
				ptsIdx2dGrid[idxu][idxv].resize(1);
				//注意 本函数中的idx有两种含义 在此之前是srcPts的下标 在此之后是用于polyPoints的下标
				ptsIdx2dGrid[idxu][idxv][0] = polyPoints->InsertNextPoint(cpt.x, cpt.y, cpt.z);
				//插入颜色
				vtkPtColor[0] = cpt.r; vtkPtColor[1] = cpt.g; vtkPtColor[2] = cpt.b;
				polyColors->InsertNextTypedTuple(vtkPtColor);
				//插入法线
				vtkPtNorm[0] = norm_out[0];
				vtkPtNorm[1] = norm_out[1];
				vtkPtNorm[2] = norm_out[2];
				polyNorms->InsertNextTuple(vtkPtNorm);

				//第一行或第一列 无法构造三角片
				if (idxu == 0 || idxv == 0)
					continue;

				//根据左上三个点的存在情况尝试构造三角片
				if (ptsIdx2dGrid[idxu][idxv - 1].empty())
				{
					if (ptsIdx2dGrid[idxu - 1][idxv].empty() || ptsIdx2dGrid[idxu - 1][idxv - 1].empty())
					{
						/*
							XO		OX		XX
							XO  或  XO 或   XO
						*/
					//	vertices->InsertCellPoint(ptsIdx2dGrid[idxu][idxv][0]);
						continue;
					}
					/*
						OO
						XO
					*/
					vtkPtId[0] = ptsIdx2dGrid[idxu - 1][idxv][0];
					vtkPtId[1] = ptsIdx2dGrid[idxu - 1][idxv - 1][0];
					vtkPtId[2] = ptsIdx2dGrid[idxu][idxv][0];
				//	polyTriangles->InsertNextCell(3, vtkPtId);
				}
				else
				{
					if (ptsIdx2dGrid[idxu - 1][idxv].empty() && ptsIdx2dGrid[idxu - 1][idxv - 1].empty())
					{
					/*
						XX
						OO
					*/
						vertices->InsertCellPoint(ptsIdx2dGrid[idxu][idxv][0]);
						continue;
					}
					if (ptsIdx2dGrid[idxu - 1][idxv].empty())
					{
					/*
						OX
						OO
					*/
						vtkPtId[0] = ptsIdx2dGrid[idxu - 1][idxv - 1][0];
						vtkPtId[1] = ptsIdx2dGrid[idxu][idxv - 1][0];
						vtkPtId[2] = ptsIdx2dGrid[idxu][idxv][0];
						polyTriangles->InsertNextCell(3, vtkPtId);
					}
					else if (ptsIdx2dGrid[idxu - 1][idxv - 1].empty())
					{
					/*
						XO
						OO
					*/
						vtkPtId[0] = ptsIdx2dGrid[idxu - 1][idxv][0];
						vtkPtId[1] = ptsIdx2dGrid[idxu][idxv - 1][0];
						vtkPtId[2] = ptsIdx2dGrid[idxu][idxv][0];
						polyTriangles->InsertNextCell(3, vtkPtId);
					}
					else
					{
					/*
						OO
						OO
					*/
						vtkPtId[0] = ptsIdx2dGrid[idxu - 1][idxv - 1][0];
						vtkPtId[1] = ptsIdx2dGrid[idxu][idxv - 1][0];
						vtkPtId[2] = ptsIdx2dGrid[idxu][idxv][0];
						polyTriangles->InsertNextCell(3, vtkPtId);

						vtkPtId[0] = ptsIdx2dGrid[idxu - 1][idxv][0];
						vtkPtId[1] = ptsIdx2dGrid[idxu - 1][idxv - 1][0];
						vtkPtId[2] = ptsIdx2dGrid[idxu][idxv][0];
						polyTriangles->InsertNextCell(3, vtkPtId);
					}
				}
			}
			else
			{	
				//ptsIdx2dGrid[idxu][idxv].size() = 0
				if (idxu == 0 || idxv == 0)
					continue;
				if (!ptsIdx2dGrid[idxu - 1][idxv].empty() && !ptsIdx2dGrid[idxu][idxv - 1].empty() && !ptsIdx2dGrid[idxu - 1][idxv - 1].empty())
				{
				/*
					OO
					OX
				*/
					vtkPtId[0] = ptsIdx2dGrid[idxu - 1][idxv][0];
					vtkPtId[1] = ptsIdx2dGrid[idxu - 1][idxv - 1][0];
					vtkPtId[2] = ptsIdx2dGrid[idxu][idxv - 1][0];
					polyTriangles->InsertNextCell(3, vtkPtId);
				}
			}
		}
	}
//DebugTimeElapse(true);

	polyData->SetPoints(polyPoints);
	polyData->GetPointData()->SetScalars(polyColors);
	polyData->GetPointData()->SetNormals(polyNorms);
	polyData->SetPolys(polyTriangles);
//	polyData->SetVerts(vertices);
	return polyData;
}


//	bound[6] xmin ymin zmin xmax ymax zmax
//	out_rect[4] umin vmin umax vmax
void _calculate_bound_projected_rect(double bound[6], Eigen::Vector3f &axis_u, Eigen::Vector3f &axis_v, double out_rect[4])
{
	Eigen::VectorXd us(8);
	Eigen::VectorXd vs(8);
	int idx = 0;
	for (int i = 0; i < 6; i+=3)
	{
		for (int j = 1; j < 6; j+=3)
		{
			for (int k = 2; k < 6; k+=3)
			{
				us(idx) = bound[i] * axis_u(0) + bound[j] * axis_u(1) + bound[k] * axis_u(2);
				vs(idx) = bound[i] * axis_v(0) + bound[j] * axis_v(1) + bound[k] * axis_v(2);
				idx++;
			}
		}
	}
//	cout << us << endl;
//	cout << vs << endl;

	out_rect[0] = us.minCoeff();
	out_rect[1] = vs.minCoeff();
	out_rect[2] = us.maxCoeff();
	out_rect[3] = vs.maxCoeff();
}

//由于是投影之后搜索的 所以上下表面的点重合在一起  需要先计算投影距离然后排序
//从外向内求表面法线 成功即返回
static pcl::PointXYZRGB _surface_point_normal(std::vector<int> &ptsIdx, pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> &octree,
	Eigen::Vector3f &norm_out)
{
	Eigen::Vector3f norm = norm_out;
	Eigen::Vector4f plane;
	float curvature;
	pcl::PointXYZRGB cpt;
	std::vector<double> norm_dist;
	norm_dist.resize(ptsIdx.size());
	for (int i = 0; i < ptsIdx.size(); i++)
	{
		const pcl::PointXYZRGB &p = octree.getInputCloud()->points[ptsIdx[i]];
		norm_dist[i] = p.x * norm[0] + p.y * norm[1] + p.z * norm[2];
	}

	//对norm_dist从大到小寻找曲面
	for (int i = 0; i < norm_dist.size(); i++)
	{
		int maxidx = 0 ;	//最大的索引
		for (int j = 0; j < norm_dist.size(); j++)
		{
			if (norm_dist[j] > norm_dist[maxidx])
			{
				maxidx = j;
			}
		}
			
		// Neighbors within radius search
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		float radius = 0.02;
		if (octree.radiusSearch(ptsIdx[maxidx], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3)
		{
			/*
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i){
				cout << pointIdxRadiusSearch[i] << 
					" (squared distance: " << pointRadiusSquaredDistance[i] << ")" << endl;
			}
			*/
			// Placeholder for the 3x3 covariance matrix at each surface patch
			EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
			// 16-bytes aligned placeholder for the XYZ centroid of a surface patch
			Eigen::Vector4f xyz_centroid;
			if (pcl::computeMeanAndCovarianceMatrix(*octree.getInputCloud(), pointIdxRadiusSearch, covariance_matrix, xyz_centroid) != 0)
			{
				pcl::solvePlaneParameters(covariance_matrix, xyz_centroid, plane, curvature);
			//	cout << "curvature: " << curvature << endl;
				cpt.x = xyz_centroid[0];
				cpt.y = xyz_centroid[1];
				cpt.z = xyz_centroid[2];
				cpt.r = octree.getInputCloud()->at(ptsIdx[maxidx]).r;
				cpt.g = octree.getInputCloud()->at(ptsIdx[maxidx]).g;
				cpt.b = octree.getInputCloud()->at(ptsIdx[maxidx]).b;

				norm_out(0) = plane(0);
				norm_out(1) = plane(1);
				norm_out(2) = plane(2);
				norm_out.normalize();

				if (norm_out.dot(norm) < 0)
				{
					norm_out = norm_out * -1;
				}
				return cpt;
			}
		}
		norm_dist[maxidx] = std::numeric_limits<double>::min();
	}
	
	cpt = octree.getInputCloud()->at(ptsIdx[0]);
	return cpt;
}

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

static void createActorFromVTKDataSet(const vtkSmartPointer<vtkDataSet> &data, vtkActor *actor);
void ScanVisualizer::ScanTaskDisplayPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkUnsignedCharArray> polyColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	polyColors->SetNumberOfComponents(3);
	polyColors->SetName("Colors");

	vtkIdType nr_points = pointCloud->size();
	// Create the supporting structures
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	vertices->Allocate(vertices->EstimateSize(1, nr_points));
	vertices->InsertNextCell(nr_points);
	unsigned char color[3];
	for (auto it = pointCloud->begin(); it != pointCloud->end(); it++)
	{
		vertices->InsertCellPoint(points->InsertNextPoint(it->x, it->y, it->z));
		color[0] = it->r; color[1] = it->g; color[2] = it->b;
		polyColors->InsertNextTypedTuple(color);
	}
	polydata->SetPoints(points);
	polydata->GetPointData()->SetScalars(polyColors);
	polydata->SetVerts(vertices);
//	cout << polydata->GetNumberOfPoints() << endl;
//	cout << polydata->GetNumberOfCells() << endl;

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
	//点云合并 全部显示

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
}


void ScanVisualizer::ScanSaveData()
{
	mutex.lock();
	mScanning = false;
	if (mScanTaskData)
	{
		pcl::io::savePCDFileASCII("PointCloudDataAscii.pcd",
			*PDATA2SCANTASKDATA(mScanTaskData)->rawCloud);

		pcl::io::savePCDFileASCII("PointCloudDataBin.pcd",
			*PDATA2SCANTASKDATA(mScanTaskData)->rawCloud);

		pcl::io::savePLYFileASCII("PointCloudDataAscii.ply",
			*PDATA2SCANTASKDATA(mScanTaskData)->rawCloud);
	}
	mutex.unlock();
}

void ScanVisualizer::ScanLoadData(QWidget *window)
{	
	ScanTaskClear();
	mScanning = false;
	mutex.lock();
	//对话框
	QString path = QFileDialog::getOpenFileName(window, "打开文件", QString(), "*.pcd");
	if (path.isEmpty())
		return;
	if (pcl::io::loadPCDFile(path.toStdString(), *PDATA2SCANTASKDATA(mScanTaskData)->rawCloud))
	{
		cout << "载入错误";
		mutex.unlock();
		return;
	}
	mutex.unlock();
	ScanTaskDisplayPoints(PDATA2SCANTASKDATA(mScanTaskData)->rawCloud);
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
