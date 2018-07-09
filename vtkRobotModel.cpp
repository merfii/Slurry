#include <visp3/core/vpHomogeneousMatrix.h>

#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkArrowSource.h>
#include <vtkAssembly.h>
#include <vtkSTLReader.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkMatrix4x4.h>

#include "vtkRobotModel.h"
#include "SystemParameter.h"

#define PDATA2VTKASM(pdat) static_cast<vtkAssembly*>(pdat)
Q_DECLARE_METATYPE(double*);

vtkRobotModel::vtkRobotModel()
{
	int metaTypeId = qRegisterMetaType<double*>();
	vtkAssembly *jointAsm[7];
	for (int i = 0; i < 7; i++)
	{
		jointAsm[i] = vtkAssembly::New();
	}

	vtkSmartPointer<vtkActor> actors[7];
	vtkSmartPointer<vtkActor> guiderBase = loadGuiderBase();
	vtkSmartPointer<vtkActor> guiderTube = loadGuiderTube();
	char modelfile[] = "UrModel/joint1.stl";
	for (int i = 0; i < 7; i++)
	{
		actors[i] = vtkSmartPointer<vtkActor>::New();
	//读取STL模型
		vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
		modelfile[13] = '1' + i;

		reader->SetFileName(modelfile);
		reader->Update();
		vtkSmartPointer<vtkPolyDataMapper> mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(reader->GetOutputPort());
		actors[i]->SetMapper(mapper);
		actors[i]->GetProperty()->SetDiffuseColor(0.65, 0.8, 0.95);
		//actors[i]->GetProperty()->SetSpecularColor(0.8, 0.8, 0.8);
		actors[i]->GetProperty()->SetSpecular(0.4);
		actors[i]->GetProperty()->SetSpecularPower(40);
	}
	//关节从属关系
	jointAsm[0]->AddPart(actors[0]);
	jointAsm[0]->AddPart(jointAsm[1]);

	jointAsm[1]->AddPart(actors[1]);
	jointAsm[1]->AddPart(jointAsm[2]);

	jointAsm[2]->AddPart(actors[2]);
	jointAsm[2]->AddPart(jointAsm[3]);

	jointAsm[3]->AddPart(actors[3]);
	jointAsm[3]->AddPart(jointAsm[4]);

	jointAsm[4]->AddPart(actors[4]);
	jointAsm[4]->AddPart(jointAsm[5]);

	jointAsm[5]->AddPart(actors[5]);
	jointAsm[5]->AddPart(jointAsm[6]);

	jointAsm[6]->AddPart(actors[6]);
	jointAsm[6]->AddPart(guiderBase);	//导引架	
	//jointAsm[6]->AddPart(guiderTube);	//导引管

	vtkAssembly *tcpAsm = vtkAssembly::New();
	tcpAsm->SetPosition(0, 0, 0);
	tcpAsm->SetOrientation(0, 0, 0);
	tcpAsm->AddPart(guiderTube);
	jointAsm[0]->AddPart(tcpAsm);
	tcpReal = tcpAsm;

	//设置关节基准点初始位置
	jointAsm[0]->SetScale(0.001, 0.001, 0.001);
	jointAsm[0]->SetPosition(0, 0, 0);
	jointAsm[1]->SetPosition(0.0, 0.0, 0.0);
	jointAsm[2]->SetPosition(0.0, 70.5, 65.2);
	jointAsm[3]->SetPosition(0.0, 0.0, 425.0);
	jointAsm[4]->SetPosition(-0.0007, -7.0024, 392.43);
	jointAsm[5]->SetPosition(0.0, 45.5, 47.4976);
	jointAsm[6]->SetPosition(-0.0007, 47.4975, 45.5);

	for (int i = 0; i < 7; i++)
	{
		vtkJoint[i] = jointAsm[i];
	}

}


vtkRobotModel::~vtkRobotModel()
{
	for (int i = 1; i < 7; i++)
	{
		PDATA2VTKASM(vtkJoint[i])->Delete();
	}
	PDATA2VTKASM(vtkJoint[0])->Delete();
}

vtkSmartPointer<vtkActor> vtkRobotModel::loadGuiderBase()
{
	vtkSmartPointer<vtkActor> guider = vtkSmartPointer<vtkActor>::New();
	//读取STL模型
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName("UrModel/guider.stl");
	reader->Update();
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader->GetOutputPort());
	guider->SetMapper(mapper);
	guider->GetProperty()->SetDiffuseColor(0.7, 0.75, 0.95);
	//actors[i]->GetProperty()->SetSpecularColor(0.8, 0.8, 0.8);
	guider->GetProperty()->SetSpecular(0.4);
	guider->GetProperty()->SetSpecularPower(40);
	guider->SetPosition(-38, 200,15);
	guider->SetOrientation(-90,0,180);

	return guider;
}

vtkSmartPointer<vtkActor> vtkRobotModel::loadGuiderTube()
{
	vtkSmartPointer<vtkActor> guider = vtkSmartPointer<vtkActor>::New();
	vtkSmartPointer<vtkArrowSource> arrowSource =
		vtkSmartPointer<vtkArrowSource>::New();
	arrowSource->SetShaftRadius(0.02);	//箭头柄半径/箭头长
	arrowSource->SetShaftResolution(3);
	arrowSource->SetTipResolution(36);	//箭头尖分辨率
	arrowSource->SetTipLength(0.2);
	arrowSource->SetTipRadius(0.04);
	arrowSource->SetInvert(true);
	arrowSource->Update();
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(arrowSource->GetOutputPort());
	guider->SetMapper(mapper);
	guider->GetProperty()->SetDiffuseColor(1, 0.8, 0.08);
	//actors[i]->GetProperty()->SetSpecularColor(0.8, 0.8, 0.8);
	guider->GetProperty()->SetSpecular(0.4);
	guider->GetProperty()->SetSpecularPower(40);
	guider->SetScale(250);
//	guider->SetPosition(-100, -70, 300);
//	guider->SetOrientation(0, 56, 0);
	guider->SetOrientation(0, 90, 0);

	vpHomogeneousMatrix Mtool;
	Mtool.buildFrom(SystemParameter::GetInstance()->GetToolMatrix());
	vtkSmartPointer<vtkMatrix4x4> Mvtk = vtkSmartPointer<vtkMatrix4x4>::New();
	Mvtk->Identity();
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			Mvtk->SetElement(i, j, Mtool[i][j]);
		}
		Mvtk->SetElement(i, 3, Mtool[i][3] * 1000);
	}
	guider->SetUserMatrix(Mvtk);
	return guider;
}


vtkActor* vtkRobotModel::GetVtkActor() const
{
	return static_cast<vtkActor*>(vtkJoint[0]);
}


void vtkRobotModel::SetBaseTS(double xyz[3])
{
	QMetaObject::invokeMethod(this, "_setBase", Qt::QueuedConnection,
		Q_ARG(double*, xyz));
}

void vtkRobotModel::_setBase(double xyz[3])
{
	PDATA2VTKASM(vtkJoint[0])->SetPosition(xyz);
}


void vtkRobotModel::SetJointTS(int jointIdx, double rad)
{
	QMetaObject::invokeMethod(this, "_setJoint", Qt::QueuedConnection,
		Q_ARG(int, jointIdx), Q_ARG(double, rad));
}

#define M_PI 3.14159265358979323846
void vtkRobotModel::_setJoint(int jointIdx, double rad)
{
	double deg = rad * 180 / M_PI;
	switch (jointIdx)
	{
	case 0:
		PDATA2VTKASM(vtkJoint[1])->SetOrientation(0, 0, deg);
		break;
	case 1:
		PDATA2VTKASM(vtkJoint[2])->SetOrientation(0, deg, 0);
		break;
	case 2:
		PDATA2VTKASM(vtkJoint[3])->SetOrientation(0, deg, 0);
		break;
	case 3:
		PDATA2VTKASM(vtkJoint[4])->SetOrientation(0, deg, 0);
		break;
	case 4:
		PDATA2VTKASM(vtkJoint[5])->SetOrientation(0, 0, deg);
		break;
	case 5:
		PDATA2VTKASM(vtkJoint[6])->SetOrientation(0, deg, 0);
		break;

	default:
		break;
	}
}

void vtkRobotModel::SetJoints(double jointsRad[6])
{
	PDATA2VTKASM(vtkJoint[1])->SetOrientation(0, 0, jointsRad[0] * 180 / M_PI + 180);
	PDATA2VTKASM(vtkJoint[2])->SetOrientation(0, jointsRad[1] * 180 / M_PI + 90, 0);
	PDATA2VTKASM(vtkJoint[3])->SetOrientation(0, jointsRad[2] * 180 / M_PI, 0);
	PDATA2VTKASM(vtkJoint[4])->SetOrientation(0, jointsRad[3] * 180 / M_PI + 90, 0);
	PDATA2VTKASM(vtkJoint[5])->SetOrientation(0, 0, jointsRad[4] * 180 / M_PI );
	PDATA2VTKASM(vtkJoint[6])->SetOrientation(0, jointsRad[5] * 180 / M_PI, 0);
}

void vtkRobotModel::SetJointsTS(double jointsRad[6])
{
	for (int i = 0; i < 6; i++)
	{
		joints[i] = jointsRad[i];
	}
	QMetaObject::invokeMethod(this, "_setJoint", Qt::QueuedConnection,
		Q_ARG(double*, jointsRad));
}

void vtkRobotModel::_setJoints(double *jointsRad)
{
	PDATA2VTKASM(vtkJoint[1])->SetOrientation(0, 0, joints[0] * 180 / M_PI + 180);
	PDATA2VTKASM(vtkJoint[2])->SetOrientation(0, -joints[1] * 180 / M_PI + 90, 0);
	PDATA2VTKASM(vtkJoint[3])->SetOrientation(0, -joints[2] * 180 / M_PI, 0);
	PDATA2VTKASM(vtkJoint[4])->SetOrientation(0, -joints[3] * 180 / M_PI + 90, 0);
	PDATA2VTKASM(vtkJoint[5])->SetOrientation(0, 0, -joints[4] * 180 / M_PI);
	PDATA2VTKASM(vtkJoint[6])->SetOrientation(0, -joints[5] * 180 / M_PI, 0);
}

//void vtkRobotModel::SetGuiderTS(double frmTcp[6])
//{
//	QMetaObject::invokeMethod(this, "_setGuider", Qt::QueuedConnection,
//		Q_ARG(double*, frmTcp));
//}

void vtkRobotModel::SetGuider(double frmTcp[6])
{
	vpHomogeneousMatrix M;
	M.buildFrom(frmTcp[0], frmTcp[1], frmTcp[2], frmTcp[3], frmTcp[4], frmTcp[5]);	//unit: m m m rad rad rad
	vtkSmartPointer<vtkMatrix4x4> Mvtk = vtkSmartPointer<vtkMatrix4x4>::New();
	Mvtk->Identity();
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			Mvtk->SetElement(i, j, M[i][j]);
		}
		Mvtk->SetElement(i, 3, M[i][3]*1000);
	}
	PDATA2VTKASM(tcpReal)->SetUserMatrix(Mvtk);
}



