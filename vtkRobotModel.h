#pragma once

#include <QObject>
#include <vtkSmartPointer.h>
#include <vtkActor.h>


typedef void* PDATA_M;

class vtkRobotModel: public QObject
{
	Q_OBJECT

public:
	vtkRobotModel();
	~vtkRobotModel();

	void SetBaseTS(double xyz[3]);
	void SetJointTS(int jointIdx, double rad);
	void SetJointsTS(double jointsRad[6]);
	void SetJoints(double jointsRad[6]);
	void SetGuider(double frmTcp[6]);

	vtkActor* GetVtkActor() const;

private:
	vtkSmartPointer<vtkActor> loadGuiderBase();
	vtkSmartPointer<vtkActor> loadGuiderTube();

	PDATA_M vtkJoint[7];
	PDATA_M tcpReal;
	double joints[6];

private slots:
	void _setBase(double xyz[3]);
	void _setJoint(int jointIdx, double rad);
	void _setJoints(double *jointsRad);
};

