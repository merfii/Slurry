#include <vector>
#include <QTimer>
#include <QElapsedTimer>
#include "RobotController.h"
#include "RobotPath.h"

#include "PathRecorder.h"


PathRecorder::PathRecorder(double MERGE_RADIUS):
_MERGE_RADIUS(MERGE_RADIUS)
{
	path = new RobotPath;
	pathTimer = new QTimer;
	connect(pathTimer, SIGNAL(timeout()), this, SLOT(recordData()));
}


PathRecorder::~PathRecorder()
{
	delete pathTimer;
	delete path;
}

RobotPath *PathRecorder::GetPath() const
{
	return path;
}

void PathRecorder::record()
{
	path->clear();
	pathTimer->start(100);
}

void PathRecorder::play(double spd)
{
	RobotController::GetInstance()->MovePath(path->GetData(), spd, _MERGE_RADIUS);
}

void PathRecorder::playReversed(double spd)
{
	RobotController::GetInstance()->MovePath(path->GetData(), spd, _MERGE_RADIUS, true);
}

bool PathRecorder::isEmpty() const
{
	return path->size() <= 1;
}

void PathRecorder::stop()
{
	pathTimer->stop();
}

inline double FrmDist(RobotFrame &p1, RobotFrame &p2)
{
	double dist = 0;
	dist += (p1.P[0] - p2.P[0]) * (p1.P[0] - p2.P[0]);
	dist += (p1.P[1] - p2.P[1]) * (p1.P[1] - p2.P[1]);
	dist += (p1.P[2] - p2.P[2]) * (p1.P[2] - p2.P[2]);
	return sqrt(dist);
}

bool PathRecorder::isCongestion(RobotFrame *newFrm)
{
	if (path->size() == 0){
		return false;
	}
	if (FrmDist(path->GetData().back(), *newFrm) < _MERGE_RADIUS){
		return true;
	}
	else if (path->GetData().size() > 1 && FrmDist(path->GetData()[path->GetData().size() - 1], *newFrm) < _MERGE_RADIUS){
		return true;
	}
	return false;
}


void PathRecorder::recordData()
{
	RobotFrame frm = RobotController::GetRobotFrame();
	if (isCongestion(&frm)){
		return;
	}
	else{
		path->push_back(frm);
	}
	//Logger::Debug(QString("path length %1").arg((int)tcpPath.size()));
}



