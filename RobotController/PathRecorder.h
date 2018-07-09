#pragma once

#include <QObject>

class QTimer;
class QElapsedTimer;
class RobotPath;

class RobotFrame;
class PathRecorder :public QObject
{
	Q_OBJECT
public:
	//m 距离过近的点则合并
	PathRecorder(double MERGE_RADIUS  = 0.008);
	~PathRecorder();

	void record();
	void stop();
	void play(double spd = 0.2);
	void playReversed(double spd = 0.2);
	bool isEmpty() const;

	RobotPath *GetPath() const;

private slots:
	void recordData();
	bool isCongestion(RobotFrame *newFrm);

private:
	QTimer *pathTimer;
	RobotPath *path;
	double _MERGE_RADIUS;

};

