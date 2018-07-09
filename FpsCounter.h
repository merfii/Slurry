#pragma once

#include <QObject>
#include <QQueue>

class QElapsedTimer;
class QTimer;
class FpsCounter : public QObject
{
	Q_OBJECT
public:
	FpsCounter(QObject *parent);
	~FpsCounter();

	float getFPSf() const;
	int getFPSi() const;

	void setAverageLength(int ms); //对ms内的帧计算平均值
	public slots:
	void tick();	//Call it every new frame received 

	private slots:
	void timeupdate();

private:
	QElapsedTimer *timer;
	QTimer *ticker;
	QQueue<qint64> queue;
	float m_fps;
	QMutex mutex;
};


