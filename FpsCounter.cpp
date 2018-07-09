#include <QTimer>
#include <QElapsedTimer>
#include <QMutexLocker>
#include "FpsCounter.h"

#include <QDebug>
#define MAX_QUEUE_LENGHT 50
#define TICK_INTERVAL 400	//ms
#define AVERAGE_LEN 2000


FpsCounter::FpsCounter(QObject *parent)
	: QObject(parent)
{
	timer = new QElapsedTimer();
	timer->start();
	ticker = new QTimer();
	connect(ticker, SIGNAL(timeout()), this, SLOT(timeupdate()));
	ticker->start(TICK_INTERVAL);
	m_fps = 0;
}

FpsCounter::~FpsCounter()
{
	delete ticker;
	delete timer;
}

float FpsCounter::getFPSf() const
{
	return m_fps;
}

int FpsCounter::getFPSi() const
{
	return (int)m_fps;
}

void FpsCounter::tick()		//记录每次记时的间隔，然后将每次的时间间隔入queue列
{
	QMutexLocker locker(&mutex);
	queue.enqueue(timer->elapsed());
	while (queue.length() > MAX_QUEUE_LENGHT)
	{
		queue.dequeue();
	}
}

void FpsCounter::timeupdate()		//计算频率m_fps
{
	QMutexLocker locker(&mutex);
	qint64 tnow = timer->elapsed();

	if (!queue.isEmpty())
	{
		while (!queue.isEmpty() && (tnow - queue.head() > AVERAGE_LEN) )
		{
				queue.removeFirst();
		}
	}

	if (queue.isEmpty())
	{
		m_fps = 0;
	}else
	{
		/*
		qint64 t;
		foreach(t, queue)
		{
			dt = tnow - t;
		}
		*/
		m_fps = (float)queue.size() / (tnow - queue.head()) *1000;
	}

}
