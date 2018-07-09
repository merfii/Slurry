# pragma execution_character_set("utf-8")
#include <opencv2/core.hpp>
#include <QPainter>
#include <QStylePainter>
#include <QStyleOptionFocusRect>
#include <QDateTime>
#include <QMutexLocker>

#include "FpsCounter.h"
#include "Viewer2D.h"


Viewer2D::Viewer2D(QWidget *parent)
{
	QSizePolicy sp;
	sp.setWidthForHeight(true);
	sp.setVerticalPolicy(QSizePolicy::Expanding);
	//sp.setHorizontalPolicy(QSizePolicy::Minimum);
	setSizePolicy(sp);

	for (int i = 0; i < 256; i++)
	{
		colorTab.append(qRgb(i, i, i));
	}

	mousePressFlag = 0;
	connect(this, SIGNAL(update_for_threadsafe()), this, SLOT(update()),Qt::QueuedConnection);

	fps = new FpsCounter(this);
	startTimer(1000);
}


Viewer2D::~Viewer2D()
{
	delete fps;
}

void Viewer2D::setColor(const QColor &color)
{
	
}

QColor Viewer2D::color() const
{
	return QColor(0, 0, 0);
}

void Viewer2D::setImage(QImage img, QString label)
{
	fps->tick();

	QMutexLocker locker(&mutex);

	m_img = img.copy();
	m_label = label;
	
	emit update_for_threadsafe();	
	//QCoreApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
	//update(); 不可重入

}

void Viewer2D::setImage(const cv::Mat image) /// Used to set the image to be viewed
{
	Q_ASSERT(!image.empty());

	QImage qimg;
	if (image.type() == CV_8UC1)
	{
		QImage qI(image.data, image.cols, image.rows, QImage::Format_Indexed8);
		createColorTable(qI);
		qimg = qI;
	}
	else if (image.type() == CV_8UC3)
	{
		QImage qI(image.data, image.cols, image.rows, QImage::Format_RGB888);
		qimg = qI;
	}
	else if (image.type() == CV_8UC4)
	{
		QImage qI(image.data, image.cols, image.rows, QImage::Format_RGBA8888);
		qimg = qI;
	}
	
	fps->tick();
	QMutexLocker locker(&mutex);
		m_img = qimg.copy();
		emit update_for_threadsafe();
}


void Viewer2D::createColorTable(QImage &img)
{
	img.setColorCount(256);
	img.setColorTable(colorTab);
}

void Viewer2D::saveImages()
{
	//判断并建立 ImagesOutput文件夹

	//获取当前时间日期

	//遍历四个通道保存图像

}


void Viewer2D::paintEvent(QPaintEvent *event)
{
	QStylePainter p(this);
	p.setRenderHints(QPainter::Antialiasing, true);

	int w = width();
	int h = height();
	// int side = qMin(width(), height());
	//p.setViewport((width() - side) / 2, (height() - side) / 2, side, side);		//目标坐标
	//p.setWindow(-50, -50, 100, 100);	//以下使用的逻辑坐标
	p.setPen(QPen(QColor(60,40,255), 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
	p.setBrush(QBrush(Qt::green, Qt::SolidPattern));
	p.setFont(QFont("Arial",12));

	mutex.lock();
	if (m_img.isNull())
	{
		p.fillRect(this->rect(), QColor(230, 235, 235));
	}else
	{
		p.drawImage(QRect(0, 0, w, m_img.height()*w / m_img.width()), m_img);
	}
	p.drawText(rect(), Qt::AlignHCenter | Qt::AlignBottom, m_label);
	p.drawText(rect(), Qt::AlignLeft | Qt::AlignBottom, QString("   FPS: %1").arg(fps->getFPSi()));
	p.drawText(rect(), Qt::AlignRight | Qt::AlignBottom, QDateTime::currentDateTime().time().toString()+ "     ");

	mutex.unlock();

	if (hasFocus())
	{
		QStyleOptionFocusRect option;
		option.initFrom(this);
		option.backgroundColor = palette().dark().color();
		p.drawPrimitive(QStyle::PE_FrameFocusRect, option);
	}
}


void Viewer2D::resizeEvent(QResizeEvent *event)
{
	setMinimumWidth(height());
	QWidget::resizeEvent(event);
}


void Viewer2D::mousePressEvent(QMouseEvent *event)
{
	mousePressFlag ++;
	//update();
	//QWidget::mousePressEvent(event);
	setCursor(Qt::PointingHandCursor);
}

void Viewer2D::mouseReleaseEvent(QMouseEvent *e)
{
	setCursor(Qt::ArrowCursor);
	//this->isPressed = false;
}

void Viewer2D::timerEvent(QTimerEvent *event)
{
//	qDebug() << "Timer ID:" << event->timerId();
//	update();
}

/*
void Viewer2D::mouseMoveEvent(QMouseEvent *e){
	if (this->isPressed){
		endPnt = e->pos();

		line->startPnt = startPnt;
		line->endPnt = endPnt;
		this->lines.push_back(line);

		update(); 
		startPnt = endPnt;
	}
}

*/


/*

void Viewer2D::paintEvent(QPaintEvent *event)
{
QStylePainter p(this);
p.setRenderHints(QPainter::Antialiasing, true);
p.fillRect(this->rect(), QColor(230, 235, 235));
// int side = qMin(width(), height());
//p.setViewport((width() - side) / 2, (height() - side) / 2, side, side);		//目标坐标
//p.setWindow(-50, -50, 100, 100);	//以下使用的逻辑坐标

return;
QTransform transform;
transform.rotate(+5.0);
transform.translate(+0.0, +0.0);
p.setWorldTransform(transform, false);

p.setPen(QPen(Qt::red, 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
p.setBrush(QBrush(Qt::green, Qt::SolidPattern));
p.setFont(QFont("Calibri"));






int startAngle = 30 * 16;
int spanAngle = 120 * 16;

p.drawArc(QRect(10 + mousePressFlag, 10 + mousePressFlag, 20, 40),startAngle, spanAngle);


p.drawText(QPointF(width() * 0.5 - 100, height() * 0.5), QObject::tr("Primary Series has NOT been set."));

if (mousePressFlag > 5)
{
if (!img)
setImage();

p.drawImage(QPoint(0,0), *img);
}


if (hasFocus())
{
QStyleOptionFocusRect option;
option.initFrom(this);
option.backgroundColor = palette().dark().color();
p.drawPrimitive(QStyle::PE_FrameFocusRect, option);

}
}


*/