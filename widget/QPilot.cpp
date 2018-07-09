# pragma execution_character_set("utf-8")
#include <QPainter>
#include <QStylePainter>
#include <QStyleOptionFocusRect>

#include "QPilot.h"


QPilot::QPilot(QWidget *parent)
{
	QSizePolicy sp;
	sp.setWidthForHeight(true);
	sp.setVerticalPolicy(QSizePolicy::Expanding);
	//sp.setHorizontalPolicy(QSizePolicy::Minimum);
	setSizePolicy(sp);

	setAutoFillBackground(false);
	m_color = palette().background().color();
	redGradient = nullptr;
	m_flicker = false;
	m_flicker_state = 0;
	startTimer(800);
}

QPilot::~QPilot()
{
	delete redGradient;
}

void QPilot::setGreen()
{
	m_flicker = false;
	m_color = Qt::green;
	update();
}

void QPilot::setRed()
{
	m_flicker = false;
	m_color = Qt::red;
	update();
}

void QPilot::setRedFlicker()
{
	m_flicker = true;
	m_color = Qt::red;

	int w = width();
	int h = height();
	if (redGradient == nullptr)
	{
		redGradient = new QRadialGradient;
	}
	redGradient->setCenter(w / 2, h / 2);
	redGradient->setRadius(w * 3 / 5);
	redGradient->setFocalPoint(redGradient->center());
	redGradient->setColorAt(0, Qt::red);
	redGradient->setColorAt(1, Qt::white);

	update();
}

void QPilot::setYellow()
{
	m_flicker = false;
	m_color = Qt::yellow;
	update();
}

void QPilot::setClose()
{
	m_flicker = false;
	m_color = Qt::white;
	update();
}


void QPilot::setText(const QString &text)
{
	m_text = text;
	update();
}

QString QPilot::text() const
{
	return m_text;
}

void QPilot::setColor(const QColor &color)
{
	m_color = color;
}

QColor QPilot::color() const
{
	return m_color;
}


void QPilot::paintEvent(QPaintEvent *event)
{
	QStylePainter p(this);
	p.setRenderHints(QPainter::Antialiasing, false);

	int w = width();
	int h = height();
	// int side = qMin(width(), height());
	//p.setViewport((width() - side) / 2, (height() - side) / 2, side, side);		//目标坐标
	//p.setWindow(-50, -50, 100, 100);	//以下使用的逻辑坐标
	//p.setBackground(QBrush(Qt::green));

	if (m_flicker)
	{
		if (m_flicker_state % 2 == 0 && redGradient){
			p.setBrush(*redGradient);
		}
		else
		{
			p.setBrush(QBrush(Qt::white, Qt::SolidPattern));
		}
	}
	else
	{
		p.setBrush(QBrush(m_color, Qt::SolidPattern));
	}
	p.setPen(Qt::PenStyle::NoPen);
	p.drawRoundedRect(0, 0, w, h, 20, 20);

	p.setPen(QPen(Qt::black, 4, Qt::SolidLine));
	p.setFont(QFont("Arial", 14));
	p.drawText(rect(), Qt::AlignHCenter | Qt::AlignVCenter, m_text);
}

void QPilot::resizeEvent(QResizeEvent *event)
{
	setMinimumWidth(height());
	QWidget::resizeEvent(event);
}


void QPilot::mousePressEvent(QMouseEvent *event)
{
	mousePressFlag ++;
	//QWidget::mousePressEvent(event);
//	setCursor(Qt::PointingHandCursor);
	//update();
}

void QPilot::mouseReleaseEvent(QMouseEvent *e)
{
//setCursor(Qt::ArrowCursor);
	//this->isPressed = false;
}

void QPilot::timerEvent(QTimerEvent *event)
{
	if (m_flicker)
	{
		m_flicker_state ++;
		update();
	}
}

/*
void QPilot::mouseMoveEvent(QMouseEvent *e){
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

