                                                                                                                                   #pragma once

#include <QWidget>
#include <QColor>

class QRadialGradient;
class QPilot: public QWidget
{
	Q_OBJECT

public:
	explicit	QPilot(QWidget *parent = 0);
	virtual		~QPilot();

	Q_PROPERTY(QString m_text READ text WRITE setText)
	QString text() const;
	void setText(const QString &text);
	
	Q_PROPERTY(QColor m_color READ color WRITE setColor)
	QColor color() const;
	void setColor(const QColor &color);

public slots:
	void setGreen();
	void setRed();
	void setRedFlicker();
	void setYellow();
	void setClose();

protected:
	void paintEvent(QPaintEvent *event);
	void resizeEvent(QResizeEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *e);
	void timerEvent(QTimerEvent *event);

private:
	QColor m_color;
	QString m_text;
	bool m_flicker;
	int m_flicker_state;
	QRadialGradient *redGradient;
	int mousePressFlag;
};

