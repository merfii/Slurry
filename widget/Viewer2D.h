#pragma once

#include <QWidget>
#include <QColor>

#include <opencv2/core.hpp>

class FpsCounter;
class Viewer2D : public QWidget
{
	Q_OBJECT

public:
	explicit	Viewer2D(QWidget *parent = 0);
	virtual		~Viewer2D();

	Q_PROPERTY(QColor color READ color WRITE setColor)
		QColor color() const;
	void	setColor(const QColor &color);

public slots:
	void	setImage(QImage img, QString label);
	void    setImage(const cv::Mat image); /// Used to set the image to be viewed
	void	saveImages();

signals:
	void    imageSizeChanged(int outW, int outH); /// Used to resize the image outside the widget
	void	update_for_threadsafe();

//	Q_INVOKABLE void invokeByMetaSystem();
protected:
	void paintEvent(QPaintEvent *event);
	void resizeEvent(QResizeEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *e);
	void timerEvent(QTimerEvent *event);

private:
	void createColorTable(QImage& img);

	int mousePressFlag;
	QImage m_img;
	QString m_label;
	QMutex mutex;
	FpsCounter *fps;
	QVector<uint> colorTab;

};

