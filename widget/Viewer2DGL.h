                                                            #pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>

#include <opencv2/core.hpp>

/*
绘图控件 与View2D 相同对外接口
后端使用OpenGL
严重拖慢系统性能
即便先建立再close掉也不行
*/

class Viewer2DGL : public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT

public:
	explicit Viewer2DGL(QWidget *parent = 0);

signals:
    void    imageSizeChanged( int outW, int outH ); /// Used to resize the image outside the widget

public slots:
    bool    setImage(const cv::Mat image); /// Used to set the image to be viewed

protected:
    void 	initializeGL(); /// OpenGL initialization
    void 	paintGL(); /// OpenGL Rendering
    void 	resizeGL(int width, int height);        /// Widget Resize Event

    void    updateScene();

private:

    QImage      mRenderQtImg;           /// Qt image to be rendered
    QImage      mResizedImg;
    cv::Mat     mOrigImage;             /// original OpenCV image to be shown

    QColor      mBgColor;		/// Background color

    float       mImgRatio;             /// height/width ratio

    int mRenderWidth;
    int mRenderHeight;
    int mRenderPosX;
    int mRenderPosY;

    void recalculatePosition();

	QMutex mutex;

};
