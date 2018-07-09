#include <opencv2/imgproc.hpp>
#include <QMutexLocker>
#include "GlobalShared.h"
#include "Viewer2DGL.h"

using namespace cv;

Viewer2DGL::Viewer2DGL(QWidget *parent):
QOpenGLWidget(parent)
{
    mBgColor = QColor::fromRgb(150, 150, 150);
	mRenderQtImg = QImage(100, 100, QImage::Format_RGBA8888);
	mRenderQtImg.fill(mBgColor);
}

void Viewer2DGL::initializeGL()
{
    initializeOpenGLFunctions();

    float r = ((float)mBgColor.darker().red())/255.0f;
    float g = ((float)mBgColor.darker().green())/255.0f;
    float b = ((float)mBgColor.darker().blue())/255.0f;
    
	//glClearColor(r,g,b,1.0f);
	glClearColor(0.9, 0.8, 0.85, 1.0f);
}

void Viewer2DGL::resizeGL(int width, int height)
{
//	makeCurrent();
	glViewport(0, 0, (GLint)width, (GLint)height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glOrtho(0, width, -height, 0, 0, 1);

	glMatrixMode(GL_MODELVIEW);

	recalculatePosition();

	emit imageSizeChanged(mRenderWidth, mRenderHeight);

	updateScene();
	//paintGL();
}

void Viewer2DGL::updateScene()
{
	if (this->isVisible())
	{
		update();
	}
}

void Viewer2DGL::paintGL()
{
	QMutexLocker locker(&mutex);

    makeCurrent();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glRectf(-0.5f, -0.5f, 0.5f, 0.5f);

	if (!mRenderQtImg.isNull())
	{
		glPushMatrix();
		glLoadIdentity();
		{
			if (mResizedImg.width() <= 0)
			{
				if (mRenderWidth == mRenderQtImg.width() && mRenderHeight == mRenderQtImg.height())
					mResizedImg = mRenderQtImg;
				else
					mResizedImg = mRenderQtImg.scaled(QSize(mRenderWidth, mRenderHeight),
					Qt::IgnoreAspectRatio,
					Qt::SmoothTransformation);
			}

			// ---> Centering image in draw area

			glRasterPos2i(mRenderPosX, mRenderPosY);
			glPixelZoom(1, -1);
			glDrawPixels(mResizedImg.width(), mResizedImg.height(), GL_RGBA, GL_UNSIGNED_BYTE, mResizedImg.bits());

		}
		glPopMatrix();
		glFlush();
	}
	updateScene();	//这一行必须有 否则界面刷新会卡死
}

void Viewer2DGL::recalculatePosition()
{
    mImgRatio = (float)mOrigImage.cols/(float)mOrigImage.rows;

    mRenderWidth = this->size().width();
    mRenderHeight = floor(mRenderWidth / mImgRatio);

    if (mRenderHeight > this->size().height())
    {
        mRenderHeight = this->size().height();
        mRenderWidth = floor(mRenderHeight * mImgRatio);
    }

    mRenderPosX = floor((this->size().width() - mRenderWidth) / 2);
    mRenderPosY = -floor((this->size().height() - mRenderHeight) / 2);

    mResizedImg = QImage();
}

bool Viewer2DGL::setImage(const cv::Mat image)
{
	QMutexLocker locker(&mutex);

    if (image.channels() == 3)
        cvtColor(image, mOrigImage, CV_BGR2RGBA);
    else if (image.channels() == 1)
        cvtColor(image, mOrigImage, CV_GRAY2RGBA);
	else if (image.channels() == 4)
		mOrigImage = image;
    else return false;

    mRenderQtImg = QImage((const unsigned char*)(mOrigImage.data),
                          mOrigImage.cols, mOrigImage.rows,
                          mOrigImage.step1(), QImage::Format_RGB32);
	
    recalculatePosition();
    updateScene();
    return true;
}
