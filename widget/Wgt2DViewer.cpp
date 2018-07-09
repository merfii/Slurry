#include "Wgt2DViewer.h"

#include "RPatient.h"
#include "RSeries.h"
#include "RSlice.h"
#include "RROI.h"
#include "RPlan.h"
#include "RDose.h"
#include "RPath.h"
#include "RBeam.h"

#include <QContextMenuEvent>

// common interactions
#include "RCrossHairInteraction.h"
#include "RMeasureAreaInteraction.h"
#include "RMeasureLengthAngleInteraction.h"
#include "RZoomPanInteraction.h"
#include "RWWWLInteraction.h"

// register-related interactions
#include "RROIInteraction.h"
#include "RMarkerInteraction.h"
#include "RRegTranslateInteraction.h"
#include "RRegRotationInteraction.h"

// contour-related interactions
#include "RPolygonPointContourInteraction.h"
#include "RFreeHandPointContourInteraction.h"
#include "RSplinePointContourInteraction.h"
#include "RSmartContourPointInteraction.h"
#include "RBrushContourInteraction.h"
#include "RTranslateContour2DInteraction.h"
#include "RRotateContour2DInteraction.h"
#include "RScaleContour2DInteraction.h"
#include "RRegionGrowing2DInteraction.h"
#include "RRegionGrowing3DInteraction.h"
#include "RDeformContourInteraction.h"
#include "RDeformContour3DInteraction.h"

#include "RDoseDisplayManager.h"

// measure tools
#include <RMeasureTools>

// itk filters
#include <itkextractimagefilter.h>

#include <vtkImageImport.h>
#include <vtkMarchingSquares.h>
#include <vtkStripper.h>
#include <vtkCell.h>

#define COLOR_TEXT QColor(255, 119, 0, 255)

float Wgt2DViewer::m_Max_Scale = 20;
float Wgt2DViewer::m_Min_Scale = 1e-5;
float Wgt2DViewer::UnitLength[15] = { 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500 };

Wgt2DViewer::Wgt2DViewer(QWidget *parent, bool standalone/* = false*/)
	: RWidget(parent)
{
	setMouseTracking(true);
	initialize(standalone);
}

Wgt2DViewer::~Wgt2DViewer()
{

}

void Wgt2DViewer::initialize(bool standalone)
{
	// 设置资源文件名(外部调用.lib文件时不会出现加载资源失败的问题)
	Q_INIT_RESOURCE(rwidgets);

	m_PrimarySeries = NULL;
	m_SecondarySeries = NULL;
	m_Plan = NULL;
	m_Dose = NULL;

	m_DrawRectOffset = QPointF(0, 0);
	m_DrawRectScale = 1;

	m_ViewDirection = E_ViewDirection_AXIAL;

	m_Opacity = 1;

	m_Opacity = 0.5;
	m_FusionViewMode = E_FusionMode_Overlay;
	m_FusionView_GridSize = 512;
	m_FusionView_PatternFliped = false;

	// 初始化所有交互模式
	m_CrossHairInteraction = new RCrossHairInteraction;
	m_MeasureAreaInteraction = new RMeasureAreaInteraction;
	m_MeasureLengthAngleInteraction = new RMeasureLengthAngleInteraction;
	m_ZoomPanInteraction = new RZoomPanInteraction;
	m_WWWLInteraction = new RWWWLInteraction;

	m_ROIInteraction = new RROIInteraction;
	m_MarkerInteraction = new RMarkerInteraction;
	m_RegTranslateInteraction = new RRegTranslateInteraction;
	m_RegRotationInteraction = new RRegRotationInteraction;
	
	m_PolygonPointInteraction = new RPolygonPointContourInteraction;
	m_FreeHandPointInteraction = new RFreeHandPointContourInteraction;
	m_SmartContourPointInteraction = new RSmartContourPointInteraction;
	m_SplinePointInteraction = new RSplinePointContourInteraction;
	m_BrushInteraction = new RBrushContourInteraction;
	m_TranslateContour2DInteraction = new RTranslateContour2DInteraction;
	m_RotateContour2DInteraction = new RRotateContour2DInteraction;
	m_ScaleContour2DInteraction = new RScaleContour2DInteraction;
	m_RegionGrowing2DInteraction = new RRegionGrowing2DInteraction;
	m_DeformContourInteraction = new RDeformContourInteraction;
	m_DeformContour3DInteraction = new RDeformContour3DInteraction;

	m_CurrentInteraction = 0;

	m_ActCrossHair = new QAction(QIcon(tr(":/RWidgets/Resources/focus.png")), tr("Cross Hair"), this);
	m_ActCrossHair->setCheckable(true);
	connect(m_ActCrossHair, &QAction::triggered, this, &Wgt2DViewer::slot_Button_CrossHair_toggled);
	m_ActZoomPan = new QAction(QIcon(tr(":/RWidgets/Resources/zoom.png")), tr("Zoom/Pan"), this);
	m_ActZoomPan->setCheckable(true);
	connect(m_ActZoomPan, &QAction::triggered, this, &Wgt2DViewer::slot_Button_ZoomPan_toggled);
	m_ActWWWL = new QAction(QIcon(tr(":/RWidgets/Resources/contrast.png")), tr("Window Level"), this);
	m_ActWWWL->setCheckable(true);
	connect(m_ActWWWL, &QAction::triggered, this, &Wgt2DViewer::slot_Button_WWWL_toggled);
	m_ActMeasure = new QAction(QIcon(tr(":/RWidgets/Resources/measure.png")), tr("Measure"), this);
	m_ActMeasure->setCheckable(true);
	connect(m_ActMeasure, &QAction::triggered, this, &Wgt2DViewer::slot_Button_Measure_toggled);
	m_Button_CrossHair = new QToolButton(this);
	m_Button_CrossHair->setAutoRaise(true);
	m_Button_CrossHair->setDefaultAction(m_ActCrossHair);
	m_Button_ZoomPan = new QToolButton(this);
	m_Button_ZoomPan->setAutoRaise(true);
	m_Button_ZoomPan->setDefaultAction(m_ActZoomPan);
	m_Button_WWWL = new QToolButton(this);
	m_Button_WWWL->setAutoRaise(true);
	m_Button_WWWL->setDefaultAction(m_ActWWWL);
	m_Button_Measure = new QToolButton(this);
	m_Button_Measure->setAutoRaise(true);
	m_Button_Measure->setDefaultAction(m_ActMeasure);
	
	// 标志位，说明该窗口是否独立使用
	if (standalone)
	{
		connect(this, &Wgt2DViewer::sgnl_SynFusionViewMode, this, &Wgt2DViewer::slot_SetFusionViewMode);
		connect(this, &Wgt2DViewer::sgnl_SynSeries, this, &Wgt2DViewer::slot_SetSeries);
		connect(this, &Wgt2DViewer::sgnl_SynDose, this, &Wgt2DViewer::slot_SetDose);
		connect(this, &Wgt2DViewer::sgnl_SynInteractionMode, this, &Wgt2DViewer::slot_SetInteractionMode);
	}
	// 图像所在矩形的缩放，这个不用同步
	connect(this, &Wgt2DViewer::sgnl_SynDrawRectScale, this, &Wgt2DViewer::slot_SetDrawRectScale);

	// 初始化为 CrossHair 交互模式
	m_CurrentInteraction = m_CrossHairInteraction;
	m_CurrentInteractionMode = E_InteractionMode_CrossHair;

	// 初始化右键快捷菜单
	m_PopMenu = new QMenu();
	m_ActReset = new QAction(tr("Reset"), this);
	m_ActRegistTranslate = new QAction(tr("Translate"), this);
	m_ActRegistRotate = new QAction(tr("Rotate"), this);
	m_ActRegistCenterAlign = new QAction(tr("Center Align"), this);

	connect(m_ActReset, &QAction::triggered, this, &Wgt2DViewer::resetScaleAndOffset);

	m_MaxShowedDistance = 30.0;
	m_MinShowedDistance = 0.0;

	m_HidePrimarySeries = false;
}

void Wgt2DViewer::paintEvent(QPaintEvent * event)
{
	QPainter p(this);
	p.setFont(QFont("Calibri"));
	p.fillRect(this->rect(), QColor(26, 26, 26));

	// 没有参考图像，不执行绘制事件
	if (!m_PrimarySeries)
	{
		p.setPen(QPen(Qt::green));
		p.drawText(QPointF(width() * 0.5 - 100, height() * 0.5), QObject::tr("Primary Series has NOT been set."));
		return;
	}
	
	// 绘制参考图像
	paintPrimaryImage(p);

	// 绘制浮动图像
	paintSecondaryImage(p);

	// 绘制剂量图像
	paintDoseImage(p);

	// 绘制配准ROI/非配准模式下，用作计算框
	paintROI(p);

	// 绘制位移向量场
	paintDisplacementField(p);

	// 绘制射野剖面
	paintBeams(p);

	// 执行交互模式下的绘制事件
	paintInteractionEvent(p);

	// 绘制测量工具
	paintMeasureTools(p);

	// 绘制标记点
	paintMarkPoints(p);

	// 绘制轮廓线
	paintContours(p);
	
	// 绘制焦点十字线
	paintFocusLine(p);

	// 文本信息的绘制
	paintSeriesInfo(p);
}

void Wgt2DViewer::resizeEvent(QResizeEvent *event)
{
	// 根据窗口缩放比例调整显示比例
	if (m_PrimarySeries != NULL)
	{
		float scale_old = getFitToWindowScale(event->oldSize());
		float scale_new = getFitToWindowScale();
		if (scale_old <= 0)
			slot_SetDrawRectScale(scale_new);
		else
			slot_SetDrawRectScale(m_DrawRectScale * scale_new / scale_old);
	}

	m_Button_CrossHair->setGeometry(width() - 4 * 20 - 1, 1, 20, 20);
	m_Button_ZoomPan->setGeometry(width() - 3 * 20 - 1, 1, 20, 20);
	m_Button_WWWL->setGeometry(width() - 2 * 20 - 1, 1, 20, 20);
	m_Button_Measure->setGeometry(width() - 1 * 20 - 1, 1, 20, 20);

	this->update();
}

void Wgt2DViewer::mouseMoveEvent(QMouseEvent *event)
{
	if (m_PrimarySeries == NULL)
		return;
	m_CursorPos = event->pos();
	if (m_CurrentInteraction)
	{
		if (isContourInteraction(m_CurrentInteraction) && (!g_Patient || !g_Patient->getCurrentROI()))
		{
			return;
		}
		m_CurrentInteraction->mouseMoveEvent(this, event);
	}
	update();
}

void Wgt2DViewer::mousePressEvent(QMouseEvent *event)
{
	if (m_PrimarySeries == NULL)
		return;
	if (m_CurrentInteraction)
	{
		if (isContourInteraction(m_CurrentInteraction) && (!g_Patient || !g_Patient->getCurrentROI()))
		{
			return;
		}
		m_CurrentInteraction->mousePressEvent(this, event);
	}
	update();
}

void Wgt2DViewer::mouseDoubleClickEvent(QMouseEvent *event)
{
	return;
}

void Wgt2DViewer::mouseReleaseEvent(QMouseEvent *event)
{
	if (m_PrimarySeries == NULL)
		return;
	if (m_CurrentInteraction)
	{
		if (isContourInteraction(m_CurrentInteraction) && (!g_Patient || !g_Patient->getCurrentROI()))
		{
			return;
		}
		m_CurrentInteraction->mouseReleaseEvent(this, event);
	}
}

void Wgt2DViewer::wheelEvent(QWheelEvent *event)
{
	if (m_PrimarySeries == NULL)
		return;
	if (m_CurrentInteraction)
	{
		if (isContourInteraction(m_CurrentInteraction) && (!g_Patient || !g_Patient->getCurrentROI()))
		{
			return;
		}
		m_CurrentInteraction->wheelEvent(this, event);
	}
}

void Wgt2DViewer::keyPressEvent(QKeyEvent *event)
{
	if (m_PrimarySeries == NULL)
		return;
	if (m_CurrentInteraction)
	{
		m_CurrentInteraction->keyPressEvent(this, event);
	}
}

void Wgt2DViewer::enterEvent(QEvent *in)
{
	if (m_PrimarySeries == NULL)
		return;
	if (m_CurrentInteraction)
	{
		m_CurrentInteraction->enterEvent(this, in);
	}
	update();
}

void Wgt2DViewer::leaveEvent(QEvent *out)
{
	if (m_PrimarySeries == NULL)
		return;
	if (m_CurrentInteraction)
	{
		m_CurrentInteraction->leaveEvent(this, out);
	}
	update();
}

void Wgt2DViewer::contextMenuEvent(QContextMenuEvent *event)
{
	if (m_PrimarySeries == NULL)
		return;
	// 仅在 Cross Hair 模式中使用交互菜单
	if (m_CurrentInteraction != m_CrossHairInteraction/*&& isContourInteraction(m_CurrentInteraction)*/)
	{
		return;
	}
	//清除原有菜单
	m_PopMenu->clear();
	m_PopMenu->addAction(m_ActReset);

// 	m_PopMenu->addAction(m_ActRegistTranslate);
// 	m_PopMenu->addAction(m_ActRegistRotate);
// 	m_PopMenu->addAction(m_ActRegistCenterAlign);

	if (m_CurrentInteraction)
	{
		m_CurrentInteraction->contextMenuEvent(this, event);
	}

	if (event->isAccepted())
	{
		//菜单出现的位置为当前鼠标的位置
		m_PopMenu->exec(QCursor::pos());
	}
}

void Wgt2DViewer::slot_SetFusionViewMode(E_FusionMode v)
{
	m_FusionViewMode = v;
	this->update();
}

void Wgt2DViewer::slot_SetFusionViewGridSize(int v)
{
	m_FusionView_GridSize = v;
	this->update();
}

void Wgt2DViewer::slot_SetFusionViewPatternFliped(bool v)
{
	m_FusionView_PatternFliped = v;
	this->update();
}

void Wgt2DViewer::slot_SetOpacity(int percentage)
{
	m_Opacity = percentage * 0.01;
	this->update();
}

void Wgt2DViewer::slot_SetSeries(RSeries* primary, RSeries* secondary)
{
	if (m_PrimarySeries == primary && m_SecondarySeries == secondary)
	{
		return;
	}
	m_PrimarySeries = primary;
	if (primary == secondary)
		m_SecondarySeries = NULL;
	else
		m_SecondarySeries = secondary;
	if (m_PrimarySeries != NULL)
	{
		if (m_SecondarySeries != NULL)
		{
			m_SecondarySeries->setFocus(m_PrimarySeries->getFocus());
		}
	}
	
	m_Opacity = 0.5;
	slot_SetDrawRectScale(getFitToWindowScale());
	update();
}

void Wgt2DViewer::slot_SetDose(RDose* s)
{
	if (m_PrimarySeries == NULL)
		return;
	if (m_Dose == s)
		return;

	if (s == NULL || s->getParentPlan() == NULL)
	{
		m_Dose = NULL;
		m_Plan = NULL;
		this->update();
		return;
	}

	m_Dose = s;
	m_Plan = s->getParentPlan();

	m_Dose->setFocus(m_PrimarySeries->getFocus());

	this->update();
}

void Wgt2DViewer::slot_SetDrawRectScale(float scale)
{
	if (scale > Wgt2DViewer::m_Max_Scale)
		m_DrawRectScale = Wgt2DViewer::m_Max_Scale;
	else if (scale < Wgt2DViewer::m_Min_Scale)
		m_DrawRectScale = Wgt2DViewer::m_Min_Scale;
	else
		m_DrawRectScale = scale;
	this->update();
}

float Wgt2DViewer::getFitToWindowScale()
{
	return getFitToWindowScale(size());
}

float Wgt2DViewer::getFitToWindowScale(QSize windowSize)
{
	float scale = 1.0;
	if (m_PrimarySeries != NULL)
	{
		RVector3F spacing = m_PrimarySeries->getSpacing();
		RVector3I size = m_PrimarySeries->getSize();
		float phywidth, phyheight;
		switch (m_ViewDirection)
		{
		case E_ViewDirection_AXIAL:
			phywidth = spacing[0] * size[0];
			phyheight = spacing[1] * size[1];
			break;
		case E_ViewDirection_CORONAL:
			phywidth = spacing[0] * size[0];
			phyheight = spacing[2] * size[2];
			break;
		case E_ViewDirection_SAGITTAL:
			phywidth = spacing[1] * size[1];
			phyheight = spacing[2] * size[2];
			break;
		}
		float scale_h = windowSize.width() / phywidth;
		float scale_v = windowSize.height() / phyheight;
		scale = qMin(scale_h, scale_v);
	}
	return scale;
}

void Wgt2DViewer::resetScaleAndOffset()
{
	slot_SetDrawRectScale(getFitToWindowScale());
	slot_SetDrawRectOffset(QPointF(0, 0));
}

void Wgt2DViewer::generatePrimaryDrawRect()
{
	// 以 Fit To Window 的方式生成 Draw Rect
	if (!m_PrimarySeries)
	{
		return;
	}
	RVector3F spacing = m_PrimarySeries->getSpacing();
	RVector3I size = m_PrimarySeries->getSize();
	float phywidth, phyheight;
	switch (m_ViewDirection)
	{
	case E_ViewDirection_AXIAL:
		phywidth = spacing[0] * size[0];
		phyheight = spacing[1] * size[1];
		break;
	case E_ViewDirection_CORONAL:
		phywidth = spacing[0] * size[0];
		phyheight = spacing[2] * size[2];
		break;
	case E_ViewDirection_SAGITTAL:
		phywidth = spacing[1] * size[1];
		phyheight = spacing[2] * size[2];
		break;
	}
	m_PrimaryDrawRect = QRectF(
		width() * 0.5 - phywidth * 0.5 * m_DrawRectScale + m_DrawRectOffset.x() * m_DrawRectScale,
		height() * 0.5 - phyheight * 0.5 * m_DrawRectScale + m_DrawRectOffset.y() * m_DrawRectScale,
		phywidth * m_DrawRectScale,
		phyheight * m_DrawRectScale
		);
}

void Wgt2DViewer::generateSecondaryDrawRect()
{
	if (m_PrimarySeries == NULL || m_SecondarySeries == NULL)
		return;
	if (m_PrimarySeries->getSlice(m_ViewDirection) == NULL)
		return;
	if (m_SecondarySeries->getSlice(m_ViewDirection) == NULL)
		return;
	RVector2F spacing_p = m_PrimarySeries->getSlice(m_ViewDirection)->getSpacing();
	RVector2I size_p = m_PrimarySeries->getSlice(m_ViewDirection)->getSize();
	RVector3F origin_p = m_PrimarySeries->getSlice(m_ViewDirection)->getOrigin();
	RVector2F spacing_s = m_SecondarySeries->getSlice(m_ViewDirection)->getSpacing();
	RVector2I size_s = m_SecondarySeries->getSlice(m_ViewDirection)->getSize();
	RVector3F origin_s = m_SecondarySeries->getSlice(m_ViewDirection)->getOrigin();
	float phywidth_p = spacing_p[0] * size_p[0];
	float phyheight_p = spacing_p[1] * size_p[1];
	float phywidth_s = spacing_s[0] * size_s[0];
	float phyheight_s = spacing_s[1] * size_s[1];
	RVector2F originOffset;
	switch (m_ViewDirection)
	{
	case E_ViewDirection_AXIAL:
		originOffset[0] = origin_s[0] - origin_p[0];
		originOffset[1] = origin_s[1] - origin_p[1];
		break;
	case E_ViewDirection_CORONAL:
		originOffset[0] = origin_s[0] - origin_p[0];
		originOffset[1] = origin_p[2] - origin_s[2];
		break;
	case E_ViewDirection_SAGITTAL:
		originOffset[0] = origin_s[1] - origin_p[1];
		originOffset[1] = origin_p[2] - origin_s[2];
		break;
	}
	m_SecondaryDrawRect = QRectF(
		width() * 0.5 - phywidth_p * 0.5 * m_DrawRectScale + m_DrawRectOffset.x() * m_DrawRectScale + originOffset[0] * m_DrawRectScale,
		height() * 0.5 - phyheight_p * 0.5 * m_DrawRectScale + m_DrawRectOffset.y() * m_DrawRectScale + originOffset[1] * m_DrawRectScale,
		phywidth_s * m_DrawRectScale,
		phyheight_s * m_DrawRectScale
		);
}

void Wgt2DViewer::generateDoseDrawRect()
{
	if (!m_PrimarySeries || !m_Dose)
	{
		return;
	}
	RVector3F spacing_p = m_PrimarySeries->getSpacing();
	RVector3I size_p = m_PrimarySeries->getSize();
	RVector3F origin_p = m_PrimarySeries->getOrigin();
	RVector3F spacing_s = m_Dose->getSpacing();
	RVector3I size_s = m_Dose->getSize();
	RVector3F origin_s = m_Dose->getOrigin();
	RVector2F originOffset;
	float phywidth_p, phyheight_p, phywidth_s, phyheight_s;
	switch (m_ViewDirection)
	{
	case E_ViewDirection_AXIAL:
		phywidth_p = spacing_p[0] * size_p[0];
		phyheight_p = spacing_p[1] * size_p[1];
		phywidth_s = spacing_s[0] * size_s[0];
		phyheight_s = spacing_s[1] * size_s[1];
		originOffset[0] = origin_s[0] - origin_p[0];
		originOffset[1] = origin_s[1] - origin_p[1];
		break;
	case E_ViewDirection_CORONAL:
		phywidth_p = spacing_p[0] * size_p[0];
		phyheight_p = spacing_p[2] * size_p[2];
		phywidth_s = spacing_s[0] * size_s[0];
		phyheight_s = spacing_s[2] * size_s[2];
		originOffset[0] = origin_s[0] - origin_p[0];
		originOffset[1] = (origin_p[2] + phyheight_p) - (origin_s[2] + phyheight_s);
		break;
	case E_ViewDirection_SAGITTAL:
		phywidth_p = spacing_p[1] * size_p[1];
		phyheight_p = spacing_p[2] * size_p[2];
		phywidth_s = spacing_s[1] * size_s[1];
		phyheight_s = spacing_s[2] * size_s[2];
		originOffset[0] = origin_s[1] - origin_p[1];
		originOffset[1] = (origin_p[2] + phyheight_p) - (origin_s[2] + phyheight_s);
		break;
	}
	m_DoseDrawRect = QRectF(
		width() * 0.5 - phywidth_p * 0.5 * m_DrawRectScale + m_DrawRectOffset.x() * m_DrawRectScale + originOffset[0] * m_DrawRectScale,
		height() * 0.5 - phyheight_p * 0.5 * m_DrawRectScale + m_DrawRectOffset.y() * m_DrawRectScale + originOffset[1] * m_DrawRectScale,
		phywidth_s * m_DrawRectScale,
		phyheight_s * m_DrawRectScale
		);
}

void Wgt2DViewer::screen2PrimaryIndex(const QPointF &screen, QPointF &image, bool &resultvalid)
{
	if (m_PrimarySeries == NULL || m_PrimarySeries->getSlice(m_ViewDirection) == NULL || !m_PrimaryDrawRect.isValid())
	{
		resultvalid = false;
		return;
	}
	RVector2I size = m_PrimarySeries->getSlice(m_ViewDirection)->getSize();
	QPointF posInRect = screen - m_PrimaryDrawRect.topLeft();

	image.setX(size[0] * posInRect.x() / m_PrimaryDrawRect.width() - 0.5);
	image.setY(size[1] * posInRect.y() / m_PrimaryDrawRect.height() - 0.5);
	resultvalid = true;
	//switch (m_ViewDirection)
	//{
	//case E_ViewDirection_AXIAL:
	//	image.setX(size[0] * posInRect.x() / m_PrimaryDrawRect.width() - 0.5);
	//	image.setY(size[1] * posInRect.y() / m_PrimaryDrawRect.height() - 0.5);
	//	resultvalid = true;
	//	break;
	//case E_ViewDirection_CORONAL:
	//case E_ViewDirection_SAGITTAL:
	//	image.setX(size[0] * posInRect.x() / m_PrimaryDrawRect.width() - 0.5);
	//	image.setY(size[1] * (m_PrimaryDrawRect.height() - posInRect.y()) / m_PrimaryDrawRect.height() - 0.5);
	//	resultvalid = true;
	//	break;
	//default:
	//	resultvalid = false;
	//	break;
	//}
}

void Wgt2DViewer::screen2SecondaryIndex(const QPointF &screen, QPointF &image, bool &resultvalid)
{
	if (m_SecondarySeries == NULL || m_SecondarySeries->getSlice(m_ViewDirection) == NULL || !m_SecondaryDrawRect.isValid())
	{
		resultvalid = false;
		return;
	}
	RVector2I size = m_SecondarySeries->getSlice(m_ViewDirection)->getSize();
	QPointF posInRect = screen - m_SecondaryDrawRect.topLeft();

	image.setX(size[0] * posInRect.x() / m_SecondaryDrawRect.width() - 0.5);
	image.setY(size[1] * posInRect.y() / m_SecondaryDrawRect.height() - 0.5);
	resultvalid = true;
	//switch (m_ViewDirection)
	//{
	//case E_ViewDirection_AXIAL:
	//	image.setX(size[0] * posInRect.x() / m_SecondaryDrawRect.width() - 0.5);
	//	image.setY(size[1] * posInRect.y() / m_SecondaryDrawRect.height() - 0.5);
	//	resultvalid = true;
	//	break;
	//case E_ViewDirection_CORONAL:
	//case E_ViewDirection_SAGITTAL:
	//	image.setX(size[0] * posInRect.x() / m_SecondaryDrawRect.width() - 0.5);
	//	image.setY(size[1] * (m_SecondaryDrawRect.height() - posInRect.y()) / m_SecondaryDrawRect.height() - 0.5);
	//	resultvalid = true;
	//	break;
	//default:
	//	resultvalid = false;
	//	break;
	//}
}

void Wgt2DViewer::screen2SecondaryPoint(const QPointF &screen, QPointF &image, bool &resultvalid)
{
	if (m_SecondarySeries == NULL || m_SecondarySeries->getSlice(m_ViewDirection) == NULL)
	{
		resultvalid = false;
		return;
	}
	bool ok;
	QPointF imageIndex;
	screen2SecondaryIndex(screen, imageIndex, ok);
	image.setX(imageIndex.x() * m_SecondarySeries->getSlice(m_ViewDirection)->getSpacing()[0]);
	image.setY(imageIndex.y() * m_SecondarySeries->getSlice(m_ViewDirection)->getSpacing()[1]);
}

void Wgt2DViewer::screen2DoseIndex(const QPointF &screen, QPointF &image, bool &resultvalid)
{
	if (m_Dose == NULL || m_Dose->getSlice(m_ViewDirection) == NULL || !m_DoseDrawRect.isValid())
	{
		resultvalid = false;
		return;
	}
	RVector2I size = m_Dose->getSlice(m_ViewDirection)->getSize();
	QPointF posInRect = screen - m_DoseDrawRect.topLeft();

	image.setX(size[0] * posInRect.x() / m_DoseDrawRect.width() - 0.5);
	image.setY(size[1] * posInRect.y() / m_DoseDrawRect.height() - 0.5);
	resultvalid = true;
	//switch (m_ViewDirection)
	//{
	//case E_ViewDirection_AXIAL:
	//	image.setX(size[0] * posInRect.x() / m_DoseDrawRect.width() - 0.5);
	//	image.setY(size[1] * posInRect.y() / m_DoseDrawRect.height() - 0.5);
	//	resultvalid = true;
	//	break;
	//case E_ViewDirection_CORONAL:
	//case E_ViewDirection_SAGITTAL:
	//	image.setX(size[0] * posInRect.x() / m_DoseDrawRect.width() - 0.5);
	//	image.setY(size[1] * (m_DoseDrawRect.height() - posInRect.y()) / m_DoseDrawRect.height() - 0.5);
	//	resultvalid = true;
	//	break;
	//default:
	//	resultvalid = false;
	//	break;
	//}
}

void Wgt2DViewer::screen2DosePoint(const QPointF &screen, QPointF &image, bool &resultvalid)
{
	if (m_Dose == NULL || m_Dose->getSlice(m_ViewDirection) == NULL)
	{
		resultvalid = false;
		return;
	}
	bool ok;
	QPointF imageIndex;
	screen2DoseIndex(screen, imageIndex, ok);
	image.setX(imageIndex.x() * m_Dose->getSlice(m_ViewDirection)->getSpacing()[0]);
	image.setY(imageIndex.y() * m_Dose->getSlice(m_ViewDirection)->getSpacing()[1]);
}

void Wgt2DViewer::primaryIndex2Screen(const QPointF &image, QPointF &screen, bool &resultvalid)
{
	if (!m_PrimarySeries || !m_PrimarySeries->getSlice(m_ViewDirection) || !m_PrimaryDrawRect.isValid())
	{
		resultvalid = false;
		return;
	}
	RVector2I size = m_PrimarySeries->getSlice(m_ViewDirection)->getSize();

	float scrX = 0;
	float scrY = 0;

	scrX = m_PrimaryDrawRect.width() * (image.x() + 0.5) / size[0];
	scrY = m_PrimaryDrawRect.height() * (image.y() + 0.5) / size[1];
	
	screen.setX(scrX + m_PrimaryDrawRect.left());
	screen.setY(scrY + m_PrimaryDrawRect.top());
	resultvalid = true;
}

void Wgt2DViewer::secondaryIndex2Screen(const QPointF &image, QPointF &screen, bool &resultvalid)
{
	if (!m_SecondarySeries || !m_SecondarySeries->getSlice(m_ViewDirection) || !m_SecondaryDrawRect.isValid())
	{
		resultvalid = false;
		return;
	}
	RVector2I size = m_SecondarySeries->getSlice(m_ViewDirection)->getSize();

	float scrX = 0;
	float scrY = 0;

	scrX = m_SecondaryDrawRect.width() * (image.x() + 0.5) / size[0];
	scrY = m_SecondaryDrawRect.height() * (image.y() + 0.5) / size[1];
	//switch (m_ViewDirection)
	//{
	//case E_ViewDirection_AXIAL:
	//	scrX = m_SecondaryDrawRect.width() * (image.x() + 0.5) / size[0];
	//	scrY = m_SecondaryDrawRect.height() * (image.y() + 0.5) / size[1];
	//	break;
	//case E_ViewDirection_CORONAL:
	//case E_ViewDirection_SAGITTAL:
	//	scrX = m_SecondaryDrawRect.width() * (image.x() + 0.5) / size[0];
	//	scrY = m_SecondaryDrawRect.height() * (size[2] - (image.y() + 0.5)) / size[2];
	//	break;
	//}

	screen.setX(scrX + m_SecondaryDrawRect.left());
	screen.setY(scrY + m_SecondaryDrawRect.top());
	resultvalid = true;
}

void Wgt2DViewer::doseIndex2Screen(const QPointF &image, QPointF &screen, bool &resultvalid)
{
	if (!m_Dose || !m_Dose->getSlice(m_ViewDirection) || !m_DoseDrawRect.isValid())
	{
		resultvalid = false;
		return;
	}
	const RVector2I &size = m_Dose->getSlice(m_ViewDirection)->getSize();

	float scrX = 0;
	float scrY = 0;

	scrX = m_DoseDrawRect.width() * (image.x() + 0.5) / size[0];
	scrY = m_DoseDrawRect.height() * (image.y() + 0.5) / size[1];
	//switch (m_ViewDirection)
	//{
	//case E_ViewDirection_AXIAL:
	//	scrX = m_DoseDrawRect.width() * (image.x() + 0.5) / size[0];
	//	scrY = m_DoseDrawRect.height() * (image.y() + 0.5) / size[1];
	//	break;
	//case E_ViewDirection_CORONAL:
	//case E_ViewDirection_SAGITTAL:
	//	scrX = m_DoseDrawRect.width() * (image.x() + 0.5) / size[0];
	//	scrY = m_DoseDrawRect.height() * (size[2] - (image.y() + 0.5)) / size[2];
	//	break;
	//}

	screen.setX(scrX + m_DoseDrawRect.left());
	screen.setY(scrY + m_DoseDrawRect.top());
	resultvalid = true;
}

void Wgt2DViewer::screen2SpacePoint(const QPointF &screen, RVector3F &space, bool &resultvalid)
{
	if (m_PrimarySeries != NULL && m_PrimarySeries->getSlice(m_ViewDirection) != NULL)
	{
		QPointF imagePoint;
		screen2PrimaryIndex(screen, imagePoint, resultvalid);
		RVector3F imageIndex;
		switch (m_ViewDirection)
		{
		case E_ViewDirection_AXIAL:
			imageIndex[0] = imagePoint.x();
			imageIndex[1] = imagePoint.y();
			imageIndex[2] = m_PrimarySeries->getSlice(m_ViewDirection)->getSliceIndex();
			break;
		case E_ViewDirection_CORONAL:
			imageIndex[0] = imagePoint.x();
			imageIndex[1] = m_PrimarySeries->getSlice(m_ViewDirection)->getSliceIndex();
			imageIndex[2] = m_PrimarySeries->getSize()[2] - 1 - imagePoint.y();
			break;
		case E_ViewDirection_SAGITTAL:
			imageIndex[0] = m_PrimarySeries->getSlice(m_ViewDirection)->getSliceIndex();
			imageIndex[1] = imagePoint.x();
			imageIndex[2] = m_PrimarySeries->getSize()[2] - 1 - imagePoint.y();
			break;
		}
		m_PrimarySeries->imageIndex2SpacePoint(imageIndex, space);
	}
	else
	{
		resultvalid = false;
	}
}

void Wgt2DViewer::spacePoint2Screen(const RVector3F &space, QPointF &screen, bool &resultvalid)
{
	if (m_PrimarySeries != NULL)
	{
		RVector3F imageIndex;
		m_PrimarySeries->spacePoint2ImageIndex(space, imageIndex);
		QPointF imagePoint;
		switch (m_ViewDirection)
		{
		case E_ViewDirection_AXIAL:
			imagePoint.setX(imageIndex[0]);
			imagePoint.setY(imageIndex[1]);
			break;
		case E_ViewDirection_CORONAL:
			imagePoint.setX(imageIndex[0]);
			imagePoint.setY(m_PrimarySeries->getSize()[2] - 1 - imageIndex[2]);
			break;
		case E_ViewDirection_SAGITTAL:
			imagePoint.setX(imageIndex[1]);
			imagePoint.setY(m_PrimarySeries->getSize()[2] - 1 - imageIndex[2]);
			break;
		}
		primaryIndex2Screen(imagePoint, screen, resultvalid);
	}
	else
	{
		resultvalid = false;
	}
}

void Wgt2DViewer::slot_SetSeriesFocus(RVector3F focus)
{
	if (m_PrimarySeries != NULL)
	{
		m_PrimarySeries->setFocus(focus);
		if (m_SecondarySeries != NULL)
			m_SecondarySeries->setFocus(focus);
		if (m_Dose != NULL)
			m_Dose->setFocus(focus);
		updateAllView();
	}
}

void Wgt2DViewer::slot_SynSeries(RSeries* primary, RSeries* secondary)
{
	emit sgnl_SynSeries(primary, secondary);
}

void Wgt2DViewer::slot_SynDose(RDose* s)
{
	emit sgnl_SynDose(s);
}

void Wgt2DViewer::slot_SynFusionViewMode(E_FusionMode v)
{
	emit sgnl_SynFusionViewMode(v);
}

void Wgt2DViewer::slot_SynFusionViewGridSize(int v)
{
	emit sgnl_SynFusionViewGridSize(v);
}

void Wgt2DViewer::slot_SynFusionViewPatternFliped(bool v)
{
	emit sgnl_SynFusionViewPatternFliped(v);
}

void Wgt2DViewer::slot_SynDrawRectScale(float scale)
{
	emit sgnl_SynDrawRectScale(scale);
}

void Wgt2DViewer::slot_SetDrawRectOffset(QPointF offset)
{
	m_DrawRectOffset = offset;
	this->update();
}

void Wgt2DViewer::slot_SetInteractionMode(E_InteractionMode v)
{
	if (v != m_CurrentInteractionMode && m_CurrentInteraction)
	{
		m_CurrentInteraction->terminate(this);
	}
	switch (v)
	{
		//////////////////////////////////////////////////////////////////////////
		// common interactions
	case E_InteractionMode_CrossHair:
		m_CurrentInteraction = m_CrossHairInteraction;
		break;
	case E_InteractionMode_MeasureArea:
		m_CurrentInteraction = m_MeasureAreaInteraction;
		break;
	case E_InteractionMode_MeasureLength:
		m_CurrentInteraction = m_MeasureLengthAngleInteraction;
		break;
	case E_InteractionMode_ZoomPan:
		m_CurrentInteraction = m_ZoomPanInteraction;
		break;
	case E_InteractionMode_WWWL:
		m_CurrentInteraction = m_WWWLInteraction;
		break;
		//////////////////////////////////////////////////////////////////////////
		// register-related interactions
	case E_InteractionMode_RegistROI:
		m_CurrentInteraction = m_ROIInteraction;
		break;
	case E_InteractionMode_RegistMarker:
		m_CurrentInteraction = m_MarkerInteraction;
		break;
	case E_InteractionMode_RegistTranslate:
		m_CurrentInteraction = m_RegTranslateInteraction;
		break;
	case E_InteractionMode_RegistRotate:
		m_CurrentInteraction = m_RegRotationInteraction;
		break;
		//////////////////////////////////////////////////////////////////////////
		// contour-related interactions
	case E_InteractionMode_Polygon:
		m_CurrentInteraction = m_PolygonPointInteraction;
		break;
	case E_InteractionMode_FreeHand:
		m_CurrentInteraction = m_FreeHandPointInteraction;
		break;
	case E_InteractionMode_Spline:
		m_CurrentInteraction = m_SplinePointInteraction;
		break;
	case E_InteractionMode_SmartContour:
		m_CurrentInteraction = m_SmartContourPointInteraction;
		break;
	case E_InteractionMode_Brush:
		m_CurrentInteraction = m_BrushInteraction;
		break;
	case E_InteractionMode_TranslateContour:
		m_CurrentInteraction = m_TranslateContour2DInteraction;
		break;
	case E_InteractionMode_RotateContour:
		m_CurrentInteraction = m_RotateContour2DInteraction;
		break;
	case E_InteractionMode_ScaleContour:
		m_CurrentInteraction = m_ScaleContour2DInteraction;
		break;
	case E_InteractionMode_2DRegionGrow:
		m_CurrentInteraction = m_RegionGrowing2DInteraction;
		break;
	case E_InteractionMode_3DRegionGrow:
		m_CurrentInteraction = m_RegionGrowing3DInteraction;
		break;
	case E_InteractionMode_DeformContour:
		m_CurrentInteraction = m_DeformContourInteraction;
		break;
	case E_InteractionMode_DeformContour3D:
		m_CurrentInteraction = m_DeformContour3DInteraction;
		break;
		//////////////////////////////////////////////////////////////////////////
	case E_InteractionMode_Invalid:
		m_CurrentInteraction = 0;
		break;
	default:
		break;
	}
	if (m_CurrentInteraction != NULL)
		m_CurrentInteraction->initialize(this);
	m_CurrentInteractionMode = v;
	m_ActCrossHair->setChecked(m_CurrentInteraction == m_CrossHairInteraction);
	m_ActZoomPan->setChecked(m_CurrentInteraction == m_ZoomPanInteraction);
	m_ActWWWL->setChecked(m_CurrentInteraction == m_WWWLInteraction);
	m_ActMeasure->setChecked(m_CurrentInteraction == m_MeasureLengthAngleInteraction);
	update();
}

void Wgt2DViewer::slot_SynInteractionMode(E_InteractionMode mode, bool v)
{
	emit sgnl_SynInteractionMode(mode, v);
}

void Wgt2DViewer::slot_ChangeToNearestIndexSlice_Previous()
{
	if (!m_PrimarySeries)
	{
		return;
	}
	m_PrimarySeries->moveFocusToLastSlice(m_ViewDirection);
	if (m_SecondarySeries != NULL)
		m_SecondarySeries->setFocus(m_PrimarySeries->getFocus());
	if (m_Dose != NULL)
		m_Dose->setFocus(m_PrimarySeries->getFocus());
	updateAllView();
}

void Wgt2DViewer::slot_ChangeToNearestIndexSlice_Next()
{
	if (!m_PrimarySeries)
	{
		return;
	}
	m_PrimarySeries->moveFocusToNextSlice(m_ViewDirection);
	if (m_SecondarySeries != NULL)
		m_SecondarySeries->setFocus(m_PrimarySeries->getFocus());
	if (m_Dose != NULL)
		m_Dose->setFocus(m_PrimarySeries->getFocus());
	updateAllView();
}

RSlice * Wgt2DViewer::getPrimaryRawSlice()
{
	return m_PrimarySeries->getSlice(m_ViewDirection);
}

void Wgt2DViewer::addMeasureAreaObj(RMeasureAreaObj* &objs)
{
	if (objs)
	{
		m_MeasureAreaObjs.append(objs);
	}
}

QVector<RMeasureAreaObj*> Wgt2DViewer::getMeasureAreaObjs()
{
	return m_MeasureAreaObjs;
}

void Wgt2DViewer::removeMeasureAreaObj(int index)
{
	if (index >= 0 && index < m_MeasureAreaObjs.size())
	{
		delete m_MeasureAreaObjs[index];
		m_MeasureAreaObjs[index] = 0;
		m_MeasureAreaObjs.remove(index);
	}
}

void Wgt2DViewer::clearMeasureAreaObjs()
{
	for (int i = 0; i < m_MeasureAreaObjs.size(); ++i)
	{
		delete m_MeasureAreaObjs[i];
		m_MeasureAreaObjs[i] = 0;
	}
	m_MeasureAreaObjs.clear();
}

void Wgt2DViewer::addMeasureLengAngleObj(RMeasureLengthAngleObj*obj)
{
	if (obj)
	{
		m_MeasureLengAngleObjs.append(obj);
	}
}

void Wgt2DViewer::removeMeasureLengAngleObj(int index)
{
	if (index >= 0 && index < m_MeasureLengAngleObjs.size())
	{
		delete m_MeasureLengAngleObjs[index];
		m_MeasureLengAngleObjs[index] = 0;
		m_MeasureLengAngleObjs.remove(index);
	}
}

void Wgt2DViewer::clearMeasureLengAngleObjs()
{
	for (int i = 0; i < m_MeasureLengAngleObjs.size(); ++i)
	{
		delete m_MeasureLengAngleObjs[i];
		m_MeasureLengAngleObjs[i] = 0;
	}
	m_MeasureLengAngleObjs.clear();
}

QVector<RMeasureLengthAngleObj*> Wgt2DViewer::getMeasureLengAngleObjs()
{
	return m_MeasureLengAngleObjs;
}

void Wgt2DViewer::addMeasureCurveObj(RMeasureLengthAngleObj*obj)
{
	if (obj)
	{
		m_MeasureCurveObjs.append(obj);
	}
}

void Wgt2DViewer::removeMeasureCurveObj(int index)
{
	if (index >= 0 && index < m_MeasureCurveObjs.size())
	{
		delete m_MeasureCurveObjs[index];
		m_MeasureCurveObjs[index] = 0;
		m_MeasureCurveObjs.remove(index);
	}
}

void Wgt2DViewer::clearMeasureCurveObjs()
{
	for (int i = 0; i < m_MeasureCurveObjs.size(); ++i)
	{
		delete m_MeasureCurveObjs[i];
		m_MeasureCurveObjs[i] = 0;
	}
	m_MeasureCurveObjs.clear();
}

QVector<RMeasureLengthAngleObj*> Wgt2DViewer::getMeasureCurveObjs()
{
	return m_MeasureCurveObjs;
}

void Wgt2DViewer::paintPrimaryImage(QPainter& painter)
{
	if (m_PrimarySeries == NULL)
		return;
	painter.save();
	painter.setRenderHint(QPainter::SmoothPixmapTransform);
	m_PrimarySeries->updateSlices(m_PrimarySeries->getFocus());
	generatePrimaryDrawRect();
	QImage* primaryDrawImage = m_PrimarySeries->getDrawImage(m_ViewDirection);
	if (primaryDrawImage == NULL)
	{
		painter.restore();
		return;
	}
	switch (m_FusionViewMode)
	{
		// 叠加显示
	case E_FusionMode_Overlay:
	{
		painter.setOpacity(1);
		if (m_HidePrimarySeries)
		{
			painter.setOpacity(0);
		}
		break;
	}
		// 网格显示
	case E_FusionMode_Chessboard:
	case E_FusionMode_HorizontalLine:
	case E_FusionMode_VerticalLine:
	{
		break;
	}
	}
	painter.drawImage(m_PrimaryDrawRect, *primaryDrawImage);
	if (m_PrimarySeries->getMaskVisible())
	{
		painter.drawImage(m_PrimaryDrawRect, *(m_PrimarySeries->getMaskImage(m_ViewDirection)));
	}
	painter.restore();
}

void Wgt2DViewer::paintSecondaryImage(QPainter& painter)
{
	if (m_SecondarySeries == NULL)
		return;
	painter.save();
	painter.setRenderHint(QPainter::SmoothPixmapTransform);
	m_SecondarySeries->updateSlices(m_PrimarySeries->getFocus());
	generateSecondaryDrawRect();
	QImage* secondaryDrawImage = m_SecondarySeries->getDrawImage(m_ViewDirection);
	if (secondaryDrawImage == NULL)
	{
		painter.restore();
		return;
	}
	switch (m_FusionViewMode)
	{
		// 叠加显示
	case E_FusionMode_Overlay:
	{
		painter.setOpacity(1 - m_Opacity);
		if (m_HidePrimarySeries)
		{
			painter.setOpacity(1);
		}
		painter.drawImage(m_SecondaryDrawRect, *secondaryDrawImage);
		break;
	}
		// 网格显示
	case E_FusionMode_Chessboard:
	case E_FusionMode_HorizontalLine:
	case E_FusionMode_VerticalLine:
	{
		float gridWidth, gridHeight;
		int gridColumn, gridRow;
		switch (m_FusionViewMode)
		{
			// 棋盘显示
		case E_FusionMode_Chessboard:
			gridWidth = m_FusionView_GridSize;
			gridHeight = m_FusionView_GridSize;
			gridColumn = width() / m_FusionView_GridSize + 1;
			gridRow = height() / m_FusionView_GridSize + 1;
			break;
			// 水平网格显示
		case E_FusionMode_HorizontalLine:
			gridWidth = float(width());
			gridHeight = m_FusionView_GridSize;
			gridColumn = 1;
			gridRow = height() / m_FusionView_GridSize + 1;
			break;
			// 竖直网格显示
		case E_FusionMode_VerticalLine:
			gridWidth = m_FusionView_GridSize;
			gridHeight = float(height());
			gridColumn = width() / m_FusionView_GridSize + 1;
			gridRow = 1;
			break;
		}
#pragma omp parallel for
		for (int i = 0; i < gridRow; i++)
		{
			for (int j = 0; j < gridColumn; j++)
			{
				if ((i + j) % 2 == (int)m_FusionView_PatternFliped)
				{
					bool isValid;
					QPointF imageRectTopLeft, imageRectBottomRight;
					QRectF windowRect = QRectF(0.0 + j * gridWidth, 0.0 + i * gridHeight, gridWidth, gridHeight);
					screen2SecondaryIndex(windowRect.topLeft(), imageRectTopLeft, isValid);
					screen2SecondaryIndex(windowRect.bottomRight(), imageRectBottomRight, isValid);
					QRectF imageRect;
					imageRect.setLeft(qMin(imageRectTopLeft.x(), imageRectBottomRight.x()) + 0.5);
					imageRect.setTop(qMin(imageRectTopLeft.y(), imageRectBottomRight.y()) + 0.5);
					imageRect.setWidth(fabs(imageRectTopLeft.x() - imageRectBottomRight.x()));
					imageRect.setHeight(fabs(imageRectTopLeft.y() - imageRectBottomRight.y()));
					painter.drawImage(windowRect, *secondaryDrawImage, imageRect);
				}
			}
		}
		break;
	}
	}
	painter.restore();
}

void Wgt2DViewer::paintDoseImage(QPainter& painter)
{
	if (m_Dose == NULL || m_Plan == NULL)
		return;
	painter.save();
	m_Dose->updateSlices(m_PrimarySeries->getFocus());
	generateDoseDrawRect();
	QImage* doseDrawImage = m_Dose->getDrawImage(m_ViewDirection);
	if (doseDrawImage == NULL)
	{
		painter.restore();
		return;
	}
	painter.setOpacity(0.4);
	painter.setRenderHint(QPainter::SmoothPixmapTransform);
	painter.drawImage(m_DoseDrawRect, *doseDrawImage);
		
	painter.setOpacity(1.0);
	painter.setRenderHint(QPainter::Antialiasing);
	int rectHeight = 200;
	int rectWidth = 6;
	int rectTop = 50;
	int rectLeft = width() - 20;
	QLinearGradient brushGradient(rectLeft, rectTop, rectLeft + rectWidth, rectTop + rectHeight);
	int keyPointCount = G_ColorTableKeyPoint[E_ColorTableType_Dose].size();
	for (int i = 0; i < keyPointCount; ++i)
	{
		int colorIndex = 255 * (G_ColorTableKeyPoint[E_ColorTableType_Dose][i] - G_ColorTableKeyPoint[E_ColorTableType_Dose].first()) / (G_ColorTableKeyPoint[E_ColorTableType_Dose].last() - G_ColorTableKeyPoint[E_ColorTableType_Dose].first());
		QColor color = QColor(G_ColorTable[E_ColorTableType_Dose][0][colorIndex], G_ColorTable[E_ColorTableType_Dose][1][colorIndex], G_ColorTable[E_ColorTableType_Dose][2][colorIndex]);
		painter.setPen(Qt::white);
		painter.setFont(QFont("Calibri", 8));
		painter.drawText(QRect(rectLeft - 22, rectTop + rectHeight - i * rectHeight / (keyPointCount - 1) - 18, 20, 20), Qt::AlignRight | Qt::AlignBottom, tr("%1").arg(G_ColorTableKeyPoint[E_ColorTableType_Dose][i] * 100, 0, 'f', 0));
		brushGradient.setColorAt(1.0 - float(i) / (keyPointCount - 1), color);
		painter.setPen(color);
		painter.drawLine(rectLeft - 2, rectTop + rectHeight - i * rectHeight / (keyPointCount - 1), rectLeft + rectWidth - 1, rectTop + rectHeight - i * rectHeight / (keyPointCount - 1));
	}
	painter.setPen(Qt::NoPen);
	painter.setBrush(brushGradient);
	painter.drawRect(QRect(rectLeft, rectTop, rectWidth, rectHeight));

	painter.restore();
}

void Wgt2DViewer::paintDisplacementField(QPainter& painter)
{
	if (m_PrimarySeries == NULL)
		return;
	if (m_SecondarySeries == NULL)
		return;
	if (!m_SecondarySeries->getDisplacementFieldVisible())
		return;
	if (m_SecondarySeries->getDisplacementFieldSlice(m_ViewDirection).IsNull())
		return;

	painter.save();
	painter.setRenderHint(QPainter::Antialiasing);
	Displacement2DFieldPointer displacementSlice = m_SecondarySeries->getDisplacementFieldSlice(m_ViewDirection);
	Displacement2DFieldType::SizeType fieldSize = displacementSlice->GetLargestPossibleRegion().GetSize();
	QPointF imagePoint;
	QPointF startPoint, endPoint;
	QVector2D arrowVector0, arrowVector1, arrowVector2;
	bool isValid;
	VectorPixelType* buffer = displacementSlice->GetBufferPointer();
	VectorPixelType fieldVector;
	int gridSize = 20;
	for (int i = 0; i < fieldSize[1]; i += gridSize)
	{
		for (int j = 0; j < fieldSize[0]; j += gridSize)
		{
			fieldVector = buffer[i * fieldSize[1] + j];
			switch (m_ViewDirection)
			{
			case E_ViewDirection_AXIAL:
				imagePoint.setX(j);
				imagePoint.setY(i);
				arrowVector0.setX(fieldVector[0]);
				arrowVector0.setY(fieldVector[1]);
				break;
			case E_ViewDirection_CORONAL:
				imagePoint.setX(j);
				imagePoint.setY(fieldSize[1] - i - 1);
				arrowVector0.setX(fieldVector[0]);
				arrowVector0.setY(fieldVector[2]);
				break;
			case E_ViewDirection_SAGITTAL:
				imagePoint.setX(j);
				imagePoint.setY(fieldSize[1] - i - 1);
				arrowVector0.setX(fieldVector[1]);
				arrowVector0.setY(fieldVector[2]);
				break;
			}
			float vectorLength = arrowVector0.length();
			int colorIndex;
			if (vectorLength < m_MinShowedDistance)
				colorIndex = 0;
			else if (vectorLength > m_MaxShowedDistance)
				colorIndex = 255;
			else
				colorIndex = (vectorLength - m_MinShowedDistance) * 255 / (m_MaxShowedDistance - m_MinShowedDistance);
			painter.setPen(QPen(QColor(
				G_ColorTable[E_ColorTableType_DisplacementField][0][colorIndex], 
				G_ColorTable[E_ColorTableType_DisplacementField][1][colorIndex], 
				G_ColorTable[E_ColorTableType_DisplacementField][2][colorIndex]), 1, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
			arrowVector0.normalize();
			arrowVector0 *= 20;
			primaryIndex2Screen(imagePoint, startPoint, isValid);
			if (QRectF(rect()).contains(startPoint))
			{
				endPoint = startPoint + arrowVector0.toPointF();
				arrowVector1.setX(arrowVector0.x() * (-0.9397) - arrowVector0.y() * (-0.3420));
				arrowVector1.setY(arrowVector0.x() * (-0.3420) + arrowVector0.y() * (-0.9397));
				arrowVector1.normalize();
				arrowVector1 *= 8;
				arrowVector2.setX(arrowVector0.x() * (-0.9397) - arrowVector0.y() * (0.3420));
				arrowVector2.setY(arrowVector0.x() * (0.3420) + arrowVector0.y() * (-0.9397));
				arrowVector2.normalize();
				arrowVector2 *= 8;
				painter.drawLine(endPoint, endPoint + arrowVector1.toPointF());
				painter.drawLine(endPoint, endPoint + arrowVector2.toPointF());
				painter.drawLine(startPoint, endPoint);
			}
		}
	}

	painter.setPen(Qt::NoPen);
	int rectHeight = qMax(240, height() - 400);
	int rectWidth = 10;
	QLinearGradient brushGradient(width() - 20, (height() - rectHeight) / 2, width() - 20 + rectWidth, (height() + rectHeight) / 2);
	for (int i = 0; i < 256; ++i)
		brushGradient.setColorAt(1.0 - float(i) / 255.0, QColor(G_ColorTable[E_ColorTableType_DisplacementField][0][i], G_ColorTable[E_ColorTableType_DisplacementField][1][i], G_ColorTable[E_ColorTableType_DisplacementField][2][i]));
	painter.setBrush(brushGradient);
	painter.drawRect(QRect(width() - 20, (height() - rectHeight) / 2, rectWidth, rectHeight));

	for (int i = 0; i <= 5; ++i)
	{
		painter.setPen(QColor(G_ColorTable[E_ColorTableType_DisplacementField][0][i * 255 / 5], G_ColorTable[E_ColorTableType_DisplacementField][1][i * 255 / 5], G_ColorTable[E_ColorTableType_DisplacementField][2][i * 255 / 5]));
		painter.drawText(width() - 40, (height() + rectHeight) / 2 - i * rectHeight / 5, tr("%1").arg(m_MinShowedDistance + i *(m_MaxShowedDistance - m_MinShowedDistance) / 5, 0, 'f', 0));
	}

	painter.restore();
}

void Wgt2DViewer::paintInteractionEvent(QPainter& painter)
{
	painter.save();
	if (m_CurrentInteraction)
	{
		if (!isContourInteraction(m_CurrentInteraction))
		{
			m_CurrentInteraction->paint(this, painter);
		}
		else
		{
			if (g_Patient && g_Patient->getCurrentROI())
			{
				m_CurrentInteraction->paint(this, painter);
			}
		}
	}
	painter.restore();
}

void Wgt2DViewer::paintFocusLine(QPainter& painter)
{
	if (m_PrimarySeries == NULL)
		return;

	painter.save();
	painter.setRenderHint(QPainter::Antialiasing);
	QPointF focusPos;
	bool result = false;
	spacePoint2Screen(m_PrimarySeries->getFocus(), focusPos, result);
	if (result)
	{
		float focus_y = focusPos.y();
		float focus_x = focusPos.x();
		QLineF horline(QPointF(0, focus_y), QPointF(width(), focus_y));
		QLineF verline(QPointF(focus_x, 0), QPointF(focus_x, height()));
		painter.setPen(QPen(Qt::gray, 1, Qt::DashLine));
		painter.setOpacity(1);
		painter.drawLine(horline);
		painter.drawLine(verline);
	}
	painter.restore();
}

void Wgt2DViewer::paintMeasureTools(QPainter& painter)
{
	painter.save();
	for (int i = 0; i < m_MeasureAreaObjs.size(); ++i)
	{
		m_MeasureAreaObjs[i]->paint(this, painter);
	}
	for (int i = 0; i < m_MeasureLengAngleObjs.size(); ++i)
	{
		m_MeasureLengAngleObjs[i]->paint(this, painter);
	}
	for (int i = 0; i < m_MeasureCurveObjs.size(); ++i)
	{
		m_MeasureCurveObjs[i]->paint(this, painter);
	}
	painter.restore();
}

void Wgt2DViewer::paintSeriesInfo(QPainter& painter)
{
	painter.save();

	//上下左右的方向标志字符显示内容
	if (m_PrimarySeries != NULL)
	{
		painter.setPen(QPen(COLOR_TEXT, 5, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
		RVector9F orient = m_PrimarySeries->getOrientation();
		int m[6];
		switch (m_ViewDirection)
		{
		case E_ViewDirection_AXIAL:
			m[0] = floor(orient[0] + 0.5);			m[1] = floor(orient[1] + 0.5);			m[2] = floor(orient[2] + 0.5);
			m[3] = floor(orient[3] + 0.5);			m[4] = floor(orient[4] + 0.5);			m[5] = floor(orient[5] + 0.5);
			break;
		case E_ViewDirection_CORONAL:
			m[0] = floor(orient[0] + 0.5);			m[1] = floor(orient[1] + 0.5);			m[2] = floor(orient[2] + 0.5);
			m[3] = -floor(orient[6] + 0.5);			m[4] = -floor(orient[7] + 0.5);			m[5] = -floor(orient[8] + 0.5);
			break;
		case E_ViewDirection_SAGITTAL:
			m[0] = floor(orient[3] + 0.5);			m[1] = floor(orient[4] + 0.5);			m[2] = floor(orient[5] + 0.5);
			m[3] = -floor(orient[6] + 0.5);			m[4] = -floor(orient[7] + 0.5);			m[5] = -floor(orient[8] + 0.5);
			break;
		default:
			break;
		}
		E_PatientOrientation leftOrient = E_PatientOrientation(m[0] * E_PatientOrientation_R + m[1] * E_PatientOrientation_A + m[2] * E_PatientOrientation_I);
		E_PatientOrientation topOrient = E_PatientOrientation(m[3] * E_PatientOrientation_R + m[4] * E_PatientOrientation_A + m[5] * E_PatientOrientation_I);
		E_PatientOrientation rightOrient = (E_PatientOrientation)-leftOrient;
		E_PatientOrientation bottomOrient = (E_PatientOrientation)-topOrient;
		leftOrient = (leftOrient < 0) ? E_PatientOrientation(leftOrient + 7) : leftOrient;
		rightOrient = (rightOrient < 0) ? E_PatientOrientation(rightOrient + 7) : rightOrient;
		topOrient = (topOrient < 0) ? E_PatientOrientation(topOrient + 7) : topOrient;
		bottomOrient = (bottomOrient < 0) ? E_PatientOrientation(bottomOrient + 7) : bottomOrient;
		painter.drawText(0, height() * 0.5 - 10, 20, 20, Qt::AlignCenter, ES_PatientOrientation[leftOrient]);
		painter.drawText(width() - 20, height() * 0.5 - 10, 20, 20, Qt::AlignCenter, ES_PatientOrientation[rightOrient]);
		painter.drawText(width() * 0.5 - 10, 0, 20, 20, Qt::AlignCenter, ES_PatientOrientation[topOrient]);
		painter.drawText(width() * 0.5 - 10, height() - 20, 20, 20, Qt::AlignCenter, ES_PatientOrientation[bottomOrient]);
	}

	if (underMouse())
	{
		painter.setPen(QPen(COLOR_TEXT, 2, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
		painter.setBrush(Qt::NoBrush);
		painter.drawRect(rect());

		if (m_PrimarySeries != NULL && m_PrimarySeries->getSlice(m_ViewDirection) != NULL)
		{
			QString labelText;
			bool ok;
			RVector3F mousePosInSpace;
			screen2SpacePoint(m_CursorPos, mousePosInSpace, ok);
			labelText += tr("Position: %1 %2 %3 cm")
				.arg(mousePosInSpace[0] / 10, 0, 'f', 1)
				.arg(mousePosInSpace[1] / 10, 0, 'f', 1)
				.arg(mousePosInSpace[2] / 10, 0, 'f', 1);
			// 获取光标对应的三维图像坐标
			QPointF mousePosInPrimaryImage;
			screen2PrimaryIndex(m_CursorPos, mousePosInPrimaryImage, ok);
			// 光标对应图像索引坐标需要进行四舍五入
			RVector2I dataIndex;
			dataIndex[0] = floor(mousePosInPrimaryImage.x() + 0.5);
			dataIndex[1] = floor(mousePosInPrimaryImage.y() + 0.5);
			// 光标对应像素值
			float mousePosPrimaryHU = 0.0;
			RVector2I primaryImageSize = m_PrimarySeries->getSlice(m_ViewDirection)->getSize();
			if (dataIndex[0] >= 0 && dataIndex[0] < primaryImageSize[0] && dataIndex[1] >= 0 && dataIndex[1] < primaryImageSize[1])
			{
				mousePosPrimaryHU = m_PrimarySeries->getSlice(m_ViewDirection)->getBuffPointer()[dataIndex[1] * primaryImageSize[0] + dataIndex[0]];
				labelText += tr("\r\nPrimary: %1 %2").arg(mousePosPrimaryHU, 0, 'f', 0).arg(ES_PixelUnitsDisplayString[m_PrimarySeries->getUnit()]);
			}
			if (m_SecondarySeries != NULL && m_SecondarySeries->getSlice(m_ViewDirection) != NULL)
			{
				// 获取光标对应的三维图像坐标
				QPointF mousePosInSecondaryImage;
				screen2SecondaryIndex(m_CursorPos, mousePosInSecondaryImage, ok);
				// 光标对应图像索引坐标需要进行四舍五入
				RVector2I dataIndex;
				dataIndex[0] = floor(mousePosInSecondaryImage.x() + 0.5);
				dataIndex[1] = floor(mousePosInSecondaryImage.y() + 0.5);
				// 光标对应像素值
				float mousePosSecondaryHU = 0.0;
				RVector2I secondaryImageSize = m_SecondarySeries->getSlice(m_ViewDirection)->getSize();
				if (dataIndex[0] >= 0 && dataIndex[0] < secondaryImageSize[0] && dataIndex[1] >= 0 && dataIndex[1] < secondaryImageSize[1])
				{
					mousePosSecondaryHU = m_SecondarySeries->getSlice(m_ViewDirection)->getBuffPointer()[dataIndex[1] * secondaryImageSize[0] + dataIndex[0]];
					labelText += tr("\r\nSecondary: %1 %2").arg(mousePosSecondaryHU, 0, 'f', 0).arg(ES_PixelUnitsDisplayString[m_SecondarySeries->getUnit()]);
				}
			}
			if (m_Dose != NULL && m_Dose->getSlice(m_ViewDirection) != NULL)
			{
				// 获取光标对应的三维图像坐标
				QPointF mousePosInDoseImage;
				screen2DoseIndex(m_CursorPos, mousePosInDoseImage, ok);
				// 光标对应图像索引坐标需要进行四舍五入
				RVector3I dataIndex;
				dataIndex[0] = floor(mousePosInDoseImage.x() + 0.5);
				dataIndex[1] = floor(mousePosInDoseImage.y() + 0.5);
				// 光标对应像素值
				float mousePosDose = 0.0;
				RVector2I doseImageSize = m_Dose->getSlice(m_ViewDirection)->getSize();
				if (dataIndex[0] >= 0 && dataIndex[0] < doseImageSize[0] && dataIndex[1] >= 0 && dataIndex[1] < doseImageSize[1])
				{
					mousePosDose = m_Dose->getSlice(m_ViewDirection)->getBuffPointer()[dataIndex[1] * doseImageSize[0] + dataIndex[0]];
					labelText += tr("\r\nDose: %1 %2").arg(mousePosDose, 0, 'f', 0).arg(tr("cGy"));
				}
			}

			painter.setPen(QPen(COLOR_TEXT, 5, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
			painter.drawText(0 + 10, 0 + 10, 180, 200, Qt::AlignLeft | Qt::AlignTop, labelText);
		}
	}

	QString labelText_leftBottom;
	if (m_PrimarySeries != NULL && m_PrimarySeries->getSlice(m_ViewDirection) != NULL)
	{
		RVector3I primarySeriesDimension = m_PrimarySeries->getSize();
		labelText_leftBottom += tr("Slice %1/%2").arg(m_PrimarySeries->getSlice(m_ViewDirection)->getSliceIndex() + 1, 0, 'f', 1).arg(primarySeriesDimension[2 - m_ViewDirection]);
		labelText_leftBottom += tr("\r\nWW/WL %1/%2")
			.arg(int(m_PrimarySeries->getWindowWidth()))
			.arg(int(m_PrimarySeries->getWindowLevel() + 0.5));
	}
	else
		labelText_leftBottom = tr("No primary series");
	painter.setPen(QPen(COLOR_TEXT, 5, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
	painter.drawText(0 + 10, height() - 200 - 10, 180, 200, Qt::AlignLeft | Qt::AlignBottom, labelText_leftBottom);

	QString labelText_rightBottom;
	if (m_SecondarySeries != NULL)
	{
		labelText_rightBottom += tr("\r\nWW/WL %1/%2")
			.arg(int(m_SecondarySeries->getWindowWidth()))
			.arg(int(m_SecondarySeries->getWindowLevel() + 0.5));
	}
	else
		labelText_rightBottom = tr("No secondary series");
	painter.setPen(QPen(COLOR_TEXT, 5, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
	painter.drawText(width() - 180 - 10, height() - 200 - 10, 180, 200, Qt::AlignRight | Qt::AlignBottom, labelText_rightBottom);

	// 绘制比例尺
	int index = -1;
	int scaleLength = 200;
	int labelLength;
	int labelCount = 0;
	do 
	{
		index++;
		QString unitLabel = QString("%1").arg(Wgt2DViewer::UnitLength[index] / 10);
		labelCount = scaleLength / (Wgt2DViewer::UnitLength[index] * m_DrawRectScale);
		labelLength = qMax(painter.fontMetrics().width(unitLabel), 40) * labelCount;
	} while (labelLength > scaleLength && index < 15);
	painter.setPen(QPen(COLOR_TEXT, 3, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
	painter.drawLine((width() - scaleLength) * 0.5, height() - 40, (width() + scaleLength) * 0.5, height() - 40);
	painter.drawText(QRect((width() + scaleLength) * 0.5, height() - 40, 40, 20), Qt::AlignCenter, tr("cm"));
	// 绘制主刻度
	for (int i = 0; i < labelCount + 1; ++i)
	{
		int textPos = (width() - scaleLength) * 0.5 + i * Wgt2DViewer::UnitLength[index] * m_DrawRectScale;
		QString text = QString("%1").arg(Wgt2DViewer::UnitLength[index] * i / 10);
		int textWidth = painter.fontMetrics().width(text);
		painter.setPen(QPen(COLOR_TEXT, 3, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin));
		painter.drawLine(textPos, height() - 40, textPos, height() - 45);
		painter.drawText(QRect(textPos - textWidth / 2, height() - 40, textWidth, 20), Qt::AlignCenter, text);
		// 绘制子刻度
		painter.setPen(QPen(COLOR_TEXT, 2, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin));
		for (int j = 1; j < 4; ++j)
		{
			int pos = textPos + j * Wgt2DViewer::UnitLength[index] * m_DrawRectScale / 4;
			if (pos > (width() + scaleLength) * 0.5)
				break;
			painter.drawLine(pos, height() - 40, pos, height() - 43);
		}
	}

	painter.restore();
}

void Wgt2DViewer::paintROI(QPainter& painter)
{
	if (m_PrimarySeries == NULL)
		return;
	painter.save();
	painter.setRenderHint(QPainter::Antialiasing);
	QPointF topleft, endpoint;
	QPointF temp1, temp2;
	bool ok;
	spacePoint2Screen(m_PrimarySeries->getROIOrigin(), temp1, ok);
	spacePoint2Screen(m_PrimarySeries->getROIEndPoint(), temp2, ok);

	topleft.setX(qMin(temp1.x(), temp2.x()));
	topleft.setY(qMin(temp1.y(), temp2.y()));
	endpoint.setX(qMax(temp1.x(), temp2.x()));
	endpoint.setY(qMax(temp1.y(), temp2.y()));
	//QRectF roi = QRectF(QPointF(bottomleft.x(), endpoint.y()), QSize(endpoint.x() - bottomleft.x(), bottomleft.y() - endpoint.y()));
	QRectF roi = QRectF(topleft, endpoint);

	if (m_CurrentInteraction == m_ROIInteraction)
	{
		painter.setBrush(Qt::NoBrush);
		painter.setPen(QPen(Qt::yellow, 1, Qt::DashLine, Qt::SquareCap, Qt::MiterJoin));
		painter.drawRect(roi);
		//QRectF up = QRectF(m_PrimaryDrawRect.topLeft(), QPointF(m_PrimaryDrawRect.topRight().x(), roi.topRight().y()));
		//QRectF left = QRectF(up.bottomLeft(), roi.bottomLeft());
		//QRectF down = QRectF(left.bottomLeft(), QPointF(m_PrimaryDrawRect.bottomRight()));
		//QRectF right = QRectF(roi.topRight(), QPointF(m_PrimaryDrawRect.topRight().x(), roi.bottomRight().y()));

		//painter.fillRect(left, QBrush(QColor(255, 0, 0, 32)));
		//painter.fillRect(up, QBrush(QColor(255, 0, 0, 32)));
		//painter.fillRect(down, QBrush(QColor(255, 0, 0, 32)));
		//painter.fillRect(right, QBrush(QColor(255, 0, 0, 32)));
		painter.setPen(QPen(Qt::yellow, 5, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
		painter.drawLine(topleft.x(), topleft.y(), topleft.x() + qMin(20.0, roi.width()), topleft.y());
		painter.drawLine(endpoint.x(), topleft.y(), endpoint.x() - qMin(20.0, roi.width()), topleft.y());
		painter.drawLine(topleft.x(), endpoint.y(), topleft.x() + qMin(20.0, roi.width()), endpoint.y());
		painter.drawLine(endpoint.x(), endpoint.y(), endpoint.x() - qMin(20.0, roi.width()), endpoint.y());
		painter.drawLine(topleft.x(), topleft.y(), topleft.x(), topleft.y() + qMin(20.0, roi.height()));
		painter.drawLine(endpoint.x(), topleft.y(), endpoint.x(), topleft.y() + qMin(20.0, roi.height()));
		painter.drawLine(topleft.x(), endpoint.y(), topleft.x(), endpoint.y() - qMin(20.0, roi.height()));
		painter.drawLine(endpoint.x(), endpoint.y(), endpoint.x(), endpoint.y() - qMin(20.0, roi.height()));
	}
	painter.restore();
}

void Wgt2DViewer::paintMarkPoints(QPainter& painter)
{
	painter.save();
	painter.setRenderHint(QPainter::Antialiasing);
	QPen pen(Qt::red);
	pen.setWidthF(1);
	painter.setPen(pen);
	int direction = m_ViewDirection;
	RSlice *slice = m_PrimarySeries->getSlice(m_ViewDirection);
	if (!slice)
	{
		painter.restore();
		return;
	}
	RVector3F slicepos = slice->getOrigin();
	for (int i = 0; i < m_PrimarySeries->getMarkPointsCount(); ++i)
	{
		QPointF screenpos;
		QPointF topleft;
		QPointF topright;
		QPointF botleft;
		QPointF botright;
		QLineF l1;
		QLineF l2;
		QLineF l3;
		QLineF l4;
		bool ok;
		spacePoint2Screen(m_PrimarySeries->getMarkPoint(i), screenpos, ok);
		if (ok)
		{
			float shortseglength = 6;
			topleft = screenpos + QPointF(-shortseglength, -shortseglength);
			topright = screenpos + QPointF(shortseglength, -shortseglength);
			botleft = screenpos + QPointF(-shortseglength, shortseglength);
			botright = screenpos + QPointF(shortseglength, shortseglength);

			l1 = QLineF(screenpos, topleft);
			l2 = QLineF(screenpos, topright);
			l3 = QLineF(screenpos, botleft);
			l4 = QLineF(screenpos, botright);

			float marker_slicepos = m_PrimarySeries->getMarkPoint(i)[2 - direction];
			if (!FLOATTYPEEUAL(marker_slicepos, slicepos[2 - direction]))
			{
				painter.setPen(QPen(QColor(255, 128, 0, 128), 2, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
			}
			else
			{
				painter.setPen(QPen(QColor(255, 128, 0, 255), 2, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
			}
			painter.drawLine(screenpos.x() - shortseglength / 1.414, screenpos.y() - shortseglength / 1.414, screenpos.x() + shortseglength / 1.414, screenpos.y() + shortseglength / 1.414);
			painter.drawLine(screenpos.x() - shortseglength / 1.414, screenpos.y() + shortseglength / 1.414, screenpos.x() + shortseglength / 1.414, screenpos.y() - shortseglength / 1.414);
			painter.drawEllipse(screenpos, shortseglength, shortseglength);
			painter.drawText(screenpos + QPointF(shortseglength, -shortseglength), tr("%1").arg(i + 1));
		}
	}
	painter.restore();
}

void Wgt2DViewer::paintContours(QPainter& painter)
{
	if (m_PrimarySeries->getSlice(m_ViewDirection) == NULL) return;
	QVector<RROI*> rois = g_Patient->getROIList();
	int nROI = rois.size();
	if (nROI == 0) return;

	QVector< QVector< QPolygonF > > ptrPolyVec(nROI);
	QVector<QPolygonF> *polyVecList = ptrPolyVec.data();

#pragma omp parallel for
	for (int i = 0; i < nROI; ++i)
	{
		RROI *roi = rois[i];
		if (roi->getROIVisible() == false) continue;

		RLayerSet polygons = roi->getDisplayContours(m_ViewDirection, m_PrimarySeries->getSlice(m_ViewDirection)->getOrigin()[2 - m_ViewDirection], m_PrimarySeries);
		int polycnt = polygons.size();
		for (int j = 0; j < polycnt; ++j)
		{
			int pntcnt = polygons[j].size();
			QPolygonF onepolygon(pntcnt);
			bool ok;
			QPointF screen;
			for (int k = 0; k < pntcnt; ++k)
			{
				const RVector3F &space = polygons[j].at(k);
				spacePoint2Screen(space, screen, ok);
				onepolygon[k] = screen;
			}
			polyVecList[i].push_back(onepolygon);
		}
	}

	painter.save();
	painter.setRenderHint(QPainter::Antialiasing);
	for (int i = 0; i < nROI; ++i)
	{
		RROI *roi = rois[i];
		painter.setPen(QPen(roi->getROIColor(), 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

		QVector<QPolygonF> polyvec = ptrPolyVec[i];
		for (int j = 0; j < polyvec.size(); ++j)
		{
			painter.drawPolygon(polyvec[j]);
			// painter.drawText(polyvec[j].last(), QString::number(polyvec[j].size()));
// 			RVector3F cenpos = roi->getCurrentContour()->getCenterPos();
// 			QPointF center;
// 			bool ok;
// 			spacePoint2Screen(cenpos, center, ok);
// 			painter.drawEllipse(center, 5, 5);
		}
	}
	painter.restore();
}

void Wgt2DViewer::paintBeams(QPainter& painter)
{
	if (m_Plan == NULL)
		return;
	painter.save();
	painter.setRenderHint(QPainter::Antialiasing);
	painter.setPen(QPen(QColor(0, 128, 255), 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
	painter.setBrush(QColor(0, 128, 255, 32));

	for (int i = 0; i < m_Plan->getPathCount(); ++i)
	{
		RPath* path = m_Plan->getPath(i);
		for (int j = 0; j < path->getBeamCount(); ++j)
		{
			RBeam* beam = path->getBeam(j);
			if (!beam->isVisible())
				continue;
			RLayerSet polygons = beam->getBeamPolygon(m_PrimarySeries->getFocus(), m_ViewDirection);
			int polycnt = polygons.size();
			for (int j = 0; j < polycnt; ++j)
			{
				int pntcnt = polygons[j].size();
				QPolygonF onepolygon(pntcnt);
				bool ok;
				QPointF screen;
				for (int k = 0; k < pntcnt; ++k)
				{
					const RVector3F &space = polygons[j].at(k);
					spacePoint2Screen(space, screen, ok);
					onepolygon[k] = screen;
				}
				painter.setPen(QPen(QColor(0, 128, 255), 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
				painter.drawPolygon(onepolygon);
				RVector3F p1 = beam->getNode();
				RVector3F p2 = beam->getTarget() - beam->getNode() + beam->getTarget();
				QPointF scrP1, scrP2;
				spacePoint2Screen(p1, scrP1, ok);
				spacePoint2Screen(p2, scrP2, ok);
				painter.setPen(QPen(QColor(0, 128, 255), 1, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin));
				painter.drawLine(scrP1, scrP2);
			}
		}
	}
	painter.restore();
}

void Wgt2DViewer::slot_SynOpacity(int v)
{
	int opacity = m_Opacity * 100;
	if (opacity == v)
	{
		return;
	}
	slot_SetOpacity(v);
}

void Wgt2DViewer::slot_SetHidePrimarySeries(bool v)
{
	if (m_HidePrimarySeries == v)
		return;
	m_HidePrimarySeries = v;
	this->update();
}

void Wgt2DViewer::slot_SynHidePrimarySeries(bool v)
{
	emit sgnl_SynHidePrimarySeries(v);
}

float Wgt2DViewer::getPrimarySlicePos(bool& ok)
{
	if (m_PrimarySeries->getSlice(m_ViewDirection) != NULL)
	{
		ok = true;
		return m_PrimarySeries->getSlice(m_ViewDirection)->getOrigin()[2 - m_ViewDirection];
	}
	ok = false;
	return -1;
}

bool Wgt2DViewer::getHidePrimarySeriesStatus()
{
	return m_HidePrimarySeries;
}

bool Wgt2DViewer::isContourInteraction(RInteractionBase *v)
{
	if (v == m_PolygonPointInteraction)
	{
		return true;
	}
	else if (v == m_BrushInteraction)
	{
		return true;
	}
	else if (v == m_SplinePointInteraction)
	{
		return true;
	}
	else if (v == m_FreeHandPointInteraction)
	{
		return true;
	}
	else if (v == m_SmartContourPointInteraction)
	{
		return true;
	}
	else if (v == m_TranslateContour2DInteraction)
	{
		return true;
	}
	else if (v == m_RotateContour2DInteraction)
	{
		return true;
	}
	else if (v == m_ScaleContour2DInteraction)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Wgt2DViewer::slot_Button_CrossHair_toggled(bool v)
{
	slot_SynInteractionMode(E_InteractionMode_CrossHair, v);
}

void Wgt2DViewer::slot_Button_ZoomPan_toggled(bool v)
{
	slot_SynInteractionMode(E_InteractionMode_ZoomPan, v);
}

void Wgt2DViewer::slot_Button_WWWL_toggled(bool v)
{
	slot_SynInteractionMode(E_InteractionMode_WWWL, v);
}

void Wgt2DViewer::slot_Button_Measure_toggled(bool v)
{
	slot_SynInteractionMode(E_InteractionMode_MeasureLength, v);
}