#ifndef WGT2DVIEWER_H
#define WGT2DVIEWER_H

#include <QtWidgets>
#include "RWidget.h"
#include <QMap>
#include <QToolTip>
#include <QString>
#include <QToolButton>
#include <QRectF>
#include <ipccommon>
#include "RColorTable.h"
#include <itkImage.h>
#include "TTransformer.h"

class RSeries;
class RSlice;
class RROI;
class RPlan;
class RDose;

class RInteractionBase;

class RMeasureObjBase;
class RMeasureAreaObj;
class RMeasureLengthAngleObj;
//////////////////////////////////////////////////////////////////////////
// common-interaction-related declearations
class RCrossHairInteraction;
class RMeasureAreaInteraction;
class RMeasureLengthAngleInteraction;
class RZoomPanInteraction;
class RWWWLInteraction;
//////////////////////////////////////////////////////////////////////////
// register-related declearations
class RROIInteraction;
class RMarkerInteraction;
class RRegTranslateInteraction;
class RRegRotationInteraction;
//////////////////////////////////////////////////////////////////////////
// contour-related declearations
class RPolygonPointContourInteraction;
class RSplinePointContourInteraction;
class RFreeHandPointContourInteraction;
class RSmartContourPointInteraction;
class RBrushContourInteraction;
class RTranslateContour2DInteraction;
class RRotateContour2DInteraction;
class RScaleContour2DInteraction;
class RRegionGrowing2DInteraction;
class RRegionGrowing3DInteraction;
class RDeformContourInteraction;
class RDeformContour3DInteraction;
//////////////////////////////////////////////////////////////////////////

typedef itk::Vector< float, 3 > VectorPixelType;
typedef itk::Image< VectorPixelType, 3 > Displacement3DFieldType;
typedef Displacement3DFieldType::Pointer Displacement3DFieldPointer;
typedef itk::Image< VectorPixelType, 2 > Displacement2DFieldType;
typedef Displacement2DFieldType::Pointer Displacement2DFieldPointer;

class Wgt2DViewer : public RWidget
{
	Q_OBJECT

public:
	Wgt2DViewer(QWidget *parent = 0, bool standalone = true);
	~Wgt2DViewer();

	// 设置/获取当前窗口的视图方向
	SETANDGETMACRO(ViewDirection, E_ViewDirection);

	// 获取前景图像的指针
	RSlice *getPrimaryRawSlice();

	// 面积测量工具相关的函数
	void addMeasureAreaObj(RMeasureAreaObj* &obj);
	void removeMeasureAreaObj(int index);
	void clearMeasureAreaObjs();
	QVector<RMeasureAreaObj*> getMeasureAreaObjs();

	// 长度/角度测量工具相关的函数
	void addMeasureLengAngleObj(RMeasureLengthAngleObj*obj);
	void removeMeasureLengAngleObj(int index);
	void clearMeasureLengAngleObjs();
	QVector<RMeasureLengthAngleObj*> getMeasureLengAngleObjs();

	// 曲线测量工具相关的函数
	void addMeasureCurveObj(RMeasureLengthAngleObj*obj);
	void removeMeasureCurveObj(int index);
	void clearMeasureCurveObjs();
	QVector<RMeasureLengthAngleObj*> getMeasureCurveObjs();

	// 获取隐藏主序列标志位
	bool getHidePrimarySeriesStatus();
protected:
	//////////////////////////////////////////////////////////////////////////
	// measure obj-related declearations
	friend class RMeasureObjBase;
	friend class RMeasureAreaObj;
	friend class RMeasureLengthAngleObj;
	//////////////////////////////////////////////////////////////////////////
	// common-interaction-related declearations
	friend class RCrossHairInteraction;
	friend class RMeasureAreaInteraction;
	friend class RMeasureLengthAngleInteraction;
	friend class RZoomPanInteraction;
	friend class RWWWLInteraction;
	//////////////////////////////////////////////////////////////////////////
	// register-related declearations
	friend class RROIInteraction;
	friend class RMarkerInteraction;
	friend class RRegTranslateInteraction;
	friend class RRegRotationInteraction;
	//////////////////////////////////////////////////////////////////////////
	// contour-related declearations
	friend class RSplinePointContourInteraction;
	friend class RFreeHandPointContourInteraction;
	friend class RSmartContourPointInteraction;
	friend class RPolygonPointContourInteraction;
	friend class RBrushContourInteraction;
	friend class RTranslateContour2DInteraction;
	friend class RRotateContour2DInteraction;
	friend class RScaleContour2DInteraction;
	friend class RRegionGrowing2DInteraction;
	friend class RRegionGrowing3DInteraction;
	friend class RDeformContourInteraction;
	friend class RDeformContour3DInteraction;

	void screen2PrimaryIndex(const QPointF &screen, QPointF &image, bool &resultvalid);
	void screen2SecondaryIndex(const QPointF &screen, QPointF &image, bool &resultvalid);
	void screen2SecondaryPoint(const QPointF &screen, QPointF &image, bool &resultvalid);
	void screen2DoseIndex(const QPointF &screen, QPointF &image, bool &resultvalid);
	void screen2DosePoint(const QPointF &screen, QPointF &image, bool &resultvalid);
	void primaryIndex2Screen(const QPointF &image, QPointF &screen, bool &resultvalid);
	void secondaryIndex2Screen(const QPointF &image, QPointF &screen, bool &resultvalid);
	void doseIndex2Screen(const QPointF &image, QPointF &screen, bool &resultvalid);
	void screen2SpacePoint(const QPointF &screen, RVector3F &space, bool &resultvalid);
	void spacePoint2Screen(const RVector3F &space, QPointF &image, bool &resultvalid);

public slots:
	void slot_SetSeriesFocus(RVector3F focus);

	void slot_SetSeries(RSeries* primary, RSeries* secondary = NULL);
	void slot_SynSeries(RSeries* primary, RSeries* secondary = NULL);

	void slot_SetDose(RDose* s);
	void slot_SynDose(RDose* s);

	// 设置融合显示模式
	void slot_SetFusionViewMode(E_FusionMode v);
	void slot_SynFusionViewMode(E_FusionMode v);

	// 设置融合显示模式网格尺寸
	void slot_SetFusionViewGridSize(int v);
	void slot_SynFusionViewGridSize(int v);

	// 设置融合显示模式网格奇偶翻转标志位
	void slot_SetFusionViewPatternFliped(bool v);
	void slot_SynFusionViewPatternFliped(bool v);

	// 用于改变显示图像的缩放比例
	void slot_SetDrawRectScale(float scale);
	void slot_SynDrawRectScale(float scale);

	// 平移绘制矩形
	void slot_SetDrawRectOffset(QPointF offset);

	// 交互模式
	void slot_SetInteractionMode(E_InteractionMode v);
	void slot_SynInteractionMode(E_InteractionMode mode, bool v);

	// 走 Index 抽层
	void slot_ChangeToNearestIndexSlice_Previous();
	void slot_ChangeToNearestIndexSlice_Next();

	// 透明度的同步
	void slot_SetOpacity(int percentage);
	void slot_SynOpacity(int percentage);

	// 设置隐藏主序列标志位
	void slot_SetHidePrimarySeries(bool v);
	void slot_SynHidePrimarySeries(bool v);

signals:
	void sgnl_SynHidePrimarySeries(bool v);
	void sgnl_SynSeries(RSeries* priamry, RSeries* secondary);
	void sgnl_SynDose(RDose *s);
	void sgnl_SynFusionViewMode(E_FusionMode v);
	void sgnl_SynFusionViewGridSize(int v);
	void sgnl_SynFusionViewPatternFliped(bool v);
	void sgnl_SynDrawRectScale(float scale);
	void sgnl_SynInteractionMode(E_InteractionMode mode, bool v);
	void sgnl_SynMarkers(RLayer &v);
	void sgnl_SynOpacity(int percentage);

	void sgnl_TranslateTransform(TTransformer::ObjectPointer);
	void sgnl_RotateTransform(TTransformer::ObjectPointer);

protected:
	void paintEvent(QPaintEvent * event);
	void resizeEvent(QResizeEvent * event);
	void mouseMoveEvent(QMouseEvent * event);
	void mousePressEvent(QMouseEvent * event);
	void mouseDoubleClickEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
	void keyPressEvent(QKeyEvent *event);
	void enterEvent(QEvent *in);
	void leaveEvent(QEvent *out);
	void contextMenuEvent(QContextMenuEvent *event);

	void initialize(bool standalone);
	bool isContourInteraction(RInteractionBase *v);
	// 计算适合窗口大小的显示比例
	float getFitToWindowScale();
	float getFitToWindowScale(QSize windowSize);
	// 重置显示比例与偏移量
	void resetScaleAndOffset();
	// 生成绘制矩形
	void generatePrimaryDrawRect();
	void generateSecondaryDrawRect();
	void generateDoseDrawRect();

	// 绘制参考图像
	void paintPrimaryImage(QPainter& painter);
	// 绘制浮动图像
	void paintSecondaryImage(QPainter& painter);
	// 绘制剂量图像
	void paintDoseImage(QPainter& painter);
	// 绘制配准结果位移向量场
	void paintDisplacementField(QPainter& painter);
	// 绘制交互模式下的绘制事件
	void paintInteractionEvent(QPainter& painter);
	// 绘制焦点十字线
	void paintFocusLine(QPainter& painter);
	// 绘制测量工具
	void paintMeasureTools(QPainter& painter);
	// 绘制文本信息
	void paintSeriesInfo(QPainter& painter);
	// 绘制ROI
	void paintROI(QPainter& painter);
	// 绘制标记点
	void paintMarkPoints(QPainter& painter);
	// 绘制轮廓项
	void paintContours(QPainter& painter);
	// 绘制射束剖面
	void paintBeams(QPainter& painter);

	// 获取当前层的位置
	float getPrimarySlicePos(bool& ok);

protected slots:
	// 快捷按钮响应槽
	void slot_Button_CrossHair_toggled(bool v);
	void slot_Button_ZoomPan_toggled(bool v);
	void slot_Button_WWWL_toggled(bool v);
	void slot_Button_Measure_toggled(bool v);

protected:
	static float m_Max_Scale; // 最大显示比例
	static float m_Min_Scale; // 最小显示比例
	static float UnitLength[15];

	// 所有交互模式的指针
	E_InteractionMode					m_CurrentInteractionMode;
	RInteractionBase*					m_CurrentInteraction;

	RCrossHairInteraction*				m_CrossHairInteraction;
	RMeasureAreaInteraction*			m_MeasureAreaInteraction;
	RMeasureLengthAngleInteraction*		m_MeasureLengthAngleInteraction;
	RZoomPanInteraction*				m_ZoomPanInteraction;
	RWWWLInteraction*					m_WWWLInteraction;

	RROIInteraction*					m_ROIInteraction;
	RMarkerInteraction*					m_MarkerInteraction;
	RRegTranslateInteraction*			m_RegTranslateInteraction;
	RRegRotationInteraction*			m_RegRotationInteraction;

	RPolygonPointContourInteraction*	m_PolygonPointInteraction;
	RFreeHandPointContourInteraction*	m_FreeHandPointInteraction;
	RSmartContourPointInteraction*		m_SmartContourPointInteraction;
	RSplinePointContourInteraction*		m_SplinePointInteraction;
	RBrushContourInteraction*			m_BrushInteraction;
	RTranslateContour2DInteraction*		m_TranslateContour2DInteraction;
	RRotateContour2DInteraction*		m_RotateContour2DInteraction;
	RScaleContour2DInteraction*			m_ScaleContour2DInteraction;
	RRegionGrowing2DInteraction*		m_RegionGrowing2DInteraction;
	RRegionGrowing3DInteraction*		m_RegionGrowing3DInteraction;
	RDeformContourInteraction*			m_DeformContourInteraction;
	RDeformContour3DInteraction*		m_DeformContour3DInteraction;

	// 快捷按钮
	QToolButton*						m_Button_CrossHair;
	QToolButton*						m_Button_ZoomPan;
	QToolButton*						m_Button_WWWL;
	QToolButton*						m_Button_Measure;
	QAction*							m_ActCrossHair;
	QAction*							m_ActZoomPan;
	QAction*							m_ActWWWL;
	QAction*							m_ActMeasure;

	// 右键菜单
	QMenu*								m_PopMenu;
	QAction*							m_ActReset;
	QAction*							m_ActRegistTranslate;
	QAction*							m_ActRegistRotate;
	QAction*							m_ActRegistCenterAlign;

	// 窗口属性(不随数据改变的属性)
	E_ViewDirection						m_ViewDirection;						// 当前窗口所显示的视图方向
	QPointF								m_CursorPos;							// 鼠标屏幕位置
	float								m_Opacity;								// 融合显示主序列透明度(1->0:主序列->副序列)
	E_FusionMode						m_FusionViewMode;						// 融合显示模式
	int									m_FusionView_GridSize;					// 融合显示网格尺寸(屏幕像素)
	bool								m_FusionView_PatternFliped;				// 融合显示网格奇偶翻转标志位
	bool								m_HidePrimarySeries;					// 对比视图下进行轮廓勾画时，置该标志位以隐藏主序列；

	// 数据属性(随数据改变的属性)
	RSeries*							m_PrimarySeries;						// 参考序列
	RSeries*							m_SecondarySeries;						// 浮动序列
	RPlan*								m_Plan;									// 治疗计划
	RDose*								m_Dose;									// 剂量矩阵
	QRectF								m_PrimaryDrawRect;						// 参考图像在窗口中的绘制矩形
	QRectF								m_SecondaryDrawRect;					// 浮动图像在窗口中的绘制矩形
	QRectF								m_DoseDrawRect;							// 剂量图像在窗口中的绘制矩形
	float								m_DrawRectScale;						// 绘制矩形的缩放比例
	QPointF								m_DrawRectOffset;						// 绘制矩形的位移，以 mm 为单位

	// 特殊属性(仅在某些交互模式下有效的属性)
	QVector<RMeasureAreaObj*>			m_MeasureAreaObjs;						// 面积测量工具
	QVector<RMeasureLengthAngleObj*>	m_MeasureLengAngleObjs;					// 长度/角度测量工具
	QVector<RMeasureLengthAngleObj*>	m_MeasureCurveObjs;						// 长度/角度测量工具
	float								m_MaxShowedDistance;					// 最大显示位移
	float								m_MinShowedDistance;					// 最小显示位移
};

#endif // WGT2DVIEWER_H
