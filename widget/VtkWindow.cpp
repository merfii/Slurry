/*
这个类必须是线程安全的
对QVTKWidget直接加互斥锁是行不通的 因为各种Event的处理函数(比如paintEvent mouseEvent)都会在主消息循环里访问其数据结构 理论上都要加锁
如果有遗漏就会导致莫名其妙的线程安全问题

我能想到的唯一方法是：借助signal-slot机制 将所有方法用queue里排到主消息循环里调用
注意：vtk对象一般不是线程安全的 已经添加进来就不要在非GUI线程修改

此外，VTK的智能指针也不是线程安全的，因此不能使用vtkSmartPointer<vtkActor>添加到renderer中
故VtkWindow类内部使用一个QHash<QString, vtkProp*> 管理所有的vtkActor。析构函数中遍历所有vtkActor* 调用Delete()逐个释放
*/

#include <vtkAutoInit.h>

//#pragma warning (disable : 4204)
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2)
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkInteractionStyle);
//VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);


#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCamera.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkTextProperty.h>
#include <vtkTextMapper.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLineWidget2.h>
#include <vtkLineRepresentation.h>
#include <QTimer>
#include <QKeyEvent>

#include "Slurry.h"
#include "GlobalShared.h"
#include "Logger.h"
#include "VtkWindow.h"

Q_DECLARE_METATYPE(vtkProp*);

// Line widget callback for the interaction
class vtkLineCallback : public vtkCommand
{

public:
	static vtkLineCallback *New()
	{
		return new vtkLineCallback;
	}

	virtual void Execute(vtkObject *caller, unsigned long eventId,	void *callData)
	{

		vtkLineWidget2 *lineWidget =
			reinterpret_cast<vtkLineWidget2*>(caller);

		// Get the actual box coordinates of the line
		vtkLineRepresentation *repr = dynamic_cast<vtkLineRepresentation*>(lineWidget->GetRepresentation());
		//vtkSmartPointer<vtkPolyData> polydata =
		//	vtkSmartPointer<vtkPolyData>::New();
		//dynamic_cast<vtkLineRepresentation*>(lineWidget->GetRepresentation())->GetPolyData(polydata);

		// Display one of the points, just so we know it's working
		double P1[3],P2[3];
		repr->GetPoint1WorldPosition(P1);
		repr->GetPoint2WorldPosition(P2);
		if (pWin){
			pWin->_vtkLineWidgetMoved(P1, P2);
		}
		//std::cout << "P1: " << P1[0] << " " << P1[1] << " " << P1[2] << std::endl;
		//std::cout << "P2: " << P2[0] << " " << P2[1] << " " << P2[2] << std::endl;
	}
	vtkLineCallback(){}

	VtkWindow *pWin = nullptr;
};

VtkWindow::VtkWindow(QWidget *parent)
	: QVTKWidget(parent)
{
	int metaTypeId = qRegisterMetaType<vtkProp*>();

	_removeActorsAll();
	mActorsId = 0;
	
	vtkSmartPointer<vtkTextProperty> textProperty =
		vtkSmartPointer<vtkTextProperty>::New();
	textProperty->SetFontSize(16);
	textProperty->SetBold(1);
	textProperty->SetColor(0, 0.05, 0.1);

	vtkTextMapper *textMapper = vtkTextMapper::New();
	textMapper->SetInput("VTK VIEW");
	textMapper->SetTextProperty(textProperty);
	mTextMapper = textMapper;

	vtkSmartPointer<vtkActor2D> textActor =
		vtkSmartPointer<vtkActor2D>::New();
	textActor->SetMapper(textMapper);
	textActor->SetPosition(8, 0);

	vtkLineWidget2* lineWidget = vtkLineWidget2::New();
	mLineWidget = lineWidget;

//	lineWidget->CreateDefaultRepresentation();
	double dPos1[3] = { -0.2, 0.4, 0.4 };
	double dPos2[3] = {-0.1, 0.2, 0.5 };
	vtkSmartPointer<vtkLineRepresentation> lineRepresentation = 
	vtkSmartPointer<vtkLineRepresentation>::New();
	lineRepresentation->SetPoint1WorldPosition(dPos1);
	lineRepresentation->SetPoint2WorldPosition(dPos2);
	lineRepresentation->SetLineColor(0.3, 0.1, 0.1);
	lineRepresentation->GetEndPointProperty()->SetDiffuseColor(0.8, 0.1, 0.1);
	lineWidget->SetRepresentation(lineRepresentation);

	vtkSmartPointer<vtkLineCallback> lineCallback =
		vtkSmartPointer<vtkLineCallback>::New();
	lineCallback->pWin = this;
	lineWidget->AddObserver(vtkCommand::InteractionEvent, lineCallback);

	vtkRenderer *render = vtkRenderer::New();
	render->SetBackground(0.98, 0.98, 0.96);
	render->AddActor(textActor);
	GetRenderWindow()->AddRenderer(render);
	//GetRenderWindow()->SetSize(300, 300);
	mRenderer = render;

	//render->SetActiveCamera(myCamera);
	vtkCamera *camera = render->GetActiveCamera();
	camera->SetClippingRange(0.01, 4); 
	camera->SetFocalPoint(0, 0, 0.5);
	camera->SetPosition(-2, 2, 2);
	camera->SetViewUp(0, 0, 1);
	camera->ComputeViewPlaneNormal();

	vtkSmartPointer<vtkRenderWindowInteractor> renderInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderInteractor->SetRenderWindow(GetRenderWindow());

	lineWidget->SetInteractor(renderInteractor);

	// By default the vtkRenderWindowInteractor instantiates an instance
	// of vtkInteractorStyle. vtkInteractorStyle translates a set of events
	// it observes into operations on the camera, actors, and/or properties
	// in the vtkRenderWindow associated with the vtkRenderWinodwInteractor.
	// Here we specify a particular interactor style.
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
		vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	style->SetMouseWheelMotionFactor(0.4);
	renderInteractor->SetInteractorStyle(style);

	// Start the event loop.
	renderInteractor->Initialize();
	GetRenderWindow()->Render();
//	renderInteractor->Start();	这个会卡住
//	connect(timer, SIGNAL(timeout()), this, SLOT(updateView()));
}

VtkWindow::~VtkWindow()
{
	static_cast<vtkRenderer*>(mRenderer)->Delete();
	static_cast<vtkTextMapper*>(mTextMapper)->Delete();
}

void VtkWindow::SetViewpointCT()
{
	vtkRenderer *render = (vtkRenderer *)mRenderer;
	render->GetActiveCamera()->SetFocalPoint(0, 0, 0);
	render->GetActiveCamera()->SetPosition(1.5, 1.5, 1.5);
	render->GetActiveCamera()->SetViewUp(0, 1, 0);
}

QString VtkWindow::AddActorTS(vtkProp *actor, QString label)
{
	mActorsId++;

	if (label.isEmpty())
	{
		label = QString("_Actor%1").arg(mActorsId, 3, QChar('0'));
	}

	QMetaObject::invokeMethod(this, "_addActor", Qt::QueuedConnection,
		Q_ARG(vtkProp*, actor), Q_ARG(QString, label));
	return label;
}

#include <QThread>
void VtkWindow::_addActor(vtkProp *actor, QString label)
{
	static_cast<vtkRenderer*>(mRenderer)->AddActor(actor);
	mActors.insertMulti(label, actor);
	update();
//	Logger::Debug(QString("Add "+ label));
}

QString VtkWindow::AddActor(vtkProp *actor, QString label)
{
	mActorsId++;
	if (label.isEmpty())
	{
		label = QString("_Actor%1").arg(mActorsId, 3, QChar('0'));
	}
	static_cast<vtkRenderer*>(mRenderer)->AddActor(actor);
	mActors.insertMulti(label, actor);
	update();
	return label;
}

void VtkWindow::RemoveActorTS(QString label)
{
	QMetaObject::invokeMethod(this, "_removeActor", Qt::QueuedConnection,
		Q_ARG(QString, label));
}

void VtkWindow::_removeActor(QString label)
{
	if (mActors.contains(label))
	{
		QList<vtkProp*> acts = mActors.values(label);
		vtkProp *it;
		foreach (it, acts)
		{
			static_cast<vtkRenderer*>(mRenderer)->RemoveActor(it);
		}
		mActors.remove(label);
		update();
	}
}

void VtkWindow::RemoveActorsAllTS()
{
	QMetaObject::invokeMethod(this, "_removeActorsAll", Qt::QueuedConnection);
}

void VtkWindow::_removeActorsAll()
{
	vtkProp *it;
	auto vals = mActors.values();
	foreach(it, vals)
	{
		static_cast<vtkRenderer*>(mRenderer)->RemoveActor(it);
	}
	mActors.clear();
	update();
}


void VtkWindow::SetTextTS(QString &text)
{
	QMetaObject::invokeMethod(this, "_setText", Qt::QueuedConnection,
		Q_ARG(QString, text));
}

void VtkWindow::_setText(QString text)
{
	static_cast<vtkTextMapper*>(mTextMapper)->SetInput(text.toStdString().c_str());
	update();
}

void VtkWindow::SetLineWidgetDisp(bool visible)
{
	if (visible)
	{
		static_cast<vtkLineWidget2*>(mLineWidget)->On();
	}
	else
	{
		static_cast<vtkLineWidget2*>(mLineWidget)->Off();
	}
	update();
}

void VtkWindow::SetLineWidgetPoints(double *P1, double *P2)
{
	vtkLineWidget2 *lineWidget = reinterpret_cast<vtkLineWidget2*>(mLineWidget);
	vtkLineRepresentation *repr = dynamic_cast<vtkLineRepresentation*>(lineWidget->GetRepresentation());
	repr->SetPoint1WorldPosition(P1);
	repr->SetPoint2WorldPosition(P2);
	//cout << "OP1:" << P1[0] << " " << P1[1] << " " << P1[2] << endl;
	//cout << "OP2:" << P2[0] << " " << P2[1] << " " << P2[2] << endl;
	update();

}

void VtkWindow::_vtkLineWidgetMoved(double *P1, double *P2)
{
	emit LineWidgetMoved(P1, P2);
}

void VtkWindow::_updateView()
{
	if (isVisible())
	{
		//actor1->SetOrientation(0, 0, m_angle);
		update();
	}
	//iren->GetActiveCamera()->Azimuth(1);
}

void VtkWindow::keyPressEvent(QKeyEvent *event)
{
	switch (event->key())
	{
	case Qt::Key::Key_L:

		break;

	default:
		QVTKWidget::keyPressEvent(event);
		update();
		break;
	}
}

void VtkWindow::debugAddCone()
{
	vtkSmartPointer<vtkConeSource> coneSource = vtkSmartPointer<vtkConeSource>::New();
	coneSource->SetHeight(3.0);
	coneSource->SetRadius(1.0);
	coneSource->SetResolution(10);
	coneSource->SetCenter(5, 0, 0);

	vtkSmartPointer<vtkPolyDataMapper> coneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	coneMapper->SetInputConnection(coneSource->GetOutputPort());

	vtkSmartPointer<vtkActor> coneActor = vtkSmartPointer<vtkActor>::New();
	coneActor->SetMapper(coneMapper);
	//	coneActor->SetVisibility(false);

	static_cast<vtkRenderer*>(mRenderer)->AddActor(coneActor);
}

#include <vtkLineWidget.h>
static void debugAddBoxWidget()
{
	// The SetInteractor method is how 3D widgets are associated with the render
	// window interactor. Internally, SetInteractor sets up a bunch of callbacks
	// using the Command/Observer mechanism (AddObserver()). The place factor
	// controls the initial size of the widget with respect to the bounding box
	// of the input to the widget.

	vtkSmartPointer<vtkLineWidget> boxWidget = vtkSmartPointer<vtkLineWidget>::New();
	// Place the interactor initially. The input to a 3D widget is used to
	// initially position and scale the widget. The EndInteractionEvent is
	// observed which invokes the SelectPolygons callback.
	boxWidget->SetAlignToNone();
	boxWidget->SetPoint1(2, 2, 2);
	boxWidget->SetPoint2(20, -1, 5);
	// boxWidget->SetInteractor(renderInteractor);
	//	vtkMyCallback *callback = vtkMyCallback::New();
	//	boxWidget->AddObserver(vtkCommand::InteractionEvent, callback);
	boxWidget->On();

}
