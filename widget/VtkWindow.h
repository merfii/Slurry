#ifndef VTKWINDOW_H
#define VTKWINDOW_H

/*
这个类必须是线程安全的
对QVTKWidget直接加互斥锁是行不通的 因为各种Event的处理函数(比如paintEvent mouseEvent)都会在主消息循环里访问其数据结构 理论上都要加锁
如果有遗漏就会导致莫名其妙的线程安全问题

我能想到的唯一方法是：借助signal-slot机制 将所有方法用queue里排到主消息循环里调用
注意：vtk对象一般不是线程安全的 已经添加进来就不要在非GUI线程修改
*/

#include <QVTKWidget.h>
#include <vtkProp.h>

typedef void* PDATA_M;
class VtkWindow : public QVTKWidget
{
	friend class vtkLineCallback;
	Q_OBJECT
public:
	VtkWindow(QWidget *parent);
	~VtkWindow();

public:
	void SetViewpointCT();
	//TS for ThreadSafe
	QString AddActorTS(vtkProp *actor, QString label = QString());
	QString AddActor(vtkProp *actor, QString label = QString());
	void RemoveActorTS(QString label);
	void RemoveActorsAllTS();
	void RemovePreviewAllTS();
	void SetTextTS(QString &text);
	void SetLineWidgetDisp(bool visible);

public slots:
	void SetLineWidgetPoints(double *P1, double *P2);

signals:
	void LineWidgetMoved(double *P1, double *P2);

protected:
	void keyPressEvent(QKeyEvent * event) override;

private slots:
	void _addActor(vtkProp *actor, QString label);
	void _removeActor(QString label);
	void _removeActorsAll();
	void _updateView();
	void _setText(QString text);

private:
	void _vtkLineWidgetMoved(double P1[3], double P2[3]);
	
	QHash<QString, vtkProp*> mActors;
	PDATA_M mRenderer, mTextMapper;
	PDATA_M mLineWidget;
	int mActorsId;

	void debugAddCone();
};

#endif // VTKWINDOW_H
