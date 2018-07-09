#include "vtkSmartPointer.h"
#include "vtkLineWidget.h"

#include "vtkGuiderWidget.h"


vtkGuiderWidget::vtkGuiderWidget()
{
	vtkSmartPointer<vtkLineWidget> lineWidget = vtkSmartPointer<vtkLineWidget>(vtkLineWidget::New());
	//放两个小球

}


vtkGuiderWidget::~vtkGuiderWidget()
{


}
