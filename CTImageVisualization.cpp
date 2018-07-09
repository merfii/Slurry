#pragma execution_character_set("utf-8")
#include <QFileDialog>
#include <QMessageBox>
#include <QProgressDialog>
#include <QTest>

#include <vtkSmartPointer.h>
#include <vtkDICOMImageReader.h>
#include <vtkAlgorithmOutput.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkActor.h>
#include <vtkImageActor.h>
#include <vtkImageMapper3D.h>
#include <vtkCamera.h>
#include <vtkMarchingCubes.h>
#include <vtkNamedColors.h>
#include <vtkOutlineFilter.h>
#include <vtkImageData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkAxes.h>
#include <vtkSphereSource.h>
#include <vtkTubeFilter.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkStripper.h>
#include <vtkLookupTable.h>
#include <vtkImageMapToColors.h>
#include <vtkSmartVolumeMapper.h>
#include <vtkColorTransferFunction.h>
#include <vtkPiecewiseFunction.h>
#include <vtkVolumeProperty.h>
#include <vtkTransform.h>

#include <algorithm>
#include <array>
#include "Slurry.h"
#include "GlobalShared.h"

#define TOCTDATA(pvoid) (static_cast<CT_DATA*>(pvoid))
struct CT_DATA
{
	vtkImageData *imageData = nullptr;
	vtkAlgorithmOutput *imagePort = nullptr;
	vtkActor* outline = nullptr;
	vtkImageActor* axial = nullptr;
	vtkImageActor* sagittal = nullptr;
	vtkImageActor* coronal = nullptr;
	vtkActor* bone = nullptr;
	vtkActor* skin = nullptr;
};

static vtkActor* addCoordinate();
static vtkActor* addSphere();

void Slurry::ctInit()
{
	CT_DATA *m = new CT_DATA;
	delete TOCTDATA(m_ctData);
	m_ctData = m;

	ui.ctWindow->SetViewpointCT();
	ui.ctWindow->AddActor(addCoordinate(), "coordinator");
}

static vtkActor* addCoordinate()
{
	vtkSmartPointer<vtkAxes> axes = vtkSmartPointer<vtkAxes>::New();
	axes->SetOrigin(0, 0, 0);	//设置坐标原点
	axes->SetScaleFactor(1.0);
	axes->Update();

	vtkSmartPointer<vtkFloatArray> axes_colors = vtkSmartPointer<vtkFloatArray>::New();
	axes_colors->Allocate(6);
	axes_colors->InsertNextValue(0.0);
	axes_colors->InsertNextValue(0.0);
	axes_colors->InsertNextValue(0.5);
	axes_colors->InsertNextValue(0.5);
	axes_colors->InsertNextValue(1.0);
	axes_colors->InsertNextValue(1.0);

	vtkSmartPointer<vtkPolyData> axes_data = axes->GetOutput();
	axes_data->GetPointData()->SetScalars(axes_colors);

	vtkSmartPointer<vtkTubeFilter> axes_tubes = vtkSmartPointer<vtkTubeFilter>::New();
	axes_tubes->SetInputData(axes_data);
	axes_tubes->SetRadius(0.01);
	axes_tubes->SetNumberOfSides(6);

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetScalarModeToUsePointData();
	mapper->SetInputConnection(axes_tubes->GetOutputPort());

	vtkActor *actor = vtkActor::New();
	actor->SetMapper(mapper);
	//actor->SetOrigin(0, 0, 0);		//设置在世界坐标系中的位置
	
	return actor;
}

void Slurry::ctDisplay()
{
	CT_DATA *m = TOCTDATA(m_ctData);
	vtkSmartPointer<vtkNamedColors> colors =
		vtkSmartPointer<vtkNamedColors>::New();

	// Set the colors.
	auto SetColor = [&colors](std::array<double, 3>& v,
		std::string const& colorName) {
		auto const scaleFactor = 255.0;
		std::transform(std::begin(v), std::end(v), std::begin(v),
			[=](double const& n) { return n / scaleFactor; });
		colors->SetColor(colorName, v.data());
		return;
	};
	std::array<double, 3> skinColor{ { 246,178, 107 } };
	SetColor(skinColor, "SkinColor");
	std::array<double, 3> boneColor{ {200, 150, 30 } };
	SetColor(boneColor, "BoneColor");
	std::array<double, 3> bkgColor{ { 51, 77, 102 } };
	SetColor(bkgColor, "BkgColor");

	// An isosurface, or contour value of 500 is known to correspond to the
	// skin of the patient.
	// The triangle stripper is used to create triangle strips from the
	// isosurface; these render much faster on many systems.
	vtkSmartPointer<vtkMarchingCubes> skinExtractor =
		vtkSmartPointer<vtkMarchingCubes>::New();
	skinExtractor->SetInputConnection(m->imagePort);
	skinExtractor->SetValue(0, -500);
	skinExtractor->SetComputeNormals(1);

	vtkSmartPointer<vtkStripper> skinStripper =
		vtkSmartPointer<vtkStripper>::New();
	skinStripper->SetInputConnection(skinExtractor->GetOutputPort());

	vtkSmartPointer<vtkPolyDataMapper> skinMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	skinMapper->SetInputConnection(skinStripper->GetOutputPort());
	skinMapper->ScalarVisibilityOff();

	vtkActor* skin = vtkActor::New();
	skin->SetMapper(skinMapper);
	skin->GetProperty()->SetDiffuseColor(colors->GetColor3d("SkinColor").GetData());
	skin->GetProperty()->SetSpecular(.3);
	skin->GetProperty()->SetSpecularPower(20);
	skin->GetProperty()->SetOpacity(.5);

	vtkSmartPointer<vtkMarchingCubes> boneExtractor =
		vtkSmartPointer<vtkMarchingCubes>::New();
	boneExtractor->SetInputConnection(m->imagePort);
	boneExtractor->SetValue(0, 100);
	boneExtractor->SetComputeNormals(1);
	
	vtkSmartPointer<vtkStripper> boneStripper =
		vtkSmartPointer<vtkStripper>::New();
	boneStripper->SetInputConnection(boneExtractor->GetOutputPort());

	vtkSmartPointer<vtkPolyDataMapper> boneMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	boneMapper->SetInputConnection(boneStripper->GetOutputPort());
	boneMapper->ScalarVisibilityOff();
	
	vtkActor* bone = vtkActor::New();
	bone->SetMapper(boneMapper);
	bone->GetProperty()->SetDiffuseColor(colors->GetColor3d("BoneColor").GetData());

	// An outline provides context around the data.
	//
	vtkSmartPointer<vtkOutlineFilter> outlineData =
		vtkSmartPointer<vtkOutlineFilter>::New();
	outlineData->SetInputConnection(m->imagePort);

	vtkSmartPointer<vtkPolyDataMapper> mapOutline =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapOutline->SetInputConnection(outlineData->GetOutputPort());

	vtkActor* outline = vtkActor::New();
	outline->SetMapper(mapOutline);
	outline->GetProperty()->SetColor(colors->GetColor3d("Black").GetData());

	// Create three orthogonal planes passing through the volume.
	// creating a black/white lookup table.
	vtkSmartPointer<vtkLookupTable> bwLut =
		vtkSmartPointer<vtkLookupTable>::New();
	bwLut->SetTableRange(-500, 500);
	bwLut->SetSaturationRange(0,0);
	bwLut->SetHueRange(0, 0);
	bwLut->SetValueRange(0, 1);
	bwLut->Build(); //effective built

	// Create the first of the three planes. The filter vtkImageMapToColors
	// maps the data through the corresponding lookup table created above.  The
	// vtkImageActor is a type of vtkProp and conveniently displays an image on
	// a single quadrilateral plane. It does this using texture mapping and as
	// a result is quite fast. (Note: the input image has to be unsigned char
	// values, which the vtkImageMapToColors produces.) Note also that by
	// specifying the DisplayExtent, the pipeline requests data of this extent
	// and the vtkImageMapToColors only processes a slice of data.
	vtkSmartPointer<vtkImageMapToColors> sagittalColors =
		vtkSmartPointer<vtkImageMapToColors>::New();
	sagittalColors->SetInputConnection(m->imagePort);
	sagittalColors->SetLookupTable(bwLut);
	sagittalColors->Update();

	vtkImageActor* sagittal = vtkImageActor::New();
	sagittal->GetMapper()->SetInputConnection(sagittalColors->GetOutputPort());
	sagittal->SetDisplayExtent(250, 250, 0, 510, 0, 250);

	// Create the second (axial) plane of the three planes. We use the
	// same approach as before except that the extent differs.
	vtkSmartPointer<vtkImageMapToColors> axialColors =
		vtkSmartPointer<vtkImageMapToColors>::New();
	axialColors->SetInputConnection(m->imagePort);
	axialColors->SetLookupTable(bwLut);
	axialColors->Update();

	vtkImageActor* axial = vtkImageActor::New();
	axial->GetMapper()->SetInputConnection(axialColors->GetOutputPort());
	axial->SetDisplayExtent(0, 510, 0, 510, 120, 120);

	// Create the third (coronal) plane of the three planes. We use
	// the same approach as before except that the extent differs.
	vtkSmartPointer<vtkImageMapToColors> coronalColors =
		vtkSmartPointer<vtkImageMapToColors>::New();
	coronalColors->SetInputConnection(m->imagePort);
	coronalColors->SetLookupTable(bwLut);
	coronalColors->Update();

	vtkImageActor* coronal = vtkImageActor::New();
	coronal->GetMapper()->SetInputConnection(coronalColors->GetOutputPort());
	coronal->SetDisplayExtent(0, 510, 250, 250, 0, 250);

	// Actors are added to the renderer.
//	ui.ctWindow->AddActor(outline, "outline");
	ui.ctWindow->AddActor(sagittal, "sagittal");
	ui.ctWindow->AddActor(axial, "axial");
	ui.ctWindow->AddActor(coronal, "coronal");
	ui.ctWindow->AddActor(skin, "skin");
	ui.ctWindow->AddActor(bone, "bone");

	m->outline = outline;
	m->sagittal = sagittal;
	m->axial = axial;
	m->coronal = coronal;
	m->skin = skin;
	m->bone = bone;

	//bone->VisibilityOff();
	
	//vtkSmartPointer<vtkOBJExporter> porter = vtkSmartPointer<vtkOBJExporter>::New();
	//porter->SetFilePrefix("E:\\PolyDataWriter.obj");
	//porter->SetInput(renWin);
	//porter->Write();
//	ui.ctWindow->AddActor(addSphere(), "sphere");
}

static vtkActor* addSphere()
{
	vtkSmartPointer<vtkSphereSource> s_sphere = vtkSmartPointer<vtkSphereSource>::New();
	s_sphere->SetRadius(0.01);
	s_sphere->SetPhiResolution(24);
	s_sphere->SetThetaResolution(24);
	s_sphere->LatLongTessellationOff();
	s_sphere->Update();

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(s_sphere->GetOutputPort());

	vtkActor *actor = vtkActor::New();
	actor->SetPosition(1, 0, 0);
	actor->SetMapper(mapper);
	actor->GetProperty()->SetDiffuseColor(0.9, 0.3, 0.3);
	return actor;
}

void Slurry::ctLoadDICOM()
{
	CT_DATA *m = TOCTDATA(m_ctData);
	vtkSmartPointer<vtkDICOMImageReader>Reader = vtkSmartPointer<vtkDICOMImageReader>::New(); //创建读取dicom图片指针对象  
	Reader->SetDirectoryName("CT"); //设置医学图像文件夹路径  
	Reader->SetDataByteOrderToLittleEndian();
	Reader->Update();
	//	cout << "读取数据完成" << endl;
	m->imageData = Reader->GetOutput();
	m->imageData->Register(m->imageData);
	m->imageData->SetOrigin(0, 0, 0);
	double spacing[3];
	m->imageData->GetSpacing(spacing);
	spacing[0] /= 1000;
	spacing[1] /= 1000;
	spacing[2] /= 1000;
	m->imageData->SetSpacing(spacing);
}

#include <vtkXMLImageDataReader.h>
#include <vtkXMLImageDataWriter.h>
void Slurry::ctLoadImage3D()
{
	CT_DATA *m = TOCTDATA(m_ctData);

	//对话框
	QString path = QFileDialog::getOpenFileName(this, "打开文件", QString(), "*.vti");
	if (path.isEmpty())
		return;

	// Read the file (to test that it was written correctly)
	vtkSmartPointer<vtkXMLImageDataReader> reader =
		vtkSmartPointer<vtkXMLImageDataReader>::New();
	reader->SetFileName(path.toStdString().c_str());
	reader->Update();

	m->imagePort = reader->GetOutputPort();
	//m->imagePort->Register(m->imagePort);
	
	ctDisplay();
}


void Slurry::ctSaveImage3D()
{
	CT_DATA *m = TOCTDATA(m_ctData);
	QString path = QFileDialog::getSaveFileName(this, "保存文件", QString(), "*.vti");

	vtkSmartPointer<vtkXMLImageDataWriter> writer =
	vtkSmartPointer<vtkXMLImageDataWriter>::New();
	writer->SetFileName(path.toStdString().c_str());
	writer->SetInputData(m->imageData);
	writer->Write();
}

void Slurry::ctReconstruct()
{

}

void Slurry::ctRegistration()
{
	CT_DATA *m = TOCTDATA(m_ctData);
	if (m && m->skin){
		//显示进度条
		QProgressDialog *progressDlg = new QProgressDialog(this);
		progressDlg->setWindowModality(Qt::WindowModal);
		progressDlg->setMinimumWidth(400);
		progressDlg->setMinimumDuration(0);
		progressDlg->setAttribute(Qt::WA_DeleteOnClose, true);
		progressDlg->setWindowTitle("坐标系配准");
		progressDlg->setLabelText("正在配准......");
		progressDlg->setCancelButtonText("取消");
		progressDlg->setRange(0, 300);
		for (int i = 1; i<300; i++)//注意这里是从1开始的
		{
			//QCoreApplication::processEvents(QEventLoop::WaitForMoreEvents, 200);	//假装在进行计算
			QTest::qWait(20);	//假装在进行计算
			progressDlg->setValue(i);
			if (progressDlg->wasCanceled())
			{
				m_ctMatched = false;
				break;
			}
			if (i == 290){	
				m_ctMatched = true;
			}
		}
		progressDlg->close();
		if (m_ctMatched){
			QMessageBox::information(this, "成功", "配准成功！请切换选项卡查看结果");
		}
		else
		{
			return;
		}

		vtkSmartPointer<vtkSmartVolumeMapper> volumeMapper =
			vtkSmartPointer<vtkSmartVolumeMapper>::New();
		volumeMapper->SetBlendModeToComposite(); // composite first
		volumeMapper->SetInputConnection(m->imagePort);

		vtkSmartPointer<vtkColorTransferFunction>volumeColor =
			vtkSmartPointer<vtkColorTransferFunction>::New();
		volumeColor->AddRGBPoint(-600, 0.0, 0.0, 0.0);
		volumeColor->AddRGBPoint(-800, 1.0, 0.5, 0.3);
		volumeColor->AddRGBPoint(0, 1.0, 0.5, 0.3);
		volumeColor->AddRGBPoint(200, 1.0, 1.0, 0.9);

		// The opacity transfer function is used to control the opacity
		// of different tissue types.
		vtkSmartPointer<vtkPiecewiseFunction> volumeScalarOpacity =
			vtkSmartPointer<vtkPiecewiseFunction>::New();
		volumeScalarOpacity->AddPoint(-1000, 0);
		volumeScalarOpacity->AddPoint(-900, 0.2);
		volumeScalarOpacity->AddPoint(-800, 0.98);
		volumeScalarOpacity->AddPoint(-100, 0.99);
		volumeScalarOpacity->AddPoint(1000, 1);

		vtkSmartPointer<vtkVolumeProperty> volumeProperty =
			vtkSmartPointer<vtkVolumeProperty>::New();
		volumeProperty->SetColor(volumeColor);
		volumeProperty->SetScalarOpacity(volumeScalarOpacity);
		volumeProperty->SetInterpolationTypeToLinear();
		volumeProperty->ShadeOn();
		volumeProperty->SetAmbient(0.4);
		volumeProperty->SetDiffuse(0.6);
		volumeProperty->SetSpecular(0.2);

		// The vtkVolume is a vtkProp3D (like a vtkActor) and controls the position
		// and orientation of the volume in world coordinates.
		vtkVolume *volume = vtkVolume::New();
		volume->SetMapper(volumeMapper);
		volume->SetProperty(volumeProperty);

		vtkSmartPointer<vtkTransform> transform =	vtkSmartPointer<vtkTransform>::New();
		transform->PostMultiply(); //this is the key line
		transform->RotateY(-90.0);
		transform->RotateX(90);
		transform->Translate(-0.1, 0.7, -0.1);
		vtkMatrix4x4 *mat = transform->GetMatrix();
		for (int i = 0; i < 3; i++){
			for (int j = 0; j < 4; j++){
				GlobalShared::slurry->m_regMatrix[i][j] = mat->GetElement(i, j);
			}
		}
		volume->SetUserTransform(transform);

		ui.vtkWindow->AddActor(volume, "CT");
		ui.vtkWindow->update();
	}
	else
	{
		QMessageBox::information(this, "错误", "请先载入CT影像！");
	}
}

void Slurry::ctDebug()
{

}

void Slurry::ctShowSkin(bool show)
{
	CT_DATA *m = TOCTDATA(m_ctData);
	if (m){
		if (m->skin){
			m->skin->SetVisibility(show);
			m->bone->SetVisibility(show);
			m->sagittal->SetVisibility(!show);
			m->coronal->SetVisibility(!show);
			m->axial->SetVisibility(!show);
			ui.ctWindow->update();
		}
	}
}
//
//static vtkSmartPointer<vtkPolyData> ReadPolyData(const char *fileName)
//{
//	vtkSmartPointer<vtkPolyData> polyData;
//	std::string extension = vtksys::SystemTools::GetFilenameExtension(std::string(fileName));
//	if (extension == ".ply")
//	{
//		vtkSmartPointer<vtkPLYReader> reader =
//			vtkSmartPointer<vtkPLYReader>::New();
//		reader->SetFileName(fileName);
//		reader->Update();
//		polyData = reader->GetOutput();
//	}
//	else if (extension == ".vtp")
//	{
//		vtkSmartPointer<vtkXMLPolyDataReader> reader =
//			vtkSmartPointer<vtkXMLPolyDataReader>::New();
//		reader->SetFileName(fileName);
//		reader->Update();
//		polyData = reader->GetOutput();
//	}
//	else if (extension == ".obj")
//	{
//		vtkSmartPointer<vtkOBJReader> reader =
//			vtkSmartPointer<vtkOBJReader>::New();
//		reader->SetFileName(fileName);
//		reader->Update();
//		polyData = reader->GetOutput();
//	}
//	else if (extension == ".stl")
//	{
//		vtkSmartPointer<vtkSTLReader> reader =
//			vtkSmartPointer<vtkSTLReader>::New();
//		reader->SetFileName(fileName);
//		reader->Update();
//		polyData = reader->GetOutput();
//	}
//	else
//	{
//		vtkSmartPointer<vtkPointSource> points =
//			vtkSmartPointer<vtkPointSource>::New();
//		points->SetNumberOfPoints(1000);
//		points->SetRadius(1.0);
//		points->SetCenter(vtkMath::Random(-1, 1),
//			vtkMath::Random(-1, 1),
//			vtkMath::Random(-1, 1));
//		points->SetDistributionToShell();
//		points->Update();
//		polyData = points->GetOutput();
//	}
//	return polyData;
//}


//
//static vtkSmartPointer<vtkActor> readSTL()
//{
//	std::string inputFilename = "model.stl";
//
//	vtkSmartPointer<vtkSTLReader> reader =
//		vtkSmartPointer<vtkSTLReader>::New();
//	reader->SetFileName(inputFilename.c_str());
//	reader->Update();
//
//	// Visualize
//	vtkSmartPointer<vtkPolyDataMapper> mapper =
//		vtkSmartPointer<vtkPolyDataMapper>::New();
//	mapper->SetInputConnection(reader->GetOutputPort());
//
//	vtkSmartPointer<vtkActor> actor =
//		vtkSmartPointer<vtkActor>::New();
//	actor->SetMapper(mapper);
//}
//
