
/*

vtkSmartPointer<vtkDataSet>
pcl::visualization::createCube (const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation,
double width, double height, double depth)
{
// coefficients = [Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth]
vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New ();
t->Identity ();
t->Translate (translation.x (), translation.y (), translation.z ());

Eigen::AngleAxisf a (rotation);
t->RotateWXYZ (pcl::rad2deg (a.angle ()), a.axis ()[0], a.axis ()[1], a.axis ()[2]);

vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New ();
cube->SetXLength (width);
cube->SetYLength (height);
cube->SetZLength (depth);

vtkSmartPointer<vtkTransformPolyDataFilter> tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
tf->SetTransform (t);
tf->SetInputConnection (cube->GetOutputPort ());
tf->Update ();

return (tf->GetOutput ());
}

vtkSmartPointer<vtkDataSet>
pcl::visualization::createPlane (const pcl::ModelCoefficients &coefficients, double x, double y, double z)
{
vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New ();


double norm_sqr = 1.0 / (coefficients.values[0] * coefficients.values[0] +
coefficients.values[1] * coefficients.values[1] +
coefficients.values[2] * coefficients.values[2] );

//  double nx = coefficients.values [0] * norm;
//  double ny = coefficients.values [1] * norm;
//  double nz = coefficients.values [2] * norm;
//  double d  = coefficients.values [3] * norm;

//  plane->SetNormal (nx, ny, nz);
plane->SetNormal (coefficients.values[0], coefficients.values[1], coefficients.values[2]);

double t = x * coefficients.values[0] + y * coefficients.values[1] + z * coefficients.values[2] + coefficients.values[3];
x -= coefficients.values[0] * t * norm_sqr;
y -= coefficients.values[1] * t * norm_sqr;
z -= coefficients.values[2] * t * norm_sqr;
plane->SetCenter (x, y, z);
plane->Update ();

return (plane->GetOutput ());
}


////////////////////////////////////////////////////////////////////////////////////////////
vtkSmartPointer<vtkDataSet>
pcl::visualization::create2DCircle (const pcl::ModelCoefficients &coefficients, double z)
{
vtkSmartPointer<vtkDiskSource> disk = vtkSmartPointer<vtkDiskSource>::New ();
// Maybe the resolution should be lower e.g. 50 or 25
disk->SetCircumferentialResolution (100);
disk->SetInnerRadius (coefficients.values[2] - 0.001);
disk->SetOuterRadius (coefficients.values[2] + 0.001);
disk->SetCircumferentialResolution (20);

// An alternative to <vtkDiskSource> could be <vtkRegularPolygonSource> with <vtkTubeFilter>
/*
vtkSmartPointer<vtkRegularPolygonSource> circle = vtkSmartPointer<vtkRegularPolygonSource>::New();
circle->SetRadius (coefficients.values[2]);
circle->SetNumberOfSides (100);

vtkSmartPointer<vtkTubeFilter> tube = vtkSmartPointer<vtkTubeFilter>::New();
tube->SetInput (circle->GetOutput());
tube->SetNumberOfSides (25);
tube->SetRadius (0.001);


// Set the circle origin
vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
t->Identity();
t->Translate(coefficients.values[0], coefficients.values[1], z);

vtkSmartPointer<vtkTransformPolyDataFilter> tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
tf->SetTransform(t);
tf->SetInputConnection(disk->GetOutputPort());
tf->Update();

return (tf->GetOutput());
}

*/