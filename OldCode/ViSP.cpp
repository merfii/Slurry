#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImagePoint.h>

int main1()
{
	try {
		vpImage<vpRGBa> I;
		vpImageIo::read(I, "monkey.bgm");

#if defined(VISP_HAVE_X11)
		vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
		vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
		vpDisplayOpenCV d(I);
#else
		std::cout << "No image viewer is available..." << std::endl;
#endif
		vpDisplay::setTitle(I, "Monkey");
		vpDisplay::display(I);

		vpDisplay::displayRectangle(I, vpImagePoint(90, 90), 70, 90, vpColor::red, false, 2);
		vpDisplay::flush(I);

		vpImage<vpRGBa> O;
		vpDisplay::getImage(I, O);

		try {
			vpImageIo::write(I, "monkey-out.jpg");
		}
		catch (...) {
			std::cout << "Cannot write the image: unsupported format..." << std::endl;
		}

		vpDisplay::getClick(I);
	}
	catch (vpException &e) {
		std::cout << "Catch an exception: " << e << std::endl;
	}
	return 0;
}


#include <visp3/gui/vpPlot.h>
void test_disp()
{
	// Create a window (700 by 700) at position (100, 200) with two graphics
	vpPlot A(2, 700, 700, 100, 200, "Curves...");
	// The first graphic contains 1 curve and the second graphic contains 2 curves
	A.initGraph(0, 1);
	A.initGraph(1, 2);
	// The color of the curve in the first graphic is red
	A.setColor(0, 0, vpColor::red);
	// The first curve in the second graphic is green
	A.setColor(1, 0, vpColor::green);
	// The second curve in the second graphic is blue
	A.setColor(1, 1, vpColor::blue);
	// Add the point (0,0) in the first graphic
	A.plot(0, 0, 0, 0);
	// Add the point (0,1) to the first curve of the second graphic
	A.plot(1, 0, 0, 1);
	// Add the point (0,2) to the second curve of the second graphic
	A.plot(1, 1, 0, 2);
	for (int i = 0; i < 50; i++) {
		// Add the point (i,sin(i*pi/10) in the first graphic
		A.plot(0, 0, i, sin(i*M_PI / 10));
		// Add the point (i,1) to the first curve of the second graphic
		A.plot(1, 0, i, 1);
		// Add the point (i,2) to the second curve of the second graphic
		A.plot(1, 1, i, 2);
	}
}