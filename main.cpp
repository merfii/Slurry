#include <QtWidgets/QApplication>
#include <vector>
#include "slurry.h"
#include "GlobalShared.h"

namespace GlobalShared
{
	QApplication* app = nullptr;
	Slurry* slurry = nullptr;
}



int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	GlobalShared::app = &app;

	Slurry w;
	w.show();

	return app.exec();
}
