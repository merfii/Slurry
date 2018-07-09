#include <QThread>
#include <memory>
#include <random>
#include "ScanVisualizer.h"

class TestPointsGenerator : public QThread
{
	Q_OBJECT
public:
	void run() override;
	void creatPoints();

	int count;
	std::default_random_engine rengine;
	ScanVisualizer *scanVisualizer;
};


//#include "TestPointsGenerator.h"
//
//void Slurry::createPointsGenerator()
//{
//	TestPointsGenerator *pts = new TestPointsGenerator;
//	pts->scanVisualizer = scanVisualizer;
//	pts->start();
//}
//