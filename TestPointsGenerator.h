#include <QThread>
#include <memory>
#include <random>
#include "ScanVisualizer.h"
#include "GlobalShared.h"

class TestPointsGenerator : public QThread
{
	Q_OBJECT
public:
	void run() override;
	void creatPoints();
	void closeGenerator();

	int count;
	std::default_random_engine rengine;
	bool should_exit;
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