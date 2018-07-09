#include <opencv2\core.hpp>
#include <iostream>


static void DebugTimeElapse(bool disp)
{
	static long long int t1;
	if(!disp)
		t1 = cv::getTickCount();
	else
	{
		std::cout << "Elapse:" << (cv::getTickCount() - t1) / cv::getTickFrequency() * 1000 << std::endl;;	//ms
	}
}
