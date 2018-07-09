                              #include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

//OpenCV内部实现了RANSAC算法 但没有公开头文件
//这里从源码里抽出了头文件   经验证可以使用
namespace cv
{

	//int RANSACUpdateNumIters(double p, double ep, int modelPoints, int maxIters);

	//class CV_EXPORTS LMSolver : public Algorithm
	//{
	//public:
	//	class CV_EXPORTS Callback
	//	{
	//	public:
	//		virtual ~Callback() {}
	//		virtual bool compute(InputArray param, OutputArray err, OutputArray J) const = 0;
	//	};

	//	virtual void setCallback(const Ptr<LMSolver::Callback>& cb) = 0;
	//	virtual int run(InputOutputArray _param0) const = 0;
	//};

	//CV_EXPORTS Ptr<LMSolver> createLMSolver(const Ptr<LMSolver::Callback>& cb, int maxIters);

	class CV_EXPORTS PointSetRegistrator : public Algorithm
	{
	public:
		class CV_EXPORTS Callback
		{
		public:
			virtual ~Callback() {}
			virtual int runKernel(InputArray m1, InputArray m2, OutputArray model) const = 0;
			virtual void computeError(InputArray m1, InputArray m2, InputArray model, OutputArray err) const = 0;
			virtual bool checkSubset(InputArray, InputArray, int) const { return true; }
		};

		virtual void setCallback(const Ptr<PointSetRegistrator::Callback>& cb) = 0;
		virtual bool run(InputArray m1, InputArray m2, OutputArray model, OutputArray mask) const = 0;
	};

	CV_EXPORTS Ptr<PointSetRegistrator> createRANSACPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& cb,
		int modelPoints, double threshold,
		double confidence = 0.99, int maxIters = 1000);

	CV_EXPORTS Ptr<PointSetRegistrator> createLMeDSPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& cb,
		int modelPoints, double confidence = 0.99, int maxIters = 1000);

}





/************************************************************************************************
*************************************************************************************************
用RANSAC算法解算平面的代码
实践证明由于数据的高度线性相关 RANSAC算法完全不适用！！！！
*************************************************************************************************
*************************************************************************************************/
//被算法调用的类接口
class LaserPlaneSolverCallback : public cv::PointSetRegistrator::Callback
{
public:
	LaserPlaneSolverCallback(int _flags = cv::SOLVEPNP_ITERATIVE)
		:flags(_flags)
	{
		;
	}

	/* Pre: True */
	/* Post: compute _model with given points an return number of found models */
	int runKernel(cv::InputArray _m1, cv::InputArray _m2, cv::OutputArray _model) const
	{
		static int count;
		cout << "Count: " << count++ << endl << endl;
		Mat points = _m1.getMat(), nodata = _m2.getMat();
		CV_Assert(points.rows == 3 && points.cols == 1);
		const double *A = points.ptr<double>(0);
		const double *B = points.ptr<double>(1);
		const double *C = points.ptr<double>(2);

		double plA, plB, plC, plD;

		plA = (B[1] - A[1])*(C[2] - A[2]) * (C[1] - A[1])*(B[2] - A[2]);
		plB = (B[2] - A[2])*(C[0] - A[0]) * (C[2] - A[2])*(B[0] - A[0]);
		plC = (B[0] - A[0])*(C[1] - A[1]) * (C[0] - A[0])*(B[1] - A[1]);

		if (abs(plA) < DOUBLE_EPS && abs(plB) < DOUBLE_EPS && abs(plC) < DOUBLE_EPS)
		{
			//3个线性点线性相关
			plD = LONG_MAX;
		}
		else
		{
			double plMax = abs(plA) > abs(plB) ? plA : plB;
			plMax = abs(plC) > abs(plMax) ? plC : plMax;
			plA /= plMax;
			plB /= plMax;
			plC /= plMax;
			plD = -(plA * A[0] + plB * A[1] + plC * A[1]);
		}

		/*
		//		使用Eigen 的代码 结果实质上相同
		#include <Eigen/dense>
		double A, B, C, D;	//Ax+By+Cz+D=0;
		Eigen::Matrix3d matrix;

		matrix <<
		1, p1[1], p1[2],		// 1 y1 z1
		1, p2[1], p2[2],		// 1 y2 z2
		1, p3[1], p3[2];		// 1 y3 z3
		A = matrix.determinant();

		matrix <<
		p1[0], 1, p1[2],		// x1 1 z1
		p2[0], 1, p2[2],		// x2 1 z2
		p3[0], 1, p3[2];		// x3 1 z3
		B = matrix.determinant();

		matrix <<
		p1[0], p1[1], 1,		// x1 y1 z1
		p2[0], p2[1], 1,		// x2 y2 z2
		p3[0], p3[1], 1;		// x3 y3 z3
		C = matrix.determinant();

		matrix <<
		p1[0], p1[1], p1[2],		// x1 y1 z1
		p2[0], p2[1], p2[2],		// x2 y2 z2
		p3[0], p3[1], p3[2];		// x3 y3 z3
		D = -matrix.determinant();
		*/
		Mat model(1, 4, CV_64FC1);
		model.at<double>(0) = plA;
		model.at<double>(1) = plB;
		model.at<double>(2) = plC;
		model.at<double>(3) = plD;
		model.copyTo(_model);
		return 1;
	}

	//_err必须为float类型
	void computeError(cv::InputArray _m1, cv::InputArray _m2, cv::InputArray _model, cv::OutputArray _err) const
	{
		Mat points = _m1.getMat(), ipoints = _m2.getMat();
		Mat model = _model.getMat();
		CV_Assert(points.cols == 1 && model.cols == 4);
		double A, B, C, D;
		A = model.at<double>(0);
		B = model.at<double>(1);
		C = model.at<double>(2);
		D = model.at<double>(3);

		_err.create(points.rows, 1, CV_32FC1);
		float *r = _err.getMat().ptr<float>();
		for (int i = 0; i < points.rows; i++)
		{
			double *p = points.ptr<double>(i);
			r[i] = (float)p[0] * A + p[1] * B + p[2] * C + D;
			r[i] = r[i] * r[i];
			if (i % 1000 == 0)
				cout << "r: " << r[i] << endl;
		}
	}

	//bool checkSubset(cv::InputArray _m1, cv::InputArray _m2, int) const { return true; }
	int flags;
};



void FrmProcessorLaserCalibration::_DATA::_debugSolve()
{
	cv::FileStorage fs("laserPoints3D.yml", cv::FileStorage::WRITE);
	fs << "Points" << laserPoints3D;
	fs.release();

	laserPoints3D.resize(50 * 10);
	for (int i = 0; i < 50; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			laserPoints3D.ptr<double>(i * 10 + j)[0] = j * 10;
			laserPoints3D.ptr<double>(i * 10 + j)[1] = 50 + (j % 5 == 0);
			laserPoints3D.ptr<double>(i * 10 + j)[2] = i * 10 + 100;
		}
	}

	/*
	laserPoints3D.release();
	cv::FileStorage fs("SystemParameters/laserPoints3D.yml", cv::FileStorage::READ);
	fs["Points"] >> laserPoints3D;
	fs.release();
	*/
	cv::Ptr<cv::PointSetRegistrator::Callback> cb = cv::makePtr<LaserPlaneSolverCallback>(0); // pointer to callback
	Mat _mask, _plane;

	//解算平面 RANSAC
	bool result = cv::createRANSACPointSetRegistrator(cb,
		3,	//model points
		10,	//inlier threshold 求解器内部会自动平方
		0.99,	//confidence
		5000	//maximum iterations
		)->run(laserPoints3D, laserPoints3D, _plane, _mask);

	if (result)
	{
		if (abs(_plane.ptr<double>()[3]) > DOUBLE_EPS)	//若平面不过原点 可以用以下算法拟合
		{
			Mat mA, mX, mB;
			mA.create(laserPoints3D.rows, 3, CV_64FC1);
			mX.create(3, 1, CV_64FC1);
			mB.create(laserPoints3D.rows, 1, CV_64FC1);
			//压缩掉outlier 只保留inlier

			int i, j;
			const uchar* mask = _mask.ptr<uchar>();
			for (i = j = 0; i < laserPoints3D.rows; i++)
			{
				double *pts = laserPoints3D.ptr<double>(i);
				if (mask[i])
				{
					mA.ptr<double>(j)[0] = pts[0];
					mA.ptr<double>(j)[1] = pts[1];
					mA.ptr<double>(j)[2] = pts[2];
					mB.ptr<double>(j)[0] = -_plane.at<double>(0, 3);
					j++;
				}
			}
			mA.resize(j);
			mB.resize(j);
			cout << "inLiers: " << j << endl;

			cv::solve(mA, mB, mX, cv::DECOMP_SVD);// cv::DECOMP_NORMAL);
			_plane.at<double>(0) = mX.at<double>(0, 0);
			_plane.at<double>(1) = mX.at<double>(1, 0);
			_plane.at<double>(2) = mX.at<double>(2, 0);

		}
		cout << "Plane: " << _plane << endl << endl;
		//	debugDispLaserPlane(laserPoints3D, _plane);
	}
	//opoints.convertTo(opoints_inliers, CV_64F);

}