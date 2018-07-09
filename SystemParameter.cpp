#pragma execution_character_set("utf-8")

#include <limits>
#include <string>
#include <iostream>
#include <QTimer>
#include <QFile>
#include <QMessageBox>
#include <QDomDocument>
#include <QtXmlPatterns/QXmlQuery> 

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "GlobalShared.h"
#include "Logger.h"
#include "Slurry.h"
#include "SystemParameter.h"

using namespace cv;

QSharedPointer<SystemParameter> SystemParameter::mpSysParam;

SystemParameter::SystemParameter()
{
	query = new QXmlQuery;
	_loadSystemParameters();
	_loadCameraParameters();
	if (!_loadToolParameters())
		Logger::WarnAndQuit("无法载入导引筒参数 " + QString(TOOL_PARAM_FILE));
}

void SystemParameter::Reload()
{
	GetInstance()->_loadSystemParameters();
	GetInstance()->_loadCameraParameters();
	if (!GetInstance()->_loadToolParameters())
		Logger::WarnAndQuit("无法载入导引筒参数 " + QString(TOOL_PARAM_FILE));
}

void SystemParameter::_loadSystemParameters()
{
	QUrl url = QUrl::fromLocalFile(GLOBAL_SETTING_FILE);
	if (!query->setFocus(url))
	{
		errstr = QStringLiteral("Fail to open setting file.");
		Logger::WarnAndQuit(errstr);
	}

	mpCalibSetting = QSharedPointer<CalibrationSetting>(new CalibrationSetting);
	if (!mpCalibSetting->load())
	{
		Logger::WarnAndQuit(mpCalibSetting->errstr);
	}
}


void SystemParameter::_loadCameraParameters()
{
	mpCamColorAParam = QSharedPointer<CameraParameter>(new CameraParameter);
	if (!mpCamColorAParam->load(CAMERA_COLOR_A_PARAM_FILE))
	{
		Logger::WarnAndQuit("无法载入彩色相机A参数" + mpCamColorAParam->errstr);
	}

	mpCamColorBParam = QSharedPointer<CameraParameter>(new CameraParameter);
	if (!mpCamColorBParam->load(CAMERA_COLOR_B_PARAM_FILE))
	{
		Logger::WarnAndQuit("无法载入彩色相机B参数" + mpCamColorBParam->errstr);
	}
}

bool SystemParameter::_loadToolParameters()
{
	std::string line;
	std::ifstream in(TOOL_PARAM_FILE);  //读入文件
	std::istringstream istream;

	toolMatrix.clear();
	if (!in.is_open())
		return false;
	while (getline(in, line)) {  //按行读取	
		if (line.empty())
			continue;
		double num[4] = { DBL_MIN, DBL_MIN, DBL_MIN, DBL_MIN };
		istream.str(line);
		istream >> num[0] >> num[1] >> num[2] >> num[3];
		for (int i = 0; i < 4; i++)
		{
			if (num[i] != DBL_MIN)
				toolMatrix.push_back(num[i]);
			else
				return false;
		}
		istream.clear();
	}
	
	if (toolMatrix.size() < 16)
		return false;
//DEBUG
	//for (auto it = toolMatrix.begin(); it != toolMatrix.end(); it++)
	//{
	//	std::cout << *it << " ";
	//}
	//std::cout << std::endl;
	return true;
}

SystemParameter::~SystemParameter()
{
	delete query;
}

SystemParameter* SystemParameter::GetInstance()
{
	if (mpSysParam.isNull())
	{
		mpSysParam = QSharedPointer<SystemParameter>(new SystemParameter);
	}
	return mpSysParam.data();
}

CalibrationSetting* SystemParameter::GetCalibSetting()
{
	return GetInstance()->mpCalibSetting.data();
}

CameraParameter* SystemParameter::GetCameraColorAParam()
{
	return GetInstance()->mpCamColorAParam.data();
}

CameraParameter* SystemParameter::GetCameraColorBParam()
{
	return GetInstance()->mpCamColorBParam.data();
}

/*
StereoParameter* SystemParameter::GetStereoParam()
{
	if (mpStereoParam.isNull())
	{
		mpStereoParam = QSharedPointer<StereoParameter>(new StereoParameter);
		if (!mpStereoParam->load(STEREO_PARAM_FILE))
		{
			//Logger::WarnAndQuit(mpStereoParam->errstr);
			Logger::PrintLog("无法载入灰度相机参数 使用默认");
		}
	}
	return mpStereoParam.data();
}
*/

QString SystemParameter::resolvePath(const QString &path)
{
	QString outString;
	query->setQuery(path);
	if (!query->isValid())
	{
		return QString();
	}
	else
	{
		query->evaluateTo(&outString);
		QDomDocument domDoc;
		domDoc.setContent(outString);
		return domDoc.firstChild().toElement().text();

		//qDebug()<<domDoc.firstChild().toElement().text();
	}
}


QString SystemParameter::GetSettingString(const QString &path)
{
	return resolvePath(path);
}


int	SystemParameter::GetSettingInt(const QString &path)
{
	QString str = resolvePath(path);
	if (str.isEmpty())
	{
		return 0;
	}
	else
	{
		return str.toInt();
	}
}

double	SystemParameter::GetSettingDouble(const  QString &path)
{

	QString str = resolvePath(path);
	if (str.isEmpty())
	{
		Logger::WarnAndQuit("Can't get " + path);
		return 0;
	}
	else
	{
		return str.toDouble();
	}

}


CalibrationSetting::CalibrationSetting()
{
	;
}

CalibrationSetting::~CalibrationSetting()
{
	;
}


bool CalibrationSetting::load(std::string filename)
{
	if (filename.empty())
	{
		filename = CALIB_SETTING_FILE;
	}
	FileStorage fs(filename, FileStorage::READ); // Read the settings
	if (!fs.isOpened())
	{
		errstr = QString("无法载入参数配置文件: %1\n").arg(filename.c_str());
		return false;
	}
	read(fs["Settings"]);
	fs.release();

	if (!validate())
	{
		errstr = QString("文件%1内容无效").arg(filename.c_str());
		return false;
	}
	return true;
}

bool CalibrationSetting::save(std::string filename)
{
	if (filename.empty())
	{
		filename = CALIB_SETTING_FILE;
	}
	FileStorage fs(filename, FileStorage::READ); // Read the settings
	if (!fs.isOpened())
	{
		errstr = QString("无法写入参数配置文件: %1\n").arg(filename.c_str());
		return false;
	}

	fs << "{"
		<< "BoardSize_Width" << boardSize.width
		<< "BoardSize_Height" << boardSize.height
		<< "Square_Size" << squareSize
		<< "Calibrate_NrOfFrameToUse" << nrFrames
		<< "Calibrate_FixAspectRatio" << aspectRatio
		<< "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
		<< "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

		<< "Write_DetectedFeaturePoints" << writePoints
		<< "Write_extrinsicParameters" << writeExtrinsics
		<< "Write_outputFileName" << outputFileName

		<< "Input_FlipAroundHorizontalAxis" << flipVertical
		<< "Show_UndistortedImage" << showUndistorsed
		<< "}";
	fs.release();
	return true;
}

void CalibrationSetting::read(const FileNode& node)                          //Read serialization for this class
{
	node["BoardSize_Width"] >> boardSize.width;
	node["BoardSize_Height"] >> boardSize.height;
	node["Square_Size"] >> squareSize;
	node["Calibrate_NrOfFrameToUse"] >> nrFrames;
	node["Calibrate_FixAspectRatio"] >> aspectRatio;
	node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
	node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;

	node["Write_DetectedFeaturePoints"] >> writePoints;
	node["Write_extrinsicParameters"] >> writeExtrinsics;
	node["Write_outputFileName"] >> outputFileName;

	node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
	node["Show_UndistortedImage"] >> showUndistorsed;

	flag = CALIB_FIX_K4 | CALIB_FIX_K5;
	if (calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
	if (calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
	if (aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;
}

bool CalibrationSetting::validate()
{
	bool goodInput = true;
	if (boardSize.width <= 0 || boardSize.height <= 0)
	{
		std::cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << std::endl;
		goodInput = false;
	}
	if (squareSize <= 10e-6)
	{
		std::cerr << "Invalid square size " << squareSize << std::endl;
		goodInput = false;
	}
	if (nrFrames <= 0)
	{
		std::cerr << "Invalid number of frames " << nrFrames << std::endl;
		goodInput = false;
	}

	return goodInput;
}

bool CalibrationSetting::readStringList(const std::string& filename, std::vector<std::string>& l)
{
	l.clear();
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((std::string)*it);
	return true;


}


CameraParameter::CameraParameter()
{
	//default parameters
	nrFrames = 0;
	square_size = 0;
	aspectRatio = 1;
	flag = 0;
	
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	distCoeffs = Mat::zeros(5, 1, CV_64F);
	rms = totalAvgErr = std::numeric_limits<double>::max();
	
	laserPlane.create(1, 4, CV_64FC1);
	reprojErrs.resize(1, -1);
	ok = false;
}


CameraParameter::~CameraParameter()
{

}

bool CameraParameter::save(std::string filename)
{
	if (filename.empty())
	{
		return false;
	}

	FileStorage fs(filename, FileStorage::WRITE);
	if (!fs.isOpened())
	{
		errstr = QString("无法写入参数配置文件: %1\n").arg(filename.c_str());
		return false;
	}

	time_t tt;
	char buf[256];
	time(&tt);
	struct tm *ts = localtime(&tt);
	strftime(buf, sizeof(buf), "%c", ts);

	fs << "calibration_time" << buf;

	fs << "nr_of_frames" << nrFrames;
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << boardSize.width;
	fs << "board_height" << boardSize.height;
	fs << "square_size" << square_size;

	if (flag & CALIB_FIX_ASPECT_RATIO)
		fs << "fix_aspect_ratio" << aspectRatio;

	if (flag)
	{
		sprintf(buf, "flags:%s%s%s%s",
			flag & CALIB_USE_INTRINSIC_GUESS ? " + use_intrinsic_guess" : "",
			flag & CALIB_FIX_ASPECT_RATIO ? " + fix_aspectRatio" : "",
			flag & CALIB_FIX_PRINCIPAL_POINT ? " + fix_principal_point" : "",
			flag & CALIB_ZERO_TANGENT_DIST ? " + zero_tangent_dist" : "");
		cvWriteComment(*fs, buf, 0);
	}

	fs << "flags" << flag;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;
	if (!reprojErrs.empty())
		fs << "per_view_reprojection_errors" << Mat(reprojErrs);

	cvWriteComment( *fs, "Rotation vector + Translation vector for each view", 0 );
	
	fs << "laser_plane" << laserPlane;
	fs << "hand2eye" << hand2eye;

	fs << "rvecs" << rvecs;
	fs << "tvecs" << tvecs;
	
	/*
	if (settings.writePoints && !imagePoints.empty())
	{
		Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
		for (size_t i = 0; i < imagePoints.size(); i++)
		{
			Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
			Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "image_points" << imagePtMat;
	}
	*/
	fs.release();
	return true;
}

bool CameraParameter::load(std::string filename)
{
	if (filename.empty())
	{
		return false;
	}
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		errstr = QString("无法打开参数配置文件: %1\n").arg(filename.c_str());
		return false;
	}

	fs["calibration_time"] >> calibration_time;
	fs["nr_of_frames"] >> nrFrames;
	fs["image_width"] >> imageSize.width;
	fs["image_height"] >> imageSize.height;
	fs["board_width"] >> boardSize.width;
	fs["board_height"] >> boardSize.height;
	fs["square_size"] >> square_size;
	fs["fix_aspect_ratio"] >> aspectRatio;
	fs["flags"] >> flag;

	fs["camera_matrix"] >> cameraMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	
	fs["avg_reprojection_error"] >> totalAvgErr;
	//fs["extrinsic_parameters"] >> extrPara;

	fs["laser_plane"] >> laserPlane;
	//将矩阵串行化成vector 行优先 用vpHomogeneousMatrix::buildFrom载入
	fs["hand2eye"] >> hand2eye;

	fs.release();
	ok = true;
	return true;
}

std::string CameraParameter::toString()
{
	std::ostringstream o;
	o << "Image Width: " << imageSize.width << std::endl;
	o << "Image Height: " << imageSize.height << std::endl;
	o << "Camera Matrix:\n" << cameraMatrix << std::endl;
	o << "Distortion Coefficients:\n" << distCoeffs << std::endl;
	o << "Avg Reprj Error: " << totalAvgErr << std::endl;
//	o << "Extrinsic Parameters" << extrPara;
	return o.str();
}


StereoParameter::StereoParameter()
{

}

StereoParameter::~StereoParameter()
{

}

bool StereoParameter::save(std::string filename)
{
	if (filename.empty())
	{
		return false;
	}

	FileStorage fs(filename, FileStorage::WRITE);
	if (!fs.isOpened())
	{
		errstr = QString("无法写入参数配置文件: %1\n").arg(filename.c_str());
		return false;
	}

	fs.release();
	return true;
}


bool StereoParameter::load(std::string filename)
{
	if (filename.empty())
	{
		return false;
	}
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		errstr = QString("无法打开参数配置文件: %1\n").arg(filename.c_str());
		return false;
	}
	return true;
}



/*
用于软件系统配置，需要在程序的各个位置分布访问，

********************考虑使用QSettings**********************

希望载入的配置能按模块组织成树状结构，并按字符串路径访问
对于xml必须用XPath
*/

//qDebug()<<GlobalShared::Setting::GetDouble("Settings/CameraControl/ExposureTime");


	/*
	装载xml文件  打开文件对话框
	QString fileName = QString
	QFileDialog::getOpenFileName(this, tr("Open Bookmark File"),
	QDir::currentPath(),
	tr("XBEL Files (*.xbel *.xml)"));

	if (xml.readNextStartElement() && xml.attributes().value("version") == "1.0")
	{
	while (xml.readNextStartElement()) {
	if (xml.name() == "CameraControl")
	readCameraControl(0);
	else if (xml.name() == "main")
	readMain(0);
	else
	xml.skipCurrentElement();
	}
	if (xml.atEnd() && !xml.hasError())
	{
	isLoadSuc = true;
	}
	}
	*/

/*
	json版本Setting
	static QJsonObject json;
	static QRegularExpression re("[(\\d+)]");

	void Slurry::initializeSettings()
	{
	QJsonParseError pErr;
	do
	{
	QFile file(QString(FILENAME));
	if (!file.open(QFile::ReadOnly | QFile::Text))
	break;
	QJsonDocument jdoc(QJsonDocument::fromJson(file.readAll(),&pErr));
	if (jdoc.isNull() || jdoc.isEmpty())
	{
	break;
	}
	json = jdoc.object();
	return;
	} while (0);

	//弹出对话框提示
	QMessageBox::warning(this, QStringLiteral("错误"), QString("无法载入参数配置文件: %1\n").arg(FILENAME) + pErr.errorString());
	//主程序退出
	QTimer::singleShot(0, QApplication::instance(), SLOT(quit()));
	}


	static QJsonValue resolveJsonPath(const QString &path)
	{
	QStringList paths = path.split(QChar('/'), QString::SkipEmptyParts);
	QJsonObject jobj = json;

	for (int i = 0; i < paths.size() - 1; i++)
	{
	//qDebug() << QString().setNum(jobj.count());
	const QString &p = paths[i];
	jobj = jobj[p].toObject();
	}
	const QString &p = paths.last();
	QRegularExpressionMatch match = re.match(p);
	if (match.hasMatch())
	{
	QString dd = match.captured(0);
	int idx = match.captured(0).toInt();
	QString keyName = p.left(match.capturedStart(0) - 1);
	QJsonValue v = jobj[keyName];
	if (v.isArray())
	{
	return v.toArray()[idx];
	}
	else
	{
	return v;
	}

	}else
	{
	QJsonValue v = jobj[p];
	if (v.isArray())
	{
	return v.toArray()[0];
	}
	else
	{
	return v;
	}
	}
	}


	QString GlobalShared::SettingGetString(const QString &path)
	{
	QJsonValue val = resolveJsonPath(path);
	if (val.isUndefined())
	return QString::null;
	else
	return val.toString();
	}

	int	GlobalShared::SettingGetInt(const QString &path)
	{
	QJsonValue val = resolveJsonPath(path);
	if (val.isDouble())
	return qRound(val.toDouble());
	else
	return 0;
	}

	double GlobalShared::SettingGetDouble(const  QString &path)
	{
	QJsonValue val = resolveJsonPath(path);
	if (val.isDouble())
	return val.toDouble();
	else
	return 0;
	}
*/

