#pragma once
#include <QString>
#include <QSharedPointer>
#include <string>
#include <opencv2/core.hpp>

#define GLOBAL_SETTING_FILE "SystemParameters/Settings.xml"
#define CALIB_SETTING_FILE "SystemParameters/CalibrationSetting.xml"
#define CAMERA_COLOR_A_PARAM_FILE "SystemParameters/CameraParameterA.yml"
#define CAMERA_COLOR_B_PARAM_FILE "SystemParameters/CameraParameterB.yml"
#define HAND_EYE_A_PARAM_FILE "SystemParameters/HandEyeAParameter.txt"
#define HAND_EYE_B_PARAM_FILE "SystemParameters/HandEyeBParameter.txt"
#define TOOL_PARAM_FILE "SystemParameters/ToolParameter.txt"
//#define STEREO_PARAM_FILE "SystemParameters/StereoParameter.yml"

class CalibrationSetting
{
public:
	CalibrationSetting();
	~CalibrationSetting();

	bool save(std::string filename = std::string());
	bool load(std::string filename = std::string());
	
	bool validate();
	cv::Size boardSize;              // The size of the board -> Number of items by width and height
	float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
	int nrFrames;                // The number of frames to use from the input for calibration
	float aspectRatio;           // The aspect ratio
	bool writePoints;            // Write detected feature points
	bool writeExtrinsics;        // Write extrinsic parameters
	bool calibZeroTangentDist;   // Assume zero tangential distortion
	bool calibFixPrincipalPoint; // Fix the principal point at the center
	bool flipVertical;           // Flip the captured images around the horizontal axis
	std::string outputFileName;       // The name of the file where to write
	bool showUndistorsed;        // Show undistorted images after calibration

	int cameraID;
	int flag;

	QString errstr;

private:
	static bool readStringList(const std::string& filename, std::vector<std::string>& l);
	void read(const cv::FileNode& node);                         //Read serialization for this class
};

class CameraParameter
{
public:

	CameraParameter();
	~CameraParameter();

	bool save(std::string filename);
	bool load(std::string filename);
	std::string toString();

	cv::Mat cameraMatrix, distCoeffs;
	std::vector<float> reprojErrs;
	double totalAvgErr;
	double rms;
	std::vector<cv::Mat> rvecs, tvecs;
//	cv::Mat extrPara;
	cv::Mat laserPlane;
	std::vector<double> hand2eye;

	std::string calibration_time;
	int nrFrames;
	cv::Size imageSize;
	cv::Size boardSize;
	int square_size;
	int flag;
	float aspectRatio;
	QString errstr;
	bool ok;

};

/*
struct LaserPlane
{
	double params[4];
	double *A = params;
	double *B = params + 1;
	double *C = params + 2;
	double *D = params + 3;
};
*/

class StereoParameter
{
public:
	StereoParameter();
	~StereoParameter();
	 
	bool save(std::string filename = std::string());
	bool load(std::string filename = std::string());

	cv::Mat R, T, E, F;
	cv::Mat R1, R2, P1, P2, Q;

	//laser parameters
	cv::Mat a, b, c;	//plane in camera coordinate
	
	double totalAvgErr;

	QString errstr;
};


class QXmlQuery;
class SystemParameter
{
public:
	virtual ~SystemParameter();
	static SystemParameter* GetInstance();
	static void Reload();

	static CalibrationSetting* GetCalibSetting();
	static CameraParameter* GetCameraColorAParam();
	static CameraParameter* GetCameraColorBParam();
	static StereoParameter* GetStereoParam();

	std::vector<double> GetToolMatrix() const{ return toolMatrix; }
	//Generata Settings
	QString GetSettingString(const QString &path);
	int		GetSettingInt(const QString &path);
	double	GetSettingDouble(const  QString &path);

protected:
	SystemParameter();
	
private:
	void _loadSystemParameters();
	void _loadCameraParameters();
	bool _loadToolParameters();
	QString resolvePath(const QString &path);
	SystemParameter(const SystemParameter&);	//Not implemented
	void operator=(const SystemParameter&);		//Not implemented

	static QSharedPointer<SystemParameter> mpSysParam;
	QSharedPointer<CalibrationSetting> mpCalibSetting;
	QSharedPointer<CameraParameter> mpCamColorAParam, mpCamColorBParam;
	QSharedPointer<StereoParameter> mpStereoParam;
	std::vector<double> toolMatrix;
	QXmlQuery *query;
	QString errstr;


};