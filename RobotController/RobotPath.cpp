#include <fstream>
#include "SystemParameter.h"
#include "Logger.h"

#include "RobotPath.h"

//#include <QTest>
//#include <QMessageBox>


RobotPath::RobotPath()
{

}

void RobotPath::clear()
{
	m_path.clear();
}

int RobotPath::size() const
{
	return m_path.size();
}

void RobotPath::push_back(RobotFrame &frm)
{
	Logger::Debug(QString("Add point;") + QString::fromStdString(frm.tcpToString()));
	m_path.push_back(frm);
}

RobotFrame& RobotPath::operator[](const int idx)
{
	return m_path[idx];
}

const RobotFrame& RobotPath::operator[](const int idx) const
{
	return m_path[idx];
}

std::vector<RobotFrame>& RobotPath::GetData()
{
	return m_path;
}

bool RobotPath::load(const std::string &filename)
{
	m_path.clear();

	std::fstream file;
	file.open(filename, std::fstream::in);		//std::fstream::binary);
	if (!file) {
		file.close();
		return false;
	}
	while (!file.eof())
	{
		char buf[256];
		buf[0] = '\0';
		file.getline(buf, 256);
		if ('\0' == buf[0])
			break;
		m_path.push_back(RobotFrame::fromString(buf));
	}
	return true;
}

/* Qt风格文件写入
QFile file(filename.c_str());
if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
return false;

while (!file.atEnd()) {
std::string line = file.readLine().toStdString();
RobotFrame frm = RobotFrame::fromString(line);
m_path.push_back(frm);
}
file.close();
*/
void RobotPath::save(const std::string &filename)
{
	std::fstream file;
	file.open(filename.c_str(), std::fstream::out | std::fstream::trunc);		//std::fstream::binary);
	if (!file) {
		file.close();
		return;
	}

	for (int i = 0; i < m_path.size(); i++)
	{
		file << m_path[i].tcpToString() << std::endl;
	}
	file.close();
}



/*

vpHinkley可以检测平均值的跳变  用于拟合轨迹中的直线

*/