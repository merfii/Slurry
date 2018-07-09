#pragma once

#include <vector>
#include <QObject>
#include "RobotFrame.h"


class RobotPath
{
public:
	RobotPath();
	bool load(const std::string &filename);
	void save(const std::string &filename);
	
	int size() const;
	void clear();
	void push_back(RobotFrame &frm);
	RobotFrame& operator[](const int idx);
	const RobotFrame& operator[](const int idx) const;

	std::vector<RobotFrame>& GetData();

private:
	std::vector<RobotFrame> m_path;

};





/*
public:
static RobotTask* GetInstance();
virtual ~RobotTask();

private:
RobotTask(const RobotTask &);	//Not implemented
void operator=(const RobotTask &);	//Not implemented
*/