#pragma once

#include <iostream>
#include <string>
#include "BaseRobotArm.h"
#include "rokae/robot.h"
#include <chrono>
#include <thread>
#include <Windows.h>

using namespace std;
using namespace rokae;

class RokaeRobotArm : public BaseRobotArm {
private:
	XMateRobot* rokaeRobot;
	string rokaeIP;
	error_code ec;
protected:
	void WaitRokaeRobot(BaseRobot* robot);
	void StopRokaeRobot(); 
	void MoveRokae2OnePoint(const vector<double>& pos, int moveMode);
public:
	// ��д���ി�麯��
	void InitRobot();
	std::vector<double> ReadRobotArmPos();
	std::string  ReadRobotArmPosString();
	void Move2OnePoint(std::string point, int moveMode);
};

