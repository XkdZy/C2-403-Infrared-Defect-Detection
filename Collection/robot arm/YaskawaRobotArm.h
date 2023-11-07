#pragma once
#include <iostream>
#include "BaseRobotArm.h"
#include "SerialPort.h"

class YaskawaRobotArm : public BaseRobotArm {
private:
	SerialPort sp;
protected:

public:
	// ��д���ി�麯��
	void InitRobot();
	std::vector<double> ReadRobotArmPos();
	std::string  ReadRobotArmPosString();
	void Move2OnePoint(std::string point, int moveMode);
};

