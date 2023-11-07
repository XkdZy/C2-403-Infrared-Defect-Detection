#pragma once
#include <iostream>
#include "BaseRobotArm.h"
#include "SerialPort.h"

class YaskawaRobotArm : public BaseRobotArm {
private:
	SerialPort sp;
protected:

public:
	// ÖØÐ´¸¸Àà´¿Ðéº¯Êý
	void InitRobot();
	std::vector<double> ReadRobotArmPos();
	std::string  ReadRobotArmPosString();
	void Move2OnePoint(std::string point, int moveMode);
};

