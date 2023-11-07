#pragma once

#include <iostream>
#include <vector>
#include <string>

using namespace std;

#define ROBOT_PI 3.141592654

enum ROBOT_TYPE_INFO {
	Yaskawa = 0,
	Rokae = 1,
};

class BaseRobotArm{
protected:
	ROBOT_TYPE_INFO _type;
    static BaseRobotArm* _robot;
private:
public:
	ROBOT_TYPE_INFO GetRobotTypeInfo() {
		return this->_type;
	}
    static BaseRobotArm* GetSingleInstance(BaseRobotArm* instance);
	virtual void InitRobot() = 0;
	virtual std::vector<double> ReadRobotArmPos() = 0;
	virtual std::string ReadRobotArmPosString() = 0;
	virtual void Move2OnePoint(std::string point, int moveMode) = 0;
	virtual ~BaseRobotArm() {};
};

// »úÆ÷±Û¾ä±ú
extern BaseRobotArm* robotArmHandle;