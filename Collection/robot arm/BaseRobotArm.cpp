#include "BaseRobotArm.h"

// »úÆ÷±Û¾ä±ú
BaseRobotArm* robotArmHandle;

BaseRobotArm* BaseRobotArm::_robot = nullptr;

BaseRobotArm* BaseRobotArm::GetSingleInstance(BaseRobotArm* instance = nullptr) {
	if (nullptr == BaseRobotArm::_robot) {
		BaseRobotArm::_robot = instance;
	}
	return BaseRobotArm::_robot;
}