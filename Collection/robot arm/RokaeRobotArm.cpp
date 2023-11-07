#include "RokaeRobotArm.h"
#include "CoordinateTransformation.h"

void RokaeRobotArm::InitRobot() {
    this->rokaeRobot = (XMateRobot*)malloc(sizeof(XMateRobot));
    this->rokaeIP = "192.168.3.159";
	this->_type = ROBOT_TYPE_INFO::Rokae;
    try {
        rokaeRobot = new XMateRobot(rokaeIP);
        //XMateRobot rokaeRobot(rokaeIP); // 连接xMate6轴机型
        auto robotinfo = rokaeRobot->robotInfo(this->ec);
        std::cout << "控制器版本号: " << robotinfo.version << ", 机型：" << robotinfo.type << std::endl;
        std::cout << "RokaeSDK版本: " << rokaeRobot->sdkVersion() << std::endl;
        //WaitRokaeRobot(rokaeRobot);
    }
    catch (exception e) {
        cout << "未连接上机器人!" << endl;
        return;
    }
}

std::vector<double> RokaeRobotArm::ReadRobotArmPos() {
    Arr6 pose = rokaeRobot->flangePos(ec);
    vector<double>vPos(6, 0.0);
    //cout << "当前坐标为：" << endl;
    for (int i = 0; i < 3; i++) {
        vPos[i] = pose[i] * 1000;
        //cout << vPos[i] << "	";
    }
    for (int i = 3; i < 6; i++) {
        vPos[i] = pose[i] * 180 / ROBOT_PI;
        //cout << vPos[i] << "	";
    }
    return vPos;
}

std::string RokaeRobotArm::ReadRobotArmPosString() {
	Arr6 pose = rokaeRobot->flangePos(ec);
	string strPos = "A";
	vector<int> vPosInfo(6, 0);
	for (int i = 0; i < 3; i++) {
		char temp[9];
		vPosInfo[i] = pose[i] * 100000;
		vector<int> vBit(6);
		vBit[0] = vPosInfo[i] / 100000;
		vBit[1] = vPosInfo[i] / 10000 % 10;
		vBit[2] = vPosInfo[i] / 1000 % 10;
		vBit[3] = vPosInfo[i] / 100 % 10;
		vBit[4] = vPosInfo[i] / 10 % 10;
		vBit[5] = vPosInfo[i] % 10;
		if (vPosInfo[i] > 0) strPos += "+";
		else strPos += "-";
		strPos += to_string(abs(vBit[0]));
		strPos += to_string(abs(vBit[1]));
		strPos += to_string(abs(vBit[2]));
		strPos += to_string(abs(vBit[3]));
		strPos += to_string(abs(vBit[4]));
		strPos += to_string(abs(vBit[5]));
	}
	for (int i = 3; i < 6; i++) {
		//cout << pose[i] << "	";
		char temp[9];
		// 08001
		vPosInfo[i] = pose[i] * 18000. / ROBOT_PI;
		//cout << vPosInfo[i] << "	";
		vector<int> vBit(5);
		vBit[0] = vPosInfo[i] / 10000;
		vBit[1] = vPosInfo[i] / 1000 % 10;
		vBit[2] = vPosInfo[i] / 100 % 10;
		vBit[3] = vPosInfo[i] / 10 % 10;
		vBit[4] = vPosInfo[i] / 1 % 10;
		if (vPosInfo[i] > 0) strPos += "+";
		else strPos += "-";
		strPos += to_string(abs(vBit[0]));
		strPos += to_string(abs(vBit[1]));
		strPos += to_string(abs(vBit[2]));
		strPos += to_string(abs(vBit[3]));
		strPos += to_string(abs(vBit[4]));
	}
	strPos.append("BC");

	return strPos;
}


void RokaeRobotArm::WaitRokaeRobot(BaseRobot* robot) {
	bool running = true;
	while (running) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		//error_code ec;
		auto st = robot->operationState(ec);
		if (st == OperationState::idle || st == OperationState::unknown) {
			running = false;
		}
	}
}

void RokaeRobotArm::StopRokaeRobot() {
	rokaeRobot->stop(ec); // 停止运动
	rokaeRobot->setPowerState(false, ec); // 自动模式下上电
	//std::this_thread::sleep_for(std::chrono::seconds(2)); //等待切换控制模式
	std::this_thread::sleep_for(std::chrono::milliseconds(5)); //等待切换控制模式
}


void RokaeRobotArm::Move2OnePoint(std::string point, int moveMode) {
	vector<double> v;
	AnalysisString26D(point, v);
	for (int i = 0; i < 3; i++) {
		v[i] = v[i] / 1000;
	}
	for (int i = 3; i < 6; i++) {
		v[i] = v[i] * ROBOT_PI / 180.;
	}
	//cout << v[0] << "	" << v[1] << "	" << v[2] << "	" << v[3] << "	" << v[4] << "	" << v[5] << "	" << endl;
	MoveRokae2OnePoint(v, moveMode);

	Sleep(500);

	return;
}

void RokaeRobotArm::MoveRokae2OnePoint(const vector<double>& pos, int moveMode) {
	this->rokaeRobot->setMotionControlMode(MotionControlMode::NrtCommand);
	this->rokaeRobot->setOperateMode(OperateMode::automatic, ec); // 切换到自动模式
	this->rokaeRobot->setPowerState(true, ec); // 自动模式下上电
	//std::this_thread::sleep_for(std::chrono::seconds(2)); //等待切换控制模式
	std::this_thread::sleep_for(std::chrono::milliseconds(5)); //等待切换控制模式
	this->rokaeRobot->moveReset(ec); // 发运动指令前必须重置缓存
	if (moveMode == 0) {
		//cout << "pose:" << pos[0] << "	" << pos[1] << "	" << pos[2] << "	"
		//	<< pos[3] << "	" << pos[4] << "	" << pos[5] << "	" << endl;
		rokae::MoveLCommand movel0({ pos[0], pos[1], pos[2], pos[3], pos[4], pos[5] }, 1000, 100);
		//rokae::MoveLCommand movel0({ pos[0], pos[1], pos[2], pos[3], pos[4], pos[5] });
		this->rokaeRobot->append({ movel0 }, ec); // 发送指令，机器人开始运动
	}
	else if (moveMode == 1) {
		rokae::MoveJCommand movel0({ pos[0], pos[1], pos[2], pos[3], pos[4], pos[5] }, 1000, 100);
		this->rokaeRobot->append({ movel0 }, ec); // 发送指令，机器人开始运动
	}
	else {
		array<double, 6> tcp;
		for (int i = 0; i < 6; i++) {
			tcp[i] = pos[i];
		}
		auto model = this->rokaeRobot->model();
		auto ik = model.calcIk(tcp, ec);
		//cout << ik[0] << "	" << ik[1] << "	" << ik[2] << "	" << ik[3] << "	" << ik[4] << "	" << ik[5] << endl;
		rokae::MoveAbsJCommand movel0({ ik[0], ik[1], ik[2], ik[3], ik[4], ik[5] }, 100, 100);
		this->rokaeRobot->append({ movel0 }, ec); // 发送指令，机器人开始运动
	}
	WaitRokaeRobot(rokaeRobot);
	//std::cout << "上一条错误信息: " << this->rokaeRobot->lastErrorCode().message() << std::endl;

	return;
}