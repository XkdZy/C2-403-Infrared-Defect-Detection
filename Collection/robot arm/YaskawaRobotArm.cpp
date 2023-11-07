#include "YaskawaRobotArm.h"
#include "CoordinateTransformation.h"

string QUERY = "query\r\n";

void YaskawaRobotArm::InitRobot() {
    this->_type = ROBOT_TYPE_INFO::Yaskawa;
    this->sp = SerialPort();
    while (1) {
        char portChar[10] = "COM";
        std::cout << "请输入机械臂通信串口：";
        int portNum;
        std::cin >> portNum;
        sprintf_s(portChar, "COM%d", portNum);
        bool retsp = sp.Serial_open(portChar, 115200);
        if (!retsp) {
            std::cout << "串口初始化错误。。。" << std::endl;
        }
        else {
            break;
        }
    }
}


bool AnaysisPosData(string& PosData, vector<double>& vPos) {
	//// 长度是否相符	len("A+1234+1234+1234+123+123+123BC") = 30
	//if (PosData.length() != 30)
	//{
	//	return false;
	//}
	//// 帧头、帧尾不符
	//if ((PosData[0] == 'A') && (PosData[28] == 'B') && (PosData[29] == 'C'))
	//{
	//	return false;
	//}
	// 解析数据
	float temp;
	temp = (PosData[2] - '0') * 100 + (PosData[3] - '0') * 10 + (PosData[4] - '0') + (PosData[5] - '0') / 10.0; // x
	if (PosData[1] == '-') {
		temp = -1 * temp;
	}
	vPos.push_back(temp);
	temp = (PosData[7] - '0') * 100 + (PosData[8] - '0') * 10 + (PosData[9] - '0') + (PosData[10] - '0') / 10.0; // y
	if (PosData[6] == '-') {
		temp = -1 * temp;
	}
	vPos.push_back(temp);
	temp = (PosData[12] - '0') * 100 + (PosData[13] - '0') * 10 + (PosData[14] - '0') + (PosData[15] - '0') / 10.0; // z
	if (PosData[11] == '-') {
		temp = -1 * temp;
	}
	vPos.push_back(temp);
	temp = (PosData[17] - '0') * 100 + (PosData[18] - '0') * 10 + (PosData[19] - '0'); // rx
	if (PosData[16] == '-') {
		temp = -1 * temp;
	}
	vPos.push_back(temp);
	temp = (PosData[21] - '0') * 100 + (PosData[22] - '0') * 10 + (PosData[23] - '0'); // ry
	if (PosData[20] == '-') {
		temp = -1 * temp;
	}
	vPos.push_back(temp);
	temp = (PosData[25] - '0') * 100 + (PosData[26] - '0') * 10 + (PosData[27] - '0'); // rz
	if (PosData[24] == '-') {
		temp = -1 * temp;
	}
	vPos.push_back(temp);

	return true;
}


std::vector<double> YaskawaRobotArm::ReadRobotArmPos() {
	vector<double> vPos;
	int queryCnt = 0;
	string readBuff;

	// 发送查询指令
	sp.Serial_write_string(QUERY, QUERY.size());
	std::cout << "查询已发送。。。" << endl;
	//Sleep(100);
	Sleep(2);
	// 读取数据
	while (1)
	{
		if (queryCnt >= 1000) { // 防止发一次接收不到数据，没1000次没接收到数据就重发查询query\r\n指令
			sp.Serial_write_string(QUERY, QUERY.size()); // QUERY.size()不是size_of(QUERY)
			std::cout << "查询已发送。。。" << endl;
			queryCnt = 0;
			Sleep(2);
		}
		std::cout << "查询中。。。" << endl;
		// 读取数据
		int len = sp.Serial_read_string(readBuff, 100);
		if (len) {
			// 读取到数据
			bool ret = AnaysisPosData(readBuff, vPos);
			break;
		}
		queryCnt++;
	}
}

std::string YaskawaRobotArm::ReadRobotArmPosString() {
	int queryCnt = 0;
	string readBuff;
	string strPos;

	// 发送查询指令
	sp.Serial_write_string(QUERY, QUERY.size());
	//cout << "查询已发送。。。" << endl;
	//Sleep(100);
	Sleep(2);
	// 读取数据
	while (1) {
		if (queryCnt >= 1000) {
			// 防止发一次接收不到数据，没1000次没接收到数据就重发查询query\r\n指令
			sp.Serial_write_string(QUERY, QUERY.size()); // QUERY.size()不是size_of(QUERY)
			//cout << "查询已发送。。。" << endl;
			queryCnt = 0;
			//Sleep(100);
			Sleep(2);
		}
		//cout << "查询中。。。" << endl;
		// 读取数据
		int len = sp.Serial_read_string(readBuff, 100);
		if (len) {
			strPos = readBuff;
			break;
		}
		queryCnt++;
	}

	return strPos;
}

void YaskawaRobotArm::Move2OnePoint(std::string point, int moveMode) {
    point += "\r\n";
    this->sp.Serial_write_string(point, 32);
    Sleep(2);
    char spdChar[5];
    sprintf_s(spdChar, "%04d", 400);
    string spdStr = "DL" + string(spdChar) + "E\r\n";
    this->sp.Serial_write_string(spdStr, spdStr.size());
    Sleep(2);
    while (!WhetherRobotArmMoveToPoint(point));
    //move2CurrTarget = true;
}