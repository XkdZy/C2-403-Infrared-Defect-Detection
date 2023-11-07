#include "YaskawaRobotArm.h"
#include "CoordinateTransformation.h"

string QUERY = "query\r\n";

void YaskawaRobotArm::InitRobot() {
    this->_type = ROBOT_TYPE_INFO::Yaskawa;
    this->sp = SerialPort();
    while (1) {
        char portChar[10] = "COM";
        std::cout << "�������е��ͨ�Ŵ��ڣ�";
        int portNum;
        std::cin >> portNum;
        sprintf_s(portChar, "COM%d", portNum);
        bool retsp = sp.Serial_open(portChar, 115200);
        if (!retsp) {
            std::cout << "���ڳ�ʼ�����󡣡���" << std::endl;
        }
        else {
            break;
        }
    }
}


bool AnaysisPosData(string& PosData, vector<double>& vPos) {
	//// �����Ƿ����	len("A+1234+1234+1234+123+123+123BC") = 30
	//if (PosData.length() != 30)
	//{
	//	return false;
	//}
	//// ֡ͷ��֡β����
	//if ((PosData[0] == 'A') && (PosData[28] == 'B') && (PosData[29] == 'C'))
	//{
	//	return false;
	//}
	// ��������
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

	// ���Ͳ�ѯָ��
	sp.Serial_write_string(QUERY, QUERY.size());
	std::cout << "��ѯ�ѷ��͡�����" << endl;
	//Sleep(100);
	Sleep(2);
	// ��ȡ����
	while (1)
	{
		if (queryCnt >= 1000) { // ��ֹ��һ�ν��ղ������ݣ�û1000��û���յ����ݾ��ط���ѯquery\r\nָ��
			sp.Serial_write_string(QUERY, QUERY.size()); // QUERY.size()����size_of(QUERY)
			std::cout << "��ѯ�ѷ��͡�����" << endl;
			queryCnt = 0;
			Sleep(2);
		}
		std::cout << "��ѯ�С�����" << endl;
		// ��ȡ����
		int len = sp.Serial_read_string(readBuff, 100);
		if (len) {
			// ��ȡ������
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

	// ���Ͳ�ѯָ��
	sp.Serial_write_string(QUERY, QUERY.size());
	//cout << "��ѯ�ѷ��͡�����" << endl;
	//Sleep(100);
	Sleep(2);
	// ��ȡ����
	while (1) {
		if (queryCnt >= 1000) {
			// ��ֹ��һ�ν��ղ������ݣ�û1000��û���յ����ݾ��ط���ѯquery\r\nָ��
			sp.Serial_write_string(QUERY, QUERY.size()); // QUERY.size()����size_of(QUERY)
			//cout << "��ѯ�ѷ��͡�����" << endl;
			queryCnt = 0;
			//Sleep(100);
			Sleep(2);
		}
		//cout << "��ѯ�С�����" << endl;
		// ��ȡ����
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