#include "CoordinateTransformation.h"
#include "BaseRobotArm.h"
#include "GlobalVariable.h"

/** 
 将机械臂6D字符串坐标转换为6D数值信息
 @para robotArmInfo:机器人类型
 @para str:机器人坐标点
 @para v:转换为6D后的坐标结果
*/
void AnalysisString26D(const string& str, vector<double>& v) {
	v = vector<double>(6, 0.0);
	if (ROBOT_TYPE_INFO::Yaskawa == robotArmHandle->GetRobotTypeInfo()) {
		// 1+5+5+5+4+4+4+2
		// A+0794-4467+3576+179-027+179BC ---> 79.4 -446.7 357.6 179 -027 179
		for (int i = 0; i < 3; i++) {
			string temp = str.substr(1 + i * 5, 5);
			int val = atoi(temp.c_str());
			v[i] = val / 10.;
			//cout << "当前字串为：" << temp << "转换后数值为：" << v[i] << endl;
		}
		for (int i = 3; i < 6; i++) {
			string temp = str.substr(16 + (i - 3) * 4, 4);
			int val = atoi(temp.c_str());
			v[i] = val;
			//cout << "当前字串为：" << temp << "转换后数值为：" << v[i] << endl;
		}
	}
	else if(ROBOT_TYPE_INFO::Rokae == robotArmHandle->GetRobotTypeInfo()) {
		// 1+7+7+7+6+6+6+2=42
		// A+079400-044670+035760+17900-02700+17900BC ---> 79.4 -446.7 357.6 179 -027 179
		//cout << str << endl;
		for (int i = 0; i < 3; i++) {
			string temp = str.substr(1 + i * 7, 7);
			int val = atoi(temp.c_str());
			v[i] = val / 100.;
			//cout << "当前字串为：" << temp << "转换后数值为：" << v[i] << endl;
		}
		for (int i = 3; i < 6; i++) {
			string temp = str.substr(22 + (i - 3) * 6, 6);
			int val = atoi(temp.c_str());
			v[i] = val / 100.;
			//cout << "当前字串为：" << temp << "转换后数值为：" << v[i] << endl;
		}
	}
}
void AnalysisString26D(const string& str, cv::Mat& pose) {
	vector<double> v;
	AnalysisString26D(str, v);
	pose = cv::Mat(1, 6, CV_64F, 0.0);
	pose.at<double>(0, 0) = v[0];
	pose.at<double>(0, 1) = v[1];
	pose.at<double>(0, 2) = v[2];
	pose.at<double>(0, 3) = v[3];
	pose.at<double>(0, 4) = v[4];
	pose.at<double>(0, 5) = v[5];
}
void ConvertMat2String(const cv::Mat& mat, string& str) {
	vector<double> v(6, 0.0);
	// 位置
	v[0] = mat.at<double>(0, 3);
	v[1] = mat.at<double>(1, 3);
	v[2] = mat.at<double>(2, 3);
	// 姿态
	cv::Mat rotate = mat({ 0,0,3,3 });
	Eigen::Vector3d euler = RotationMatrixToEulerAngles(rotate);
	v[3] = euler[0];
	v[4] = euler[1];
	v[5] = euler[2];
	Convert6D2String(v, str);

	return;
}
void Convert6D2String(const vector<double>& v, string& str) {
	vector<int> vI;
	string tempStr = "A";
	if (ROBOT_TYPE_INFO::Yaskawa == robotArmHandle->GetRobotTypeInfo()) {
		for (int i = 0; i < v.size(); i++) {
			if (i < 3) {
				// 前三位乘10
				vI.push_back(int(v[i] * 10)); // 乘10转换为整数保存
			}
			else {
				vI.push_back(int(v[i])); // 乘10转换为整数保存
			}
		}
		char printChar[32];
		sprintf_s(printChar, "A");
		for (int i = 0; i < vI.size(); i++) {
			if (vI[i] > 0) {
				// 大于0加+号
				tempStr += "+";
				if (i < 3) {
					sprintf_s(printChar, "%04d", vI[i]);
					tempStr += string(printChar);
				}
				else {
					sprintf_s(printChar, "%03d", vI[i]);
					tempStr += string(printChar);
				}
			}
			else {
				tempStr += "-";
				if (i < 3) {
					sprintf_s(printChar, "%04d", -vI[i]);
					tempStr += string(printChar);
				}
				else {
					sprintf_s(printChar, "%03d", -vI[i]);
					tempStr += string(printChar);
				}
			}
		}
	}
	else if (ROBOT_TYPE_INFO::Rokae == robotArmHandle->GetRobotTypeInfo()) {
		for (int i = 0; i < v.size(); i++) {
			vI.push_back(int(v[i] * 100)); // 乘10转换为整数保存
		}
		char printChar[44];
		sprintf_s(printChar, "A");
		for (int i = 0; i < vI.size(); i++) {
			if (vI[i] > 0) {
				// 大于0加+号
				tempStr += "+";
				if (i < 3) {
					sprintf_s(printChar, "%06d", vI[i]);
					tempStr += string(printChar);
				}
				else {
					sprintf_s(printChar, "%05d", vI[i]);
					tempStr += string(printChar);
				}
			}
			else {
				tempStr += "-";
				if (i < 3) {
					sprintf_s(printChar, "%06d", -vI[i]);
					tempStr += string(printChar);
				}
				else {
					sprintf_s(printChar, "%05d", -vI[i]);
					tempStr += string(printChar);
				}
			}
		}
	}
	tempStr += "BC";
	str = tempStr;
}

bool WhetherRobotArmMoveToPoint(const string dstPoint) {
	if (ROBOT_TYPE_INFO::Yaskawa == robotArmHandle->GetRobotTypeInfo()) {
		vector<double> vDstPoint;
		AnalysisString26D(dstPoint, vDstPoint); // 将6D字符串转化为6D坐标
		string str = robotArmHandle->ReadRobotArmPosString(); // 全部字符串
		if (str.length() < 30) return false;
		//cout << "当前机器人位姿为：" << str << endl;
		// 保存实时坐标
		//if (USE_MULTITHREAD) {
		//	// 多线程是，写入坐标时上锁，防止其他线程读取
		//	mutWritePos.lock();
		//	sRealTimePos = str;
		//	mutWritePos.unlock();
		//	Sleep(2);
		//}
		int cIndex = str.find('C'); // 截取A...BC的30个字符
		if (cIndex < 29) return false;
		str = str.substr(cIndex - 29, 30);
		vector<double> vnowPoint;
		//cout << "-------------" << str << endl;
		AnalysisString26D(str, vnowPoint); // 将6D字符串转化为6D坐标
		double diff = sqrt(
			pow(vDstPoint[0] - vnowPoint[0], 2) + pow(vDstPoint[1] - vnowPoint[1], 2) +
			pow(vDstPoint[2] - vnowPoint[2], 2) + pow(vDstPoint[3] - vnowPoint[3], 2) +
			pow(vDstPoint[4] - vnowPoint[4], 2) + pow(vDstPoint[5] - vnowPoint[5], 2)
		); // 计算距离
		//cout << "nowdiff:" << diff << endl;
		return (diff < 0.01) ? true : false; // 距离小于阈值返回真
	}
	else if (ROBOT_TYPE_INFO::Rokae == robotArmHandle->GetRobotTypeInfo()) {
		vector<double> vDstPoint;
		AnalysisString26D(dstPoint, vDstPoint); // 将6D字符串转化为6D坐标
		string str = robotArmHandle->ReadRobotArmPosString(); // 全部字符串
		//cout << str << endl;
		if (str == "" || str.length() < 42) {
			return false;
		}
		//// 保存实时坐标
		//if (USE_MULTITHREAD) {
		//	// 多线程是，写入坐标时上锁，防止其他线程读取
		//	mutWritePos.lock();
		//	sRealTimePos = str;
		//	mutWritePos.unlock();
		//	Sleep(1);
		//}
		int cIndex = str.find('C'); // 截取A...BC的42个字符
		str = str.substr(cIndex - 41, 42);
		vector<double> vnowPoint;
		//cout << "-------------" << str << endl;
		AnalysisString26D(str, vnowPoint); // 将6D字符串转化为6D坐标
		double diff = sqrt(
			pow(vDstPoint[0] - vnowPoint[0], 2) + pow(vDstPoint[1] - vnowPoint[1], 2) +
			pow(vDstPoint[2] - vnowPoint[2], 2) + pow(vDstPoint[3] - vnowPoint[3], 2) +
			pow(vDstPoint[4] - vnowPoint[4], 2) + pow(vDstPoint[5] - vnowPoint[5], 2)
		); // 计算距离
		//cout << "nowdiff:" << diff << endl;
		return (diff < 2.) ? true : false; // 距离小于阈值返回真
	}
	else {
		return false;
	}
}

/**
 * 功能： 通过给定的旋转矩阵计算对应的欧拉角
 **/
Eigen::Vector3d RotationMatrixToEulerAngles(cv::Mat& R) {
	assert(isRotationMatrix(R));
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
	bool singular = sy < 1e-6;
	double x, y, z;
	if (!singular) {
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else {
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	x = x / CV_PI * 180.0;
	y = y / CV_PI * 180.0;
	z = z / CV_PI * 180.0;
	return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d RotationMatrixToEulerAngles(Eigen::Matrix3d& R) {
	Eigen::Vector3d n = R.col(0);
	Eigen::Vector3d o = R.col(1);
	Eigen::Vector3d a = R.col(2);

	Eigen::Vector3d ypr(3);
	double y = atan2(n(1), n(0));
	double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
	double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
	ypr(0) = y;
	ypr(1) = p;
	ypr(2) = r;

	return ypr / CV_PI * 180.0;
}
/*
* 读取运动点坐标
* 输入：
*       fHandle：文件操作句柄
*       ch：二维数组首地址，存放运动点坐标
*       length：二维数组的宽度
* 输出：
*       index：运动点的个数
* 示例：
*		fstream fHandle1("../test.txt", ios::out | ios::in);
*       char buff[20][100] = { 0 };
*       int ret = ReadPointFile(&fHandle1, (char*)buff, 100);
*		fHandle1.close();
*/
int ReadPointFile(fstream* fHandle, char* ch, int length) {
	int index = 0;
	char tempbuff[100];
	while (1) {
		fHandle->getline(tempbuff, 100);
		if (tempbuff[0] == '\0') {
			return index;
		}
		else { // 读取数据
			for (int i = 0; i < length; i++) {
				if (tempbuff[i] != '\0') {
					ch[i + index * length] = tempbuff[i];
				}
				else {
					break;
				}
			}
		}
		index++;
	}
}


