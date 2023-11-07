#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

using namespace std;

void RobotArmReset(); // ��ʼ����е��ĩ��rzָ��x����
void RobotArm2Goal(const Eigen::Vector3d& goal, int cameraType = 0); // �������λ��ָ��Ŀ��λ��
void RobotArm2GoalUpdate(const Eigen::Vector3d& agvGoal, int cameraType = 0);
void RobotArm2Goal(cv::Mat gripperPoseMat);
/// <summary>
/// ��е������㾭��AGV��ת��ƽ�ƺ�����
/// </summary>
/// <param name="agvMoveInfo">0��x�����ƶ�����ǰ��Գ�ʼ����1��y�����ƶ�����ǰ��Գ�ʼ����2����ת�ǣ���ǰ��Գ�ʼ��</param>
/// <param name="robotPos">curr2oriΪtureʱ��Ϊ��ǰAGVλ���»�е������㣬falseΪ��ʼAGVλ���»�е�������</param>
/// <param name="curr2ori">true����е�۵�ǰ�㵽��ʼ��</param>
/// <returns>��ת��ƽ�ƺ��е������</returns>
Eigen::Vector3d AGVMove2ArmPos(const Eigen::Vector3d& agvMoveInfo, Eigen::Vector3d robotPos, const bool curr2ori, const double oriTheta); // ��е������㾭����ת��ƽ�ƺ��Ӧ��ʼAGV״̬�µ�����
Eigen::Vector3d AGVMove2ArmPos(const Eigen::Vector3d& agvMoveInfo, Eigen::Vector3d robotPos, const bool curr2ori, const double oriTheta, Eigen::Vector3d IMU);
void ConvertAGVMove2ArmTra(const vector<Eigen::Vector3d>& agvPos, vector<Eigen::Vector3d>& armPos); // ��AGV�˶��켣ת��Ϊ��е���˶��켣
void ConvertOriMovePos2Curr(const vector<Eigen::Vector3d>& oriPos, vector<Eigen::Vector3d>& currPos); // ��AGV��ʼλ�û�е���˶��켣ת������ǰλ����

cv::Mat arm2agv(const double x, const double y, const double theta);
cv::Mat arm2agv(const Eigen::Vector3d agv);
Eigen::Vector3d arm2agv(const Eigen::Vector3d& agvInfo, const Eigen::Vector3d& armPoint);
cv::Mat arm2agv(const Eigen::Vector3d& agvInfo, const cv::Mat& armPoint);
vector<Eigen::Vector3d> CalcAgvPosForRGBD(const Eigen::Vector3d& g, double radius, double safedis, double theta, bool display = true);
