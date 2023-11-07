#pragma once

#include <iostream>
#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

using namespace std;

void RobotArmReset(); // 初始化机械臂末端rz指向x方向
void RobotArm2Goal(const Eigen::Vector3d& goal, int cameraType = 0); // 调整相机位姿指向目标位姿
void RobotArm2GoalUpdate(const Eigen::Vector3d& agvGoal, int cameraType = 0);
void RobotArm2Goal(cv::Mat gripperPoseMat);
/// <summary>
/// 机械臂坐标点经过AGV旋转、平移后坐标
/// </summary>
/// <param name="agvMoveInfo">0：x方向移动（当前相对初始）；1：y方向移动（当前相对初始）；2：旋转角（当前相对初始）</param>
/// <param name="robotPos">curr2ori为ture时，为当前AGV位姿下机械臂坐标点，false为初始AGV位姿下机械臂坐标点</param>
/// <param name="curr2ori">true：机械臂当前点到初始点</param>
/// <returns>旋转、平移后机械臂坐标</returns>
Eigen::Vector3d AGVMove2ArmPos(const Eigen::Vector3d& agvMoveInfo, Eigen::Vector3d robotPos, const bool curr2ori, const double oriTheta); // 机械臂坐标点经过旋转、平移后对应初始AGV状态下的坐标
Eigen::Vector3d AGVMove2ArmPos(const Eigen::Vector3d& agvMoveInfo, Eigen::Vector3d robotPos, const bool curr2ori, const double oriTheta, Eigen::Vector3d IMU);
void ConvertAGVMove2ArmTra(const vector<Eigen::Vector3d>& agvPos, vector<Eigen::Vector3d>& armPos); // 将AGV运动轨迹转化为机械臂运动轨迹
void ConvertOriMovePos2Curr(const vector<Eigen::Vector3d>& oriPos, vector<Eigen::Vector3d>& currPos); // 将AGV初始位置机械臂运动轨迹转化到当前位置下

cv::Mat arm2agv(const double x, const double y, const double theta);
cv::Mat arm2agv(const Eigen::Vector3d agv);
Eigen::Vector3d arm2agv(const Eigen::Vector3d& agvInfo, const Eigen::Vector3d& armPoint);
cv::Mat arm2agv(const Eigen::Vector3d& agvInfo, const cv::Mat& armPoint);
vector<Eigen::Vector3d> CalcAgvPosForRGBD(const Eigen::Vector3d& g, double radius, double safedis, double theta, bool display = true);
