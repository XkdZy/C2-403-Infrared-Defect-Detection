#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include "Eigen/Dense"
#include "opencv2/opencv.hpp"
#include "BaseRobotArm.h"
using namespace std;

// 通用方法
// 将机械臂CMD转换为6DMat
void AnalysisString26D(const std::string& str, std::vector<double>& v);
void AnalysisString26D(const std::string& str, cv::Mat& pose);
// 将6D坐标转化为机械臂CMD
void Convert6D2String(const std::vector<double>& v, std::string& str);
// 将4*4RT转化为机械臂CMD
void ConvertMat2String(const cv::Mat& mat, std::string& str);
//// 将6DMat转化为4*4Mat
//void Convert6DMat2Mat(const cv::Mat& src, cv::Mat& dst);
// 判断机械臂是否到达目标点
bool WhetherRobotArmMoveToPoint(const std::string dstPoint);
// 旋转矩阵转欧拉角
Eigen::Vector3d RotationMatrixToEulerAngles(Eigen::Matrix3d& R);
Eigen::Vector3d RotationMatrixToEulerAngles(cv::Mat& R);
// 读取运动点坐标
int ReadPointFile(std::fstream* fHandle, char* ch, int length);