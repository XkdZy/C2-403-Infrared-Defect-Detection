#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include "Eigen/Dense"
#include "opencv2/opencv.hpp"
#include "BaseRobotArm.h"
using namespace std;

// ͨ�÷���
// ����е��CMDת��Ϊ6DMat
void AnalysisString26D(const std::string& str, std::vector<double>& v);
void AnalysisString26D(const std::string& str, cv::Mat& pose);
// ��6D����ת��Ϊ��е��CMD
void Convert6D2String(const std::vector<double>& v, std::string& str);
// ��4*4RTת��Ϊ��е��CMD
void ConvertMat2String(const cv::Mat& mat, std::string& str);
//// ��6DMatת��Ϊ4*4Mat
//void Convert6DMat2Mat(const cv::Mat& src, cv::Mat& dst);
// �жϻ�е���Ƿ񵽴�Ŀ���
bool WhetherRobotArmMoveToPoint(const std::string dstPoint);
// ��ת����תŷ����
Eigen::Vector3d RotationMatrixToEulerAngles(Eigen::Matrix3d& R);
Eigen::Vector3d RotationMatrixToEulerAngles(cv::Mat& R);
// ��ȡ�˶�������
int ReadPointFile(std::fstream* fHandle, char* ch, int length);