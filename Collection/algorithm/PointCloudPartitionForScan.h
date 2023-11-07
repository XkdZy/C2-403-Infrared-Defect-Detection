#pragma once

#include <iostream>
#include <string>
#include <vector>
// robot arm
#include "BaseRobotArm.h"
// opencv
#include <opencv2/opencv.hpp>
//open3d
#include <Open3D/Open3D.h>
using namespace open3d;
using namespace std;

// engin
#include "Eigen/Dense"

struct PointCloudInfo {
	// ���Ʋο�������
	Eigen::Vector3d _refRx;
	Eigen::Vector3d _refRy;
	// ������С�����߽�
	Eigen::Vector3d _minBound;
	Eigen::Vector3d _maxBound;
	// ���Ƴ�������
	Eigen::Vector3d _extent;
};

// ����һ�κ���ɨ������Ҫ����Ϣ
struct ThermalScanInfo {
	// �����Ϣ
	Eigen::Vector3d _cameraPosition; // 3dλ��
	Eigen::Vector3d _cameraRx; // 3d Rx
	Eigen::Vector3d _cameraRy; // 3d Ry
	Eigen::Vector3d _cameraRz; // 3d Rz
	cv::Mat _cameraPosetrue;  // 3*3 �����̬
	// �����Ϣ��Ӧ��ĩ����Ϣ
	Eigen::Vector3d _gripperPosition; // 3d ĩ��λ��
	Eigen::Vector6d _gripperPoseVec; // 6d ĩ��λ��
	cv::Mat _gripperPosetrue; // 3*3ĩ����̬
	cv::Mat _gripperPoseMat; // 4*4ĩ��λ��
	string _cmdStr; // ĩ��6d��������
};

// �ֳ�ʽ�⼤�������ȳ����豸�ߴ�
//#define DEVICE_LENGTH (double)299. // ����29.9cm
//#define DEVICE_WIDTH (double)200. // ��26.5cm
#define DEVICE_LENGTH (double)299. // ����29.9cm
#define DEVICE_WIDTH (double)265. // ��26.5cm
#define PLAIN2CAMERA (double)154. // ����ͷ������ƽ�棺15.4cm
#define REMAIN_PLAIN2CAMERA (double)20. // ����ͷ������ƽ�棺15.4cm
#define INTERSECTION_COFF (double)1.2 // �ߴ糤���ϵ����1.2


void InitPointCloudInfo(const std::shared_ptr<open3d::geometry::PointCloud>& pcl, PointCloudInfo& pcli);
void CalcPointCloudXYOrient(Eigen::Vector3d xOrientRec, Eigen::Vector3d yOrientRec, Eigen::Vector3d zOrient,
	Eigen::Vector3d& xOrient, Eigen::Vector3d& yOrient);
vector<std::shared_ptr<open3d::geometry::PointCloud>> PointCloudPartition(const std::shared_ptr<open3d::geometry::PointCloud>& pcl);

vector<std::shared_ptr<open3d::geometry::LineSet>> PointCloudPartitionPCA(
	const PointCloudInfo& pcli,
	vector<std::shared_ptr<open3d::geometry::PointCloud>>& vCropedCloudForPCA,
	vector<ThermalScanInfo>& _scanInfo
);
void IntergateScanInfo(ThermalScanInfo& tsi);
void CalcPointCloudNormPCA(std::shared_ptr<open3d::geometry::PointCloud>& pointCloud, Eigen::Vector3d& norm, Eigen::Vector3d& center);
cv::Mat RotateRzMat(double rotateTheta);
cv::Mat RotateRyMat(double rotateTheta);
cv::Mat RotateRxMat(double rotateTheta);