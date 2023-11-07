#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <open3d/Open3D.h>

#include "PointCloudPartitionForScan.h"
#include "OperatePointCloud.h"
// astra
#include "astra/AstraCamera.h"
#include "astra/AstraCameraD2C.h"
#include "astra/d2cSwapper.h"
#include "astra/FrameListener.h"

using namespace std;

void DisplayRealTimeRGBD();
cv::Mat Convert2PseudoColor(const cv::Mat& depthImage);
void InteractObtainGoalInfo(Eigen::Vector3d& pos, double& length, double& width);
bool SaveRGBDInfo(const string& root, int imgIdx, cameraImgInfo& cii);
void ObtainPartitionFromPointCloud(const shared_ptr<open3d::geometry::PointCloud>& ptr, vector<ThermalScanInfo>& vScanInfo, bool display = true);
void CalcThermalPoseAndScan(const shared_ptr<open3d::geometry::PointCloud>& ptr, const vector<ThermalScanInfo>& vScanInfo, const string& root);
std::shared_ptr<open3d::geometry::PointCloud> RemoveButtonPoints(const std::shared_ptr<open3d::geometry::PointCloud>& pcl);

void Stage1Process(const Eigen::Vector3d& armGoal);
void Stage2Process();
void Stage3Process();