#pragma once

#include "Seer.h"

#include <opencv2/opencv.hpp>
#include <Open3D/Open3D.h>
#include "Eigen/Dense"

#include "CalibrateEyeInHand.h"
#include "RoutePlaning.h"
#include "PointCloudPartitionForScan.h"

using namespace open3d;
using namespace std;




#define PI 3.14159265
#define X 84
#define Y 63

extern cv::Mat_<double> H_Camera2Gripper;
extern int FR[X][Y];
extern int FR1[X][Y];
extern int FR2[X][Y];

//int map_obstacles();
int map_obstacles(string path);
int* dijkstra(float x, float y, float xx, float yy);
int map_Laser_res(SOCKET& client);
void Move2Goal(const Eigen::Vector2d turegoal, const Eigen::Vector2d goal);
void SubThreadAdjustRobotArmPosture();
Eigen::MatrixXd cloud_ICP(std::shared_ptr<geometry::PointCloud>& source, std::shared_ptr<geometry::PointCloud>& target);
cv::Mat PointCloudICP(std::shared_ptr<geometry::PointCloud>& source, std::shared_ptr<geometry::PointCloud>& target);