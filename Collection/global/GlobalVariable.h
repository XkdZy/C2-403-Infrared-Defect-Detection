#pragma once

#include <iostream>

#include "Eigen/Dense"
#include "opencv2/opencv.hpp"
#include <Open3D/Open3D.h>

using namespace std;

#define COLLECT_STAGE 2 // 采集RGBD数据；0：采集RGBD；1：推理后画框，采集热像仪数据；2：热像仪上色
#define AGV_CONTROL 0 // AGV
#define ROBOT_ARM 0 // 机械臂
#define RGBD_CAMERA 0 // 是否使用深度相机
#define INTERACTION 0 // 是否交互
#define HDRGB_CAMERA 0 // 是否使用高清视觉
// 定时器定时时间
#define TIMER_TIMING_TIME 20 // 20ms
#define sign(x) ( ((x) <0 )? -1 : 1 )
// rgbd astra
#define HORIZON_ANGLE (double)(63.1 / 2)
#define VERTICAL_ANGLE (double)(49.4 / 2)
#define EDGE_ANGLE (double)atan(sqrt(tan(HORIZON_ANGLE * 3.1415926 / 180) * tan(HORIZON_ANGLE * 3.1415926 / 180) + tan(VERTICAL_ANGLE * 3.1415926 / 180) * tan(VERTICAL_ANGLE * 3.1415926 / 180)))
#define IMAGE_WIDTH (int)640
#define IMAGE_HEIGHT (int)480
// hd mind vision
#define HORIZON_ANGLE_HD (double)(71.2 / 2) // 4/3''
#define VERTICAL_ANGLE_HD (double)(56.3 / 2)
#define EDGE_ANGLE_HD (double)(83.4 / 2)
#define IMAGE_WIDTH_HD (int)4096
#define IMAGE_HEIGHT_HD (int)3000

struct AgvMoveInfo {
	/*	AGV是否到达目标
	* 0：未到达;
	* 1：AGV激光被阻塞；
	* 2：RGBD检测到阻塞;
	* 3：RGBD实时检测子线程退出
	* 4：
	*/
	int _move2goal = 0;
	Eigen::Vector3d _worldGoalPos = Eigen::Vector3d{ 0., 0., 0. }; // 世界坐标xyz
	Eigen::Vector3d _currAgvPos = Eigen::Vector3d{ 0., 0., 0. }; // 当前AGV坐标朝向xyθ（θ弧度）
	Eigen::Vector3d _oriAgvPos = Eigen::Vector3d{ 0., 0., 0. }; // 初始AGV坐标朝向xyθ（θ弧度）
	Eigen::Vector3d _oriArmPos = Eigen::Vector3d{ 0., 0., 0. }; // 初始机械臂末端初始坐标
	Eigen::Vector3d IMU_data = Eigen::Vector3d{ 0., 0., 0. };

};

// 实时AGV位姿
extern AgvMoveInfo agi;
extern std::shared_ptr<open3d::geometry::PointCloud> pcl_ptr; // 全局点云
extern std::shared_ptr<open3d::geometry::VoxelGrid> pcl_voxel; // 全局点云体素后信息

extern cv::Mat_<double> ThermalMatrix;
extern cv::Mat_<double> H_Thermal2Gripper;
extern cv::Mat_<double> CameraMatrix;
extern cv::Mat_<double> H_Camera2Gripper;
extern cv::Mat_<double> CameraMatrix_HD;
extern cv::Mat_<double> H_Camera2Gripper_HD;