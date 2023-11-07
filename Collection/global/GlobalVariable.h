#pragma once

#include <iostream>

#include "Eigen/Dense"
#include "opencv2/opencv.hpp"
#include <Open3D/Open3D.h>

using namespace std;

#define COLLECT_STAGE 2 // �ɼ�RGBD���ݣ�0���ɼ�RGBD��1������󻭿򣬲ɼ����������ݣ�2����������ɫ
#define AGV_CONTROL 0 // AGV
#define ROBOT_ARM 0 // ��е��
#define RGBD_CAMERA 0 // �Ƿ�ʹ��������
#define INTERACTION 0 // �Ƿ񽻻�
#define HDRGB_CAMERA 0 // �Ƿ�ʹ�ø����Ӿ�
// ��ʱ����ʱʱ��
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
	/*	AGV�Ƿ񵽴�Ŀ��
	* 0��δ����;
	* 1��AGV���ⱻ������
	* 2��RGBD��⵽����;
	* 3��RGBDʵʱ������߳��˳�
	* 4��
	*/
	int _move2goal = 0;
	Eigen::Vector3d _worldGoalPos = Eigen::Vector3d{ 0., 0., 0. }; // ��������xyz
	Eigen::Vector3d _currAgvPos = Eigen::Vector3d{ 0., 0., 0. }; // ��ǰAGV���곯��xy�ȣ��Ȼ��ȣ�
	Eigen::Vector3d _oriAgvPos = Eigen::Vector3d{ 0., 0., 0. }; // ��ʼAGV���곯��xy�ȣ��Ȼ��ȣ�
	Eigen::Vector3d _oriArmPos = Eigen::Vector3d{ 0., 0., 0. }; // ��ʼ��е��ĩ�˳�ʼ����
	Eigen::Vector3d IMU_data = Eigen::Vector3d{ 0., 0., 0. };

};

// ʵʱAGVλ��
extern AgvMoveInfo agi;
extern std::shared_ptr<open3d::geometry::PointCloud> pcl_ptr; // ȫ�ֵ���
extern std::shared_ptr<open3d::geometry::VoxelGrid> pcl_voxel; // ȫ�ֵ������غ���Ϣ

extern cv::Mat_<double> ThermalMatrix;
extern cv::Mat_<double> H_Thermal2Gripper;
extern cv::Mat_<double> CameraMatrix;
extern cv::Mat_<double> H_Camera2Gripper;
extern cv::Mat_<double> CameraMatrix_HD;
extern cv::Mat_<double> H_Camera2Gripper_HD;