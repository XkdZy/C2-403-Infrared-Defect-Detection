#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Eigen/Dense"
using namespace std;

typedef struct _SearchVoxelRegion {
	Eigen::Vector3i v; // ���ؿ�����xyz��С�����Ӧ����
	int width; // ��
	int length; // ��
	int height; // ��
}SearchVoxelRegion;

struct Voxel2PclInfo {
	Eigen::Vector3d minPoint; // xyz��Сֵ��Ӧ�Ķ���
	Eigen::Vector3d size; // �����
}; // ����ת�������Ƶı�Ҫ��Ϣ
extern Voxel2PclInfo v2pInfo;

// ·���滮
typedef struct _vertex {
	bool occupy;
	int num; // ������
	Eigen::Vector3i v; // ����ռ�λ��
	Eigen::Vector3d vPcl; // �������ؿ����Ķ�Ӧ������ʵ��λ��
	//vector<int> vVertexNum; // ���ӵĽڵ�
	int* vVertexNum; // ���ӵĽڵ���
}vertex;

extern vertex startVertex;
extern vertex goalVertex;
extern vertex* allVertexArr;
extern vertex* expandedVertexArr;

extern vector<int> vCurrentTrajectory;
extern vector<vertex> vCurrentTrajectoryVertex;
extern int CurrAGVMoveIdx; // ��ǰAGV�ƶ���λ����
extern vector<Eigen::Vector3d> vAGVMovePosition; // AGV�ƶ����꣨AGV����ϵ�£�
extern vector<Eigen::Vector3d> vArmMovePosition; // ��е���ƶ����꣨AGV��ʼλ��ʱ�Ļ�е������ϵ�£�

extern vector<int> vTest;

extern Eigen::Vector3d objectCenterPos; // �ֶ�λĿ����������

// �ֳ�ʽ�⼤�������ȳ����豸�ߴ�
#define DEVICE_LENGTH (double)299.0 // ����29.9cm
#define DEVICE_WIDTH (double)265.0 // ��26.5cm
#define DEVICE_HEIGHT (double)320.0 // ����ƽ�浽��Ļƽ��߶ȣ�32.0cm
//#define DEVICE_LENGTH (double)164.4 // ����29.9cm
//#define DEVICE_WIDTH (double)72.4 // ��26.5cm
//#define DEVICE_HEIGHT (double)164.4 // ����ƽ�浽��Ļƽ��߶ȣ�32.0cm
//#define DEVICE_LENGTH (double)1. // ����29.9cm
//#define DEVICE_WIDTH (double)1. // ��26.5cm
//#define DEVICE_HEIGHT (double)1. // ����ƽ�浽��Ļƽ��߶ȣ�32.0cm
#define DEVICE_PLAIN2CAM (double)154.0 // ����ͷ������ƽ�棺15.4cm
#define DEVICE_REMAINPLAIN2CAM (double)20.0 // Ԥ������ͷ���浽����
#define DEVICE_HALFHYPOTENUSE (double)(0.5 * sqrt(DEVICE_LENGTH * DEVICE_LENGTH + DEVICE_WIDTH * DEVICE_WIDTH + DEVICE_HEIGHT * DEVICE_HEIGHT)) // �������豸��б�߳�
#define DEVICE_OCCUPYVOXEL (int)ceil(DEVICE_HALFHYPOTENUSE / VOXEL_SIZE)

#define VOXEL_SIZE (double)20. // ÿ�����ؿ��СVOXEL_SIZE*VOXEL_SIZE*VOXEL_SIZE


/// <summary>
/// ����Ŀ����ȷ����Ŀ��λ����������Ĺ۲�λ��
/// </summary>
/// <param name="objPos"></param>
/// <param name="camPos"></param>
void FromObjPos2CamObservePos(const Eigen::Vector3d& camPos, const Eigen::Vector3d& objPos, Eigen::Vector3i camDisPos);
/// <summary>
/// ����ʼ�㡢Ŀ����Լ��豸�ߴ�ȷ�����������ط�Χ
/// </summary>
/// <param name="start">��ʼ��</param>
/// <param name="goal">Ŀ���</param>
/// <param name="svr">���������ط�Χ</param>
void GetSearchRegion(const Eigen::Vector3i& start, const Eigen::Vector3i& goal, SearchVoxelRegion& svr);
/// <summary>
/// ��������������ؿ���б��
/// </summary>
/// <param name="svr">���������ط�Χ</param>
/// <param name="start">��ʼ��</param>
/// <param name="goal">Ŀ���</param>
/// <param name="allVertex">���б�úŵ����ض���</param>
/// <param name="startVertex">��ʼ����</param>
/// <param name="goalVertex">Ŀ�궥��</param>
void VertexNumFromSearchRegion(const SearchVoxelRegion& svr, const Eigen::Vector3i& start, const Eigen::Vector3i& goal, vertex& startVertex, vertex& goalVertex);
/// <summary>
/// �����������ڵ����ذ��豸�ߴ��������
/// </summary>
/// <param name="svr">���������ط�Χ</param>
/// <param name="allVertex">���ж���</param>
/// <param name="expandedVertex">���ͺ����ж���</param>
void ExpandBarrierRegion(SearchVoxelRegion& svr, vertex* expandedVertex);
/// <summary>
/// �������л�����Ȩͼ����BFS����ȷ�����·��
/// </summary>
/// <param name="len">�������</param>
/// <param name="start">��ʼ��</param>
/// <param name="goal">Ŀ���</param>
/// <param name="bfsLine">����·��</param>
bool BreadthFirst(const int len, const vertex& start, const vertex& goal, vector<int>& bfsLine);
/// <summary>
/// ���ݵ�ǰ����pclRealTime_ptr���¹滮һ��·��vCurrentTrajectoryVertex
/// </summary>
/// <param name="start">��ǰλ��</param>
/// <param name="end">Ŀ��λ��</param>
void UpDateRouteBaseCurrentPointCloud(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);
/// <summary>
/// �ж�֮ǰ·��route�Ƿ����㵱ǰ����pclRealTime_ptr
/// </summary>
/// <param name="bfsLine">֮ǰ·��</param>
/// <returns>���㣺true</returns>
bool VertifyRouteWhetherMeetRequire(const int startIndex);

/// ����ת��
// ��������AB = OB - OA
Eigen::Vector3d CalcVectorAB(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

void ConvertString2Mat44(const string& str, const bool gripper2cam, cv::Mat& m44);
// �������rzָ�򣨶�⣬ѡ��һ����ȷ�������̬
cv::Mat CalcRxRyBaseRz(const Eigen::Vector3d& rz, Eigen::Vector3d& rx, Eigen::Vector3d& ry);
cv::Mat CalcRxRyBaseRz(const Eigen::Vector3d& rz, const Eigen::Vector3d& rx, const Eigen::Vector3d& ry, Eigen::Vector3d& rxAdjusted, Eigen::Vector3d& ryAdjusted);
// ����Ŀ��λ�õ��������̬
void AdjustCameraPosture(const string& nowGripperPose, const Eigen::Vector3d& objPos, string& adjustedGripperCmd, cv::Mat& adjustedR);
void AdjustCameraPosture(const Eigen::Vector3d& nowPose, const Eigen::Vector3d& objPos, cv::Mat& adjustedR);
void UpdateRealTimePointCloud();

