#pragma once

#include <iostream>
using namespace std;

#include <string>

#include <opencv2/opencv.hpp>
//open3d
#include <Open3D/Open3D.h>
using namespace open3d;
#include "OperateImage.h"

struct reconstructInfo {
	/*
		0x01��ת����е����������ϵ
		0x02��ת��AGV������������
		0x04���޷����������õ㣨set point��һ����Χ��dmm���ڵĵ㱣��
		0x08������Ĥ���п�ͼ
		0x10��thermal��ɫ
		0x20�����������ʱ����ȫ�ֵ������ػ�����бȽϣ��ж��Ƿ�ռ��
		0x40��RGBD�ں���ɫ
	*/
	int _2whichbase = 0x01;
	// _2whichbase == 0x01ʱ��Ч
	bool _agvRT = false; // false������תƽ�ƣ�ture����תƽ��
	// _agvRT=tureʱ��Ч
	Eigen::Vector3d _agvRTInfo = Eigen::Vector3d{ 0.,0.,0. }; // AGV������ת����
	double _agvOriDir = 0.; // ����diffʱ��AGV��ʼ����
	// _2whichbase == 0x02ʱ��Ч
	Eigen::Vector3d _agvCurrInfo = Eigen::Vector3d{ 0.,0.,0. }; // AGV��ǰλ��
	// _2whichbase == 0x04ʱ��Ч
	Eigen::Vector3d _setPoint = Eigen::Vector3d{ 0.,0.,0. }; // ���õ�
	double _dmm = 100000; // �����õ����С��<_dmm������Ĭ��100m	
	// _2whichbase == 0x08ʱ��Ч
	cv::Mat _mask = cv::Mat::zeros(480, 640, CV_8U); // ��Ĥ
	// _2whichbase == 0x10ʱ��Ч
	vector<cameraImgInfo> _thermals; // thermal��ɫͼƬcv::Mat::zeros(480, 640, CV_8UC3)
	// _2whichbase == 0x40 | 0x10ʱ��Ч
	vector<cameraImgInfo> _rgbds; // thermal��ɫͼƬcv::Mat::zeros(480, 640, CV_8UC3)
	int _rgbdIdx; // ��ǰ������Ӧagv��ʼλ��
	// _2whichbase == 0x20ʱ��Ч
	std::shared_ptr<open3d::geometry::VoxelGrid> _totalVoxel = std::make_shared<open3d::geometry::VoxelGrid>(); // ȫ�ֵ���դ��
	// _2whichbase == 0x80ʱ��Ч->z�޷�
	double _height = 0.;
};

struct PointCloudRotateInfo {
	bool _isNeedRotate = false; // �Ƿ���Ҫ��ת����
	//double _rotateTheta = 0.; // ��ת�Ƕ�
	Eigen::Vector3d _rotateXYZ = Eigen::Vector3d{ 0.,0.,0. }; // ��ת�Ƕȣ�˳ʱ��Ϊ��
	double _minRotatAngle = 2.; // ��С��ת�Ƕ�
	Eigen::Vector3d _rotateCenter = Eigen::Vector3d{ 0.,0.,0. }; // ��ת����
};

struct drawAxisInfo {
	double _xLen = 100.;
	double _yLen = 200.;
	double _zLen = 500.;
	Eigen::Vector3d _center = Eigen::Vector3d{ 0., 0., 0. };
};

Eigen::Vector3d GetColorFromImageVec(const Eigen::Vector3d& pAgv, const vector<cameraImgInfo>& vImgInfos, vector<cv::Mat>& vAmend, bool amend = false);
Eigen::Vector3d GetColorFromImageVec(const cv::Mat& p_base, const Eigen::Vector3d& refPos, const vector<cameraImgInfo>& vImg);
/// <summary>
/// �������굽��������ϵ������
/// </summary>
/// <param name="row">������</param>
/// <param name="col">������</param>
/// <param name="cii">ͼƬ��Ϣ</param>
/// <param name="depth">���</param>
/// <returns>��������ϵ����</returns>
Eigen::Vector3d ConvertPixel2World(int row, int col, const cameraImgInfo& cii, ushort depth = 0);
/// <summary>
/// ���ݵ���ͼƬ���ض�Ӧ�ع��ľֲ�����
/// </summary>
/// <param name="cii">ͼƬ�ṹ����Ϣ</param>
/// <param name="ri">�ع���Ҫ�Ķ�����Ϣ��ֱ�ӷ���ֱ�Ӷ���ṹ�弴��</param>
/// <returns></returns>
std::shared_ptr<open3d::geometry::PointCloud> ReconstructFromOneImg(const cameraImgInfo& cii, reconstructInfo& ri);
/// <summary>
/// ����С�ĵ��ƴӽϴ�ĵ�����ȥ��
/// </summary>
/// <param name="bigPC">�ϴ�ĵ���</param>
/// <param name="smallPC">��С�ĵ���</param>
/// <returns>ȥ����ĵ���</returns>
std::shared_ptr<open3d::geometry::PointCloud> RemoveSmallerPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& bigPC, const std::shared_ptr<open3d::geometry::PointCloud>& smallPC);

void CombinePartialPointCloud(std::vector<std::shared_ptr<open3d::geometry::Geometry>>& vPartialPC1,
							std::vector<std::shared_ptr<open3d::geometry::Geometry>>& vPartialPC2, 
							std::vector<std::shared_ptr<const open3d::geometry::Geometry>>& totalPC);
/// <summary>
/// ������ת����
/// </summary>
/// <param name="srcPointCloud">����ԭʼ����</param>
/// <param name="R">�������</param>
/// <param name="rotateCenter">��ת����</param>
/// <param name="dstPointCloud">�����ת��ĵ���</param>
std::shared_ptr<open3d::geometry::PointCloud> RotatePointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& srcPcl, const PointCloudRotateInfo pcri);
/// <summary>
/// ���ݵ�����Ӿ��εĳߴ��ʵ���豸��С������������
/// </summary>
/// <param name="srcPointCloud">�������</param>
/// <param name="rowScanTimes">�зָ����</param>
/// <param name="colScanTimes">�зָ����</param>
/// <param name="rowScanDis">�зָ���</param>
/// <param name="colScanDis">�зָ���</param>
/// <param name="vvBoxSegmentPos">�ָ������</param>
/// <param name="vCropedPointCloud">�ָ������е���</param>
void CropPointCloudThroughBoundingBox(std::shared_ptr<open3d::geometry::PointCloud>& srcPointCloud,
	int& rowScanTimes, int& colScanTimes, double& rowScanDis, double& colScanDis, vector<vector<Eigen::Vector3d>>& vvBoxSegmentPos,
	std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& vCropedPointCloud, int& lengthMoreWidth);
/// <summary>
/// ����OrientBoundingBox�������ϵı߽��
/// </summary>
/// <param name="srcPointCloud">���룺ԭʼ����</param>
/// <param name="vEightPoint">�����8������</param>
void GenerateBoundingBox(std::shared_ptr<open3d::geometry::PointCloud> srcPointCloud, vector<Eigen::Vector3d>& vEightPoint);
/// <summary>
/// ����OrientBoundingBox�����ɴ�z����xyƽ���2Dͼ��
/// </summary>
/// <param name="srcPointCloud">���룺ԭʼ����</param>
/// <param name="vBoundingBox">���룺8������</param>
/// <param name="img">�����ͶӰ�õ���ͼƬ</param>
/// <param name="validPixel">�������Ч���صĸ���</param>
/// <param name="outImgType">�����ͼƬ����0���ڰס�1���Ҷȡ�2��ɫ</param>
/// <param name="dir">���������0��1���£���ʾ��б����</param>
///0------  1------
/// |\       |    /
/// | \      |   /
/// |  \     |  /
/// |   \    | /
/// |    \   |/
/// --------  --------
void ProjectPointCloudFromZOrient(std::shared_ptr<open3d::geometry::PointCloud>& srcPointCloud, vector<Eigen::Vector3d>& vBoundingBox, cv::Mat& img, int& validPixel, uchar outImgType, bool& dir);
/// <summary>
/// ��2Dͼ�������x��y��ָ��ͨ����Ч���صĸ�������������������С��е�����ֵ��ȷ����б�н�
/// </summary>
/// <param name="src">���룺z����ͶӰ�Ķ�άͼƬ</param>
/// <param name="validCnt">���룺��Ч�����ظ���</param>
/// <param name="xOrient">�����x����ķ�������</param>
/// <param name="yOrient">�����y����ķ�������</param>
void FromImgGetXYOrient(cv::Mat& src, int validCnt, bool leanDir, Eigen::Vector3d& xOrient, Eigen::Vector3d& yOrient, int lengthMoreWidth);
/// <summary>
/// ������������ϵ��XYZ�ᣬ���ȵ���
/// </summary>
/// <param name="axis">�����ĵ���</param>
std::shared_ptr<open3d::geometry::LineSet> DrawXYZAxisAtOrient(const drawAxisInfo dai);
/// <summary>
/// �����豸����3d����
/// </summary>
/// <param name="device_cloud">�����ĵ���</param>
std::shared_ptr<open3d::geometry::LineSet> DrawDeviceLinePointCloud(Eigen::Vector3d position, Eigen::Vector3d xOrient, Eigen::Vector3d yOrient, Eigen::Vector3d zOrient);
/// <summary>
/// �������z�᷽��FOVȷ����ǰλ��������ӳ���Χ
/// </summary>
/// <param name="x">���z����</param>
/// <param name="y">���y����</param>
/// <param name="z">���x����</param>
/// <returns>�����б�4��������0/1����Ӧ���ߣ�2/3����Ӧ�̱ߣ����ĸ��������λ�ã�</returns>
vector<Eigen::Vector3d> GetAngleBounding(Eigen::Vector3d x, Eigen::Vector3d y, Eigen::Vector3d z, bool hd);
/// <summary>
/// ��ǰ�����Ƿ�������ӽǷ�Χ��
/// </summary>
/// <param name="vec">��ǰ�������Ƶ㹹������</param>
/// <param name="camZOrient">���z����</param>
/// <param name="maxAngle">���Ƕ�</param>
/// <returns>true:���ӳ���</returns>
bool VectorInRange(Eigen::Vector3d vec, Eigen::Vector3d camZOrient);

/// <summary>
/// ���ض������̬��3d����ͶӰ��2dͼ��
/// </summary>
/// <param name="pd">����</param>
/// <param name="pose">���λ��</param>
/// <returns>ͼ���б�����0��������ͼƬ������1���Ҷ�ͼƬ������2����ɫͼƬ������3����ʴ���ͺ�Ķ�����ͼƬ������4��α���ͼ</returns>
vector<cv::Mat> PerspectivePointCloudFromSpecialPose(const std::shared_ptr<open3d::geometry::PointCloud>& pd, const cv::Mat pose);
vector<cv::Mat> PerspectivePointCloudFromSpecialPoseHD(const std::shared_ptr<open3d::geometry::PointCloud>& pd, const cv::Mat pose);

controlRobotInfo ConvertMask2CameraPose(const cameraImgInfo& cami, const bboxImgInfo& bbi, int bboxIndex, int con2whichcam);
//void PerspectivePointCloudFromSpecialPose(const std::shared_ptr<open3d::geometry::PointCloud>& pd, const cv::Mat pose, vector<cv::Mat>& vMat);
/// <summary>
/// ��ͼ����Ĥ�Ͳο���Ĥ�Ľ���
/// </summary>
/// <param name="src">ͼ���б�����0��������ͼƬ������1���Ҷ�ͼƬ������2����ɫͼƬ������3����ʴ���ͺ�Ķ�����ͼƬ</param>
/// <param name="ref">�ο���Ĥ</param>
/// <param name="dst"></param>
void MaskIntersection(const vector<cv::Mat>& src, const vector<cv::Mat>& refImg, const vector<cv::Rect>& refRect, vector<cv::Mat>& dst);
void MaskIntersection(const cv::Mat& src, const vector<cv::Mat>& refImg, const vector<cv::Rect>& refRect, cv::Mat& dst);
/// <summary>
/// ������Ĥ�е��쳣��
/// </summary>
/// <param name="src">ͼ���б�����0��������ͼƬ������1���Ҷ�ͼƬ������2����ɫͼƬ������3����ʴ���ͺ�Ķ�����ͼƬ</param>
void DealMaskRegion(vector<cv::Mat>& src);
void ConvertPoint2VoxelPos(const Eigen::Vector3d p, Eigen::Vector3i& pos);