#pragma once

#include <iostream>
using namespace std;
#include <string>
#include <opencv2/opencv.hpp>

#include "dijkstra.h"

// ������ͣ����ͼ������Ϣ��ͼƬ��Ϣ+λ����Ϣ
typedef struct _cameraImgInfo {
	// ͼƬ���ͣ�0��hd rgb��1��rgbd��2��thermal
	int _type;
	// ͼƬ��Ϣ
	cv::Mat _RgbImg;
	cv::Mat _DepthImg;
	// λ����Ϣ
	cv::Mat _Gripper2Base; // ��е�۶�ȡ����Gripperλ��
	Eigen::Vector3d _CameraPosition; // ���λ��
	cv::Mat _CameraPose; // ���λ��
	Eigen::Vector3d _CameraPostureEig; // �����̬ŷ������ʽ
	cv::Mat _CameraPostureMat; // �����̬��ת������ʽ
	// 6Dλ���ַ�����ʽ
	string _poseStr;
	Eigen::Vector3d _agvPose; // �ɼ���ǰͼƬAGVλ��
	Eigen::Vector3d _agvIMU; // �ɼ���ǰͼƬAGV IMU
}cameraImgInfo;


// ��ǰͼƬ��Ӧ����Ĥ��Ϣ
typedef struct _bboxImgInfo {
	int _imgType; // 0��RGBD��1��HD
	cv::Mat _src; // bgr
	cv::Mat _selectmask; // mask
	vector<cv::Mat> _img; // ������ĤͼƬ binary
	//vector<cv::Mat> _maskSrc; // ������Ĥ��ͼ gary
	vector<string> _imgPath; // ������ĤͼƬ binary
	//vector<string> _maskSrc; // ������Ĥ��ͼ gary
	vector<double> _maskArea; // ��Ĥ���
	vector<cv::Rect> _bboxRect; // ����bbox��Ϣ(x,y,width,height)
	vector<Eigen::Vector4i> _bboxAB; // ����bbox��Ϣ(x1,y1,x2,y2)
	vector<vector<cv::Point>> _totalContours; // ��ǰimage��������
	vector<vector<cv::Point>> _maskContour; // ÿһ��bbox��Ĥ��Ӧ�������
	vector<vector<cv::Point>> _contour; // ÿһ��bbox��Ӧ���������
	vector<double> _contourErr; // ÿһ��bbox��Ӧ������������
}bboxImgInfo;

typedef struct _controlRobotInfo {
	cv::Mat _campose; // 4*4
	cv::Mat _camposture; // 3*3
	cv::Mat _campositionMat; // 4*1
	Eigen::Vector3d _campositionEigen;
	Eigen::Vector3d _cameuler;
	string _grippercmdStr;
}controlRobotInfo;

bool PixelInBBox(const cv::Rect& rec, const cv::Point& pixel);
/// <summary>
/// ��ʱ���ж�
/// </summary>
void TimerInterpute(void);
/// <summary>
/// �ɼ���ʾʵʱ��RGBD����
/// </summary>
cv::Point CollectImgAndInteraction(cameraImgInfo& cii);
/// <summary>
/// ������ͼƬƴ�ӵ�һ��ͼƬ��
/// </summary>
/// <param name="m1">����ͼ</param>
/// <param name="m2">ǰ��ͼ</param>
/// <param name="m2">ƴ�Ӻ��ͼƬ</param>
void CombinateTwoImg(const cv::Mat& mVertical, const cv::Mat& mFront, cv::Mat& dst);
/// <summary>
/// ����ʵʱͼƬ��ϢdepthImgInfo
/// </summary>
/// <param name="rgb">rgb</param>
/// <param name="depth">���ͼ</param>
/// <param name="imgPath">��е��ĩ����Ϣ</param>
void UpdateRealTimePosInfo(const cv::Mat& rgb, const cv::Mat& depth, string imgPose, cameraImgInfo& cii, const Eigen::Vector3d agvPose = Eigen::Vector3d(0., 0., 0.), const Eigen::Vector3d agvIMU = Eigen::Vector3d(0., 0., 0.));
/// <summary>
/// ��һ��ͼ���е���ģ���մ�С��������
/// </summary>
/// <param name="vImg">ͼ������</param>
/// <returns>���شӴ�С������</returns>
vector<int> CompareImageArea(const vector<cv::Mat>& vImg);
/// <summary>
/// ����maskͳ��α�����Ч������
/// </summary>
/// <param name="img">��ֵ��ԭʼͼ��</param>
/// <param name="depth">double���ͼ</param>
/// <param name="mask">���ͼ����Ĥ</param>
/// <returns>����ͼ����Ч�������ֵ��Ϊ255�������ֵ��</returns>
int CountImageWhiteArea(const cv::Mat& img, const cv::Mat& depth, const double moreThanVal, cv::Mat& mask);
double CountImageAgvDepth(const cv::Mat& depth, cv::Mat& mask);
/// <summary>
/// ����ͼƬ��Ϣ����ĩ��ת����ϵ�����λ�ˡ�ĩ��λ�˵�
/// </summary>
/// <param name="rgb">����rgbͼ��</param>
/// <param name="depth">����depthͼ��typeΪ0ʱ��rgb=depth��</param>
/// <param name="pose">���뵱ǰĩ��λ��</param>
/// <param name="type">����ͼƬ���ͣ�0��rgbd	1��hd rgb	2��thermal</param>
/// <param name="cameraInfo">�����ͼƬ���Ϻ���Ϣ</param>
void IntegrateImgInfo(const cv::Mat& rgb, const cv::Mat& depth, string pose, int type, cameraImgInfo& cameraInfo, const Eigen::Vector3d agvPose = Eigen::Vector3d(0., 0., 0.), const Eigen::Vector3d agvIMU = Eigen::Vector3d(0., 0., 0.));
/// <summary>
/// ����VILD����BBoxͼƬ��Ϣ��bboxλ�á�binary��Ĥ·���ȣ����建��ɱ�̫��ֻ����ͼƬ·����
/// </summary>
/// <param name="bboxPth">bbox��Ŀ¼λ�ã�������������./imgs/hdvild��</param>
/// <param name="bboxInfo">bbox�ṹ��</param>
/// <param name="type">����ͼƬ���ͣ�0��rgbd	1��hd rgb	2��thermal</param>
/// <param name="type">ͼƬ�������ͣ�0��vild	>=1��bts ����ʾͼƬ����</param>
void IntegrateBBoxInfo(const string& bboxPth, bboxImgInfo& bboxInfo, int imgtype, int inftype);
void IntegrateVildBBoxInfo(const string& bboxPth, bboxImgInfo& bboxInfo, int imgtype);
/// <summary>
/// ����������ϵת����ͼ������ϵ
/// </summary>
/// <param name="u">��������ϵ����</param>
/// <param name="v">��������ϵ����</param>
/// <param name="d">��ǰ��u��v�����ض�Ӧ���</param>
/// <param name="cameraType">������ͣ�0��ȣ�1HD��2Thermal</param>
/// <returns></returns>
Eigen::Vector3d ConvertPixel2Camera(const int& u, const int& v, const ushort& d, const int& cameraType);
/// <summary>
/// ����python�ű�
/// </summary>
/// <param name="vildOrbts">VILD����BTS-Net</param>
/// <returns></returns>
bool CallPythonScriptInference(bool vildOrbts);
/// <summary>
/// ������ɢ�㼯��Ѱ����С͹������������ɢ�㼯
/// </summary>
/// <param name="vPoints">��ɢ�㼯</param>
/// <returns>͹����Ĥ</returns>
cv::Mat FillImageBasePoint(const vector<cv::Point>& vPoints);
double FindHHOptimumExposureTime();
/// <summary>
/// Ѱ��ͼƬ������
/// </summary>
/// <param name="src">ͼƬ</param>
/// <param name="oneMax">1��ֻ�����������0��������������</param>
/// <returns>�����б�</returns>
vector<vector<cv::Point>> FindContours(const cv::Mat& src, bool oneMax);
/// <summary>
/// �ҳ�Ŀ������������������������С������
/// </summary>
/// <param name="src">��������</param>
/// <param name="mask">Ŀ������</param>
/// <param name="optimumSrc">���������������С��ԭʼ����</param>
/// <param name="optimumed">optimumSrc�������С�������ֵ�</param>
/// <returns>��ǰ�������</returns>
double FindOptimumContours(const vector<vector<cv::Point>>& src, const vector<vector<cv::Point>>& mask, vector<cv::Point>& optimumSrc, vector<cv::Point>& optimumed);
bool InterSect(const cv::Mat& m1, const cv::Mat& m2);
bool WhetherUpdate(const cv::Mat& src, const cv::Mat& update);
/// <summary>
/// ��ȡtxt�ı�������imgs/data/��ʽ˵��.txtΪ��
/// </summary>
/// <param name="path">�ļ�·��</param>
/// <param name="imgType">������������0��rgbd��1��thermal</param>
/// <param name="vCam">��е����Ϣ</param>
/// <param name="vAgv">agv��Ϣ</param>
void IntegrateTextInfo(const string path, const int imgType, vector<cameraImgInfo>& vCam, vector<Eigen::Vector3d>& vAgv);