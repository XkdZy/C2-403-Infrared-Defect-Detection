#pragma once
#define _CRT_SECURE_NO_WARNINGS

#ifndef WIN32_LEAN_AND_MEAN
#include WIN32_LEAN_AND_MEAN
#endif

#include <iostream>
#include <cstdint>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "cJSON.h"
#include <string>
#include <winsock2.h>  


#include<stdlib.h>
#include <fstream>
#include<vector>
#include <time.h>

#include "Eigen/Dense"

using namespace std;

#define SEERCONTROLIP "192.168.3.38"
#define AGV_PI 3.1415926

extern SOCKET m_SockClient10;//�˿�19210  DO���  ��բ��
extern SOCKET m_SockClient04;//�˿�19204  ��ѯ
extern SOCKET m_SockClient05;//�˿�19204  ����
extern SOCKET m_SockClient06;//�˿�19206  ����

void InitSeerAGV();
/// <summary>
/// 
/// </summary>
/// <param name="ms"></param>
void delay_ms(int ms, int x);
/// <summary>
/// ��ʼ�����绷��
/// </summary>
void InitInternet();
/// <summary>
/// �����ɹ�ֻ�ܿ�����
/// </summary>
/// <param name="port">������״̬��19204�����ƣ�19205������ API	19206�����ã�19207��������19210�����ͣ�19301</param>
/// <param name="client">�ͻ����׽���</param>
/// <returns>�����Ƿ�ɹ�true���ɹ�</returns>
bool ConnectSeer(const u_short port, SOCKET& client);
/// <summary>
/// ��ѯ��������Ϣָ�תSeer����֡
/// </summary>
/// <param name="m_number">ָ����</param>
/// <param name="m_type">ָ������</param>
/// <returns>Seer����֡</returns>
char* RobotState2SeerFrame(const uint16_t m_number, const uint16_t m_type);
/// <summary>
/// �����˵���ָ�JsonתSeer����֡
/// </summary>
/// <param name="m_number">ָ����</param>
/// <param name="m_type">ָ������</param>
/// <param name="json">json����֡</param>
/// <returns>Seer����֡</returns>
char* RobotNavigationJSON2SeerFrame(const uint16_t m_number, const uint16_t m_type, const char* json);

/// <summary>
/// ��������ϵ�µ�����ָ��λ��
/// </summary>
/// <param name="x"></param>
/// <param name="y"></param>
/// <param name="theta"></param>
/// <param name="client"></param>
void AMRRobotNavigation(const double x, const double y, const double theta, SOCKET& client);

/// <summary>
/// ��������ϵ�µ�����ָ��λ��  ����Ҫָ������
/// </summary>
/// <param name="x"></param>
/// <param name="y"></param>
/// <param name="client"></param>
void AMRRobotNavigation(const double x, const double y, SOCKET& client);

/// <summary>
/// ���ɵ���
/// </summary>
/// <param name="x"></param>
/// <param name="y"></param>
/// <param name="theta"></param>
/// <param name="client"></param>
void AMRgofree(const double x, const double y, const double theta, SOCKET& client);

#pragma region ����ָ��

/// <summary>
/// AMRƽ��
/// </summary>
/// <param name="dist">����</param>
/// <param name="vx">x�����ٶ�</param>
/// <param name="vy">y�����ٶ�</param>
/// <returns>Seer����֡</returns>
char* AMRLibration(const double dist, const double vx, const double vy, SOCKET& client, const bool mode);
/// <summary>
/// AMRת��
/// </summary>
/// <param name="angle">���ȣ�2��=360�ȣ�</param>
/// <param name="vw">�ٶȣ�+��˳ʱ��   -����ʱ�룩</param>
/// <param name="client">�����׽���</param>
void AMRRotation(const double angle, const double vw, SOCKET& client);
/// <summary>
/// ֹͣ����
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
char* AMRstop_Navigation(const SOCKET& client);
#pragma endregion










#pragma region ��ȡ��̫�����ݣ���������

/// <summary>
/// 
/// </summary>
/// <param name="client"></param>
/// <returns></returns>

char* ReadAMRRespond(const SOCKET& client);
int ReadAMRRespond(const SOCKET& client, char*& data);
/// <summary>
/// ��ȡAMRλ��
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
char* ReadAMRPostion(const SOCKET& client);
/// <summary>
/// �����״����ݶ�ȡ
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
char* ReadAMRLaser_res(const SOCKET& client);
int ReadAMRLaser_res(const SOCKET& client, char*& data, bool multi);
/// <summary>
/// ���������ȡ
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
char* ReadAMRnavigation(const SOCKET& client);
/// <summary>
/// 
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
char* ReadAMRInfo(const SOCKET& client);
int ReadAMRInfo(const SOCKET& client, char*& data);

char* ReadAMblocked(const SOCKET& client);

int AMRblocked_status(SOCKET& client);

#pragma endregion

#pragma region �򿪱�բ
/// <summary>
/// �򿪱�բָ��
/// </summary>
/// <param name="id">DO��</param>
/// <param name="status">״̬</param>
/// <param name="client"></param>
char* AMRUnlock(const double id, const bool status, SOCKET& client);

#pragma endregion

#pragma region ��jsonתΪ������ʽ

/// <summary>
/// ����״̬��ȡ
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
int AMRnavigation_status(SOCKET& client);

/// <summary>
/// ��ȡagvĿǰλ��
/// </summary>
/// <param name="ss"></param>   x��    y��
/// <param name="client"></param>
/// <returns></returns>
float AMRnow_position(const string ss,SOCKET& client);

#pragma endregion

void AMRStartCreateMap(int slam_tpye, bool real_time, int screen_width, int screen_height);
void AMREndCreateMap();
void AMRQueryScanMapStatus();
void AMRQueryToatlMap();

struct AMRLocalInfo {
	double _x;
	double _y;
	double _angle;
};
void RequestAMRLocal(AMRLocalInfo& amrli);
bool WhetherAMRMove2Goal(const SOCKET& client, AMRLocalInfo& goal, AMRLocalInfo& last, int which);
// ����ɨ���ͼ
void ReScanMap();
/// <summary>
/// ��ȡIMU
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
char* status_imu_req(const SOCKET& client);
/// <summary>
/// ����IMU����
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
Eigen::Vector3d robot_status_imu_req(SOCKET& client);


#pragma region ����

/// <summary>
/// ����״̬��ȡ
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
char* robot_control_reloc_req(const double x, const double y, const double theta, SOCKET& client);
/// <summary>
/// 
/// </summary>
/// <param name="client"></param>
/// <returns></returns>
char* robot_control_comfirmloc_req(const SOCKET& client);

void robot_reloc_req(void);

#pragma endregion



