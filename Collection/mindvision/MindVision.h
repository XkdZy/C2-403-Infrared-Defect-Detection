#pragma once

#include <iostream>
using namespace std;
#include <string>

bool init_mindvision();
/// <summary>
/// ��������Ӿ�ͼƬ
/// </summary>
/// <param name="fileName">·�������ļ���ʽ</param>
void save_mindvision(char* fileName);
/// <summary>
/// ���ø����Ӿ��ع�ʱ��
/// </summary>
/// <param name="exptime">�ع�ʱ�䵥λms</param>
void set_mindivision_exposuretime(double exptime);
