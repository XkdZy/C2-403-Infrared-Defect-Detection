#pragma once

#include <iostream>
#include <Windows.h>
#include <string>

using namespace std;

class SerialPort
{
public:
	SerialPort(void);
	~SerialPort(void);
	//�򿪴���
	bool Serial_open(LPCSTR COMx, int BaudRate);
	// ��������
	int Serial_write_string(std::string& Buf, int size);
	int Serial_write_char(unsigned char* Buf, int size);
	// ��������
	int Serial_read_string(std::string& OutBuf, int maxSize);
	int Serial_read_char(unsigned char* OutBuf, int maxSize);
public:
	HANDLE m_hComm;//���ھ��
};
