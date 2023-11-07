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
	//打开串口
	bool Serial_open(LPCSTR COMx, int BaudRate);
	// 发送数据
	int Serial_write_string(std::string& Buf, int size);
	int Serial_write_char(unsigned char* Buf, int size);
	// 接收数据
	int Serial_read_string(std::string& OutBuf, int maxSize);
	int Serial_read_char(unsigned char* OutBuf, int maxSize);
public:
	HANDLE m_hComm;//串口句柄
};
