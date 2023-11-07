#include "SerialPort.h"

SerialPort::SerialPort(void) {
	m_hComm = INVALID_HANDLE_VALUE;
}

SerialPort::~SerialPort(void) {
	if (m_hComm != INVALID_HANDLE_VALUE) {
		CloseHandle(m_hComm);
		std::cout << "serial close!" << std::endl;
	}
}
/*******************************************************************
* 功能     ：  打开串口
* COMx     :   串口号, 如_T("COM2:")
* BaudRate:    波特率
*******************************************************************/
bool SerialPort::Serial_open(LPCSTR COMx, int BaudRate) {
	DCB dcb = { 0 };
	m_hComm = CreateFileA(COMx,
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		0,//FILE_FLAG_OVERLAPPED,	//同步方式 或 重叠方式
		0
	);

	if (m_hComm == INVALID_HANDLE_VALUE) {
		DWORD dwError = GetLastError();
		return FALSE;
	}

	dcb.DCBlength = sizeof(DCB);

	if (!GetCommState(m_hComm, &dcb)) {
		DWORD dwError = GetLastError();
		return FALSE;
	}

	dcb.BaudRate = BaudRate;	//波特率
	dcb.ByteSize = 8;			//位数
	dcb.Parity = NOPARITY;		//奇偶检验
	dcb.StopBits = ONESTOPBIT;	//停止位数

	if (!SetCommState(m_hComm, &dcb)) {
		DWORD dwError = GetLastError();
		return FALSE;
	}
	if (!PurgeComm(m_hComm, PURGE_RXCLEAR))    return -1;

	SetupComm(m_hComm, 1024, 1024);
	return TRUE;
}

/**
  serial write
  @param Buf:data buf
  @param size:bytes of Buf
  @return The len of writen
*/
int SerialPort::Serial_write_string(string& Buf, int size) {
	DWORD dw;
	char charBuf[1024];
	strcpy_s(charBuf, Buf.c_str());
	WriteFile(m_hComm, charBuf, size, &dw, NULL);
	return dw;
}
int SerialPort::Serial_write_char(unsigned char* Buf, int size)
{
	DWORD dw;
	WriteFile(m_hComm, Buf, size, &dw, NULL);
	return dw;
}
/**
  serial read
  @param OutBuf:返回得到的数据
  @param maxSize:存放数据最大宽度
  @return The len of read data
*/
int SerialPort::Serial_read_string(string& OutBuf, int maxSize) {
	DWORD readSize = 0;
	DWORD dwError = 0;
	BOOL  bReadStatus;
	COMSTAT cs;
	ClearCommError(m_hComm, &dwError, &cs);
	if (cs.cbInQue > maxSize) {
		PurgeComm(m_hComm, PURGE_RXCLEAR);
		return 0;
	}
	char OutBufchar[100] = "";
	//memset(OutBufchar, 0, sizeof(OutBuf));
	bReadStatus = ReadFile(m_hComm, OutBufchar, cs.cbInQue, &readSize, 0);

	OutBuf = OutBufchar;

	return readSize;
}

int SerialPort::Serial_read_char(unsigned char* OutBuf, int maxSize) {
	DWORD readSize = 0;
	DWORD dwError = 0;
	BOOL  bReadStatus;
	COMSTAT cs;
	ClearCommError(m_hComm, &dwError, &cs);
	if (cs.cbInQue > maxSize) {
		PurgeComm(m_hComm, PURGE_RXCLEAR);
		return 0;
	}
	bReadStatus = ReadFile(m_hComm, OutBuf, cs.cbInQue, &readSize, 0);
	OutBuf[readSize] = '\0';

	return readSize;
}
