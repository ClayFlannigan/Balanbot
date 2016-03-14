
//CSerialPortEx类是在Remon Spekreijse设计的CSerialPort类基础上设计的，并增加
//了对二进制数据传输和块读写方式的支持和其他功能函数。关于CSerialPort类的声明如下：

/*
**	FILENAME			SerialPort.h
**
**	PURPOSE				This class can read, write and watch one serial port.
**						It sends messages to its owner when something happends on the port
**						The class creates a thread for reading and writing so the main
**						program is not blocked.
**
**	CREATION DATE		15-09-1997
**	LAST MODIFICATION	12-11-1997
**
**	AUTHOR				Remon Spekreijse
**
**
*/

#ifndef __SERIALPORTEX_H__
#define __SERIALPORTEX_H__

#define WM_COMM_BREAK_DETECTED		WM_USER+1	// A break was detected on input.
#define WM_COMM_CTS_DETECTED		WM_USER+2	// The CTS (clear-to-send) signal changed state. 
#define WM_COMM_DSR_DETECTED		WM_USER+3	// The DSR (data-set-ready) signal changed state. 
#define WM_COMM_ERR_DETECTED		WM_USER+4	// A line-status error occurred. Line-status errors are CE_FRAME, CE_OVERRUN, and CE_RXPARITY. 
#define WM_COMM_RING_DETECTED		WM_USER+5	// A ring indicator was detected. 
#define WM_COMM_RLSD_DETECTED		WM_USER+6	//The RLSD (receive-line-signal-detect) signal changed state. 
#define WM_COMM_RXCHAR				WM_USER+7	// A character was received and placed in the input buffer. 
#define WM_COMM_RXFLAG_DETECTED		WM_USER+8	// The event character was received and placed in the input buffer.  
#define WM_COMM_TXEMPTY_DETECTED	WM_USER+9	// The last character in the output buffer was sent.  

//#define WM_COMM_STATUS				WM_USER+21


//const	BYTE	ASK[6]={0,1,0,0,0,0};


class CSerialPortEx
{														 
public:

	// contruction and destruction
	CSerialPortEx();
	virtual		~CSerialPortEx();

	// port initialisation											
	BOOL		InitPort(CWnd* pPortOwner, UINT portnr = 1, UINT baud = 19200, char parity = 'N', UINT databits = 8, UINT stopsbits = 1, DWORD dwCommEvents = EV_RXCHAR | EV_CTS, UINT nBufferSize = 512);
	
	// start/stop comm watching
	BOOL		StartMonitoring();
	BOOL		RestartMonitoring();
	BOOL		StopMonitoring();

	DWORD		GetWriteBufferSize();
	DWORD		GetCommEvents();

	DCB			GetDCB();

	//Transfer

public:
	void		WriteToPort(BYTE* string,int nLength);
	void		WriteToPort(CString str);
	void        ClosePort();                     // add by mrlong 2007-12-2  

protected:
	// protected memberfunctions
	void		ProcessErrorMessage(char* ErrorText);
	static UINT	CommThread(LPVOID pParam);
	static void	ReceiveChar(CSerialPortEx* port, COMSTAT comstat);
	static void	WriteChar(CSerialPortEx* port);
public:
	static BYTE* ReadBlock(CSerialPortEx *port, int& readLen);

protected:

	int m_nToSend;

	// thread
	CWinThread*			m_Thread;

	// synchronisation objects
	CRITICAL_SECTION	m_csCommunicationSync;


	// handles
	HANDLE				m_hShutdownEvent;
	HANDLE				m_hComm;
	HANDLE				m_hWriteEvent;

	// Event array. 
	// One element is used for each event. There are two event handles for each port.
	// A Write event and a receive character event which is located in the overlapped structure (m_ov.hEvent).
	// There is a general shutdown when the port is closed. 
	HANDLE				m_hEventArray[3];

	// structures
	OVERLAPPED			m_ov;
	COMMTIMEOUTS		m_CommTimeouts;
	
	CString				m_strRec;


public:

	// owner window
	CWnd*				m_pOwner;


	// misc
	UINT				m_nPortNr;
	BYTE*				m_szWriteBuffer;
	DWORD				m_dwCommEvents;
	DWORD				m_nWriteBufferSize;
	DCB					m_dcb;

	BOOL				m_bThreadAlive;
	BOOL				m_bBlockRead;

	//CONTROL CHARACTERS
public:

};

#endif __SERIALPORTEX_H__


