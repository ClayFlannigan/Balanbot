#ifndef ComPacket_H_H
#define ComPacket_H_H

#include <Arduino.h>

#define uchar  unsigned char
#define uint   unsigned int



class ComPacket
{
public:
	bool m_PackageOK;
	uchar m_Buffer[6];
	uchar m_ComNum;
	uint m_baud;
	
	ComPacket()
	{
	m_PackageOK = false;
	m_Buffer[0] = 0;m_Buffer[1] = 0;m_Buffer[2] = 0;m_Buffer[3] = 0;m_Buffer[4] = 0;m_Buffer[5] = 0;
	}	
};

#endif
