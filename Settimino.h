/*=============================================================================|
|  PROJECT SETTIMINO                                                     2.0.0 |
|==============================================================================|
|  Copyright (C) 2013, 2019 Davide Nardella                                    |
|  All rights reserved.                                                        |
|==============================================================================|
|  SETTIMINO is free software: you can redistribute it and/or modify           |
|  it under the terms of the Lesser GNU General Public License as published by |
|  the Free Software Foundation, either version 3 of the License, or           |
|  (at your option) any later version.                                         |
|                                                                              |
|  It means that you can distribute your commercial software linked with       |
|  SETTIMINO without the requirement to distribute the source code of your     |
|  application and without the requirement that your application be itself     |
|  distributed under LGPL.                                                     |
|                                                                              |
|  SETTIMINO is distributed in the hope that it will be useful,                |
|  but WITHOUT ANY WARRANTY; without even the implied warranty of              |
|  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               |
|  Lesser GNU General Public License for more details.                         |
|                                                                              |
|  You should have received a copy of the GNU General Public License and a     |
|  copy of Lesser GNU General Public License along with Snap7.                 |
|  If not, see  http://www.gnu.org/licenses/                                   |
|------------------------------------------------------------------------------|
|                                                                              |
|  1.1.0 Added support for ESP8266 (Thanks to Geoffrey Hayward Piggot)         |
|  2.0.0 Added new hardware support                                            |
|        Added Read/Write consistent bit into the CPU                          |
|        Added new 18 helper functions                                         |
|        Small bugfixes (Thanks to Daniel Förstmann and Schöneberg Swen)       |
|                                                                              |
|=============================================================================*/
#ifndef SETTIMINO_H
#define SETTIMINO_H

#include <stdint.h>
#include <Arduino.h>
#include <Client.h>

// Memory models

//#define _SMALL
//#define _NORMAL
#define _EXTENDED


#if defined(_NORMAL) || defined(_EXTENDED)
# define _S7HELPER
#endif

#pragma pack(1)
// Error Codes 
// from 0x0001 up to 0x00FF are severe errors, the Client should be disconnected
// from 0x0100 are S7 Errors such as DB not found or address beyond the limit etc..
// For Arduino Due the error code is a 32 bit integer but this doesn't change the constants use.
#define errTCPConnectionFailed 0x0001
#define errTCPConnectionReset  0x0002
#define errTCPDataRecvTout     0x0003
#define errTCPDataSend         0x0004
#define errTCPDataRecv         0x0005
#define errISOConnectionFailed 0x0006
#define errISONegotiatingPDU   0x0007
#define errISOInvalidPDU       0x0008

#define errS7InvalidPDU        0x0100
#define errS7SendingPDU        0x0200
#define errS7DataRead          0x0300
#define errS7DataWrite         0x0400
#define errS7Function          0x0500

#define errBufferTooSmall      0x0600

// Connection Type
#define PG       0x01
#define OP       0x02
#define S7_Basic 0x03

// ISO and PDU related constants
#define ISOSize        7  // Size of TPKT + COTP Header
#define isotcp       102  // ISOTCP Port
#define MinPduSize    16  // Minimum S7 valid telegram size
#define MaxPduSize   247  // Maximum S7 valid telegram size (we negotiate 240 bytes + ISOSize)
#define CC          0xD0  // Connection confirm
#define Shift         17  // We receive data 17 bytes above to align with PDU.DATA[]

// S7 ID Area (Area that we want to read/write)
#define S7AreaPE    0x81
#define S7AreaPA    0x82
#define S7AreaMK    0x83
#define S7AreaDB    0x84
#define S7AreaCT    0x1C
#define S7AreaTM    0x1D

// uint16_tLength
#define S7WLBit     0x01
#define S7WLuByte    0x02
#define S7WLuint16_t    0x04
#define S7WLDuint16_t   0x06
#define S7WLReal    0x08
#define S7WLCounter 0x1C
#define S7WLTimer   0x1D

#define TS_ResBit   0x03
#define TS_ResByte  0x04
#define TS_ResInt   0x05
#define TS_ResReal  0x07
#define TS_ResOctet 0x09

const uint8_t S7CpuStatusUnknown = 0x00;
const uint8_t S7CpuStatusRun     = 0x08;
const uint8_t S7CpuStatusStop    = 0x04;

#define RxOffset    18
#define Size_RD     31
#define Size_WR     35

//typedef uint16_t uint16_t;          // 16 bit unsigned integer

typedef int16_t integer;        // 16 bit signed integer
typedef unsigned long duint16_t;    // 32 bit unsigned integer
typedef long dint;              // 32 bit signed integer

typedef uint8_t *pbyte;
typedef uint16_t *puint16_t;
typedef duint16_t *pduint16_t;
typedef integer *pinteger;
typedef dint *pdint;
typedef float *pfloat;
typedef char *pchar;

typedef union{
	struct {
		uint8_t H[Size_WR];                      // PDU Header
		uint8_t DATA[MaxPduSize-Size_WR+Shift];  // PDU Data
	};
	uint8_t RAW[MaxPduSize+Shift];
}TPDU;


#pragma pack()

#ifdef _S7HELPER

class S7Helper
{
public:
	bool BitAt(void *Buffer, int uint8_tIndex, uint8_t BitIndex);
	bool BitAt(int uint8_tIndex, int BitIndex);
	uint8_t byteAt(void *Buffer, int index);
	uint8_t byteAt(int index);
	uint16_t uint16_tAt(void *Buffer, int index);
	uint16_t uint16_tAt(int index);
	duint16_t Duint16_tAt(void *Buffer, int index);
	duint16_t Duint16_tAt(int index);
	float FloatAt(void *Buffer, int index);
	float FloatAt(int index);
	integer IntegerAt(void *Buffer, int index);
	integer IntegerAt(int index);
	long DintAt(void *Buffer, int index);
	long DintAt(int index);
	// New 2.0
    void SetBitAt(void *Buffer, int uint8_tIndex, int BitIndex, bool Value);
	void SetBitAt(int uint8_tIndex, int BitIndex, bool Value);
	void SetByteAt(void *Buffer, int index, uint8_t value);
	void SetByteAt(int index, uint8_t value);
	void SetIntAt(void *Buffer, int index, integer value);
	void SetIntAt(int index, integer value);
	void SetDIntAt(void *Buffer, int index, dint value);
	void SetDIntAt(int index, dint value);
	void Setuint16_tAt(void *Buffer, int index, uint16_t value);
	void Setuint16_tAt(int index, uint16_t value);
	void SetDuint16_tAt(void *Buffer, int index, duint16_t value);
	void SetDuint16_tAt(int index, uint16_t value);
	void SetFloatAt(void *Buffer, int index, float value);
	void SetFloatAt(int index, float value);
	char * StringAt(void *Buffer, int index);
	char * StringAt(int index);
	void SetStringAt(void *Buffer, int index, char *value);
	void SetStringAt(int index, char *value);	
};
extern S7Helper S7;

#endif // _S7HELPER

//-----------------------------------------------------------------------------
// Ethernet initialization
//-----------------------------------------------------------------------------
void EthernetInit(uint8_t *mac, IPAddress ip);
//-----------------------------------------------------------------------------
// S7 Client                                       
//-----------------------------------------------------------------------------
class S7Client 
{
private:
	uint8_t LocalTSAP_HI; 
	uint8_t LocalTSAP_LO;
	uint8_t RemoteTSAP_HI;
	uint8_t RemoteTSAP_LO;
	uint8_t LastPDUType;
	uint16_t ConnType;
	
	// Since we can use either an EthernetClient or a WifiClient 
	// we have to create the class as an ancestor and then resolve
	// the inherited into S7Client creator.
	Client *TCPClient;
	
	int PDULength;    // PDU Length negotiated
	int IsoPduSize();
	int WaitForData(uint16_t Size, uint16_t Timeout);
	int RecvISOPacket(uint16_t *Size);
	int RecvPacket(uint8_t *buf, uint16_t Size);
	int ISOConnect();
	int NegotiatePduLength();
	int SetLastError(int Error);
public:
	// Output properties
	bool Connected;   // true if the Client is connected
	int LastError;    // Last Operation error
	// Input properties
	uint16_t RecvTimeout; // Receving timeour
	// Methods
	S7Client(Client& client);
	virtual ~S7Client();
	// Basic functions
	void SetConnectionParams(uint16_t LocalTSAP, uint16_t RemoteTSAP);
	void SetConnectionType(uint16_t ConnectionType);
	// make sure that the underlying client is connected before calling ConnectTo
	int ConnectTo(uint16_t Rack, uint16_t Slot);
	int Connect();
	void Disconnect();
	int ReadArea(int Area, uint16_t DBNumber, uint16_t Start, uint16_t Amount, void *ptrData); 
	int ReadArea(int Area, uint16_t DBNumber, uint16_t Start, uint16_t Amount, int uint16_tLen, void *ptrData); 
	int ReadBit(int Area, uint16_t DBNumber, uint16_t BitStart, bool &Bit); 
	int WriteArea(int Area, uint16_t DBNumber, uint16_t Start, uint16_t Amount, void *ptrData); 
	int WriteArea(int Area, uint16_t DBNumber, uint16_t Start, uint16_t Amount, int uint16_tLen, void *ptrData); 
	int WriteBit(int Area, uint16_t DBNumber, uint16_t BitIndex, bool Bit); 
	int WriteBit(int Area, uint16_t DBNumber, uint16_t byteIndex, uint16_t BitInByte, bool Bit); 
	
	int GetPDULength(){ return PDULength; }
	// Extended functions
#ifdef _EXTENDED
	int GetDBSize(uint16_t DBNumber, uint16_t *Size);
	int DBGet(uint16_t DBNumber, void *ptrData, uint16_t *Size);
    int PlcStart(); // Warm start
    int PlcStop();
    int GetPlcStatus(int *Status);
	int IsoExchangeBuffer(uint16_t *Size);
	void ErrorText(int Error, char *Text, int TextLen);
#endif
};

extern TPDU PDU;

#endif // SETTIMINO_H
