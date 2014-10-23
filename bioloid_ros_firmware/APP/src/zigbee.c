// Zigbee SDK platform independent source
#include "zgb_hal.h"
#include "zigbee.h"

unsigned char gbRcvPacket[6];
unsigned char gbRcvPacketNum;
unsigned short gwRcvData;
unsigned char gbRcvFlag;
int csf = 0;

int zgb_initialize( int devIndex )
{
	if( zgb_hal_open( devIndex, 57600 ) == 0) // Always fixed baudrate
		return 0;

	gbRcvFlag = 0;
	gwRcvData = 0;
	gbRcvPacketNum = 0;
	return 1;
}

void zgb_terminate(void)
{
	zgb_hal_close();
}

int zgb_tx_data(int data)
{
	unsigned char SndPacket[6];
	unsigned short word = (unsigned short)data;
	unsigned char lowbyte = (unsigned char)(word & 0xff);
	unsigned char highbyte = (unsigned char)((word >> 8) & 0xff);

	SndPacket[0] = 0xff;
	SndPacket[1] = 0x55;
	SndPacket[2] = lowbyte;
	SndPacket[3] = ~lowbyte;
	SndPacket[4] = highbyte;
	SndPacket[5] = ~highbyte;

	if( zgb_hal_tx( SndPacket, 6 ) != 6 )
		return 0;

	return 1;
}

int zgb_rx_check(void)
{
	int RcvNum;
	unsigned char checksum;
	int i, j;

	if(gbRcvFlag == 1)
		return 1;

	// Fill packet buffer
	if(gbRcvPacketNum < 6)
	{
		RcvNum = zgb_hal_rx( &gbRcvPacket[gbRcvPacketNum], (6 - gbRcvPacketNum) );
		if( RcvNum != -1 )
			gbRcvPacketNum += RcvNum;
	}

	// Find header
	if(gbRcvPacketNum >= 2)
	{
		for( i=0; i<gbRcvPacketNum; i++ )
		{
			if(gbRcvPacket[i] == 0xff)
			{
				if(i <= (gbRcvPacketNum - 2))
				{
					if(gbRcvPacket[i+1] == 0x55)
						break;
				}
			}
		}

		if(i > 0)
		{
			if(i == gbRcvPacketNum)
			{
				// Can not find header
				if(gbRcvPacket[i - 1] == 0xff)
					i--;
			}

			// Remove data before header
			for( j=i; j<gbRcvPacketNum; j++)
			{
				gbRcvPacket[j - i] = gbRcvPacket[j];
			}
			gbRcvPacketNum -= i;
		}
	}

	// Verify packet
	if(gbRcvPacketNum == 6)
	{
		if(gbRcvPacket[0] == 0xff && gbRcvPacket[1] == 0x55)
		{
			//TxDString("RcvPacket[2] : ");
			//TxDByte16(gbRcvPacket[2]);
			//TxDString("  RcvPacket[3] : ");
			//TxDByte16(gbRcvPacket[3]);
			//TxDString("\r\n");
			checksum = ~gbRcvPacket[3];
			if(gbRcvPacket[2] == checksum)
			{
				//TxDString("RcvPacket[4] : ");
				//TxDByte16(gbRcvPacket[4]);
				//TxDString("  RcvPacket[5] : ");
				//TxDByte16(gbRcvPacket[5]);
				//TxDString("\r\n");
				checksum = ~gbRcvPacket[5];
				if(gbRcvPacket[4] == checksum)
				{
					gwRcvData = (unsigned short)((gbRcvPacket[4] << 8) & 0xff00);
					gwRcvData += gbRcvPacket[2];
					gbRcvFlag = 1;
					csf = 0;
				} else {
					//TxDString("Checksum2 failed!\r\n");
					csf = -1;
				}
			} else {
				//TxDString("Checksum1 failed!\r\n");
				csf = -1;
			}
		} else {
			//TxDString("Header not Correct\r\n");
		}

		gbRcvPacket[0] = 0x00;
		gbRcvPacketNum = 0;
	}

	return gbRcvFlag;
}

int zgb_rx_data(void)
{
	gbRcvFlag = 0;
	return (int)gwRcvData;
}
