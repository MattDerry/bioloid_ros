//##########################################################
//##                      R O B O T I S                   ##
//##       Send/Recieve Example code for Dynamixel.       ##
//##                                           2009.11.10 ##
//##########################################################
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <zigbee.h>

// Defulat setting
#define DEFAULT_DEVICEINDEX	0
#define TIMEOUT_TIME		1000 // msec

int main()
{
	int TxData, RxData;
	int i;

	// Open device
	if( zgb_initialize(DEFAULT_DEVICEINDEX) == 0 )
	{
		printf( "Failed to open Zig2Serial!\n" );
		printf( "Press any key to terminate...\n" );
		getchar();
		return 0;
	}
	else
		printf( "Succeed to open Zig2Serial!\n" );

	while(1)
	{
		printf( "Press any key to continue!(press ESC to quit)\n" );
		if(getchar() == 0x1b)
			break;

		// Wait user's input
		printf( "Input number to transmit: " );
		if(scanf("%d", &TxData));

		// Transmit data
		if(zgb_tx_data(TxData) == 0)
			printf( "Failed to transmit\n" );

		for( i=0; i<TIMEOUT_TIME; i++)
		{
			// Verify data recieved
			if(zgb_rx_check() == 1)
			{
				// Get data verified
				RxData = zgb_rx_data();
				printf( "Recieved: %d\n", RxData );
				break;
			}

			sleep(1);
		}

		if(i == TIMEOUT_TIME)
			printf( "Timeout: Failed to recieve\n" );
	}

	// Close device
	zgb_terminate();
	printf( "Press any key to terminate...\n" );
	getchar();
	return 0;
}
