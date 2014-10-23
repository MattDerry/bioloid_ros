/************************* (C) COPYRIGHT 2013 *********************************
* File Name          : main.c
* Author             :
* Version            : V0.0.1
* Date               :
* Description        : Main program body
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
//#include "includes.h"
#include "stm32f10x_lib.h"
#include "dynamixel.h"
#include "dxl_hal.h"
#include "map.h"
#include "zgb_hal.h"
#include "zigbee.h"

/* Define labels, don't change unless you know what you are doing ------------*/
#define P_GOAL_POSITION_L		30
#define P_GOAL_POSITION_H		31
#define P_GOAL_SPEED_L			32
#define P_GOAL_SPEED_H			33
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_PRESENT_SPEED_L		38
#define P_PRESENT_SPEED_H		39
#define P_PRESENT_LOAD_L        40
#define P_PRESENT_LOAD_H        41
#define P_MOVING				46
#define P_CW_ANGLE_LIMIT_L      6
#define P_CW_ANGLE_LIMIT_H      7
#define P_CCW_ANGLE_LIMIT_L     8
#define P_CCW_ANGLE_LIMIT_H     9

#define PORT_SIG_MOT1P          GPIOA
#define PORT_SIG_MOT1M          GPIOA
#define PORT_SIG_MOT2P          GPIOA
#define PORT_SIG_MOT2M          GPIOA
#define PORT_SIG_MOT5P          GPIOA
#define PORT_SIG_MOT5M          GPIOA
#define PORT_ZIGBEE_RESET		GPIOA

#define PORT_ENABLE_TXD			GPIOB
#define PORT_ENABLE_RXD			GPIOB
#define PORT_DXL_TXD			GPIOB
#define PORT_DXL_RXD			GPIOB
#define PORT_SIG_MOT6P          GPIOB
#define PORT_SIG_MOT6M          GPIOB

#define PORT_ADC_SELECT0        GPIOC
#define PORT_ADC_SELECT1        GPIOC
#define PORT_LED_POWER			GPIOC
#define PORT_SIG_MOT3P          GPIOC
#define PORT_SIG_MOT3M          GPIOC
#define PORT_SIG_MOT4P          GPIOC
#define PORT_SIG_MOT4M          GPIOC
#define PORT_ZIGBEE_TXD			GPIOC

#define PORT_ZIGBEE_RXD			GPIOD


#define PIN_SIG_MOT1P           GPIO_Pin_0
#define PIN_SIG_MOT1M           GPIO_Pin_1
#define PIN_SIG_MOT2P           GPIO_Pin_2
#define PIN_SIG_MOT2M           GPIO_Pin_3
#define PIN_SIG_MOT3P           GPIO_Pin_6
#define PIN_SIG_MOT3M           GPIO_Pin_7
#define PIN_SIG_MOT4P           GPIO_Pin_8
#define PIN_SIG_MOT4M           GPIO_Pin_9
#define PIN_SIG_MOT5P           GPIO_Pin_8
#define PIN_SIG_MOT5M           GPIO_Pin_11
#define PIN_SIG_MOT6P           GPIO_Pin_8
#define PIN_SIG_MOT6M           GPIO_Pin_9

#define PIN_ADC_SELECT0         GPIO_Pin_1
#define PIN_ADC_SELECT1         GPIO_Pin_2
#define PIN_ADC0				GPIO_Pin_0
#define PIN_ADC1                GPIO_Pin_5
#define PIN_VDD_VOLT			GPIO_Pin_3

#define PIN_ENABLE_TXD			GPIO_Pin_4
#define PIN_ENABLE_RXD			GPIO_Pin_5
#define PIN_DXL_TXD				GPIO_Pin_6
#define PIN_DXL_RXD				GPIO_Pin_7
#define PIN_PC_TXD				GPIO_Pin_10
#define PIN_PC_RXD              GPIO_Pin_11
#define PIN_ZIGBEE_TXD			GPIO_Pin_12
#define PIN_ZIGBEE_RXD			GPIO_Pin_2
#define PIN_ZIGBEE_RESET		GPIO_Pin_12

#define PIN_LED_POWER			GPIO_Pin_13

#define USART_DXL			    0
#define USART_PC			    2
#define USART_ZIGBEE			1

#define word                    u16
#define byte                    u8

#define PACKET_DATA0    		2
#define INVERSE_PACKET_DATA0 	3
#define PACKET_DATA1    		4
#define INVERSE_PACKET_DATA1 	5
#define PACKET_LENGTH 			6

#define ARRAYSIZE(x) (sizeof(x)/sizeof(x[0]))

#define ACK						(word)0
#define RDY						(word)-1
#define RTX						(word)-2

/* Set motor function responses */
#define COMMAND_SUCCESS         0
#define COMMUNICATION_ERROR     -1
#define ARRAY_SIZE_MISMATCH     2

//#define DEBUG                   1

/* Uncomment to reset the internal map structure for building */
//#define MAP_BUILDING             1

/* Global variables, add variable here as needed ----------------------------*/
volatile byte                   gbpRxInterruptBuffer[256]; // dxl buffer
volatile byte                   gbRxBufferWritePointer,gbRxBufferReadPointer;
volatile vu32                   gwTimingDelay,gw1msCounter;
u32                             Baudrate_DXL = 	1000000;
u32								Baudrate_ZIGBEE = 57600;
u32                             Baudrate_PC = 57600;
vu16                            CCR1_Val = 100; 		// 1ms
vu32                            capture = 0;
word							RcvData;
byte							highb, lowb;
int								rcvflag;
byte							sync_ids[16];
word							sync_pos[16];
byte							sync_dirs[16];
word							sync_speeds[16];

volatile byte                   gbPacketWritePointer = 0; // PC, Wireless
volatile byte                   gbPacketReadPointer = 0;
volatile byte                   gbPacketPointer = 0;
volatile byte                   gbpPacketDataBuffer[16+1+16];
volatile byte                   gbpPacket[PACKET_LENGTH+2];
volatile byte                   gbNewPacket;
volatile word                   gwZigbeeRxData;

extern int csf;

/* Add your variables and function declarations here ------------------------*/


/* Utility function declarations --------------------------------------------*/
byte SetMotorTargetPosition(byte, word);							// done
byte SetMotorTargetPositionsSync(byte[], word[], byte);
byte SetMotorTargetSpeed(byte, word);								// done
word GetMotorTargetPosition(byte);
word GetMotorCurrentPosition(byte);
byte GetIsMotorMoving(byte);
word GetSensorValue(byte);
byte SetMotorWheelSpeed(byte Motor_ID, word Direction, word speed);
byte SetWheelSpeedsSync(byte motor_IDs[], byte directions[], word targetspeeds[], byte idSize);
word GetMotorWheelSpeed(byte Motor_ID);
byte SetMotorMode(byte Motor_ID, byte Mode);

/* Configuration function declarations --------------------------------------------*/


void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void SysTick_Configuration(void);
void USART_Configuration(u8, u32);
void EnableZigbee(void);
void DisableZigbee(void);
void Timer_Configuration(void);
void ADC_Configuration(void);
void TxDByte_PC(byte);
void TxDByte_Zigbee(byte);
byte RxDByte_Zigbee(void);
void TxDString(char*);
void TxDWord16(word);
void TxDByte16(byte);
byte CheckNewArriveZigbee(void);
void mDelay(u32);
void uDelay(u32);
void EnterFSM(void);
void ProcessCommand(byte, byte);
void RxD2Interrupt(void);
void __ISR_DELAY(void);
void RxD0Interrupt(void);

/*******************************************************************************
* Function Name  : main
* Description    : Main program, place your behavior code in here.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
    /* System Configurations, all run once at startup (do not edit) */
	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	SysTick_Configuration();
	Timer_Configuration();
	ADC_Configuration();
	dxl_initialize( 0, 1 );
	zgb_initialize(0);
	USART_Configuration(USART_PC, Baudrate_PC);
    TxDByte_PC('x');

    TxDString("waiting for initial RDY from PC\r\n");
    while(1) {
    	if (zgb_rx_check() == 1) {
    		RcvData = zgb_rx_data();
    		TxDWord16(RcvData);
    		TxDString("\r\n");
    		TxDWord16(RDY==RcvData);
    		TxDString("\r\n");
    		if (RcvData==RDY) {
    			zgb_tx_data(ACK);
    			break;
    		}
    	}
    }

    /* Main control loop, add behavior code here */
    EnterFSM();

	return 0;
}

/******************************************************************************
 * Function Name	: EnterFSM
 * Description		: Main loop of the firmware's finite state machine
 * 					Receives command packet, send ack back, process command, then
 * 					let PC side know that it's ready to get
 * Input			: None
 * Output			: None
 * Return			: None
 ******************************************************************************/

void EnterFSM(void)
{
	TxDString("Entered Main Loop\r\n");
	while (1) {
		//send ready
		//TxDString("Sending RDY\r\n");
		//zgb_tx_data(RDY);
		//wait for a packet
		//TxDString("Waiting for a command\r\n");
		while(1) {
			if (zgb_rx_check()==1){
				RcvData = zgb_rx_data();
				//TxDString("Got something\r\n");
				lowb = RcvData & 0xff;
				highb = (RcvData >> 8) & 0xff;
				break;
			} else if (csf == -1) {
				TxDString("One of checksums failed!\r\n");
				TxDString("Requesting Retransmission(RTX)\r\n");
				zgb_tx_data(RTX);
				csf = 0;
			}
		}
		//process packet according to command in the packet (probably in another function)
		//TxDString("Received Command\r\n");
		//TxDByte_PC(highb);
		//TxDString("\r\n");
		ProcessCommand(highb, lowb);
	}
}

void ProcessCommand(byte command, byte detail)
{
	byte result, byte_ret;
	word word_ret;
	int i;
	int n;
	int flag = 0;
	//depending on command, do different stuff... probably should use switch-case
	switch(command) {
	//Set functions
	case 'P':
		//SetMotorTargetPosition
		//TxDString("Waiting for value\r\n");
		zgb_tx_data(ACK);
		while(1) {
			if(zgb_rx_check() == 1) {
				RcvData = zgb_rx_data();
				zgb_tx_data(ACK);
				break;
			}
		}
		result = SetMotorTargetPosition(detail, RcvData);
		break;

		// ALEX : OLD CODE IS COMMENTED OUT BELOW
		/*zgb_tx_data(ACK);
		while(1) {
			if(zgb_rx_check() == 1) {
				RcvData = zgb_rx_data();
				zgb_tx_data(ACK);
				break;
			}
		}
		result = SetMotorTargetPosition(detail, RcvData);
		break;*/
	case 'S':
		//TxDString("Waiting for value\r\n");
		//SetMotorTargetSpeed
		zgb_tx_data(ACK);
		while(1) {
			if(zgb_rx_check() == 1) {
				RcvData = zgb_rx_data();
				zgb_tx_data(ACK);
				break;
			}
		}
		result = SetMotorTargetSpeed(detail, RcvData);
		break;
	case 'C':
		//SetMotorTargetPositionsSync
		n = detail/2;
		if (n*2 != detail){
			n = n + 1;
		}
		zgb_tx_data(ACK);
		for (i = 0; i < n; i++) {
			while(1) {
				if (zgb_rx_check() == 1){
					RcvData = zgb_rx_data();
					zgb_tx_data(ACK);
					break;
				}
			}
			sync_ids[i*2] = (RcvData >> 8) & 0xff;
			sync_ids[i*2+1] = RcvData & 0xff;
		}
		n = detail;
		for (i = 0; i < n; i++) {
			while(1) {
				if (zgb_rx_check() == 1){
					RcvData = zgb_rx_data();
					zgb_tx_data(ACK);
					break;
				}
			}
			sync_pos[i] = RcvData;
		}
		result = SetMotorTargetPositionsSync(sync_ids, sync_pos, n);
		break;
	case 'M':
		//SetMotorMode
		zgb_tx_data(ACK);
		while(1) {
			if(zgb_rx_check() == 1) {
				RcvData = zgb_rx_data();
				lowb = RcvData & 0xff;
				zgb_tx_data(ACK);
				break;
			}
		}
		result = SetMotorMode(detail, lowb);
		break;
	case 'W':
		//SetMotorWheelSpeed
		zgb_tx_data(ACK);
		while(1) {
			if(zgb_rx_check() == 1) {
				RcvData = zgb_rx_data();
				zgb_tx_data(ACK);
				break;
			}
		}

		if(RcvData > 1023) //Direction: Clockwise
			result = SetMotorWheelSpeed(detail, 1, RcvData-1024);
		else               //Direction: Counter-clockwise
			result = SetMotorWheelSpeed(detail, 0, RcvData);
		break;
	case 'Y':
		//SetWheelSpeedSync
		n = detail/2;
		if (n*2 != detail){
			n = n + 1;
		}
		zgb_tx_data(ACK);
		for (i = 0; i < n; i++) {
			while(1) {
				if (zgb_rx_check() == 1){
					RcvData = zgb_rx_data();
					zgb_tx_data(ACK);
					break;
				}
			}
			sync_ids[i*2] = (RcvData >> 8) & 0xff;
			sync_ids[i*2+1] = RcvData & 0xff;
		}
		n = detail;
		for (i = 0; i < n; i++) {
			while(1) {
				if (zgb_rx_check() == 1){
					RcvData = zgb_rx_data();
					zgb_tx_data(ACK);
					break;
				}
			}
			if(RcvData > 1023) //Direction: Clockwise
			{
				sync_dirs[i] = 1;
				sync_speeds[i] = RcvData-1024;
			}
			else               //Direction: Counter-clockwise
			{
				sync_dirs[i] = 0;
				sync_speeds[i] = RcvData;
			}
		}
		result = SetWheelSpeedsSync(sync_ids, sync_dirs, sync_speeds, n);
		break;
	//Get functions
	case 'p':
		//GetMotorTargetPosition
		word_ret = GetMotorTargetPosition(detail);
		while(1) {
			zgb_tx_data(word_ret);
			for (i = 0; i < 3000; i++) {
				if (zgb_rx_check() == 1){
					RcvData = zgb_rx_data();
					if (RcvData == ACK) {
						flag = 1;
						break;
					}
				}
				uDelay(10);
			}
			if (flag)
				break;
		}
		break;
	case 'q':
		//GetCurrentPosition
		word_ret = GetMotorCurrentPosition(detail);
		while(1) {
			zgb_tx_data(word_ret);
			for (i = 0; i < 3000; i++) {
				if (zgb_rx_check() == 1){
					RcvData = zgb_rx_data();
					if (RcvData == ACK) {
						flag = 1;
						break;
					}
				}
				uDelay(10);
			}
			//if (flag)
				break;
		}
		break;
	case 'm':
		//GetIsMotorMoving
		byte_ret = GetIsMotorMoving(detail);
		while(1) {
			zgb_tx_data(byte_ret);
			for (i = 0; i < 3000; i++) {
				if (zgb_rx_check() == 1){
					RcvData = zgb_rx_data();
					if (RcvData == ACK) {
						flag = 1;
						break;
					}
				}
				uDelay(10);
			}
			if (flag)
				break;
		}
		break;
	case 'v':
		//GetSensorValue
		word_ret = GetSensorValue(detail);
		while(1) {
			zgb_tx_data(word_ret);
			for (i = 0; i < 3000; i++) {
				// TxDString("Checking for ACK\r\n");
				if (zgb_rx_check() == 1){
					RcvData = zgb_rx_data();
					if (RcvData == ACK) {
						// TxDString("Received ACK: \r\n");
						// TxDByte_PC(i);
						flag = 1;
						break;
					}
				}
				uDelay(10);
			}
			if (flag)
				break;
		}
		break;
	case 's':
		//GetMotorWheelSpeed
		word_ret = GetMotorWheelSpeed(detail);
		while(1) {
			zgb_tx_data(word_ret);
			for (i = 0; i < 3000; i++) {
				if (zgb_rx_check() == 1){
					RcvData = zgb_rx_data();
					if (RcvData == ACK) {
						flag = 1;
						break;
					}
				}
				uDelay(10);
			}
			if (flag)
				break;
		}
		break;
	default:
		//unrecognized command
		TxDString("an ack was lost... resend\r\n");
		zgb_tx_data(ACK);
		break;
	}
}


/*******************************************************************************
* Function Name  : SetMotorTargetPosition
* Description    : Sets the value for the target position of the motor
* Input          : Motor_ID: ID of the dynamixel motor to set, dependent on configuration
*                : targetPos: The raw value of the target position to set the motor to (0-1023).
* Output         : None
* Return         : COMMAND_SUCCESS if successful, COMMUNICATION_ERROR if unsuccessful
*******************************************************************************/
byte SetMotorTargetPosition(byte Motor_ID, word targetPos)
{
	byte result;
	dxl_write_word( Motor_ID, P_GOAL_POSITION_L, targetPos);
	if (dxl_get_result() == COMM_RXSUCCESS)
	{
	    result = COMMAND_SUCCESS;
	}
	else
	{
		result = COMMUNICATION_ERROR;
	}

	return result;
}
/*******************************************************************************
* Function Name  : SetMotorTargetPositionsSync
* Description    : Sets the target position value of each id in the motor_IDs array, to the corresponding
*                  position of the motor in the targetPositions array
* Input          : motor_IDs: IDs of the dynamixel motors to set, dependent on configuration
*                  targetPositions: Array of raw values of the target positions to set the motors to (0-1023).
* Output         : None
* Return         : COMMAND_SUCCESS if successful, COMMUNICATION_ERROR if unsuccessful,
*                  ARRAY_SIZE_MISMATCH if input array sizes don't match.
*******************************************************************************/
byte SetMotorTargetPositionsSync(byte motor_IDs[], word targetPositions[], byte idSize)
{
	byte result;

    // Make syncwrite packet
    dxl_set_txpacket_id(BROADCAST_ID);
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	dxl_set_txpacket_parameter(0, P_GOAL_POSITION_L);
	dxl_set_txpacket_parameter(1, 2);

	byte loopCount = 0;
	while(loopCount<idSize)
	{
		dxl_set_txpacket_parameter(2+3*loopCount, motor_IDs[loopCount]);
		dxl_set_txpacket_parameter(2+3*loopCount+1, dxl_get_lowbyte(targetPositions[loopCount]));
		dxl_set_txpacket_parameter(2+3*loopCount+2, dxl_get_highbyte(targetPositions[loopCount]));

		loopCount++;
	}

	dxl_set_txpacket_length((2+1)*idSize+4);

	dxl_txrx_packet();

	if (dxl_get_result() == COMM_RXSUCCESS)
	{
		result = COMMAND_SUCCESS;
	}
	else
	{
		result = COMMUNICATION_ERROR;
	}

	return result;
}
/*******************************************************************************
* Function Name  : SetMotorTargetSpeed
* Description    : Sets the value for the target speed of the motor
* Input          : Motor_ID: ID of the dynamixel motor to set, dependent on configuration
*                : Speed_Val: The raw value of the motor speed (0-1023, 0 is max).
* Output         : None
* Return         : COMMAND_SUCCESS if successful, COMMUNICATION_ERROR if unsuccessful
*******************************************************************************/
byte SetMotorTargetSpeed(byte Motor_ID, word Speed_Val)
{
	byte result;
	dxl_write_word( Motor_ID, P_GOAL_SPEED_L, Speed_Val);
	if (dxl_get_result() == COMM_RXSUCCESS)
	{
	    result = COMMAND_SUCCESS;
	}
	else
	{
		result = COMMUNICATION_ERROR;
	}

	return result;
}
/*******************************************************************************
* Function Name  : GetMotorTargetPosition
* Description    : Gets the value for the target position of the motor
* Input          : Motor_ID: ID of the dynamixel motor to set, dependent on configuration
* Output         : None
* Return         : The raw target position value (0-1023), -1 if there is an error.
*******************************************************************************/
word GetMotorTargetPosition(byte Motor_ID)
{
	word result;
	result = dxl_read_word( Motor_ID, P_GOAL_POSITION_L);
	if (dxl_get_result() != COMM_RXSUCCESS)
	{
	    result = COMMUNICATION_ERROR;
	}

	return result;
}
/*******************************************************************************
* Function Name  : GetMotorCurrentPosition
* Description    : Gets the value for the current position of the motor
* Input          : Motor_ID: ID of the dynamixel motor to set, dependent on configuration
* Output         : None
* Return         : The raw current position value (0-1023), -1 if there is an error.
*******************************************************************************/
word GetMotorCurrentPosition(byte Motor_ID)
{
	word result;
	result = dxl_read_word( Motor_ID, P_PRESENT_POSITION_L);
	if (dxl_get_result() != COMM_RXSUCCESS)
	{
	    result = COMMUNICATION_ERROR;
	}

	return result;
}
/*******************************************************************************
* Function Name  : GetIsMotorMoving
* Description    : Gets the value for the current position of the motor
* Input          : Motor_ID: ID of the dynamixel motor to set, dependent on configuration
* Output         : None
* Return         : 0 if the motor is not moving, 1 if the motor is moving
*******************************************************************************/
byte GetIsMotorMoving(byte Motor_ID)
{
	byte result;
	result = dxl_read_byte( Motor_ID, P_MOVING);
	if (dxl_get_result() != COMM_RXSUCCESS)
	{
	    result = COMMUNICATION_ERROR;
	}

	return result;
}
/*******************************************************************************
* Function Name  : GetSensorValue
* Description    : Retrieves the value from the sensor on a given IO port
* Input          : ADC_Port_ID: the port number the sensor is attached to (1 to 6)
* Output         : None
* Return         : The raw sensor value (0-1023)
*******************************************************************************/
word GetSensorValue(byte ADC_PIN_ID)
{
	word Sensor_Val = 0;

	switch(ADC_PIN_ID)
	{
		case 1:
			GPIO_SetBits(PORT_SIG_MOT1P, PIN_SIG_MOT1P);
			GPIO_ResetBits(PORT_SIG_MOT1M, PIN_SIG_MOT1M);
			break;
		case 2:
			GPIO_SetBits(PORT_SIG_MOT2P, PIN_SIG_MOT2P);
			GPIO_ResetBits(PORT_SIG_MOT2M, PIN_SIG_MOT2M);
			break;
		case 3:
			GPIO_SetBits(PORT_SIG_MOT3P, PIN_SIG_MOT3P);
			GPIO_ResetBits(PORT_SIG_MOT3M, PIN_SIG_MOT3M);
			break;
		case 4:
			GPIO_SetBits(PORT_SIG_MOT4P, PIN_SIG_MOT4P);
			GPIO_ResetBits(PORT_SIG_MOT4M, PIN_SIG_MOT4M);
			break;
		case 5:
			GPIO_SetBits(PORT_SIG_MOT5P, PIN_SIG_MOT5P);
			GPIO_ResetBits(PORT_SIG_MOT5M, PIN_SIG_MOT5M);
			break;
		case 6:
			GPIO_SetBits(PORT_SIG_MOT6P, PIN_SIG_MOT6P);
			GPIO_ResetBits(PORT_SIG_MOT6M, PIN_SIG_MOT6M);
			break;
	}

	switch(ADC_PIN_ID)
	{
		case 1:
			GPIO_ResetBits(PORT_ADC_SELECT0,PIN_ADC_SELECT0);
		    GPIO_ResetBits(PORT_ADC_SELECT1,PIN_ADC_SELECT1);
			break;
		case 2:
			GPIO_SetBits(PORT_ADC_SELECT0,PIN_ADC_SELECT0);
			GPIO_ResetBits(PORT_ADC_SELECT1,PIN_ADC_SELECT1);
			break;
		case 3:
			GPIO_ResetBits(PORT_ADC_SELECT0,PIN_ADC_SELECT0);
			GPIO_SetBits(PORT_ADC_SELECT1,PIN_ADC_SELECT1);
			break;
		case 4:
			GPIO_ResetBits(PORT_ADC_SELECT0,PIN_ADC_SELECT0);
			GPIO_ResetBits(PORT_ADC_SELECT1,PIN_ADC_SELECT1);
			break;
		case 5:
			GPIO_SetBits(PORT_ADC_SELECT0,PIN_ADC_SELECT0);
			GPIO_ResetBits(PORT_ADC_SELECT1,PIN_ADC_SELECT1);
			break;
		case 6:
			GPIO_ResetBits(PORT_ADC_SELECT0,PIN_ADC_SELECT0);
			GPIO_SetBits(PORT_ADC_SELECT1,PIN_ADC_SELECT1);
			break;
	}

	uDelay(30);

	/* Start ADC1,ADC2 Software Conversion */
	switch(ADC_PIN_ID)
	{
		case 1:
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
			break;
		case 2:
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
			break;
		case 3:
			ADC_SoftwareStartConvCmd(ADC1, ENABLE);
			break;
		case 4:
			ADC_SoftwareStartConvCmd(ADC2, ENABLE);
			break;
		case 5:
			ADC_SoftwareStartConvCmd(ADC2, ENABLE);
			break;
		case 6:
			ADC_SoftwareStartConvCmd(ADC2, ENABLE);
			break;
	}

	uDelay(5);

	switch(ADC_PIN_ID)
	{
		case 1:
			Sensor_Val = (ADC_GetConversionValue(ADC1));
			break;
		case 2:
			Sensor_Val = (ADC_GetConversionValue(ADC1));
			break;
		case 3:
			Sensor_Val = (ADC_GetConversionValue(ADC1));
			break;
		case 4:
			Sensor_Val = (ADC_GetConversionValue(ADC2));
			break;
		case 5:
			Sensor_Val = (ADC_GetConversionValue(ADC2));
			break;
		case 6:
			Sensor_Val = (ADC_GetConversionValue(ADC2));
			break;
	}

	switch(ADC_PIN_ID)
	{
		case 1:
			GPIO_ResetBits(PORT_SIG_MOT1P, PIN_SIG_MOT1P);
			GPIO_ResetBits(PORT_SIG_MOT1M, PIN_SIG_MOT1M);
			break;
		case 2:
			GPIO_ResetBits(PORT_SIG_MOT2P, PIN_SIG_MOT2P);
			GPIO_ResetBits(PORT_SIG_MOT2M, PIN_SIG_MOT2M);
			break;
		case 3:
			GPIO_ResetBits(PORT_SIG_MOT3P, PIN_SIG_MOT3P);
			GPIO_ResetBits(PORT_SIG_MOT3M, PIN_SIG_MOT3M);
			break;
		case 4:
			GPIO_ResetBits(PORT_SIG_MOT4P, PIN_SIG_MOT4P);
			GPIO_ResetBits(PORT_SIG_MOT4M, PIN_SIG_MOT4M);
			break;
		case 5:
			GPIO_ResetBits(PORT_SIG_MOT5P, PIN_SIG_MOT5P);
			GPIO_ResetBits(PORT_SIG_MOT5M, PIN_SIG_MOT5M);
			break;
		case 6:
			GPIO_ResetBits(PORT_SIG_MOT6P, PIN_SIG_MOT6P);
			GPIO_ResetBits(PORT_SIG_MOT6M, PIN_SIG_MOT6M);
			break;
	}

	return Sensor_Val;
}

/*******************************************************************************
* Function Name  : SetMotorMode
* Description    : Sets the mode for the target speed of the motor between Joint and Wheel Mode
* Input          : Motor_ID: ID of the dynamixel motor to set, dependent on configuration
*                : Mode: 0 is Joint Mode, 1 is Wheel Mode
* Output         : None
* Return         : COMMAND_SUCCESS if successful, COMMUNICATION_ERROR if unsuccessful
*******************************************************************************/
byte SetMotorMode(byte Motor_ID, byte Mode)
{
	byte result;
	word ccw_limit = 1023;
    if(Mode == 1)
	{
		ccw_limit = 0;
	}

	dxl_write_word( Motor_ID, P_CCW_ANGLE_LIMIT_L, ccw_limit);
	if (dxl_get_result() == COMM_RXSUCCESS)
	{
	    result = COMMAND_SUCCESS;
	}
	else
	{
		result = COMMUNICATION_ERROR;
	}

	return result;
}

//Direction: ccw = 0, cw = 1
/*******************************************************************************
* Function Name  : SetMotorWheelSpeed
* Description    : When in wheel mode, this sets the rotational speed, via
*                : controlling torque, as well as the direction of rotation
* Input          : Motor_ID: ID of the dynamixel motor to set, dependent on configuration
*                : Direction: 0 is CCW, 1 is CW
*                : Speed: The speed, a number between 0-1023
* Output         : None
* Return         : COMMAND_SUCCESS if successful, COMMUNICATION_ERROR if unsuccessful
*******************************************************************************/
byte SetMotorWheelSpeed(byte Motor_ID, word Direction, word speed)
{
	byte result;
	word raw_speed;
    if(Direction == 0)
	{
    	raw_speed = speed;
	}
    else
    {
    	raw_speed = speed + 1024;
    }

	dxl_write_word( Motor_ID, P_GOAL_SPEED_L, raw_speed);
	if (dxl_get_result() == COMM_RXSUCCESS)
	{
	    result = COMMAND_SUCCESS;
	}
	else
	{
		result = COMMUNICATION_ERROR;
	}

	return result;
}

//Direction: ccw = 0, cw = 1
/*******************************************************************************
* Function Name  : GetMotorWheelSpeed

* Description    : When in wheel mode, this gets the current rotational speed, via
*                : controlling torque, as well as the direction of rotation
* Input          : Motor_ID: ID of the dynamixel motor to get speed, dependent on configuration
* Output         : NONE
* Return         : 0-1023 CCW or 1024-2047 CW, COMMUNICATION_ERROR if unsuccessful

*******************************************************************************/
word GetMotorWheelSpeed(byte Motor_ID)
{
	word result;
	result = dxl_read_word( Motor_ID, P_PRESENT_SPEED_L);
	if (dxl_get_result() != COMM_RXSUCCESS)
	{
	    result = COMMUNICATION_ERROR;
	}

	return result;
}

/*******************************************************************************
* Function Name  : SetWheelSpeedsSync
* Description    : Sets the target position value of each id in the motor_IDs array, to the corresponding
*                  position of the motor in the targetPositions array
* Input          : motor_IDs: IDs of the dynamixel motors to set, dependent on configuration
*                  targetPositions: Array of raw values of the target positions to set the motors to (0-1023).
* Output         : None
* Return         : COMMAND_SUCCESS if successful, COMMUNICATION_ERROR if unsuccessful,
*                  ARRAY_SIZE_MISMATCH if input array sizes don't match.
*******************************************************************************/
byte SetWheelSpeedsSync(byte motor_IDs[], byte directions[], word targetspeeds[], byte idSize)
{
	byte result;

    // Make syncwrite packet
    dxl_set_txpacket_id(BROADCAST_ID);
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	dxl_set_txpacket_parameter(0, P_GOAL_SPEED_L);
	dxl_set_txpacket_parameter(1, 2);

	byte loopCount = 0;
	while(loopCount<idSize)
	{
	    if(directions[loopCount] == 1)
		{
	    	targetspeeds[loopCount] = targetspeeds[loopCount] + 1024;
		}

		dxl_set_txpacket_parameter(2+3*loopCount, motor_IDs[loopCount]);
		dxl_set_txpacket_parameter(2+3*loopCount+1, dxl_get_lowbyte(targetspeeds[loopCount]));
		dxl_set_txpacket_parameter(2+3*loopCount+2, dxl_get_highbyte(targetspeeds[loopCount]));

		loopCount++;
	}

	dxl_set_txpacket_length((2+1)*idSize+4);

	dxl_txrx_packet();

	if (dxl_get_result() == COMM_RXSUCCESS)
	{
		result = COMMAND_SUCCESS;
	}
	else
	{
		result = COMMUNICATION_ERROR;
	}

	return result;
}

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while(RCC_GetSYSCLKSource() != 0x08)
		{
		}
	}

	/* Enable peripheral clocks --------------------------------------------------*/

	/* Enable GPIOA, GPIOB, GPIOC and ADC1 clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
			               RCC_APB2Periph_GPIOD | RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);

	/* Enable USART3 clocks */
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_UART5 | RCC_APB1Periph_USART3 | RCC_APB1Periph_TIM2, ENABLE);

	PWR_BackupAccessCmd(ENABLE);
}
/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	#ifdef  VECT_TAB_RAM
		// Set the Vector Table base location at 0x20000000
		NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	#else  // VECT_TAB_FLASH
		// Set the Vector Table base location at 0x08003000
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);
	#endif

	// Configure the NVIC Preemption Priority Bits
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	// Enable the USART1 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Enable the TIM2 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	// PORTA CONFIG
	GPIO_InitStructure.GPIO_Pin = 	 PIN_SIG_MOT1P | PIN_SIG_MOT1M | PIN_SIG_MOT2P | PIN_SIG_MOT2M | PIN_SIG_MOT5P | PIN_SIG_MOT5M | PIN_ZIGBEE_RESET;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  PIN_ADC1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// PORTB CONFIG
	GPIO_InitStructure.GPIO_Pin = 	PIN_ENABLE_TXD | PIN_ENABLE_RXD | PIN_SIG_MOT6P | PIN_SIG_MOT6M;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DXL_RXD | PIN_PC_RXD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DXL_TXD | PIN_PC_TXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PORTC CONFIG
	GPIO_InitStructure.GPIO_Pin =   PIN_ADC_SELECT0 | PIN_ADC_SELECT1 |  PIN_LED_POWER | PIN_SIG_MOT3P | PIN_SIG_MOT3M | PIN_SIG_MOT4P | PIN_SIG_MOT4M;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  PIN_ADC0 | PIN_VDD_VOLT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  PIN_ZIGBEE_TXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// PORTD CONFIG
	GPIO_InitStructure.GPIO_Pin = PIN_ZIGBEE_RXD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_PinRemapConfig( GPIO_Remap_USART1, ENABLE);
	GPIO_PinRemapConfig( GPIO_Remap_SWJ_Disable, ENABLE);

	GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
	GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable

}
void USART1_Configuration(u32 baudrate)
{
	USART_Configuration(USART_DXL, baudrate);
}
void USART_Configuration(u8 PORT, u32 baudrate)
{

	USART_InitTypeDef USART_InitStructure;

	USART_StructInit(&USART_InitStructure);


	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


	if( PORT == USART_DXL )
	{
		USART_DeInit(USART1);
		uDelay(10);
		/* Configure the USART1 */
		USART_Init(USART1, &USART_InitStructure);

		/* Enable USART1 Receive and Transmit interrupts */
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		//USART_ITConfig(USART1, USART_IT_TC, ENABLE);

		/* Enable the USART1 */
		USART_Cmd(USART1, ENABLE);
	}

	else if( PORT == USART_PC )
	{
		USART_DeInit(USART3);
		uDelay(10);
		/* Configure the USART3 */
		USART_Init(USART3, &USART_InitStructure);

		/* Enable USART3 Receive and Transmit interrupts */
		//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		//USART_ITConfig(USART3, USART_IT_TC, ENABLE);

		/* Enable the USART3 */
		USART_Cmd(USART3, ENABLE);
	}

	else if( PORT == USART_ZIGBEE )
	{
		TxDString("usart_config\r\n");
		USART_DeInit(UART5);
		uDelay(10);
		/* Configure the UART5 */
		USART_Init(UART5, &USART_InitStructure);

		/* Enable UART5 Receive and Transmit interrupts */
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

		/* Enable the UART5 */
		USART_Cmd(UART5, ENABLE);
	}
}

void EnableZigbee(void)
{
	USART_Configuration(USART_ZIGBEE, 57600);
	GPIO_ResetBits(PORT_ZIGBEE_RESET, PIN_ZIGBEE_RESET);
}

void DisableZigbee(void)
{
	USART_Cmd(UART5, DISABLE);
	GPIO_SetBits(PORT_ZIGBEE_RESET, PIN_ZIGBEE_RESET);
}

void DisableUSART1(void)
{
	USART_Cmd(USART1, DISABLE);
}
void ClearBuffer256(void)
{
	gbRxBufferReadPointer = gbRxBufferWritePointer = 0;
}
byte CheckNewArrive(void)
{
	if(gbRxBufferReadPointer != gbRxBufferWritePointer){
		return 1;
	}
	else
		return 0;
}
byte CheckNewArriveZigbee(void)
{
	if (gbPacketReadPointer != gbPacketWritePointer) {
		return 1;
	}
	else
		return 0;
}
void TxDByte_DXL(byte bTxdData)
{
	GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Disable
	GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Enable

	USART_SendData(USART1,bTxdData);
	while( USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET );

	GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
	GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable
}
byte RxDByte_DXL(void)
{
	byte bTemp;

	while(1)
	{
		if(gbRxBufferReadPointer != gbRxBufferWritePointer) break;
	}

	bTemp = gbpRxInterruptBuffer[gbRxBufferReadPointer];
	gbRxBufferReadPointer++;

	return bTemp;
}

void TxDByte_Zigbee(byte bTxdData)
{
	USART_SendData(UART5,bTxdData);
	while( USART_GetFlagStatus(UART5, USART_FLAG_TC)==RESET );
}

byte RxDByte_Zigbee(void)
{
	byte bTemp;

	while(1)
	{
		if(gbPacketReadPointer != gbPacketWritePointer) break;
	}

	bTemp = gbpPacketDataBuffer[gbPacketReadPointer];
	gbPacketReadPointer++;
	gbPacketReadPointer = gbPacketReadPointer & 0x1F;
	return bTemp;
}

// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
	case COMM_TXFAIL:
		TxDString("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;

	case COMM_TXERROR:
		TxDString("COMM_TXERROR: Incorrect instruction packet!\n");
		break;

	case COMM_RXFAIL:
		TxDString("COMM_RXFAIL: Failed get status packet from device!\n");
		break;

	case COMM_RXWAITING:
		TxDString("COMM_RXWAITING: Now recieving status packet!\n");
		break;

	case COMM_RXTIMEOUT:
		TxDString("COMM_RXTIMEOUT: There is no status packet!\n");
		break;

	case COMM_RXCORRUPT:
		TxDString("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;

	default:
		TxDString("This is unknown error code!\n");
		break;
	}
}
// Print error bit of status packet
void PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		TxDString("Input voltage error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		TxDString("Angle limit error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		TxDString("Overheat error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		TxDString("Out of range error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		TxDString("Checksum error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		TxDString("Overload error!\n");

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		TxDString("Instruction code error!\n");
}
void TxDString(char *bData)
{
	while (*bData)
		TxDByte_PC(*bData++);
}
void TxDWord16(word wSentData)
{
	TxDByte16((wSentData >> 8) & 0xff);
	TxDByte16(wSentData & 0xff);
}
void TxDByte16(byte bSentData)
{
	byte bTmp;

	bTmp = ((byte) (bSentData >> 4) & 0x0f) + (byte) '0';
	if (bTmp > '9')
		bTmp += 7;
	TxDByte_PC(bTmp);
	bTmp = (byte) (bSentData & 0x0f) + (byte) '0';
	if (bTmp > '9')
		bTmp += 7;
	TxDByte_PC(bTmp);
}
void TxDByte_PC(byte bTxdData)
{
	USART_SendData(USART3,bTxdData);
	while( USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET );
}
void Timer_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_DeInit(TIM2);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM2, 722, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val ;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}
void TimerInterrupt_1ms(void) //OLLO CONTROL
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) // 1ms//
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

		capture = TIM_GetCapture1(TIM2);
		TIM_SetCompare1(TIM2, capture + CCR1_Val);

		if(gw1msCounter > 0)
			gw1msCounter--;
	}
}
/*__interrupt*/
void RxD0Interrupt(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
		gbpRxInterruptBuffer[gbRxBufferWritePointer++] = USART_ReceiveData(USART1);
}
void SysTick_Configuration(void)
{
	  /* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */
	  SysTick_SetReload(9);

	  /* Enable SysTick interrupt */
	  SysTick_ITConfig(ENABLE);
}
void __ISR_DELAY(void)
{
	if (gwTimingDelay != 0x00)
		gwTimingDelay--;
}
void mDelay(u32 nTime)
{
	uDelay(nTime*1000);
}
void uDelay(u32 nTime)
{
	  /* Enable the SysTick Counter */
	  SysTick_CounterCmd(SysTick_Counter_Enable);

	  gwTimingDelay = nTime;

	  while(gwTimingDelay != 0);

	  /* Disable SysTick Counter */
	  SysTick_CounterCmd(SysTick_Counter_Disable);
	  /* Clear SysTick Counter */
	  SysTick_CounterCmd(SysTick_Counter_Clear);
}
void StartDiscount(s32 StartTime)
{
	gw1msCounter = StartTime;
}
u8 CheckTimeOut(void)
{
	// Check timeout
	// Return: 0 is false, 1 is true(timeout occurred)

	if(gw1msCounter == 0)
		return 1;
	else
		return 0;
}
void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure1;
	ADC_InitTypeDef ADC_InitStructure2;
	ADC_StructInit(&ADC_InitStructure1);
/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure1.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure1.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure1.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure1.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure1.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure1.ADC_NbrOfChannel = 1;

	ADC_Init(ADC1, &ADC_InitStructure1);
	/* ADC1 regular channels configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1 , ADC_SampleTime_239Cycles5);
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	/* Enable ADC1 reset calibration register */
	/* Check the end of ADC1 reset calibration register */
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	/* Start ADC1 calibration */
	/* Check the end of ADC1 calibration */
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);


	ADC_StructInit(&ADC_InitStructure2);
/* ADC2 configuration ------------------------------------------------------*/
	ADC_InitStructure2.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure2.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure2.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure2.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure2.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure2.ADC_NbrOfChannel = 1;

	ADC_Init(ADC2, &ADC_InitStructure2);
	/* ADC2 regular channels configuration */
	ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5);
	/* Enable ADC2 */
	ADC_Cmd(ADC2, ENABLE);
	/* Enable ADC2 reset calibration register */
	/* Check the end of ADC2 reset calibration register */
	ADC_ResetCalibration(ADC2);
	while(ADC_GetResetCalibrationStatus(ADC2));
	/* Start ADC2 calibration */
	/* Check the end of ADC2 calibration */
	ADC_StartCalibration(ADC2);
	while(ADC_GetCalibrationStatus(ADC2));
	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConvCmd(ADC2, ENABLE);
}

void RxD2Interrupt(void)
{
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		word temp;
		temp = USART_ReceiveData(UART5);
		gbpPacketDataBuffer[gbPacketWritePointer] = temp;
		gbPacketWritePointer++;
		gbPacketWritePointer = gbPacketWritePointer & 0x1F;
	}
}

