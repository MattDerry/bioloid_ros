/*
 * map.c
 *
 *  Created on: Jan 23, 2013
 *  Author: Matt Derry
 *
 *  File for performing simple path planning and map building for EECS 295
 */
#include "stm32f10x_lib.h"
#include "map.h"
#include "main.h"

#define word 			u16
#define byte 			u8

byte horizontalWalls[9][8] = { {1, 1, 1, 1, 1, 1, 1, 1},
		                       {0, 0, 0, 0, 0, 0, 1, 0},
		                       {0, 0, 1, 1, 1, 0, 0, 0},
		                       {0, 1, 1, 0, 0, 0, 1, 1},
		                       {0, 1, 1, 1, 1, 0, 1, 1},
		                       {0, 0, 1, 1, 1, 0, 0, 0},
		                       {0, 0, 0, 0, 0, 0, 0, 0},
		                       {0, 1, 1, 0, 1, 1, 1, 0},
		                       {1, 1, 1, 1, 1, 1, 1, 1}
                             };

byte verticalWalls[8][9] = {   {1, 0, 1, 0, 0, 0, 1, 0, 1},
		                       {1, 1, 1, 0, 0, 1, 0, 1, 1},
		                       {1, 1, 0, 0, 0, 0, 0, 0, 1},
		                       {1, 0, 0, 0, 0, 0, 1, 0, 1},
		                       {1, 1, 0, 0, 0, 0, 1, 0, 1},
		                       {1, 1, 1, 0, 0, 1, 1, 1, 1},
		                       {1, 1, 0, 1, 1, 1, 0, 1, 1},
		                       {1, 0, 0, 0, 0, 0, 0, 1, 1}
                             };

word costMap[8][8] = { {0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0, 0, 0}
                     };

/*******************************************************************************
* Function Name  : getNeighborObstacle
* Description    : Checks if the neighboring cell is blocked on the map.
* Input          : i: The row coordinate of the current cell on the map.
*                : j: The column coordinate of the current cell on the map
*                : dir: A Direction enumeration (North, South, East, West) indicating
*                :      which neighboring cell to check for obstacles
* Output         : None
* Return         : 1 if neighboring cell is blocked, 0 if neighboring cell is clear
*******************************************************************************/
byte getNeighborObstacle(byte i, byte j, DIRECTION dir)
{
	byte isBlocked = 0;

	switch(dir)
	{
	    case North:
           isBlocked = horizontalWalls[i][j];
		   break;
	    case South:
           isBlocked = horizontalWalls[i+1][j];
		   break;
	    case West:
		   isBlocked = verticalWalls[i][j];
		   break;
	    case East:
		   isBlocked = verticalWalls[i][j+1];
		   break;
	}

	return isBlocked;
}

/*******************************************************************************
* Function Name  : setObstacle
* Description    : Used for map building, sets the obstacle status of a given map cell
* Input          : i: The row coordinate of the current cell on the map.
*                : j: The column coordinate of the current cell on the map
*                : isBlocked: A boolean (0 or 1) value indicated if the cell is blocked
*                : dir: A Direction enumeration (North, South, East, West) indicating
*                :      which neighboring cell to set for obstacles
* Output         : None
* Return         : 0 if successful, 1 if i or j is out of map bounds, 2 if isBlocked is not 0 or 1
*******************************************************************************/
byte setObstacle(byte i, byte j, byte isBlocked, DIRECTION dir)
{
	if(((i > 7 || j > 8) && (dir == West || dir == East)) || ((j > 7 || i > 8) && (dir == North || dir == South)))
	{
		return 1;
	}

	if(isBlocked > 1)
	{
		return 2;
	}

	switch(dir)
	{
	    case North:
           horizontalWalls[i][j] = isBlocked;
		   break;
	    case South:
           horizontalWalls[i+1][j] = isBlocked;
		   break;
	    case West:
		   verticalWalls[i][j] = isBlocked;
		   break;
	    case East:
		   verticalWalls[i][j+1] = isBlocked;
		   break;
	}

	return 0;
}

/*******************************************************************************
* Function Name  : getNeighborCost
* Description    : Retrieves the calculated cost of a neighboring cell on the map.
* Input          : i: The row coordinate of the current cell on the map.
*                : j: The column coordinate of the current cell on the map
*                : dir: A Direction enumeration (North, South, East, West) indicating
*                :      which neighboring cell to retrieve the cost.
* Output         : None
* Return         : Integer cost (0 to 1023) for the neighboring cell
*******************************************************************************/
word getNeighborCost(byte i, byte j, DIRECTION dir)
{
	word cellValue = 0;

	switch(dir)
	{
	    case North:
		   if(i == 0) { cellValue = 1; }
		   else { cellValue = costMap[i-1][j]; }
		   break;
	    case South:
		   if(i == 7) { cellValue = 1; }
	       else { cellValue = costMap[i+1][j]; }
		   break;
	    case West:
		   if(j == 0) { cellValue = 1; }
		   else { cellValue = costMap[i][j-1]; }
		   break;
	    case East:
		   if(j == 7) { cellValue = 1; }
		   else { cellValue = costMap[i][j+1]; }
		   break;
	}

	return cellValue;
}

/*******************************************************************************
* Function Name  : setCost
* Description    : Used for map building, sets the calculated cost of a given map cell
* Input          : i: The row coordinate of the current cell on the map.
*                : j: The column coordinate of the current cell on the map
*                : val: An integer value (0 to 1023) indicated the cost of a map cell
* Output         : None
* Return         : 0 if successful, 1 if i or j is out of map bounds
*******************************************************************************/
byte setCost(byte i, byte j, word val)
{
	if(i > 7 || j > 7)
	{
		return 1;
	}

	costMap[i][j] = val;

	return 0;
}

/*******************************************************************************
* Function Name  : clearCostMap
* Description    : Sets all of the values in the cost map to 0
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void clearCostMap()
{
	byte i, j;
	for(i = 0; i < 8; i++)
	{
		for(j = 0; j < 8; j++)
		{
			costMap[i][j] = 0;
		}
	}
}

/*******************************************************************************
* Function Name  : clearObstacleMap
* Description    : Sets all of the values in the obstacle map to 0
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void clearObstacleMap()
{
	byte i, j;
	for(i = 0; i < 8; i++)
	{
		for(j = 0; j < 9; j++)
		{
			verticalWalls[i][j] = 0;
		}
	}

	for(i = 0; i < 9; i++)
	{
		for(j = 0; j < 8; j++)
		{
			horizontalWalls[i][j] = 0;
		}
	}
}

/*******************************************************************************
* Function Name  : printCostMap
* Description    : When connected to a terminal, will print out the 8x8 cost map
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void printCostMap()
{
	byte i, j;
	TxDString("Cost Map:\r\n");
	for(i = 0; i < 8; i++)
	{
		for(j = 0; j < 8; j++)
		{
			TxDWord16(costMap[i][j]);
			TxDByte_PC(' ');
		}
		TxDByte_PC('\r');
		TxDByte_PC('\n');
	}

}

/*******************************************************************************
* Function Name  : printObstacleMap
* Description    : When connected to a terminal, will print out the 8x8 obstacle map
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void printObstacleMap()
{
	byte i, j;
	TxDString("Obstacle Map:\r\n");

	for(i = 0; i < 8; i++)
	{
		for(j = 0; j < 8; j++)
		{
			if(horizontalWalls[i][j] == 0)
			{
			   TxDByte_PC(' ');
			   TxDByte_PC(' ');
			   TxDByte_PC(' ');
			   TxDByte_PC(' ');
			}
			else
			{
			   TxDByte_PC(' ');
			   TxDByte_PC('-');
			   TxDByte_PC('-');
			   TxDByte_PC('-');
			}
		}
		TxDByte_PC('\r');
		TxDByte_PC('\n');

		for(j = 0; j < 8; j++)
		{
			if(verticalWalls[i][j] == 0)
			{
				TxDByte_PC(' ');
				TxDByte_PC(' ');
				TxDByte_PC('O');
				TxDByte_PC(' ');
			}
			else
			{
				TxDByte_PC('|');
				TxDByte_PC(' ');
				TxDByte_PC('O');
				TxDByte_PC(' ');
			}

		}
		TxDByte_PC('|');
		TxDByte_PC('\r');
		TxDByte_PC('\n');
	}

	for(j = 0; j < 8; j++)
	{
		if(horizontalWalls[8][j] == 0)
		{
		   TxDByte_PC(' ');
		   TxDByte_PC(' ');
		   TxDByte_PC(' ');
		   TxDByte_PC(' ');
		}
		else
		{
		   TxDByte_PC(' ');
		   TxDByte_PC('-');
		   TxDByte_PC('-');
		   TxDByte_PC('-');
		}
	}

	TxDByte_PC('\r');
    TxDByte_PC('\n');
}



