/*
 * map.h
 *
 *  Created on: Jan 23, 2013
 *      Author: Matt
 */

#ifndef MAP_H
#define MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#define word 			u16
#define byte 			u8

typedef enum {
	North,
	South,
	East,
	West
} DIRECTION;

byte getNeighborObstacle(byte i, byte j, DIRECTION dir);
byte setObstacle(byte i, byte j, byte isBlocked, DIRECTION dir);
word getNeighborCost(byte i, byte j, DIRECTION dir);
byte setCost(byte i, byte j, word val);
void clearCostMap();
void clearObstacleMap();
void printCostMap();
void printObstacleMap();

#ifdef __cplusplus
}
#endif

#endif /* MAP_H */
