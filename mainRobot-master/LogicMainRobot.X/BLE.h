/**
 * @file BLE.h
 * @author bULBot 2015-2016
 * @author Denis Thomas
 * @ 
 *
 */
 
#ifndef DEF_BLE_H
#define DEF_BLE_H 
#include "dataTypes.h"

void initBLE(void);
void updateData(char* data);
void updateTaskAction(char* data);
int isTaskActionUpToDate(void);
int* getTaskAction(void);
void updatePosAlly(char* data);
DectectedReponse detectReponse(char* data,int start,int nbr);
inline char radioGetChar(void);
inline void radioSendChar(char data);
void radioSendString(char* data);
int radioGetInt(char* data);
void radioSendInt(int data);
float radioGetFloat(void);
void radioSendFloat(float data);
void ResetBLE(void);
inline int radioGetRxBufferSpace(void);
positionInteger getAllyPos(void);
int getTaskRequest(void);
void setTaskRequest(int tmp);
#endif
