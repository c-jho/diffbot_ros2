#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <float.h>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "std_msgs/msg/int16_multi_array.hpp"

using namespace std;

#define _X		          0
#define _Y		          1
#define _THETA            2

#define LW		          0
#define RW		          1
#define WHEEL_NUM         2

#define ENABLE            1
#define DISABLE           0
#define FAIL              0
#define SUCCESS           1

#define ON                1
#define OFF               0
#define RESET             0

#define CCW               -1
#define CW                1

#define Abs(a)            (((a)<(0)) ? -(a):(a))

#define BIT0              0x01
#define BIT1              0x02
#define BIT2              0x04
#define BIT3              0x08
#define BIT4              0x10
#define BIT5              0x20
#define BIT6              0x40
#define BIT7              0x80

#define ID_BLDC_CTRL        1
#define ID_MDUI             2
#define ID_ALL              0xfe

#define PID_REQ_PID_DATA    4
#define PID_TQ_OFF	        5
#define PID_COMMAND         10
#define PID_ALARM_RESET     12
#define PID_POSI_RESET      13
#define PID_VEL_CMD         130
#define PID_BAUDRATE        135
#define PID_VOLT_IN         143
#define PID_SLOW_START      153
#define PID_SLOW_DOWN       154
#define PID_MAIN_DATA       193

#define CMD_MAIN_BC_ON      5
#define CMD_MAIN_BC_OFF     6

#define MAX_PACKET_SIZE     26
#define MAX_DATA_SIZE       23

#define REQUEST_PNT_MAIN_DATA 2

#define DURATION            0.0001

#define TIME_50MS           1
#define TIME_100MS          2
#define TIME_1S             20
#define TIME_5S             100

#define CHECK_EMERGENCY_SW  0
#define SEND_DATA_AFTER_1S  1

#define CMD_ALARM_RESET     8

typedef unsigned char  BYTE;
typedef unsigned short WORD;
typedef unsigned int   DWORD;

typedef struct {
    BYTE bySndBuf[MAX_PACKET_SIZE];
    BYTE byRcvBuf[WHEEL_NUM+1][MAX_PACKET_SIZE];
    BYTE byPacketSize;
    BYTE byPacketNum;
    BYTE byIn, byStep;
    BYTE byChkSend;
    BYTE byChkRcv;
    BYTE fgInIdleLine, fgPacketOK, fgComComple;
    BYTE byTotalRcvDataNum;
    BYTE fgChk;
    BYTE byChkSum, byMaxDataNum, byDataNum;

    int nIDPC, nIDMDUI, nIDMDT, nRMID;
    int nBaudrate, nWheelLength, nGearRatio, fgDirSign;
    short sSetDia, sSetWheelLen, sSetGear;
    int nCmdSpeed, nCmdAngSpeed;
    int nSlowstart, nSlowdown;
    float nWheelDiameter;

    long lPosi[2], lTempPosi[2];
    short sTheta, sTempTheta, sExTheta;

    short sMotorRPM[3];
    long lMotorPosi[3];

    BYTE byChkComError;
    BYTE fgComDataChk;
    BYTE fgInitsetting;

    int nHallType, nMaxRPM, nAngResol;

}Communication;
extern Communication Com;

typedef struct {
    BYTE byLow;
    BYTE byHigh;
}IByte;

extern IByte Short2Byte(short sIn);
extern int Byte2Short(BYTE byLow, BYTE byHigh);
extern int Byte2LInt(BYTE byData1, BYTE byData2, BYTE byData3, BYTE byData4);

extern int InitSerial(void);
extern int InitSetParam(void);
extern int InitSetSlowStart(void);
extern int InitSetSlowDown(void);
extern int InitRobotMotDir(void);
extern int PutMdData(BYTE byPID, BYTE byID, int id_num, int nArray[]);
extern int PutMdData(BYTE byPID, BYTE byMID, int nArray[]);
extern long *GetMdData(BYTE byPID);
extern int MdReceiveProc(void);
extern int ReceiveDataFromController(BYTE init);
extern int AnalyzeReceivedData(BYTE byArray[], BYTE byBufNum);
extern BYTE IDupdate(int &id, BYTE &sendCmdvel);
