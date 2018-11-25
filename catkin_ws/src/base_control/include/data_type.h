#ifndef _DATA_TYPE_H
#define _DATA_TYPE_H
#include<iostream>
using namespace std;
#pragma pack(1)


#define ENCODER_MIN -2147483647
#define ENCODER_MAX 2147483647
#define PI 3.141592653
#define GRAVITY 9.7964

typedef unsigned char uint8;
typedef char int8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef signed int int32;

namespace base_controller
{


#define SYNC1 0xAA
#define SYNC2 0xBB

#define TAIL1 0XCC
#define TAIL2 0xDD

#define SPEED_MUTIPLIER 10

#define ODOM_ID 0x0002
#define ODOM_DATA_LENGTH (4*5+2)
#define ODOM_TOTAL_LENGTH sizeof(Odometry_t)

#define MOTOR_CMD_TYPE 1
#define MOTOR_CMD_LENGTH sizeof(motor_cmd_t)

#define IMU_ID 0x16
#define IMU_DATA_LENGTH 0x0023
#define IMU_TOTAL_LENGTH (IMU_DATA_LENGTH+7)



#define CMD_LENGTH sizeof(cmd_t)

//fram from base to mcu.  
#define CMD_BASE_TO_MCU_FRAME 0x0003
#define CMD_MCU_TO_BASE_ACK_FRAME 0x0004
#define CMD_DATA_LENGTH 0x0000
#define CMD_RESET_ODOM 0x0080
//clear alarm
#define CMD_CLEAR_ALARM 0x0082
//lock motor
#define CMD_LOCK_MOTOR 0x0084 
//unlock motor
#define CMD_UNLOCK_MOTOR 0x0086

#define CMD_COMM 0x0001

// cmd frame from mcu to base
#define CMD_MCU_TO_BASE_FRAME 0x0006
//acklowedge from base to mcu
#define CMD_BASE_TO_MCU_ACK_FRAME 0x0005
//power switch
#define CMD_POWER_ON 0x0002
#define CMD_POWER_OFF 0x0003
//emergency cmd
#define CMD_EMERGENCY_ON 0x0004
#define CMD_EMERGENCY_OFF 0x0005

#define CMD_REMOVE_EMERENCY 0x0070
//
#define CMD_BUMPER_SWITCH_ON 0x0070

#define CMD_BATTERY 0x0060

#define CMD_MOTOR_STATUS 0x0080

//encoder datagram 
struct Odometry_t{
    uint8 sync1;
    uint8 sync2;
    uint16 frame_type;
    uint32 timestamp;//sec
    uint32 millisecs;//0.1ms
    uint32 seq;
    int32 encoder_A;
    int32 encoder_B;
    uint16 check_sum;
    uint8 tail1;
    uint8 tail2;
};

struct Imu_t{
    uint8 sync1;
    uint8 sync2;
    uint8 id;
    uint16 length;
    uint8 utc_hour;
    uint8 utc_min;
    uint8 utc_sec;
    uint32 utc_micro;
    uint32 timetag;
    int32 gyro_x;
    int32 gyro_y;
    int32 gyro_z;
    int32 acc_x;
    int32 acc_y;
    int32 acc_z;
    uint8 checksum_A;
    uint8 checksum_B;
};

//motor cmd structure
struct motor_cmd_t{
    uint8 sync1;
    uint8 sync2;
    uint16 frame_type;
    uint32 seq;
    int16 left_motor;
    int16 right_motor;
    uint16 crc;
    uint8 tail1;
    uint8 tail2;
};

// cmd structure
struct cmd_t{
    uint8 sync1;
    uint8 sync2;
    uint16 frame_type;
    uint16 cmd;
    uint16 length;
    uint16 crc;
    uint8 tail1;
    uint8 tail2;
};

// cmd structure
struct bumper_t{
    uint8 sync1;
    uint8 sync2;
    uint16 frame_type;
    uint16 cmd;
    uint16 length;
    uint16 sw;
    uint16 crc;
    uint8 tail1;
    uint8 tail2;
};



//convert buffer data to two byte length
uint16 getLength(uint8 *buffer, const int size)
{	
	uint16 length = 0x00;
	memcpy((uint8 *)&length, buffer, size);
	return length;

}

//convert utc time to ros time


}// end of namespace
#endif