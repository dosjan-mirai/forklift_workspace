#ifndef __MOTOR_CONTORL_H
#define __MOTOR_CONTORL_H

#include <stdint.h>
#include "main.h" // stm32 hal
#include "ris_protocol.h"
#pragma pack(1)
typedef union
{
	int32_t     L;
	uint8_t     u8[4];
	uint16_t    u16[2];
	uint32_t    u32;
	float       F;
} COMData32;

typedef struct
{
	// 定义 数据包头 ::define data packet header
	unsigned char start[2]; // 包头
	unsigned char motorID;  // 电机ID  0,1,2,3 ...   0xBB 表示向所有电机广播（此时无返回）:: Motor ID 0,1,2,3 ... 0xBB means broadcasting to all motors (no return at this time)
	unsigned char reserved;
} COMHead;

typedef struct
{     // 以 4个字节一组排列 ，不然编译器会凑整::Arrange them in groups of 4 bytes, otherwise the compiler will round them up
	// 定义 数据::define data
	uint8_t mode;        // 当前关节模式 ::   Current joint mode
	uint8_t ReadBit;   // 电机控制参数修改     是否成功位 :: Motor control parameter modification success bit
	int8_t Temp;        // 电机当前平均温度 :: Current average temperature of the motor
	uint8_t MError;     // 电机错误 标识 :: Motor error identification

	COMData32 Read;    // 读取的当前 电机 的控制数据 :: Read the current control data of the motor
	int16_t T;          // 当前实际电机输出力矩  7 + 8 描述::Current actual motor output torque 7 + 8 Description

	int16_t W;          // 当前实际电机速度（高速）   8 + 7 描述:: Current actual motor speed (high speed) 8 + 7 Description
	float LW;            // 当前实际电机速度（低速）     Current actual motor speed (low speed)

	int16_t W2;          // 当前实际关节速度（高速）   8 + 7 描述:: Current actual joint speed (high speed) 8 + 7 Description
	float LW2;          // 当前实际关节速度（低速）::     Current actual joint speed (low speed)

	int16_t Acc;        // 电机转子加速度 15+0 描述  惯量较小:: Motor rotor acceleration 15+0 Description Small inertia
	int16_t OutAcc;      // 输出轴加速度 12+3 描述  惯量较大:: Output shaft acceleration 12+3 Description Large inertia

	int32_t Pos;         // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准::
	/*Current motor position (main control 0 point correction, the motor
      joint is still based on the encoder 0 point)*/
	int32_t Pos2;        // 关节编码器位置(输出编码器):: Joint encoder position (output encoder)

	int16_t gyro[3];    // ���������6�ᴫ��������
	int16_t acc[3];

	// 力传感器的数据 ::     Force sensor data
	int16_t Fgyro[3];
	int16_t Facc[3];
	int16_t Fmag[3];
	uint8_t Ftemp;      // 8位表示的温度  7位（-28~100度）  1位0.5度分辨率
	/*// Temperature represented by 8 bits 7 bits
	 * (-28~100 degrees) 1 bit 0.5 degree resolution*/

	int16_t Force16;    // 力传感器高16位数据:: High 16-bit data of force sensor
	int8_t Force8;       // 力传感器低8位数据:: Force sensor lower 8-bit data

	uint8_t FError;     //  足端传感器错误标识:: Foot Sensor Error Identification

	int8_t Res[1];       // 通讯 保留字节::     Communication reserved bytes

} ServoComdV3; // 加上数据包的包头 和CRC 78字节（4+70+4）
/*Plus the packet header and CRC 78 bytes (4+70+4)*/

typedef struct
{
	uint8_t head[2];    // 包头         2Byte
	RIS_Mode_t mode;    // 电机控制模式  1Byte
	RIS_Fbk_t   fbk;    // 电机反馈数据 11Byte
	uint16_t  CRC16;    // CRC          2Byte
} MotorData_t;   //返回数据


typedef struct
{
	uint8_t none[8];             // 保留

} LowHzMotorCmd;

typedef struct
{  // 以 4个字节一组排列 ，不然编译器会凑整::Arrange them in groups of 4 bytes, otherwise the compiler will round them up
	// 定义 数据::Definition data
	uint8_t mode;                // 关节模式选择::Joint mode selection
	uint8_t ModifyBit;          // 电机控制参数修改位::Motor control parameter modification bit
	uint8_t ReadBit;           // 电机控制参数发送位::Motor control parameter sending bit
	uint8_t reserved;

	COMData32 Modify;           // 电机参数修改 的数据::Motor parameter modification data
	//实际给FOC的指令力矩为： The actual command torque given to FOC is
	// K_P*delta_Pos + K_W*delta_W + T
	int16_t T;                 // 期望关节的输出力矩（电机本身的力矩）x256, 7 + 8 描述 :: Desired output torque of the joint (motor torque itself) x256, 7 + 8 Description
	int16_t W;                  //期望关节速度 （电机本身的速度） x128, 8 + 7描述 :: Desired joint speed (speed of the motor itself) x128, 8 + 7 description
	int32_t Pos;                  // 期望关节位置 x 16384/6.2832, 14位编码器（主控0点修正，电机关节还是以编码器0点为准）
	/*Desired joint position x 16384/6.2832, 14-bit encoder
	 * (main control 0 point correction, motor joints are still based on the encoder 0 point)*/
	int16_t K_P;                 // 关节刚度系数 x2048  4+11 描述 :: Joint stiffness coefficient x2048 4+11 Description
	int16_t K_W;                 // 关节速度系数 x1024  5+10 描述 :: Joint speed coefficient x1024 5+10 Description


	uint8_t LowHzMotorCmdIndex; // 保留 ::reserve
	uint8_t LowHzMotorCmdByte;   // 保留 ::reserve

	COMData32 Res[1];            // 通讯 保留字节  用于实现别的一些通讯内容
	/*
Communication reserved bytes are used to implement other communication content*/
} MasterComdV3; // 加上数据包的包头 和CRC 34字节
/*Add the header of the data packet and CRC 34 bytes*/
typedef struct
{
	// 定义 电机控制命令数据包 :: Define motor control command data packet
	uint8_t head[2];    // 包头         2Byte
	RIS_Mode_t mode;    // 电机控制模式  1Byte
	RIS_Comd_t comd;    // 电机期望数据 12Byte
	uint16_t   CRC16;   // CRC          2Byte
} ControlData_t;     //电机控制命令数据包 :: Motor control command packet


#pragma pack()
//go电机发送接收数据结构体变量

typedef struct
{
	// 定义 发送格式化数据
	ControlData_t motor_send_data;   //电机控制数据结构体 ::    Motor control data structure
	int hex_len;                     //发送的16进制命令数组长度, 34 ::    The length of the hexadecimal command array sent, 34
	long long send_time;               //发送该命令的时间, 微秒(us) :: Time to send the command, microseconds (us)
	unsigned short id;                   //电机ID，0代表全部电机 :: Motor ID, 0 represents all motors
	unsigned short mode;                 // 0:空闲, 5:开环转动, 10:闭环FOC控制 ::  0: Idle, 5: Open-loop rotation, 10: Closed-loop FOC control
	//实际给FOC的指令力矩为：
	// K_P*delta_Pos + K_W*delta_W + T
	float T;                           //期望关节的输出力矩（电机本身的力矩）（Nm） ::
	//The output torque of the desired joint (the torque of the motor itself) (Nm)
	float W;                           //期望关节速度（电机本身的速度）(rad/s)
	//Desired joint speed (speed of the motor itself) (rad/s)
	float Pos;                          //期望关节位置（rad）
	//Desired joint position (rad)
	float K_P;                          //关节刚度系数
	//joint stiffness coefficient
	float K_W;                          //关节速度系数
	//Joint speed coefficient
	COMData32 Res;                    // 通讯 保留字节  用于实现别的一些通讯内容
	// Communication reserved bytes are used to implement other communication content
} MOTOR_send;

typedef struct
{
	// ���� ��������
	MotorData_t motor_recv_data;     //电机接收数据结构体，详见motor_msg.h
	//Motor receiving data structure, see motor_msg.h for details
	int hex_len;                        //接收的16进制命令数组长度, 78
	//The length of the received hexadecimal command array, 78
	long long resv_time;                 //接收该命令的时间, 微秒(us)
	//Time to receive the command, microseconds (us)
	int correct;                         //接收数据是否完整（1完整，0不完整）
	//Whether the received data is complete (1 complete, 0 incomplete)
	//解读得出的电机数据

	//Interpret the motor data
	unsigned char motor_id;              //电机ID
	unsigned char mode;                 // 0:空闲, 5:开环转动, 10:闭环FOC控制
	// 0: Idle, 5: Open-loop rotation, 10: Closed-loop FOC control
	int Temp;                            //温度
	unsigned char MError;                 //错误码
	float T;                            // 当前实际电机输出力矩
	//Current actual motor output torque
	float W;														// speed
	float Pos;                            // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
	//Current motor position (main control 0 point correction, the motor joint is still based on the encoder 0 point)
	float footForce;												  // 足端气压传感器数据 12bit (0-4095)

	// Foot pressure sensor data 12bit (0-4095)

} MOTOR_recv;

#define SET_485_DE_UP() HAL_GPIO_WritePin(UNITREE485_DE_GPIO_Port, UNITREE485_DE_Pin, GPIO_PIN_SET)
#define SET_485_DE_DOWN() HAL_GPIO_WritePin(UNITREE485_DE_GPIO_Port, UNITREE485_DE_Pin, GPIO_PIN_RESET)

#define SET_485_RE_UP() HAL_GPIO_WritePin(UNITREE485_DE_GPIO_Port, UNITREE485_DE_Pin, GPIO_PIN_SET)
#define SET_485_RE_DOWN() HAL_GPIO_WritePin(UNITREE485_DE_GPIO_Port, UNITREE485_DE_Pin, GPIO_PIN_RESET)

uint32_t crc32_core(uint32_t *ptr, uint32_t len);
int modify_data(MOTOR_send *motor_s);
int extract_data(MOTOR_recv *motor_r);
HAL_StatusTypeDef SERVO_Send_recv( MOTOR_send *pData, MOTOR_recv *rData);

#endif
