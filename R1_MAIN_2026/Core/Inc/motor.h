/*
 * motor.h
 *
 *  Created on: Jan 2, 2025
 *      Author: doska
 *       *  Энэ header файл нь:
 *  - PID контроллерын бүтэц
 *  - Encoder speed хэмжих бүтэц
 *  - DC моторын GPIO/PWM бүтэц
 *  - CAN motor ID-ууд
 *  - motor.c дотор байгаа функцүүдийн prototype
 *  зэргийг тодорхойлно.
 */
#include "main.h"
#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#define BLDC_Init_Speed 48
#define BLDC_MAX_SPEED 120
#define BLDC_MIN_SPEED 54
#define BLDC_STEP 8
#define BLDC_BRAKE 45
#define ROBOMASTER_MAX_SPEED 10000
#define ROBOMASTER_MIN_SPEED 0
#define MA_FILTER_SIZE 10
/* Exponential filter-ийн коэффициент
 * Alpha их байх тусам шинэ speed утгад илүү их итгэнэ.
 * Alpha бага бол өмнөх speed утга илүү хүчтэй нөлөөлнө.
 */
#define Alpha 0.8

#define MAX_CAN_RM		 8
#define NUM_OF_CAN		 1
#define RM_TX_GRP1_ID	 0x200
#define RM_TX_GRP2_ID	 0x1FF
#define CAN_DATA_SIZE	 8
#define CAN1_RX_ID_START 0x201

typedef struct
{
	float KP;	 /* Proportional gain: алдаанд шууд үржигдэх коэффициент */
	float KI;	 /* Integral gain: хуримтлагдсан алдаанд үржигдэх коэффициент */
	float KD;		/* Derivative gain: алдааны өөрчлөлтөд үржигдэх коэффициент */
	float error;	  /* Одоогийн алдаа: SP - PV */
	float I_error;		/* Integral error: алдааны хуримтлал */
	float D_error;		  /* Derivative error: алдааны өөрчлөлтийн шүүсэн утга */
	float lastError;		 /* Өмнөх давталтын алдаа */
	float SP; 	/*SetPoint*/
	float PV; 	/*ProcessVariable*/
	int16_t CO; 	/*ControlOutput*/
	int16_t SP_MAX;			 /* SetPoint-ийн хамгийн их зөвшөөрөгдөх утга */
	int16_t SP_MIN;			 /* SetPoint-ийн хамгийн бага зөвшөөрөгдөх утга */
	int16_t CO_MAX;			 /* Control Output-ийн хамгийн их утга */
	int16_t CO_MIN;			 /* Control Output-ийн хамгийн бага утга */



} PID_Variables;

typedef struct {
    int16_t Oldpos;                /* Өмнөх encoder counter утга */
    int16_t CurrentPos;             /* Одоогийн encoder counter утга */
    int16_t difference;             /* CurrentPos - Oldpos */
    int16_t Speed;                   /* Тооцоолсон speed */

    TIM_HandleTypeDef *timer_address; // Pointer to timer structure
    /* Encoder mode дээр ажиллаж байгаа timer-ийн хаяг.
     * Жишээ:
     * Encoder.timer_address = &htim3;
     */
    float SpeedHistory[MA_FILTER_SIZE]; // Moving average filter data
    /* Moving average filter-д ашиглах speed-ийн түүх */

    uint8_t SpeedIndex;            // Current index in speed history
    /* SpeedHistory массивын одоогийн индекс */

    float PreviousFilteredSpeed;   // Last filtered speed value
    /* Exponential filter-ийн өмнөх filtered speed */
} Motor_Encoder_Variables;

typedef enum {
	CAN1_MOTOR0,
	CAN1_MOTOR1,
	CAN1_MOTOR2,
	CAN1_MOTOR3,
	CAN1_MOTOR4,
	CAN1_MOTOR5,
	CAN1_MOTOR6,
	CAN1_MOTOR7,
	MAX_NUM_OF_MOTORS
} Motor;

typedef enum {
	CAN_GROUP_ID = 0x200,
	CAN_3508_M1_ID,
	CAN_3508_M2_ID,
	CAN_3508_M3_ID,
	CAN_3508_M4_ID,
	CAN_3508_M5_ID,
	CAN_3508_M6_ID,
	CAN_3508_M7_ID,
	CAN_3508_M8_ID,
} can_msg_id_e;
//typedef struct{
//	double LX;
//	double LY;
//	double RX;
//	double RY;
//	double X;
//	double Y;
//}joystick_variables;
typedef struct {
    GPIO_TypeDef *portA;
    uint16_t pinA;
    GPIO_TypeDef *portB;
    uint16_t pinB;
    uint32_t *PWM_CHANNEL_ADDRESS;
} DC_Motor_GPIO_t;
/* Encoder counter-ийн өөрчлөлтөөс speed тооцоолох функц */

void Estimate_Speed(Motor_Encoder_Variables *Encoder, uint16_t SamplingTimeMs);
/* PID контроллерын гаралт тооцоолох функц */
void PID_estimate(PID_Variables *PID);

float map(float x, float in_min, float in_max, float out_min, float out_max);
/* PID-ийн эхний тохиргоо хийх функц */
void PID_init(PID_Variables *PID, float KP , float KI, float KD,int16_t S_MAX, int16_t S_MIN,int16_t P_MAX,int16_t P_MIN);
//void Estimate_Setpoint_BRM(joystick_variables *joy , PID_Variables *Motor[]);
void Lift_Motor(int mot_out);
void DC_MOTOR_SET_DIRECTION(DC_Motor_GPIO_t *Motor,int16_t speed );
float SATURATE(float _IN, float _MIN, float _MAX);
#endif /* INC_MOTOR_H_ */
