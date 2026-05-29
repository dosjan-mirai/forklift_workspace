/*
 * motor.c
 *
 *  Created on: Jan 2, 2025
 *      Author: doska
  *  Энэ source файл нь:
 *  - Encoder speed тооцоолох
 *  - PID control output гаргах
 *  - DC моторын чиглэл/PWM тохируулах
 *  - map болон saturate туслах функцүүд
 *  зэргийг агуулна.
 */
#include "motor.h"
#include "math.h"
#include <stdint.h> // Include for standard integer types like uint16_t, int16_t
#include <stdlib.h> // Include for abs()
double ML;
double MR;
double speed;
double angle;
double pi=3.14159265359;

/* ============================================================
 * SATURATE()
 *
 * Үүрэг:
 *   Оролтын утгыг MIN болон MAX хязгаарын хооронд барина.
 *
 * Жишээ:
 *   SATURATE(150, -100, 100) -> 100
 *   SATURATE(-120, -100, 100) -> -100
 *   SATURATE(50, -100, 100) -> 50
 * ============================================================ */
float SATURATE(float _IN, float _MIN, float _MAX) {
    if (_IN < _MIN) return _MIN;
    if (_IN > _MAX) return _MAX;
    return _IN;
}
/* ============================================================
 * Estimate_Speed()
 *
 * Үүрэг:
 *   Encoder timer-ийн counter утгаас motor-ийн хурдыг тооцно.
 *
 * Ажиллах зарчим:
 *   1. Timer-ийн одоогийн CNT утгыг уншина.
 *   2. Өмнөх CNT утгатай харьцуулж difference гаргана.
 *   3. difference / SamplingTimeMs ашиглаж speed тооцно.
 *   4. Exponential filter хэрэглэнэ.
 *   5. Moving average filter хэрэглэнэ.
 *   6. Oldpos утгыг шинэчилнэ.
 *
 * Encoder:
 *   Encoder->timer_address нь encoder mode дээр ажиллаж байгаа timer байх ёстой.
 *   Жишээ:
 *      Encoder.timer_address = &htim3;
 *
 * SamplingTimeMs:
 *   Энэ функцийг хэдэн ms тутам дуудаж байгаагаа өгнө.
 *   Жишээ:
 *      Estimate_Speed(&encoder, 10);
 * ============================================================ */

void Estimate_Speed(Motor_Encoder_Variables *Encoder, uint16_t SamplingTimeMs) {
    /* Sampling time 0 байвал 0-д хуваах алдаа гарах учраас шууд return */

	if (SamplingTimeMs == 0) return;


    /* Timer-ийн CNT register-ээс одоогийн encoder position уншина */
	Encoder->CurrentPos = Encoder->timer_address->Instance->CNT;
    /* Одоогийн position болон өмнөх position-ийн зөрүү */

	Encoder->difference = Encoder->CurrentPos - Encoder->Oldpos;
    /* Speed тооцох:
     * difference = encoder tick-ийн өөрчлөлт
     * SamplingTimeMs = хэмжилтийн хугацаа
     */
	// Convert difference to speed
	Encoder->Speed = (float)Encoder->difference / SamplingTimeMs;
    /* Exponential filter:
     * Шинэ speed болон өмнөх filtered speed-ийг холин зөөлрүүлнэ.
     */
	// Apply exponential filtering
	Encoder->Speed = Alpha * Encoder->Speed + (1.0f - Alpha) * Encoder->PreviousFilteredSpeed;

	// Update moving average filter
	Encoder->SpeedHistory[Encoder->SpeedIndex] = Encoder->Speed;
    /* Дараагийн бичих индекс рүү шилжинэ.
     * % MA_FILTER_SIZE хийж массивын төгсгөлд хүрвэл 0 руу буцна.
     */
	Encoder->SpeedIndex = (Encoder->SpeedIndex + 1) % MA_FILTER_SIZE;

	// Calculate moving average
	float sum = 0.0f;
	for (uint8_t i = 0; i < MA_FILTER_SIZE; i++) {
		sum += Encoder->SpeedHistory[i];
	}
	Encoder->Speed =(int16_t)( sum / MA_FILTER_SIZE);

	// Update previous filtered speed and position
	Encoder->PreviousFilteredSpeed = Encoder->Speed;
	Encoder->Oldpos = Encoder->CurrentPos;
}

/* ============================================================
 * PID_init()
 *
 * Үүрэг:
 *   PID controller-ийн эхний тохиргоог хийнэ.
 *
 * Parameter:
 *   PID    - PID_Variables struct-ийн pointer
 *   KP     - Proportional коэффициент
 *   KI     - Integral коэффициент
 *   KD     - Derivative коэффициент
 *   S_MIN  - SetPoint-ийн доод хязгаар
 *   S_MAX  - SetPoint-ийн дээд хязгаар
 *   CO_MIN - Control Output-ийн доод хязгаар
 *   CO_MAX - Control Output-ийн дээд хязгаар
 *
 * Жишээ:
 *   PID_init(&motor_pid, 4.0f, 0.0f, 0.5f,
 *            -200, 200,
 *            -10000, 10000);
 * ============================================================ */
void PID_init(PID_Variables *PID, float KP, float KI, float KD, int16_t S_MIN, int16_t S_MAX, int16_t CO_MIN, int16_t CO_MAX) {
    PID->KP = KP;
    PID->KI = KI;
    PID->KD = KD;
    PID->SP_MAX = S_MAX;
    PID->SP_MIN = S_MIN;
    PID->CO_MIN = CO_MIN;
    PID->CO_MAX = CO_MAX;
    PID->I_error = 0.0f;
    PID->lastError = 0.0f;
    PID->CO = 0.0f;
    PID->D_error = 0.0f;
}
/* ============================================================
 * PID_estimate()
 *
 * Үүрэг:
 *   SP болон PV дээр үндэслэн PID гаралт CO-г тооцно.
 *
 * SP:
 *   SetPoint буюу хүссэн утга
 *
 * PV:
 *   Process Variable буюу бодит хэмжигдсэн утга
 *
 * CO:
 *   Control Output буюу мотор руу өгөх команд
 *
 * PID томьёо:
 *   error = SP - PV
 *   CO = KP * error + KI * I_error + KD * D_error
 * ============================================================ */
void PID_estimate(PID_Variables *PID) {
    /* SP утгыг зөвшөөрөгдөх хязгаарт барина */
	PID->SP=SATURATE(PID->SP, PID->SP_MIN, PID->SP_MAX);
	// Calculate error
    /* Одоогийн алдаа */

	PID->error = PID->SP - PID->PV;
	PID->I_error += PID->error;
	PID->I_error=SATURATE(PID->I_error, PID->CO_MIN, PID->CO_MAX);
	float D_term = PID->error - PID->lastError;
	PID->D_error = 0.9f * PID->D_error + 0.1f * D_term; // Adjustable low-pass filter
	// Compute control output
	PID->CO = (PID->KP * PID->error) + (PID->KI * PID->I_error) + (PID->KD * PID->D_error);
	// Saturate control output
	PID->CO=SATURATE(PID->CO, PID->CO_MIN, PID->CO_MAX);
	// Store current error for next derivative calculation
	PID->lastError = PID->error;
}
/* ============================================================
 * SET_PID()
 *
 * Үүрэг:
 *   PID-ийн KP, KI, KD коэффициентийг ажиллаж байх үед өөрчилнө.
 *
 * Жишээ:
 *   SET_PID(&motor_pid, 3.0f, 0.1f, 0.2f);
 * ============================================================ */
void SET_PID(PID_Variables *PID , float P ,float I,float D )
{
	PID->KP=P;
	PID->KI=I;
	PID->KD=D;
}
/* ============================================================
 * map()
 *
 * Үүрэг:
 *   Нэг мужийн утгыг өөр муж руу хувиргана.
 *
 * Жишээ:
 *   Joystick 0-255 утгыг -100...100 болгох:
 *
 *   map(x, 0, 255, -100, 100)
 *
 * Formula:
 *   output = (x - in_min) * (out_max - out_min)
 *            / (in_max - in_min) + out_min
 * ============================================================ */
float map(float x, float in_min, float in_max, float out_min, float out_max) {
	// Check for invalid input range
	if (in_min >= in_max) {
		return 0.0f; // Handle error
	}
	// Ensure x is within bounds
	if (x < in_min) x = in_min;
	if (x > in_max) x = in_max;
	// Map input to output range
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* ============================================================
 * DC_MOTOR_SET_DIRECTION()
 *
 * Үүрэг:
 *   DC моторын эргэх чиглэл болон PWM хурдыг тохируулна.
 *
 * speed > 0:
 *   Урагшаа эргэнэ.
 *
 * speed < 0:
 *   Хойшоо эргэнэ.
 *
 * speed == 0:
 *   Мотор зогсоно.
 *
 * Motor->PWM_CHANNEL_ADDRESS:
 *   PWM-ийн CCR register-ийн хаяг.
 *   Жишээ:
 *      &TIM2->CCR1
 *
 * Direction logic:
 *   pinA = HIGH, pinB = LOW   -> forward
 *   pinA = LOW,  pinB = HIGH  -> backward
 *   pinA = LOW,  pinB = LOW   -> stop/coast
 * ============================================================ */
void DC_MOTOR_SET_DIRECTION(DC_Motor_GPIO_t *Motor,int16_t speed)
{
	if(speed>0)
	{
		*(Motor->PWM_CHANNEL_ADDRESS)=abs(speed);
		Motor->portA->BSRR = Motor->pinA;                         // Set GPIO_A high
		Motor->portB->BSRR = (uint32_t)Motor->pinB << 16;         // Set GPIO_B low
	}
	else if(speed<0)
	{
		// Set GPIO_A low and GPIO_B high for backward direction
		*(Motor->PWM_CHANNEL_ADDRESS)=abs(speed);
		Motor->portA->BSRR = (uint32_t)Motor->pinA << 16;         // Set GPIO_A low
		Motor->portB->BSRR = Motor->pinB;   // Set GPIO_B high
	}
	else
	{
		Motor->portA->BSRR = (uint32_t)Motor->pinA << 16;         // Set GPIO_A low
		Motor->portB->BSRR = (uint32_t)Motor->pinB << 16;         // Set GPIO_B low
	}
}


