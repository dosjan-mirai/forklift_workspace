/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "communication.h"
#include "motor.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	int LX;
	int LY;
	int RX;
	int RY;
}joystick_variables;
typedef struct{
	float FL;
	float FR;
	float RL;
	float RR;
}Wheel_variables;
typedef struct{
	float vx;
	float vy;
	float wz;
}ChassisCommand;

/* Fixed mode counter: 1-ээс эхэлнэ */
uint8_t X_AXIS_FIXED_STEP_COUNTER = 1;
uint8_t Y_AXIS_FIXED_STEP_COUNTER = 1;
/*
 * Fixed mode байрлалууд
 * Өөрийн механик limit-д тааруулж эдгээр утгыг өөрчилж болно.
 */
static const float X_FIXED_SP[4] = {0.0f, 100.0f, 310.0f, 400.0f};
static const float Y_FIXED_SP[4] = {0.0f, 120.0f, 320.0f, 520.0f};
#define X_FIXED_STEP_COUNT  4
#define Y_FIXED_STEP_COUNT  4

Wheel_variables wheel;
PID_Variables X_AXIS_M3508_SPEED_PID;
PID_Variables X_AXIS_M3508_POS_PID;
PID_Variables Y_AXIS_M3508_SPEED_PID;
PID_Variables Y_AXIS_M3508_POS_PID;
Motor_Encoder_Variables X_AXIS_M3508_ENCODER;
Motor_Encoder_Variables Y_AXIS_M3508_ENCODER;
int32_t Encoder_SUM=0;
int32_t Encoder_SUM_GRIP=0;
int16_t GRIPPER_OUT=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define JOY_AXIS_MAX          127.0f
#define JOY_AXIS_INT_DEADBAND 4
#define JOY_AXIS_SLEW_STEP    8
#define JOY_TRANSLATION_DZ    0.05f
#define JOY_ROTATION_DZ       0.05f
#define WHEEL_CMD_LIMIT       60.0f
#define WHEEL_RADIUS_M        0.150f
#define CHASSIS_HALF_LEN_M    0.297f
#define CHASSIS_HALF_WIDTH_M  0.440f
#define CHASSIS_K_M           (CHASSIS_HALF_LEN_M + CHASSIS_HALF_WIDTH_M)
#define MAX_VX_MPS            4.0f
#define MAX_VY_MPS            4.0f
#define MAX_WZ_RADPS          6.0f
#define PS2_RESPONSE_HEADER   0x5AU
#define PS2_FRAME_SIZE        9U
#define PS2_INIT_RETRIES      5U
#define PS2_SPI_TIMEOUT_MS    10U
#define PS2_CLK_Pin           GPIO_PIN_10
#define PS2_CLK_GPIO_Port     GPIOC
#define PS2_CMD_Pin           GPIO_PIN_2
#define PS2_CMD_GPIO_Port     GPIOB
#define PS2_DAT_Pin           GPIO_PIN_4
#define PS2_DAT_GPIO_Port     GPIOB
/* X axis encoder нэг tick тутамд хэдэн mm шилжих коэффициент */
#define X_PER_STEP_MM 0.0431406384814495
/* Y axis encoder нэг tick тутамд хэдэн mm шилжих коэффициент */

#define Y_PER_STEP_MM 0.0016492518393929
/* Gripper encoder tick-ийг mm рүү хувиргах коэффициент
 * Анхаар: 60/30802 нь integer division болж 0 болох эрсдэлтэй.
 * Илүү зөв нь:
 * #define GRIPPER_PER_STEP_MM (60.0f / 30802.0f)
 */
#define GRIPPER_PER_STEP_MM (60.0f / 30802.0f)
/* Slow mode үед wheel speed-ийг 12% болгож багасгах scale */

/* Fixed байрлал дээр аль хэдийн ойрхон байгаа эсэхийг шалгах жижиг tolerance */
#define FIXED_STEP_EPS_MM 1.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2004c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2004c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2004c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2004c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

CAN_HandleTypeDef hcan1;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
I2C_HandleTypeDef hi2c4;

SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for X_Y_ARM_PID */
osThreadId_t X_Y_ARM_PIDHandle;
const osThreadAttr_t X_Y_ARM_PID_attributes = {
  .name = "X_Y_ARM_PID",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Gripper */
osThreadId_t GripperHandle;
const osThreadAttr_t Gripper_attributes = {
  .name = "Gripper",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
volatile joystick_variables filtered_joy_data;
volatile joystick_variables raw_joy_data;
volatile Wheel_variables wheel_targets;
volatile uint8_t joystick_frame_valid;
/* RX[4] == 254 үед joystick-ийг шууд/linear mode-оор ажиллуулах flag */
volatile uint8_t direct_slow_mode = 0;
volatile int16_t shift_cmd = 0;
volatile int16_t dc_cmd = 0;
volatile int16_t angle_setpoint = 0;
static int8_t mecanum_direction = 1;
static uint8_t prev_reverse_button_rx3 = 0;
static uint32_t btn251_pressed_tick = 0;
static uint8_t btn251_state = 0;
PID_Variables GRIPPER_POS_PID;
PID_Variables GRIPPER_SPEED_PID;
Motor_Encoder_Variables GRIPPER_ENCODER;

uint8_t RX[9];
uint8_t TX[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t Axis_X_CNT=0;
uint8_t Axis_Y_CNT=0;
uint8_t CLOSE_FLAG=0;
uint8_t Display_color_state = 0;

float Scale_number = 0.12f;
//-----------------------------------------------------------------------
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_UART8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
void CAN_Task(void *argument);
void JoyStick_Task(void *argument);
void Mecanum_Task(void *argument);
void ARM_PID(void *argument);
void Gripper_task(void *argument);

/* USER CODE BEGIN PFP */
static int map_joystick(uint8_t value);
static float clampf_local(float value, float min_value, float max_value);
static int apply_deadband(int value, int deadband);
static int slew_step(int current, int target, int step);
static void radial_deadzone(float *x, float *y, float dz);
static float shape_cubic(float value);
static HAL_StatusTypeDef PS2_Receive_JoyStick_Data(SPI_HandleTypeDef *hspi, uint8_t *pRxData, uint16_t size);
static HAL_StatusTypeDef PS2_TransferFrame(SPI_HandleTypeDef *hspi, const uint8_t *tx, uint8_t *rx, uint16_t size);
static void PS2_DelayShort(void);
static void PS2_GPIO_Init(void);
static HAL_StatusTypeDef PS2_EnterAnalogMode(SPI_HandleTypeDef *hspi);
static void update_button_commands(uint8_t angle_shift_byte, uint8_t dc_byte);

static void CAN_StartWithFilter(void);
static void clear_joystick_inputs(void);
static void stop_chassis_command(void);
static ChassisCommand joystick_to_chassis(const joystick_variables *joy, uint8_t use_cubic);
static Wheel_variables mecanum_inverse(const ChassisCommand *cmd);
static void normalize_wheel_targets(Wheel_variables *wheel, float limit);
static void can_transmit_triple(CAN_HandleTypeDef *hcan_ptr, uint16_t id, int16_t msg1, int16_t msg2, int16_t msg3);

void Update_Setpoints(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int map_joystick(uint8_t value)
{
	return (int)value - 128;
}

static int apply_deadband(int value, int deadband)
{
	if (value > -deadband && value < deadband) {
		return 0;
	}
	return value;
}

static float clampf_local(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

static int slew_step(int current, int target, int step)
{
	const int delta = target - current;

	if (delta > step) {
		return current + step;
	}
	if (delta < -step) {
		return current - step;
	}
	return target;
}

static void radial_deadzone(float *x, float *y, float dz)
{
	const float magnitude = sqrtf((*x) * (*x) + (*y) * (*y));

	if (magnitude < dz) {
		*x = 0.0f;
		*y = 0.0f;
		return;
	}

	const float scaled = (magnitude - dz) / (1.0f - dz);
	const float inv_magnitude = 1.0f / magnitude;

	*x = (*x) * inv_magnitude * scaled;
	*y = (*y) * inv_magnitude * scaled;
}

static float shape_cubic(float value)
{
	return value * value * value;
}

static void PS2_DelayShort(void)
{
	for (volatile uint32_t i = 0; i < 1200U; i++) {
		__NOP();
	}
}

static void PS2_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = SPI3_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);
}

static HAL_StatusTypeDef PS2_TransferFrame(SPI_HandleTypeDef *hspi, const uint8_t *tx, uint8_t *rx, uint16_t size)
{
	HAL_StatusTypeDef status = HAL_OK;

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
	PS2_DelayShort();
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	PS2_DelayShort();

	for (uint16_t i = 0; i < size; i++) {
		status = HAL_SPI_TransmitReceive(hspi, (uint8_t *)&tx[i], &rx[i], 1U, PS2_SPI_TIMEOUT_MS);
		if (status != HAL_OK) {
			break;
		}
		PS2_DelayShort();
	}

	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
	return status;
}

static HAL_StatusTypeDef PS2_Receive_JoyStick_Data(SPI_HandleTypeDef *hspi, uint8_t *pRxData, uint16_t size)
{
	return PS2_TransferFrame(hspi, TX, pRxData, size);
}

static HAL_StatusTypeDef PS2_EnterAnalogMode(SPI_HandleTypeDef *hspi)
{
	uint8_t rx[PS2_FRAME_SIZE];
	uint8_t enter_config[PS2_FRAME_SIZE] = {0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t set_analog[PS2_FRAME_SIZE] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
	uint8_t exit_config[PS2_FRAME_SIZE] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

	for (uint8_t attempt = 0; attempt < PS2_INIT_RETRIES; attempt++) {
		if (PS2_TransferFrame(hspi, enter_config, rx, PS2_FRAME_SIZE) != HAL_OK) {
			osDelay(20);
			continue;
		}

		osDelay(20);

		if (PS2_TransferFrame(hspi, set_analog, rx, PS2_FRAME_SIZE) != HAL_OK) {
			osDelay(20);
			continue;
		}

		osDelay(20);

		if (PS2_TransferFrame(hspi, exit_config, rx, PS2_FRAME_SIZE) == HAL_OK) {
			osDelay(20);
			return HAL_OK;
		}

		osDelay(20);
	}

	return HAL_ERROR;
}

static void CAN_StartWithFilter(void)
{
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}
}

static void clear_joystick_inputs(void)
{
	raw_joy_data.LX = 0;
	raw_joy_data.LY = 0;
	raw_joy_data.RX = 0;
	raw_joy_data.RY = 0;
}

static void stop_chassis_command(void)
{
	/* Controller data буруу үед өмнөх явалт/эргэлтийн command-ийг шууд устгана */
	clear_joystick_inputs();
	filtered_joy_data.LX = 0;
	filtered_joy_data.LY = 0;
	filtered_joy_data.RX = 0;
	filtered_joy_data.RY = 0;
}

//static void update_button_commands(uint8_t angle_shift_byte, uint8_t dc_byte)
//{
//	shift_cmd = 0;
//
//
//	if (RX[3] == 239U) {
//		shift_cmd = -6000;
//	}
//	else if (RX[3] == 191U) {
//		shift_cmd = 6000;
//	}
//	if(RX[3]==223U && RX[4]==251)
//	{
//		angle_setpoint=fminf(angle_setpoint + 2.0f, 250.0f);
//
//	}
//	else if(RX[3]==191 && RX[4]==251)
//	{
//		angle_setpoint=fmaxf(angle_setpoint -2.0f, 0.0f);
//	}
//	else  if (RX[3] == 127U)
//	{
//		angle_setpoint = 73;
//	}
//	else if (RX[3] == 223U) {
//		angle_setpoint  = 0;
//	}
//
//	if(RX[4]==251U && RX[3]==239)
//	{
//		dc_cmd=1;
//	}
//	else if(RX[4]==251U && RX[3]==191)
//	{
//		dc_cmd=0;
//	}
//	//	if (dc_byte == 251U) {
//	//		if (btn251_state == 0) {
//	//			btn251_state = 1;
//	//			btn251_pressed_tick = osKernelGetTickCount();
//	//		}
//	//		else if (btn251_state == 1) {
//	//			if ((osKernelGetTickCount() - btn251_pressed_tick) >= 200) {
//	//				dc_cmd = !dc_cmd;
//	//				btn251_state = 2;
//	//			}
//	//		}
//	//	}
//	//	else {
//	//		btn251_state = 0;
//	//	}
//}
static void update_button_commands(uint8_t angle_shift_byte, uint8_t dc_byte)
{
    (void)angle_shift_byte;
    (void)dc_byte;

#define BUTTON_DEBOUNCE_MS 120U

    static uint8_t prev_rx3_event = 255U;
    static uint8_t prev_rx4_event = 255U;

    static uint32_t last_repeat_tick = 0U;
    static uint32_t last_event_tick = 0U;

    uint32_t now = osKernelGetTickCount();

    uint8_t l1_pressed = (RX[4] == 251U);

    /*
     * ======================================================
     * 1) L1 + LEFT / RIGHT repeat command
     * 80ms тутам +2 / -2 ажиллана.
     * ======================================================
     */
    if (l1_pressed)
    {
        if ((now - last_repeat_tick) >= BUTTON_DEBOUNCE_MS)
        {
            if (RX[3] == 127U)
            {
                /*
                 * L1 + LEFT
                 * angle_setpoint +2
                 */
                angle_setpoint = fminf(angle_setpoint + 3.0f, 250.0f);
                last_repeat_tick = now;
            }
            else if (RX[3] == 223U)
            {
                /*
                 * L1 + RIGHT
                 * angle_setpoint -2
                 */
                angle_setpoint = fmaxf(angle_setpoint - 3.0f, 0.0f);
                last_repeat_tick = now;
            }
        }
    }
//    if(RX[3]==254 && lastRX3 != 254){
//    	Display_color_state++;
//    	if(Display_color_state>1){           //odoogoor 00 01 gesen 2 tolovtei uchraas 1ees deesh tolovuud haagdsan.
//    		Display_color_state = 0;		 //nemehiig husvel Display_color_state>3 hurtel bichij bolno.
//    	}
//    	lastRX3 = RX[3]
//    }
    else if (((now - last_repeat_tick) >= 240)){
        if (RX[3] == 254U)
        {
        	Display_color_state++;
			if(Display_color_state>1){           //odoogoor 00 01 gesen 2 tolovtei uchraas 1ees deesh tolovuud haagdsan.
				Display_color_state = 0;		 //nemehiig husvel Display_color_state>3 hurtel bichij bolno.
			}
            last_repeat_tick = now;
        }
    }

    /*
     * ======================================================
     * 2) Дан UP / DOWN shift command
     * L1 дараагүй үед л ажиллана.
     * ======================================================
     */
    shift_cmd = 0;

    if (!l1_pressed)
    {
        if (RX[3] == 239U)
        {
            /*
             * UP
             */
            shift_cmd = -6000;
        }
        else if (RX[3] == 191U)
        {
            /*
             * DOWN
             */
            shift_cmd = 6000;
        }
    }

    /*
     * ======================================================
     * 3) One-shot debounce event
     * DC ON/OFF, angle preset зэрэг нэг удаагийн command.
     * ======================================================
     */
    if ((RX[3] == prev_rx3_event) && (RX[4] == prev_rx4_event))
    {
        return;
    }

    if ((now - last_event_tick) < BUTTON_DEBOUNCE_MS)
    {
        return;
    }

    prev_rx3_event = RX[3];
    prev_rx4_event = RX[4];
    last_event_tick = now;

    /*
     * ======================================================
     * 4) L1 + UP / DOWN one-shot command
     * ======================================================
     */
    if (l1_pressed)
    {
        if (RX[3] == 239U)
        {
            /*
             * L1 + UP
             * dc_cmd = 1
             */
            dc_cmd = 1;
        }
        else if (RX[3] == 191U)
        {
            /*
             * L1 + DOWN
             * dc_cmd = 0
             */
            dc_cmd = 0;
        }

        return;
    }

    /*
     * ======================================================
     * 5) Дан LEFT / RIGHT one-shot command
     * ======================================================
     */
    if (RX[3] == 127U)
    {
        /*
         * LEFT
         */
        angle_setpoint = 80;
    }
    else if (RX[3] == 223U)
    {
        /*
         * RIGHT
         */
        angle_setpoint = 0;
    }
}
static ChassisCommand joystick_to_chassis(const joystick_variables *joy, uint8_t use_cubic)
{
	ChassisCommand cmd;
	float vx_norm = clampf_local(-(float)joy->LY / JOY_AXIS_MAX, -1.0f, 1.0f);
	float vy_norm = clampf_local(-(float)joy->LX / JOY_AXIS_MAX, -1.0f, 1.0f);
	float wz_norm = clampf_local(-(float)joy->RX / JOY_AXIS_MAX, -1.0f, 1.0f);
	radial_deadzone(&vx_norm, &vy_norm, JOY_TRANSLATION_DZ);
	if (fabsf(wz_norm) < JOY_ROTATION_DZ) {
		wz_norm = 0.0f;
	}
	/* Шууд slow mode биш үед joystick response-ийг зөөлөн cubic curve болгоно */
	if (use_cubic != 0U) {
		vx_norm = shape_cubic(vx_norm);
		vy_norm = shape_cubic(vy_norm);
		wz_norm = shape_cubic(wz_norm);
	}
	cmd.vx = vx_norm * MAX_VX_MPS;
	cmd.vy = vy_norm * MAX_VY_MPS;
	cmd.wz = wz_norm * MAX_WZ_RADPS;
	return cmd;
}

static Wheel_variables mecanum_inverse(const ChassisCommand *cmd)
{

	wheel.FL = (cmd->vx - cmd->vy - CHASSIS_K_M * cmd->wz) / WHEEL_RADIUS_M;
	wheel.FR = (cmd->vx + cmd->vy + CHASSIS_K_M * cmd->wz) / WHEEL_RADIUS_M;
	wheel.RL = (cmd->vx + cmd->vy - CHASSIS_K_M * cmd->wz) / WHEEL_RADIUS_M;
	wheel.RR = (cmd->vx - cmd->vy + CHASSIS_K_M * cmd->wz) / WHEEL_RADIUS_M;

	return wheel;
}

static void normalize_wheel_targets(Wheel_variables *wheel, float limit)
{
	float max_mag = fabsf(wheel->FL);

	if (fabsf(wheel->FR) > max_mag) max_mag = fabsf(wheel->FR);
	if (fabsf(wheel->RL) > max_mag) max_mag = fabsf(wheel->RL);
	if (fabsf(wheel->RR) > max_mag) max_mag = fabsf(wheel->RR);

	if (max_mag > limit) {
		const float scale = limit / max_mag;
		wheel->FL *= scale;
		wheel->FR *= scale;
		wheel->RL *= scale;
		wheel->RR *= scale;
	}
}

static void can_transmit_triple(CAN_HandleTypeDef *hcan_ptr, uint16_t id, int16_t msg1, int16_t msg2, int16_t msg3)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t data[8] = {0};
	uint32_t pTxMailbox;

	if (hcan_ptr == NULL) {
		return;
	}

	if (HAL_CAN_GetState(hcan_ptr) != HAL_CAN_STATE_LISTENING) {
		return;
	}

	if (HAL_CAN_GetTxMailboxesFreeLevel(hcan_ptr) == 0U) {
		return;
	}

	tx_header.StdId = id;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 6;
	tx_header.TransmitGlobalTime = DISABLE;

	data[0] = (uint8_t)(msg1 >> 8);
	data[1] = (uint8_t)msg1;
	data[2] = (uint8_t)(msg2 >> 8);
	data[3] = (uint8_t)msg2;
	data[4] = (uint8_t)(msg3 >> 8);
	data[5] = (uint8_t)msg3;

	(void)HAL_CAN_AddTxMessage(hcan_ptr, &tx_header, data, &pTxMailbox);
}
#if 0
static uint8_t Find_Fixed_Step_By_Direction(float current_sp,
		const float *fixed_sp,
		uint8_t count,
		int8_t direction)
{
	/*
	 * FIXED_MODE дээр command ирэхэд одоогийн SP-ээс чиглэлтэйгээр дараагийн
	 * fixed байрлалыг сонгоно. FIXED_MODE руу орох мөчид SP-г шууд хөдөлгөхгүй.
	 */
	if (count == 0U) {
		return 1U;
	}

	if (direction > 0)
	{
		/* Дээш чиглэл: одоогийн SP-ээс дээш байгаа хамгийн ойр fixed байрлал */
		for (uint8_t i = 0; i < count; i++)
		{
			if (fixed_sp[i] > (current_sp + FIXED_STEP_EPS_MM))
			{
				return i + 1U;
			}
		}

		/* Аль хэдийн хамгийн дээд талд байвал сүүлийн fixed байрлал дээр үлдээнэ */
		return count;
	}

	if (direction < 0)
	{
		/* Доош чиглэл: одоогийн SP-ээс доош байгаа хамгийн ойр fixed байрлал */
		for (uint8_t i = count; i > 0U; i--)
		{
			if (fixed_sp[i - 1U] < (current_sp - FIXED_STEP_EPS_MM))
			{
				return i;
			}
		}

		/* Аль хэдийн хамгийн доод талд байвал эхний fixed байрлал дээр үлдээнэ */
		return 1U;
	}

	/* Direction буруу дамжсан онцгой тохиолдолд доод байрлал сонгоно */
	return 1U;

	/*
	 * counter 1-ээс эхэлж байгаа учраас +1 буцаана.
	 * fixed_sp array index бол 0-оос эхэлнэ.
	 */
}

void X_Axis_Set_Mode(AxisMode_t new_mode)
{
	if (X_AXIS_MODE == new_mode)
	{
		return;
	}

	/*
	 * STEP_MODE -> FIXED_MODE үед:
	 * Одоогийн STEP SP-д хамгийн ойр fixed step-ийг олж counter sync хийнэ.
	 * Гэхдээ SP-г шууд fixed утга руу өөрчлөхгүй.
	 */
	if (new_mode == FIXED_MODE)
	{
		/* FIXED_MODE руу орохдоо SP-г хөдөлгөхгүй; дараагийн command дээр fixed байрлал сонгоно */
	}

	/*
	 * FIXED_MODE -> STEP_MODE үед:
	 * SP аль хэдийн хамгийн сүүлд байсан setpoint дээрээ байгаа.
	 * Тиймээс SP-г өөрчлөхгүй.
	 */

	X_AXIS_MODE = new_mode;
}

void Y_Axis_Set_Mode(AxisMode_t new_mode)
{
	if (Y_AXIS_MODE == new_mode)
	{
		return;
	}

	if (new_mode == FIXED_MODE)
	{
		/* FIXED_MODE руу орохдоо SP-г хөдөлгөхгүй; дараагийн command дээр fixed байрлал сонгоно */
	}

	Y_AXIS_MODE = new_mode;
}

void Toggle_X_Axis_Mode(void)
{
	if (X_AXIS_MODE == STEP_MODE)
	{
		X_Axis_Set_Mode(FIXED_MODE);
	}
	else
	{
		X_Axis_Set_Mode(STEP_MODE);
	}
}

void Toggle_Y_Axis_Mode(void)
{
	if (Y_AXIS_MODE == STEP_MODE)
	{
		Y_Axis_Set_Mode(FIXED_MODE);
	}
	else
	{
		Y_Axis_Set_Mode(STEP_MODE);
	}
}

#endif

#if 0
void Update_Setpoints(void)
{
	/*
	 * Өмнөх RX[4] утгыг хадгална.
	 * Нэг товч удаан дарахад setpoint тасралтгүй нэмэгдэхээс хамгаална.
	 */
	static uint8_t prev_RX4 = 0;

	if (RX[4] != prev_RX4)
	{
		switch (RX[4])
		{
		/* X axis + чиглэл */
		case 127:
			if (X_AXIS_MODE == STEP_MODE)
			{
				X_AXIS_M3508_POS_PID.SP =
						fminf(X_AXIS_M3508_POS_PID.SP + 30.0f, 400.0f);
			}
			else if (X_AXIS_MODE == FIXED_MODE)
			{
				/* X+ үед одоогийн SP-ээс дээш байгаа хамгийн ойр fixed байрлал руу явна */
				X_AXIS_FIXED_STEP_COUNTER =
						Find_Fixed_Step_By_Direction(X_AXIS_M3508_POS_PID.SP,
								X_FIXED_SP,
								X_FIXED_STEP_COUNT,
								1);

				X_AXIS_M3508_POS_PID.SP =
						X_FIXED_SP[X_AXIS_FIXED_STEP_COUNTER - 1];
			}
			break;



			/* X axis - чиглэл */
		case 223:
			if (X_AXIS_MODE ==STEP_MODE)
			{
				X_AXIS_M3508_POS_PID.SP =
						fmaxf(X_AXIS_M3508_POS_PID.SP - 30.0f, 0.0f);
			}
			else if (X_AXIS_MODE == FIXED_MODE)
			{
				/* X- үед одоогийн SP-ээс доош байгаа хамгийн ойр fixed байрлал руу явна */
				X_AXIS_FIXED_STEP_COUNTER =
						Find_Fixed_Step_By_Direction(X_AXIS_M3508_POS_PID.SP,
								X_FIXED_SP,
								X_FIXED_STEP_COUNT,
								-1);

				X_AXIS_M3508_POS_PID.SP =
						X_FIXED_SP[X_AXIS_FIXED_STEP_COUNTER - 1];
			}
			break;

			/* Y axis + чиглэл */
		case 239:
			if (Y_AXIS_MODE == STEP_MODE)
			{
				Y_AXIS_M3508_POS_PID.SP =
						fminf(Y_AXIS_M3508_POS_PID.SP + 30.0f, 600.0f);
			}
			else if (Y_AXIS_MODE == FIXED_MODE)
			{
				/* Y+ үед одоогийн SP-ээс дээш байгаа хамгийн ойр fixed байрлал руу явна */
				Y_AXIS_FIXED_STEP_COUNTER =
						Find_Fixed_Step_By_Direction(Y_AXIS_M3508_POS_PID.SP,
								Y_FIXED_SP,
								Y_FIXED_STEP_COUNT,
								1);

				Y_AXIS_M3508_POS_PID.SP =
						Y_FIXED_SP[Y_AXIS_FIXED_STEP_COUNTER - 1];
			}
			break;

			/* Y axis - чиглэл */
		case 191:
			if (Y_AXIS_MODE == STEP_MODE)
			{
				Y_AXIS_M3508_POS_PID.SP =
						fmaxf(Y_AXIS_M3508_POS_PID.SP - 30.0f, 0.0f);
			}
			else if (Y_AXIS_MODE == FIXED_MODE)
			{
				/* Y- үед одоогийн SP-ээс доош байгаа хамгийн ойр fixed байрлал руу явна */
				Y_AXIS_FIXED_STEP_COUNTER =
						Find_Fixed_Step_By_Direction(Y_AXIS_M3508_POS_PID.SP,
								Y_FIXED_SP,
								Y_FIXED_STEP_COUNT,
								-1);

				Y_AXIS_M3508_POS_PID.SP =
						Y_FIXED_SP[Y_AXIS_FIXED_STEP_COUNTER - 1];
			}
			break;

			/* Gripper close */
		case 247:
			GRIPPER_POS_PID.SP = 35.0f;
			CLOSE_FLAG = 2;
			break;

			/* Gripper open */
		case 253:
			GRIPPER_POS_PID.SP = 0.0f;
			CLOSE_FLAG = 1;
			break;

		default:
			break;
		}
	}

	prev_RX4 = RX[4];
}
#endif

//void Update_Setpoints(void)
//{
//	/*
//	 * Өмнөх RX[4] утгыг хадгална.
//	 * Нэг товч удаан дарахад fixed байрлал тасралтгүй солигдохоос хамгаална.
//	 */
//	static uint8_t prev_RX4 = 0;
//
//	if (RX[4] != prev_RX4)
//	{
//		switch (RX[4])
//		{
//		/* X axis дараагийн fixed байрлал руу шилжинэ */
//		case 127:
//			if (X_AXIS_FIXED_STEP_COUNTER < X_FIXED_STEP_COUNT)
//			{
//				X_AXIS_FIXED_STEP_COUNTER++;
//			}
//			X_AXIS_M3508_POS_PID.SP = X_FIXED_SP[X_AXIS_FIXED_STEP_COUNTER - 1U];
//			break;
//
//			/* X axis өмнөх fixed байрлал руу шилжинэ */
//		case 223:
//			if (X_AXIS_FIXED_STEP_COUNTER > 1U)
//			{
//				X_AXIS_FIXED_STEP_COUNTER--;
//			}
//			X_AXIS_M3508_POS_PID.SP = X_FIXED_SP[X_AXIS_FIXED_STEP_COUNTER - 1U];
//			break;
//
//			/* Y axis дараагийн fixed байрлал руу шилжинэ */
//		case 239:
//			if (Y_AXIS_FIXED_STEP_COUNTER < Y_FIXED_STEP_COUNT)
//			{
//				Y_AXIS_FIXED_STEP_COUNTER++;
//			}
//			Y_AXIS_M3508_POS_PID.SP = Y_FIXED_SP[Y_AXIS_FIXED_STEP_COUNTER - 1U];
//			break;
//
//			/* Y axis өмнөх fixed байрлал руу шилжинэ */
//		case 191:
//			if (Y_AXIS_FIXED_STEP_COUNTER > 1U)
//			{
//				Y_AXIS_FIXED_STEP_COUNTER--;
//			}
//			Y_AXIS_M3508_POS_PID.SP = Y_FIXED_SP[Y_AXIS_FIXED_STEP_COUNTER - 1U];
//			break;
//
//			/* Gripper close */
//		case 247:
//			GRIPPER_POS_PID.SP = 35.0f;
//			CLOSE_FLAG = 2;
//			break;
//
//			/* Gripper open */
//		case 253:
//			GRIPPER_POS_PID.SP = 0.0f;
//			CLOSE_FLAG = 1;
//			break;
//
//		default:
//			break;
//		}
//	}
//
//	prev_RX4 = RX[4];
//}

void Update_Setpoints(void)
{
#define SETPOINT_REPEAT_MS 120U

    static uint8_t prev_RX4 = 0;
    static uint32_t last_repeat_tick = 0U;

    uint32_t now = osKernelGetTickCount();

    /*
     * ======================================================
     * 1) R1 + товч: manual SP +30 / -30 repeat
     * ======================================================
     */
    if ((now - last_repeat_tick) >= SETPOINT_REPEAT_MS)
    {
        switch (RX[4])
        {
        /*
         * R1 + TRIANGLE
         * X SP +30
         */
        case 119:
            X_AXIS_M3508_POS_PID.SP =
                fminf(X_AXIS_M3508_POS_PID.SP + 30.0f, 400.0f);
            last_repeat_tick = now;
            break;

        /*
         * R1 + X
         * X SP -30
         */
        case 215:
            X_AXIS_M3508_POS_PID.SP =
                fmaxf(X_AXIS_M3508_POS_PID.SP - 30.0f, 0.0f);
            last_repeat_tick = now;
            break;

        /*
         * R1 + CIRCLE
         * Y SP +30
         */
        case 231:
            Y_AXIS_M3508_POS_PID.SP =
                fminf(Y_AXIS_M3508_POS_PID.SP + 30.0f, 600.0f);
            last_repeat_tick = now;
            break;

        /*
         * R1 + SQUARE / RECTANGLE
         * Y SP -30
         */
        case 183 :
            Y_AXIS_M3508_POS_PID.SP =
                fmaxf(Y_AXIS_M3508_POS_PID.SP - 30.0f, 0.0f);
            last_repeat_tick = now;
            break;


        }
    }

    /*
     * ======================================================
     * 2) R1 дараагүй үе: fixed step
     * Нэг дарахад нэг удаа л ажиллана.
     * ======================================================
     */
    if (RX[4] != prev_RX4)
    {
        switch (RX[4])
        {
        /*
         * TRIANGLE
         * X fixed next
         */
        case 127:
            if (X_AXIS_FIXED_STEP_COUNTER < X_FIXED_STEP_COUNT)
            {
                X_AXIS_FIXED_STEP_COUNTER++;
            }

            X_AXIS_M3508_POS_PID.SP =
                X_FIXED_SP[X_AXIS_FIXED_STEP_COUNTER - 1U];
            break;

        /*
         * X
         * X fixed previous
         */
        case 223:
            if (X_AXIS_FIXED_STEP_COUNTER > 1U)
            {
                X_AXIS_FIXED_STEP_COUNTER--;
            }

            X_AXIS_M3508_POS_PID.SP =
                X_FIXED_SP[X_AXIS_FIXED_STEP_COUNTER - 1U];
            break;

        /*
         * CIRCLE
         * Y fixed next
         */
        case 239:
            if (Y_AXIS_FIXED_STEP_COUNTER < Y_FIXED_STEP_COUNT)
            {
                Y_AXIS_FIXED_STEP_COUNTER++;
            }

            Y_AXIS_M3508_POS_PID.SP =
                Y_FIXED_SP[Y_AXIS_FIXED_STEP_COUNTER - 1U];
            break;

        /*
         * SQUARE / RECTANGLE
         * Y fixed previous
         */
        case 191:
            if (Y_AXIS_FIXED_STEP_COUNTER > 1U)
            {
                Y_AXIS_FIXED_STEP_COUNTER--;
            }

            Y_AXIS_M3508_POS_PID.SP =
                Y_FIXED_SP[Y_AXIS_FIXED_STEP_COUNTER - 1U];
            break;

        /*
         * Gripper close
         */
            /*
             * Gripper open / close toggle
             */
            case 253:
                if (CLOSE_FLAG != 2)
                {
                    GRIPPER_POS_PID.SP = 35.0f;
                    CLOSE_FLAG = 2;
                }
                else
                {
                    GRIPPER_POS_PID.SP = 0.0f;
                    CLOSE_FLAG = 1;
                }
                break;
        }
    }

    prev_RX4 = RX[4];
}
//void Update_Setpoints(void)
//{
//	/*
//	 * Өмнөх RX[4] утгыг хадгална.
//	 * Нэг товч удаан дарахад fixed байрлал тасралтгүй солигдохоос хамгаална.
//	 */
//	static uint8_t prev_RX4 = 0;
//
//	if (RX[4] != prev_RX4)
//	{
//		switch (RX[4])
//		{
//		/* X axis дараагийн fixed байрлал руу шилжинэ */
//		case 127:
//			if (X_AXIS_FIXED_STEP_COUNTER < X_FIXED_STEP_COUNT)
//			{
//				X_AXIS_FIXED_STEP_COUNTER++;
//			}
//			X_AXIS_M3508_POS_PID.SP =
//					X_FIXED_SP[X_AXIS_FIXED_STEP_COUNTER - 1U];
//			break;
//
//			/* X axis өмнөх fixed байрлал руу шилжинэ */
//		case 223:
//			if (X_AXIS_FIXED_STEP_COUNTER > 1U)
//			{
//				X_AXIS_FIXED_STEP_COUNTER--;
//			}
//			X_AXIS_M3508_POS_PID.SP =
//					X_FIXED_SP[X_AXIS_FIXED_STEP_COUNTER - 1U];
//			break;
//
//			/* Y axis дараагийн fixed байрлал руу шилжинэ */
//		case 239:
//			if (Y_AXIS_FIXED_STEP_COUNTER < Y_FIXED_STEP_COUNT)
//			{
//				Y_AXIS_FIXED_STEP_COUNTER++;
//			}
//			Y_AXIS_M3508_POS_PID.SP =
//					Y_FIXED_SP[Y_AXIS_FIXED_STEP_COUNTER - 1U];
//			break;
//
//			/* Y axis өмнөх fixed байрлал руу шилжинэ */
//		case 191:
//			if (Y_AXIS_FIXED_STEP_COUNTER > 1U)
//			{
//				Y_AXIS_FIXED_STEP_COUNTER--;
//			}
//			Y_AXIS_M3508_POS_PID.SP =
//					Y_FIXED_SP[Y_AXIS_FIXED_STEP_COUNTER - 1U];
//			break;
//
//			/* Gripper close */
//		case 247:
//			GRIPPER_POS_PID.SP = 35.0f;
//			CLOSE_FLAG = 2;
//			break;
//
//			/* Gripper open */
//		case 253:
//			GRIPPER_POS_PID.SP = 0.0f;
//			CLOSE_FLAG = 1;
//			break;
//
//			/*
//			 * R1 + TRIANGLE
//			 * RX[4] = 231
//			 */
//		case 231:
//			/*
//			 * Энд R1 + TRIANGLE дарахад хийх үйлдлээ бичнэ.
//			 * Жишээ:
//			 */
//			X_AXIS_M3508_POS_PID.SP =
//					fminf(X_AXIS_M3508_POS_PID.SP + 30.0f, 400.0f);
//			break;
//
//			/*
//			 * R1 + RECTANGLE / SQUARE
//			 * RX[4] = 119
//			 */
//		case 119:
//			/*
//			 * Энд R1 + RECTANGLE дарахад хийх үйлдлээ бичнэ.
//			 */
//			X_AXIS_M3508_POS_PID.SP =
//					fmaxf(X_AXIS_M3508_POS_PID.SP - 30.0f, 0.0f);
//			break;
//
//			/*
//			 * R1 + CIRCLE
//			 * RX[4] = 215
//			 */
//		case 215:
//			/*
//			 * Энд R1 + CIRCLE дарахад хийх үйлдлээ бичнэ.
//			 */
//			Y_AXIS_M3508_POS_PID.SP =
//					fminf(Y_AXIS_M3508_POS_PID.SP + 30.0f, 600.0f);
//			break;
//
//			/*
//			 * R1 + X
//			 * RX[4] = 183
//			 */
//		case 183:
//			/*
//			 * Энд R1 + X дарахад хийх үйлдлээ бичнэ.
//			 */
//			// GRIPPER_POS_PID.SP = 0.0f;
//			// CLOSE_FLAG = 1;
//			Y_AXIS_M3508_POS_PID.SP =
//					fmaxf(Y_AXIS_M3508_POS_PID.SP -30.0f , 0.0f);
//			break;
//
//		default:
//			break;
//		}
//	}
//
//	prev_RX4 = RX[4];
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	CAN_StartWithFilter();
	PS2_GPIO_Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(CAN_Task, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(JoyStick_Task, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(Mecanum_Task, NULL, &myTask03_attributes);

  /* creation of X_Y_ARM_PID */
  X_Y_ARM_PIDHandle = osThreadNew(ARM_PID, NULL, &X_Y_ARM_PID_attributes);

  /* creation of Gripper */
  GripperHandle = osThreadNew(Gripper_task, NULL, &Gripper_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00302F47;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00302F47;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00302F47;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00302F47;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 7;
  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SPI4_CS_Pin|IR_SENSOR3_Pin|IR_SENSOR4_Pin|IR_SENSOR5_Pin
                          |PWM2_PIN1_Pin|PWM2_PIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ESP_SIGNAL_OUT1_Pin|SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|RELAY6_Pin|LD3_Pin|RELAY5_Pin
                          |ESP_SIGNAL_OUT2_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RELAY1_Pin|IR_SENSOR2_Pin|RELAY4_Pin|RELAY3_Pin
                          |MAX485_DE_RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, SPI5_CS_Pin|RELAY2_Pin|UNITREE485_RE_Pin|UNITREE485_DE_Pin
                          |PWM1_PIN2_Pin|PWM1_PIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IR_SENSOR1_GPIO_Port, IR_SENSOR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI4_CS_Pin IR_SENSOR3_Pin IR_SENSOR4_Pin IR_SENSOR5_Pin
                           PWM2_PIN1_Pin PWM2_PIN2_Pin */
  GPIO_InitStruct.Pin = SPI4_CS_Pin|IR_SENSOR3_Pin|IR_SENSOR4_Pin|IR_SENSOR5_Pin
                          |PWM2_PIN1_Pin|PWM2_PIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW2_Pin SW6_Pin */
  GPIO_InitStruct.Pin = SW2_Pin|SW6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : SW7_Pin SW8_Pin SW9_Pin */
  GPIO_InitStruct.Pin = SW7_Pin|SW8_Pin|SW9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SW10_Pin */
  GPIO_InitStruct.Pin = SW10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ESP_SIGNAL_OUT1_Pin SPI3_CS_Pin */
  GPIO_InitStruct.Pin = ESP_SIGNAL_OUT1_Pin|SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin RELAY6_Pin LD3_Pin RELAY5_Pin
                           ESP_SIGNAL_OUT2_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|RELAY6_Pin|LD3_Pin|RELAY5_Pin
                          |ESP_SIGNAL_OUT2_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW5_Pin SW11_Pin SW12_Pin IR_SENSOR6_Pin */
  GPIO_InitStruct.Pin = SW5_Pin|SW11_Pin|SW12_Pin|IR_SENSOR6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY1_Pin IR_SENSOR2_Pin RELAY4_Pin RELAY3_Pin
                           MAX485_DE_RE_Pin */
  GPIO_InitStruct.Pin = RELAY1_Pin|IR_SENSOR2_Pin|RELAY4_Pin|RELAY3_Pin
                          |MAX485_DE_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI5_CS_Pin RELAY2_Pin UNITREE485_RE_Pin UNITREE485_DE_Pin
                           PWM1_PIN2_Pin PWM1_PIN1_Pin */
  GPIO_InitStruct.Pin = SPI5_CS_Pin|RELAY2_Pin|UNITREE485_RE_Pin|UNITREE485_DE_Pin
                          |PWM1_PIN2_Pin|PWM1_PIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_SENSOR1_Pin */
  GPIO_InitStruct.Pin = IR_SENSOR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IR_SENSOR1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW4_Pin SW3_Pin IR_SENSOR7_Pin IR_SENSOR8_Pin */
  GPIO_InitStruct.Pin = SW4_Pin|SW3_Pin|IR_SENSOR7_Pin|IR_SENSOR8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_CAN_Task */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_CAN_Task */
void CAN_Task(void *argument)
{
  /* USER CODE BEGIN 5 */

	/* Infinite loop */
	for(;;)
	{
		Wheel_variables wheel_snapshot = wheel_targets;

		can_transmit(&hcan1,
				BOARD_ID_RECEIVE,
				(int16_t)wheel_snapshot.FL,
				(int16_t)wheel_snapshot.FR,
				(int16_t)wheel_snapshot.RL,
				(int16_t)wheel_snapshot.RR);
		can_transmit_triple(&hcan1,
				0x100,
				angle_setpoint,
				shift_cmd,
				dc_cmd);
		/* Gripper, X axis, Y axis motor command-уудыг RoboMaster group ID 0x200-аар илгээнэ */

		can_transmit(&hcan1,
				0x200,
				GRIPPER_OUT,
				0,
				X_AXIS_M3508_SPEED_PID.CO*(-12),
				Y_AXIS_M3508_SPEED_PID.CO*12);
		osDelay(20);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_JoyStick_Task */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_JoyStick_Task */
void JoyStick_Task(void *argument)
{
  /* USER CODE BEGIN JoyStick_Task */

	(void)PS2_EnterAnalogMode(&hspi3);

	const uint8_t joystick_threshold = 115;
	/* Infinite loop */
	for (;;)
	{
		const HAL_StatusTypeDef spi_status =
				PS2_Receive_JoyStick_Data(&hspi3, RX, PS2_FRAME_SIZE);

		if (RX[1] != joystick_threshold)
		{
			/* Header буруу ирвэл controller data-д итгэхгүй, chassis command-ийг шууд 0 болгоно */
			stop_chassis_command();
			osDelay(10);
			continue;
		}

		joystick_frame_valid =
				(uint8_t)((spi_status == HAL_OK) && (RX[2] == PS2_RESPONSE_HEADER));

		if (joystick_frame_valid == 0U)
		{
			/* Frame invalid үед өмнөх joystick утга гацахаас хамгаалж command-ийг шууд 0 болгоно */
			stop_chassis_command();

			osDelay(10);
			continue;
		}

		raw_joy_data.LX = apply_deadband(map_joystick(RX[7]), JOY_AXIS_INT_DEADBAND);
		raw_joy_data.LY = apply_deadband(map_joystick(RX[8]), JOY_AXIS_INT_DEADBAND);
		raw_joy_data.RX = apply_deadband(map_joystick(RX[5]), JOY_AXIS_INT_DEADBAND);
		raw_joy_data.RY = apply_deadband(map_joystick(RX[6]), JOY_AXIS_INT_DEADBAND);

		if(abs(raw_joy_data.LX) >= 5 && abs(raw_joy_data.LY) <= 5 && abs(raw_joy_data.RX) <= 5){
			slew_step(filtered_joy_data.LX, raw_joy_data.LX, 4);
			Scale_number = 0.3f;
		}else{
			Scale_number = 0.25f;
		}
		/* RX[4] == 254 үед slow direct mode асааж, бусад үед унтраана */
		direct_slow_mode = (RX[4] == 254U) ? 1U : 0U;

		/* RX[4] == 254 нь одоо зөвхөн mecanum slow/direct mode-д нөлөөлнө */
		if (direct_slow_mode != 0U)
		{
			/* Slow direct mode үед slew_step алгасаад raw joystick утгыг шууд хэрэглэнэ */
			filtered_joy_data.LX = raw_joy_data.LX;
			filtered_joy_data.LY = raw_joy_data.LY;
			filtered_joy_data.RX = raw_joy_data.RX;
			filtered_joy_data.RY = raw_joy_data.RY;
		}
		else
		{
			/* Normal mode үед joystick command-ийг slew_step-ээр зөөлрүүлнэ */
			filtered_joy_data.LX =
					slew_step(filtered_joy_data.LX, raw_joy_data.LX, JOY_AXIS_SLEW_STEP);

			filtered_joy_data.LY =
					slew_step(filtered_joy_data.LY, raw_joy_data.LY, JOY_AXIS_SLEW_STEP);

			filtered_joy_data.RX =
					slew_step(filtered_joy_data.RX, raw_joy_data.RX, JOY_AXIS_SLEW_STEP);

			filtered_joy_data.RY =
					slew_step(filtered_joy_data.RY, raw_joy_data.RY, JOY_AXIS_SLEW_STEP);
		}

		update_button_commands(RX[3], RX[4]);

		/*		DELGETSEER HARUULAH TOLOVIIG ESP32 LUU YVUULAH GPIO		*/
		switch(Display_color_state){
			case 0: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
					break;
			case 1: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
					break;
			case 2: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
					break;
			case 3: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
					break;
		}

		if ((RX[3] == 247U) && (prev_reverse_button_rx3 != 247U))
		{
			mecanum_direction *= -1;
		}

		prev_reverse_button_rx3 = RX[3];

#if 0
		/*
		 * Mode button hold:
		 * RX[4] 254 болсон мөчид л нэг удаа ажиллана.
		 */
		if (direct_slow_mode != 0U)
		{
			/* RX[4] == 254 дарж байх хугацаанд mecanum slow mode + arm STEP_MODE зэрэг идэвхтэй байна */
			X_Axis_Set_Mode(STEP_MODE);
			Y_Axis_Set_Mode(STEP_MODE);
		}
		else
		{
			/* RX[4] != 254 үед arm fixed байрлалын command mode руу буцна */
			X_Axis_Set_Mode(FIXED_MODE);
			Y_Axis_Set_Mode(FIXED_MODE);
		}
#endif

		Update_Setpoints();

		osDelay(10);
	}

  /* USER CODE END JoyStick_Task */
}

/* USER CODE BEGIN Header_Mecanum_Task */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Mecanum_Task */
void Mecanum_Task(void *argument)
{
  /* USER CODE BEGIN Mecanum_Task */
	/* Infinite loop */
	for(;;)
	{
		joystick_variables joy_snapshot;
		ChassisCommand cmd;
		Wheel_variables wheel_cmd;

		joy_snapshot = filtered_joy_data;

		/* Slow direct mode үед shape_cubic алгасаж, normal үед cubic curve хэрэглэнэ */
		cmd = joystick_to_chassis(&joy_snapshot, (uint8_t)(direct_slow_mode == 0U));

		if (mecanum_direction == -1)
		{
			cmd.vx *= -1.0f;
			cmd.vy *= -1.0f;
		}

		wheel_cmd = mecanum_inverse(&cmd);

		if(RX[4] == 254)
		{
			wheel_cmd.FL *= Scale_number;
			wheel_cmd.FR *= Scale_number;
			wheel_cmd.RL *= Scale_number;
			wheel_cmd.RR *= Scale_number;
		}

		normalize_wheel_targets(&wheel_cmd, WHEEL_CMD_LIMIT);

		wheel_targets = wheel_cmd;

		osDelay(10);

	}
  /* USER CODE END Mecanum_Task */
}

/* USER CODE BEGIN Header_ARM_PID */
/**
 * @brief Function implementing the X_Y_ARM_PID thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ARM_PID */
void ARM_PID(void *argument)
{
  /* USER CODE BEGIN ARM_PID */
	/* X axis болон Y axis-ийн encoder timer-үүдийг эхлүүлж байна.
	 * htim4 -> X axis encoder
	 * htim3 -> Y axis encoder
	 */
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	/* Encoder speed estimate хийхэд ямар timer ашиглахыг зааж өгнө */

	X_AXIS_M3508_ENCODER.timer_address=&htim4;
	Y_AXIS_M3508_ENCODER.timer_address=&htim3;
	/* Home буюу initial position олох flag-ууд */

	uint8_t x_init_flag=0,y_init_flag=0;
	/* Initial homing үед motor руу өгөх speed command */

	int16_t x_init_speed=0,y_init_speed=0;
	/* X axis speed PID тохиргоо
	 * KP = 4
	 * KI = 0
	 * KD = 0.5
	 * SP range = -200 ~ 200
	 * CO range = -10000 ~ 10000
	 */
	PID_init(&X_AXIS_M3508_SPEED_PID, 3, 0, 0.5, -200, 200, -10000, 10000);
	/* X axis position PID тохиргоо
	 * Position setpoint range = 0 ~ 400 mm
	 * Output нь speed PID-ийн setpoint болно.
	 */
	PID_init(&X_AXIS_M3508_POS_PID, 1.25, 0, 0.5, 0, 400, -200, 200);

	PID_init(&Y_AXIS_M3508_SPEED_PID, 3, 0, 0.5, -200, 200, -10000, 10000);
	/* Y axis position PID тохиргоо
	 * Position setpoint range = 0 ~ 605 mm
	 */
	PID_init(&Y_AXIS_M3508_POS_PID, 1.25, 0, 0.5, 0, 605, -200, 200);
	int32_t current=0;
	int32_t last=0;
	/* Y encoder-ийн overflow/underflow болон direction тооцоход ашиглана */

	/* ============================================================
	 * HOMING хэсэг
	 *
	 * Зорилго:
	 *   X/Y axis-ийг limit switch хүргэж 0 байрлал олох.
	 *
	 * SW8 -> X axis home switch
	 * SW2 -> Y axis home switch
	 *
	 * Switch SET үед motor-оо home чиглэл рүү явуулна.
	 * Switch RESET болоход home position хүрсэн гэж үзээд flag = 1 болгоно.
	 * ============================================================ */

	while(!(x_init_flag && y_init_flag))
	{
		if(HAL_GPIO_ReadPin(SW8_GPIO_Port,SW8_Pin)==GPIO_PIN_SET)
		{
			x_init_speed=3000;
		}
		else
		{
			x_init_speed=0;
			x_init_flag=1;
		}
		if(HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin)==GPIO_PIN_SET)
		{
			y_init_speed=-1000;
		}
		else
		{
			y_init_speed=0;
			y_init_flag=1;
		}
		can_transmit(&hcan1, 0x200, 0, 0, x_init_speed, y_init_speed);



	}
	/* Homing дууссаны дараа encoder counter-уудыг 0 болгоно.
	 * Энэ байрлал robot-ийн reference буюу эхлэл болно.
	 */
	TIM4->CNT=0;
	TIM3->CNT=0;
	/* Infinite loop */
	for(;;)
	{
		/* ============================================================
		 * Main ARM PID loop
		 *
		 * Давталт бүрт:
		 *   1. Limit switch шалгана
		 *   2. Encoder position уншина
		 *   3. Position PID тооцно
		 *   4. Position PID output-ийг speed PID setpoint болгоно
		 *   5. Encoder speed тооцно
		 *   6. Speed PID output гаргана
		 *   7. CAN_Task энэ output-ийг motor руу явуулна
		 * ============================================================ */
		/* X axis home switch дахин дарагдсан бол:
		 * - speed output 0 болгоно
		 * - encoder counter reset хийнэ
		 *
		 * Энэ нь axis mechanical limit дээр ирэхэд reference алдахгүй байхад хэрэгтэй.
		 */
		if(HAL_GPIO_ReadPin(SW8_GPIO_Port,SW8_Pin)==GPIO_PIN_RESET)
		{
			X_AXIS_M3508_SPEED_PID.CO=0;
			TIM4->CNT=0;
		}
		if(HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin)==GPIO_PIN_RESET)
		{
			Y_AXIS_M3508_SPEED_PID.CO=0;
			TIM3->CNT=0;
		}


		/* ================= X AXIS POSITION ================= */

		/* TIM4 encoder count-ийг mm position болгож хувиргана.
		 *
		 * X_PER_STEP_MM:
		 *   1 encoder step тутамд хэдэн mm шилжиж байгааг илэрхийлнэ.
		 */

		X_AXIS_M3508_POS_PID.PV=TIM4->CNT*X_PER_STEP_MM;

		current = TIM3->CNT;
		int16_t diff = (int16_t)(current - last);
		Encoder_SUM += diff*(-1);
		Y_AXIS_M3508_POS_PID.PV=Encoder_SUM;
		last = current;
		Y_AXIS_M3508_POS_PID.PV=Encoder_SUM*Y_PER_STEP_MM;

		/* X position PID тооцно.
		 * Input:
		 *   SP = хүссэн X position
		 *   PV = одоогийн X position
		 *
		 * Output:
		 *   CO = X speed PID-ийн setpoint
		 */
		PID_estimate(&X_AXIS_M3508_POS_PID);
		/* Position PID-ийн output-ийг speed PID-ийн target speed болгоно */
		X_AXIS_M3508_SPEED_PID.SP=X_AXIS_M3508_POS_PID.CO;
		/* X encoder-оос speed тооцно.
		 * 10 ms sampling time гэж үзэж байна.
		 */
		Estimate_Speed(&X_AXIS_M3508_ENCODER, 10);
		/* Encoder speed-ийг mm/s маягийн утга руу хувиргана.
		 *
		 * Speed нь tick/ms маягтай гарч байгаа тул:
		 *   Speed * X_PER_STEP_MM * 100
		 *
		 * 100 гэдэг нь 10ms sampling-тэй холбоотой scale.
		 */
		X_AXIS_M3508_SPEED_PID.PV=X_AXIS_M3508_ENCODER.Speed*X_PER_STEP_MM*100.0f;
		PID_estimate(&X_AXIS_M3508_SPEED_PID);
		PID_estimate(&Y_AXIS_M3508_POS_PID);
		Y_AXIS_M3508_SPEED_PID.SP=Y_AXIS_M3508_POS_PID.CO;
		Estimate_Speed(&Y_AXIS_M3508_ENCODER, 10);
		Y_AXIS_M3508_SPEED_PID.PV=Y_AXIS_M3508_ENCODER.Speed*Y_PER_STEP_MM*(-100.0f);
		PID_estimate(&Y_AXIS_M3508_SPEED_PID);
		osDelay(10);
	}
  /* USER CODE END ARM_PID */
}

/* USER CODE BEGIN Header_Gripper_task */
/**
 * @brief Function implementing the Gripper thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Gripper_task */
void Gripper_task(void *argument)
{
  /* USER CODE BEGIN Gripper_task */
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
	GRIPPER_ENCODER.timer_address=&htim8;
	PID_init(&GRIPPER_POS_PID, 3.5, 0, 0.05, 0, 60, -30, 30);
	PID_init(&GRIPPER_SPEED_PID, 5, 0, 0.05, -30, 30, -2000, 2000);
	uint8_t GRIPPER_FLAG=0;
	int16_t gripper_init_speed=0;
	/* ============================================================
	 * GRIPPER HOMING
	 *
	 * Зорилго:
	 *   Gripper-ийг limit switch SW12 хүртэл онгойлгож,
	 *   тэр байрлалыг 0 position гэж үзэх.
	 *
	 * SW12 == SET:
	 *   Limit switch хүрээгүй гэж үзээд gripper-ийг open чиглэлд хөдөлгөнө.
	 *
	 * SW12 != SET:
	 *   Limit switch хүрсэн гэж үзээд motor зогсоно.
	 * ============================================================ */
	while(!GRIPPER_FLAG)
	{
		if(HAL_GPIO_ReadPin(SW12_GPIO_Port,SW12_Pin)==GPIO_PIN_SET)
		{
			gripper_init_speed=-2000;
		}
		else
		{
			gripper_init_speed=0;
			GRIPPER_FLAG=1;
		}
		can_transmit(&hcan1, 0x200, gripper_init_speed, 0, 0, 0);
	}
	TIM8->CNT=0;
	/* Infinite loop */
	/* ============================================================
	 * Main gripper loop
	 *
	 * CLOSE_FLAG:
	 *   0 = idle
	 *   1 = open буюу home switch хүртэл онгойлгох
	 *   2 = close буюу position PID-ээр хаах
	 * ============================================================ */
	for(;;)
	{
		/* ========================================================
		 * CLOSE_FLAG == 1
		 *
		 * Gripper open command.
		 * Limit switch SW12 хүртэл open чиглэлд явна.
		 * Switch хүрмэгц:
		 *   - motor stop
		 *   - CLOSE_FLAG = 0
		 *   - encoder reset
		 * ======================================================== */
		if(CLOSE_FLAG==1)
		{
			if(HAL_GPIO_ReadPin(SW12_GPIO_Port,SW12_Pin)==GPIO_PIN_SET)
			{
				gripper_init_speed=-3000;
			}
			else
			{
				gripper_init_speed=0;
				CLOSE_FLAG=0;
				TIM8->CNT=0;

			}
			can_transmit(&hcan1, 0x200, gripper_init_speed, 0, 0, 0);
		}
		/* ========================================================
		 * CLOSE_FLAG == 2
		 *
		 * Gripper close command.
		 * Position PID + Speed PID cascade control ажиллана.
		 *
		 * GRIPPER_POS_PID.SP:
		 *   хүссэн gripper position
		 *
		 * GRIPPER_POS_PID.PV:
		 *   encoder-оос тооцсон одоогийн position
		 *
		 * GRIPPER_SPEED_PID.SP:
		 *   position PID-ийн output
		 *
		 * GRIPPER_SPEED_PID.PV:
		 *   encoder speed-ээс тооцсон бодит speed
		 *
		 * GRIPPER_OUT:
		 *   CAN_Task-аар motor руу илгээгдэх эцсийн command
		 * ======================================================== */
		else if(CLOSE_FLAG==2)
		{
			GRIPPER_POS_PID.PV= (int16_t)TIM8->CNT*GRIPPER_PER_STEP_MM*(-1);
			PID_estimate(&GRIPPER_POS_PID);
			GRIPPER_SPEED_PID.SP=GRIPPER_POS_PID.CO;
			Estimate_Speed(&GRIPPER_ENCODER, 15);
			GRIPPER_SPEED_PID.PV=GRIPPER_ENCODER.Speed*100.0f*GRIPPER_PER_STEP_MM;
			PID_estimate(&GRIPPER_SPEED_PID);
			GRIPPER_OUT=GRIPPER_SPEED_PID.CO*15;
		}
		osDelay(10);
	}
  /* USER CODE END Gripper_task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d
", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
