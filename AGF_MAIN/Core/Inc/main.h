/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW1_Pin GPIO_PIN_3
#define SW1_GPIO_Port GPIOE
#define SPI4_CS_Pin GPIO_PIN_4
#define SPI4_CS_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_2
#define SW2_GPIO_Port GPIOF
#define SW6_Pin GPIO_PIN_6
#define SW6_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define SW7_Pin GPIO_PIN_0
#define SW7_GPIO_Port GPIOC
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define SW8_Pin GPIO_PIN_2
#define SW8_GPIO_Port GPIOC
#define SW9_Pin GPIO_PIN_3
#define SW9_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define SW10_Pin GPIO_PIN_4
#define SW10_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define SW5_Pin GPIO_PIN_0
#define SW5_GPIO_Port GPIOG
#define SW11_Pin GPIO_PIN_1
#define SW11_GPIO_Port GPIOG
#define IR_SENSOR3_Pin GPIO_PIN_10
#define IR_SENSOR3_GPIO_Port GPIOE
#define IR_SENSOR4_Pin GPIO_PIN_12
#define IR_SENSOR4_GPIO_Port GPIOE
#define IR_SENSOR5_Pin GPIO_PIN_13
#define IR_SENSOR5_GPIO_Port GPIOE
#define PWM2_PIN1_Pin GPIO_PIN_14
#define PWM2_PIN1_GPIO_Port GPIOE
#define PWM2_PIN2_Pin GPIO_PIN_15
#define PWM2_PIN2_GPIO_Port GPIOE
#define RELAY6_Pin GPIO_PIN_11
#define RELAY6_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define RELAY5_Pin GPIO_PIN_15
#define RELAY5_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define RELAY1_Pin GPIO_PIN_10
#define RELAY1_GPIO_Port GPIOD
#define IR_SENSOR2_Pin GPIO_PIN_11
#define IR_SENSOR2_GPIO_Port GPIOD
#define RELAY4_Pin GPIO_PIN_14
#define RELAY4_GPIO_Port GPIOD
#define RELAY3_Pin GPIO_PIN_15
#define RELAY3_GPIO_Port GPIOD
#define SW12_Pin GPIO_PIN_2
#define SW12_GPIO_Port GPIOG
#define SPI5_CS_Pin GPIO_PIN_3
#define SPI5_CS_GPIO_Port GPIOG
#define RELAY2_Pin GPIO_PIN_4
#define RELAY2_GPIO_Port GPIOG
#define UNITREE485_RE_Pin GPIO_PIN_7
#define UNITREE485_RE_GPIO_Port GPIOG
#define UNITREE485_DE_Pin GPIO_PIN_8
#define UNITREE485_DE_GPIO_Port GPIOG
#define IR_SENSOR1_Pin GPIO_PIN_8
#define IR_SENSOR1_GPIO_Port GPIOC
#define UNITREE485_TX_Pin GPIO_PIN_9
#define UNITREE485_TX_GPIO_Port GPIOA
#define UNITREE485_RX_Pin GPIO_PIN_10
#define UNITREE485_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define SW4_Pin GPIO_PIN_0
#define SW4_GPIO_Port GPIOD
#define SW3_Pin GPIO_PIN_1
#define SW3_GPIO_Port GPIOD
#define IR_SENSOR7_Pin GPIO_PIN_3
#define IR_SENSOR7_GPIO_Port GPIOD
#define IR_SENSOR8_Pin GPIO_PIN_4
#define IR_SENSOR8_GPIO_Port GPIOD
#define MAX485_RO_RX_Pin GPIO_PIN_6
#define MAX485_RO_RX_GPIO_Port GPIOD
#define MAX485_DE_RE_Pin GPIO_PIN_7
#define MAX485_DE_RE_GPIO_Port GPIOD
#define IR_SENSOR6_Pin GPIO_PIN_10
#define IR_SENSOR6_GPIO_Port GPIOG
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define PWM1_PIN2_Pin GPIO_PIN_12
#define PWM1_PIN2_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define PWM1_PIN1_Pin GPIO_PIN_15
#define PWM1_PIN1_GPIO_Port GPIOG
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
