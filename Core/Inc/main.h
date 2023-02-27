/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define _5V_SIG3_Pin GPIO_PIN_3
#define _5V_SIG3_GPIO_Port GPIOF
#define _5V_SIG1_Pin GPIO_PIN_0
#define _5V_SIG1_GPIO_Port GPIOC
#define _5V_SIG2_Pin GPIO_PIN_1
#define _5V_SIG2_GPIO_Port GPIOC
#define BATV_MEAS_Pin GPIO_PIN_2
#define BATV_MEAS_GPIO_Port GPIOC
#define VBAT_SIG0_Pin GPIO_PIN_0
#define VBAT_SIG0_GPIO_Port GPIOA
#define VBAT_SIG1_Pin GPIO_PIN_1
#define VBAT_SIG1_GPIO_Port GPIOA
#define VBAT_SIG2_Pin GPIO_PIN_2
#define VBAT_SIG2_GPIO_Port GPIOA
#define VBAT_SIG3_Pin GPIO_PIN_3
#define VBAT_SIG3_GPIO_Port GPIOA
#define VBAT_SIG4_Pin GPIO_PIN_4
#define VBAT_SIG4_GPIO_Port GPIOA
#define VBAT_SIG5_Pin GPIO_PIN_5
#define VBAT_SIG5_GPIO_Port GPIOA
#define VBAT_SIG6_Pin GPIO_PIN_6
#define VBAT_SIG6_GPIO_Port GPIOA
#define _5V_SIG0_Pin GPIO_PIN_7
#define _5V_SIG0_GPIO_Port GPIOA
#define _5V_MEAS_Pin GPIO_PIN_5
#define _5V_MEAS_GPIO_Port GPIOC
#define VBAT_EN6_Pin GPIO_PIN_0
#define VBAT_EN6_GPIO_Port GPIOB
#define VBAT_EN5_Pin GPIO_PIN_1
#define VBAT_EN5_GPIO_Port GPIOB
#define VBAT_EN4_Pin GPIO_PIN_2
#define VBAT_EN4_GPIO_Port GPIOB
#define VBAT_EN3_Pin GPIO_PIN_11
#define VBAT_EN3_GPIO_Port GPIOF
#define VBAT_EN2_Pin GPIO_PIN_14
#define VBAT_EN2_GPIO_Port GPIOF
#define VBAT_EN1_Pin GPIO_PIN_15
#define VBAT_EN1_GPIO_Port GPIOF
#define VBAT_EN0_Pin GPIO_PIN_0
#define VBAT_EN0_GPIO_Port GPIOG
#define PROG_TX_Pin GPIO_PIN_14
#define PROG_TX_GPIO_Port GPIOB
#define PROG_RX_Pin GPIO_PIN_15
#define PROG_RX_GPIO_Port GPIOB
#define SDMMC1_CD_Pin GPIO_PIN_2
#define SDMMC1_CD_GPIO_Port GPIOG
#define LED0_Pin GPIO_PIN_4
#define LED0_GPIO_Port GPIOG
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOG
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOG
#define BUS_SNS_Pin GPIO_PIN_9
#define BUS_SNS_GPIO_Port GPIOA
#define XBEE_TX_Pin GPIO_PIN_5
#define XBEE_TX_GPIO_Port GPIOD
#define XBEE_RX_Pin GPIO_PIN_6
#define XBEE_RX_GPIO_Port GPIOD
#define LED5_Pin GPIO_PIN_9
#define LED5_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
