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
#include "stm32l0xx_hal.h"

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
#define ENUM_SLAVE_ADDR 0x7F
#define PRIMARY_SLAVE_ADDR 0x56
#define _4G_UART_BAUD_RATE 115200
#define THFT_BATV_Pin GPIO_PIN_14
#define THFT_BATV_GPIO_Port GPIOC
#define GPS_EN_Pin GPIO_PIN_15
#define GPS_EN_GPIO_Port GPIOC
#define USART4_TX_Pin GPIO_PIN_0
#define USART4_TX_GPIO_Port GPIOA
#define USART4_RX_Pin GPIO_PIN_1
#define USART4_RX_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define SYS_STATUS_Pin GPIO_PIN_4
#define SYS_STATUS_GPIO_Port GPIOA
#define nTEST_Pin GPIO_PIN_5
#define nTEST_GPIO_Port GPIOA
#define _4G_nON_OFF_Pin GPIO_PIN_6
#define _4G_nON_OFF_GPIO_Port GPIOA
#define _4G_nRESET_Pin GPIO_PIN_7
#define _4G_nRESET_GPIO_Port GPIOA
#define _4G_nDTR_Pin GPIO_PIN_0
#define _4G_nDTR_GPIO_Port GPIOB
#define _4G_PWR_EN_Pin GPIO_PIN_1
#define _4G_PWR_EN_GPIO_Port GPIOB
#define I2C3_SCL_Pin GPIO_PIN_8
#define I2C3_SCL_GPIO_Port GPIOA
#define I2C1_SCL_Pin GPIO_PIN_9
#define I2C1_SCL_GPIO_Port GPIOA
#define I2C1_SDA_Pin GPIO_PIN_10
#define I2C1_SDA_GPIO_Port GPIOA
#define USART1_nCTS_Pin GPIO_PIN_11
#define USART1_nCTS_GPIO_Port GPIOA
#define USART1_nRTS_Pin GPIO_PIN_12
#define USART1_nRTS_GPIO_Port GPIOA
#define I2C3_SDA_Pin GPIO_PIN_4
#define I2C3_SDA_GPIO_Port GPIOB
#define INT_OUT_Pin GPIO_PIN_5
#define INT_OUT_GPIO_Port GPIOB
#define USART1_TX_Pin GPIO_PIN_6
#define USART1_TX_GPIO_Port GPIOB
#define USART1_RX_Pin GPIO_PIN_7
#define USART1_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//#define USART1_nRTS_Pin GPIO_PIN_12
//#define USART1_nRTS_GPIO_Port GPIOA
#define _4G_UART	USART1
#define LOG_UART	USART2
#define GPS_UART	USART4
#define SOCKET_MAX_RETRIES 5 // A.G. added
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
