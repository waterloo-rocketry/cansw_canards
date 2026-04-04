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
#include "stm32h7xx_hal.h"

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
#define ADXRS_ST1_Pin GPIO_PIN_2
#define ADXRS_ST1_GPIO_Port GPIOE
#define ADXRS_ST2_Pin GPIO_PIN_3
#define ADXRS_ST2_GPIO_Port GPIOE
#define CHG_MUX_EN_Pin GPIO_PIN_4
#define CHG_MUX_EN_GPIO_Port GPIOE
#define EN_EXT_5V_Pin GPIO_PIN_5
#define EN_EXT_5V_GPIO_Port GPIOE
#define PG_EXT_5V_Pin GPIO_PIN_6
#define PG_EXT_5V_GPIO_Port GPIOE
#define LED_R_Pin GPIO_PIN_13
#define LED_R_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_15
#define LED_B_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define M_SYNC_IN1_Pin GPIO_PIN_1
#define M_SYNC_IN1_GPIO_Port GPIOH
#define M_SYNC_OUT_Pin GPIO_PIN_0
#define M_SYNC_OUT_GPIO_Port GPIOC
#define SD_CLK_Pin GPIO_PIN_1
#define SD_CLK_GPIO_Port GPIOC
#define ISENS_3V3_Pin GPIO_PIN_2
#define ISENS_3V3_GPIO_Port GPIOC
#define ISENS_5V_Pin GPIO_PIN_3
#define ISENS_5V_GPIO_Port GPIOC
#define SD_CMD_Pin GPIO_PIN_0
#define SD_CMD_GPIO_Port GPIOA
#define VSENS_RKT_Pin GPIO_PIN_1
#define VSENS_RKT_GPIO_Port GPIOA
#define VSENS_CHG_Pin GPIO_PIN_2
#define VSENS_CHG_GPIO_Port GPIOA
#define VSENS_USB_Pin GPIO_PIN_3
#define VSENS_USB_GPIO_Port GPIOA
#define ISENS_BAT2_Pin GPIO_PIN_4
#define ISENS_BAT2_GPIO_Port GPIOA
#define ISENS_BAT1_Pin GPIO_PIN_5
#define ISENS_BAT1_GPIO_Port GPIOA
#define FLASH_IO3_Pin GPIO_PIN_6
#define FLASH_IO3_GPIO_Port GPIOA
#define FLASH_IO2_Pin GPIO_PIN_7
#define FLASH_IO2_GPIO_Port GPIOA
#define VSENS_BAT1_Pin GPIO_PIN_4
#define VSENS_BAT1_GPIO_Port GPIOC
#define VSENS_BAT2_Pin GPIO_PIN_5
#define VSENS_BAT2_GPIO_Port GPIOC
#define FLASH_IO1_Pin GPIO_PIN_0
#define FLASH_IO1_GPIO_Port GPIOB
#define FLASH_IO0_Pin GPIO_PIN_1
#define FLASH_IO0_GPIO_Port GPIOB
#define FLASH_CLK_Pin GPIO_PIN_2
#define FLASH_CLK_GPIO_Port GPIOB
#define SERVO3_RX_Pin GPIO_PIN_7
#define SERVO3_RX_GPIO_Port GPIOE
#define SERVO3_TX_Pin GPIO_PIN_8
#define SERVO3_TX_GPIO_Port GPIOE
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOE
#define FLASH_CS_Pin GPIO_PIN_10
#define FLASH_CS_GPIO_Port GPIOE
#define PWM2_Pin GPIO_PIN_11
#define PWM2_GPIO_Port GPIOE
#define ADC_INT_Pin GPIO_PIN_12
#define ADC_INT_GPIO_Port GPIOE
#define PWM3_Pin GPIO_PIN_13
#define PWM3_GPIO_Port GPIOE
#define PWM4_Pin GPIO_PIN_14
#define PWM4_GPIO_Port GPIOE
#define ADXL_INT_Pin GPIO_PIN_15
#define ADXL_INT_GPIO_Port GPIOE
#define ADI_SCL_Pin GPIO_PIN_10
#define ADI_SCL_GPIO_Port GPIOB
#define ADI_SDA_Pin GPIO_PIN_11
#define ADI_SDA_GPIO_Port GPIOB
#define M_CAN_RX_Pin GPIO_PIN_12
#define M_CAN_RX_GPIO_Port GPIOB
#define M_CAN_TX_Pin GPIO_PIN_13
#define M_CAN_TX_GPIO_Port GPIOB
#define SD_D0_Pin GPIO_PIN_14
#define SD_D0_GPIO_Port GPIOB
#define SD_D1_Pin GPIO_PIN_15
#define SD_D1_GPIO_Port GPIOB
#define MOVELLA_TX_Pin GPIO_PIN_8
#define MOVELLA_TX_GPIO_Port GPIOD
#define MOVELLA_RX_Pin GPIO_PIN_9
#define MOVELLA_RX_GPIO_Port GPIOD
#define M_CAN_STB_Pin GPIO_PIN_10
#define M_CAN_STB_GPIO_Port GPIOD
#define R_CAN_STB_Pin GPIO_PIN_11
#define R_CAN_STB_GPIO_Port GPIOD
#define R_CAN_RX_Pin GPIO_PIN_12
#define R_CAN_RX_GPIO_Port GPIOD
#define R_CAN_TX_Pin GPIO_PIN_13
#define R_CAN_TX_GPIO_Port GPIOD
#define PWR_EN_Pin GPIO_PIN_6
#define PWR_EN_GPIO_Port GPIOC
#define BAT_FLT1_Pin GPIO_PIN_7
#define BAT_FLT1_GPIO_Port GPIOC
#define BAT_FLT2_Pin GPIO_PIN_8
#define BAT_FLT2_GPIO_Port GPIOC
#define BAR_SDA_Pin GPIO_PIN_9
#define BAR_SDA_GPIO_Port GPIOC
#define BAR_SCL_Pin GPIO_PIN_8
#define BAR_SCL_GPIO_Port GPIOA
#define SERVO2_TX_Pin GPIO_PIN_9
#define SERVO2_TX_GPIO_Port GPIOA
#define SERVO2_RX_Pin GPIO_PIN_10
#define SERVO2_RX_GPIO_Port GPIOA
#define USB_D__Pin GPIO_PIN_11
#define USB_D__GPIO_Port GPIOA
#define USB_D_A12_Pin GPIO_PIN_12
#define USB_D_A12_GPIO_Port GPIOA
#define DEBUG_SWDIO_Pin GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA
#define S_CAN_STB_Pin GPIO_PIN_15
#define S_CAN_STB_GPIO_Port GPIOA
#define SERVO1_TX_Pin GPIO_PIN_10
#define SERVO1_TX_GPIO_Port GPIOC
#define SERVO1_RX_Pin GPIO_PIN_11
#define SERVO1_RX_GPIO_Port GPIOC
#define S_CAN_RX_Pin GPIO_PIN_0
#define S_CAN_RX_GPIO_Port GPIOD
#define S_CAN_TX_Pin GPIO_PIN_1
#define S_CAN_TX_GPIO_Port GPIOD
#define SERVO4_TX_Pin GPIO_PIN_5
#define SERVO4_TX_GPIO_Port GPIOD
#define SERVO4_RX_Pin GPIO_PIN_6
#define SERVO4_RX_GPIO_Port GPIOD
#define IMU_INT1_Pin GPIO_PIN_7
#define IMU_INT1_GPIO_Port GPIOD
#define SD_D2_Pin GPIO_PIN_3
#define SD_D2_GPIO_Port GPIOB
#define SD_D3_Pin GPIO_PIN_4
#define SD_D3_GPIO_Port GPIOB
#define IMU_INT2_Pin GPIO_PIN_5
#define IMU_INT2_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_6
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_7
#define IMU_SDA_GPIO_Port GPIOB
#define MAG_SCL_Pin GPIO_PIN_8
#define MAG_SCL_GPIO_Port GPIOB
#define MAG_SDA_Pin GPIO_PIN_9
#define MAG_SDA_GPIO_Port GPIOB
#define INT_MAG_Pin GPIO_PIN_0
#define INT_MAG_GPIO_Port GPIOE
#define INT1_MAG_ACCEL_Pin GPIO_PIN_1
#define INT1_MAG_ACCEL_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
