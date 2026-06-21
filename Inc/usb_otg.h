/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_otg.h
  * @brief          : Header for usb_otg.c file.
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

#ifndef __USB_OTG_H__
#define __USB_OTG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

void MX_USB_OTG_HS_PCD_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_OTG_H__ */