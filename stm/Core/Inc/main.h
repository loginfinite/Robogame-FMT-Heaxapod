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
#include "stm32f1xx_hal.h"
#include "main.h"
#include "Servo.h"
#include <stdio.h>
#include <stdarg.h>
#include "movementTable.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef enum{
    normal_forward=1,
    normal_backward,
    turn_left,
    turn_right,
    rotate_left,
    rotate_right,
    high_backward,
    high_forward,
    default_pos,
    unlock_lf,
    unlock_lm,
    unlock_lh,
    unlock_rf,
    unlock_rm,
    unlock_rh,
    unlock_up_lf,
    unlock_up_lm,
    unlock_up_lh,
    unlock_up_rf,
    unlock_up_rm,
    unlock_up_rh,
    ripple_forward,
    tetrapod_forward,
    wave_forward,
    lay_down=25,
    lift_up,
    low_down,
    flower_convert=28,
    normal_rightward,
    normal_leftward,
    stand_up=31,
    flower_open=32,
    up_flower_open=33,
    unknown,
    up_flower_hold=35,
    up_crawl_hold=36,
    unknow2,
    asyn_rolling
}movement;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLUETOOTH_TX_Pin GPIO_PIN_2
#define BLUETOOTH_TX_GPIO_Port GPIOA
#define BLUETOOTH_RX_Pin GPIO_PIN_3
#define BLUETOOTH_RX_GPIO_Port GPIOA
#define SERVO_TX_Pin GPIO_PIN_9
#define SERVO_TX_GPIO_Port GPIOA
#define SERVO_RX_Pin GPIO_PIN_10
#define SERVO_RX_GPIO_Port GPIOA
#define UPPER_SERVO_TX_Pin GPIO_PIN_12
#define UPPER_SERVO_TX_GPIO_Port GPIOC
#define UPPER_SERVO_RX_Pin GPIO_PIN_2
#define UPPER_SERVO_RX_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
