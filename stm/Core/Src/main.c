/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
//


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Servo.h"
#include <stdio.h>
#include <stdarg.h>
#include "movementTable.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UPPER_SERVO_UART;
UART_HandleTypeDef SERVO_UART;
UART_HandleTypeDef BLUETOOTH_UART;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
extern char BLUETOOTH_RX_BUF[400];
extern char SERVO_RX_BUF[400];
extern char CMD_ARG_BUF[100];
extern char UPPER_SERVO_RX_BUF[400];
extern uint16_t CMD_FLAG;
extern uint16_t ANGLE[30];
extern int mutex_lock;
extern int mutex_lock_upper;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
    char * initInfo = "Stm init......\n\0";
    HAL_UART_Transmit(&BLUETOOTH_UART,initInfo,16,HAL_MAX_DELAY);
    __HAL_UART_CLEAR_IDLEFLAG(&SERVO_UART);
    __HAL_UART_ENABLE_IT(&SERVO_UART, UART_IT_IDLE | UART_IT_RXNE);
    HAL_UART_Receive_IT(&SERVO_UART, (uint8_t *)SERVO_RX_BUF, 100);
    __HAL_UART_CLEAR_IDLEFLAG(&BLUETOOTH_UART);
    __HAL_UART_ENABLE_IT(&BLUETOOTH_UART, UART_IT_IDLE | UART_IT_RXNE);
    HAL_UART_Receive_IT(&BLUETOOTH_UART, (uint8_t *)BLUETOOTH_RX_BUF, 100);
    __HAL_UART_CLEAR_IDLEFLAG(&UPPER_SERVO_UART);
    __HAL_UART_ENABLE_IT(&UPPER_SERVO_UART, UART_IT_IDLE | UART_IT_RXNE);
    HAL_UART_Receive_IT(&UPPER_SERVO_UART, (uint8_t *)UPPER_SERVO_RX_BUF, 100);

    Uart_Init(&SERVO_UART, &UPPER_SERVO_UART);
    CMD_FLAG = '0';
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* Infinite loop */
        while (1){
            switch (CMD_FLAG) {
                case 1:
                    movement_normal_forward();
                    break;
                case 2:
                    movement_normal_backward();
                    break;
                case 3:
                    movement_turn_left();
                    break;
                case 4:
                    movement_turn_right();
                    break;
                case 5:
                    movement_rotate_right();
                    break;
                case 6:
                    movement_rotate_left();
                    break;
                case 7:
                    movement_highlifting_backward();
                    break;
                case 8:
                    movement_highlifting_forward();
                    break;
                case 9:
                    movement_default_pos();
                    break;
                case 10:
                    unloadServos(3,0,1,2,3);//LF
                    CMD_FLAG = 0;
                    break;
                case 11:
                    unloadServos(3,0,4,5,6);//LM
                    CMD_FLAG = 0;
                    break;
                case 12:
                    unloadServos(3,0,10,11,12);//LH
                    CMD_FLAG = 0;
                    break;
                case 13:
                    unloadServos(3,0,16,17,18);//RF
                    CMD_FLAG = 0;
                    break;
                case 14:
                    unloadServos(3,0,7,8,9);//RM
                    CMD_FLAG = 0;
                    break;
                case 15:
                    unloadServos(3,0,13,14,15);//RH
                    CMD_FLAG = 0;
                    break;
                case 16:
                    unloadServos(2,19,20);//LF
                    CMD_FLAG = 0;
                    break;
                case 17:
                    unloadServos(2,29,30);//LM
                    CMD_FLAG = 0;
                    break;
                case 18:
                    unloadServos(2, 27,28);//LH
                    CMD_FLAG = 0;
                    break;
                case 19://RF
                    unloadServos(2,21,22);
                    CMD_FLAG = 0;
                    break;
                case 20://RM
                    unloadServos(2,23,24);
                    CMD_FLAG = 0;
                    break;
                case 21://RH
                    unloadServos(2,25,26);
                    CMD_FLAG = 0;
                    break;
                case 22:
                    movement_twist_rotate_right();break;
                case 23:
                    movement_tetrapod_forward();break;
                case 24:
                    movement_wave_forward();
                    break;
                case lay_down:
                    movement_convert();
                    moveServos(9, 1000
                            ,1 ,488
                            ,2 ,545
                            ,3 ,830
                            ,7 ,494
                            ,8 ,575
                            ,9 ,0
                            ,10 ,500
                            ,11 ,497
                            ,12 ,835
                    );
                    HAL_Delay(1000);
                    moveServos(9, 1000
                            ,1 ,488
                            ,2 ,170
                            ,3 ,330
                            ,7 ,494
                            ,8 ,950
                            ,9 ,490
                            ,10 ,500
                            ,11 ,122
                            ,12 ,335
                    );
                    HAL_Delay(1000);
                    moveServos(9, 400
                            ,4 ,488
                            ,5 ,517
                            ,6 ,755
                            ,13 ,436
                            ,14 ,540
                            ,15 ,825
                            ,16 ,496
                            ,17 ,570
                            ,18 ,760
                    );
                    HAL_Delay(400);
                    //
                    unloadServos(18,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18);
                    break;
                case 26:
                    movement_lift_up();
                    break;
                case 27:
                    movement_lowdown();
                    break;
                case flower_convert:
                    moveServos(12, 500
                            ,7 ,791
                            ,10 ,797
                            ,1 ,778
                            ,4 ,785
                            ,13 ,733
                            ,16 ,793
                            ,21 ,886
                            ,22 ,844
                            ,23 ,13
                            ,24 ,315
                            ,25 ,966
                            ,26 ,804
                    );
                    HAL_Delay(500);
                    moveServos(6,500,
                               19 ,984
                            ,20 ,809
                            ,27 ,1014
                            ,28 ,704
                            ,29 ,1049
                            ,30 ,544);
                    HAL_Delay(500);
                    break;
                case 29:
                    movement_normal_rightward();
                    break;
                case 30:
                    movement_normal_leftward();
                    break;
                case stand_up:
                    movement_stand_up();
                    movement_default_pos();
                    break;
                case flower_open:
                    unloadServos(6,0,3,6,9,12,15,18);
                    moveServos(24, 1000,7 ,494,8 ,554,10 ,500,11 ,517,1 ,488,2 ,565,4 ,488,5 ,537,13 ,436,14 ,560,16 ,496,17 ,590,19 ,880,20 ,809,27 ,910,28 ,704,29 ,945,30 ,544,21 ,795,22 ,844,23 ,105,24 ,315,25 ,875,26 ,804);
                    HAL_Delay(1000);
                    unloadServos(18,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18);
                    break;
                case 34:
                    unloadServos(18,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18);
                    moveServos(12, 500,19 ,505,20 ,809,27 ,535,28 ,704,29 ,570,30 ,544,21 ,420,22 ,844,23 ,480,24 ,315,25 ,500,26 ,804);
                    HAL_Delay(500);
                    moveServos(6, 1000,9 ,219,12 ,605,3 ,600,6 ,525,15 ,595,18 ,530);
                    HAL_Delay(1000);
                    unloadServos(18,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18);
                    break;
                case up_flower_open://STAGE1
                    movement_up_half_open();
                    break;
                case up_flower_hold://STAGE2
                    movement_up_hold();
                    break;
                case up_crawl_hold://STAGE3
                    movement_up_crawl_hold();
                    //rolling_down();
                    break;
                case asyn_rolling://STAGE4
                    movement_asyn_rolling();
                    //up_down_down();
                    break;
                case 39://屈膝
                    movement_leg_hold_up();
                    break;
                case 40://花动作1
                    convert_on_and_on();
                    break;
                case 41://花动作2
                    rolling_down();
                    break;
                case 42://花动作3
                    up_down_down();
                    break;
                case 43:
                    //
                    // movement_twist_rotate_right();
                case 110:
                    unloadServos(30,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30);
                    CMD_FLAG = 0;
                    break;
                case 111:
                    mutex_lock = 1;
                    mutex_lock_upper = 1;
                    getServoAngle(30,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30);
                    while (mutex_lock&&mutex_lock_upper){}
                    double angle[31];
                    convertAngleData(angle, ANGLE);
                    CMD_FLAG = 0;
                    break;
                default:break;
            }
        }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  UPPER_SERVO_UART.Instance = UART5;
  UPPER_SERVO_UART.Init.BaudRate = 9600;
  UPPER_SERVO_UART.Init.WordLength = UART_WORDLENGTH_8B;
  UPPER_SERVO_UART.Init.StopBits = UART_STOPBITS_1;
  UPPER_SERVO_UART.Init.Parity = UART_PARITY_NONE;
  UPPER_SERVO_UART.Init.Mode = UART_MODE_TX_RX;
  UPPER_SERVO_UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UPPER_SERVO_UART.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&UPPER_SERVO_UART) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  SERVO_UART.Instance = USART1;
  SERVO_UART.Init.BaudRate = 9600;
  SERVO_UART.Init.WordLength = UART_WORDLENGTH_8B;
  SERVO_UART.Init.StopBits = UART_STOPBITS_1;
  SERVO_UART.Init.Parity = UART_PARITY_NONE;
  SERVO_UART.Init.Mode = UART_MODE_TX_RX;
  SERVO_UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  SERVO_UART.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&SERVO_UART) != HAL_OK)
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
  BLUETOOTH_UART.Instance = USART2;
  BLUETOOTH_UART.Init.BaudRate = 9600;
  BLUETOOTH_UART.Init.WordLength = UART_WORDLENGTH_8B;
  BLUETOOTH_UART.Init.StopBits = UART_STOPBITS_1;
  BLUETOOTH_UART.Init.Parity = UART_PARITY_NONE;
  BLUETOOTH_UART.Init.Mode = UART_MODE_TX_RX;
  BLUETOOTH_UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  BLUETOOTH_UART.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&BLUETOOTH_UART) != HAL_OK)
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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
