/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
uint8_t TxData[8];
CAN_TxHeaderTypeDef TxHeader;
uint8_t RxData[8];
CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef sFilterConfig;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_CAN1_Init();
  MX_UART4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, SET);

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14; // if dual CAN (F4/F7/H7 series)

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
      Error_Handler();
  }



  HAL_TIM_Base_Start_IT(&htim6);

  if(HAL_CAN_Start(&hcan1) != HAL_OK){
	  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, RESET);
	  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, SET);
  	  Error_Handler();
  }

  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){
	  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, RESET);
	  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, SET);
  	  Error_Handler();
  }

  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK){
	  HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, RESET);
	  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, SET);
	  Error_Handler();
  }

    uint32_t TxMailBoxNum;

    TxData[0] = 'X';

    TxHeader.DLC = 1;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.ExtId = 0x00;
    TxHeader.StdId = 0x0AD;
    TxHeader.TransmitGlobalTime = DISABLE;

    if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBoxNum) != HAL_OK){
    	HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, RESET);
    	HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, SET);
    	Error_Handler();
    }

    TxData[0] = 'D';
    TxData[1] = 'E';
    TxData[2] = 'S';
    TxData[3] = 'D';

    TxHeader.DLC = 4;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.ExtId = 0x00;
    TxHeader.StdId = 0x0A9;
    TxHeader.TransmitGlobalTime = DISABLE;

    if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBoxNum) != HAL_OK){
    	HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, RESET);
    	HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, SET);
    	Error_Handler();
    }

    TxData[0] = 'S';
    TxData[1] = 'U';
    TxData[2] = 'N';
    TxData[3] = 'B';
    TxData[4] = 'E';
    TxData[5] = 'A';
    TxData[6] = 'M';

    TxHeader.DLC = 7;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.ExtId = 0x00;
    TxHeader.StdId = 0x0AA;
    TxHeader.TransmitGlobalTime = DISABLE;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBoxNum) != HAL_OK){
	      	HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, RESET);
	      	HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, SET);
	      	Error_Handler();
	  }
	  HAL_Delay(1000);
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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
