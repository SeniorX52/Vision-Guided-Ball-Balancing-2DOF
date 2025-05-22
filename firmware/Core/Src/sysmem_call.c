///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2025 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include "servo.h"
//#include "data_handler.h"
//#include "MPU6050.h"
//#include "control.h"
//#include "math.h"
//#include "FreeRTOS.h"
//#include "timers.h"
//#include "task.h"
//#include "semphr.h"
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//I2C_HandleTypeDef hi2c2;
//
//TIM_HandleTypeDef htim1;
//
//UART_HandleTypeDef huart1;
//UART_HandleTypeDef huart2;
//
///* USER CODE BEGIN PV */
//
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_TIM1_Init(void);
//static void MX_USART2_UART_Init(void);
//static void MX_I2C2_Init(void);
//static void MX_USART1_UART_Init(void);
///* USER CODE BEGIN PFP */
//void UART_Task(void *parameters);
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//#define RX_BUFFER_SIZE  256
////uint8_t rx_buffer[RX_BUFFER_SIZE];
////static uint8_t bufferIndex=0;
//static volatile uint8_t rx_buffer[PACKET_SIZE + 3];  // Data + 2 start bytes + 1 checksum
//static volatile uint16_t rx_index = 0;
//static volatile bool sync_found = false;
//static volatile uint8_t prev_byte = 0;
//HAL_StatusTypeDef status;
//DataHandler dataHandler;
//MPU6050_HandleTypeDef mpu;
//SemaphoreHandle_t dataHandler_Mutex = NULL;
//QueueHandle_t uartRxQueue;
//static volatile uint8_t rx_byte;
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//
//  /* USER CODE BEGIN 1 */
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//   // PA1
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//   // PA0
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_TIM1_Init();
//  MX_USART2_UART_Init();
//  //MX_I2C2_Init();
//  //MX_USART1_UART_Init();
//  /* USER CODE BEGIN 2 */
//
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
////  mpu.hi2c = &hi2c2;
////  status = MPU6050_Init(&mpu);
////    if (status != HAL_OK) {
////  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
////  	  while (status) {
////  	          UART_sendString("Unable to initialize IMU Sensor: MPU6050, retrying...\n");
////  	          status = MPU6050_Init(&mpu);
////  	          HAL_Delay(2000);
////  	      }
////    }
//
//  //status = HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
////  if (status != HAL_OK) {
////	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
////	while (status) {
////		UART_sendString("UART Receive Interrupt Enable Error\n");
////		status = HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
////		HAL_Delay(2000);
////
////		}
////    }
//  //MPU6050_Calibrate();
//  Servo_Init();
//  HAL_NVIC_SetPriority(USART2_IRQn, 4, 0); // Must be <= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5)
//  HAL_NVIC_EnableIRQ(USART2_IRQn);
//  //HAL_Delay(1000);
//  DataHandler_Init(&dataHandler);
//  DataHandler_Reset(&dataHandler);
//  Control_init(&dataHandler);
//  dataHandler_Mutex = xSemaphoreCreateMutex();
//  uartRxQueue = xQueueCreate(128, sizeof(uint8_t));
//  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
//  xTaskCreate(UART_Task, "UART Task", 512, NULL, 2, NULL);
//  xTaskCreate(updateController_Task, "Update Controller", 512, NULL, 1, NULL);
//  vTaskStartScheduler();
//  //initHC();
//  while (1)
//  {
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//
//	  //updateController_noUpdate();
//	  //HAL_Delay(1);
//	  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
//
//
//
//  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
//  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief I2C2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_I2C2_Init(void)
//{
//
//  /* USER CODE BEGIN I2C2_Init 0 */
//
//  /* USER CODE END I2C2_Init 0 */
//
//  /* USER CODE BEGIN I2C2_Init 1 */
//
//  /* USER CODE END I2C2_Init 1 */
//  hi2c2.Instance = I2C2;
//  hi2c2.Init.ClockSpeed = 100000;
//  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
//  hi2c2.Init.OwnAddress1 = 0;
//  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//  hi2c2.Init.OwnAddress2 = 0;
//  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN I2C2_Init 2 */
//
//  /* USER CODE END I2C2_Init 2 */
//
//}
//
///**
//  * @brief TIM1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM1_Init(void)
//{
//
//  /* USER CODE BEGIN TIM1_Init 0 */
//
//  /* USER CODE END TIM1_Init 0 */
//
//  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM1_Init 1 */
//
//  /* USER CODE END TIM1_Init 1 */
//  htim1.Instance = TIM1;
//  htim1.Init.Prescaler = 0;
//  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim1.Init.Period = 65535;
//  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim1.Init.RepetitionCounter = 0;
//  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
//  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
//  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM1_Init 2 */
//
//  /* USER CODE END TIM1_Init 2 */
//
//}
//
///**
//  * @brief USART1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART1_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART1_Init 0 */
//
//  /* USER CODE END USART1_Init 0 */
//
//  /* USER CODE BEGIN USART1_Init 1 */
//
//  /* USER CODE END USART1_Init 1 */
//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 9600;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART1_Init 2 */
//
//  /* USER CODE END USART1_Init 2 */
//
//}
//
///**
//  * @brief USART2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART2_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART2_Init 0 */
//
//  /* USER CODE END USART2_Init 0 */
//
//  /* USER CODE BEGIN USART2_Init 1 */
//
//  /* USER CODE END USART2_Init 1 */
//  huart2.Instance = USART2;
//  huart2.Init.BaudRate =921600;
//  huart2.Init.WordLength = UART_WORDLENGTH_8B;
//  huart2.Init.StopBits = UART_STOPBITS_1;
//  huart2.Init.Parity = UART_PARITY_NONE;
//  huart2.Init.Mode = UART_MODE_TX_RX;
//  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART2_Init 2 */
//
//  /* USER CODE END USART2_Init 2 */
//
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  /* USER CODE BEGIN MX_GPIO_Init_1 */
//  /* USER CODE END MX_GPIO_Init_1 */
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//
//  /*Configure GPIO pins : PA0 PA1 */
//  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PB0 */
//  GPIO_InitStruct.Pin = GPIO_PIN_0;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN MX_GPIO_Init_2 */
//  /* USER CODE END MX_GPIO_Init_2 */
//}
//
///* USER CODE BEGIN 4 */
//char rx_buffer2[128];
//
//void HC05_SendCommand(UART_HandleTypeDef *hc05_uart, const char *cmd) {
//    HAL_UART_Transmit(hc05_uart, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
//    memset(rx_buffer2, 0, sizeof(rx_buffer2));
//    HAL_UART_Receive(hc05_uart, (uint8_t*)rx_buffer2, sizeof(rx_buffer2)-1, 200);
//    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\nHC-05> ", 8, HAL_MAX_DELAY);
//    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
//    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\nResponse> ", 12, HAL_MAX_DELAY);
//    HAL_UART_Transmit(&huart1, (uint8_t*)rx_buffer2, strlen(rx_buffer2), HAL_MAX_DELAY);
//    HAL_Delay(100);
//}
//
//void initHC() {
//    huart2.Init.BaudRate = 38400;
//    HAL_UART_Init(&huart2);
//    HAL_Delay(1000);
//    HC05_SendCommand(&huart2, "AT\r\n");
//    HC05_SendCommand(&huart2, "AT+UART=115200,1,0\r\n");
//    HC05_SendCommand(&huart2, "AT+NAME=AVENGERS\r\n");
//    HC05_SendCommand(&huart2, "AT+PSWD=1234\r\n");
//    HC05_SendCommand(&huart2, "AT+ROLE=0\r\n");
//    huart2.Init.BaudRate = 115200;
//    HAL_UART_Init(&huart2);
//    HC05_SendCommand(&huart2, "AT\r\n");
//}
//void updateController(void) {
//
//	static DataHandler* handler = &dataHandler;
//    if (DataHandler_IsNewDataAvailable(handler)) {
//        DataHandler_ClearNewDataFlag(handler);
//        updateController_noUpdate();
//    }
//}
//void updateController_Task(void *parameters){
//	const TickType_t period = pdMS_TO_TICKS(50);
//	TickType_t xLastTick = xTaskGetTickCount();
//	    while (1) {
//	    	xSemaphoreTake(dataHandler_Mutex, portMAX_DELAY);
//	    	updateController_noUpdate();
//	    	xSemaphoreGive(dataHandler_Mutex);
//	        vTaskDelayUntil(&xLastTick, period);
//
//	    }
//}
//void UART_Task(void *parameters) {
//    uint8_t current_byte;
//
//    while(1) {
//        if (xQueueReceive(uartRxQueue, &current_byte, portMAX_DELAY) == pdPASS) {
//            if (xSemaphoreTake(dataHandler_Mutex, portMAX_DELAY) == pdTRUE) {
//                if (!sync_found) {
//                    if (prev_byte == START_BYTE_1 && current_byte == START_BYTE_2) {
//                        sync_found = true;
//                        rx_index = 0;
//                        rx_buffer[rx_index++] = prev_byte;
//                        rx_buffer[rx_index++] = current_byte;
//                    }
//                    prev_byte = current_byte;
//                }
//                else {
//                    if (rx_index < sizeof(rx_buffer)) {
//                        rx_buffer[rx_index++] = current_byte;
//                    }
//
//                    if (rx_index == sizeof(rx_buffer)) {
//                        uint8_t checksum = 0;
//                        for (uint16_t i = 2; i < PACKET_SIZE + 2; i++) {
//                            checksum ^= rx_buffer[i];
//                        }
//
//                        if (checksum == rx_buffer[PACKET_SIZE + 2]) {
//                            DataHandler_ProcessUARTData(&dataHandler, rx_buffer + 2, PACKET_SIZE);
//                        }
//
//                        sync_found = false;
//                        rx_index = 0;
//                    }
//                }
//                xSemaphoreGive(dataHandler_Mutex);
//            }
//        }
//    }
//}
//
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    if (huart->Instance == USART2) {
//        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//        xQueueSendFromISR(uartRxQueue, &rx_byte, &xHigherPriorityTaskWoken);
//        HAL_UART_Receive_IT(huart, &rx_byte, 1);
//        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//    }
//}
//void updateController_noUpdate(void){
//			static DataHandler* handler = &dataHandler;
//	        switch(DataHandler_GetOperationMode(handler)) {
//	            case MODE_IDLE:
//	            	Servo_X_SetAngle(0);
//	            	Servo_Y_SetAngle(0);
//	            	Control_Reset();
//
//	            	break;
//	            case MODE_CALIBRATION:
//	            	Control_Reset();
//	            	switch(DataHandler_GetCalibrationMode(handler)){
//	            		case MODE_IDLE_CALIB:
//	            			Servo_X_Calib(DataHandler_GetCalibAngleX(handler));
//	            			Servo_Y_Calib(DataHandler_GetCalibAngleY(handler));
//	            			break;
//	            		case MODE_SAVE:
//	            			Servo_X_Save_Calib();
//	            			Servo_Y_Save_Calib();
//	            			break;
//	            		case MODE_RESET:
//	            			Servo_Reset_Calib();
//	            			break;
//	            	}
//	                break;
//
//	            case MODE_AUTOMATIC: {
//	            	switch (handler->trajectoryMode) {
//	            	        case MODE_POINT:
//	            	            Control_Loop();
//	            	            break;
//
//	            	        case MODE_CIRCLE:
//	            	            generate_circle_points();
//	            	            break;
//
//	            	        case MODE_INFINITY:
//	            	            generate_infinity_points();
//	            	            break;
//	            	        case MODE_PATH:
//	            	        	Control_Loop();
//	            	        	break;
//	            	        default:
//	            	        	//Control_Loop();
//	            	            break;
//	            	    }
//	                break;
//	            }
//
//	            case MODE_MANUAL:
//	            	Control_Reset();
//	            	Servo_X_SetAngle(DataHandler_GetManualAngleX(handler));
//	            	Servo_Y_SetAngle(DataHandler_GetManualAngleY(handler));
//	            	char debug_buf2[255];
//	            	snprintf(debug_buf2, sizeof(debug_buf2),
//	            	         "{\"Servo_X\":%.2f,\"Servo_Y\":%.2f,\"desired_X\":%.2f,\"desired_Y\":%.2f}",
//	            	         handler->manualAngleX, handler->manualAngleY,0.0f,0.0f);
//	            	UART_sendString(debug_buf2);
//	            	UART_sendString("\r\n");
//	            	memset(debug_buf2, 0, sizeof(debug_buf2));
//	                break;
//
//	            case MODE_SAFE:
//	            	Control_Reset();
//
//	                safeShutdownProcedure();
//	                break;
//
//	            default:
//	                break;
//	        }
//}
//void safeShutdownProcedure(void){
//
//}
//
////
////void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
////    if (huart->Instance == USART2) {
////        uint8_t current_byte = rx_buffer[rx_index];
////        if (!sync_found) {
////            if (prev_byte == START_BYTE_1 && current_byte == START_BYTE_2) {
////                sync_found = true;
////                rx_index = 0;
////                rx_buffer[rx_index++] = prev_byte;
////                rx_buffer[rx_index++] = current_byte;
////            }
////            prev_byte = current_byte;
////        }
////        else {
////            if (rx_index < sizeof(rx_buffer)) {
////                rx_buffer[rx_index++] = current_byte;
////            }
////
////            if (rx_index == sizeof(rx_buffer)) {
////                uint8_t checksum = 0;
////                for (uint16_t i = 2; i < PACKET_SIZE + 2; i++) {
////                    checksum ^= rx_buffer[i];
////                }
////
////                if (checksum == rx_buffer[PACKET_SIZE + 2]) {
////                    DataHandler_ProcessUARTData(&dataHandler, rx_buffer + 2, PACKET_SIZE);
////                }
////                sync_found = false;
////                rx_index = 0;
////            }
////        }
////        HAL_UART_Receive_IT(huart, &rx_buffer[rx_index], 1);
////    }
////}
//
//
//void UART_sendString(char *str) {
//    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
//}
//
//void UART_send_Char(char data) {
//    HAL_UART_Transmit(&huart2, (uint8_t*)&data, 1, HAL_MAX_DELAY);
//}
//void UART_receive_Data(char* data,uint16_t size)
//{
//    HAL_UART_Receive(&huart2, (uint8_t*)data, size, 1000);
//}
//
//void UART_send_Data(char* data)
//{
//    HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 1000);
//}
//void UART_sendNumber(int32_t TxNumber) {
//    if (TxNumber == 0) {
//        UART_send_Char('0');
//        return;
//    }
//
//    char buffer[12];
//    int index = 0;
//    if (TxNumber < 0) {
//        UART_send_Char('-');
//        TxNumber = -TxNumber;
//    }
//    while (TxNumber != 0) {
//        buffer[index++] = (TxNumber % 10) + '0';
//        TxNumber /= 10;
//    }
//    for (int i = index - 1; i >= 0; i--) {
//        UART_send_Char(buffer[i]);
//    }
//}
//void SendPIDValues(DataHandler* handler) {
//    char buffer[32];
//    snprintf(buffer, sizeof(buffer), "PID_KP: %.4f\r\n", DataHandler_GetPID_KP(handler));
//    UART_sendString(buffer);
//    memset(buffer, 0, sizeof(buffer));
//    snprintf(buffer, sizeof(buffer), "PID_KI: %.4f\r\n", DataHandler_GetPID_KI(handler));
//    UART_sendString(buffer);
//    memset(buffer, 0, sizeof(buffer));
//    snprintf(buffer, sizeof(buffer), "PID_KD: %.4f\r\n", DataHandler_GetPID_KD(handler));
//    UART_sendString(buffer);
//
//}
//
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
