/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
struct str1
{
	uint32_t Pressed_Duration_Now;
	uint32_t Pressed_Duration_old;
	uint32_t Pressed_Duration_old_old;
	uint32_t Released_Duration_Now;
	uint32_t Released_Duration_old;
	uint32_t Morse_Bit;
	int Morse_bit[5];
	int Morse_bit_old[5];
	char Letter[10]; // words include Max 10 characters.
	char Word[10];
};
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Button_Pressed			(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == 1)	// Button pressed -> Led On -> IR Receiver = 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Filter_value			50
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t Morce_Bit_Reading_Flag = 0;
uint8_t Index_Of_Morse_Character = 0;
uint8_t Button_Released_Counter_Starter_Flag = 0;
uint8_t n = 0; 			//index of BUTTON.Letter
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct str1 BUTTON = {0, 0, 0, 0, 0, 0, {2, 2, 2, 2, 2}, {2, 2, 2, 2, 2}, {2, 2, 2, 2, 2, 2, 2, 2, 2, 2}, '\0', '\0'};
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  if (Button_Pressed)
  {
	  BUTTON.Pressed_Duration_Now++;
	  Button_Released_Counter_Starter_Flag = 1;
	  if(BUTTON.Released_Duration_Now != 0)
	  {
		  BUTTON.Released_Duration_old = BUTTON.Released_Duration_Now;
		  BUTTON.Released_Duration_Now = 0;
	  }

  }
  else
  {
	  if ( Button_Released_Counter_Starter_Flag == 1)
	  {
		  BUTTON.Released_Duration_Now++;
		  if (BUTTON.Pressed_Duration_Now != 0)
		  {
			  BUTTON.Pressed_Duration_old = BUTTON.Pressed_Duration_Now;
			  BUTTON.Pressed_Duration_Now = 0;
		  }

	  }
  }

  if (BUTTON.Pressed_Duration_old != BUTTON.Pressed_Duration_old_old)
  {
	  Morce_Bit_Reading_Flag = 1;
	  BUTTON.Pressed_Duration_old_old = BUTTON.Pressed_Duration_old;
  }

  if (Morce_Bit_Reading_Flag == 1)
  {
	  if(BUTTON.Pressed_Duration_old > Filter_value && BUTTON.Pressed_Duration_old < 300)
	  {
		  BUTTON.Morse_Bit = 0;
		  Morce_Bit_Reading_Flag = 0;

	  }
	  else if(BUTTON.Pressed_Duration_old > 300 )
	  {
		  BUTTON.Morse_Bit = 1;
		  Morce_Bit_Reading_Flag = 0;

	  }
	  BUTTON.Morse_bit[Index_Of_Morse_Character] = BUTTON.Morse_Bit;
	  Index_Of_Morse_Character++;


  }
  if (BUTTON.Released_Duration_Now == 3000)
  {
	  Index_Of_Morse_Character = 0;
	  for (int m = 0; m < 5; m++)
	  {
		  BUTTON.Morse_bit_old[m] = BUTTON.Morse_bit[m];
	  }
	  for (int m = 0;  m<5; m++)
	  {
		  BUTTON.Morse_bit[m] = 2;
	  }

	  if (BUTTON.Morse_bit_old[0] == 0 && BUTTON.Morse_bit_old[1] == 1 && BUTTON.Morse_bit_old[2] == 2 && BUTTON.Morse_bit_old[3] == 2 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'A';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 1 && BUTTON.Morse_bit_old[1] == 0 && BUTTON.Morse_bit_old[2] == 0 && BUTTON.Morse_bit_old[3] == 0 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'B';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 1 && BUTTON.Morse_bit_old[1] == 0 && BUTTON.Morse_bit_old[2] == 1 && BUTTON.Morse_bit_old[3] == 0 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'C';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 1 && BUTTON.Morse_bit_old[1] == 0 && BUTTON.Morse_bit_old[2] == 0 && BUTTON.Morse_bit_old[3] == 2 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'D';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 0 && BUTTON.Morse_bit_old[1] == 2 && BUTTON.Morse_bit_old[2] == 2 && BUTTON.Morse_bit_old[3] == 2 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'E';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 0 && BUTTON.Morse_bit_old[1] == 0 && BUTTON.Morse_bit_old[2] == 1 && BUTTON.Morse_bit_old[3] == 0 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'F';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 1 && BUTTON.Morse_bit_old[1] == 1 && BUTTON.Morse_bit_old[2] == 0 && BUTTON.Morse_bit_old[3] == 2 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'G';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 0 && BUTTON.Morse_bit_old[1] == 0 && BUTTON.Morse_bit_old[2] == 0 && BUTTON.Morse_bit_old[3] == 0 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'H';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 0 && BUTTON.Morse_bit_old[1] == 0 && BUTTON.Morse_bit_old[2] == 2 && BUTTON.Morse_bit_old[3] == 2 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'I';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 0 && BUTTON.Morse_bit_old[1] == 1 && BUTTON.Morse_bit_old[2] == 1 && BUTTON.Morse_bit_old[3] == 1 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'J';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 1 && BUTTON.Morse_bit_old[1] == 0 && BUTTON.Morse_bit_old[2] == 1 && BUTTON.Morse_bit_old[3] == 2 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'K';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 0 && BUTTON.Morse_bit_old[1] == 1 && BUTTON.Morse_bit_old[2] == 0 && BUTTON.Morse_bit_old[3] == 0 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'L';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 1 && BUTTON.Morse_bit_old[1] == 1 && BUTTON.Morse_bit_old[2] == 2 && BUTTON.Morse_bit_old[3] == 2 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'M';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 1 && BUTTON.Morse_bit_old[1] == 0 && BUTTON.Morse_bit_old[2] == 2 && BUTTON.Morse_bit_old[3] == 2 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'N';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 1 && BUTTON.Morse_bit_old[1] == 1 && BUTTON.Morse_bit_old[2] == 1 && BUTTON.Morse_bit_old[3] == 2 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'O';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 0 && BUTTON.Morse_bit_old[1] == 1 && BUTTON.Morse_bit_old[2] == 1 && BUTTON.Morse_bit_old[3] == 0 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'P';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 1 && BUTTON.Morse_bit_old[1] == 1 && BUTTON.Morse_bit_old[2] == 0 && BUTTON.Morse_bit_old[3] == 1 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'Q';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 0 && BUTTON.Morse_bit_old[1] == 1 && BUTTON.Morse_bit_old[2] == 0 && BUTTON.Morse_bit_old[3] == 2 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'R';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 0 && BUTTON.Morse_bit_old[1] == 0 && BUTTON.Morse_bit_old[2] == 0 && BUTTON.Morse_bit_old[3] == 2 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'S';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 1 && BUTTON.Morse_bit_old[1] == 2 && BUTTON.Morse_bit_old[2] == 2 && BUTTON.Morse_bit_old[3] == 2 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'T';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 0 && BUTTON.Morse_bit_old[1] == 0 && BUTTON.Morse_bit_old[2] == 1 && BUTTON.Morse_bit_old[3] == 2 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'U';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 0 && BUTTON.Morse_bit_old[1] == 0 && BUTTON.Morse_bit_old[2] == 0 && BUTTON.Morse_bit_old[3] == 1 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'V';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 0 && BUTTON.Morse_bit_old[1] == 1 && BUTTON.Morse_bit_old[2] == 1 && BUTTON.Morse_bit_old[3] == 2 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'W';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 1 && BUTTON.Morse_bit_old[1] == 0 && BUTTON.Morse_bit_old[2] == 0 && BUTTON.Morse_bit_old[3] == 1 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'X';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 1 && BUTTON.Morse_bit_old[1] == 0 && BUTTON.Morse_bit_old[2] == 1 && BUTTON.Morse_bit_old[3] == 1 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'Y';
	  }
	  else if (BUTTON.Morse_bit_old[0] == 1 && BUTTON.Morse_bit_old[1] == 1 && BUTTON.Morse_bit_old[2] == 0 && BUTTON.Morse_bit_old[3] == 0 &&  BUTTON.Morse_bit_old[4] == 2)
	  {
		  BUTTON.Letter[n] = 'Z';
	  }
	  n++; // index of BUTTON.Letter

  }
  if (BUTTON.Released_Duration_Now == 7000)
  {
	  memcpy(BUTTON.Word, BUTTON.Letter, 10);
	  for (int m = 0; m<10; m++)
	  {
		  BUTTON.Letter[m] = 2;
	  }
	  n = 0;  //index of BUTTON.Letter

  }



  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
