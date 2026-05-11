/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Cortex-M exception handlers and dispatched device interrupts.
  *
  * IRQs not declared in this file (DMA1_Channel4, TIM1_UP_TIM16) are
  * defined in Core/Src/dshot.c so the DShot driver owns its transitions
  * end-to-end. The .ioc has those vectors set to "Generate IRQ handler = false".
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /* USER CODE END NonMaskableInt_IRQn 0 */
  while (1) { }
}

void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /* USER CODE END HardFault_IRQn 0 */
  while (1) { }
}

void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1) { }
}

void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  /* USER CODE END BusFault_IRQn 0 */
  while (1) { }
}

void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  /* USER CODE END UsageFault_IRQn 0 */
  while (1) { }
}

void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */
  /* USER CODE END SVCall_IRQn 0 */
}

void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */
  /* USER CODE END DebugMonitor_IRQn 0 */
}

void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
  /* USER CODE END PendSV_IRQn 0 */
}

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  App_Tick();
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* DMA1_Channel4_IRQHandler and TIM1_UP_TIM16_IRQHandler are defined in       */
/* Core/Src/dshot.c.                                                          */
/******************************************************************************/

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
