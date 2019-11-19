/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
extern uint32_t buffer[5];
extern uint8_t tx_count;
extern uint8_t rx_count;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();
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
  /* USER CODE BEGIN 2 */
/* Port D as USART */
//  uint32_t u_portD = 0X40020C00; /* Base address of port D */
  SET_BIT(GPIOD->PUPDR, GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR8_0); /* Port D pull up pull down -> Pull up selected */
  SET_BIT(GPIOD->MODER, GPIO_MODER_MODER9_1 | GPIO_MODER_MODER8_1); /* MODER: Select alternate functionality for PD8 and PD9 */
  /* Select AF7 as alternate functionality for PD8 and PD9 */
  SET_BIT(GPIOD->AFR[1], GPIO_AFRH_AFRH1_0 | GPIO_AFRH_AFRH1_1 | GPIO_AFRH_AFRH1_2 | GPIO_AFRH_AFRH0_0 | GPIO_AFRH_AFRH0_1 | GPIO_AFRH_AFRH0_2);  
//  *(uint32_t *) (u_portD + 0x00000024) = 0x00000077; /* Select AF7 as alternate functionality for PD8 and PD9 */
  
  /* USART 3 configurations*/
//  uint32_t u_USART3 = 0x40004800; /* Base address of USART3 */
  
  /* Value of CR1 register 
  Bit 27 = M1
  Bit 12 = M0 
  Bit 15 = Over sampling mode (by 16)
  Bit 9 = Parity selection
  Bit 0 = UE
  */
//  *(uint32_t*)(u_USART3) = (0x00000001); //
  SET_BIT(USART3->CR1, USART_CR1_UE);
  
  /* Value of CR2 register 
  Bit 13-12 = Stop bit 
  */
  //*(uint32_t*)(u_USART3 + 0x00000004) &= ~(0x00003000);
//  *(uint32_t*)(u_USART3 + 0x00000004) = (0x00);
  CLEAR_BIT(USART3->CR2, USART_CR2_STOP_0 | USART_CR2_STOP_1);
  
  /* Value of BRR (Baud rate) register 
  For oversampling 16
  BRR = USARTDIV (USARTDIV  = (fclk = 16Mhz)/ (baudrate = 9600))  
  */

//  *(uint32_t*)(u_USART3 + 0x0000000C) = (0x00000683);
  USART3->BRR = 0x00000683;
  

   /* Enable global USART interrupts*/
  uint32_t prioritygroup_user = NVIC_GetPriorityGrouping();
  NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(prioritygroup_user, 2, 0));
  NVIC_EnableIRQ(USART3_IRQn);
  
  SET_BIT(USART3->CR1, USART_CR1_RXNEIE); // Enable rx interrupt
  /* Enable USART Reception: Value of CR1 register
  Bit 2 = RE
  */
//   *(uint32_t*)(u_USART3) |= (1 << 2);
   SET_BIT(USART3->CR1, USART_CR1_RE);
   
     /* Enable USART transmission: Value of CR1 register
  Bit 3 = TE
  */
  
//   *(uint32_t*)(u_USART3) |= (1 << 3);
   SET_BIT(USART3->CR1, USART_CR1_TE); /* Can have fullduplex mode?*/ 
//   SET_BIT(USART3->CR1, USART_CR1_TXEIE); // Enable tx interrupt
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if((!READ_BIT(USART3->CR1, USART_CR1_RXNEIE)) && (READ_BIT(USART3->CR1, USART_CR1_TE)))
    {     
      
      SET_BIT(USART3->CR1, USART_CR1_TXEIE); // Enable tx interrupt
      
      USART3->TDR = buffer[tx_count];
      tx_count++;        
      if(tx_count == 5)
      {
        CLEAR_BIT(USART3->CR1, USART_CR1_TE);
      }
      HAL_Delay(50);
    } 
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
