/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define the buzzer pin for clarity
#define BUZZER_GPIO_PORT GPIOA
#define BUZZER_GPIO_PIN GPIO_ODR_ODR0 // For PA0

/* USER CODE END PD */
uint8_t flag = 0;
uint8_t flag1 = 0;


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void tim1_config() {
    // Enable TIM1 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;


    TIM1->PSC = 72 - 1;  // Assuming clock is 8
    TIM1->ARR = 0xFFFF;  // Max ARR

    // Enable the timer
    TIM1->CR1 |= TIM_CR1_CEN;
}

void Delay_us(uint16_t us) {
    TIM1->CNT = 0;  // reset counter
   while (TIM1->CNT < us);
}

void Delay_ms(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) {
        Delay_us(1000);  // 1 ms
    }
}





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
  //void tim1_config();
  //void hell_nah(uint16_t us);
 // void delay_ms(uint16_t ms);
      RCC->APB2ENR |= 0b10000;  // Enable clock for C
      GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);  // Clear
      GPIOC->CRH |= GPIO_CRH_MODE13_1;  // Output

      RCC->APB2ENR |= 0b1000;//enable B port

      //PB1 input
      GPIOB -> CRL &= ~(GPIO_CRL_MODE1); //clear
      GPIOB -> CRL &= ~(GPIO_CRL_CNF1);//clear
      GPIOB -> CRL |= GPIO_CRL_CNF1_1; //input mode
      GPIOB -> ODR |= GPIO_ODR_ODR1; // pull up resister


      GPIOB->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);  // Clear
      GPIOB->CRH |= GPIO_CRH_MODE10_1;
      GPIOB->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);  // Clear
      GPIOB->CRH |= GPIO_CRH_MODE11_1;



      // Configure Timer 1
      tim1_config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 /* RCC ->APB2ENR |= RCC_APB2ENR_IOPCEN  ; // line 1289 enable clock for c port
  GPIOC ->CRH |=  GPIO_CRH_MODE13_1;
  GPIOC ->CRH &= ~GPIO_CRH_CNF13;
  GPIOC ->ODR &= ~GPIO_ODR_ODR13;
	  delay_ms(500);

      GPIOC ->ODR |=  GPIO_ODR_ODR13;
*/

      GPIOB->ODR |= GPIO_ODR_ODR11;
      GPIOB->ODR |= GPIO_ODR_ODR10;
  while (1)
  {
	  Delay_ms(10);

	  if ((GPIOB->IDR & GPIO_IDR_IDR1) == 0) {
	 //(working) // Toggle LED
			   GPIOB->ODR &= ~GPIO_ODR_ODR11;
	       if (!flag){
	          for (int i = 0;i <4;i++){
	        	  if((GPIOB->IDR & GPIO_IDR_IDR1) != 0){
	        		  GPIOB->ODR |= (1 << 11);
	              	  	    Delay_ms(700);
	              	  	    GPIOB->ODR ^= GPIO_ODR_ODR10;

	        	  }
	              else{
	            	  Delay_ms(700);
	            	  GPIOB->ODR ^= GPIO_ODR_ODR10;
	              }
	          if(i == 3){
	        	   flag =1;
	           }
	          }//blinking twice


	       }
			  BUZZER_GPIO_PORT->ODR |= BUZZER_GPIO_PIN;


	       flag = 0;
	  }

	  else {
		  GPIOB->ODR |= (1 << 10);
		  GPIOB->ODR |= (1 << 11);
    	  	BUZZER_GPIO_PORT->ODR &= ~BUZZER_GPIO_PIN;


		  flag = 0;

	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  //}
  /* USER CODE END 3 */
	  }
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
