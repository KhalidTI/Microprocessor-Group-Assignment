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
// Define buzzer control pin (PA0)
#define BUZZER_GPIO_PORT GPIOA
#define BUZZER_GPIO_PIN_ODR (1 << 0) // For PA0
/* USER CODE END PD */
// uint8_t flag = 0; // flag and flag1 are no longer used for continuous blinking
// uint8_t flag1 = 0;

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

// Timer 1 configuration for microsecond delay
void tim1_config() {
    RCC->APB2ENR |= (1 << 11);     // Enable TIM1 clock
    TIM1->PSC = 72 - 1;            // Prescaler for 1 MHz timer (assuming 72 MHz system clock)
    TIM1->ARR = 0xFFFF;            // Set auto-reload register to max value
    TIM1->CR1 |= (1 << 0);         // Enable TIM1
}

// Microsecond delay using TIM1
void Delay_us(uint16_t us) {
    TIM1->CNT = 0;                 // Reset counter
    while (TIM1->CNT < us);       // Wait until desired time has passed
}

// Millisecond delay based on Delay_us
void Delay_ms(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) {
        Delay_us(1000);           // 1 millisecond delay
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  RCC->APB2ENR |= (1 << 4);          // Enable GPIOC clock
  GPIOC->CRH &= ~((15) << (20));     // Clear PC13 config bits
  GPIOC->CRH |= (1 << 20);           // Set PC13 as output (10 MHz)

  RCC->APB2ENR |= (1 << 3);          // Enable GPIOB clock

  // Configure PB1 as input with pull-up
  GPIOB->CRL &= ~((3) << (4));       // Clear MODE1
  GPIOB->CRL |= (1 << (4 + 2));      // Set CNF1 to 10 (input pull-up/down)
  GPIOB->ODR |= (1 << 1);            // Enable pull-up on PB1

  // Configure PB10 as output
  GPIOB->CRH &= ~((15) << (8));      // Clear config bits
  GPIOB->CRH |= (1 << (8));          // Set as output (10 MHz)

  // Configure PB11 as output
  GPIOB->CRH &= ~((15) << (12));     // Clear config bits
  GPIOB->CRH |= (1 << (12));         // Set as output (10 MHz)

  // Configure PA0 as output for buzzer
  RCC->APB2ENR |= (1 << 2);          // Enable GPIOA clock
  GPIOA->CRL &= ~((15) << (0));      // Clear PA0 config bits
  GPIOA->CRL |= (1 << (0));          // Set PA0 as output (10 MHz)

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  GPIOB->ODR |= (1 << 11);           // Turn on LED at PB11
  GPIOB->ODR |= (1 << 10);           // Turn on LED at PB10

  while (1)
  {
    Delay_ms(10);                    // Short polling delay

    // Check if button at PB1 is pressed (active low)
    if (((GPIOB->IDR >> 1) & 1) == 0) {
        GPIOB->ODR &= ~(1 << 11);    // Turn off PB11 LED
        GPIOB->ODR ^= (1 << 10);     // Toggle PB10 LED
        BUZZER_GPIO_PORT->ODR |= BUZZER_GPIO_PIN_ODR; // Turn buzzer ON
        Delay_ms(250);               // Buzzer/LED ON delay
    }
    else {
        // Button not pressed: turn both LEDs ON, turn buzzer OFF
        GPIOB->ODR |= (1 << 10);     // Turn PB10 LED ON
        GPIOB->ODR |= (1 << 11);     // Turn PB11 LED ON
        BUZZER_GPIO_PORT->ODR &= ~BUZZER_GPIO_PIN_ODR; // Turn buzzer OFF
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
