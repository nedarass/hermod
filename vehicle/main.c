/**
  ******************************************************************************
  * @file           : main_test.c
  * @brief          : Hyperloop Navigasyon Sistemi - TEST MODU
  * @author         : Hyperloop Takımı
  ******************************************************************************
  * NOT: Bu dosya sadece test için kullanılır.
  *      Normal çalışma için main_normal.c kullanın.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "optical_sensor.h"
#include "shared_data.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2; // Debug UART
TIM_HandleTypeDef htim2;   // Timer for simulation

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void Test_Menu(void);
void Test_ManualTrigger(void);
void Test_AutoSimulation(void);

/* Global variables ----------------------------------------------------------*/
volatile uint8_t sensor_triggered = 0;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();  // Debug UART
  MX_TIM2_Init();         // Timer for simulation
  
  // LED GPIO'yu başlat (PC13)
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /* Test butonu için GPIO (manuel tetikleme için) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* Başlangıç mesajı */
  printf("\r\n========================================\r\n");
  printf("HYPERLOOP NAVIGASYON SISTEMI - TEST MODU\r\n");
  printf("STM32F1 - Optical Sensor Test\r\n");
  printf("========================================\r\n\r\n");
  
  /* Optik sensörü başlat */
  OpticalSensor_Init();
  printf("Optical sensor initialized.\r\n");
  printf("First reflector at: %.1f m\r\n", FIRST_REFLECTOR_DIST);
  printf("Reflector spacing: %.1f m\r\n", REFLECTOR_SPACING);
  printf("\r\n");
  
  /* Test menüsünü göster */
  Test_Menu();
  
  /* Sonsuz döngü - Test modu */
  while (1)
  {
    // Ana döngüde sadece küçük bir bekleme
    HAL_Delay(10);
  }
}

/**
  * @brief Test menüsü
  */
void Test_Menu(void)
{
  uint8_t choice = 0;
  char buffer[10];
  
  printf("\r\n=== TEST MENUSU ===\r\n");
  printf("1. Tam Otomatik Test (OpticalSensor_RealTest)\r\n");
  printf("2. Manuel Tetikleme Testi\r\n");
  printf("3. Otomatik Simulasyon (Timer ile)\r\n");
  printf("4. Debug Ciktisi\r\n");
  printf("5. Sensor Durumunu Goster\r\n");
  printf("Seciminiz (1-5): ");
  
  // Basit bir bekleme ve varsayılan seçim
  HAL_Delay(1000);
  choice = 1; // Varsayılan olarak tam otomatik test
  
  printf("%d\r\n", choice);
  
  switch(choice)
  {
    case 1:
      printf("\r\n=== TAM OTOMATIK TEST BASLATILIYOR ===\r\n");
      OpticalSensor_RealTest();
      break;
      
    case 2:
      printf("\r\n=== MANUEL TETIKLEME TESTI ===\r\n");
      printf("PA0 pinini baglayarak veya butona basarak test edin.\r\n");
      printf("Her tetiklemede bir reflektor sayilacak.\r\n");
      printf("Cikmak icin reset atin.\r\n\r\n");
      Test_ManualTrigger();
      break;
      
    case 3:
      printf("\r\n=== OTOMATIK SIMULASYON ===\r\n");
      printf("Timer ile otomatik sensor sinyalleri uretilecek.\r\n");
      Test_AutoSimulation();
      break;
      
    case 4:
      printf("\r\n=== DEBUG CIKTISI ===\r\n");
      OpticalSensor_DebugOutput();
      Test_Menu(); // Menüye geri dön
      break;
      
    case 5:
      printf("\r\n=== SENSOR DURUMU ===\r\n");
      printf("Reflector Count: %lu\r\n", VehicleState.reflector_count);
      printf("Position: %.2f m\r\n", VehicleState.current_position);
      printf("Velocity: %.2f m/s\r\n", VehicleState.current_velocity);
      printf("System Status: %d\r\n", VehicleState.system_status);
      Test_Menu(); // Menüye geri dön
      break;
      
    default:
      printf("Gecersiz secim!\r\n");
      Test_Menu();
      break;
  }
}

/**
  * @brief Manuel tetikleme testi
  */
void Test_ManualTrigger(void)
{
  uint32_t last_trigger_time = 0;
  uint8_t last_button_state = 1;
  uint8_t trigger_count = 0;
  
  printf("Manuel test basladi. Her tetiklemede LED yanip donecek.\r\n");
  printf("Tetikleme sayisi: 0\r");
  
  while(1)
  {
    uint8_t button_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    
    // Butona basıldığında (active low)
    if (button_state == 0 && last_button_state == 1)
    {
      // Debounce
      if (HAL_GetTick() - last_trigger_time > 50)
      {
        // Sensor sinyalini simüle et
        OpticalSensor_EXTI_Callback(OPTICAL_SENSOR_PIN);
        
        trigger_count++;
        printf("Tetikleme sayisi: %d | Konum: %.2fm | Hiz: %.2fm/s\r", 
               trigger_count, 
               VehicleState.current_position,
               VehicleState.current_velocity);
        
        last_trigger_time = HAL_GetTick();
      }
    }
    
    last_button_state = button_state;
    
    // Her 10 tetiklemede bir özet göster
    if (trigger_count > 0 && trigger_count % 10 == 0)
    {
      printf("\r\n--- OZET ---\r\n");
      printf("Toplam tetikleme: %d\r\n", trigger_count);
      printf("Son konum: %.2f m\r\n", VehicleState.current_position);
      printf("Son hiz: %.2f m/s\r\n", VehicleState.current_velocity);
      printf("---\r\n");
    }
    
    // 100 tetikleme sonunda testi bitir
    if (trigger_count >= 100)
    {
      printf("\r\n\r\nTest tamamlandi! 100 tetikleme yapildi.\r\n");
      printf("Son durum:\r\n");
      OpticalSensor_DebugOutput();
      break;
    }
    
    HAL_Delay(10);
  }
  
  // Test bittikten sonra menüye dön
  printf("\r\nTest bitti. 3 saniye sonra menu goruntulenecek...\r\n");
  HAL_Delay(3000);
  Test_Menu();
}

/**
  * @brief Timer ile otomatik simulasyon
  */
void Test_AutoSimulation(void)
{
  printf("Otomatik simulasyon basliyor...\r\n");
  printf("Simulasyon hizi: 8 m/s\r\n");
  printf("Reflektorler arasi sure: %.0f ms\r\n", REFLECTOR_SPACING / 8.0f * 1000.0f);
  
  // Timer'ı başlat (her 500ms'de bir kesme)
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200 - 1;    // 72MHz / 7200 = 10kHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000 - 1;       // 10kHz / 5000 = 2Hz (500ms)
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);
  HAL_TIM_Base_Start_IT(&htim2);
  
  uint32_t simulation_time = 0;
  uint32_t last_print_time = 0;
  
  printf("\r\nSimulasyon basladi...\r\n");
  printf("Zaman | Reflektor | Konum | Hiz\r\n");
  printf("--------------------------------\r\n");
  
  while(1)
  {
    // Timer kesmesi geldi mi?
    if (sensor_triggered)
    {
      sensor_triggered = 0;
      
      // Sensor sinyalini simüle et
      OpticalSensor_EXTI_Callback(OPTICAL_SENSOR_PIN);
      
      simulation_time += 500; // 500ms aralıklarla
      
      // Her 5 saniyede bir çıktı ver
      if (simulation_time - last_print_time >= 5000)
      {
        printf("%5.1fs | %9lu | %6.1fm | %5.2fm/s\r\n", 
               simulation_time / 1000.0f,
               VehicleState.reflector_count,
               VehicleState.current_position,
               VehicleState.current_velocity);
        last_print_time = simulation_time;
      }
      
      // Tünel sonuna ulaşıldı mı?
      if (VehicleState.current_position >= 186.0f)
      {
        printf("\r\n=== TUNEL SONUNA ULASILDI ===\r\n");
        printf("Toplam simulasyon suresi: %.1f saniye\r\n", simulation_time / 1000.0f);
        printf("Toplam reflektor: %lu\r\n", VehicleState.reflector_count);
        printf("Ortalama hiz: %.2f m/s\r\n", 
               186.0f / (simulation_time / 1000.0f));
        break;
      }
    }
    
    // 30 saniye sonra otomatik dur
    if (simulation_time >= 30000)
    {
      printf("\r\nSimulasyon 30 saniye sonra durduruldu.\r\n");
      break;
    }
    
    HAL_Delay(10);
  }
  
  // Timer'ı durdur
  HAL_TIM_Base_Stop_IT(&htim2);
  
  // Test bittikten sonra menüye dön
  printf("\r\nSimulasyon bitti. 3 saniye sonra menu goruntulenecek...\r\n");
  HAL_Delay(3000);
  Test_Menu();
}

/**
  * @brief Timer kesme callback
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    sensor_triggered = 1;
  }
}

/**
  * @brief  System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

/**
  * @brief TIM2 Initialization Function
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    // Hata durumunda LED hızlı yanıp sönsün
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(50);
  }
}
