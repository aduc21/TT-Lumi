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
#include <stdbool.h>   // thêm cái này để dùng bool, true, false
#include <stdint.h>    // cho uint32_t, uint8_t

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Board green LED (LD2 on Nucleo) */
#define BOARD_GREEN_PORT   GPIOA
#define BOARD_GREEN_PIN    GPIO_PIN_5

/* LED1 (RGB) pins */
#define LED1_R_PORT  GPIOA
#define LED1_R_PIN   GPIO_PIN_1
#define LED1_G_PORT  GPIOA
#define LED1_G_PIN   GPIO_PIN_0
#define LED1_B_PORT  GPIOA
#define LED1_B_PIN   GPIO_PIN_3

/* LED2 (RGB) pins */
#define LED2_R_PORT  GPIOB
#define LED2_R_PIN   GPIO_PIN_13
#define LED2_G_PORT  GPIOA
#define LED2_G_PIN   GPIO_PIN_11
#define LED2_B_PORT  GPIOA
#define LED2_B_PIN   GPIO_PIN_10

/* Buzzer */
#define BUZZ_PORT    GPIOC
#define BUZZ_PIN     GPIO_PIN_9

/* Buttons */
#define B2_PORT      GPIOB
#define B2_PIN       GPIO_PIN_3   // nút B2
#define B3_PORT      GPIOA
#define B3_PIN       GPIO_PIN_4   // nút B3 (nhấn 5 lần)
#define B4_PORT      GPIOB
#define B4_PIN       GPIO_PIN_0   // nút B4

/* Timing thresholds (ms) */
#define DEBOUNCE_MS      20
#define B3_INTERVAL_MS   1000
#define LONGPRESS_MS     500
#define DOUBLECLICK_MS   400

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static inline void pin_set(GPIO_TypeDef* P, uint16_t pin) { HAL_GPIO_WritePin(P, pin, GPIO_PIN_SET); }
static inline void pin_reset(GPIO_TypeDef* P, uint16_t pin) { HAL_GPIO_WritePin(P, pin, GPIO_PIN_RESET); }

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    bool prev_raw;
    uint32_t press_ts;
    bool long_active;
    uint32_t last_release;
    uint8_t click_cnt;
    bool toggle_state;
} btn_t;

btn_t btn2 = {B2_PORT, B2_PIN, false, 0, false, 0, 0, false};
btn_t btn4 = {B4_PORT, B4_PIN, false, 0, false, 0, 0, false};

/* B3 counter */
uint8_t b3_count = 0;
uint32_t b3_last_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void led1_set_color(uint8_t r, uint8_t g, uint8_t b);
static void led2_set_color(uint8_t r, uint8_t g, uint8_t b);
static void all_rgb_set_green(void);
static void all_rgb_off(void);
static void buzzer_beep(uint8_t times, uint32_t on_ms, uint32_t off_ms);

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
  /* USER CODE BEGIN 2 */
  /* Test LED sau khi init GPIO */
  // BLUE_1 (PA3)
  HAL_GPIO_WritePin(BLUE_1_GPIO_Port, BLUE_1_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(BLUE_1_GPIO_Port, BLUE_1_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);

  // RED_2 (PB13)
  HAL_GPIO_WritePin(RED_2_GPIO_Port, RED_2_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(RED_2_GPIO_Port, RED_2_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);

  /* Startup: nháy LED board 5 lần */
  for (int i = 0; i < 4; ++i) {
    HAL_GPIO_WritePin(GREEN_1_GPIO_Port, GREEN_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GREEN_2_GPIO_Port, GREEN_2_Pin, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GREEN_1_GPIO_Port, GREEN_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GREEN_2_GPIO_Port, GREEN_2_Pin, GPIO_PIN_RESET);
    HAL_Delay(150);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* ---------- B3: nhấn 5 lần ---------- */
    static bool b3_prev = false;
    bool b3_raw = (HAL_GPIO_ReadPin(B3_PORT, B3_PIN) == GPIO_PIN_RESET);
    if (b3_raw && !b3_prev) {
      HAL_Delay(DEBOUNCE_MS);
      if (HAL_GPIO_ReadPin(B3_PORT, B3_PIN) == GPIO_PIN_RESET) {
        if (now - b3_last_ms <= B3_INTERVAL_MS) b3_count++; else b3_count = 1;
        b3_last_ms = now;
        if (b3_count >= 5) {
          b3_count = 0;
          /* Nháy LED xanh + buzzer */
          for (int i = 0; i < 5; ++i) {
            pin_set(LED1_G_PORT, LED1_G_PIN);
            pin_set(LED2_G_PORT, LED2_G_PIN);
            HAL_Delay(200);
            pin_reset(LED1_G_PORT, LED1_G_PIN);
            pin_reset(LED2_G_PORT, LED2_G_PIN);
            HAL_Delay(150);
          }
          buzzer_beep(2, 200, 150);
        }
        while (HAL_GPIO_ReadPin(B3_PORT, B3_PIN) == GPIO_PIN_RESET);
      }
    }
    b3_prev = b3_raw;


    /* ---------- B2 & B4 unified handling ---------- */
    btn_t *bs[2] = { &btn2, &btn4 };
    for (int i = 0; i < 2; i++) {
      btn_t *b = bs[i];
      bool raw = (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_RESET);

      /* cạnh xuống (bắt đầu nhấn) */
      if (raw && !b->prev_raw) {
        HAL_Delay(DEBOUNCE_MS);
        if (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_RESET) {
          b->press_ts = now;
          b->long_active = false;
        }
      }

      /* đang giữ: nếu vượt quá LONGPRESS_MS -> long_active = true (momentary ON) */
      if (raw && !b->long_active && b->press_ts != 0) {
        if (now - b->press_ts >= LONGPRESS_MS) {
          b->long_active = true;
          /* DO NOT change toggle_state here: long_active chỉ để bật tạm thời khi giữ */
        }
      }

      /* cạnh lên (nhả) */
      if (!raw && b->prev_raw) {
        HAL_Delay(DEBOUNCE_MS);
        if (HAL_GPIO_ReadPin(b->port, b->pin) != GPIO_PIN_RESET) {
          if (b->long_active) {
            /* nếu đã là long press — đây là hành vi momentary: tắt khi nhả */
            b->long_active = false;
          } else {
            /* short press -> double-click detection */
            if (b->last_release != 0 && (now - b->last_release) <= DOUBLECLICK_MS) {
              /* double click -> toggle persistent state */
              b->click_cnt = 0;
              b->toggle_state = !b->toggle_state;
              b->last_release = 0;
            } else {
              /* ghi nhận 1 lần nhấn, chờ xem có double không */
              b->click_cnt = 1;
              b->last_release = now;
            }
          }
          b->press_ts = 0;
        }
      }

      b->prev_raw = raw;
    }

    /* single-click timeout => coi là single click -> turn OFF (như spec cũ) */
    if (btn2.click_cnt == 1 && (now - btn2.last_release) > DOUBLECLICK_MS) {
      btn2.click_cnt = 0;
      btn2.toggle_state = false;
      btn2.last_release = 0;
    }
    if (btn4.click_cnt == 1 && (now - btn4.last_release) > DOUBLECLICK_MS) {
      btn4.click_cnt = 0;
      btn4.toggle_state = false;
      btn4.last_release = 0;
    }

    /* ---------- Áp dụng trạng thái LED ---------- */
    /* BLUE_1: sáng khi giữ (long_active) OR khi đã toggle_state = ON */
    if (btn2.long_active || btn2.toggle_state)
      HAL_GPIO_WritePin(BLUE_1_GPIO_Port, BLUE_1_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(BLUE_1_GPIO_Port, BLUE_1_Pin, GPIO_PIN_RESET);

    /* RED_2: tương tự */
    if (btn4.long_active || btn4.toggle_state)
      HAL_GPIO_WritePin(RED_2_GPIO_Port, RED_2_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(RED_2_GPIO_Port, RED_2_Pin, GPIO_PIN_RESET);

    HAL_Delay(10);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GREEN_1_Pin|RED1_Pin|BLUE_1_Pin|BLUE_2_Pin
                          |GREEN_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED_2_GPIO_Port, RED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GREEN_1_Pin RED1_Pin BLUE_1_Pin BLUE_2_Pin
                           GREEN_2_Pin */
  GPIO_InitStruct.Pin = GREEN_1_Pin|RED1_Pin|BLUE_1_Pin|BLUE_2_Pin
                          |GREEN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : B3_Pin */
  GPIO_InitStruct.Pin = B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B4_Pin B2_Pin B5_Pin B1_Pin */
  GPIO_InitStruct.Pin = B4_Pin|B2_Pin|B5_Pin|B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RED_2_Pin */
  GPIO_InitStruct.Pin = RED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void led1_set_color(uint8_t r, uint8_t g, uint8_t b) {
  HAL_GPIO_WritePin(LED1_R_PORT, LED1_R_PIN, r ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED1_G_PORT, LED1_G_PIN, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED1_B_PORT, LED1_B_PIN, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void led2_set_color(uint8_t r, uint8_t g, uint8_t b) {
  HAL_GPIO_WritePin(LED2_R_PORT, LED2_R_PIN, r ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_G_PORT, LED2_G_PIN, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_B_PORT, LED2_B_PIN, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void all_rgb_off(void) {
  led1_set_color(0,0,0); led2_set_color(0,0,0);
}
static void buzzer_beep(uint8_t times, uint32_t on_ms, uint32_t off_ms) {
  for (uint8_t i=0;i<times;i++){
    HAL_GPIO_WritePin(BUZZ_PORT, BUZZ_PIN, GPIO_PIN_SET);
    HAL_Delay(on_ms);
    HAL_GPIO_WritePin(BUZZ_PORT, BUZZ_PIN, GPIO_PIN_RESET);
    if (i < times-1) HAL_Delay(off_ms);
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
