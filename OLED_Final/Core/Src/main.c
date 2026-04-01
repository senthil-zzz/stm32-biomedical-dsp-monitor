/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Twin's Medical Monitor (Fixed Build)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include <math.h>
#include "arm_math.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2; // Buzzer Timer
TIM_HandleTypeDef htim3; // ADC Trigger
UART_HandleTypeDef huart2;

// --- DSP SETTINGS ---
#define FS_HZ 200U
#define N_SAMPLES (FS_HZ * 3)
#define FFT_LEN 1024
#define PI_F 3.14159265358979f

// --- BUFFERS ---
uint16_t adc_raw[N_SAMPLES];
float32_t fft_in[FFT_LEN];
float32_t rfft_out[FFT_LEN];
float32_t mag[FFT_LEN/2];
float32_t hamming[N_SAMPLES];
arm_rfft_fast_instance_f32 rfft_inst;
volatile uint8_t dma_done = 0;

// --- STATE VARIABLES ---
float32_t current_bpm = 98.0f;
int current_spo2 = 98; // Low value to test alarm
uint8_t system_on = 1;

// --- UI BITMAPS ---
const unsigned char Heart_Big[] = {
  0x00, 0x00, 0x00, 0x03, 0xC0, 0xF0, 0x07, 0xE1, 0xF8, 0x0F, 0xF3, 0xFC,
  0x1F, 0xFF, 0xFE, 0x1F, 0xFF, 0xFE, 0x3F, 0xFF, 0xFE, 0x3F, 0xFF, 0xFE,
  0x3F, 0xFF, 0xFE, 0x3F, 0xFF, 0xFC, 0x1F, 0xFF, 0xFC, 0x1F, 0xFF, 0xF8,
  0x0F, 0xFF, 0xF8, 0x07, 0xFF, 0xF0, 0x03, 0xFF, 0xE0, 0x01, 0xFF, 0xC0,
  0x00, 0xFF, 0x80, 0x00, 0x7F, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x1C, 0x00,
  0x00, 0x00, 0x00
};
const unsigned char Heart_Small[] = {
  0x00, 0x00, 0x0E, 0x70, 0x1F, 0xF8, 0x3F, 0xFC, 0x7F, 0xFE, 0x7F, 0xFE,
  0x7F, 0xFE, 0x3F, 0xFC, 0x1F, 0xF8, 0x0F, 0xF0, 0x07, 0xE0, 0x03, 0xC0,
  0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const unsigned char Icon_Danger[] = {
  0x00, 0x00, 0x01, 0x80, 0x02, 0x40, 0x04, 0x20, 0x08, 0x10, 0x11, 0x88,
  0x21, 0x84, 0x41, 0x82, 0x81, 0x81, 0x80, 0x01, 0x81, 0x81, 0x80, 0x01,
  0x40, 0x02, 0x3F, 0xFC, 0x00, 0x00, 0x00, 0x00
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void); // BUZZER TIMER

// --- FUNCTIONS ---
void RunMedicalMonitor(void);
void HandleSafetyAlarms(void);
void CheckPowerButton(void);
void ShowLoadingScreen(void);
void ProcessDSP(void);

// UI Helpers
void SSD1306_DrawBitmap_Custom(int x, int y, const unsigned char* bitmap, int w, int h, int color);
void DrawVerticalBar(int level);
void DrawDashedLine(int y);
void DrawBoldString(int x, int y, char* str);

// --- PRINTF ---
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE { HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY); return ch; }

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init(); // Initialises Buzzer Timer

  // --- INIT ---
  ssd1306_Init();
  ShowLoadingScreen();

  // DSP Setup
  for (int n = 0; n < N_SAMPLES; ++n) {
      hamming[n] = 0.54f - 0.46f * cosf(2.0f * PI_F * (float)n / (float)(N_SAMPLES - 1));
  }
  arm_rfft_fast_init_f32(&rfft_inst, FFT_LEN);

  // Start Hardware
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, N_SAMPLES);

  while (1)
  {
      CheckPowerButton();

      if (system_on) {
          // A. Data Processing
          if (dma_done) {
              dma_done = 0;
              ProcessDSP();
              HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, N_SAMPLES);
          }

          // B. Safety Systems
          HandleSafetyAlarms();
          RunMedicalMonitor();
      }
      else {
          HAL_Delay(100); // Sleep
      }
  }
}

// =========================================================
// === CORE LOGIC ==========================================
// =========================================================

void ProcessDSP(void) {
    float32_t mean = 0.0f;
    for(int i=0; i<N_SAMPLES; i++) mean += (float32_t)adc_raw[i];
    mean /= (float32_t)N_SAMPLES;

    for(int i=0; i<N_SAMPLES; i++) fft_in[i] = ((float32_t)adc_raw[i] - mean) * hamming[i];
    for(int i=N_SAMPLES; i<FFT_LEN; i++) fft_in[i] = 0.0f;

    arm_rfft_fast_f32(&rfft_inst, fft_in, rfft_out, 0);

    for(int k=0; k<FFT_LEN/2; k++) {
        float32_t re = rfft_out[2*k];
        float32_t im = rfft_out[2*k+1];
        mag[k] = sqrtf(re*re + im*im);
    }

    int bin_min = (int)ceilf(0.5f * (float)FFT_LEN / (float)FS_HZ);
    int bin_max = (int)floorf(3.5f * (float)FFT_LEN / (float)FS_HZ);
    float32_t peak_val = 0.0f;
    int peak_bin = bin_min;

    for(int b=bin_min; b<=bin_max; b++) {
        if (mag[b] > peak_val) { peak_val = mag[b]; peak_bin = b; }
    }

    float32_t peak_freq = (float32_t)peak_bin * ((float32_t)FS_HZ / (float32_t)FFT_LEN);
    float32_t new_bpm = peak_freq * 60.0f;

    if(new_bpm > 40 && new_bpm < 200) {
        current_bpm = (current_bpm * 0.7f) + (new_bpm * 0.3f);
    }
}

void RunMedicalMonitor(void) {
    ssd1306_Fill(Black);

    if (HAL_GetTick() % 600 < 300) SSD1306_DrawBitmap_Custom(0, 4, Heart_Big, 24, 21, White);
    else SSD1306_DrawBitmap_Custom(4, 6, Heart_Small, 16, 16, White);

    DrawBoldString(26, 4, "Heart");
    DrawBoldString(26, 14, "Rate");

    char buf[10];
    sprintf(buf, "%.0f", current_bpm);
    ssd1306_SetCursor(80, 5);
    ssd1306_WriteString(buf, Font_11x18, White);

    DrawDashedLine(30);

    int bar_lvl = 0;
    if (current_spo2 >= 95) bar_lvl = 4;
    else if (current_spo2 >= 90) bar_lvl = 3;
    else if (current_spo2 >= 80) bar_lvl = 2;
    else bar_lvl = 1;

    DrawVerticalBar(bar_lvl);

    DrawBoldString(35, 45, "SPO2:");
    sprintf(buf, "%d%%", current_spo2);
    ssd1306_SetCursor(72, 42);
    ssd1306_WriteString(buf, Font_11x18, White);

    if (current_spo2 < 88) SSD1306_DrawBitmap_Custom(110, 40, Icon_Danger, 16, 16, White);

    ssd1306_UpdateScreen();
}

void HandleSafetyAlarms(void) {
    uint32_t tick = HAL_GetTick();
    uint32_t cycle = tick % 3000;

    if (current_spo2 < 88) {
        // === DANGER MODE ===
        // Blink PA8 (Red) and PB3 (Buzzer)
        if (cycle < 2000) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // LED ON
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);           // BUZZER ON
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);            // BUZZER OFF
        }
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); // Green OFF
    }
    else {
        // === NORMAL MODE ===
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);   // Green ON
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // Red OFF
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // IR ON
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // RED OFF
}

void CheckPowerButton(void) {
    static int lastPwr = 1;
    int currPwr = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);

    if (currPwr == 1 && lastPwr == 0) {
        system_on = !system_on;

        if (system_on) {
            ssd1306_Init();
            ShowLoadingScreen();
            HAL_TIM_Base_Start(&htim3);
            HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, N_SAMPLES);
        } else {
            ssd1306_Fill(Black);
            ssd1306_UpdateScreen();
            ssd1306_SetDisplayOn(0);

            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); // Kill Buzzer
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

            HAL_ADC_Stop_DMA(&hadc1);
            HAL_TIM_Base_Stop(&htim3);
        }
        HAL_Delay(500);
    }
    lastPwr = currPwr;
}

void ShowLoadingScreen(void) {
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);
    char *text = "Hello Twin";
    int x_start = 9; int y_pos = 22;
    for (int i = 0; i < 10; i++) {
        char letter[2] = {text[i], '\0'};
        ssd1306_SetCursor(x_start + (i * 11), y_pos);
        ssd1306_WriteString(letter, Font_11x18, White);
        ssd1306_UpdateScreen();
        HAL_Delay(100);
    }
    int tm_x = x_start + (10 * 11) + 2;
    ssd1306_SetCursor(tm_x, y_pos - 0);
    ssd1306_WriteString("TM", Font_7x10, White);
    ssd1306_UpdateScreen();
    HAL_Delay(1000);
}

// --- HELPERS ---
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) { if(hadc->Instance==ADC1) dma_done = 1; }
void DrawBoldString(int x, int y, char* str) { ssd1306_SetCursor(x, y); ssd1306_WriteString(str, Font_7x10, White); ssd1306_SetCursor(x+1, y); ssd1306_WriteString(str, Font_7x10, White); }
void DrawDashedLine(int y) { for(int x=0; x<128; x+=4) { ssd1306_DrawPixel(x, y, White); ssd1306_DrawPixel(x+1, y, White); } }
void DrawVerticalBar(int level) { for(int i=0; i<level; i++) { int y=58-(i*7); for(int w=0; w<6; w++) for(int h=0; h<5; h++) ssd1306_DrawPixel(10+w, y+h, White); } }
void SSD1306_DrawBitmap_Custom(int x, int y, const unsigned char* bitmap, int w, int h, int color) { int bw=(w+7)/8; for(int j=0; j<h; j++) for(int i=0; i<w; i++) if(bitmap[j*bw + i/8] & (128>>(i&7))) ssd1306_DrawPixel(x+i, y+j, color); }

// --- INIT FUNCTIONS ---
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}
static void MX_ADC1_Init(void) {
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);
  multimode.Mode = ADC_MODE_INDEPENDENT;
  HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}
static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100D14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}
static void MX_TIM2_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim2);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
}
static void MX_TIM3_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim3);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
}
static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  HAL_UART_Init(&huart2);
}
static void MX_DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  // PA5 Power
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA4, PA6, PA8 Outputs
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PB5 Output (IR)
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
static void MX_TIM1_Init(void) { }

void Error_Handler(void) { __disable_irq(); while (1) { } }
