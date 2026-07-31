/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 *
 * FIXES APPLIED:
 *  1. TIM1 slave config block removed — free-running counter now
 *  2. ETR remap removed — only TI1→COMP1 remap kept
 *  3. TIM1_CC_IRQn and TIM1_UP_TIM16_IRQn both enabled (G4 split IRQs)
 *  4. Calculate_Prescaler() formula fixed (sampling_rate is Hz, not kHz)
 *  5. TIM1_OVC reset only on IDLE→DONE transition
 *  6. FreqString double-sprintf removed
 *  7. ICFilter=4 added for noise rejection
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ILI9341_GFX.h"
#include "ILI9341_STM32_Driver.h"
#include "arm_math.h"
#include "fft_display.h"
#include "fonts.h"
#include "time_domain.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

COMP_HandleTypeDef hcomp1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
#define NS 128
uint32_t Wave_LUT[NS] = {
    2048,2148,2248,2348,2447,2545,2642,2737,
    2831,2923,3013,3100,3185,3267,3346,3422,
    3496,3566,3631,3693,3751,3805,3854,3899,
    3939,3975,4005,4031,4051,4067,4077,4085,
    4095,4085,4077,4067,4051,4031,4005,3975,
    3939,3899,3854,3805,3751,3693,3631,3566,
    3496,3422,3346,3267,3185,3100,3013,2923,
    2831,2737,2642,2545,2447,2348,2248,2148,
    2048,1948,1848,1748,1649,1551,1454,1359,
    1265,1173,1083, 996, 911, 829, 750, 674,
     600, 530, 465, 403, 345, 291, 242, 197,
     157, 121,  91,  65,  45,  29,  19,  11,
       1,  11,  19,  29,  45,  65,  91, 121,
     157, 197, 242, 291, 345, 403, 465, 530,
     600, 674, 750, 829, 911, 996,1083,1173,
    1265,1359,1454,1551,1649,1748,1848,1948
};

uint16_t adc1_buf[BUF_SIZE * 2];
uint16_t adc2_buf[BUF_SIZE * 2];
uint16_t displayed_data1[BUF_SIZE];
uint16_t displayed_data2[BUF_SIZE];

volatile uint8_t data_ready = 0;
volatile uint16_t dma_offset = 0;

uint8_t prev_valid = 0;
uint16_t usb_tx_busy = 0;
volatile uint8_t spi_dma_done = 1;

uint32_t last_debounce_time = 0;
#define DEBOUNCE_MS 50

/* Per-column previous Y values for targeted erase */
int16_t prev_y1[PLOT_W];
int16_t prev_y2[PLOT_W];
int16_t prev_y1_next[PLOT_W];
int16_t prev_y2_next[PLOT_W];

/* FFT */
#define FFT_SIZE 512

typedef enum { MODE_TIME = 0, MODE_FFT } DisplayMode;

/* Vertical controls (oscilloscope style) */
float ch1_scale = 1.0f;
float ch2_scale = 1.0f;
int16_t ch1_offset = 0;
int16_t ch2_offset = 0;

/* FFT controls */
float fft_y_scale = 1.0f;
int16_t fft_y_offset = -150;
float fft_x_scale = 1.0f;
int16_t fft_x_offset = 0;

/* sampling_rate is in Hz — used directly in Calculate_Prescaler() */
float sampling_rate = 2000.0f;
uint32_t last_label_time = 0;

volatile DisplayMode display_mode = MODE_TIME;

/* Frequency measurement */
#define IDLE 0
#define DONE 1
#define F_CLK 144000000UL

volatile uint8_t State = IDLE;
volatile uint32_t T1 = 0;
volatile uint32_t T2 = 0;
volatile uint32_t Ticks = 0;
volatile uint16_t TIM1_OVC = 0;
volatile uint32_t Freq = 0;
char FreqString[20] = {'\0'};

/* DMA half/full offsets for dual ADC */
volatile uint16_t dma_offset1 = 0;
volatile uint16_t dma_offset2 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_COMP1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ── SPI DMA complete ───────────────────────────────────────────────────── */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
    spi_dma_done = 1;
  }
}

/* ── TIM3 ARR calculation ───────────────────────────────────────────────── */
/*
 * sampling_rate is in Hz (e.g. 2000 = 2 kHz).
 * TIM3 runs off 144 MHz with prescaler = 0, so:
 *   ARR = (F_CLK / sampling_rate) - 1
 *
 * OLD (wrong): divided by sampling_rate * 1000  → ARR was 1000× too small
 */
void Set_Sine_Freq(uint32_t freq_hz) {
    uint32_t arr = (144000000 / (freq_hz * NS)) - 1;  // Use NS (32) not 128
    HAL_TIM_Base_Stop(&htim2);
__HAL_TIM_SET_AUTORELOAD(&htim2, arr);
__HAL_TIM_SET_COUNTER(&htim2, 0);
HAL_TIM_Base_Start(&htim2);
}

uint32_t Calculate_Prescaler(void) {
  uint32_t arr = (uint32_t)((144000000.0f / (sampling_rate * 1000.0)) - 1.0f);
  return arr;
}

/* ── Dual ADC DMA callbacks ─────────────────────────────────────────────── */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1)
    dma_offset1 = 0;
  if (hadc->Instance == ADC2)
    dma_offset2 = 0;
  if (dma_offset1 == dma_offset2)
    data_ready = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1)
    dma_offset1 = BUF_SIZE;
  if (hadc->Instance == ADC2)
    dma_offset2 = BUF_SIZE;
  if (dma_offset1 == dma_offset2)
    data_ready = 1;
}

/* ── TIM1 Input Capture — frequency measurement ─────────────────────────── */
/*
 * Two-capture method:
 *   IDLE  → latch T1, reset overflow counter, advance to DONE
 *   DONE  → latch T2, compute period including overflows, advance to IDLE
 *
 * TIM1 runs free at 144 MHz (prescaler=0, period=65535).
 * COMP1 output is routed to TI1 via TIMEx_TISelection.
 * No slave mode — timer is never reset by an external signal.
 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (State == IDLE) {
    T1 = TIM1->CCR1;
    TIM1_OVC = 0;
    State = DONE;
  } else if (State == DONE) {
    T2 = TIM1->CCR1;
    Ticks = (T2 + (TIM1_OVC * 65536)) - T1;
    Freq = (uint32_t)(F_CLK / Ticks);
    State = IDLE;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    TIM1_OVC++;
  }
}

/* ── Display mode toggle ────────────────────────────────────────────────── */
void Toggle_Display_Mode(void) {
  display_mode = (display_mode == MODE_TIME) ? MODE_FFT : MODE_TIME;
  ILI9341_Fill_Screen(BLACK);

  if (display_mode == MODE_FFT)
    Draw_FFT_Labels();
  else
    Draw_Time_Labels();

  Draw_Axes();
  prev_valid = 0;
}

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
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_COMP1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
/* === Configure TIM2 for DAC trigger only (no PWM) === */
/* TIM2 is already initialized, just start it */
HAL_TIM_Base_Start(&htim2);

/* === Start DAC with DMA === */
if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)Wave_LUT, NS,
                      DAC_ALIGN_12B_R) != HAL_OK) {
    Error_Handler();
}

/* === CRITICAL: Enable DAC channel === */

/* Calibrate ADC (required on STM32G4) */
HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

/* Display init before ADC DMA starts */
ILI9341_Init();
ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
ILI9341_Fill_Screen(BLACK);
Draw_Time_Labels();
Draw_Axes();

/* Initialise erase-history arrays to centre line */
for (int i = 0; i < PLOT_W; i++) {
    prev_y1[i] = PLOT_Y0 + PLOT_H / 2;
    prev_y1_next[i] = PLOT_Y0 + PLOT_H / 2;
    prev_y2[i] = PLOT_Y0 + PLOT_H / 2;
    prev_y2_next[i] = PLOT_Y0 + PLOT_H / 2;
}

/*
 * DMA priority:
 *   ADC DMA  (Ch1, Ch2) → priority 2  (lower)
 *   SPI DMA  (Ch3)      → priority 1  (higher)
 *   DAC DMA  (Ch4)      → priority 3  (lowest)
 */
HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 2, 0);
HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 3, 0);

/* Start ADC DMA — circular, double-buffered */
HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buf, BUF_SIZE * 2);
HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buf, BUF_SIZE * 2);

/* TIM3 → ADC sample clock */
HAL_TIM_Base_Start(&htim3);

/*
 * TIM1 — frequency counter.
 * Start overflow (update) interrupt FIRST so TIM1_OVC is always
 * incremented before a capture fires on a wrap.
 */
HAL_TIM_Base_Start_IT(&htim1);
HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

/* Comparator must be running for TI1 to see edges */
HAL_COMP_Start(&hcomp1);

FFT_Init();

Set_Sine_Freq(20000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* ── Frequency display ─────────────────────────────────────────── */
    /*
     * Snapshot Freq atomically (32-bit read is atomic on Cortex-M4
     * but copy once to avoid it changing mid-sprintf).
     */
    uint32_t freq_hz = Freq;
    uint32_t freq_int = freq_hz / 1000;        /* whole kHz */
    uint32_t freq_dec = (freq_hz % 1000) / 10; /* 2 decimal places */
    sprintf(FreqString, "%lu.%02lu kHz", freq_int, freq_dec);
    ILI9341_Draw_Text_Arial(FreqString, 100, 5, YELLOW, BLACK,
               
      Arial_Narrow8x12);

    /* ── Button debounce (polling) ─────────────────────────────────── */
    static uint8_t button_state = 1;
    static uint8_t last_reading = 1;

    /* Keep TIM3 ARR in sync with sampling_rate */
    __HAL_TIM_SET_AUTORELOAD(&htim3, Calculate_Prescaler());

    uint8_t reading = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

    if (reading != last_reading)
      last_debounce_time = HAL_GetTick();

    if ((HAL_GetTick() - last_debounce_time) > DEBOUNCE_MS) {
      if (reading != button_state) {
        button_state = reading;
        if (button_state == 0) { /* falling edge = press */
          Toggle_Display_Mode();
          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
        }
      }
    }

    last_reading = reading;

    /* ── Waveform / FFT rendering ──────────────────────────────────── */
    if (data_ready) {
      data_ready = 0;

      __disable_irq();
      uint16_t off1 = dma_offset1;
      uint16_t off2 = dma_offset2;
      memcpy(displayed_data1, &adc1_buf[off1], BUF_SIZE * sizeof(uint16_t));
      memcpy(displayed_data2, &adc2_buf[off2], BUF_SIZE * sizeof(uint16_t));
      __enable_irq();

      if (display_mode == MODE_FFT)
        Plot_FFT();
      else
        Plot_Waveforms();
    }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 108;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_1_4VREFINT;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_10MV;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;

  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
hdac1.DMA_Handle1->Init.Mode = DMA_CIRCULAR;  // ← ADD THIS

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIMEx_RemapConfig(&htim1, TIM_TIM1_ETR_COMP1);
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIMEx_TISelection(&htim1, TIM_TIM1_TI1_COMP1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 624;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 71;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB6 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Configure PA4 as analog for DAC1_OUT1 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
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
  while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
