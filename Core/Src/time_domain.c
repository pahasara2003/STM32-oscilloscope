#include "time_domain.h"
#include "ILI9341_GFX.h"
#include "ILI9341_STM32_Driver.h"
#include "fonts.h" // <-- add this
#include <stdio.h>

extern TIM_HandleTypeDef htim3;

#ifndef ADC_MAX
#define ADC_MAX 4095
#endif

extern float ch1_scale;
extern float ch2_scale;
extern int16_t ch1_offset;
extern int16_t ch2_offset;
extern float sampling_rate;

static int16_t y1_arr[PLOT_W];
static int16_t y2_arr[PLOT_W];
static int16_t y1_next_arr[PLOT_W];
static int16_t y2_next_arr[PLOT_W];
static uint8_t spibuf[PLOT_W * 2];
extern uint32_t last_label_time;

void Draw_Time_Labels(void) {
  if (sampling_rate <= 0.0f)
    return;

  // sampling_rate is in kHz → gives ms directly
  float ms_per_px = 1.0f / sampling_rate;

  uint16_t x_positions[3] = {PLOT_X0 + PLOT_W / 4, PLOT_X0 + PLOT_W / 2,
                             PLOT_X0 + 3 * PLOT_W / 4};

  for (int i = 0; i < 3; i++) {
    uint16_t px = x_positions[i] - PLOT_X0;
    float t_ms = px * ms_per_px;

    char buf[16];
    if (t_ms >= 1.0f) {
      int whole = (int)t_ms;
      int frac = (int)((t_ms - whole) * 100);
      sprintf(buf, "%d.%02d ms", whole, frac);
    } else if (t_ms >= 0.001f) {
      sprintf(buf, "%d us", (int)(t_ms * 1000.0f));
    } else {
      sprintf(buf, "%d ns", (int)(t_ms * 1000000.0f));
    }

    uint16_t x = x_positions[i]  - 15;
    uint16_t y = PLOT_Y1 + 5; // above plot

    ILI9341_Draw_Text_Arial(buf, x, y, WHITE, BLACK, Arial_Narrow8x12);

    uint16_t y_positions[3] = {
      PLOT_Y0 + PLOT_H / 4,
      PLOT_Y0 + PLOT_H / 2,
      PLOT_Y0 + 3 * PLOT_H / 4
  };

  float voltages[3] = {2.47f, 1.65f, 0.82f};

  for (int i = 0; i < 3; i++) {
    char buf[16];
    int whole = (int)voltages[i];
    int frac  = (int)((voltages[i] - whole) * 100);
    sprintf(buf, "%d.%02d V", whole, frac);

    ILI9341_Draw_Text_Arial_Vertical(buf, PLOT_X0 - 15, y_positions[i] - 15,
                            WHITE, BLACK, Arial_Narrow8x12);
  }
  }
}
static inline int16_t adc_to_y(uint16_t v, float scale, int16_t offset) {
  float norm = (float)v / ADC_MAX;
  float y = (norm - 0.5f) * PLOT_H;
  y += offset;
  y *= scale;
  return (int16_t)(PLOT_Y0 + (PLOT_H / 2) - y);
}

void Draw_Axes(void) {
  ILI9341_Draw_Horizontal_Line(PLOT_X0, PLOT_Y0 + PLOT_H / 2, PLOT_W, DARKGREY);
  ILI9341_Draw_Vertical_Line(PLOT_X0, PLOT_Y0, PLOT_H, DARKGREY);
  for (int t = 0; t <= 4; t++) {
    uint16_t ty = PLOT_Y0 + (PLOT_H * t) / 4;
    ILI9341_Draw_Horizontal_Line(PLOT_X0, ty, 5, DARKGREY);
  }
}

uint16_t Find_Trigger(uint16_t *buf, uint16_t len, uint16_t threshold,
                      uint16_t hysteresis, uint16_t holdoff) {
  uint16_t thresh_high = threshold + hysteresis;
  uint16_t thresh_low = threshold - hysteresis;
  uint16_t search_end = len - PLOT_W - 1;

  uint16_t i = holdoff;
  for (; i < search_end; i++) {
    if (buf[i] < thresh_low)
      break;
  }
  for (; i < search_end; i++) {
    if (buf[i - 1] < thresh_high && buf[i] >= thresh_high)
      return i;
  }
  return 0;
}

static void Auto_Threshold(uint16_t *buf, uint16_t len, uint16_t *threshold,
                           uint16_t *hysteresis) {
  uint16_t buf_min = buf[0];
  uint16_t buf_max = buf[0];
  for (int i = 1; i < len; i++) {
    if (buf[i] < buf_min)
      buf_min = buf[i];
    if (buf[i] > buf_max)
      buf_max = buf[i];
  }
  uint16_t swing = buf_max - buf_min;
  *threshold = buf_min + swing / 2;
  *hysteresis = swing / 6;
}

#define SKIP_SAMPLES 0
#define HOLDOFF 50

void Plot_Waveforms(void) {
  static uint16_t last_trig1 = 0;
  static uint16_t last_trig2 = 0;

  uint16_t th1, hy1;
  Auto_Threshold(displayed_data1, BUF_SIZE, &th1, &hy1);
  uint16_t trig1 = Find_Trigger(displayed_data1, BUF_SIZE, th1, hy1, HOLDOFF);

  if (trig1 == 0 || trig1 + SKIP_SAMPLES + PLOT_W >= BUF_SIZE)
    trig1 = last_trig1;
  else
    last_trig1 = trig1;

  if (trig1 + SKIP_SAMPLES + PLOT_W >= BUF_SIZE)
    trig1 = BUF_SIZE - PLOT_W - SKIP_SAMPLES - 1;
  if (trig1 < HOLDOFF + SKIP_SAMPLES + 1)
    trig1 = HOLDOFF + SKIP_SAMPLES + 1;

  uint16_t th2, hy2;
  Auto_Threshold(displayed_data2, BUF_SIZE, &th2, &hy2);
  uint16_t trig2 = trig1;
  if (trig2 == 0 || trig2 + SKIP_SAMPLES + PLOT_W >= BUF_SIZE)
    trig2 = last_trig2;
  else
    last_trig2 = trig2;

  if (trig2 + SKIP_SAMPLES + PLOT_W >= BUF_SIZE)
    trig2 = BUF_SIZE - PLOT_W - SKIP_SAMPLES - 1;
  if (trig2 < HOLDOFF + SKIP_SAMPLES + 1)
    trig2 = HOLDOFF + SKIP_SAMPLES + 1;

  for (uint16_t px = 0; px < PLOT_W; px++) {
    uint16_t s1 = trig1 + SKIP_SAMPLES + px;
    uint16_t s1_next = trig1 + SKIP_SAMPLES + px + 1;
    uint16_t s2 = trig2 + SKIP_SAMPLES + px;
    uint16_t s2_next = trig2 + SKIP_SAMPLES + px + 1;

    if (s1 >= BUF_SIZE)
      s1 = BUF_SIZE - 1;
    if (s1_next >= BUF_SIZE)
      s1_next = BUF_SIZE - 1;
    if (s2 >= BUF_SIZE)
      s2 = BUF_SIZE - 1;
    if (s2_next >= BUF_SIZE)
      s2_next = BUF_SIZE - 1;

    y1_arr[px] = adc_to_y(displayed_data1[s1], ch1_scale, ch1_offset);
    y1_next_arr[px] = adc_to_y(displayed_data1[s1_next], ch1_scale, ch1_offset);
    y2_arr[px] = adc_to_y(displayed_data2[s2], ch2_scale, ch2_offset);
    y2_next_arr[px] = adc_to_y(displayed_data2[s2_next], ch2_scale, ch2_offset);
  }

  /* ── DMA render loop ── */
  ILI9341_Set_Address(PLOT_X0, PLOT_Y0, PLOT_X1 - 1, PLOT_Y1 - 1);
  HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);

  for (int16_t screen_y = PLOT_Y0; screen_y < PLOT_Y1; screen_y++) {
    for (uint16_t px = 0; px < PLOT_W; px++) {
      uint16_t color = BLACK;

      if (screen_y == PLOT_Y0 + PLOT_H / 2)
        color = DARKGREY;
      if (screen_y == PLOT_Y0 + PLOT_H / 4)
        color = DARKGREY;
      if (screen_y == PLOT_Y0 + 3 * PLOT_H / 4)
        color = DARKGREY;
      if (px == PLOT_W / 2)
        color = DARKGREY;
      if (px == PLOT_W / 4)
        color = DARKGREY;
      if (px == 3 * PLOT_W / 4)
        color = DARKGREY;

      int16_t d1_top =
          y1_arr[px] < y1_next_arr[px] ? y1_arr[px] : y1_next_arr[px];
      int16_t d1_bot =
          y1_arr[px] > y1_next_arr[px] ? y1_arr[px] : y1_next_arr[px];
      if (screen_y >= d1_top && screen_y <= d1_bot)
        color = CYAN;

      int16_t d2_top =
          y2_arr[px] < y2_next_arr[px] ? y2_arr[px] : y2_next_arr[px];
      int16_t d2_bot =
          y2_arr[px] > y2_next_arr[px] ? y2_arr[px] : y2_next_arr[px];
      if (screen_y >= d2_top && screen_y <= d2_bot)
        color = YELLOW;

      spibuf[px * 2] = color >> 8;
      spibuf[px * 2 + 1] = color & 0xFF;
    }

    while (!spi_dma_done) {
    }
    spi_dma_done = 0;
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(&hspi1, spibuf, PLOT_W * 2);
  }

  /* ── Wait for last DMA to finish, then release SPI back to blocking mode ──
   */
  while (!spi_dma_done) {
  }
  spi_dma_done = 1;
  uint32_t now = DWT->CYCCNT;

  /* only draw labels every ~144M cycles = 1 second at 144MHz */
  if ((now - last_label_time) > 144000000UL) {
    last_label_time = now;
    Draw_Time_Labels();
  }
  HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);

  /* ── Now safe to use blocking driver functions ── */
}