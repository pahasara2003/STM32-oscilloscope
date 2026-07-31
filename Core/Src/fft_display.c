#include "fft_display.h"
#include "ILI9341_GFX.h"
#include "ILI9341_STM32_Driver.h"
#include "fonts.h"
#include <math.h>
#include <stdio.h>

#define FFT_SIZE 512

static arm_rfft_fast_instance_f32 fft_instance;
static float32_t fft_in[FFT_SIZE];
static float32_t fft_out[FFT_SIZE];
static float32_t fft_mag[FFT_SIZE / 2];

extern float    fft_y_scale;
extern int16_t  fft_y_offset;
extern float    fft_x_scale;
extern int16_t  fft_x_offset;
extern float    sampling_rate;
extern uint32_t last_label_time;

/* ======================================= */
/*  Draw_FFT_Labels
 *
 *  FIX 1: Function signature now matches the call site — no parameter needed
 *          because sampling_rate is already extern.
 *
 *  FIX 2: X-axis frequency labels now computed directly from the visible
 *          frequency window instead of inverting the per-pixel transform,
 *          which gave wrong values whenever fft_x_scale != 1 or
 *          fft_x_offset != 0.
 *
 *  FIX 3: Y-axis inversion order was already mathematically correct
 *          (divide scale first, then subtract offset) — kept as-is.
 * ======================================= */
void Draw_FFT_Labels(void)
{
    float nyquist_khz = sampling_rate / 2.0f;       /* kHz → Nyquist */

    /* ------------------------------------------------------------------
     * X axis: frequency labels at 25 %, 50 %, 75 % of the plot width.
     *
     * The visible frequency window in bin-space is:
     *   left  bin-pixel = fft_x_offset                         (pan)
     *   right bin-pixel = PLOT_W / fft_x_scale + fft_x_offset  (pan + zoom)
     *
     * A screen pixel px maps to bin-pixel:
     *   bp = px / fft_x_scale + fft_x_offset
     *
     * Frequency of that bin-pixel:
     *   freq_khz = (bp / PLOT_W) * nyquist_khz
     * ------------------------------------------------------------------ */
    float x_fracs[3] = { 0.25f, 0.50f, 0.75f };

    for (int i = 0; i < 3; i++)
    {
        /* screen pixel at this fraction */
        float px = x_fracs[i] * (float)PLOT_W;

        /* map screen pixel → bin-pixel using the same transform as Plot_FFT */
        float bp = (px - (PLOT_W / 2.0f)) / fft_x_scale
                   + (PLOT_W / 2.0f)
                   + fft_x_offset;

        if (bp < 0.0f)        bp = 0.0f;
        if (bp >= (float)PLOT_W) bp = (float)PLOT_W - 1.0f;

        float freq_khz = (bp / (float)PLOT_W) * nyquist_khz;

        char buf[16];
        if (freq_khz >= 1.0f)
        {
            int whole = (int)freq_khz;
            int frac  = (int)((freq_khz - whole) * 100.0f);
            sprintf(buf, "%d.%02d kHz", whole, frac);
        }
        else
        {
            sprintf(buf, "%d Hz", (int)(freq_khz * 1000.0f));
        }

        uint16_t x_draw = (uint16_t)(PLOT_X0 + x_fracs[i] * PLOT_W) - 15;
        uint16_t y_draw = PLOT_Y1 + 5;
        ILI9341_Draw_Text_Arial(buf, x_draw, y_draw, WHITE, BLACK, Arial_Narrow8x12);
    }

    /* ------------------------------------------------------------------
     * Y axis: dB labels at 25 %, 50 %, 75 % of the plot height.
     *
     * Inverse of mag_to_y():
     *   Forward:  screen_y = PLOT_Y0 + PLOT_H/2 - ((norm-0.5)*PLOT_H + offset)*scale
     *   Inverse:
     *     1. y_raw   = PLOT_Y0 + PLOT_H/2 - screen_y
     *     2. y_unscl = y_raw / fft_y_scale          (undo scale)
     *     3. y_unoff = y_unscl - fft_y_offset        (undo shift)
     *     4. norm    = y_unoff / PLOT_H + 0.5
     *     5. db      = norm * (0 - db_min) + db_min
     * ------------------------------------------------------------------ */
    float db_min = -60.0f;

    float y_fracs[3] = { 0.25f, 0.50f, 0.75f };

    for (int i = 0; i < 3; i++)
    {
        uint16_t screen_y = (uint16_t)(PLOT_Y0 + y_fracs[i] * PLOT_H);

        /* step 1 – undo screen flip */
        float y = (float)(PLOT_Y0 + PLOT_H / 2) - (float)screen_y;

        /* step 2 – undo scale */
        y /= fft_y_scale;

        /* step 3 – undo shift */
        y -= (float)fft_y_offset;

        /* step 4 – undo centering → normalised 0..1 */
        float norm = y / (float)PLOT_H + 0.5f;

        /* step 5 – normalised → dB */
        float db = norm * (0.0f - db_min) + db_min;

        if (db >  0.0f)   db =  0.0f;
        if (db < db_min)  db = db_min;

        char buf[16];
        int whole = (int)db;                          /* already negative */
        int frac  = (int)(-(db - (float)whole) * 10); /* one decimal place */
        sprintf(buf, "%d.%d dB", whole, frac);

        uint16_t x_draw = PLOT_X0 - 15;
        uint16_t y_draw = screen_y - 15;
        ILI9341_Draw_Text_Arial_Vertical(buf, x_draw, y_draw, WHITE, BLACK, Arial_Narrow8x12);
    }
}

/* ======================================= */
void FFT_Init(void)
{
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
}

/* ======================================= */
/*  mag_to_y
 *  Maps a linear magnitude value to a screen Y coordinate.
 *  Zoom (fft_y_scale) is applied AFTER the pan (fft_y_offset) so that
 *  the offset defines the zoom centre.
 * ======================================= */
static inline int16_t mag_to_y(float32_t mag, float32_t max_mag)
{
    if (mag    < 1e-6f) mag    = 1e-6f;
    if (max_mag < 1e-6f) max_mag = 1e-6f;

    float32_t db     = 20.0f * log10f(mag);
    float32_t db_min = -60.0f;

    if (db < db_min) db = db_min;

    /* normalise 0..1  (0 = quietest, 1 = loudest) */
    float32_t norm = (db - db_min) / (0.0f - db_min);

    /* centre around the mid-line */
    float32_t y = (norm - 0.5f) * (float32_t)PLOT_H;

    /* 1. shift (defines the zoom centre) */
    y += (float32_t)fft_y_offset;

    /* 2. scale */
    y *= fft_y_scale;

    /* convert to screen row (Y increases downward) */
    int16_t out = (int16_t)((float32_t)(PLOT_Y0 + PLOT_H / 2) - y);

    /* clamp to plot area */
    if (out < PLOT_Y0)      out = PLOT_Y0;
    if (out >= (int16_t)PLOT_Y1) out = (int16_t)(PLOT_Y1 - 1);

    return out;
}

/* ======================================= */
void Plot_FFT(void)
{
    static int16_t y1_arr[PLOT_W];
    static int16_t y1_next_arr[PLOT_W];
    static int16_t y2_arr[PLOT_W];
    static int16_t y2_next_arr[PLOT_W];
    static uint8_t spibuf[PLOT_W * 2];

    /* ===== CH1 FFT ===== */
    for (int i = 0; i < FFT_SIZE; i++)
        fft_in[i] = (float32_t)displayed_data1[i] / ADC_MAX;

    arm_rfft_fast_f32(&fft_instance, fft_in, fft_out, 0);
    arm_cmplx_mag_f32(fft_out, fft_mag, FFT_SIZE / 2);
    fft_mag[0] = 0.0f;   /* zero DC bin */

    float32_t max_mag1 = 0.0f;
    for (int i = 1; i < FFT_SIZE / 2; i++)
        if (fft_mag[i] > max_mag1) max_mag1 = fft_mag[i];

    /* snapshot CH1 spectrum before re-using the shared buffer */
    float32_t fft_mag_ch1[FFT_SIZE / 2];
    for (int i = 0; i < FFT_SIZE / 2; i++)
        fft_mag_ch1[i] = fft_mag[i];

    /* ===== CH2 FFT ===== */
    for (int i = 0; i < FFT_SIZE; i++)
        fft_in[i] = (float32_t)displayed_data2[i] / ADC_MAX;

    arm_rfft_fast_f32(&fft_instance, fft_in, fft_out, 0);
    arm_cmplx_mag_f32(fft_out, fft_mag, FFT_SIZE / 2);
    fft_mag[0] = 0.0f;   /* zero DC bin */

    float32_t max_mag2 = 0.0f;
    for (int i = 1; i < FFT_SIZE / 2; i++)
        if (fft_mag[i] > max_mag2) max_mag2 = fft_mag[i];

    /* ===== Shared amplitude scale (both channels normalised together) ===== */
    float32_t global_max = (max_mag1 > max_mag2) ? max_mag1 : max_mag2;

    /* ===== Pre-compute Y pixel arrays with X zoom/pan transform ===== */
    for (uint16_t px = 0; px < PLOT_W; px++)
    {
        /* ---- X transform: screen pixel → bin-pixel ---- */
        float x = ((float)px - (PLOT_W / 2.0f)) / fft_x_scale
                  + (PLOT_W / 2.0f)
                  + fft_x_offset;

        if (x < 0.0f)          x = 0.0f;
        if (x >= (float)PLOT_W) x = (float)PLOT_W - 1.0f;

        float x_next = ((float)(px + 1) - (PLOT_W / 2.0f)) / fft_x_scale
                       + (PLOT_W / 2.0f)
                       + fft_x_offset;

        if (x_next < 0.0f)          x_next = 0.0f;
        if (x_next >= (float)PLOT_W) x_next = (float)PLOT_W - 1.0f;

        /* bin-pixel → FFT bin index */
        uint16_t bin      = (uint16_t)((uint32_t)x      * (FFT_SIZE / 2) / PLOT_W);
        uint16_t bin_next = (uint16_t)((uint32_t)x_next * (FFT_SIZE / 2) / PLOT_W);

        if (bin      >= FFT_SIZE / 2) bin      = FFT_SIZE / 2 - 1;
        if (bin_next >= FFT_SIZE / 2) bin_next = FFT_SIZE / 2 - 1;

        /* CH1 */
        y1_arr[px]      = mag_to_y(fft_mag_ch1[bin],      global_max);
        y1_next_arr[px] = mag_to_y(fft_mag_ch1[bin_next], global_max);

        /* CH2 — fft_mag[] still holds CH2 result */
        y2_arr[px]      = mag_to_y(fft_mag[bin],      global_max);
        y2_next_arr[px] = mag_to_y(fft_mag[bin_next], global_max);
    }

    /* ===== Render scanline by scanline via DMA ===== */
    ILI9341_Set_Address(PLOT_X0, PLOT_Y0, PLOT_X1 - 1, PLOT_Y1 - 1);
    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);

    for (int16_t screen_y = PLOT_Y0; screen_y < PLOT_Y1; screen_y++)
    {
        for (uint16_t px = 0; px < PLOT_W; px++)
        {
            uint16_t color = BLACK;

            /* ---- Grid lines ---- */
            /* bottom border and left border */
            if (screen_y == PLOT_Y1 - 1 || px == 0)
                color = DARKGREY;

            /* vertical grid lines at 25 %, 50 %, 75 % */
            if (px == PLOT_W / 4 || px == PLOT_W / 2 || px == 3 * PLOT_W / 4)
                color = DARKGREY;

            /* horizontal grid lines at 25 %, 50 %, 75 % */
            if (screen_y == PLOT_Y0 + PLOT_H / 4  ||
                screen_y == PLOT_Y0 + PLOT_H / 2  ||
                screen_y == PLOT_Y0 + 3 * PLOT_H / 4)
                color = DARKGREY;

            /* ---- CH1 (CYAN) — filled vertical segment ---- */
            int16_t d1_top = (y1_arr[px] < y1_next_arr[px]) ? y1_arr[px] : y1_next_arr[px];
            int16_t d1_bot = (y1_arr[px] > y1_next_arr[px]) ? y1_arr[px] : y1_next_arr[px];
            if (screen_y >= d1_top && screen_y <= d1_bot)
                color = CYAN;

            /* ---- CH2 (YELLOW) — filled vertical segment ---- */
            int16_t d2_top = (y2_arr[px] < y2_next_arr[px]) ? y2_arr[px] : y2_next_arr[px];
            int16_t d2_bot = (y2_arr[px] > y2_next_arr[px]) ? y2_arr[px] : y2_next_arr[px];
            if (screen_y >= d2_top && screen_y <= d2_bot)
                color = YELLOW;

            spibuf[px * 2]     = (uint8_t)(color >> 8);
            spibuf[px * 2 + 1] = (uint8_t)(color & 0xFF);
        }

        while (!spi_dma_done) {}
        spi_dma_done = 0;
        HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
        HAL_SPI_Transmit_DMA(&hspi1, spibuf, PLOT_W * 2);
    }

    while (!spi_dma_done) {}
    spi_dma_done = 1;

    /* ===== Periodic axis label refresh ===== */
    uint32_t now = DWT->CYCCNT;
    if ((now - last_label_time) > 144000000UL)
    {
        last_label_time = now;
        Draw_FFT_Labels();   /* FIX: no argument — sampling_rate is extern */
    }
}