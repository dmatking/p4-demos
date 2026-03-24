// Copyright 2025 David M. King
// SPDX-License-Identifier: Apache-2.0
//
// Animated graphics demos for Waveshare ESP32-P4 720x720 LCD (ST7703, MIPI-DSI)

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_cache.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_st7703.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "driver/ppa.h"

#define TAG "APP"

// Display geometry
#define W       720
#define H       720
#define BPP     3
#define FB_SIZE (W * H * BPP)

// MIPI-DSI config for Waveshare ST7703
#define DSI_LANE_NUM    2
#define DSI_LANE_MBPS   480
#define DSI_DPI_CLK_MHZ 38
#define DSI_PHY_LDO_CHAN 3
#define DSI_PHY_LDO_MV  2500
#define DSI_BK_LIGHT_GPIO 26
#define DSI_RST_GPIO      27

#define DEMO_SECS 30
#define NUM_STARS 800

static uint8_t *fb;         // panel's live framebuffer (scanned by hardware)
static uint8_t *backbuf;    // render buffer (draw here, then copy to fb)
static uint8_t *backbuf_a;  // double-buffer A
static uint8_t *backbuf_b;  // double-buffer B
static uint8_t *blackbuf;   // all-zeros buffer for PPA fade-to-black blend
static esp_lcd_panel_handle_t panel;
static ppa_client_handle_t ppa_blend_client;
static ppa_client_handle_t ppa_srm_client;
static SemaphoreHandle_t flush_done_sem;
static bool flush_pending;

// --- Sin/Hue lookup tables ---
#define SIN_LUT_SIZE 1024
#define SIN_LUT_MASK (SIN_LUT_SIZE - 1)
static int16_t sin_lut[SIN_LUT_SIZE];

static void init_sin_lut(void)
{
    for (int i = 0; i < SIN_LUT_SIZE; i++)
        sin_lut[i] = (int16_t)(sinf(i * 2.0f * M_PI / SIN_LUT_SIZE) * 256.0f);
}

static inline int fast_sin(int angle) { return sin_lut[angle & SIN_LUT_MASK]; }

// Hue LUT: 360 entries, each 3 bytes (B, G, R)
static uint8_t hue_lut[360][3];

static void init_hue_lut(void)
{
    for (int h = 0; h < 360; h++) {
        float hf = (float)h;
        float c = 1.0f, x = c * (1.0f - fabsf(fmodf(hf / 60.0f, 2.0f) - 1.0f));
        float r, g, b;
        if      (hf < 60)  { r = c; g = x; b = 0; }
        else if (hf < 120) { r = x; g = c; b = 0; }
        else if (hf < 180) { r = 0; g = c; b = x; }
        else if (hf < 240) { r = 0; g = x; b = c; }
        else if (hf < 300) { r = x; g = 0; b = c; }
        else               { r = c; g = 0; b = x; }
        hue_lut[h][0] = (uint8_t)(b * 255);
        hue_lut[h][1] = (uint8_t)(g * 255);
        hue_lut[h][2] = (uint8_t)(r * 255);
    }
}

static inline void set_pixel_hue(int x, int y, int h)
{
    h = h % 360;
    if (h < 0) h += 360;
    uint8_t *p = backbuf + (y * W + x) * BPP;
    p[0] = hue_lut[h][0];
    p[1] = hue_lut[h][1];
    p[2] = hue_lut[h][2];
}

static inline void set_pixel_rgb(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t *p = backbuf + (y * W + x) * BPP;
    p[0] = b; p[1] = g; p[2] = r;
}

// PPA SRM completion callback — signals semaphore from ISR context
static bool flush_done_cb(ppa_client_handle_t client, ppa_event_data_t *event_data, void *user_data)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(flush_done_sem, &woken);
    return (woken == pdTRUE);
}

// Wait for any in-flight async flush to complete
static void flush_wait(void)
{
    if (flush_pending) {
        xSemaphoreTake(flush_done_sem, portMAX_DELAY);
        esp_cache_msync(fb, FB_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        esp_lcd_panel_draw_bitmap(panel, 0, 0, W, H, fb);
        flush_pending = false;
    }
}

// Start async DMA copy of backbuf → fb, then swap backbuf so CPU can
// immediately begin rendering the next frame while the copy runs.
static void flush(void)
{
    flush_wait();
    esp_cache_msync(backbuf, FB_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    ppa_srm_oper_config_t srm_cfg = {
        .in = {
            .buffer = backbuf,
            .pic_w = W, .pic_h = H,
            .block_w = W, .block_h = H,
            .block_offset_x = 0, .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB888,
        },
        .out = {
            .buffer = fb,
            .buffer_size = FB_SIZE,
            .pic_w = W, .pic_h = H,
            .block_offset_x = 0, .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB888,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = 1.0f,
        .scale_y = 1.0f,
        .mode = PPA_TRANS_MODE_NON_BLOCKING,
    };
    ESP_ERROR_CHECK(ppa_do_scale_rotate_mirror(ppa_srm_client, &srm_cfg));
    flush_pending = true;
    // Swap to other backbuf — CPU can render next frame while DMA copies
    backbuf = (backbuf == backbuf_a) ? backbuf_b : backbuf_a;
}

// ==========================================================================
// Display init
// ==========================================================================
static void display_init(void)
{
    // Power MIPI PHY
    esp_ldo_channel_handle_t ldo = NULL;
    esp_ldo_channel_config_t ldo_cfg = { .chan_id = DSI_PHY_LDO_CHAN, .voltage_mv = DSI_PHY_LDO_MV };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo));

    // DSI bus
    esp_lcd_dsi_bus_handle_t dsi_bus;
    esp_lcd_dsi_bus_config_t bus_cfg = {
        .bus_id = 0,
        .num_data_lanes = DSI_LANE_NUM,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = DSI_LANE_MBPS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &dsi_bus));

    // DBI command IO
    esp_lcd_panel_io_handle_t dbi_io;
    esp_lcd_dbi_io_config_t dbi_cfg = { .virtual_channel = 0, .lcd_cmd_bits = 8, .lcd_param_bits = 8 };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(dsi_bus, &dbi_cfg, &dbi_io));

    // DPI video panel
    esp_lcd_dpi_panel_config_t dpi_cfg = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = DSI_DPI_CLK_MHZ,
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB888,
        .num_fbs = 1,
        .video_timing = {
            .h_size = W, .v_size = H,
            .hsync_back_porch = 50, .hsync_pulse_width = 20, .hsync_front_porch = 50,
            .vsync_back_porch = 20, .vsync_pulse_width = 4, .vsync_front_porch = 20,
        },
        .flags = { .use_dma2d = true },
    };

    st7703_vendor_config_t vendor_cfg = {
        .flags = { .use_mipi_interface = 1 },
        .mipi_config = { .dsi_bus = dsi_bus, .dpi_config = &dpi_cfg },
    };
    esp_lcd_panel_dev_config_t dev_cfg = {
        .reset_gpio_num = DSI_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 24,
        .vendor_config = &vendor_cfg,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7703(dbi_io, &dev_cfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    // Backlight (active low on Waveshare)
    gpio_config_t bk_cfg = { .mode = GPIO_MODE_OUTPUT, .pin_bit_mask = 1ULL << DSI_BK_LIGHT_GPIO };
    ESP_ERROR_CHECK(gpio_config(&bk_cfg));
    gpio_set_level(DSI_BK_LIGHT_GPIO, 0);

    // Get panel framebuffer
    void *fb0 = NULL;
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(panel, 1, &fb0));
    fb = (uint8_t *)fb0;

    // Allocate double back buffers in PSRAM for tear-free rendering
    backbuf_a = heap_caps_aligned_calloc(64, FB_SIZE, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    backbuf_b = heap_caps_aligned_calloc(64, FB_SIZE, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    assert(backbuf_a && backbuf_b);
    backbuf = backbuf_a;
    memset(fb, 0, FB_SIZE);

    // Allocate black buffer for PPA fade-to-black blend
    blackbuf = heap_caps_aligned_calloc(64, FB_SIZE, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    assert(blackbuf);

    // Register PPA clients
    ppa_client_config_t blend_cfg = { .oper_type = PPA_OPERATION_BLEND, .max_pending_trans_num = 1 };
    ESP_ERROR_CHECK(ppa_register_client(&blend_cfg, &ppa_blend_client));
    ppa_client_config_t srm_cfg = { .oper_type = PPA_OPERATION_SRM, .max_pending_trans_num = 1 };
    ESP_ERROR_CHECK(ppa_register_client(&srm_cfg, &ppa_srm_client));

    // Register async flush callback and create semaphore
    flush_done_sem = xSemaphoreCreateBinary();
    assert(flush_done_sem);
    ppa_event_callbacks_t srm_cbs = { .on_trans_done = flush_done_cb };
    ESP_ERROR_CHECK(ppa_client_register_event_callbacks(ppa_srm_client, &srm_cbs));

    flush();
    ESP_LOGI(TAG, "Display init done: %dx%d RGB888, PPA SRM+blend enabled", W, H);
}

// ==========================================================================
// Demo 1: Mandelbrot zoom
// ==========================================================================
static const float zoom_targets[][2] = {
    { -0.74364388703f, 0.13182590421f },
    { -0.16070135f,    1.03757843f    },
    { -1.25066f,       0.02012f       },
};
#define NUM_TARGETS (sizeof(zoom_targets) / sizeof(zoom_targets[0]))

static void run_mandelbrot(void)
{
    ESP_LOGI(TAG, "Mandelbrot zoom");
    int target_idx = 0;
    float scale = 0.006f;
    int64_t start = esp_timer_get_time();

    while ((esp_timer_get_time() - start) < (int64_t)DEMO_SECS * 1000000) {
        float cx = zoom_targets[target_idx][0];
        float cy = zoom_targets[target_idx][1];

        for (int py = 0; py < H; py++) {
            float y0 = cy + (py - H / 2) * scale;
            for (int px = 0; px < W; px++) {
                float x0 = cx + (px - W / 2) * scale;
                float x = 0, y = 0;
                int iter = 0;
                while (x * x + y * y <= 4.0f && iter < 64) {
                    float xt = x * x - y * y + x0;
                    y = 2.0f * x * y + y0;
                    x = xt;
                    iter++;
                }
                if (iter == 64) {
                    set_pixel_rgb(px, py, 0, 0, 0);
                } else {
                    set_pixel_hue(px, py, (iter * 8) % 360);
                }
            }
        }
        flush();
        scale *= 0.50f;
        if (scale < 1e-6f) {
            target_idx = (target_idx + 1) % NUM_TARGETS;
            scale = 0.006f;
        }
    }
}

// ==========================================================================
// Demo 2: Animated plasma
// ==========================================================================
static void run_plasma(void)
{
    ESP_LOGI(TAG, "Animated plasma");
    int t = 0;
    int64_t start = esp_timer_get_time();
    const int SCALE = 4;

    while ((esp_timer_get_time() - start) < (int64_t)DEMO_SECS * 1000000) {
        for (int py = 0; py < H; py++) {
            int ya = py * SCALE + t * 18;
            int ysin = fast_sin(ya);
            for (int px = 0; px < W; px++) {
                int xa = px * SCALE + t * 25;
                int v = fast_sin(xa) + ysin
                      + fast_sin(xa + ya)
                      + fast_sin((px * px + py * py) / 80 + t * 33);
                int hue = (v + 1024) * 360 / 2048 + t * 2;
                set_pixel_hue(px, py, hue);
            }
        }
        flush();
        t++;
    }
}

// ==========================================================================
// Demo 3: Rotating rainbow
// ==========================================================================
static void run_rainbow(void)
{
    ESP_LOGI(TAG, "Rotating rainbow");
    int t = 0;
    int64_t start = esp_timer_get_time();

    while ((esp_timer_get_time() - start) < (int64_t)DEMO_SECS * 1000000) {
        for (int py = 0; py < H; py++) {
            int base_hue = py * 360 / H + t * 3;
            for (int px = 0; px < W; px++) {
                int wave = fast_sin((px + py) * 5 + t * 8) / 8;
                set_pixel_hue(px, py, base_hue + wave);
            }
        }
        flush();
        t++;
    }
}

// ==========================================================================
// Demo 4: Flying starfield
// ==========================================================================
typedef struct {
    float x, y, z;
    uint8_t bright, tint;
} star_t;

static star_t stars[NUM_STARS];

static void init_star(star_t *s, bool far)
{
    s->x = (rand() % 2000 - 1000) / 100.0f;
    s->y = (rand() % 2000 - 1000) / 100.0f;
    s->z = far ? (rand() % 900 + 100) / 100.0f : (rand() % 1000 + 1) / 100.0f;
    s->bright = 150 + rand() % 106;
    s->tint = rand() % 3;
}

static void run_starfield(void)
{
    ESP_LOGI(TAG, "Flying starfield");
    srand(42);
    for (int i = 0; i < NUM_STARS; i++) init_star(&stars[i], false);

    // Precompute nebula
    uint8_t *nebula = heap_caps_malloc(FB_SIZE, MALLOC_CAP_SPIRAM);
    if (nebula) {
        for (int py = 0; py < H; py++) {
            for (int px = 0; px < W; px++) {
                int dx1 = px - W * 35 / 100, dy1 = py - H * 40 / 100;
                int dx2 = px - W * 70 / 100, dy2 = py - H * 65 / 100;
                int d1 = (dx1 * dx1 + dy1 * dy1) / 100 + 1;
                int d2 = (dx2 * dx2 + dy2 * dy2) / 100 + 1;
                int r = 6000 / d1 + 9000 / d2;
                int g = 1500 / d1 + 3000 / d2;
                int b = 12000 / d1 + 4500 / d2;
                if (r > 255) r = 255;
                if (g > 255) g = 255;
                if (b > 255) b = 255;
                uint8_t *p = nebula + (py * W + px) * BPP;
                p[0] = b; p[1] = g; p[2] = r;
            }
        }
    }

    int64_t start = esp_timer_get_time();
    float speed = 0.04f;

    while ((esp_timer_get_time() - start) < (int64_t)DEMO_SECS * 1000000) {
        if (nebula) {
            memcpy(backbuf, nebula, FB_SIZE);
        } else {
            memset(backbuf, 0, FB_SIZE);
        }

        for (int i = 0; i < NUM_STARS; i++) {
            star_t *s = &stars[i];
            s->z -= speed;
            if (s->z <= 0.1f) { init_star(s, true); continue; }

            int sx = W / 2 + (int)(s->x / s->z * 80.0f);
            int sy = H / 2 + (int)(s->y / s->z * 80.0f);
            if (sx < 1 || sx >= W - 1 || sy < 1 || sy >= H - 1) {
                init_star(s, true); continue;
            }

            float depth = 1.0f / (s->z * 0.3f);
            if (depth > 1.0f) depth = 1.0f;
            uint8_t br = (uint8_t)(s->bright * depth);

            uint8_t r = br, g = br, b = br;
            if (s->tint == 0) { r = 255; g = br * 4 / 5; b = br * 3 / 5; }
            else if (s->tint == 1) { r = br * 7 / 10; g = br * 4 / 5; b = 255; }

            set_pixel_rgb(sx, sy, r, g, b);
            if (s->z < 3.0f) {
                set_pixel_rgb(sx + 1, sy, r, g, b);
                set_pixel_rgb(sx, sy + 1, r, g, b);
            }
            if (s->z < 1.5f) {
                set_pixel_rgb(sx - 1, sy, r, g, b);
                set_pixel_rgb(sx, sy - 1, r, g, b);
                set_pixel_rgb(sx + 1, sy + 1, r, g, b);
                set_pixel_rgb(sx - 1, sy + 1, r, g, b);
            }

            if (s->z < 2.5f) {
                int tlen = (int)(4.0f / s->z);
                if (tlen > 8) tlen = 8;
                for (int j = 1; j <= tlen; j++) {
                    int ty = sy - j;
                    if (ty >= 0) {
                        uint8_t fade = br / (j + 1);
                        set_pixel_rgb(sx, ty, fade, fade, fade);
                    }
                }
            }
        }

        flush();
        speed += 0.0002f;
    }

    if (nebula) heap_caps_free(nebula);
}

// ==========================================================================
// Demo 5: Mystify (modern take on the classic Windows screensaver)
// ==========================================================================
#define MYSTIFY_SHAPES  2
#define MYSTIFY_VERTS   4
#define MYSTIFY_MARGIN  5

typedef struct {
    int x[MYSTIFY_VERTS];
    int y[MYSTIFY_VERTS];
    int dx[MYSTIFY_VERTS];
    int dy[MYSTIFY_VERTS];
    int base_hue;
    int hue_speed;
} mystify_poly_t;

static void draw_line(int x0, int y0, int x1, int y1,
                      uint8_t r, uint8_t g, uint8_t b, int thickness)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int half = thickness / 2;

    while (1) {
        for (int oy = -half; oy <= half; oy++) {
            for (int ox = -half; ox <= half; ox++) {
                int px = x0 + ox, py = y0 + oy;
                if (px >= 0 && px < W && py >= 0 && py < H) {
                    set_pixel_rgb(px, py, r, g, b);
                }
            }
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void run_mystify(void)
{
    ESP_LOGI(TAG, "Mystify");
    srand(esp_timer_get_time());
    flush_wait();
    memset(backbuf_a, 0, FB_SIZE);
    memset(backbuf_b, 0, FB_SIZE);

    mystify_poly_t polys[MYSTIFY_SHAPES];
    for (int s = 0; s < MYSTIFY_SHAPES; s++) {
        mystify_poly_t *p = &polys[s];
        for (int v = 0; v < MYSTIFY_VERTS; v++) {
            p->x[v] = MYSTIFY_MARGIN + rand() % (W - 2 * MYSTIFY_MARGIN);
            p->y[v] = MYSTIFY_MARGIN + rand() % (H - 2 * MYSTIFY_MARGIN);
            p->dx[v] = (rand() % 4 + 2) * (rand() % 2 ? 1 : -1);
            p->dy[v] = (rand() % 4 + 2) * (rand() % 2 ? 1 : -1);
        }
        p->base_hue = s * 120;
        p->hue_speed = 1;
    }

    int64_t start = esp_timer_get_time();

    while ((esp_timer_get_time() - start) < (int64_t)DEMO_SECS * 1000000) {
        // Fade toward black using PPA hardware blend
        // After flush, backbuf was swapped — the previous frame is in the OTHER buffer.
        // Blend: previous frame (faded) → current backbuf
        {
            uint8_t *prev = (backbuf == backbuf_a) ? backbuf_b : backbuf_a;
            flush_wait(); // ensure previous flush finished reading prev
            esp_cache_msync(prev, FB_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
            ppa_blend_oper_config_t blend_cfg = {
                .in_bg = {
                    .buffer = prev,
                    .pic_w = W, .pic_h = H,
                    .block_w = W, .block_h = H,
                    .block_offset_x = 0, .block_offset_y = 0,
                    .blend_cm = PPA_BLEND_COLOR_MODE_RGB888,
                },
                .in_fg = {
                    .buffer = blackbuf,
                    .pic_w = W, .pic_h = H,
                    .block_w = W, .block_h = H,
                    .block_offset_x = 0, .block_offset_y = 0,
                    .blend_cm = PPA_BLEND_COLOR_MODE_RGB888,
                },
                .out = {
                    .buffer = backbuf,
                    .buffer_size = FB_SIZE,
                    .pic_w = W, .pic_h = H,
                    .block_offset_x = 0, .block_offset_y = 0,
                    .blend_cm = PPA_BLEND_COLOR_MODE_RGB888,
                },
                .bg_alpha_update_mode = PPA_ALPHA_NO_CHANGE,
                .fg_alpha_update_mode = PPA_ALPHA_FIX_VALUE,
                .fg_alpha_fix_val = 16,
                .mode = PPA_TRANS_MODE_BLOCKING,
            };
            ESP_ERROR_CHECK(ppa_do_blend(ppa_blend_client, &blend_cfg));
            esp_cache_msync(backbuf, FB_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
        }

        // Draw current shape for each polygon, then update vertices
        for (int s = 0; s < MYSTIFY_SHAPES; s++) {
            mystify_poly_t *p = &polys[s];
            int hue = p->base_hue % 360;
            uint8_t r = hue_lut[hue][2];
            uint8_t g = hue_lut[hue][1];
            uint8_t b = hue_lut[hue][0];

            for (int i = 0; i < MYSTIFY_VERTS; i++) {
                int next = (i + 1) % MYSTIFY_VERTS;
                draw_line(p->x[i], p->y[i], p->x[next], p->y[next], r, g, b, 3);
            }

            // Update vertices — bounce off edges
            for (int v = 0; v < MYSTIFY_VERTS; v++) {
                p->x[v] += p->dx[v];
                p->y[v] += p->dy[v];
                if (p->x[v] <= MYSTIFY_MARGIN)     { p->x[v] = MYSTIFY_MARGIN;     p->dx[v] =  abs(p->dx[v]); }
                if (p->x[v] >= W - MYSTIFY_MARGIN)  { p->x[v] = W - MYSTIFY_MARGIN - 1; p->dx[v] = -abs(p->dx[v]); }
                if (p->y[v] <= MYSTIFY_MARGIN)      { p->y[v] = MYSTIFY_MARGIN;     p->dy[v] =  abs(p->dy[v]); }
                if (p->y[v] >= H - MYSTIFY_MARGIN)  { p->y[v] = H - MYSTIFY_MARGIN - 1; p->dy[v] = -abs(p->dy[v]); }
            }

            p->base_hue = (p->base_hue + p->hue_speed) % 360;
        }

        flush();
    }
}

// ==========================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "P4 demos starting...");
    display_init();
    init_sin_lut();
    init_hue_lut();

    while (1) {
        run_mandelbrot();
        run_plasma();
        run_rainbow();
        run_starfield();
        run_mystify();
    }
}
