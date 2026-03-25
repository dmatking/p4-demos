# P4 Demos

Animated graphics demos for the **Waveshare ESP32-P4-WIFI6-Touch-LCD-4B** (720×720 ST7703 LCD, MIPI-DSI).

## Demos

Four demos cycle every 30 seconds:

- **Mandelbrot zoom** — zooms into three different fractal coordinates, resets when scale gets too small
- **Plasma** — interference pattern using integer sin LUT for performance
- **Rainbow** — rotating hue gradient with sine wave distortion
- **Starfield** — 800 stars flying through nebula clouds, with motion trails and tinted stars

## Hardware

| Detail | Value |
|--------|-------|
| Board | [Waveshare ESP32-P4-WIFI6-Touch-LCD-4B](https://www.waveshare.com/esp32-p4-wifi6-touch-lcd-4b.htm) |
| MCU | ESP32-P4 @ 360 MHz |
| Display | 720×720 RGB888, ST7703 via MIPI-DSI (2 lanes, 480 Mbps) |
| RAM | HEX PSRAM @ 200 MHz |
| Flash | 16 MB |

## Display Notes

- **MIPI-DSI DPI mode** — the panel continuously scans from a PSRAM framebuffer at ~38 MHz pixel clock
- **Back buffer** — all rendering goes to a separate PSRAM buffer, then `memcpy` to the live framebuffer on flush to prevent tearing
- **Cache sync** — `esp_cache_msync()` required after writing to the framebuffer before `esp_lcd_panel_draw_bitmap()`
- **Backlight** — GPIO 26, active low
- **Reset** — GPIO 27

## Build

Requires ESP-IDF v5.5+ with the `esp_lcd_st7703` component (vendored in `components/`).

```sh
idf.py set-target esp32p4   # first time only
idf.py build flash monitor
```
