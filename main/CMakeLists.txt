idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_lcd esp_lcd_st7703 esp_psram esp_mm esp_timer log driver esp_driver_ppa
)
