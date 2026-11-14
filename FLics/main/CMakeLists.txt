idf_component_register(
    SRCS "main.cpp" "VL53L0X.cpp" "i2c.cpp" "bno086_imu.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_driver_uart esp_driver_i2c esp_driver_gpio ceva_sh2
)
