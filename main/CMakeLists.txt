idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES hal_i2c mpu6050 motion_estimator esp_timer
)
