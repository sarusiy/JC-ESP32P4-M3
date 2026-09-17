idf_component_register(
	SRCS "bringup_s3.c"
	REQUIRES esp_driver_gpio led_strip spi_flash
)
