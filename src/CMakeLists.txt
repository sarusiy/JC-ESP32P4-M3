idf_component_register(
	SRCS "bringup_s3.c"
	REQUIRES esp_driver_gpio led_strip spi_flash esp_wifi esp_netif esp_event esp_http_server nvs_flash bt driver
)
# ESP-IDF v6.1-beta1's legacy driver/twai.h emits a #warning nudging
# towards the new esp_twai.h API -- this project's default -Werror
# promotes that to a build failure. Bring-up code deliberately still uses
# the legacy install/start/receive API (simpler, well-documented, fine for
# a throwaway test -- see bringup_s3.c's can_receive_task doc comment);
# suppressing just this one warning class rather than switching APIs.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=cpp)
