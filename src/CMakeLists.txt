idf_component_register(
	SRCS "main.c"
	REQUIRES esp_tinyusb esp_netif esp_event esp_wifi nvs_flash bt espressif__esp_wifi_remote espressif__esp_hosted
)
