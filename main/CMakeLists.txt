
idf_component_register(SRCS "main.cpp"
                            "bluetooth_handler.cpp"
                            "i2s_handler.cpp"
                            "screen_handler.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES 
                    arduino
                    spi_flash 
                    esp_partition
                    "nvs_flash"

                    )

