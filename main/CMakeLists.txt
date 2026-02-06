set(srcs)

# if(CONFIG_EXAMPLE_ENABLE_RF_TESTING_CONFIGURATION_COMMAND)
# list(APPEND srcs
#     )
# endif()

idf_component_register(SRCS "doorbell.c" "${srcs}"
                    PRIV_REQUIRES bt nvs_flash esp_driver_uart console
                    REQUIRES driver
                    INCLUDE_DIRS ".")
