idf_component_register(SRCS "app_main.c"
                            "tube_drv.c"
                            "led_strip_encoder.c"
                            "led_drv.c"
                            "temp_drv.c"
                            "spp_console_init.c"
                    INCLUDE_DIRS "." "./include")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
