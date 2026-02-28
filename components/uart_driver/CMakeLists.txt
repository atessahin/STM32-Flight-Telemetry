idf_component_register(SRCS "uart_rx.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES telemetry_core driver log)