zephyr_library()
zephyr_library_sources_ifdef(CONFIG_UART_DW_APB uart_dw_apb.c)
