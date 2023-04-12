include_guard()
message("driver_cmsis_lpuart component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/fsl_lpuart_cmsis.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/.
)


include(driver_lpuart_MIMX8UD7_cm33)

include(CMSIS_Driver_Include_USART_MIMX8UD7_cm33)

