# Add set(CONFIG_USE_utility_assert true) in config.cmake to use this component

include_guard(GLOBAL)
message("${CMAKE_CURRENT_LIST_FILE} component is included.")

if(CONFIG_USE_utility_debug_console)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/fsl_assert.c
)

else()

message(SEND_ERROR "utility_assert.MIMX8UD7 dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()
