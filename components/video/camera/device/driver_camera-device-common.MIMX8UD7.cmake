# Add set(CONFIG_USE_driver_camera-device-common true) in config.cmake to use this component

include_guard(GLOBAL)
message("${CMAKE_CURRENT_LIST_FILE} component is included.")

if(CONFIG_USE_driver_camera-common AND CONFIG_USE_driver_common)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/.
)

else()

message(SEND_ERROR "driver_camera-device-common.MIMX8UD7 dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()
