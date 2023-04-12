include_guard()
message("component_codec_adapters component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/port/fsl_codec_adapter.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/port
)


#OR Logic component
if(CONFIG_USE_driver_codec_MIMX8UD7_cm33)
     include(driver_codec_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_component_wm8904_adapter_MIMX8UD7_cm33)
     include(component_wm8904_adapter_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_driver_codec_MIMX8UD7_cm33)
     include(driver_codec_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_component_wm8960_adapter_MIMX8UD7_cm33)
     include(component_wm8960_adapter_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_driver_codec_MIMX8UD7_cm33)
     include(driver_codec_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_component_wm8962_adapter_MIMX8UD7_cm33)
     include(component_wm8962_adapter_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_driver_codec_MIMX8UD7_cm33)
     include(driver_codec_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_component_wm8524_adapter_MIMX8UD7_cm33)
     include(component_wm8524_adapter_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_driver_codec_MIMX8UD7_cm33)
     include(driver_codec_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_component_sgtl_adapter_MIMX8UD7_cm33)
     include(component_sgtl_adapter_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_driver_codec_MIMX8UD7_cm33)
     include(driver_codec_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_component_da7212_adapter_MIMX8UD7_cm33)
     include(component_da7212_adapter_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_driver_codec_MIMX8UD7_cm33)
     include(driver_codec_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_component_ak4497_adapter_MIMX8UD7_cm33)
     include(component_ak4497_adapter_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_driver_codec_MIMX8UD7_cm33)
     include(driver_codec_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_component_tfa9xxx_adapter_MIMX8UD7_cm33)
     include(component_tfa9xxx_adapter_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_driver_codec_MIMX8UD7_cm33)
     include(driver_codec_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_component_tfa9896_adapter_MIMX8UD7_cm33)
     include(component_tfa9896_adapter_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_driver_codec_MIMX8UD7_cm33)
     include(driver_codec_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_component_cs42888_adapter_MIMX8UD7_cm33)
     include(component_cs42888_adapter_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_driver_common_MIMX8UD7_cm33)
     include(driver_common_MIMX8UD7_cm33)
endif()
if(CONFIG_USE_component_cs42448_adapter_MIMX8UD7_cm33)
     include(component_cs42448_adapter_MIMX8UD7_cm33)
endif()
if(NOT (CONFIG_USE_driver_codec_MIMX8UD7_cm33 OR CONFIG_USE_component_wm8904_adapter_MIMX8UD7_cm33 OR CONFIG_USE_driver_codec_MIMX8UD7_cm33 OR CONFIG_USE_component_wm8960_adapter_MIMX8UD7_cm33 OR CONFIG_USE_driver_codec_MIMX8UD7_cm33 OR CONFIG_USE_component_wm8962_adapter_MIMX8UD7_cm33 OR CONFIG_USE_driver_codec_MIMX8UD7_cm33 OR CONFIG_USE_component_wm8524_adapter_MIMX8UD7_cm33 OR CONFIG_USE_driver_codec_MIMX8UD7_cm33 OR CONFIG_USE_component_sgtl_adapter_MIMX8UD7_cm33 OR CONFIG_USE_driver_codec_MIMX8UD7_cm33 OR CONFIG_USE_component_da7212_adapter_MIMX8UD7_cm33 OR CONFIG_USE_driver_codec_MIMX8UD7_cm33 OR CONFIG_USE_component_ak4497_adapter_MIMX8UD7_cm33 OR CONFIG_USE_driver_codec_MIMX8UD7_cm33 OR CONFIG_USE_component_tfa9xxx_adapter_MIMX8UD7_cm33 OR CONFIG_USE_driver_codec_MIMX8UD7_cm33 OR CONFIG_USE_component_tfa9896_adapter_MIMX8UD7_cm33 OR CONFIG_USE_driver_codec_MIMX8UD7_cm33 OR CONFIG_USE_component_cs42888_adapter_MIMX8UD7_cm33 OR CONFIG_USE_driver_common_MIMX8UD7_cm33 OR CONFIG_USE_component_cs42448_adapter_MIMX8UD7_cm33))
    message(WARNING "Since driver_codec_MIMX8UD7_cm33/component_wm8904_adapter_MIMX8UD7_cm33/component_wm8960_adapter_MIMX8UD7_cm33/component_wm8962_adapter_MIMX8UD7_cm33/component_wm8524_adapter_MIMX8UD7_cm33/component_sgtl_adapter_MIMX8UD7_cm33/component_da7212_adapter_MIMX8UD7_cm33/component_ak4497_adapter_MIMX8UD7_cm33/component_tfa9xxx_adapter_MIMX8UD7_cm33/component_tfa9896_adapter_MIMX8UD7_cm33/component_cs42888_adapter_MIMX8UD7_cm33/driver_common_MIMX8UD7_cm33/component_cs42448_adapter_MIMX8UD7_cm33 is not included at first or config in config.cmake file, use driver_codec_MIMX8UD7_cm33/component_wm8904_adapter_MIMX8UD7_cm33 by default.")
    include(driver_codec_MIMX8UD7_cm33)
    include(component_wm8904_adapter_MIMX8UD7_cm33)
endif()

