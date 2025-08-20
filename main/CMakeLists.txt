# file(GLOB_RECURSE ADAPTERS_SRC "adapters/*.cpp")
# file(GLOB_RECURSE USECASES_SRC "usecases/*.cpp")
# file(GLOB_RECURSE ENTITIES_SRC "entities/*.cpp" "entities/*.c")
# file(GLOB_RECURSE IMAGES_SRC_BASE "images/*.c")
# file(GLOB_RECURSE FONTS_SRC_BASE "fonts/*.c")
# file(GLOB_RECURSE IMAGES_SRC_DARK  "images/dark_theme/*.c")

file(GLOB_RECURSE BLE "./BLE/*.cpp")
file(GLOB_RECURSE Entities "./entities/*.cpp")
file(GLOB_RECURSE UI  "./UI/*.cpp")
file(GLOB_RECURSE adapt  "./adapt/*.cpp")




idf_component_register(SRCS "main.cpp" ${BLE} ${Entities} ${UI} ${adapt}
                    REQUIRES spi_flash esp32_display_panel esp_timer lvgl esp-nimble-cpp nvs_flash
                    INCLUDE_DIRS "." "BLE" "entities" "UI" "HeaderFiles" "adapt")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)
# The following code is to avoid the error:
# lvgl_v8_port/managed_components/lvgl__lvgl/demos/stress/lv_demo_stress.c:92:29: error: format '%d' expects argument of
# type 'int', but argument 6 has type 'uint32_t' {aka 'long unsigned int'} [-Werror=format=]

# Get the exact component name
idf_build_get_property(build_components BUILD_COMPONENTS)
foreach(COMPONENT ${build_components})
    if(COMPONENT MATCHES "lvgl" OR COMPONENT MATCHES "lvgl__lvgl")
        set(TARGET_COMPONENT ${COMPONENT})
        break()
    endif()
endforeach()
# Get the component library
if(TARGET_COMPONENT STREQUAL "")
    message(FATAL_ERROR "Component 'lvgl' not found.")
else()
    idf_component_get_property(LVGL_LIB ${TARGET_COMPONENT} COMPONENT_LIB)
endif()
target_compile_options(${LVGL_LIB} PRIVATE "-Wno-format")
set(TARGET_COMPONENT "")
