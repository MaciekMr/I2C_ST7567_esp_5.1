set(srcs "lcd_SSD1306.cpp" "i2c_class.cpp" "main.cpp")

idf_component_register(SRCS "lcd_SSD1306.cpp" "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES #[[esp_driver_i2c]] driver spi_flash)

target_compile_options(${COMPONENT_LIB} PUBLIC "-Wno-unused-variable" "-Wno-missing-field-initializers" "-fpermissive" -std=gnu++20)                    