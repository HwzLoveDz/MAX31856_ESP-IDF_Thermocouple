# set(src_dirs ./src)
# set(include_dirs ./src)
# idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs})
idf_component_register(SRCS "src/Adafruit_MAX31856.cpp"
                    REQUIRES driver esp_timer
                    INCLUDE_DIRS "src"
                    )
