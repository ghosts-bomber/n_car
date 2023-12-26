idf_component_register(SRCS 
                    "main.cpp"
                    "servo.c"
                    "servo_task.c"                   
                    "oled_task.cpp"
                    "wifi.c"
                    "mqtt.c"

                    "./eye/Eye.cpp"
                    "./eye/AsyncTimer.cpp"
                    "./eye/Eye.cpp"
                    "./eye/EyeVariation.cpp"
                    "./eye/FaceExpression.cpp"
                    "./eye/BlinkAssistant.cpp"
                    "./eye/EyeTransformation.cpp"
                    "./eye/FaceBehavior.cpp"
                    "./eye/LookAssistant.cpp"
                    "./eye/EyeBlink.cpp"
                    "./eye/EyeTransition.cpp"
                    "./eye/Face.cpp"
                    "./eye/Common.cpp"

                    INCLUDE_DIRS 
                    "./"
                    "./eye/"

                    REQUIRES
                    bootloader_support
                    u8g2
                    nvs_flash
                    spi_flash

                    PRIV_REQUIRES
                    mbedtls
                    esp_wifi
                    mqtt
                    json
                    )
target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++11)

