idf_component_register(SRCS 
                    "main.cpp"
                    "servo.c" 
                    "servo_task.c"                   
                    "oled_task.cpp"
                    "wifi.c"
                    "mqtt.c"
                    "led_task.c"
                    "led_strip_encoder.c"
                    "hid_host.cpp"
                    "esp_hid_gap.c"
                    "XboxControllerNotificationParser.cpp"
                    "motor_task.c"
                    # "adc.c"
                    "potentiometer.c"

                    "controller/process_control.cpp"
                    "controller/motor_controller.cpp"
                    "controller/car_light_controller.cpp"

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
                    "./controller/"

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
                    # bt
                    esp_hid
                    esp_adc
                    )
target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++23)

