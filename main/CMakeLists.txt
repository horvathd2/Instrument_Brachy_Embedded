add_executable(main
        main.c
        )

pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/encoder.pio)

add_library(motor_control motor_control.c)
add_library(micro_servo micro_servo.c)

target_include_directories(main PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )

target_link_libraries(motor_control PRIVATE
        pico_stdlib
        pico_time
        pico_multicore
        hardware_pio
        hardware_pwm
) 

target_link_libraries(micro_servo PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_pwm
) 

target_link_libraries(main
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_pwm
        motor_control
        )

pico_add_extra_outputs(main)
pico_enable_stdio_usb(main 1)