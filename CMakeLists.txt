if(CONFIG_ESP_SERIAL_FLASHER)

  zephyr_interface_library_named(esp_serial_flasher)

  target_include_directories(esp_serial_flasher INTERFACE include)

  zephyr_library_named(modules_esp_serial_flasher)

  zephyr_include_directories(
    include
    )

  zephyr_library_include_directories(
    private_include
    )

  zephyr_library_sources(
    src/esp_loader.c
    src/serial_comm.c
    src/md5_hash.c
    port/zephyr_port.c
    )

  zephyr_library_link_libraries(esp_serial_flasher)
  target_link_libraries(esp_serial_flasher INTERFACE zephyr_interface)

  # This symbol seems too generic to define globally with
  # `zephyr_compile_definitions`, so users will be required to define this
  # as well when including esp_loader.h
  target_compile_definitions(esp_serial_flasher INTERFACE "MD5_ENABLED")

endif()
