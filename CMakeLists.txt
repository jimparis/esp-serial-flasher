if(CONFIG_ESP_SERIAL_FLASHER)

add_library(ESP_SERIAL_FLASHER INTERFACE)

target_include_directories(ESP_SERIAL_FLASHER INTERFACE include)

zephyr_library()
zephyr_library_sources(
  src/esp_loader.c
  src/serial_comm.c
  src/md5_hash.c
  port/zephyr_port.c
  )
zephyr_library_include_directories(
  private_include
  )

zephyr_library_link_libraries(ESP_SERIAL_FLASHER)
target_link_libraries(ESP_SERIAL_FLASHER INTERFACE zephyr_interface)

endif()
