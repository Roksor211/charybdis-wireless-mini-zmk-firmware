if(CONFIG_SHIELD_CHARYBDIS_RIGHT)
  if(CONFIG_INPUT_PMW3610)
    message(FATAL_ERROR
      "Set CONFIG_INPUT_PMW3610=n in config/charybdis_right.conf")
  endif()

  zephyr_library()
  zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/input)
  zephyr_library_sources(src/pmw3610_rotated.c)

  # Diagnostic: reduce split scanning while retaining the host connection.
  if(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    set(split_source
      "${APPLICATION_SOURCE_DIR}/src/split/bluetooth/central.c")

    if(NOT EXISTS "${split_source}")
      message(FATAL_ERROR "ZMK split source not found: ${split_source}")
    endif()

    file(READ "${split_source}" split_content)

    set(scan_marker "Charybdis split scan: window=30ms interval=300ms")
    string(FIND "${split_content}" "${scan_marker}" marker_position)

    if(marker_position EQUAL -1)
      set(old_scan
        "int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, split_central_device_found);")

      set(new_scan [=[
LOG_INF("Charybdis split scan: window=30ms interval=300ms");
    int err = bt_le_scan_start(
        BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_PASSIVE,
                         BT_LE_SCAN_OPT_FILTER_DUPLICATE, 480, 48),
        split_central_device_found);
]=])

      string(FIND "${split_content}" "${old_scan}" scan_position)
      if(scan_position EQUAL -1)
        message(FATAL_ERROR
          "Expected ZMK scan call not found; review the scan patch")
      endif()

      string(REPLACE "${old_scan}" "${new_scan}"
        split_content "${split_content}")
      file(WRITE "${split_source}" "${split_content}")
    endif()

    message(STATUS "Charybdis split scan configured: 30ms / 300ms")
  endif()
endif()