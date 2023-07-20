#
# Copyright (c) 2023 Christian Spielberger
#

zephyr_library()

zephyr_include_directories(
        ${ZEPHYR_CURRENT_MODULE_DIR}/include
)

zephyr_library_sources(
  src/wm8960.c
)
