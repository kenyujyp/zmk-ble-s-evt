# SPDX-License-Identifier: MIT

board_runner_args(jlink "--device=nRF52840_xxAA" "--speed=4000")
include(${ZEPHYR_BASE}/boards/common/uf2.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)


zephyr_library_sources(${CMAKE_CURRENT_SOURCE_DIR}/board.c)