# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# sets the board name to our custom board
set(BOARD awaresite_nrf52840)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(awaresite)

# adds our include directory to the app so that it can find the header files
target_include_directories(app PRIVATE include)

# adds the source files to our app
target_sources(app PRIVATE src/main.c)
