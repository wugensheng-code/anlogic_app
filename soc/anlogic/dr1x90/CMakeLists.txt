#
# Copyright 2022 Huawei France Technologies SASU
#
# SPDX-License-Identifier: Apache-2.0
#

add_subdirectory(${SOC_SERIES})

zephyr_include_directories(.)
zephyr_include_directories(${SOC_SERIES})

zephyr_include_directories(${SOC_SERIES}/include)
