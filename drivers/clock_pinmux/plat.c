/*
 * Copyright 2020 Broadcom
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/barrier.h>
#include <string.h>
#include <al_hwcfg.h>
#include <al_type.h>
#include <al_reg_io.h>

extern void board_init();

// static int plat_init(void)
// {
// 	board_init();

// 	return 0;
// }
// SYS_INIT(plat_init, PRE_KERNEL_1,
// 	 CONFIG_PLAT_INIT_PRIORITY);

extern void z_arm64_el_highest_plat_init(void);

#define TSG_CTRL_EN_CNT_BIT_POS  (0)

AL_VOID Altop_Syscnts_CounterCtrl(AL_FUNCTION CntStatus)
{
    AL_REG32_SET_BIT(TOP_SYSCNT_S_BASE_ADDR, TSG_CTRL_EN_CNT_BIT_POS, CntStatus);
}

void z_arm64_el_highest_plat_init(void)
{
	board_init();
	Altop_Syscnts_CounterCtrl(AL_FUNC_ENABLE);
}
