/*
 * OMAP TWL4030 power scripts header file
 *
 * Author: Lesly A M <leslyam@ti.com>
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Lesly A M <leslyam@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP3_TWL4030_SCRIPT_H
#define __ARCH_ARM_MACH_OMAP3_TWL4030_SCRIPT_H

#ifdef CONFIG_TWL4030_POWER
extern struct twl4030_power_data twl4030_generic_script;
#else
#define twl4030_generic_script	NULL;
#endif

#endif
