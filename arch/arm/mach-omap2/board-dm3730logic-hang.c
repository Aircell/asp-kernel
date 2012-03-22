/*
 * linux/arch/arm/mach-omap2/board-dm3730logic-hang.c
 *
 * Copyright (C) 2011 Logic Product Development
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include "board-dm3730logic.h"

volatile int dm3730logic_hang;

static int __init dm_hang_onstart(char *str)
{
	dm3730logic_hang = 1;
	return 1;
}

__setup("hang", dm_hang_onstart);
