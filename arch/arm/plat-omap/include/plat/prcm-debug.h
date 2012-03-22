/*
 * arch/arm/plat-omap/include/mach/prcm.h
 *
 * Access definitions for use in OMAP34XX PM suspend entry debug
 *
 * Copyright (C) 2010 Logic Product Development, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifdef CONFIG_DEBUG_SUSPEND_ENTRY
/* Table of entries held in SRAM */
struct omap_pm_debug_sram_register {
	unsigned int *v_ptr;	/* virtual address of value to read */
	unsigned int value;	/* value of *v_ptr */
};

struct omap_pm_debug_sram_registers {
	int valid;	/* valid if save_debug_suspend_regs in sleep34xx.S called */
	struct omap_pm_debug_sram_register regs[];
};

struct omap_pm_debug_register_names {
	int have_expected_value;	/* Is Expected value set? */
	unsigned int expected_value;	/* expeted value */
	char *name;			/* name of register */
	unsigned int *v_ptr;		/* virtual address of register */
};

/* Structure in SRAM that captures PM register before WFI instruction */
extern struct omap_pm_debug_sram_registers *_omap34xx_sram_dbg_suspend_struct;

/* Function to copy the omap_pm_debug_registers structure into SRAM and
 * initialize *ptr to its address (making it accessible to
 * omap34xx_cpu_suspend code in SRAM). */
extern void pm_debug_suspend_entry_init(void **ptr);

#endif
