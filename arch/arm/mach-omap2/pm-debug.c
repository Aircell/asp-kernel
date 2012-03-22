/*
 * OMAP Power Management debug routines
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Copyright (C) 2006-2008 Nokia Corporation
 *
 * Written by:
 * Richard Woodruff <r-woodruff2@ti.com>
 * Tony Lindgren
 * Juha Yrjola
 * Amit Kucheria <amit.kucheria@nokia.com>
 * Igor Stoppa <igor.stoppa@nokia.com>
 * Jouni Hogander
 *
 * Based on pm.c for omap2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>

#include <plat/clock.h>
#include <plat/board.h>
#include <plat/powerdomain.h>
#include <plat/clockdomain.h>
#include <plat/prcm-debug.h>
#include <plat/sram.h>

#include "prm.h"
#include "cm.h"
#include "pm.h"
#include "prm-regbits-34xx.h"

int omap2_pm_debug;

#define DUMP_PRM_MOD_REG(mod, reg)    \
	regs[reg_count].name = #mod "." #reg; \
	regs[reg_count++].val = prm_read_mod_reg(mod, reg)
#define DUMP_CM_MOD_REG(mod, reg)     \
	regs[reg_count].name = #mod "." #reg; \
	regs[reg_count++].val = cm_read_mod_reg(mod, reg)
#define DUMP_PRM_REG(reg) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = __raw_readl(reg)
#define DUMP_CM_REG(reg) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = __raw_readl(reg)
#define DUMP_INTC_REG(reg, off) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = \
			 __raw_readl(OMAP2_L4_IO_ADDRESS(0x480fe000 + (off)))

#ifdef CONFIG_DEBUG_FS
static int __init pm_dbg_init(void);
#endif

void omap2_pm_dump(int mode, int resume, unsigned int us)
{
	struct reg {
		const char *name;
		u32 val;
	} regs[32];
	int reg_count = 0, i;
	const char *s1 = NULL, *s2 = NULL;

	if (!resume) {
#if 0
		/* MPU */
		DUMP_PRM_MOD_REG(OCP_MOD, OMAP2_PRM_IRQENABLE_MPU_OFFSET);
		DUMP_CM_MOD_REG(MPU_MOD, CM_CLKSTCTRL);
		DUMP_PRM_MOD_REG(MPU_MOD, PM_PWSTCTRL);
		DUMP_PRM_MOD_REG(MPU_MOD, PM_PWSTST);
		DUMP_PRM_MOD_REG(MPU_MOD, PM_WKDEP);
#endif
#if 0
		/* INTC */
		DUMP_INTC_REG(INTC_MIR0, 0x0084);
		DUMP_INTC_REG(INTC_MIR1, 0x00a4);
		DUMP_INTC_REG(INTC_MIR2, 0x00c4);
#endif
#if 0
		DUMP_CM_MOD_REG(CORE_MOD, CM_FCLKEN1);
		if (cpu_is_omap24xx()) {
			DUMP_CM_MOD_REG(CORE_MOD, OMAP24XX_CM_FCLKEN2);
			DUMP_PRM_MOD_REG(OMAP24XX_GR_MOD,
					OMAP2_PRCM_CLKEMUL_CTRL_OFFSET);
			DUMP_PRM_MOD_REG(OMAP24XX_GR_MOD,
					OMAP2_PRCM_CLKSRC_CTRL_OFFSET);
		}
		DUMP_CM_MOD_REG(WKUP_MOD, CM_FCLKEN);
		DUMP_CM_MOD_REG(CORE_MOD, CM_ICLKEN1);
		DUMP_CM_MOD_REG(CORE_MOD, CM_ICLKEN2);
		DUMP_CM_MOD_REG(WKUP_MOD, CM_ICLKEN);
		DUMP_CM_MOD_REG(PLL_MOD, CM_CLKEN);
		DUMP_CM_MOD_REG(PLL_MOD, CM_AUTOIDLE);
		DUMP_PRM_MOD_REG(CORE_MOD, PM_PWSTST);
#endif
#if 0
		/* DSP */
		if (cpu_is_omap24xx()) {
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_FCLKEN);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_ICLKEN);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_IDLEST);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_AUTOIDLE);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_CLKSEL);
			DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_CLKSTCTRL);
			DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, RM_RSTCTRL);
			DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, RM_RSTST);
			DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, PM_PWSTCTRL);
			DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, PM_PWSTST);
		}
#endif
	} else {
		DUMP_PRM_MOD_REG(CORE_MOD, PM_WKST1);
		if (cpu_is_omap24xx())
			DUMP_PRM_MOD_REG(CORE_MOD, OMAP24XX_PM_WKST2);
		DUMP_PRM_MOD_REG(WKUP_MOD, PM_WKST);
		DUMP_PRM_MOD_REG(OCP_MOD, OMAP2_PRCM_IRQSTATUS_MPU_OFFSET);
#if 1
		DUMP_INTC_REG(INTC_PENDING_IRQ0, 0x0098);
		DUMP_INTC_REG(INTC_PENDING_IRQ1, 0x00b8);
		DUMP_INTC_REG(INTC_PENDING_IRQ2, 0x00d8);
#endif
	}

	switch (mode) {
	case 0:
		s1 = "full";
		s2 = "retention";
		break;
	case 1:
		s1 = "MPU";
		s2 = "retention";
		break;
	case 2:
		s1 = "MPU";
		s2 = "idle";
		break;
	}

	if (!resume)
#ifdef CONFIG_NO_HZ
		printk(KERN_INFO
		       "--- Going to %s %s (next timer after %u ms)\n", s1, s2,
		       jiffies_to_msecs(get_next_timer_interrupt(jiffies) -
					jiffies));
#else
		printk(KERN_INFO "--- Going to %s %s\n", s1, s2);
#endif
	else
		printk(KERN_INFO "--- Woke up (slept for %u.%03u ms)\n",
			us / 1000, us % 1000);

	for (i = 0; i < reg_count; i++)
		printk(KERN_INFO "%-20s: 0x%08x\n", regs[i].name, regs[i].val);
}

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <asm/cacheflush.h>

static void pm_dbg_regset_store(u32 *ptr);

struct dentry *pm_dbg_dir;

static int pm_dbg_init_done;

enum {
	DEBUG_FILE_COUNTERS = 0,
	DEBUG_FILE_TIMERS,
};

struct pm_module_def {
	char *name; /* Name of the module */
	short type; /* CM or PRM */
	short offset;
	unsigned char low; /* First register address on this module */
	unsigned char high; /* Last register address on this module */
};

#define MOD_CM 0
#define MOD_PRM 1

static const struct pm_module_def *pm_dbg_reg_modules;
static const struct pm_module_def omap3_pm_reg_modules[] = {
	{ "IVA2", MOD_CM, OMAP3430_IVA2_MOD, 0x00, 0x04 },
	{ "IVA2", MOD_CM, OMAP3430_IVA2_MOD, 0x20, 0x24 },
	{ "IVA2", MOD_CM, OMAP3430_IVA2_MOD, 0x34, 0x34 },
	{ "IVA2", MOD_CM, OMAP3430_IVA2_MOD, 0x40, 0x4c },
	{ "OCP", MOD_CM, OCP_MOD, 0x00, 0x00 },
	{ "OCP", MOD_CM, OCP_MOD, 0x10, 0x10 },
	{ "MPU", MOD_CM, MPU_MOD, 0x04, 0x04 },
	{ "MPU", MOD_CM, MPU_MOD, 0x20, 0x24 },
	{ "MPU", MOD_CM, MPU_MOD, 0x34, 0x34 },
	{ "MPU", MOD_CM, MPU_MOD, 0x40, 0x4c },
	{ "CORE", MOD_CM, CORE_MOD, 0x08, 0x08 },
	{ "CORE", MOD_CM, CORE_MOD, 0x10, 0x10 },
#if 0
	{ "CORE", MOD_CM, CORE_MOD, 0x14, 0x14 }, /* reservered for non-GP */
#endif
	{ "CORE", MOD_CM, CORE_MOD, 0x18, 0x18 },
	{ "CORE", MOD_CM, CORE_MOD, 0x20, 0x20 },
#if 0
	{ "CORE", MOD_CM, CORE_MOD, 0x24, 0x24 }, /* reservered for non-GP */
#endif
	{ "CORE", MOD_CM, CORE_MOD, 0x28, 0x28 },
	{ "CORE", MOD_CM, CORE_MOD, 0x30, 0x30 },
#if 0
	{ "CORE", MOD_CM, CORE_MOD, 0x34, 0x34 }, /* reservered for non-GP */
#endif
	{ "CORE", MOD_CM, CORE_MOD, 0x38, 0x38 },
	{ "CORE", MOD_CM, CORE_MOD, 0x40, 0x40 },
	{ "CORE", MOD_CM, CORE_MOD, 0x48, 0x4c },
	{ "SGX", MOD_CM, OMAP3430ES2_SGX_MOD, 0x00, 0x00 },
	{ "SGX", MOD_CM, OMAP3430ES2_SGX_MOD, 0x10, 0x10 },
	{ "SGX", MOD_CM, OMAP3430ES2_SGX_MOD, 0x20, 0x20 },
	{ "SGX", MOD_CM, OMAP3430ES2_SGX_MOD, 0x40, 0x4c },
	{ "WKUP", MOD_CM, WKUP_MOD, 0x00, 0x00 },
	{ "WKUP", MOD_CM, WKUP_MOD, 0x10, 0x10 },
	{ "WKUP", MOD_CM, WKUP_MOD, 0x20, 0x20 },
	{ "WKUP", MOD_CM, WKUP_MOD, 0x30, 0x30 },
	{ "WKUP", MOD_CM, WKUP_MOD, 0x40, 0x40 },
	{ "CCR", MOD_CM, PLL_MOD, 0x00, 0x04 },
	{ "CCR", MOD_CM, PLL_MOD, 0x20, 0x24 },
	{ "CCR", MOD_CM, PLL_MOD, 0x30, 0x34 },
	{ "CCR", MOD_CM, PLL_MOD, 0x40, 0x50 },
	{ "DSS", MOD_CM, OMAP3430_DSS_MOD, 0x00, 0x00 },
	{ "DSS", MOD_CM, OMAP3430_DSS_MOD, 0x10, 0x10 },
	{ "DSS", MOD_CM, OMAP3430_DSS_MOD, 0x20, 0x20 },
	{ "DSS", MOD_CM, OMAP3430_DSS_MOD, 0x30, 0x30 },
	{ "DSS", MOD_CM, OMAP3430_DSS_MOD, 0x40, 0x4c },
	{ "CAM", MOD_CM, OMAP3430_CAM_MOD, 0x00, 0x00 },
	{ "CAM", MOD_CM, OMAP3430_CAM_MOD, 0x10, 0x10 },
	{ "CAM", MOD_CM, OMAP3430_CAM_MOD, 0x20, 0x20 },
	{ "CAM", MOD_CM, OMAP3430_CAM_MOD, 0x30, 0x30 },
	{ "CAM", MOD_CM, OMAP3430_CAM_MOD, 0x40, 0x4c },
	{ "PER", MOD_CM, OMAP3430_PER_MOD, 0x00, 0x00 },
	{ "PER", MOD_CM, OMAP3430_PER_MOD, 0x10, 0x10 },
	{ "PER", MOD_CM, OMAP3430_PER_MOD, 0x20, 0x20 },
	{ "PER", MOD_CM, OMAP3430_PER_MOD, 0x30, 0x30 },
	{ "PER", MOD_CM, OMAP3430_PER_MOD, 0x40, 0x4c },
	{ "EMU", MOD_CM, OMAP3430_EMU_MOD, 0x40, 0x40 },
	{ "EMU", MOD_CM, OMAP3430_EMU_MOD, 0x48, 0x54 },
	{ "NEON", MOD_CM, OMAP3430_NEON_MOD, 0x20, 0x20 },
	{ "NEON", MOD_CM, OMAP3430_NEON_MOD, 0x48, 0x48 },
	{ "USB", MOD_CM, OMAP3430ES2_USBHOST_MOD, 0x00, 0x00 },
	{ "USB", MOD_CM, OMAP3430ES2_USBHOST_MOD, 0x10, 0x10 },
	{ "USB", MOD_CM, OMAP3430ES2_USBHOST_MOD, 0x20, 0x20 },
	{ "USB", MOD_CM, OMAP3430ES2_USBHOST_MOD, 0x30, 0x30 },
	{ "USB", MOD_CM, OMAP3430ES2_USBHOST_MOD, 0x40, 0x4c },
	{ "IVA2", MOD_PRM, OMAP3430_IVA2_MOD, 0x50, 0x50 },
	{ "IVA2", MOD_PRM, OMAP3430_IVA2_MOD, 0x58, 0x58 },
	{ "IVA2", MOD_PRM, OMAP3430_IVA2_MOD, 0xc8, 0xc8 },
	{ "IVA2", MOD_PRM, OMAP3430_IVA2_MOD, 0xe0, 0xe8 },
	{ "IVA2", MOD_PRM, OMAP3430_IVA2_MOD, 0xf8, 0xfc },
	{ "OCP", MOD_PRM, OCP_MOD, 0x04, 0x4 },
	{ "OCP", MOD_PRM, OCP_MOD, 0x14, 0x1c },
	{ "MPU", MOD_PRM, MPU_MOD, 0x58, 0x58 },
	{ "MPU", MOD_PRM, MPU_MOD, 0xc8, 0xc8 },
	{ "MPU", MOD_PRM, MPU_MOD, 0xd4, 0xe8 },
	{ "CORE", MOD_PRM, CORE_MOD, 0x58, 0x58 },
	{ "CORE", MOD_PRM, CORE_MOD, 0xa0, 0xa8 },
	{ "CORE", MOD_PRM, CORE_MOD, 0xb0, 0xb0 },
	{ "CORE", MOD_PRM, CORE_MOD, 0xb8, 0xb8 },
	{ "CORE", MOD_PRM, CORE_MOD, 0xe0, 0xe8 },
	{ "CORE", MOD_PRM, CORE_MOD, 0xf0, 0xf8 },
	{ "SGX", MOD_PRM, OMAP3430ES2_SGX_MOD, 0x58, 0x58 },
	{ "SGX", MOD_PRM, OMAP3430ES2_SGX_MOD, 0xc8, 0xc8 },
	{ "SGX", MOD_PRM, OMAP3430ES2_SGX_MOD, 0xe0, 0xe8 },
	{ "WKUP", MOD_PRM, WKUP_MOD, 0xa0, 0xa8 },
	{ "WKUP", MOD_PRM, WKUP_MOD, 0xb0, 0xb0 },
	{ "CCR", MOD_PRM, PLL_MOD, 0x40, 0x40 },
	{ "CCR", MOD_PRM, PLL_MOD, 0x70, 0x70 },
	{ "DSS", MOD_PRM, OMAP3430_DSS_MOD, 0x58, 0x58 },
	{ "DSS", MOD_PRM, OMAP3430_DSS_MOD, 0xa0, 0xa0 },
	{ "DSS", MOD_PRM, OMAP3430_DSS_MOD, 0xc8, 0xc8 },
	{ "DSS", MOD_PRM, OMAP3430_DSS_MOD, 0xe0, 0xe8 },
	{ "CAM", MOD_PRM, OMAP3430_CAM_MOD, 0x58, 0xe8 },
	{ "PER", MOD_PRM, OMAP3430_PER_MOD, 0x58, 0x58 },
	{ "PER", MOD_PRM, OMAP3430_PER_MOD, 0xa0, 0xa8 },
	{ "PER", MOD_PRM, OMAP3430_PER_MOD, 0xb0, 0xb0 },
	{ "PER", MOD_PRM, OMAP3430_PER_MOD, 0xc8, 0xc8 },
	{ "PER", MOD_PRM, OMAP3430_PER_MOD, 0xe0, 0xe8 },
	{ "EMU", MOD_PRM, OMAP3430_EMU_MOD, 0x58, 0x58 },
	{ "EMU", MOD_PRM, OMAP3430_EMU_MOD, 0xe4, 0xe4 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0x20, 0x30 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0x34, 0x3c },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0x50, 0x58 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0x60, 0x60 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0x64, 0x64 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0x70, 0x70 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0x80, 0x80 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0x90, 0xa0 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0xb0, 0xb0 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0xb4, 0xc0 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0xc4, 0xc4 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0xd0, 0xe4 },
	{ "GLBL", MOD_PRM, OMAP3430_GR_MOD, 0xf0, 0xf4 },
	{ "NEON", MOD_PRM, OMAP3430_NEON_MOD, 0x58, 0x58 },
	{ "NEON", MOD_PRM, OMAP3430_NEON_MOD, 0xc8, 0xc8 },
	{ "NEON", MOD_PRM, OMAP3430_NEON_MOD, 0xe0, 0xe8 },
	{ "USB", MOD_PRM, OMAP3430ES2_USBHOST_MOD, 0x58, 0x58 },
	{ "USB", MOD_PRM, OMAP3430ES2_USBHOST_MOD, 0xa0, 0xa8 },
	{ "USB", MOD_PRM, OMAP3430ES2_USBHOST_MOD, 0xb0, 0xb0 },
	{ "USB", MOD_PRM, OMAP3430ES2_USBHOST_MOD, 0xc8, 0xc8 },
	{ "USB", MOD_PRM, OMAP3430ES2_USBHOST_MOD, 0xe0, 0xe8 },
	{ NULL, 0, 0, 0, 0 },
};

#define PM_DBG_MAX_REG_SETS 4

static void *pm_dbg_reg_set[PM_DBG_MAX_REG_SETS];

static int pm_dbg_get_regset_size(void)
{
	static int regset_size;

	if (regset_size == 0) {
		int i = 0;

		while (pm_dbg_reg_modules[i].name) {
			regset_size += pm_dbg_reg_modules[i].high +
				4 - pm_dbg_reg_modules[i].low;
			i++;
		}
	}
	return regset_size;
}

static int pm_dbg_show_regs(struct seq_file *s, void *unused)
{
	int i, j;
	unsigned long val;
	int reg_set = (int)s->private;
	u32 *ptr;
	void *store = NULL;
	int regs = 0;
	int linefeed = 0;
	short type = -1;
	short offset = -1;

	if (reg_set == 0) {
		store = kmalloc(pm_dbg_get_regset_size(), GFP_KERNEL);
		ptr = store;
		pm_dbg_regset_store(ptr);
	} else {
		ptr = pm_dbg_reg_set[reg_set - 1];
	}

	i = 0;

	while (pm_dbg_reg_modules[i].name) {
		if (type != pm_dbg_reg_modules[i].type
			|| offset != pm_dbg_reg_modules[i].offset) {
			if (regs) {
				seq_printf(s, "\n");
				regs = 0;
			}
			offset = pm_dbg_reg_modules[i].offset;
			type = pm_dbg_reg_modules[i].type;
			regs = 0;
			linefeed = 0;
			if (pm_dbg_reg_modules[i].type == MOD_CM)
				seq_printf(s, "MOD: CM_%s (%08x)\n",
					pm_dbg_reg_modules[i].name,
					(u32)(OMAP3430_CM_BASE +
						offset));
			else
				seq_printf(s, "MOD: PRM_%s (%08x)\n",
					pm_dbg_reg_modules[i].name,
					(u32)(OMAP3430_PRM_BASE +
						offset));
		}

		for (j = pm_dbg_reg_modules[i].low;
			j <= pm_dbg_reg_modules[i].high; j += 4) {
			val = *(ptr++);
			if (val != 0) {
				if (linefeed) {
					seq_printf(s, "\n");
					linefeed = 0;
					regs = 0;
				}
				seq_printf(s, "  %02x => %08lx", j, val);
				if (++regs % 4 == 0)
					linefeed = 1;
			}
		}
		i++;
	}

	if (regs)
		seq_printf(s, "\n");

	if (store != NULL)
		kfree(store);

	return 0;
}

static void pm_dbg_regset_store(u32 *ptr)
{
	int i, j;
	u32 val;

	i = 0;

	while (pm_dbg_reg_modules[i].name != 0) {
		for (j = pm_dbg_reg_modules[i].low;
			j <= pm_dbg_reg_modules[i].high; j += 4) {
			if (pm_dbg_reg_modules[i].type == MOD_CM)
				val = cm_read_mod_reg(
					pm_dbg_reg_modules[i].offset, j);
			else
				val = prm_read_mod_reg(
					pm_dbg_reg_modules[i].offset, j);
			*(ptr++) = val;
		}
		i++;
	}
}

int pm_dbg_regset_save(int reg_set)
{
	if (pm_dbg_reg_set[reg_set-1] == NULL)
		return -EINVAL;

	pm_dbg_regset_store(pm_dbg_reg_set[reg_set-1]);

	return 0;
}

static const char pwrdm_state_names[][PWRDM_MAX_PWRSTS] = {
	"OFF",
	"RET",
	"INA",
	"ON"
};

void pm_dbg_update_time(struct powerdomain *pwrdm, int prev)
{
	s64 t;

	if (!pm_dbg_init_done)
		return ;

	/* Update timer for previous state */
	t = sched_clock();

	pwrdm->state_timer[prev] += t - pwrdm->timer;

	pwrdm->timer = t;
}

static int clkdm_dbg_show_counter(struct clockdomain *clkdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;

	if (strcmp(clkdm->name, "emu_clkdm") == 0 ||
		strcmp(clkdm->name, "wkup_clkdm") == 0 ||
		strncmp(clkdm->name, "dpll", 4) == 0)
		return 0;

	seq_printf(s, "%s->%s (%d)", clkdm->name,
			clkdm->pwrdm.ptr->name,
			atomic_read(&clkdm->usecount));
	seq_printf(s, "\n");

	return 0;
}

static int pwrdm_dbg_show_counter(struct powerdomain *pwrdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;
	int i;

	if (strcmp(pwrdm->name, "emu_pwrdm") == 0 ||
		strcmp(pwrdm->name, "wkup_pwrdm") == 0 ||
		strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	if (pwrdm->state != pwrdm_read_pwrst(pwrdm))
		printk(KERN_ERR "pwrdm state mismatch(%s) %d != %d\n",
			pwrdm->name, pwrdm->state, pwrdm_read_pwrst(pwrdm));

	seq_printf(s, "%s (%s)", pwrdm->name,
			pwrdm_state_names[pwrdm->state]);
	for (i = 0; i < PWRDM_MAX_PWRSTS; i++)
		seq_printf(s, ",%s:%d", pwrdm_state_names[i],
			pwrdm->state_counter[i]);

	seq_printf(s, "\n");

	return 0;
}

static int pwrdm_dbg_show_timer(struct powerdomain *pwrdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;
	int i;

	if (strcmp(pwrdm->name, "emu_pwrdm") == 0 ||
		strcmp(pwrdm->name, "wkup_pwrdm") == 0 ||
		strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	pwrdm_state_switch(pwrdm);

	seq_printf(s, "%s (%s)", pwrdm->name,
		pwrdm_state_names[pwrdm->state]);

	for (i = 0; i < 4; i++)
		seq_printf(s, ",%s:%lld", pwrdm_state_names[i],
			pwrdm->state_timer[i]);

	seq_printf(s, "\n");
	return 0;
}

static int pm_dbg_show_counters(struct seq_file *s, void *unused)
{
	pwrdm_for_each(pwrdm_dbg_show_counter, s);
	clkdm_for_each(clkdm_dbg_show_counter, s);

	return 0;
}

static int pm_dbg_show_timers(struct seq_file *s, void *unused)
{
	pwrdm_for_each(pwrdm_dbg_show_timer, s);
	return 0;
}

static int pm_dbg_open(struct inode *inode, struct file *file)
{
	switch ((int)inode->i_private) {
	case DEBUG_FILE_COUNTERS:
		return single_open(file, pm_dbg_show_counters,
			&inode->i_private);
	case DEBUG_FILE_TIMERS:
	default:
		return single_open(file, pm_dbg_show_timers,
			&inode->i_private);
	};
}

static int pm_dbg_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, pm_dbg_show_regs, inode->i_private);
}

static const struct file_operations debug_fops = {
	.open           = pm_dbg_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static const struct file_operations debug_reg_fops = {
	.open           = pm_dbg_reg_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

int pm_dbg_regset_init(int reg_set)
{
	char name[2];

	if (!pm_dbg_init_done)
		pm_dbg_init();

	if (reg_set < 1 || reg_set > PM_DBG_MAX_REG_SETS ||
		pm_dbg_reg_set[reg_set-1] != NULL)
		return -EINVAL;

	pm_dbg_reg_set[reg_set-1] =
		kmalloc(pm_dbg_get_regset_size(), GFP_KERNEL);

	if (pm_dbg_reg_set[reg_set-1] == NULL)
		return -ENOMEM;

	if (pm_dbg_dir != NULL) {
		sprintf(name, "%d", reg_set);

		(void) debugfs_create_file(name, S_IRUGO,
			pm_dbg_dir, (void *)reg_set, &debug_reg_fops);
	}

	return 0;
}

static int pwrdm_suspend_get(void *data, u64 *val)
{
	int ret;
	ret = omap3_pm_get_suspend_state((struct powerdomain *)data);
	*val = ret;

	if (ret >= 0)
		return 0;
	return *val;
}

static int pwrdm_suspend_set(void *data, u64 val)
{
	return omap3_pm_set_suspend_state((struct powerdomain *)data, (int)val);
}

DEFINE_SIMPLE_ATTRIBUTE(pwrdm_suspend_fops, pwrdm_suspend_get,
			pwrdm_suspend_set, "%llu\n");

static int __init pwrdms_setup(struct powerdomain *pwrdm, void *dir)
{
	int i;
	s64 t;
	struct dentry *d;

	t = sched_clock();

	for (i = 0; i < 4; i++)
		pwrdm->state_timer[i] = 0;

	pwrdm->timer = t;

	if (strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	d = debugfs_create_dir(pwrdm->name, (struct dentry *)dir);

	(void) debugfs_create_file("suspend", S_IRUGO|S_IWUSR, d,
			(void *)pwrdm, &pwrdm_suspend_fops);

	return 0;
}

static int option_get(void *data, u64 *val)
{
	u32 *option = data;

	*val = *option;

	return 0;
}

static int option_set(void *data, u64 val)
{
	u32 *option = data;

	*option = val;

	if (option == &enable_off_mode)
		omap3_pm_off_mode_enable(val);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pm_dbg_option_fops, option_get, option_set, "%llu\n");

#ifdef CONFIG_DEBUG_SUSPEND_ENTRY

struct omap_pm_debug_register_names pm_debug_reg_names[] = {
	{ 1, 0x00000000, "CM_FCLKEN_IVA2",
	  OMAP34XX_CM_REGADDR(OMAP3430_IVA2_MOD, CM_FCLKEN) },
	{ 0, 0x00000000, "CM_CLKEN_PLL_IVA2",
	  OMAP34XX_CM_REGADDR(OMAP3430_IVA2_MOD, OMAP3430_CM_CLKEN_PLL) },
	{ 1, 0x00000001, "CM_IDLEST_IVA2",
	  OMAP34XX_CM_REGADDR(OMAP3430_IVA2_MOD, CM_IDLEST) },
	{ 1, 0x00000000, "CM_IDLEST_PLL_IVA2",
	  OMAP34XX_CM_REGADDR(OMAP3430_IVA2_MOD, OMAP3430_CM_IDLEST_PLL) },
	{ 0, 0x00000000, "CM_AUTOIDLE_PLL_IVA2",
	  OMAP34XX_CM_REGADDR(OMAP3430_IVA2_MOD, CM_AUTOIDLE2) },
	{ 1, 0x00000003, "CM_CLKSTCTRL_IVA2",
	  OMAP34XX_CM_REGADDR(OMAP3430_IVA2_MOD, CM_CLKSTCTRL) },
	{ 0, 0x00000000, "CM_CLKSTST_IVA2",
	  OMAP34XX_CM_REGADDR(OMAP3430_IVA2_MOD, OMAP3430_CM_CLKSTST) },

	{ 0, 0x00000000, "RM_RSTCTRL_IVA2",
	  OMAP34XX_PRM_REGADDR(OMAP3430_IVA2_MOD, OMAP3_PRM_RSTCTRL_OFFSET) },
	{ 0, 0x00000000, "RM_RSTST_IVA2",
	  OMAP34XX_PRM_REGADDR(OMAP3430_IVA2_MOD, OMAP3_PRM_RSTST_OFFSET) },
	{ 0, 0x00000000, "PM_WKDEP_IVA2",
	  OMAP34XX_PRM_REGADDR(OMAP3430_IVA2_MOD, PM_WKDEP) },
	{ 0, 0x00000000, "PM_PWSTCTRL_IVA2",
	  OMAP34XX_PRM_REGADDR(OMAP3430_IVA2_MOD, PM_PWSTCTRL) },
	{ 1, 0x00000555, "PM_PWSTST_IVA2",
	  OMAP34XX_PRM_REGADDR(OMAP3430_IVA2_MOD, PM_PWSTST) },
	{ 0, 0x00000000, "PM_PREPWSTST_IVA2",
	  OMAP34XX_PRM_REGADDR(OMAP3430_IVA2_MOD, OMAP3430_PM_PREPWSTST) },
	{ 0, 0x00000000, "PRM_IRQSTATUS_IVA2",
	  OMAP34XX_PRM_REGADDR(OMAP3430_IVA2_MOD, OMAP3430_PRM_IRQSTATUS_IVA2) },
	{ 0, 0x00000000, "PRM_IRQENABLE_IVA2",
	  OMAP34XX_PRM_REGADDR(OMAP3430_IVA2_MOD, OMAP3430_PRM_IRQENABLE_IVA2) },


	{ 1, 0x00000000, "CM_FCLKEN1_CORE",
	  OMAP34XX_CM_REGADDR(CORE_MOD, CM_FCLKEN1) },
	{ 1, 0x00000000, "CM_FCLKEN3_CORE",
	  OMAP34XX_CM_REGADDR(CORE_MOD, OMAP3430ES2_CM_FCLKEN3) },
	{ 1, 0x00000042, "CM_ICLKEN1_CORE",
	  OMAP34XX_CM_REGADDR(CORE_MOD, CM_ICLKEN1) },
	{ 0, 0x00000000, "CM_ICLKEN2_CORE",
	  OMAP34XX_CM_REGADDR(CORE_MOD, CM_ICLKEN2) },
	{ 1, 0x00000000, "CM_ICLKEN3_CORE",
	  OMAP34XX_CM_REGADDR(CORE_MOD, CM_ICLKEN3) },
	{ 1, 0xffffffbd, "CM_IDLEST1_CORE",
	  OMAP34XX_CM_REGADDR(CORE_MOD, CM_IDLEST1) },
	{ 1, 0x0000001f, "CM_IDLEST2_CORE",
	  OMAP34XX_CM_REGADDR(CORE_MOD, CM_IDLEST2) },
	{ 1, 0x0000000d, "CM_IDLEST3_CORE",
	  OMAP34XX_CM_REGADDR(CORE_MOD, CM_IDLEST3) },
	{ 1, 0xfffffed9, "CM_AUTOIDLE1_CORE",
	  OMAP34XX_CM_REGADDR(CORE_MOD, CM_AUTOIDLE1) },
	{ 0, 0x00000000, "CM_AUTOIDLE2_CORE",
	  OMAP34XX_CM_REGADDR(CORE_MOD, CM_AUTOIDLE2) },
	{ 0, 0x00000000, "CM_AUTOIDLE3_CORE",
	  OMAP34XX_CM_REGADDR(CORE_MOD, CM_AUTOIDLE3) },
	{ 0, 0x00000000, "CM_CLKSEL_CORE",
	  OMAP34XX_CM_REGADDR(CORE_MOD, CM_CLKSEL) },
	{ 1, 0x0000003f, "CM_CLKSTCTRL_CORE",
	  OMAP34XX_CM_REGADDR(CORE_MOD, CM_CLKSTCTRL) },
	{ 0, 0x00000000, "CM_CLKSTST_IVA2",
	  OMAP34XX_CM_REGADDR(CORE_MOD, OMAP3430_CM_CLKSTST) },

	{ 0, 0x00000000, "RM_RSTST_CORE",
	  OMAP34XX_PRM_REGADDR(CORE_MOD, OMAP3_PRM_RSTST_OFFSET) },
	{ 0, 0x00000000, "PM_WKEN1_CORE",
	  OMAP34XX_PRM_REGADDR(CORE_MOD, PM_WKEN1) },
	{ 0, 0x00000000, "PM_MPUGRPSEL1_CORE",
	  OMAP34XX_PRM_REGADDR(CORE_MOD, OMAP3430_PM_MPUGRPSEL) },
	{ 0, 0x00000000, "PM_IVA2GRPSEL1_CORE",
	  OMAP34XX_PRM_REGADDR(CORE_MOD, OMAP3430_PM_IVAGRPSEL) },
	{ 1, 0x00000000, "PM_WKST1_CORE",
	  OMAP34XX_PRM_REGADDR(CORE_MOD, PM_WKST1) },
	{ 1, 0x00000000, "PM_WKST3_CORE",
	  OMAP34XX_PRM_REGADDR(CORE_MOD, PM_WKST3) },
	{ 0, 0x00000000, "PM_PWSTCTRL_CORE",
	  OMAP34XX_PRM_REGADDR(CORE_MOD, PM_PWSTCTRL) },
	{ 0, 0x00000000, "PM_PWSTST_CORE",
	  OMAP34XX_PRM_REGADDR(CORE_MOD, PM_PWSTST) },
	{ 0, 0x00000000, "PM_PREPWSTST_CORE",
	  OMAP34XX_PRM_REGADDR(CORE_MOD, OMAP3430_PM_PREPWSTST) },
	{ 0, 0x00000000, "PM_WKEN3_CORE",
	  OMAP34XX_PRM_REGADDR(CORE_MOD, OMAP3430ES2_PM_WKEN3) },
	{ 0, 0x00000000, "PM_IVA2GRPSEL3_CORE",
	  OMAP34XX_PRM_REGADDR(CORE_MOD, OMAP3430ES2_PM_IVAGRPSEL3) },
	{ 0, 0x00000000, "PM_MPUGRPSEL3_CORE",
	  OMAP34XX_PRM_REGADDR(CORE_MOD, OMAP3430ES2_PM_MPUGRPSEL3) },


	{ 1, 0x00000000, "PM_PWSTST_SGX",
	  OMAP34XX_PRM_REGADDR(OMAP3430ES2_SGX_MOD, PM_PWSTST) },
	{ 1, 0x00000001, "PM_PWSTST_DSS",
	  OMAP34XX_PRM_REGADDR(OMAP3430_DSS_MOD, PM_PWSTST) },
	{ 1, 0x00000001, "PM_PWSTST_CAM",
	  OMAP34XX_PRM_REGADDR(OMAP3430_CAM_MOD, PM_PWSTST) },
	{ 1, 0x00000007, "PM_PWSTST_PER",
	  OMAP34XX_PRM_REGADDR(OMAP3430_PER_MOD, PM_PWSTST) },
	{ 1, 0x00000003, "PM_PWSTST_NEON",
	  OMAP34XX_PRM_REGADDR(OMAP3430_NEON_MOD, PM_PWSTST) },
	{ 1, 0x00000001, "PM_PWSTST_USBHOST",
	  OMAP34XX_PRM_REGADDR(OMAP3430ES2_USBHOST_MOD, PM_PWSTST) },

	{ 1, 0x00000001, "CM_IDLEST_SGX",
	  OMAP34XX_CM_REGADDR(OMAP3430ES2_SGX_MOD, CM_IDLEST) },
	{ 1, 0x00000001, "CM_IDLEST_CKGEN",
	  OMAP34XX_CM_REGADDR(PLL_MOD, CM_IDLEST) },
	{ 1, 0x00000000, "CM_IDLEST2_CKGEN",
	  OMAP34XX_CM_REGADDR(PLL_MOD, CM_IDLEST2) },
	{ 1, 0x00000003, "CM_IDLEST_DSS",
	  OMAP34XX_CM_REGADDR(OMAP3430_DSS_MOD, CM_IDLEST) },
	{ 1, 0x00000001, "CM_IDLEST_CAM",
	  OMAP34XX_CM_REGADDR(OMAP3430_CAM_MOD, CM_IDLEST) },
	{ 1, 0x00041fff, "CM_IDLEST_PER",
	  OMAP34XX_CM_REGADDR(OMAP3430_PER_MOD, CM_IDLEST) },
	{ 1, 0x00000000, "CM_IDLEST_NEON",
	  OMAP34XX_CM_REGADDR(OMAP3430_NEON_MOD, CM_IDLEST) },
	{ 1, 0x00000003, "CM_IDLEST_USBHOST",
	  OMAP34XX_CM_REGADDR(OMAP3430ES2_USBHOST_MOD, CM_IDLEST) },
	{ 0, 0x00000000, "PM_WKEN_WKUP",
	  OMAP34XX_PRM_REGADDR(WKUP_MOD, PM_WKEN) },
	{ 1, 0x00010000, "PM_WKST_WKUP",
	  OMAP34XX_PRM_REGADDR(WKUP_MOD, PM_WKST) },
	{ 1, 0x00000000, "PM_WKST_PER",
	  OMAP34XX_PRM_REGADDR(OMAP3430_PER_MOD, PM_WKST) },
	{ 1, 0x00000000, "PM_WKST_USBHOST",
	  OMAP34XX_PRM_REGADDR(OMAP3430ES2_USBHOST_MOD, PM_WKST) },

	{ 1, 0x00000000, "CM_FCLKEN_SGX",
	  OMAP34XX_CM_REGADDR(OMAP3430ES2_SGX_MOD, CM_FCLKEN) },
	{ 1, 0x00000000, "CM_FCLKEN_DSS",
	  OMAP34XX_CM_REGADDR(OMAP3430_DSS_MOD, CM_FCLKEN) },
	{ 1, 0x00000000, "CM_FCLKEN_CAM",
	  OMAP34XX_CM_REGADDR(OMAP3430_CAM_MOD, CM_FCLKEN) },
	{ 1, 0x00000000, "CM_FCLKEN_PER",
	  OMAP34XX_CM_REGADDR(OMAP3430_PER_MOD, CM_FCLKEN) },
	{ 1, 0x00000000, "CM_FCLKEN_USBHOST",
	  OMAP34XX_CM_REGADDR(OMAP3430ES2_USBHOST_MOD, CM_FCLKEN) },
	{ 1, 0x00000000, "CM_ICLKEN_SGX",
	  OMAP34XX_CM_REGADDR(OMAP3430ES2_SGX_MOD, CM_ICLKEN) },
	{ 1, 0x00000000, "CM_ICLKEN_DSS",
	  OMAP34XX_CM_REGADDR(OMAP3430_DSS_MOD, CM_ICLKEN) },
	{ 1, 0x00000001, "CM_AUTOIDLE_DSS",
	  OMAP34XX_CM_REGADDR(OMAP3430_DSS_MOD, CM_AUTOIDLE) },
	{ 1, 0x00000000, "CM_ICLKEN_CAM",
	  OMAP34XX_CM_REGADDR(OMAP3430_CAM_MOD, CM_ICLKEN) },
	{ 1, 0x00000001, "CM_AUTOIDLE_CAM",
	  OMAP34XX_CM_REGADDR(OMAP3430_CAM_MOD, CM_AUTOIDLE) },
	{ 1, 0x0003e000, "CM_ICLKEN_PER",
	  OMAP34XX_CM_REGADDR(OMAP3430_PER_MOD, CM_ICLKEN) },
	{ 1, 0x0003ffff, "CM_AUTOIDLE_PER",
	  OMAP34XX_CM_REGADDR(OMAP3430_PER_MOD, CM_AUTOIDLE) },
	{ 1, 0x00000000, "CM_ICLKEN_USBHOST",
	  OMAP34XX_CM_REGADDR(OMAP3430ES2_USBHOST_MOD, CM_ICLKEN) },
	{ 1, 0x00000001, "CM_AUTOIDLE_USBHOST",
	  OMAP34XX_CM_REGADDR(OMAP3430ES2_USBHOST_MOD, CM_AUTOIDLE) },
	{ 1, 0x00000003, "CM_CLKSTCTRL_MPU",
	  OMAP34XX_CM_REGADDR(MPU_MOD, CM_CLKSTCTRL) },
	{ 1, 0x00000003, "CM_CLKSTCTRL_NEON",
	  OMAP34XX_CM_REGADDR(OMAP3430_NEON_MOD, CM_CLKSTCTRL) },
	{ 1, 0x00000003, "CM_CLKSTCTRL_USBHOST",
	  OMAP34XX_CM_REGADDR(OMAP3430ES2_USBHOST_MOD, CM_CLKSTCTRL) },
};

/* Ptr to structure in SRAM holding PM register just before wfi instruction */
struct omap_pm_debug_sram_registers *_omap34xx_sram_dbg_suspend_struct;

void pm_debug_suspend_entry_init(void **dbg_sram_ptr)
{
	int i, size;
	int nelems;

	size = offsetof(struct omap_pm_debug_sram_registers, regs);
	nelems = ARRAY_SIZE(pm_debug_reg_names);
	/* Debug registers in sram have NULL terminator (hence nelems+1) */
	size += (nelems+1) * sizeof(struct omap_pm_debug_sram_register);

	_omap34xx_sram_dbg_suspend_struct = omap_sram_alloc(size);
	memset(_omap34xx_sram_dbg_suspend_struct, 0, size);

	/* Copy in v_ptrs into sram structure */
	for (i=0; i<nelems; ++i) {
		_omap34xx_sram_dbg_suspend_struct->regs[i].v_ptr =
			pm_debug_reg_names[i].v_ptr;
	}

	/* Flush out of caches into SRAM */
	flush_icache_range((unsigned long)_omap34xx_sram_dbg_suspend_struct, size);
	*dbg_sram_ptr = _omap34xx_sram_dbg_suspend_struct;
}

static void suspend_dbg_reg_show_addr(struct seq_file *s)
{
	int i;
	void *phys;
	struct omap_pm_debug_register_names *name;

	seq_printf(s, "%-8s %-20s    %-8s\n", "PhysAddr", "Register Name", "Expected");
	for (i=0; i<ARRAY_SIZE(pm_debug_reg_names); ++i) {
		name = &pm_debug_reg_names[i];
		phys = OMAP2_L4_IO_PA(name->v_ptr);
		if (name->have_expected_value)
			seq_printf(s, "%p %-20s ?= %08x\n", phys, name->name, name->expected_value);
		else
			seq_printf(s, "%p %-20s ?\n", phys, name->name);
	}
}

static int suspend_dbg_reg_show_values(struct seq_file *s)
{
	struct omap_pm_debug_sram_register *sram;
	struct omap_pm_debug_register_names *name;
	int i;
	void *phys;

	if (!_omap34xx_sram_dbg_suspend_struct->valid)
		return 0;

	sram = _omap34xx_sram_dbg_suspend_struct->regs;
	if (sram->v_ptr)
		seq_printf(s, "%-8s %-20s %-8s    %-8s\n", "PhysAddr", "Register Name", "Value", "Expected");
	while (sram->v_ptr) {
		phys = OMAP2_L4_IO_PA(sram->v_ptr);
		for (i=0; i<ARRAY_SIZE(pm_debug_reg_names); ++i) {
			name = &pm_debug_reg_names[i];
			if (sram->v_ptr == name->v_ptr) {
				if (name->have_expected_value) {
					if (sram->value == name->expected_value)
						seq_printf(s, "%p %-20s %08x\n", phys, name->name, sram->value);
					else
						seq_printf(s, "%p %-20s %08x != %08x\n", phys, name->name, sram->value, name->expected_value);
				} else {
					seq_printf(s, "%p %-20s %08x?\n", phys, name->name, sram->value);
				}

				break;
			}
		}
		if (i == ARRAY_SIZE(pm_debug_reg_names)) {
			char buf[32];
			sprintf(buf, "*(int *)0x%p", phys);
			seq_printf(s, "%-20s %08x\n", buf, sram->value);
		}
		sram++;
	}
	return 0;
}

static int suspend_dbg_reg_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "Suspend Entry Debug Struct: %p\n", _omap34xx_sram_dbg_suspend_struct);
	if (!_omap34xx_sram_dbg_suspend_struct->valid)
		suspend_dbg_reg_show_addr(s);
	else
		suspend_dbg_reg_show_values(s);
	return 0;
}

static int suspend_dbg_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, suspend_dbg_reg_show, inode->i_private);
}

static const struct file_operations suspend_dbg_fops = {
	.open           = suspend_dbg_reg_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static void __init omap_dbg_suspend_init(struct dentry *d)
{
	(void) debugfs_create_file("on_suspend", S_IRUGO,
				d, NULL, &suspend_dbg_fops);
}
#else
static void __init omap_dbg_suspend_init(struct dentry *d)
{
}
#endif

static int __init pm_dbg_init(void)
{
	int i;
	struct dentry *d;
	char name[2];

	if (pm_dbg_init_done)
		return 0;

	if (cpu_is_omap34xx())
		pm_dbg_reg_modules = omap3_pm_reg_modules;
	else {
		printk(KERN_ERR "%s: only OMAP3 supported\n", __func__);
		return -ENODEV;
	}

	d = debugfs_create_dir("pm_debug", NULL);
	if (IS_ERR(d))
		return PTR_ERR(d);

	(void) debugfs_create_file("count", S_IRUGO,
		d, (void *)DEBUG_FILE_COUNTERS, &debug_fops);
	(void) debugfs_create_file("time", S_IRUGO,
		d, (void *)DEBUG_FILE_TIMERS, &debug_fops);

	pwrdm_for_each_nolock(pwrdms_setup, (void *)d);

	pm_dbg_dir = debugfs_create_dir("registers", d);
	if (IS_ERR(pm_dbg_dir))
		return PTR_ERR(pm_dbg_dir);

	(void) debugfs_create_file("current", S_IRUGO,
		pm_dbg_dir, (void *)0, &debug_reg_fops);

	for (i = 0; i < PM_DBG_MAX_REG_SETS; i++)
		if (pm_dbg_reg_set[i] != NULL) {
			sprintf(name, "%d", i+1);
			(void) debugfs_create_file(name, S_IRUGO,
				pm_dbg_dir, (void *)(i+1), &debug_reg_fops);

		}

	(void) debugfs_create_file("enable_off_mode", S_IRUGO | S_IWUGO, d,
				   &enable_off_mode, &pm_dbg_option_fops);
	(void) debugfs_create_file("sleep_while_idle", S_IRUGO | S_IWUGO, d,
				   &sleep_while_idle, &pm_dbg_option_fops);
	(void) debugfs_create_file("wakeup_timer_seconds", S_IRUGO | S_IWUGO, d,
				   &wakeup_timer_seconds, &pm_dbg_option_fops);

	/* Only enable for >= ES2.1 . Going to 0V on anything under
	 * ES2.1 will eventually cause a crash */
	if (omap_rev() > OMAP3430_REV_ES2_0)
		(void) debugfs_create_file("voltage_off_while_idle",
					   S_IRUGO | S_IWUGO, d,
					   &voltage_off_while_idle,
					   &pm_dbg_option_fops);

	omap_dbg_suspend_init(pm_dbg_dir);

	pm_dbg_init_done = 1;

	return 0;
}
arch_initcall(pm_dbg_init);

#endif
