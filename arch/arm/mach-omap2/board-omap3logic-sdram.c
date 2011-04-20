/*
 * SDRC register values for the Samsung K4X1G323PC
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Lauri Leukkunen <lauri.leukkunen@nokia.com>
 *
 * Original code by Juha Yrjölä <juha.yrjola@solidboot.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>

#include <plat/io.h>
#include <plat/common.h>
#include <plat/clock.h>
#include <plat/sdrc.h>


/* In picoseconds, except for tREF (ns), tXP, tCKE, tWTR (clks) */
struct sdram_timings {
	u32 casl;
	u32 tDAL;
	u32 tDPL;
	u32 tRRD;
	u32 tRCD;
	u32 tRP;
	u32 tRAS;
	u32 tRC;
	u32 tRFC;
	u32 tXSR;

	u32 tREF; /* in ns */

	u32 tXP;
	u32 tCKE;
	u32 tWTR;
};

struct omap_sdrc_params omap3logic_sdrc_params[4];

/* Timings are for the -6 parts */
static const struct sdram_timings omap3logic_timings[] = {
	{
		.casl = 3,
		.tDAL = 15000 + 18000, /* (tWR/tCK) + (tRP/tCK) */
		.tDPL = 15000,  /* tWR */
		.tRRD = 12000,	/* -5=10, -54=10.8, -6=12, -75=15 */
		.tRCD = 18000,	/* -5=15, -54=58.2, -6=60, -75=67.5 */
		.tRP = 18000,	/* -5=15, -54=16.2, -6=18, -75=22.5 */
		.tRAS = 42000,	/* -5=40, -54=42.0, -6=42, -75-45 */
		.tRC = 60000,	/* -5=55, -54=58.2, -6=50, -75=67.5 */
		.tRFC = 110000, /* =110 */
		.tXSR = 138000,	/* =138 */

		.tREF = 64000,  /* =64mSec */
		.tXP = 2,	/* =2 */
		.tCKE = 1,	/* =1 */
		.tWTR = 1,	/* -5=2, -54=2, -6=1, -75=1 */
	},
};

static unsigned long sdrc_get_fclk_period(long rate)
{
	/* In picoseconds */
	return 1000000000 / rate;
}

static unsigned int sdrc_ps_to_ticks(unsigned int time_ps, long rate)
{
	unsigned long tick_ps;

	/* Calculate in picosecs to yield more exact results */
	tick_ps = sdrc_get_fclk_period(rate);

	return (time_ps + tick_ps - 1) / tick_ps;
}

#undef DEBUG
#ifdef DEBUG
static int set_sdrc_timing_regval(u32 *regval, int st_bit, int end_bit,
				int ticks, long rate, const char *name)
#else
static int set_sdrc_timing_regval(u32 *regval, int st_bit, int end_bit,
			       int ticks)
#endif
{
	int mask, nr_bits;

	nr_bits = end_bit - st_bit + 1;
	if (ticks >= 1 << nr_bits)
		return -1;
	mask = (1 << nr_bits) - 1;
	*regval &= ~(mask << st_bit);
	*regval |= ticks << st_bit;
#ifdef DEBUG
	printk(KERN_INFO "SDRC %s: %i ticks %i ns\n", name, ticks,
			(unsigned int)sdrc_get_fclk_period(rate) * ticks /
			1000);
#endif

	return 0;
}

#ifdef DEBUG
#define SDRC_SET_ONE(reg, st, end, field, rate) \
	if (set_sdrc_timing_regval((reg), (st), (end), \
			omap3logic_timings->field, (rate), #field) < 0) \
		err = -1;
#else
#define SDRC_SET_ONE(reg, st, end, field, rate) \
	if (set_sdrc_timing_regval((reg), (st), (end), \
			omap3logic_timings->field) < 0) \
		err = -1;
#endif

#ifdef DEBUG
static int set_sdrc_timing_regval_ps(u32 *regval, int st_bit, int end_bit,
				int time, long rate, const char *name)
#else
static int set_sdrc_timing_regval_ps(u32 *regval, int st_bit, int end_bit,
				int time, long rate)
#endif
{
	int ticks, ret;
	ret = 0;

	if (time == 0)
		ticks = 0;
	else
		ticks = sdrc_ps_to_ticks(time, rate);

#ifdef DEBUG
	ret = set_sdrc_timing_regval(regval, st_bit, end_bit, ticks,
				     rate, name);
#else
	ret = set_sdrc_timing_regval(regval, st_bit, end_bit, ticks);
#endif

	return ret;
}

#ifdef DEBUG
#define SDRC_SET_ONE_PS(reg, st, end, field, rate) \
	if (set_sdrc_timing_regval_ps((reg), (st), (end), \
			omap3logic_timings->field, \
			(rate), #field) < 0) \
		err = -1;

#else
#define SDRC_SET_ONE_PS(reg, st, end, field, rate) \
	if (set_sdrc_timing_regval_ps((reg), (st), (end), \
			omap3logic_timings->field, (rate)) < 0) \
		err = -1;
#endif

static int sdrc_timings(int id, long rate)
{
	u32 ticks_per_ms;
	u32 rfr, l;
	u32 actim_ctrla = 0, actim_ctrlb = 0;
	u32 rfr_ctrl;
	int err = 0;
	long l3_rate = rate / 1000;

	SDRC_SET_ONE_PS(&actim_ctrla,  0,  4, tDAL, l3_rate);
	SDRC_SET_ONE_PS(&actim_ctrla,  6,  8, tDPL, l3_rate);
	SDRC_SET_ONE_PS(&actim_ctrla,  9, 11, tRRD, l3_rate);
	SDRC_SET_ONE_PS(&actim_ctrla, 12, 14, tRCD, l3_rate);
	SDRC_SET_ONE_PS(&actim_ctrla, 15, 17, tRP, l3_rate);
	SDRC_SET_ONE_PS(&actim_ctrla, 18, 21, tRAS, l3_rate);
	SDRC_SET_ONE_PS(&actim_ctrla, 22, 26, tRC, l3_rate);
	SDRC_SET_ONE_PS(&actim_ctrla, 27, 31, tRFC, l3_rate);

	SDRC_SET_ONE_PS(&actim_ctrlb,  0,  7, tXSR, l3_rate);

	SDRC_SET_ONE(&actim_ctrlb,  8, 10, tXP, l3_rate);
	SDRC_SET_ONE(&actim_ctrlb, 12, 14, tCKE, l3_rate);
	SDRC_SET_ONE(&actim_ctrlb, 16, 17, tWTR, l3_rate);

	ticks_per_ms = l3_rate;
#ifdef DEBUG
	printk("%s: l3_rate %ld tRef %d\n", __FUNCTION__, l3_rate,
		omap3logic_timings[0].tREF);
#endif
	rfr = omap3logic_timings[0].tREF * ticks_per_ms / 1000000;
	if (rfr > 65535 + 50)
		rfr = 65535;
	else
		rfr -= 50;

#ifdef DEBUG
	printk(KERN_INFO "SDRC tREF: %i ticks\n", rfr);
#endif

	l = rfr << 8;
	rfr_ctrl = l | 0x1; /* autorefresh, reload counter with 1xARCV */

	omap3logic_sdrc_params[id].rate = rate;
	omap3logic_sdrc_params[id].actim_ctrla = actim_ctrla;
	omap3logic_sdrc_params[id].actim_ctrlb = actim_ctrlb;
	omap3logic_sdrc_params[id].rfr_ctrl = rfr_ctrl;
	omap3logic_sdrc_params[id].mr = 0x32;

	omap3logic_sdrc_params[id + 1].rate = 0;

	return err;
}

struct omap_sdrc_params *omap3logic_get_sdram_timings(void)
{
	int err;

	err = sdrc_timings(0, 41500000);
	err |= sdrc_timings(1, 83000000);
	err |= sdrc_timings(2, 166000000);

	return &omap3logic_sdrc_params[0];
}

void dump_omap3logic_timings(void)
{
	int i;
	struct omap_sdrc_params *p;
	for (i=0; i<ARRAY_SIZE(omap3logic_sdrc_params); i++) {
		p = &omap3logic_sdrc_params[i];
		if (p->rate) {
			printk("SDRC[%d]: rate %ld ctrla %08x ctrlb %08x\n",
				i, p->rate, p->actim_ctrla, p->actim_ctrlb);
			printk("       : rfr_ctrl %08x mr %08x\n",
				p->rfr_ctrl, p->mr);
		}
	}
}
