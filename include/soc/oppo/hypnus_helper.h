/*
 * Copyright (C) 2016 OPPO, Inc.
 * Author: Jie Cheng <jie.cheng@oppo.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __HYPNUS_HELPER_H__
#define __HYPNUS_HELPER_H__

#define DEFAULT_CAPACITY 50
#define HEAVY_TRAFFIC 100000	//100(KBytes)

/* mmc */
extern int hypnus_mmc_scaling_enable(int index, int value);

/* power */
extern int hypnus_get_batt_capacity(void);
extern bool hypnus_get_charger_status(void);
extern bool hypnus_get_traffic_status(void);

/* sched */
extern int sched_set_prefer_idle(unsigned int is_prefer_idle);
extern int sched_set_small_task(int load_pct);
extern unsigned int sched_get_small_task(void);
extern int sched_set_cpu_mostly_idle_load(int cpu, int mostly_idle_pct);
extern int sched_get_cpu_mostly_idle_load(int cpu);
extern int sched_set_cpu_mostly_idle_nr_run(int cpu, int nr_run);
extern int sched_get_cpu_mostly_idle_nr_run(int cpu);

#endif
