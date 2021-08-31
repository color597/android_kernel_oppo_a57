#ifdef VENDOR_EDIT
//xiaocheng.li@Swdp.shanghai, 2015/11/9, Add new lpm governor
/*
 * Copyright (c) 2015, Guangdong OPPO Mobile Communication(Shanghai)
 * Corp.,Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/cpuidle.h>
#include <linux/pm_qos.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/sched.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <soc/qcom/event_timer.h>

#define MAX_INTERESTING 20000
#define TARGET_RESIDENCY_LIMIT 5000
#define HISTORY_SIZE 5

struct scoring_device {
	int		last_state_idx;
	int		needs_update;
	unsigned int	expected_us;
	u64		predicted_us;
	unsigned int	exit_us;
	unsigned int	latency_min;
	unsigned int 	last_residency;
	unsigned int	history[HISTORY_SIZE];
	unsigned int	history_index;
	unsigned int	history_sum;
	struct hrtimer	cpuidle_hrtimer;
};

#if 0
static enum hrtimer_restart cpuidle_hrtimer_cb(struct hrtimer *h)
{
	return HRTIMER_NORESTART;
}

static void scoring_set_timer(struct scoring_device *data, uint32_t modified_time_us)
{
	u64 modified_time_ns = modified_time_us * NSEC_PER_USEC;
	ktime_t modified_ktime = ns_to_ktime(modified_time_ns);
	data->cpuidle_hrtimer.function = cpuidle_hrtimer_cb;
	hrtimer_start(&data->cpuidle_hrtimer, modified_ktime, HRTIMER_MODE_REL_PINNED);
}
#endif
static DEFINE_PER_CPU(struct scoring_device, scoring_devices);

static void scoring_update(struct cpuidle_driver *drv, struct cpuidle_device *dev);

static int ideal_expectation(struct cpuidle_driver *drv,
		unsigned int expected, unsigned int last_res)
{
	int target_res, i;
	if (!drv)
		return 1;
	if (expected * 4/5 > last_res) {
		for (i = drv->state_count - 1; i > CPUIDLE_DRIVER_STATE_START; i--) {
			target_res = drv->states[i].target_residency;
			if (expected >= target_res) {
				return last_res >= target_res ? 1 : 0;
			}
		}
	}
	//Per the case last_res >= expected (nohz_timer), we have to treat it as correct.
	return 1;
}

/*
 * Use last_residency as prediction if nohz_timer error
 * ratio is above the threshold.
 */
static u64 get_next_residency(struct scoring_device *data)
{
	u64 next_res = data->expected_us;
	if (data->history_sum * 2 <= HISTORY_SIZE &&
		data->last_residency < data->expected_us)
		next_res = data->last_residency;
	return next_res > MAX_INTERESTING ? MAX_INTERESTING : next_res;
}

/**
 * scoring_select - selects the next idle state to enter
 * @drv: cpuidle driver containing state data
 * @dev: the CPU
 */
static int scoring_select(struct cpuidle_driver *drv, struct cpuidle_device *dev)
{
	struct scoring_device *data = &__get_cpu_var(scoring_devices);
	int i, latency_req = pm_qos_request(PM_QOS_CPU_DMA_LATENCY);
	bool qos_disable_lpm = false;
	uint32_t next_event_time = 0;
	uint32_t next_event_us = 0;
	struct timespec t;

	if (data->needs_update) {
		scoring_update(drv, dev);
		data->needs_update = 0;
	}

	data->last_state_idx = 0;
	data->exit_us = 0;

	/* Special case when user has set very strict latency requirement */
	if (unlikely(latency_req == 0))
		return 0;

	/* determine the expected residency time, round up */
	t = ktime_to_timespec(tick_nohz_get_sleep_length());
	data->expected_us =
		t.tv_sec * USEC_PER_SEC + t.tv_nsec / NSEC_PER_USEC;

	data->predicted_us = get_next_residency(data);

	if (!dev->cpu)
		next_event_us = (uint32_t)(ktime_to_us(get_next_event_time(dev->cpu)));
	/*
	 * Find the idle state with the lowest power while satisfying
	 * our constraints.
	 */
	for (i = CPUIDLE_DRIVER_STATE_START; i < drv->state_count; i++) {
		struct cpuidle_state *s = &drv->states[i];
		struct cpuidle_state_usage *su = &dev->states_usage[i];

		if (s->disabled || su->disable)
			continue;
		if (next_event_us) {
			if (s->exit_latency > next_event_us)
				continue;
			if (next_event_us < data->expected_us)
				next_event_time = next_event_us - s->exit_latency;
		}
		if (s->target_residency > data->predicted_us)
			continue;
		if (next_event_time > data->expected_us){
			next_event_time = 0;
			continue;
		}
		if (s->exit_latency > latency_req) {
			qos_disable_lpm = true;
			continue;
		}

		data->last_state_idx = i;
		data->exit_us = s->exit_latency;
	}
#if 0
	if (data->exit_us == data->latency_min &&
		data->expected_us > TARGET_RESIDENCY_LIMIT &&
		!qos_disable_lpm)
		scoring_set_timer(data, TARGET_RESIDENCY_LIMIT);
	if (data->exit_us > data->latency_min){
		if (next_event_time && !dev->cpu)
			scoring_set_timer(data, next_event_time);
		else
			scoring_set_timer(data, data->expected_us - data->exit_us);
	}
#endif
	return data->last_state_idx;
}

/**
 * scoring_reflect - records that data structures need update
 * @dev: the CPU
 * @index: the index of actual entered state
 *
 * NOTE: it's important to be fast here because this operation will add to
 *       the overall exit latency.
 */
static void scoring_reflect(struct cpuidle_device *dev, int index)
{
	struct scoring_device *data = &__get_cpu_var(scoring_devices);
	data->last_state_idx = index;
	if (index >= 0) {
		data->needs_update = 1;
	}
	if (hrtimer_active(&data->cpuidle_hrtimer))
		hrtimer_try_to_cancel(&data->cpuidle_hrtimer);
}

/**
 * scoring_update - attempts to guess what happened after entry
 * @drv: cpuidle driver containing state data
 * @dev: the CPU
 */
static void scoring_update(struct cpuidle_driver *drv, struct cpuidle_device *dev)
{
	struct scoring_device *data = &__get_cpu_var(scoring_devices);
	struct cpuidle_state *target = &drv->states[data->last_state_idx];
	unsigned int last_idle_us;

	/*
	 * Ugh, this idle state doesn't support residency measurements, so we
	 * are basically lost in the dark.  As a compromise, assume we slept
	 * for the whole expected time.
	 */
	if (unlikely(!(target->flags & CPUIDLE_FLAG_TIME_VALID)))
		last_idle_us = data->expected_us;
	else
		last_idle_us = cpuidle_get_last_residency(dev);

	if (last_idle_us > MAX_INTERESTING)
		last_idle_us = MAX_INTERESTING;
	data->history_sum -= data->history[data->history_index];
	data->history[data->history_index] = ideal_expectation(drv,
			data->expected_us, last_idle_us);
	data->history_sum += data->history[data->history_index];
	if (++data->history_index >= HISTORY_SIZE)
		data->history_index = 0;
	data->last_residency = last_idle_us;
}

/**
 * scoring_enable_device - scans a CPU's states and does setup
 * @drv: cpuidle driver
 * @dev: the CPU
 */
static int scoring_enable_device(struct cpuidle_driver *drv,
				struct cpuidle_device *dev)
{
	struct scoring_device *data = &per_cpu(scoring_devices, dev->cpu);
	unsigned int i, latency_min = UINT_MAX;

	memset(data, 0, sizeof(struct scoring_device));
	hrtimer_init(&data->cpuidle_hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	for (i = CPUIDLE_DRIVER_STATE_START; i < drv->state_count; i++) {
		if (latency_min > drv->states[i].exit_latency)
			latency_min = drv->states[i].exit_latency;
	}
	data->latency_min = latency_min;
	return 0;
}

static struct cpuidle_governor scoring_governor = {
	.name =		"scoring",
	.rating =	25,
	.enable =	scoring_enable_device,
	.select =	scoring_select,
	.reflect =	scoring_reflect,
	.owner =	THIS_MODULE,
};

/**
 * init_scoring - initializes the governor
 */
static int __init init_scoring(void)
{
	return cpuidle_register_governor(&scoring_governor);
}

MODULE_LICENSE("GPL");
module_init(init_scoring);
#endif
