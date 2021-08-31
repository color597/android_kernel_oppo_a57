#undef TRACE_SYSTEM
#define TRACE_SYSTEM regulator

#if !defined(_TRACE_REGULATOR_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_REGULATOR_H

#include <linux/ktime.h>
#include <linux/tracepoint.h>
//#ifdef VENDOR_EDIT //yixue.ge@bsp.drv modify b11
/*
 * Events that take a range of old and new numerical values, mostly for
 * cpr voltages and so on.
 */
DECLARE_EVENT_CLASS(cpr_regulator_range,

	TP_PROTO(const char *name, u64 func_cnt, int old_apc, int new_apc,
		int old_acc_1, int old_acc_2, int new_acc),

	TP_ARGS(name, func_cnt, old_apc, new_apc, old_acc_1,
		old_acc_2, new_acc),

	TP_STRUCT__entry(
		__string(       name,		name            )
		__field(        u64,            func_cnt        )
		__field(        int,            old_apc         )
		__field(        int,            new_apc         )
		__field(        int,            old_acc_1       )
		__field(        int,            old_acc_2       )
		__field(        int,            new_acc         )
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->func_cnt   = func_cnt;
		__entry->old_apc    = old_apc;
		__entry->new_apc    = new_apc;
		__entry->old_acc_1  = old_acc_1;
		__entry->old_acc_2  = old_acc_2;
		__entry->new_acc    = new_acc;
	),

	TP_printk("name=%s call_count = %llu, APC_corner [%d -> %d], MEM_ACC [%d(%d) -> %d]",
			__get_str(name), (u64)__entry->func_cnt,
		  (int)__entry->old_apc, (int)__entry->new_apc,
		  (int)__entry->old_acc_1, (int)__entry->old_acc_2,
		  (int)__entry->new_acc)
);

DEFINE_EVENT(cpr_regulator_range, cpr_regulator_set_voltage,

	TP_PROTO(const char *name, u64 func_cnt, int old_apc, int new_apc,
		int old_acc_1, int old_acc_2, int new_acc),

	TP_ARGS(name, func_cnt, old_apc, new_apc, old_acc_1, old_acc_2, new_acc)

);


/*
 * Events that take a single value, mostly for readback and refcounts.
 */
DECLARE_EVENT_CLASS(cpr_regulator_value,

	TP_PROTO(const char *name, u64 func_cnt, unsigned int val, int rc),

	TP_ARGS(name, func_cnt, val, rc),

	TP_STRUCT__entry(
		__string(       name,		name            )
		__field(        u64,            func_cnt        )
		__field(        unsigned int,   val             )
		__field(        int,		rc              )
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->func_cnt  = func_cnt;
		__entry->val  = val;
		__entry->rc  = rc;
	),

	TP_printk("name=%s, call_count = %llu, APC_corner = %u, rc = %d",
		__get_str(name), (u64)__entry->func_cnt,
		(int)__entry->val, (int)__entry->rc)
);

DEFINE_EVENT(cpr_regulator_value, cpr_regulator_set_voltage_complete,

	TP_PROTO(const char *name, u64 func_cnt, unsigned int value, int rc),

	TP_ARGS(name, func_cnt, value, rc)

);


/*
 * Events that take a range of old and new numerical values, mostly for
 * cpr voltages and so on.
 */
DECLARE_EVENT_CLASS(mem_acc_regulator_range,

	TP_PROTO(const char *name, u64 func_cnt, int old_acc, int new_acc),

	TP_ARGS(name, func_cnt, old_acc, new_acc),

	TP_STRUCT__entry(
		__string(       name,		name            )
		__field(        u64,            func_cnt        )
		__field(        int,            old_acc         )
		__field(        int,            new_acc         )
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->func_cnt  = func_cnt;
		__entry->old_acc  = old_acc;
		__entry->new_acc  = new_acc;
	),

	TP_printk("name=%s call_count = %llu, MEM_ACC [%d -> %d]",
			__get_str(name), (u64)__entry->func_cnt,
		  (int)__entry->old_acc, (int)__entry->new_acc)
);

DEFINE_EVENT(mem_acc_regulator_range, mem_acc_regulator_set_voltage,

	TP_PROTO(const char *name, u64 func_cnt, int old_acc, int new_acc),

	TP_ARGS(name, func_cnt, old_acc, new_acc)

);


/*
 * Events that take a single value, mostly for readback and refcounts.
 */
DECLARE_EVENT_CLASS(mem_acc_regulator_value,

	TP_PROTO(const char *name, u64 func_cnt, unsigned int val, int rc),

	TP_ARGS(name, func_cnt, val, rc),

	TP_STRUCT__entry(
		__string(       name,		name            )
		__field(        u64,            func_cnt        )
		__field(        unsigned int,   val             )
		__field(        int,		rc              )
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->func_cnt  = func_cnt;
		__entry->val  = val;
		__entry->rc  = rc;
	),

	TP_printk("name=%s, call_count = %llu, MEM_ACC_corner = %u, rc = %d",
		__get_str(name), (u64)__entry->func_cnt,
		(int)__entry->val, (int)__entry->rc)
);

DEFINE_EVENT(mem_acc_regulator_value, mem_acc_regulator_set_voltage_complete,

	TP_PROTO(const char *name, u64 func_cnt, unsigned int value, int rc),

	TP_ARGS(name, func_cnt, value, rc)

);


/*
 * Events that take a range of old and new numerical values, mostly for
 * spm voltages and so on.
 */
DECLARE_EVENT_CLASS(spm_regulator_range,

	TP_PROTO(const char *name, u64 func_cnt, int old_vol, int new_vol),

	TP_ARGS(name, func_cnt, old_vol, new_vol),

	TP_STRUCT__entry(
		__string(       name,		name            )
		__field(        u64,            func_cnt        )
		__field(        int,            old_vol         )
		__field(        int,            new_vol         )
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->func_cnt  = func_cnt;
		__entry->old_vol  = old_vol;
		__entry->new_vol  = new_vol;
	),

	TP_printk("name=%s call_count = %llu, PMIC_VOLTAGE [%d -> %d]",
			__get_str(name), (u64)__entry->func_cnt,
		  (int)__entry->old_vol, (int)__entry->new_vol)
);

DEFINE_EVENT(spm_regulator_range, spm_regulator_set_voltage,

	TP_PROTO(const char *name, u64 func_cnt, int old_vol, int new_vol),

	TP_ARGS(name, func_cnt, old_vol, new_vol)

);


/*
 * Events that take a single value, mostly for readback and refcounts.
 */
DECLARE_EVENT_CLASS(spm_regulator_value,

	TP_PROTO(const char *name, u64 func_cnt, unsigned int val, int rc),

	TP_ARGS(name, func_cnt, val, rc),

	TP_STRUCT__entry(
		__string(       name,		name            )
		__field(        u64,            func_cnt        )
		__field(        unsigned int,   val             )
		__field(        int,		rc              )
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->func_cnt  = func_cnt;
		__entry->val  = val;
		__entry->rc  = rc;
	),

	TP_printk("name=%s, call_count = %llu, PMIC_VOLTAGE = %u, rc = %d",
		__get_str(name), (u64)__entry->func_cnt,
		(int)__entry->val, (int)__entry->rc)
);

DEFINE_EVENT(spm_regulator_value, spm_regulator_set_voltage_complete,

	TP_PROTO(const char *name, u64 func_cnt, unsigned int value, int rc),

	TP_ARGS(name, func_cnt, value, rc)

);


//#endif
/*
 * Events which just log themselves and the regulator name for enable/disable
 * type tracking.
 */
DECLARE_EVENT_CLASS(regulator_basic,

	TP_PROTO(const char *name),

	TP_ARGS(name),

	TP_STRUCT__entry(
		__string(	name,	name	)
	),

	TP_fast_assign(
		__assign_str(name, name);
	),

	TP_printk("name=%s", __get_str(name))

);

DEFINE_EVENT(regulator_basic, regulator_enable,

	TP_PROTO(const char *name),

	TP_ARGS(name)

);

DEFINE_EVENT(regulator_basic, regulator_enable_delay,

	TP_PROTO(const char *name),

	TP_ARGS(name)

);

DEFINE_EVENT(regulator_basic, regulator_enable_complete,

	TP_PROTO(const char *name),

	TP_ARGS(name)

);

DEFINE_EVENT(regulator_basic, regulator_disable,

	TP_PROTO(const char *name),

	TP_ARGS(name)

);

DEFINE_EVENT(regulator_basic, regulator_disable_complete,

	TP_PROTO(const char *name),

	TP_ARGS(name)

);

/*
 * Events that take a range of numerical values, mostly for voltages
 * and so on.
 */
DECLARE_EVENT_CLASS(regulator_range,

	TP_PROTO(const char *name, int min, int max),

	TP_ARGS(name, min, max),

	TP_STRUCT__entry(
		__string(	name,		name		)
		__field(        int,            min             )
		__field(        int,            max             )
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->min  = min;
		__entry->max  = max;
	),

	TP_printk("name=%s (%d-%d)", __get_str(name),
		  (int)__entry->min, (int)__entry->max)
);

DEFINE_EVENT(regulator_range, regulator_set_voltage,

	TP_PROTO(const char *name, int min, int max),

	TP_ARGS(name, min, max)

);


/*
 * Events that take a single value, mostly for readback and refcounts.
 */
DECLARE_EVENT_CLASS(regulator_value,

	TP_PROTO(const char *name, unsigned int val),

	TP_ARGS(name, val),

	TP_STRUCT__entry(
		__string(	name,		name		)
		__field(        unsigned int,   val             )
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->val  = val;
	),

	TP_printk("name=%s, val=%u", __get_str(name),
		  (int)__entry->val)
);

DEFINE_EVENT(regulator_value, regulator_set_voltage_complete,

	TP_PROTO(const char *name, unsigned int value),

	TP_ARGS(name, value)

);

#endif /* _TRACE_POWER_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
