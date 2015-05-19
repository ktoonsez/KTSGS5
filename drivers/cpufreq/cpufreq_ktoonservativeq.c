/*
 *  drivers/cpufreq/cpufreq_ktoonservative.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *            (C)  2009 Alexander Clouter <alex@digriz.org.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpufreq_kt.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>

/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

#define DEF_BOOST_CPU				(1804800)
#define DEF_BOOST_GPU				(462400)
#define DEF_BOOST_HOLD_CYCLES			(22)
#define DEF_DISABLE_hotplug			(0)
#define CPUS_AVAILABLE				num_possible_cpus()

bool ktoonservative_is_active = false;
static int hotplug_cpu_enable_up[] = { 0, 52, 65, 78 };
static int hotplug_cpu_enable_down[] = { 0, 35, 45, 55 };
static int hotplug_cpu_single_up[] = { 0, 0, 0, 0 };
static int hotplug_cpu_single_down[] = { 0, 0, 0, 0 };
static int hotplug_cpu_lockout[] = { 0, 0, 0, 0 };
static int hotplug_cpu_boosted[] = { 0, 0, 0, 0 };
unsigned int cpu_load[] = { -1, -1, -1, -1 };
static bool hotplug_flag_on = false;
static bool disable_hotplug_chrg_override;
static bool disable_hotplug_media_override;

void setExtraCores(unsigned int requested_freq, bool isFirst);
void set_core_flag_up(unsigned int cpu, unsigned int val);
void set_core_flag_down(unsigned int cpu, unsigned int val);
unsigned int kt_freq_control[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
/*
 * The polling frequency of this governor depends on the capability of
 * the processor. Default polling frequency is 1000 times the transition
 * latency of the processor. The governor will work on any processor with
 * transition latency <= 10mS, using appropriate sampling
 * rate.
 * For CPUs with transition latency > 10mS (mostly drivers with CPUFREQ_ETERNAL)
 * this governor will not work.
 * All times here are in uS.
 */
#define MIN_SAMPLING_RATE_RATIO			(2)

static bool turned_off_super_conservative_screen_off = false;
static bool fake_screen_on = false;

static bool disable_hotplug_bt_active = false;
static unsigned int min_sampling_rate;
static unsigned int stored_sampling_rate = 35000;
static bool boostpulse_relayf = false;
static int boost_hold_cycles_cnt = 0;
static bool screen_is_on = true;
extern void boost_the_gpu(unsigned int freq, bool getfreq);

extern void apenable_auto_hotplug(bool state);
extern bool apget_enable_auto_hotplug(void);
static bool prev_apenable;
static bool hotplugInProgress = false;

#define LATENCY_MULTIPLIER			(1000)
#define MIN_LATENCY_MULTIPLIER			(100)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(10)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)
#define OVERRIDE_DISABLER			(-999999)

struct work_struct hotplug_offline_work;
struct work_struct hotplug_online_work;
static spinlock_t cpufreq_up_lock;
static spinlock_t cpufreq_down_lock;

static void do_dbs_timer(struct work_struct *work);

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	unsigned int down_skip;
	unsigned int requested_freq;
	int cpu;
	unsigned int enable:1;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
	unsigned int Lblock_cycles_online;
	int Lblock_cycles_offline;
	unsigned int Lblock_cycles_raise;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, cs_cpu_dbs_info);
static unsigned int Lblock_cycles_online_OVERRIDE[] = { OVERRIDE_DISABLER, OVERRIDE_DISABLER, OVERRIDE_DISABLER, OVERRIDE_DISABLER };
static int Lblock_cycles_offline_OVERRIDE[] = { OVERRIDE_DISABLER, OVERRIDE_DISABLER, OVERRIDE_DISABLER, OVERRIDE_DISABLER };

static unsigned int cpus_online;	/* number of CPUs using this policy */

/*
 * dbs_mutex protects cpus_online in governor start/stop.
 */
static struct mutex dbs_mutex;

static struct workqueue_struct *dbs_wq;

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int sampling_rate_screen_off;
	unsigned int sampling_down_factor;
	unsigned int up_threshold_screen_on;
	unsigned int up_threshold_screen_on_hotplug_1;
	unsigned int up_threshold_screen_on_hotplug_2;
	unsigned int up_threshold_screen_on_hotplug_3;
	unsigned int up_threshold_screen_off;
	unsigned int up_threshold_screen_off_hotplug_1;
	unsigned int up_threshold_screen_off_hotplug_2;
	unsigned int up_threshold_screen_off_hotplug_3;
	unsigned int down_threshold_screen_on;
	unsigned int down_threshold_screen_on_hotplug_1;
	unsigned int down_threshold_screen_on_hotplug_2;
	unsigned int down_threshold_screen_on_hotplug_3;
	unsigned int down_threshold_screen_off;
	unsigned int down_threshold_screen_off_hotplug_1;
	unsigned int down_threshold_screen_off_hotplug_2;
	unsigned int down_threshold_screen_off_hotplug_3;
	unsigned int block_cycles_online_screen_on;
	int block_cycles_offline_screen_on;
	unsigned int block_cycles_raise_screen_on;
	unsigned int block_cycles_online_screen_off;
	int block_cycles_offline_screen_off;
	unsigned int block_cycles_raise_screen_off;
	unsigned int super_conservative_screen_on;
	unsigned int super_conservative_screen_off;
	unsigned int touch_boost_cpu;
	unsigned int touch_boost_cpu_all_cores;
	unsigned int touch_boost_2nd_core;
	unsigned int touch_boost_3rd_core;
	unsigned int touch_boost_4th_core;
	unsigned int boost_2nd_core_on_button_screen_on;
	unsigned int boost_3rd_core_on_button_screen_on;
	unsigned int boost_4th_core_on_button_screen_on;
	unsigned int boost_2nd_core_on_button_screen_off;
	unsigned int boost_3rd_core_on_button_screen_off;
	unsigned int boost_4th_core_on_button_screen_off;
	unsigned int lockout_2nd_core_hotplug_screen_on;
	unsigned int lockout_3rd_core_hotplug_screen_on;
	unsigned int lockout_4th_core_hotplug_screen_on;
	unsigned int lockout_2nd_core_hotplug_screen_off;
	unsigned int lockout_3rd_core_hotplug_screen_off;
	unsigned int lockout_4th_core_hotplug_screen_off;
	unsigned int lockout_changes_when_boosting;
	unsigned int touch_boost_gpu;
	unsigned int cpu_load_adder_at_max_gpu;
	unsigned int cpu_load_adder_at_max_gpu_ignore_tb;
	unsigned int sync_extra_cores_screen_on;
	unsigned int sync_extra_cores_screen_off;
	unsigned int boost_hold_cycles;
	unsigned int disable_hotplug;
	unsigned int disable_hotplug_chrg;
	unsigned int disable_hotplug_media;
	unsigned int disable_hotplug_bt;
	unsigned int no_extra_cores_screen_off;
	unsigned int ignore_nice;
	unsigned int freq_step_raise_screen_on;
	unsigned int freq_step_raise_screen_off;
	unsigned int freq_step_lower_screen_on;
	unsigned int freq_step_lower_screen_off;
	unsigned int debug_enabled;
} dbs_tuners_ins = {
	.up_threshold_screen_on = 57,
	.up_threshold_screen_on_hotplug_1 = 52,
	.up_threshold_screen_on_hotplug_2 = 65,
	.up_threshold_screen_on_hotplug_3 = 68,
	.up_threshold_screen_off = 57,
	.up_threshold_screen_off_hotplug_1 = 58,
	.up_threshold_screen_off_hotplug_2 = 68,
	.up_threshold_screen_off_hotplug_3 = 78,
	.down_threshold_screen_on = 52,
	.down_threshold_screen_on_hotplug_1 = 35,
	.down_threshold_screen_on_hotplug_2 = 45,
	.down_threshold_screen_on_hotplug_3 = 55,
	.down_threshold_screen_off = 52,
	.down_threshold_screen_off_hotplug_1 = 35,
	.down_threshold_screen_off_hotplug_2 = 45,
	.down_threshold_screen_off_hotplug_3 = 55,
	.block_cycles_online_screen_on = 3,
	.block_cycles_offline_screen_on = 11,
	.block_cycles_raise_screen_on = 3,
	.block_cycles_online_screen_off = 11,
	.block_cycles_offline_screen_off =1,
	.block_cycles_raise_screen_off = 11,
	.super_conservative_screen_on = 0,
	.super_conservative_screen_off = 0,
	.touch_boost_cpu = DEF_BOOST_CPU,
	.touch_boost_cpu_all_cores = 0,
	.touch_boost_2nd_core = 1,
	.touch_boost_3rd_core = 0,
	.touch_boost_4th_core = 0,
	.boost_2nd_core_on_button_screen_on = 1,
	.boost_3rd_core_on_button_screen_on = 0,
	.boost_4th_core_on_button_screen_on = 0,
	.boost_2nd_core_on_button_screen_off = 1,
	.boost_3rd_core_on_button_screen_off = 0,
	.boost_4th_core_on_button_screen_off = 0,
	.lockout_2nd_core_hotplug_screen_on = 0,
	.lockout_3rd_core_hotplug_screen_on = 0,
	.lockout_4th_core_hotplug_screen_on = 0,
	.lockout_2nd_core_hotplug_screen_off = 0,
	.lockout_3rd_core_hotplug_screen_off = 0,
	.lockout_4th_core_hotplug_screen_off = 0,
	.lockout_changes_when_boosting = 0,
	.touch_boost_gpu = DEF_BOOST_GPU,
	.cpu_load_adder_at_max_gpu = 0,
	.cpu_load_adder_at_max_gpu_ignore_tb = 0,
	.sync_extra_cores_screen_on = 0,
	.sync_extra_cores_screen_off = 0,
	.boost_hold_cycles = DEF_BOOST_HOLD_CYCLES,
	.disable_hotplug = DEF_DISABLE_hotplug,
	.disable_hotplug_chrg = 0,
	.disable_hotplug_media = 0,
	.disable_hotplug_bt = 0,
	.no_extra_cores_screen_off = 1,
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
	.sampling_rate = 35000,
	.sampling_rate_screen_off = 40000,
	.ignore_nice = 0,
	.freq_step_raise_screen_on = 5,
	.freq_step_raise_screen_off = 1,
	.freq_step_lower_screen_on = 2,
	.freq_step_lower_screen_off = 8,
	.debug_enabled = 0,
};

static inline cputime64_t get_cpu_idle_time_jiffy(unsigned int cpu,
						  cputime64_t *wall)
{
	u64 idle_time;
	u64 cur_wall_time;
	u64 busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());

	busy_time  = kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];

	idle_time = cur_wall_time - busy_time;
	if (wall)
		*wall = jiffies_to_usecs(cur_wall_time);

	return jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, wall);

	if (idle_time == -1ULL)
		idle_time = get_cpu_idle_time_jiffy(cpu, wall);
	
	return idle_time;
}

/* keep track of frequency transitions */
static int dbs_cpufreq_notifier(struct notifier_block *nb, unsigned long val,
		     void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cpu_dbs_info_s *this_dbs_info = &per_cpu(cs_cpu_dbs_info,
							freq->cpu);

	struct cpufreq_policy *policy;

	if (!this_dbs_info->enable)
		return 0;

	policy = this_dbs_info->cur_policy;

	/*
	 * we only care if our internally tracked freq moves outside
	 * the 'valid' ranges of freqency available to us otherwise
	 * we do not change it
	*/
	if (this_dbs_info->requested_freq > policy->max
			|| this_dbs_info->requested_freq < policy->min)
		this_dbs_info->requested_freq = freq->new;

	return 0;
}

static struct notifier_block dbs_cpufreq_notifier_block = {
	.notifier_call = dbs_cpufreq_notifier
};

void set_bluetooth_state_kt(bool val)
{
	if (val == true && dbs_tuners_ins.disable_hotplug_bt == 1)
	{
		disable_hotplug_bt_active = true;
		if (num_online_cpus() < 2)
		{
			int cpu;
			for (cpu = 1; cpu < CPUS_AVAILABLE; cpu++)
			{
				if (!cpu_online(cpu))
					set_core_flag_up(cpu, 1);
			}
			queue_work_on(0, dbs_wq, &hotplug_online_work);
		}
	}
	else
		disable_hotplug_bt_active = false;
}

void send_cable_state_kt(unsigned int state)
{
	int cpu;
	if (state && dbs_tuners_ins.disable_hotplug_chrg)
	{
		disable_hotplug_chrg_override = true;
		for (cpu = 1; cpu < CPUS_AVAILABLE; cpu++)
			set_core_flag_up(cpu, 1);
		queue_work_on(0, dbs_wq, &hotplug_online_work);
	}
	else
		disable_hotplug_chrg_override = false;
}

bool set_music_playing_statekt(bool state)
{
	int cpu;
	bool ret = false;
	if (state && dbs_tuners_ins.disable_hotplug_media)
	{
		disable_hotplug_media_override = true;
		for (cpu = 1; cpu < CPUS_AVAILABLE; cpu++)
			set_core_flag_up(cpu, 1);
		queue_work_on(0, dbs_wq, &hotplug_online_work);
		ret = true;
	}
	else
		disable_hotplug_media_override = false;
	
	return ret;
}

/************************** sysfs interface ************************/
static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", min_sampling_rate);
}
define_one_global_ro(sampling_rate_min);

static ssize_t show_touch_boost_cpu(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dbs_tuners_ins.touch_boost_cpu);
}

static ssize_t show_touch_boost_cpu_all_cores(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dbs_tuners_ins.touch_boost_cpu_all_cores);
}

static ssize_t show_sync_extra_cores_screen_on(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dbs_tuners_ins.sync_extra_cores_screen_on);
}

static ssize_t show_sync_extra_cores_screen_off(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", dbs_tuners_ins.sync_extra_cores_screen_off);
}

/* cpufreq_ktoonservative Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(sampling_rate_screen_off, sampling_rate_screen_off);
show_one(sampling_down_factor, sampling_down_factor);
show_one(up_threshold_screen_on, up_threshold_screen_on);
show_one(up_threshold_screen_on_hotplug_1, up_threshold_screen_on_hotplug_1);
show_one(up_threshold_screen_on_hotplug_2, up_threshold_screen_on_hotplug_2);
show_one(up_threshold_screen_on_hotplug_3, up_threshold_screen_on_hotplug_3);
show_one(up_threshold_screen_off, up_threshold_screen_off);
show_one(up_threshold_screen_off_hotplug_1, up_threshold_screen_off_hotplug_1);
show_one(up_threshold_screen_off_hotplug_2, up_threshold_screen_off_hotplug_2);
show_one(up_threshold_screen_off_hotplug_3, up_threshold_screen_off_hotplug_3);
show_one(down_threshold_screen_on, down_threshold_screen_on);
show_one(down_threshold_screen_on_hotplug_1, down_threshold_screen_on_hotplug_1);
show_one(down_threshold_screen_on_hotplug_2, down_threshold_screen_on_hotplug_2);
show_one(down_threshold_screen_on_hotplug_3, down_threshold_screen_on_hotplug_3);
show_one(down_threshold_screen_off, down_threshold_screen_off);
show_one(down_threshold_screen_off_hotplug_1, down_threshold_screen_off_hotplug_1);
show_one(down_threshold_screen_off_hotplug_2, down_threshold_screen_off_hotplug_2);
show_one(down_threshold_screen_off_hotplug_3, down_threshold_screen_off_hotplug_3);
show_one(block_cycles_online_screen_on, block_cycles_online_screen_on);
show_one(block_cycles_offline_screen_on, block_cycles_offline_screen_on);
show_one(block_cycles_raise_screen_on, block_cycles_raise_screen_on);
show_one(block_cycles_online_screen_off, block_cycles_online_screen_off);
show_one(block_cycles_offline_screen_off, block_cycles_offline_screen_off);
show_one(block_cycles_raise_screen_off, block_cycles_raise_screen_off);
show_one(super_conservative_screen_on, super_conservative_screen_on);
show_one(super_conservative_screen_off, super_conservative_screen_off);
show_one(touch_boost_2nd_core, touch_boost_2nd_core);
show_one(touch_boost_3rd_core, touch_boost_3rd_core);
show_one(touch_boost_4th_core, touch_boost_4th_core);
show_one(boost_2nd_core_on_button_screen_on, boost_2nd_core_on_button_screen_on);
show_one(boost_3rd_core_on_button_screen_on, boost_3rd_core_on_button_screen_on);
show_one(boost_4th_core_on_button_screen_on, boost_4th_core_on_button_screen_on);
show_one(boost_2nd_core_on_button_screen_off, boost_2nd_core_on_button_screen_off);
show_one(boost_3rd_core_on_button_screen_off, boost_3rd_core_on_button_screen_off);
show_one(boost_4th_core_on_button_screen_off, boost_4th_core_on_button_screen_off);
show_one(lockout_2nd_core_hotplug_screen_on, lockout_2nd_core_hotplug_screen_on);
show_one(lockout_3rd_core_hotplug_screen_on, lockout_3rd_core_hotplug_screen_on);
show_one(lockout_4th_core_hotplug_screen_on, lockout_4th_core_hotplug_screen_on);
show_one(lockout_2nd_core_hotplug_screen_off, lockout_2nd_core_hotplug_screen_off);
show_one(lockout_3rd_core_hotplug_screen_off, lockout_3rd_core_hotplug_screen_off);
show_one(lockout_4th_core_hotplug_screen_off, lockout_4th_core_hotplug_screen_off);
show_one(lockout_changes_when_boosting, lockout_changes_when_boosting);
show_one(touch_boost_gpu, touch_boost_gpu);
show_one(cpu_load_adder_at_max_gpu, cpu_load_adder_at_max_gpu);
show_one(cpu_load_adder_at_max_gpu_ignore_tb, cpu_load_adder_at_max_gpu_ignore_tb);
show_one(boost_hold_cycles, boost_hold_cycles);
show_one(disable_hotplug, disable_hotplug);
show_one(disable_hotplug_chrg, disable_hotplug_chrg);
show_one(disable_hotplug_media, disable_hotplug_media);
show_one(disable_hotplug_bt, disable_hotplug_bt);
show_one(no_extra_cores_screen_off, no_extra_cores_screen_off);
show_one(ignore_nice_load, ignore_nice);
show_one(freq_step_raise_screen_on, freq_step_raise_screen_on);
show_one(freq_step_raise_screen_off, freq_step_raise_screen_off);
show_one(freq_step_lower_screen_on, freq_step_lower_screen_on);
show_one(freq_step_lower_screen_off, freq_step_lower_screen_off);
show_one(debug_enabled, debug_enabled);

static ssize_t store_sampling_down_factor(struct kobject *a,
					  struct attribute *b,
					  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;

	dbs_tuners_ins.sampling_down_factor = input;
	return count;
}

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.sampling_rate = max(input, min_sampling_rate);
	stored_sampling_rate = max(input, min_sampling_rate);
	return count;
}

static ssize_t store_sampling_rate_screen_off(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.sampling_rate_screen_off = max(input, min_sampling_rate);
	return count;
}

static ssize_t store_up_threshold_screen_on(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > 100 ||
			input <= dbs_tuners_ins.down_threshold_screen_on)
		return -EINVAL;

	dbs_tuners_ins.up_threshold_screen_on = input;
	return count;
}

static ssize_t store_up_threshold_screen_on_hotplug_1(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > 100 ||
			input <= dbs_tuners_ins.down_threshold_screen_on_hotplug_1)
		return -EINVAL;

	dbs_tuners_ins.up_threshold_screen_on_hotplug_1 = input;
	if (screen_is_on)
		hotplug_cpu_enable_up[1] = input;
	return count;
}

static ssize_t store_up_threshold_screen_on_hotplug_2(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > 100 ||
			input <= dbs_tuners_ins.down_threshold_screen_on_hotplug_2)
		return -EINVAL;

	dbs_tuners_ins.up_threshold_screen_on_hotplug_2 = input;
	if (screen_is_on)
		hotplug_cpu_enable_up[2] = input;
	return count;
}

static ssize_t store_up_threshold_screen_on_hotplug_3(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > 100 ||
			input <= dbs_tuners_ins.down_threshold_screen_on_hotplug_3)
		return -EINVAL;

	dbs_tuners_ins.up_threshold_screen_on_hotplug_3 = input;
	if (screen_is_on)
		hotplug_cpu_enable_up[3] = input;
	return count;
}

static ssize_t store_up_threshold_screen_off(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > 100 ||
			input <= dbs_tuners_ins.down_threshold_screen_off)
		return -EINVAL;

	dbs_tuners_ins.up_threshold_screen_off = input;
	return count;
}

static ssize_t store_up_threshold_screen_off_hotplug_1(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > 100 ||
			input <= dbs_tuners_ins.down_threshold_screen_off_hotplug_1)
		return -EINVAL;

	dbs_tuners_ins.up_threshold_screen_off_hotplug_1 = input;
	if (!screen_is_on)
		hotplug_cpu_enable_up[1] = input;
	return count;
}

static ssize_t store_up_threshold_screen_off_hotplug_2(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > 100 ||
			input <= dbs_tuners_ins.down_threshold_screen_off_hotplug_2)
		return -EINVAL;

	dbs_tuners_ins.up_threshold_screen_off_hotplug_2 = input;
	if (!screen_is_on)
		hotplug_cpu_enable_up[2] = input;
	return count;
}

static ssize_t store_up_threshold_screen_off_hotplug_3(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > 100 ||
			input <= dbs_tuners_ins.down_threshold_screen_off_hotplug_3)
		return -EINVAL;

	dbs_tuners_ins.up_threshold_screen_off_hotplug_3 = input;
	if (!screen_is_on)
		hotplug_cpu_enable_up[3] = input;
	return count;
}

static ssize_t store_down_threshold_screen_on(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (ret != 1 || input < 11 || input > 100 ||
			input >= dbs_tuners_ins.up_threshold_screen_on)
		return -EINVAL;

	dbs_tuners_ins.down_threshold_screen_on = input;
	return count;
}

static ssize_t store_down_threshold_screen_on_hotplug_1(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (ret != 1 || input < 11 || input > 100 ||
			input >= dbs_tuners_ins.up_threshold_screen_on_hotplug_1)
		return -EINVAL;

	dbs_tuners_ins.down_threshold_screen_on_hotplug_1 = input;
	if (screen_is_on)
		hotplug_cpu_enable_down[1] = input;
	return count;
}

static ssize_t store_down_threshold_screen_on_hotplug_2(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (ret != 1 || input < 11 || input > 100 ||
			input >= dbs_tuners_ins.up_threshold_screen_on_hotplug_2)
		return -EINVAL;

	dbs_tuners_ins.down_threshold_screen_on_hotplug_2 = input;
	if (screen_is_on)
		hotplug_cpu_enable_down[2] = input;
	return count;
}

static ssize_t store_down_threshold_screen_on_hotplug_3(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (ret != 1 || input < 11 || input > 100 ||
			input >= dbs_tuners_ins.up_threshold_screen_on_hotplug_3)
		return -EINVAL;

	dbs_tuners_ins.down_threshold_screen_on_hotplug_3 = input;
	if (screen_is_on)
		hotplug_cpu_enable_down[3] = input;
	return count;
}

static ssize_t store_down_threshold_screen_off(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (ret != 1 || input < 11 || input > 100 ||
			input >= dbs_tuners_ins.up_threshold_screen_off)
		return -EINVAL;

	dbs_tuners_ins.down_threshold_screen_off = input;
	return count;
}

static ssize_t store_down_threshold_screen_off_hotplug_1(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (ret != 1 || input < 11 || input > 100 ||
			input >= dbs_tuners_ins.up_threshold_screen_off_hotplug_1)
		return -EINVAL;

	dbs_tuners_ins.down_threshold_screen_off_hotplug_1 = input;
	if (!screen_is_on)
		hotplug_cpu_enable_down[1] = input;
	return count;
}

static ssize_t store_down_threshold_screen_off_hotplug_2(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (ret != 1 || input < 11 || input > 100 ||
			input >= dbs_tuners_ins.up_threshold_screen_off_hotplug_2)
		return -EINVAL;

	dbs_tuners_ins.down_threshold_screen_off_hotplug_2 = input;
	if (!screen_is_on)
		hotplug_cpu_enable_down[2] = input;
	return count;
}

static ssize_t store_down_threshold_screen_off_hotplug_3(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (ret != 1 || input < 11 || input > 100 ||
			input >= dbs_tuners_ins.up_threshold_screen_off_hotplug_3)
		return -EINVAL;

	dbs_tuners_ins.down_threshold_screen_off_hotplug_3 = input;
	if (!screen_is_on)
		hotplug_cpu_enable_down[3] = input;
	return count;
}

static ssize_t store_block_cycles_online_screen_on(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (input < 0)
		return -EINVAL;

	dbs_tuners_ins.block_cycles_online_screen_on = input;
	return count;
}

static ssize_t store_block_cycles_offline_screen_on(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (input < 0)
		return -EINVAL;

	dbs_tuners_ins.block_cycles_offline_screen_on = input;
	return count;
}

static ssize_t store_block_cycles_raise_screen_on(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (input < 0)
		return -EINVAL;

	dbs_tuners_ins.block_cycles_raise_screen_on = input;
	return count;
}

static ssize_t store_block_cycles_online_screen_off(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (input < 0)
		return -EINVAL;

	dbs_tuners_ins.block_cycles_online_screen_off = input;
	return count;
}

static ssize_t store_block_cycles_offline_screen_off(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (input < 0)
		return -EINVAL;

	dbs_tuners_ins.block_cycles_offline_screen_off = input;
	return count;
}

static ssize_t store_block_cycles_raise_screen_off(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	/* cannot be lower than 11 otherwise freq will not fall */
	if (input < 0)
		return -EINVAL;

	dbs_tuners_ins.block_cycles_raise_screen_off = input;
	return count;
}

static ssize_t store_super_conservative_screen_on(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.super_conservative_screen_on = input;
	return count;
}

static ssize_t store_super_conservative_screen_off(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.super_conservative_screen_off = input;
	return count;
}

static ssize_t store_touch_boost_cpu(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > GLOBALKT_MAX_FREQ_LIMIT)
		input = GLOBALKT_MAX_FREQ_LIMIT;
	if (input < 0)
		input = 0;
	dbs_tuners_ins.touch_boost_cpu = input;
	return count;
}

static ssize_t store_touch_boost_cpu_all_cores(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret, i;

	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input != 0 && input != 1)
		input = 1;
	dbs_tuners_ins.touch_boost_cpu_all_cores = input;

	if (((screen_is_on && dbs_tuners_ins.sync_extra_cores_screen_on == 0) || (!screen_is_on && dbs_tuners_ins.sync_extra_cores_screen_off == 0)) && dbs_tuners_ins.touch_boost_cpu_all_cores == 0)
	{
		for (i = 0; i < CPUS_AVAILABLE; i++)
			kt_freq_control[i] = 0;
	}
	return count;
}

static ssize_t store_sync_extra_cores_screen_on(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret, i;

	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input != 0 && input != 1)
		input = 1;
	dbs_tuners_ins.sync_extra_cores_screen_on = input;
	
	if (screen_is_on && dbs_tuners_ins.sync_extra_cores_screen_on == 0 && dbs_tuners_ins.touch_boost_cpu_all_cores == 0)
	{
		for (i = 0; i < CPUS_AVAILABLE; i++)
			kt_freq_control[i] = 0;
	}
	return count;
}

static ssize_t store_sync_extra_cores_screen_off(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret, i;

	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input != 0 && input != 1)
		input = 1;
	dbs_tuners_ins.sync_extra_cores_screen_off = input;
	
	if (!screen_is_on && dbs_tuners_ins.sync_extra_cores_screen_off == 0 && dbs_tuners_ins.touch_boost_cpu_all_cores == 0)
	{
		for (i = 0; i < CPUS_AVAILABLE; i++)
			kt_freq_control[i] = 0;
	}
	return count;
}

static ssize_t store_touch_boost_2nd_core(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.touch_boost_2nd_core = input;
	return count;
}

static ssize_t store_touch_boost_3rd_core(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.touch_boost_3rd_core = input;
	return count;
}

static ssize_t store_touch_boost_4th_core(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.touch_boost_4th_core = input;
	return count;
}

static ssize_t store_lockout_2nd_core_hotplug_screen_on(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1 && input != 2)
		input = 0;

	dbs_tuners_ins.lockout_2nd_core_hotplug_screen_on = input;
	if (screen_is_on)
		hotplug_cpu_lockout[1] = input;
	if (screen_is_on && input == 1)
	{
		set_core_flag_up(1, 1);
		queue_work_on(0, dbs_wq, &hotplug_online_work);
	}
	else if (screen_is_on && input == 2 && !main_cpufreq_control[1])
	{
		set_core_flag_down(1, 1);
		queue_work_on(0, dbs_wq, &hotplug_offline_work);
	}
	return count;
}

static ssize_t store_lockout_3rd_core_hotplug_screen_on(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1 && input != 2)
		input = 0;

	dbs_tuners_ins.lockout_3rd_core_hotplug_screen_on = input;
	if (screen_is_on)
		hotplug_cpu_lockout[2] = input;
	if (screen_is_on && input == 1)
	{
		set_core_flag_up(2, 1);
		queue_work_on(0, dbs_wq, &hotplug_online_work);
	}
	else if (screen_is_on && input == 2 && !main_cpufreq_control[2])
	{
		set_core_flag_down(2, 1);
		queue_work_on(0, dbs_wq, &hotplug_offline_work);
	}
	return count;
}

static ssize_t store_lockout_4th_core_hotplug_screen_on(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1 && input != 2)
		input = 0;

	dbs_tuners_ins.lockout_4th_core_hotplug_screen_on = input;
	if (screen_is_on)
		hotplug_cpu_lockout[3] = input;
	if (screen_is_on && input == 1)
	{
		set_core_flag_up(3, 1);
		queue_work_on(0, dbs_wq, &hotplug_online_work);
	}
	else if (screen_is_on && input == 2 && !main_cpufreq_control[3])
	{
		set_core_flag_down(3, 1);
		queue_work_on(0, dbs_wq, &hotplug_offline_work);
	}
	return count;
}

static ssize_t store_lockout_2nd_core_hotplug_screen_off(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1 && input != 2)
		input = 0;

	dbs_tuners_ins.lockout_2nd_core_hotplug_screen_off = input;
	if (!screen_is_on)
		hotplug_cpu_lockout[1] = input;
	if (!screen_is_on && input == 1)
	{
		set_core_flag_up(1, 1);
		queue_work_on(0, dbs_wq, &hotplug_online_work);
	}
	else if (!screen_is_on && input == 2 && !main_cpufreq_control[1])
	{
		set_core_flag_down(1, 1);
		queue_work_on(0, dbs_wq, &hotplug_offline_work);
	}
	return count;
}

static ssize_t store_lockout_3rd_core_hotplug_screen_off(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1 && input != 2)
		input = 0;

	dbs_tuners_ins.lockout_3rd_core_hotplug_screen_off = input;
	if (!screen_is_on)
		hotplug_cpu_lockout[2] = input;
	if (!screen_is_on && input == 1)
	{
		set_core_flag_up(2, 1);
		queue_work_on(0, dbs_wq, &hotplug_online_work);
	}
	else if (!screen_is_on && input == 2 && !main_cpufreq_control[2])
	{
		set_core_flag_down(2, 1);
		queue_work_on(0, dbs_wq, &hotplug_offline_work);
	}
	return count;
}

static ssize_t store_lockout_4th_core_hotplug_screen_off(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1 && input != 2)
		input = 0;

	dbs_tuners_ins.lockout_4th_core_hotplug_screen_off = input;
	if (!screen_is_on)
		hotplug_cpu_lockout[3] = input;
	if (!screen_is_on && input == 1)
	{
		set_core_flag_up(3, 1);
		queue_work_on(0, dbs_wq, &hotplug_online_work);
	}
	else if (!screen_is_on && input == 2 && !main_cpufreq_control[3])
	{
		set_core_flag_down(3, 1);
		queue_work_on(0, dbs_wq, &hotplug_offline_work);
	}
	return count;
}

static ssize_t store_lockout_changes_when_boosting(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.lockout_changes_when_boosting = input;
	return count;
}

static ssize_t store_touch_boost_gpu(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 128000 && input != 200000 && input != 320000 && input != 389000 && input != 462400 && input != 578000 && input != 657500)
		input = 0;
	
	if (input == 0)
		boost_the_gpu(dbs_tuners_ins.touch_boost_gpu, false);
		
	dbs_tuners_ins.touch_boost_gpu = input;
	return count;
}

static ssize_t store_cpu_load_adder_at_max_gpu(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input < 0 || input > 100)
		input = 0;
	
	dbs_tuners_ins.cpu_load_adder_at_max_gpu = input;
	return count;
}

static ssize_t store_cpu_load_adder_at_max_gpu_ignore_tb(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input < 0 || input > 100)
		input = 0;
	
	dbs_tuners_ins.cpu_load_adder_at_max_gpu_ignore_tb = input;
	return count;
}

static ssize_t store_boost_hold_cycles(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input < 0)
		return -EINVAL;

	dbs_tuners_ins.boost_hold_cycles = input;
	return count;
}

static ssize_t store_disable_hotplug(struct kobject *a, struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret, cpu;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.disable_hotplug = input;
	if (input == 1)
	{
		for (cpu = 1; cpu < CPUS_AVAILABLE; cpu++)
			set_core_flag_up(cpu, 1);
		queue_work_on(0, dbs_wq, &hotplug_online_work);
	}
	return count;
}

static ssize_t store_disable_hotplug_chrg(struct kobject *a, struct attribute *b, const char *buf, size_t count)
{
	unsigned int input, c_state, c_stateW;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.disable_hotplug_chrg = input;
	c_state = is_charging;
	c_stateW = is_charging;

	if (c_state != 0 || c_stateW != 0)
		send_cable_state_kt(1);
	else
		send_cable_state_kt(0);
		
	return count;
}

static ssize_t store_disable_hotplug_media(struct kobject *a, struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.disable_hotplug_media = input;
		
	return count;
}

static ssize_t store_no_extra_cores_screen_off(struct kobject *a, struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.no_extra_cores_screen_off = input;
	return count;
}

static ssize_t store_boost_2nd_core_on_button_screen_on(struct kobject *a, struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.boost_2nd_core_on_button_screen_on = input;

	return count;
}

static ssize_t store_boost_3rd_core_on_button_screen_on(struct kobject *a, struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.boost_3rd_core_on_button_screen_on = input;

	return count;
}

static ssize_t store_boost_4th_core_on_button_screen_on(struct kobject *a, struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.boost_4th_core_on_button_screen_on = input;

	return count;
}

static ssize_t store_boost_2nd_core_on_button_screen_off(struct kobject *a, struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.boost_2nd_core_on_button_screen_off = input;

	return count;
}

static ssize_t store_boost_3rd_core_on_button_screen_off(struct kobject *a, struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.boost_3rd_core_on_button_screen_off = input;

	return count;
}

static ssize_t store_boost_4th_core_on_button_screen_off(struct kobject *a, struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.boost_4th_core_on_button_screen_off = input;

	return count;
}

static ssize_t store_disable_hotplug_bt(struct kobject *a, struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (input != 0 && input != 1)
		input = 0;

	dbs_tuners_ins.disable_hotplug_bt = input;
	return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_tuners_ins.ignore_nice) /* nothing to do */
		return count;

	dbs_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(cs_cpu_dbs_info, j);
		dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&dbs_info->prev_cpu_wall);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
	}
	return count;
}

static ssize_t store_freq_step_raise_screen_on(struct kobject *a, struct attribute *b,
			       const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 100)
		input = 100;

	/* no need to test here if freq_step_raise_screen_on is zero as the user might actually
	 * want this, they would be crazy though :) */
	dbs_tuners_ins.freq_step_raise_screen_on = input;
	return count;
}

static ssize_t store_freq_step_raise_screen_off(struct kobject *a, struct attribute *b,
			       const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 100)
		input = 100;

	/* no need to test here if freq_step_raise_screen_off is zero as the user might actually
	 * want this, they would be crazy though :) */
	dbs_tuners_ins.freq_step_raise_screen_off = input;
	return count;
}

static ssize_t store_freq_step_lower_screen_on(struct kobject *a, struct attribute *b,
			       const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 100)
		input = 100;

	/* no need to test here if freq_step_lower_screen_on is zero as the user might actually
	 * want this, they would be crazy though :) */
	dbs_tuners_ins.freq_step_lower_screen_on = input;
	return count;
}

static ssize_t store_freq_step_lower_screen_off(struct kobject *a, struct attribute *b,
			       const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 100)
		input = 100;

	/* no need to test here if freq_step_lower_screen_off is zero as the user might actually
	 * want this, they would be crazy though :) */
	dbs_tuners_ins.freq_step_lower_screen_off = input;
	return count;
}

static ssize_t store_debug_enabled(struct kobject *a, struct attribute *b,
			       const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.debug_enabled = input;
	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(sampling_rate_screen_off);
define_one_global_rw(sampling_down_factor);
define_one_global_rw(up_threshold_screen_on);
define_one_global_rw(up_threshold_screen_on_hotplug_1);
define_one_global_rw(up_threshold_screen_on_hotplug_2);
define_one_global_rw(up_threshold_screen_on_hotplug_3);
define_one_global_rw(up_threshold_screen_off);
define_one_global_rw(up_threshold_screen_off_hotplug_1);
define_one_global_rw(up_threshold_screen_off_hotplug_2);
define_one_global_rw(up_threshold_screen_off_hotplug_3);
define_one_global_rw(down_threshold_screen_on);
define_one_global_rw(down_threshold_screen_on_hotplug_1);
define_one_global_rw(down_threshold_screen_on_hotplug_2);
define_one_global_rw(down_threshold_screen_on_hotplug_3);
define_one_global_rw(down_threshold_screen_off);
define_one_global_rw(down_threshold_screen_off_hotplug_1);
define_one_global_rw(down_threshold_screen_off_hotplug_2);
define_one_global_rw(down_threshold_screen_off_hotplug_3);
define_one_global_rw(block_cycles_online_screen_on);
define_one_global_rw(block_cycles_offline_screen_on);
define_one_global_rw(block_cycles_raise_screen_on);
define_one_global_rw(block_cycles_online_screen_off);
define_one_global_rw(block_cycles_offline_screen_off);
define_one_global_rw(block_cycles_raise_screen_off);
define_one_global_rw(super_conservative_screen_on);
define_one_global_rw(super_conservative_screen_off);
define_one_global_rw(touch_boost_cpu);
define_one_global_rw(touch_boost_cpu_all_cores);
define_one_global_rw(touch_boost_2nd_core);
define_one_global_rw(touch_boost_3rd_core);
define_one_global_rw(touch_boost_4th_core);
define_one_global_rw(boost_2nd_core_on_button_screen_on);
define_one_global_rw(boost_3rd_core_on_button_screen_on);
define_one_global_rw(boost_4th_core_on_button_screen_on);
define_one_global_rw(boost_2nd_core_on_button_screen_off);
define_one_global_rw(boost_3rd_core_on_button_screen_off);
define_one_global_rw(boost_4th_core_on_button_screen_off);
define_one_global_rw(lockout_2nd_core_hotplug_screen_on);
define_one_global_rw(lockout_3rd_core_hotplug_screen_on);
define_one_global_rw(lockout_4th_core_hotplug_screen_on);
define_one_global_rw(lockout_2nd_core_hotplug_screen_off);
define_one_global_rw(lockout_3rd_core_hotplug_screen_off);
define_one_global_rw(lockout_4th_core_hotplug_screen_off);
define_one_global_rw(lockout_changes_when_boosting);
define_one_global_rw(touch_boost_gpu);
define_one_global_rw(cpu_load_adder_at_max_gpu);
define_one_global_rw(cpu_load_adder_at_max_gpu_ignore_tb);
define_one_global_rw(sync_extra_cores_screen_on);
define_one_global_rw(sync_extra_cores_screen_off);
define_one_global_rw(boost_hold_cycles);
define_one_global_rw(disable_hotplug);
define_one_global_rw(disable_hotplug_chrg);
define_one_global_rw(disable_hotplug_media);
define_one_global_rw(disable_hotplug_bt);
define_one_global_rw(no_extra_cores_screen_off);
define_one_global_rw(ignore_nice_load);
define_one_global_rw(freq_step_raise_screen_on);
define_one_global_rw(freq_step_raise_screen_off);
define_one_global_rw(freq_step_lower_screen_on);
define_one_global_rw(freq_step_lower_screen_off);
define_one_global_rw(debug_enabled);

static struct attribute *dbs_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&sampling_rate_screen_off.attr,
	&sampling_down_factor.attr,
	&up_threshold_screen_on.attr,
	&up_threshold_screen_on_hotplug_1.attr,
	&up_threshold_screen_on_hotplug_2.attr,
	&up_threshold_screen_on_hotplug_3.attr,
	&up_threshold_screen_off.attr,
	&up_threshold_screen_off_hotplug_1.attr,
	&up_threshold_screen_off_hotplug_2.attr,
	&up_threshold_screen_off_hotplug_3.attr,
	&down_threshold_screen_on.attr,
	&down_threshold_screen_on_hotplug_1.attr,
	&down_threshold_screen_on_hotplug_2.attr,
	&down_threshold_screen_on_hotplug_3.attr,
	&down_threshold_screen_off.attr,
	&down_threshold_screen_off_hotplug_1.attr,
	&down_threshold_screen_off_hotplug_2.attr,
	&down_threshold_screen_off_hotplug_3.attr,
	&block_cycles_online_screen_on.attr,
	&block_cycles_offline_screen_on.attr,
	&block_cycles_raise_screen_on.attr,
	&block_cycles_online_screen_off.attr,
	&block_cycles_offline_screen_off.attr,
	&block_cycles_raise_screen_off.attr,
	&super_conservative_screen_on.attr,
	&super_conservative_screen_off.attr,
	&touch_boost_cpu.attr,
	&touch_boost_cpu_all_cores.attr,
	&touch_boost_2nd_core.attr,
	&touch_boost_3rd_core.attr,
	&touch_boost_4th_core.attr,
	&boost_2nd_core_on_button_screen_on.attr,
	&boost_3rd_core_on_button_screen_on.attr,
	&boost_4th_core_on_button_screen_on.attr,
	&boost_2nd_core_on_button_screen_off.attr,
	&boost_3rd_core_on_button_screen_off.attr,
	&boost_4th_core_on_button_screen_off.attr,
	&lockout_2nd_core_hotplug_screen_on.attr,
	&lockout_3rd_core_hotplug_screen_on.attr,
	&lockout_4th_core_hotplug_screen_on.attr,
	&lockout_2nd_core_hotplug_screen_off.attr,
	&lockout_3rd_core_hotplug_screen_off.attr,
	&lockout_4th_core_hotplug_screen_off.attr,
	&lockout_changes_when_boosting.attr,
	&touch_boost_gpu.attr,
	&cpu_load_adder_at_max_gpu.attr,
	&cpu_load_adder_at_max_gpu_ignore_tb.attr,
	&sync_extra_cores_screen_on.attr,
	&sync_extra_cores_screen_off.attr,
	&boost_hold_cycles.attr,
	&disable_hotplug.attr,
	&disable_hotplug_chrg.attr,
	&disable_hotplug_media.attr,
	&disable_hotplug_bt.attr,
	&no_extra_cores_screen_off.attr,
	&ignore_nice_load.attr,
	&freq_step_raise_screen_on.attr,
	&freq_step_raise_screen_off.attr,
	&freq_step_lower_screen_on.attr,
	&freq_step_lower_screen_off.attr,
	&debug_enabled.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "ktoonservativeq",
};

/************************** sysfs end ************************/

static bool check_freq_increase(struct cpu_dbs_info_s *this_dbs_info, struct cpufreq_policy *policy, unsigned int max_load)
{
	unsigned int freq_target;
	if ((screen_is_on && max_load > dbs_tuners_ins.up_threshold_screen_on) || (!screen_is_on && max_load > dbs_tuners_ins.up_threshold_screen_off)) {
		if ((screen_is_on && this_dbs_info->Lblock_cycles_raise >= dbs_tuners_ins.block_cycles_raise_screen_on) || (!screen_is_on && this_dbs_info->Lblock_cycles_raise >= dbs_tuners_ins.block_cycles_raise_screen_off)) // || ((screen_is_on && dbs_tuners_ins.super_conservative_screen_on == 0) || call_in_progress) || ((!screen_is_on && dbs_tuners_ins.super_conservative_screen_off == 0) || call_in_progress))
		{
			if (screen_is_on)
				freq_target = (dbs_tuners_ins.freq_step_raise_screen_on * policy->max) / 100;
			else
				freq_target = (dbs_tuners_ins.freq_step_raise_screen_off * policy->max) / 100;

			/* max freq cannot be less than 100. But who knows.... */
			if (unlikely(freq_target == 0))
				freq_target = 5;

			this_dbs_info->requested_freq += freq_target;
			if (this_dbs_info->requested_freq > policy->max)
				this_dbs_info->requested_freq = policy->max;

			if ((!call_in_progress && screen_is_on && dbs_tuners_ins.super_conservative_screen_on) || (!call_in_progress && !screen_is_on && dbs_tuners_ins.super_conservative_screen_off))
				this_dbs_info->Lblock_cycles_raise = 0;
			return true;
		}
		if (this_dbs_info->Lblock_cycles_raise < 1000)
			this_dbs_info->Lblock_cycles_raise++;
	}
	else if ((!call_in_progress && screen_is_on && dbs_tuners_ins.super_conservative_screen_on) || (!call_in_progress && !screen_is_on && dbs_tuners_ins.super_conservative_screen_off))
		this_dbs_info->Lblock_cycles_raise = 0;

	return false;
}

static bool check_freq_decrease(struct cpu_dbs_info_s *this_dbs_info, struct cpufreq_policy *policy, unsigned int max_load)
{
	unsigned int freq_target;
	if (((screen_is_on && max_load < (dbs_tuners_ins.down_threshold_screen_on - 10)) || (!screen_is_on && max_load < (dbs_tuners_ins.down_threshold_screen_off - 10))))
	{
		if (screen_is_on)
			freq_target = (dbs_tuners_ins.freq_step_lower_screen_on * policy->max) / 100;
		else
			freq_target = (dbs_tuners_ins.freq_step_lower_screen_off * policy->max) / 100;

		this_dbs_info->requested_freq -= freq_target;
		if (this_dbs_info->requested_freq < policy->min)
			this_dbs_info->requested_freq = policy->min;

		/* if we cannot reduce the frequency anymore, break out early */
		if (policy->cur == policy->min && this_dbs_info->requested_freq == policy->min)
		{
			this_dbs_info->Lblock_cycles_raise = 0;
			return false;
		}
		return true;
	}
	return false;
}

static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int load = 0;
	unsigned int max_load = 0;
	unsigned int freq_target;
	int cpu;
	bool had_load_but_counting = false;
	struct cpufreq_policy *policy;
	unsigned int j;
	bool retInc;
	bool retDec;
	
	policy = this_dbs_info->cur_policy;

	/*
	 * Every sampling_rate, we check, if current idle time is less
	 * than 20% (default), then we try to increase frequency
	 * Every sampling_rate*sampling_down_factor, we check, if current
	 * idle time is more than 80%, then we try to decrease frequency
	 *
	 * Any frequency increase takes it to the maximum frequency.
	 * Frequency reduction happens at minimum steps of
	 * 5% (default) of maximum frequency
	 */

	/* Get Absolute Load */
	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_info_s *j_dbs_info;
		cputime64_t cur_wall_time, cur_idle_time;
		unsigned int idle_time, wall_time;
		j_dbs_info = &per_cpu(cs_cpu_dbs_info, j);
		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);
		wall_time = (unsigned int)
			(cur_wall_time - j_dbs_info->prev_cpu_wall);
		j_dbs_info->prev_cpu_wall = cur_wall_time;
		idle_time = (unsigned int)
			(cur_idle_time - j_dbs_info->prev_cpu_idle);
		j_dbs_info->prev_cpu_idle = cur_idle_time;
		if (dbs_tuners_ins.ignore_nice) {
			u64 cur_nice;
			unsigned long cur_nice_jiffies;
			cur_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE] -
					 j_dbs_info->prev_cpu_nice;
			cur_nice_jiffies = (unsigned long)
					cputime64_to_jiffies64(cur_nice);
			j_dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
			idle_time += jiffies_to_usecs(cur_nice_jiffies);
		}
		if (unlikely(!wall_time || wall_time < idle_time))
			continue;
		load = 100 * (wall_time - idle_time) / wall_time;
		if (load > max_load)
			max_load = load;
		if (dbs_tuners_ins.debug_enabled)
			pr_alert("CHECK LOAD : CPU=%d  LOAD=%d  Block Offline=%d", j, load, this_dbs_info->Lblock_cycles_offline);
		cpu_load[policy->cpu] = max_load;
	}
		
	//Check for block cycle overrides
	if (Lblock_cycles_offline_OVERRIDE[policy->cpu] != OVERRIDE_DISABLER)
	{
		this_dbs_info->Lblock_cycles_offline = Lblock_cycles_offline_OVERRIDE[policy->cpu];
		Lblock_cycles_offline_OVERRIDE[policy->cpu] = OVERRIDE_DISABLER;
	}
	if (Lblock_cycles_online_OVERRIDE[policy->cpu] != OVERRIDE_DISABLER)
	{
		this_dbs_info->Lblock_cycles_online = Lblock_cycles_online_OVERRIDE[policy->cpu];
		Lblock_cycles_online_OVERRIDE[policy->cpu] = OVERRIDE_DISABLER;
	}

	 //Adjust CPU load when GPU is maxed out
	if (dbs_tuners_ins.cpu_load_adder_at_max_gpu > 0)
	{
		if ((!boostpulse_relayf || (boostpulse_relayf && !dbs_tuners_ins.cpu_load_adder_at_max_gpu_ignore_tb)) && cur_gpu_step == cur_max_pwrlevel)
		{
			max_load += dbs_tuners_ins.cpu_load_adder_at_max_gpu;
			if (max_load > 100)
				max_load = 100;
		}
	}
	
	//Hotplugable CPU's only
	if (policy->cpu != 0)
	{
		//Remove up flag from cpu if it is already online
		if (cpu_online(policy->cpu) && hotplug_cpu_single_up[policy->cpu])
			set_core_flag_up(policy->cpu, 0);

		if (!boostpulse_relayf || (boostpulse_relayf && !dbs_tuners_ins.lockout_changes_when_boosting))
		{
			//Use CPU0 load if we are low just to keep things evened out
			if (max_load < cpu_load[0])
				max_load = cpu_load[0];
			//Check to see if we can take this CPU offline
			if (max_load <= hotplug_cpu_enable_down[policy->cpu] && hotplug_cpu_lockout[policy->cpu] != 1 && !dbs_tuners_ins.disable_hotplug && !disable_hotplug_chrg_override && !disable_hotplug_media_override && !disable_hotplug_bt_active)
			{
				//Make CPU's go offline in reverse order
				bool got_higher_online = false;
				if (policy->cpu < (CPUS_AVAILABLE - 1))
				{
					if (cpu_online(policy->cpu + 1))
						got_higher_online = true;
				}
			
				if (!main_cpufreq_control[policy->cpu] && !hotplug_cpu_single_down[policy->cpu] && (!boostpulse_relayf || (boostpulse_relayf && !hotplug_cpu_boosted[policy->cpu])))
				{
					if (!got_higher_online && ((screen_is_on && this_dbs_info->Lblock_cycles_offline > dbs_tuners_ins.block_cycles_offline_screen_on) || (!screen_is_on && this_dbs_info->Lblock_cycles_offline > dbs_tuners_ins.block_cycles_offline_screen_off)))
					{
						set_core_flag_down(policy->cpu, 1);
						set_core_flag_up(policy->cpu, 0);
						queue_work_on(policy->cpu, dbs_wq, &hotplug_offline_work);
						this_dbs_info->Lblock_cycles_offline = 0;
						return;
					}
					if (this_dbs_info->Lblock_cycles_offline < 1000)
						this_dbs_info->Lblock_cycles_offline++;
				}
			}
			else
			{
				if (this_dbs_info->Lblock_cycles_offline > 0)
					this_dbs_info->Lblock_cycles_offline--;
			}
			if (!((screen_is_on && (dbs_tuners_ins.sync_extra_cores_screen_on || (boostpulse_relayf && (dbs_tuners_ins.touch_boost_cpu_all_cores || dbs_tuners_ins.lockout_changes_when_boosting)))) || (!screen_is_on && dbs_tuners_ins.sync_extra_cores_screen_off)))
			{
				/* Check for frequency increase */
				retInc = check_freq_increase(this_dbs_info, policy, max_load);
				/* Check for frequency decrease */
				retDec = check_freq_decrease(this_dbs_info, policy, max_load);
				if (retInc || retDec)
				{
					__cpufreq_driver_target(policy, this_dbs_info->requested_freq, CPUFREQ_RELATION_H);
					return;
				}
			}
		}

		//Sync with CPU 0 when the sync flag is on
		if ((screen_is_on && (dbs_tuners_ins.sync_extra_cores_screen_on || (boostpulse_relayf && (dbs_tuners_ins.touch_boost_cpu_all_cores || dbs_tuners_ins.lockout_changes_when_boosting)))) || (!screen_is_on && dbs_tuners_ins.sync_extra_cores_screen_off))
		{
			freq_target = kt_freq_control[policy->cpu];
			this_dbs_info->requested_freq = freq_target;
			if (freq_target != policy->cur)
				__cpufreq_driver_target(policy, this_dbs_info->requested_freq, CPUFREQ_RELATION_H);
		}
		return;
	}
	
	if (dbs_tuners_ins.debug_enabled)
	{
		//pr_alert("LOAD=%d\n   CPUsonline=%d\n   CPUsonlineish=%d\n   CPU1flag=%d | %d | %d\n   CPU2flag=%d | %d | %d\n   CPU3flag=%d | %d | %d\n   KT Freq1=%d\n   KT Freq2=%d\n   KT Freq3=%d\n   Block Offline=%d\n"
		pr_alert("CPUsonline=%d\n   CPU0flag=%d | %d | %d\n   CPU1flag=%d | %d | %d\n   CPU2flag=%d | %d | %d\n   CPU3flag=%d | %d | %d\n   KT Freq1=%d\n   KT Freq2=%d\n   KT Freq3=%d\n"
			, cpus_online
			, hotplug_cpu_single_up[0], hotplug_cpu_single_down[0], cpu_load[0]
			, hotplug_cpu_single_up[1], hotplug_cpu_single_down[1], cpu_load[1]
			, hotplug_cpu_single_up[2], hotplug_cpu_single_down[2], cpu_load[2]
			, hotplug_cpu_single_up[3], hotplug_cpu_single_down[3], cpu_load[3]
			, kt_freq_control[1]
			, kt_freq_control[2]
			, kt_freq_control[3]);
	}
	
	//If we are in boost mode and user requests to lockout all changes during boost skip all frequency modification
	if (boostpulse_relayf && dbs_tuners_ins.lockout_changes_when_boosting)
		goto skip_it_all;

	if ((screen_is_on && dbs_tuners_ins.freq_step_raise_screen_on == 0) || (!screen_is_on && dbs_tuners_ins.freq_step_raise_screen_off == 0))
		return;
	
	//Check cpu online status
	if (!dbs_tuners_ins.no_extra_cores_screen_off || screen_is_on)
	{
		for (cpu = 1; cpu < CPUS_AVAILABLE; cpu++)
		{
			if (max_load >= hotplug_cpu_enable_up[cpu] && (!cpu_online(cpu)) && hotplug_cpu_lockout[cpu] != 2)
			{
				if (!hotplug_cpu_single_up[cpu] && ((screen_is_on && this_dbs_info->Lblock_cycles_online >= dbs_tuners_ins.block_cycles_online_screen_on) || (!screen_is_on && this_dbs_info->Lblock_cycles_online >= dbs_tuners_ins.block_cycles_online_screen_off)))
				{
					set_core_flag_up(cpu, 1);
					if (dbs_tuners_ins.debug_enabled)
						pr_alert("BOOST CORES %d - %d - %d - %d - %d", cpu, this_dbs_info->Lblock_cycles_online, hotplug_cpu_single_up[1], hotplug_cpu_single_up[2], hotplug_cpu_single_up[3]);
					set_core_flag_down(cpu, 0);
					hotplug_flag_on = true;
					this_dbs_info->Lblock_cycles_online = 0;
				}
				if (this_dbs_info->Lblock_cycles_online < 1000)
					this_dbs_info->Lblock_cycles_online++;
				had_load_but_counting = true;
				break;
			}
		}
	}
	/* Check to see if we set the hotplug_on flag to bring up more cores */
	if (hotplug_flag_on)
	{
		if (policy->cur > (policy->min * 2))
		{
			hotplug_flag_on = false;
			if ((screen_is_on && dbs_tuners_ins.sync_extra_cores_screen_on) || (!screen_is_on && dbs_tuners_ins.sync_extra_cores_screen_off))
				setExtraCores(policy->cur, false);
			queue_work_on(policy->cpu, dbs_wq, &hotplug_online_work);
		}
		else
		{
			for (cpu = 1; cpu < CPUS_AVAILABLE; cpu++)
				set_core_flag_up(cpu, 0);
		}
	}
	else if (!call_in_progress && ((screen_is_on && dbs_tuners_ins.super_conservative_screen_on) || (!screen_is_on && dbs_tuners_ins.super_conservative_screen_off)))
	{
		if (!had_load_but_counting)
			this_dbs_info->Lblock_cycles_online = 0;
	}
	
	/* Check for frequency increase */
	retInc = check_freq_increase(this_dbs_info, policy, max_load);
	/* Check for frequency decrease */
	retDec = check_freq_decrease(this_dbs_info, policy, max_load);
	if (!boostpulse_relayf && (retInc || retDec))
	{
		__cpufreq_driver_target(policy, this_dbs_info->requested_freq, CPUFREQ_RELATION_H);
		if (((screen_is_on && dbs_tuners_ins.sync_extra_cores_screen_on) || (!screen_is_on && dbs_tuners_ins.sync_extra_cores_screen_off)))
			setExtraCores(this_dbs_info->requested_freq, false);
		return;
	}
	
skip_it_all:
	//boost code
	if (boostpulse_relayf)
	{
		if (stored_sampling_rate != 0 && screen_is_on)
			dbs_tuners_ins.sampling_rate = stored_sampling_rate;
		
		//Boost is complete
		if (boost_hold_cycles_cnt >= dbs_tuners_ins.boost_hold_cycles)
		{
			boostpulse_relayf = false;
			boost_hold_cycles_cnt = 0;
			for (cpu = 0; cpu < CPUS_AVAILABLE; cpu++)
				hotplug_cpu_boosted[cpu] = 0;
			if ((screen_is_on && dbs_tuners_ins.sync_extra_cores_screen_on == 0) || (!screen_is_on && dbs_tuners_ins.sync_extra_cores_screen_off == 0))
			{
				for (cpu = 0; cpu < CPUS_AVAILABLE; cpu++)
					kt_freq_control[cpu] = 0;
			}
			boost_the_gpu(dbs_tuners_ins.touch_boost_gpu, false);
			if (turned_off_super_conservative_screen_off)
			{
				dbs_tuners_ins.super_conservative_screen_off = 1;
				turned_off_super_conservative_screen_off = false;
			}
			//pr_alert("BOOST ENDED: %d - %d - %d - %d", trmlpolicy[0].cur, trmlpolicy[1].cur, trmlpolicy[2].cur, trmlpolicy[3].cur);
			if (fake_screen_on)
			{
				//if (!screen_is_on)
				//	cpufreq_gov_suspend();
				fake_screen_on = false;
			}
			goto boostcomplete;
		}
		boost_hold_cycles_cnt++;
			
		if (dbs_tuners_ins.touch_boost_cpu_all_cores)
		{
			if (dbs_tuners_ins.lockout_changes_when_boosting || (dbs_tuners_ins.touch_boost_cpu > this_dbs_info->requested_freq && dbs_tuners_ins.touch_boost_cpu > policy->cur))
				setExtraCores(dbs_tuners_ins.touch_boost_cpu, false);
			else if (this_dbs_info->requested_freq > dbs_tuners_ins.touch_boost_cpu && this_dbs_info->requested_freq > policy->cur)
				setExtraCores(this_dbs_info->requested_freq, false);
			else
				setExtraCores(policy->cur, false);
		}
		if (dbs_tuners_ins.lockout_changes_when_boosting)
			this_dbs_info->requested_freq = dbs_tuners_ins.touch_boost_cpu;
			
		if (dbs_tuners_ins.touch_boost_cpu > this_dbs_info->requested_freq)
			this_dbs_info->requested_freq = dbs_tuners_ins.touch_boost_cpu;
		
		if (this_dbs_info->requested_freq != policy->cur)
			__cpufreq_driver_target(policy, this_dbs_info->requested_freq, CPUFREQ_RELATION_H);
boostcomplete:
		return;
	}

}

void setExtraCores(unsigned int requested_freq, bool isFirst)
{
	unsigned int cpu;
	for (cpu = 1; cpu < CPUS_AVAILABLE; cpu++)
	{
		if (!isFirst || (isFirst && requested_freq > kt_freq_control[cpu]))
			kt_freq_control[cpu] = requested_freq;
	}
}

void set_core_flag_up(unsigned int cpu, unsigned int val)
{
	spin_lock(&cpufreq_up_lock);
	hotplug_cpu_single_up[cpu] = val;
	spin_unlock(&cpufreq_up_lock);
}

void set_core_flag_down(unsigned int cpu, unsigned int val)
{
	spin_lock(&cpufreq_down_lock);
	hotplug_cpu_single_down[cpu] = val;
	spin_unlock(&cpufreq_down_lock);
}

void check_boost_cores_up(bool dec1, bool dec2, bool dec3)
{
	bool got_boost_core = false;

	if (!hotplug_cpu_single_up[1] && !cpu_online(1) && (dec1 || hotplug_cpu_lockout[1] == 1) && hotplug_cpu_lockout[1] != 2)
	{
		set_core_flag_up(1, 1);
		hotplug_cpu_boosted[1] = 1;
		got_boost_core = true;
	}
	if (!hotplug_cpu_single_up[2] && !cpu_online(2) && (dec2 || hotplug_cpu_lockout[2] == 1) && hotplug_cpu_lockout[2] != 2)
	{
		set_core_flag_up(2, 1);
		hotplug_cpu_boosted[2] = 1;
		got_boost_core = true;
	}
	if (!hotplug_cpu_single_up[3] && !cpu_online(3) && (dec3 || hotplug_cpu_lockout[3] == 1) && hotplug_cpu_lockout[3] != 2)
	{
		set_core_flag_up(3, 1);
		hotplug_cpu_boosted[3] = 1;
		got_boost_core = true;
	}
	if (got_boost_core)
	{
		if (dbs_tuners_ins.debug_enabled)
			pr_alert("CHECK BOOST CORES UP %d - %d - %d", hotplug_cpu_single_up[1], hotplug_cpu_single_up[2], hotplug_cpu_single_up[3]);
		queue_work_on(0, dbs_wq, &hotplug_online_work);
	}
}

void ktoonservative_screen_is_on(bool state)
{
	unsigned int need_to_queue = 0;
	unsigned int cpu;
	
	if (state == true)
	{
		//Set hotplug options when screen is on
		hotplug_cpu_enable_up[1] = dbs_tuners_ins.up_threshold_screen_on_hotplug_1;
		hotplug_cpu_enable_up[2] = dbs_tuners_ins.up_threshold_screen_on_hotplug_2;
		hotplug_cpu_enable_up[3] = dbs_tuners_ins.up_threshold_screen_on_hotplug_3;
		hotplug_cpu_enable_down[1] = dbs_tuners_ins.down_threshold_screen_on_hotplug_1;
		hotplug_cpu_enable_down[2] = dbs_tuners_ins.down_threshold_screen_on_hotplug_2;
		hotplug_cpu_enable_down[3] = dbs_tuners_ins.down_threshold_screen_on_hotplug_3;
	
		//Set core lockout options when screen is on
		hotplug_cpu_lockout[1] = dbs_tuners_ins.lockout_2nd_core_hotplug_screen_on;
		hotplug_cpu_lockout[2] = dbs_tuners_ins.lockout_3rd_core_hotplug_screen_on;
		hotplug_cpu_lockout[3] = dbs_tuners_ins.lockout_4th_core_hotplug_screen_on;
		for (cpu = 1; cpu < CPUS_AVAILABLE; cpu++)
		{
			if (hotplug_cpu_lockout[cpu] == 1)
				set_core_flag_up(cpu, 1);
			if (hotplug_cpu_lockout[cpu] == 2)
				set_core_flag_down(cpu, 1);
		}
			
		if (stored_sampling_rate > 0)
			dbs_tuners_ins.sampling_rate = stored_sampling_rate; //max(input, min_sampling_rate);
		ktoonservative_boostpulse(true);
	}
	else
	{
		//Set hotplug options when screen is off
		hotplug_cpu_enable_up[1] = dbs_tuners_ins.up_threshold_screen_off_hotplug_1;
		hotplug_cpu_enable_up[2] = dbs_tuners_ins.up_threshold_screen_off_hotplug_2;
		hotplug_cpu_enable_up[3] = dbs_tuners_ins.up_threshold_screen_off_hotplug_3;
		hotplug_cpu_enable_down[1] = dbs_tuners_ins.down_threshold_screen_off_hotplug_1;
		hotplug_cpu_enable_down[2] = dbs_tuners_ins.down_threshold_screen_off_hotplug_2;
		hotplug_cpu_enable_down[3] = dbs_tuners_ins.down_threshold_screen_off_hotplug_3;

		//Set core lockout options when screen is on
		hotplug_cpu_lockout[1] = dbs_tuners_ins.lockout_2nd_core_hotplug_screen_off;
		hotplug_cpu_lockout[2] = dbs_tuners_ins.lockout_3rd_core_hotplug_screen_off;
		hotplug_cpu_lockout[3] = dbs_tuners_ins.lockout_4th_core_hotplug_screen_off;
		for (cpu = 1; cpu < CPUS_AVAILABLE; cpu++)
		{
			if (hotplug_cpu_lockout[cpu] == 1 && (dbs_tuners_ins.no_extra_cores_screen_off == 0 || (dbs_tuners_ins.no_extra_cores_screen_off == 1 && screen_is_on)))
			{
				set_core_flag_up(cpu, 1);
				if (need_to_queue == 0)
					need_to_queue = 1;
				else
					need_to_queue = 3;
			}
			if (hotplug_cpu_lockout[cpu] == 2 && !main_cpufreq_control[cpu])
			{
				set_core_flag_down(cpu, 1);
				if (need_to_queue == 0)
					need_to_queue = 2;
				else
					need_to_queue = 3;
			}
		}
		if (need_to_queue == 1 || need_to_queue == 3)
		{
			queue_work_on(0, dbs_wq, &hotplug_online_work);
		}
		if (need_to_queue == 2 || need_to_queue == 3)
		{
			queue_work_on(0, dbs_wq, &hotplug_offline_work);
		}	

		boost_the_gpu(dbs_tuners_ins.touch_boost_gpu, false);
		stored_sampling_rate = dbs_tuners_ins.sampling_rate;
		dbs_tuners_ins.sampling_rate = dbs_tuners_ins.sampling_rate_screen_off;
	}
	screen_is_on = state;
}

void ktoonservative_boostpulse(bool boost_for_button)
{
	unsigned int cpu;
	if (!boostpulse_relayf)
	{
		if (dbs_tuners_ins.touch_boost_gpu > 0)  // && screen_is_on
		{
			boost_the_gpu(dbs_tuners_ins.touch_boost_gpu, true);
			boostpulse_relayf = true;
			boost_hold_cycles_cnt = 0;
		}
		
		if (dbs_tuners_ins.touch_boost_2nd_core || dbs_tuners_ins.touch_boost_3rd_core || dbs_tuners_ins.touch_boost_4th_core || dbs_tuners_ins.touch_boost_cpu)
		{
			if (boost_for_button)
			{
				if (screen_is_on)
					check_boost_cores_up(dbs_tuners_ins.boost_2nd_core_on_button_screen_on, dbs_tuners_ins.boost_3rd_core_on_button_screen_on, dbs_tuners_ins.boost_4th_core_on_button_screen_on);
				else
				{
					//cpufreq_gov_resume();
					fake_screen_on = true;
					if (dbs_tuners_ins.super_conservative_screen_off)
					{
						dbs_tuners_ins.super_conservative_screen_off = 0;
						turned_off_super_conservative_screen_off = true;
					}
					check_boost_cores_up(dbs_tuners_ins.boost_2nd_core_on_button_screen_off, dbs_tuners_ins.boost_3rd_core_on_button_screen_off, dbs_tuners_ins.boost_4th_core_on_button_screen_off);
				}
			}
			else
				check_boost_cores_up(dbs_tuners_ins.touch_boost_2nd_core, dbs_tuners_ins.touch_boost_3rd_core, dbs_tuners_ins.touch_boost_4th_core);
			boostpulse_relayf = true;
			boost_hold_cycles_cnt = 0;
		}
		if (screen_is_on)
		{
			for (cpu = 0; cpu < CPUS_AVAILABLE; cpu++)
			{
				if (Lblock_cycles_offline_OVERRIDE[cpu] > 0 || Lblock_cycles_offline_OVERRIDE[cpu] == OVERRIDE_DISABLER)
					Lblock_cycles_offline_OVERRIDE[cpu] = 0;
			}
		}
		else
		{
			for (cpu = 0; cpu < CPUS_AVAILABLE; cpu++)
			{
				if (Lblock_cycles_offline_OVERRIDE[cpu] > 0 || Lblock_cycles_offline_OVERRIDE[cpu] == OVERRIDE_DISABLER)
					Lblock_cycles_offline_OVERRIDE[cpu] = (dbs_tuners_ins.block_cycles_offline_screen_on * -1);
			}
		}
		for (cpu = 0; cpu < CPUS_AVAILABLE; cpu++)
		{
			if (Lblock_cycles_online_OVERRIDE[cpu] > 0 || Lblock_cycles_online_OVERRIDE[cpu] == OVERRIDE_DISABLER)
				Lblock_cycles_online_OVERRIDE[cpu] = 0;
		}
		
		setExtraCores(dbs_tuners_ins.touch_boost_cpu, true);
		//dbs_tuners_ins.sampling_rate = min_sampling_rate;
		if (dbs_tuners_ins.debug_enabled)
			pr_info("BOOSTPULSE RELAY KT - %d", screen_is_on);
	}
	else
	{
		boost_hold_cycles_cnt = 0;
		for (cpu = 0; cpu < CPUS_AVAILABLE; cpu++)
		{
			if (Lblock_cycles_offline_OVERRIDE[cpu] > 0 || Lblock_cycles_offline_OVERRIDE[cpu] == OVERRIDE_DISABLER)
				Lblock_cycles_offline_OVERRIDE[cpu] = 0;
		}
		if (dbs_tuners_ins.debug_enabled)
			pr_info("BOOSTPULSE RELAY KT RESET VALS- %d", screen_is_on);
	}
}

static void __cpuinit hotplug_offline_work_fn(struct work_struct *work)
{
	int cpu;
	for_each_online_cpu(cpu) {
		if (likely(cpu_online(cpu) && (cpu))) {
			if (hotplug_cpu_single_down[cpu] && !hotplug_cpu_single_up[cpu] && !main_cpufreq_control[cpu])
			{
				if (dbs_tuners_ins.debug_enabled)
					pr_alert("BOOST CORES DOWN WORK FUNC %d - %d - %d - %d", cpu, hotplug_cpu_single_down[1], hotplug_cpu_single_down[2], hotplug_cpu_single_down[3]);
				cpu_down(cpu);
				set_core_flag_down(cpu, 0);
			}
		}
		if (hotplug_cpu_single_up[cpu])
			set_core_flag_up(cpu, 0);
	}
	hotplugInProgress = false;
}

static void __cpuinit hotplug_online_work_fn(struct work_struct *work)
{
	int cpu;
	for_each_possible_cpu(cpu) {
		if (likely(!cpu_online(cpu) && (cpu))) {
			if (hotplug_cpu_single_up[cpu] && !hotplug_cpu_single_down[cpu])
			{
				if (dbs_tuners_ins.debug_enabled)
					pr_alert("BOOST CORES UP WORK FUNC %d - %d - %d - %d", cpu, hotplug_cpu_single_up[1], hotplug_cpu_single_up[2], hotplug_cpu_single_up[3]);
				cpu_up(cpu);
				set_core_flag_up(cpu, 0);
			}
		}
		if (hotplug_cpu_single_down[cpu])
			set_core_flag_down(cpu, 0);
	}
	hotplugInProgress = false;
}

static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	unsigned int cpu = dbs_info->cpu;

	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

	delay -= jiffies % delay;

	mutex_lock(&dbs_info->timer_mutex);

	dbs_check_cpu(dbs_info);

	if (dbs_info->enable)
		schedule_delayed_work_on(cpu, &dbs_info->work, delay);
		//queue_delayed_work_on(cpu, dbs_wq, &dbs_info->work, delay);
	mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
	delay -= jiffies % delay;

	dbs_info->enable = 1;
	//INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);
	//queue_delayed_work_on(dbs_info->cpu, dbs_wq, &dbs_info->work, delay);
	INIT_DELAYED_WORK(&dbs_info->work, do_dbs_timer);
	schedule_delayed_work_on(dbs_info->cpu, &dbs_info->work, delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	dbs_info->enable = 0;
	cancel_delayed_work_sync(&dbs_info->work);
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j, i;
	int rc;

	this_dbs_info = &per_cpu(cs_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		ktoonservative_is_active = true;
		
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&dbs_mutex);

		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;

			j_dbs_info = &per_cpu(cs_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall);
			
			if (dbs_tuners_ins.ignore_nice) {
				j_dbs_info->prev_cpu_nice =
						kcpustat_cpu(j).cpustat[CPUTIME_NICE];
			}
		}

		this_dbs_info->Lblock_cycles_online = 0;
		Lblock_cycles_online_OVERRIDE[cpu] = OVERRIDE_DISABLER;
		this_dbs_info->Lblock_cycles_offline = 0;
		Lblock_cycles_offline_OVERRIDE[cpu] = OVERRIDE_DISABLER;
		this_dbs_info->Lblock_cycles_raise = 0;
		
		this_dbs_info->cpu = cpu;
		this_dbs_info->down_skip = 0;
		this_dbs_info->requested_freq = policy->cur;

		mutex_init(&this_dbs_info->timer_mutex);
		cpus_online++;
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (cpus_online == 1) {
			unsigned int latency;

			prev_apenable = apget_enable_auto_hotplug();
			apenable_auto_hotplug(false);

			spin_lock_init(&cpufreq_up_lock);
			spin_lock_init(&cpufreq_down_lock);
			/* policy latency is in nS. Convert it to uS first */
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;

			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&dbs_mutex);
				return rc;
			}

			min_sampling_rate = (MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(10)) / 20;
			/* Bring kernel and HW constraints together */
			min_sampling_rate = max(min_sampling_rate,
					MIN_LATENCY_MULTIPLIER * latency);
			dbs_tuners_ins.sampling_rate = 35000;
				//max((min_sampling_rate * 20),
				    //latency * LATENCY_MULTIPLIER);

			cpufreq_register_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
		}
		mutex_unlock(&dbs_mutex);

		dbs_timer_init(this_dbs_info);

		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&dbs_mutex);
		cpus_online--;
		mutex_destroy(&this_dbs_info->timer_mutex);
		cpu_load[this_dbs_info->cur_policy->cpu] = -1;
		/*
		 * Stop the timerschedule work, when this governor
		 * is used for first time
		 */
		if (cpus_online == 0)
		{
			ktoonservative_is_active = false;
		
			apenable_auto_hotplug(prev_apenable);
		
			boost_the_gpu(dbs_tuners_ins.touch_boost_gpu, false);

			for (i = 0; i < CPUS_AVAILABLE; i++)
				kt_freq_control[i] = 0;
			cpufreq_unregister_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);
		}
		mutex_unlock(&dbs_mutex);

		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);
		if (policy->max < this_dbs_info->cur_policy->cur)
		{
			__cpufreq_driver_target(this_dbs_info->cur_policy, policy->max, CPUFREQ_RELATION_H);
		}
		else if (policy->min > this_dbs_info->cur_policy->cur)
		{
			__cpufreq_driver_target(this_dbs_info->cur_policy, policy->min, CPUFREQ_RELATION_L);
		}
		dbs_check_cpu(this_dbs_info);
		mutex_unlock(&this_dbs_info->timer_mutex);

		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_KTOONSERVATIVEQ
static
#endif
struct cpufreq_governor cpufreq_gov_ktoonservative = {
	.name			= "ktoonservativeq",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	dbs_wq = alloc_workqueue("ktoonservativeq_dbs_wq", WQ_HIGHPRI | WQ_UNBOUND, 0);
	if (!dbs_wq) {
		printk(KERN_ERR "Failed to create ktoonservativeq_dbs_wq workqueue\n");
		return -EFAULT;
	}

	INIT_WORK(&hotplug_offline_work, hotplug_offline_work_fn);
	INIT_WORK(&hotplug_online_work, hotplug_online_work_fn);
	mutex_init(&dbs_mutex);
	return cpufreq_register_governor(&cpufreq_gov_ktoonservative);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cancel_work_sync(&hotplug_offline_work);
	cancel_work_sync(&hotplug_online_work);
	cpufreq_unregister_governor(&cpufreq_gov_ktoonservative);
	destroy_workqueue(dbs_wq);
}

MODULE_AUTHOR("ktoonsez");
MODULE_DESCRIPTION("'cpufreq_ktoonservativeq' - A dynamic cpufreq governor for "
		"Low Latency Frequency Transition capable processors "
		"optimised for use in a battery environment");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_KTOONSERVATIVEQ
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
