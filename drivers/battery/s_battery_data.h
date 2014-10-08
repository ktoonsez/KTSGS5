/*
 * battery_data.h
 * Samsung Mobile Battery data Header
 *
 *
 * Copyright (C) 2012 Samsung Electronics, Inc.
 *
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
#ifndef __SEC_BATTERY_DATA_H
#define __SEC_BATTERY_DATA_H __FILE__

static struct battery_data_t samsung_battery_data[] = {
	{
		.RCOMP0 = 0x5B,
		.RCOMP_charging = 0x5B,
		.temp_cohot = -1400,
		.temp_cocold = -4700,
		.is_using_model_data = true,
		.type_str = "SDI",
	}
};

#define CAPACITY_MAX			990
#define CAPACITY_MAX_MARGIN	50
#define CAPACITY_MIN			0

#if defined(CONFIG_MACH_SLTE_CMCC) || defined(CONFIG_MACH_SLTE_CTC) || \
	defined(CONFIG_MACH_SLTE_CU) || defined(CONFIG_MACH_SLTE_TD)
static sec_bat_adc_table_data_t temp_table[] = {
	{25859,	900},
	{26080,	850},
	{26366,	800},
	{26686,	750},
	{27030,	700},
	{27434,	650},
	{27912,	600},
	{28473,	550},
	{29121,	500},
	{29863,	450},
	{30692,	400},
	{31716,	350},
	{32619,	300},
	{33703,	250},
	{34827,	200},
	{35879,	150},
	{37055,	100},
	{38099,	50},
	{38997,	0},
	{39650,	-50},
	{40400,	-100},
	{41289,	-150},
	{41762,	-200},
	{42141,	-250},
	{42437,	-300},
};
#else
static sec_bat_adc_table_data_t temp_table[] = {
	{25859,	900},
	{26080,	850},
	{26366,	800},
	{26686,	750},
	{27030,	700},
	{27434,	650},
	{27912,	600},
	{28473,	550},
	{29121,	500},
	{29863,	450},
	{30692,	400},
	{31716,	350},
	{32619,	300},
	{33703,	250},
	{34827,	200},
	{35879,	150},
	{37055,	100},
	{38099,	50},
	{38997,	0},
	{39897,	-50},
	{40593,	-100},
	{41289,	-150},
	{41762,	-200},
	{42141,	-250},
	{42437,	-300},
};
#endif

#define TEMP_HIGHLIMIT_THRESHOLD_EVENT		800
#define TEMP_HIGHLIMIT_RECOVERY_EVENT		750
#define TEMP_HIGHLIMIT_THRESHOLD_NORMAL		800
#define TEMP_HIGHLIMIT_RECOVERY_NORMAL		750
#define TEMP_HIGHLIMIT_THRESHOLD_LPM		800
#define TEMP_HIGHLIMIT_RECOVERY_LPM			750

#if defined(CONFIG_MACH_SLTE_CMCC) || defined(CONFIG_MACH_SLTE_CTC) || \
	defined(CONFIG_MACH_SLTE_CU) || defined(CONFIG_MACH_SLTE_TD)
#define TEMP_HIGH_THRESHOLD_EVENT	620
#define TEMP_HIGH_RECOVERY_EVENT	460
#define TEMP_LOW_THRESHOLD_EVENT	-50
#define TEMP_LOW_RECOVERY_EVENT		0
#define TEMP_HIGH_THRESHOLD_NORMAL	600
#define TEMP_HIGH_RECOVERY_NORMAL	460
#define TEMP_LOW_THRESHOLD_NORMAL	-50
#define TEMP_LOW_RECOVERY_NORMAL	0
#define TEMP_HIGH_THRESHOLD_LPM		600
#define TEMP_HIGH_RECOVERY_LPM		460
#define TEMP_LOW_THRESHOLD_LPM		-50
#define TEMP_LOW_RECOVERY_LPM		0
#else
#define TEMP_HIGH_THRESHOLD_EVENT	620
#define TEMP_HIGH_RECOVERY_EVENT	470
#define TEMP_LOW_THRESHOLD_EVENT	-30
#define TEMP_LOW_RECOVERY_EVENT		10
#define TEMP_HIGH_THRESHOLD_NORMAL	535
#define TEMP_HIGH_RECOVERY_NORMAL	470
#define TEMP_LOW_THRESHOLD_NORMAL	-30
#define TEMP_LOW_RECOVERY_NORMAL	10
#define TEMP_HIGH_THRESHOLD_LPM		480
#define TEMP_HIGH_RECOVERY_LPM		465
#define TEMP_LOW_THRESHOLD_LPM		-40
#define TEMP_LOW_RECOVERY_LPM		0
#endif
#endif /* __SEC_BATTERY_DATA_H */
