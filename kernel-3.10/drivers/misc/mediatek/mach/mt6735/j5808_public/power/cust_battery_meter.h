#ifndef _CUST_BATTERY_METER_H
#define _CUST_BATTERY_METER_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
//#define SOC_BY_AUXADC
#define SOC_BY_HW_FG
//#define HW_FG_FORCE_USE_SW_OCV
//#define SOC_BY_SW_FG

//#define CONFIG_DIS_CHECK_BATTERY
//#define FIXED_TBAT_25

/* ADC Channel Number */
#if 0
#define CUST_TABT_NUMBER 17
#define VBAT_CHANNEL_NUMBER      7
#define ISENSE_CHANNEL_NUMBER	 6
#define VCHARGER_CHANNEL_NUMBER  4
#define VBATTEMP_CHANNEL_NUMBER  5
#endif
/* ADC resistor  */
#define R_BAT_SENSE 4					
#define R_I_SENSE 4						
#define R_CHARGER_1 330
#define R_CHARGER_2 39

#define TEMPERATURE_T0             110
#define TEMPERATURE_T1             0
#define TEMPERATURE_T2             25
#define TEMPERATURE_T3             50
#define TEMPERATURE_T              255 // This should be fixed, never change the value

#define FG_METER_RESISTANCE 	0

#if 1//lisong 2015-2-28 [cust battery capacity for D5116(35M) project: ZCV of 2200mAh]start
/* Qmax for battery  */   
/*--------------------
fenghua_2200mAh_budaigangpian_1:
Q_MAX_POS_50:2249
Q_MAX_POS_25:2230
Q_MAX_POS_0:2194
Q_MAX_NEG_10:2100
Q_MAX_POS_50_H_CURRENT:2237
Q_MAX_POS_25_H_CURRENT:2220
Q_MAX_POS_0_H_CURRENT:2053
Q_MAX_NEG_10_H_CURRENT:1520

fenghua_2200mAh_budaigangpian_2:
Q_MAX_POS_50:2243
Q_MAX_POS_25:2227
Q_MAX_POS_0:2200
Q_MAX_NEG_10:2160
Q_MAX_POS_50_H_CURRENT:2232
Q_MAX_POS_25_H_CURRENT:2214
Q_MAX_POS_0_H_CURRENT:2044
Q_MAX_NEG_10_H_CURRENT:1260

weike_2200mAh_budaigangpian:
Q_MAX_POS_50:2299
Q_MAX_POS_25:2326
Q_MAX_POS_0:2226
Q_MAX_NEG_10:2002
Q_MAX_POS_50_H_CURRENT:2291
Q_MAX_POS_25_H_CURRENT:2306
Q_MAX_POS_0_H_CURRENT:1991
Q_MAX_NEG_10_H_CURRENT:1292

---------------------*/
/* Qmax for battery  */
#define Q_MAX_POS_50	2249
#define Q_MAX_POS_25	2230
#define Q_MAX_POS_0		2194
#define Q_MAX_NEG_10	2100
    
#define Q_MAX_POS_50_H_CURRENT	2237
#define Q_MAX_POS_25_H_CURRENT	2220
#define Q_MAX_POS_0_H_CURRENT	2053
#define Q_MAX_NEG_10_H_CURRENT	1520
#else
/* Qmax for battery  */
#define Q_MAX_POS_50	1523
#define Q_MAX_POS_25	1489
#define Q_MAX_POS_0		1272
#define Q_MAX_NEG_10	1189

#define Q_MAX_POS_50_H_CURRENT	1511
#define Q_MAX_POS_25_H_CURRENT	1462
#define Q_MAX_POS_0_H_CURRENT	  818
#define Q_MAX_NEG_10_H_CURRENT	149
#endif//lisong 2015-2-28 [cust battery capacity for D5116(35M) project: ZCV of 2200mAh]end

/* Discharge Percentage */
#define OAM_D5		 0		//  1 : D5,   0: D2


/* battery meter parameter */
#define CHANGE_TRACKING_POINT
#define CUST_TRACKING_POINT  1
#define CUST_R_SENSE         56
#define CUST_HW_CC 		    0
#define AGING_TUNING_VALUE   103
#define CUST_R_FG_OFFSET    0

#define OCV_BOARD_COMPESATE	0 //mV 
#define R_FG_BOARD_BASE		1000
#define R_FG_BOARD_SLOPE	1000 //slope
#define CAR_TUNE_VALUE		191 //1.00


/* HW Fuel gague  */
#define CURRENT_DETECT_R_FG	10  //1mA
#define MinErrorOffset       1000
#define FG_VBAT_AVERAGE_SIZE 18
#define R_FG_VALUE 			10 // mOhm, base is 20

#define CUST_POWERON_DELTA_CAPACITY_TOLRANCE	30
#define CUST_POWERON_LOW_CAPACITY_TOLRANCE		5
#define CUST_POWERON_MAX_VBAT_TOLRANCE			90
#define CUST_POWERON_DELTA_VBAT_TOLRANCE		30
#define CUST_POWERON_DELTA_HW_SW_OCV_CAPACITY_TOLRANCE	10


/* Disable Battery check for HQA */
#ifdef MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
#define FIXED_TBAT_25
#endif

/* Dynamic change wake up period of battery thread when suspend*/
#define VBAT_NORMAL_WAKEUP		3600		//3.6V
#define VBAT_LOW_POWER_WAKEUP		3500		//3.5v
#define NORMAL_WAKEUP_PERIOD		5400 		//90 * 60 = 90 min
#define LOW_POWER_WAKEUP_PERIOD		300		//5 * 60 = 5 min
#define CLOSE_POWEROFF_WAKEUP_PERIOD	30	//30 s

#define FG_BAT_INT
#define IS_BATTERY_REMOVE_BY_PMIC


#endif	//#ifndef _CUST_BATTERY_METER_H
