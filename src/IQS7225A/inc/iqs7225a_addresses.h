/******************************************************************************
 *                                                                            *
 *                                                                            *
 *                                Copyright by                                *
 *                                                                            *
 *                              Azoteq (Pty) Ltd                              *
 *                          Republic of South Africa                          *
 *                                                                            *
 *                           Tel: +27(0)21 863 0033                           *
 *                           E-mail: info@azoteq.com                          *
 *                                                                            *
* =========================================================================== *
* Refer to IQS7225A datasheet for more information, available here:           *
* - https://www.azoteq.com/design/datasheets/                                 *
* =========================================================================== *
*                       IQS7225A - Registers & Memory Map                     *
******************************************************************************/

#ifndef __IQS7225A_ADDRESSES_H
#define __IQS7225A_ADDRESSES_H

/* Device Information - Read Only */

/* APPLICATION VERSION INFO: 0x0000 - 0x0004 */
#define IQS7225A_MM_PROD_NUM                          0x0000
#define IQS7225A_MM_MAJOR_VERSION_NUM                 0x0001
#define IQS7225A_MM_MINOR_VERSION_NUM                 0x0002
#define IQS7225A_MM_PATCH_NUM1                        0x0003
#define IQS7225A_MM_PATCH_NUM2                        0x0004

/* ROM VERSION INFO: 0x0100 - 0x0104 */
#define IQS7225A_MM_ROM_LIB_NUM                       0x0100
#define IQS7225A_MM_ROM_MAJOR_VERSION_NUM             0x0101
#define IQS7225A_MM_ROM_MINOR_VERSION_NUM             0x0102
#define IQS7225A_MM_ROM_PATCH_NUM1                    0x0103
#define IQS7225A_MM_ROM_PATCH_NUM2                    0x0104

/* DEVICE STATUS: 0x1000 - 0x100C */
#define IQS7225A_MM_SYSTEM_STATUS		                  0x1000
#define IQS7225A_MM_EVENTS                            0x1001
#define IQS7225A_MM_TIER0_1_STATES                    0x1002
#define IQS7225A_MM_TIER2_STATES                      0x1003
#define IQS7225A_MM_ENCODER_GRAY_STATES               0x1004
#define IQS7225A_MM_ENCODER_ANGLE                     0x1005
#define IQS7225A_MM_ENCODER_COUNTER                   0x1006
#define IQS7225A_MM_CH0_TRIGGER_LEVEL	                0x1007
#define IQS7225A_MM_CH1_TRIGGER_LEVEL                 0x1008
#define IQS7225A_MM_CH2_TRIGGER_LEVEL                 0x1009
#define IQS7225A_MM_CH3_TRIGGER_LEVEL                 0x100A
#define IQS7225A_MM_CH4_TRIGGER_LEVEL                 0x100B
#define IQS7225A_MM_CH5_TRIGGER_LEVEL                 0x100C

/* CHANNEL COUNTS: 0x1100 - 0x1105 */
#define IQS7225A_MM_CHANNEL_0_COUNTS                  0x1100
#define IQS7225A_MM_CHANNEL_1_COUNTS                  0x1101
#define IQS7225A_MM_CHANNEL_2_COUNTS                  0x1102
#define IQS7225A_MM_CHANNEL_3_COUNTS                  0x1103
#define IQS7225A_MM_CHANNEL_4_COUNTS                  0x1104
#define IQS7225A_MM_CHANNEL_5_COUNTS                  0x1105

/* CHANNEL LTA: 0x1200 - 0x1205 */
#define IQS7225A_MM_CHANNEL_0_LTA         	          0x1200
#define IQS7225A_MM_CHANNEL_1_LTA         	          0x1201
#define IQS7225A_MM_CHANNEL_2_LTA         	          0x1202
#define IQS7225A_MM_CHANNEL_3_LTA         	          0x1203
#define IQS7225A_MM_CHANNEL_4_LTA         	          0x1204
#define IQS7225A_MM_CHANNEL_5_LTA         	          0x1205

/* CHANNEL DELTA: 0x1300 - 0x1305 */
#define IQS7225A_MM_CHANNEL_0_DELTA                   0x1300
#define IQS7225A_MM_CHANNEL_1_DELTA                   0x1301
#define IQS7225A_MM_CHANNEL_2_DELTA                   0x1302
#define IQS7225A_MM_CHANNEL_3_DELTA                   0x1303
#define IQS7225A_MM_CHANNEL_4_DELTA                   0x1304
#define IQS7225A_MM_CHANNEL_5_DELTA                   0x1305

/* CHANNEL UNFILTERED COUNTS: 0x1400 - 0x1405 */
#define IQS7225A_MM_CHANNEL_0_LTA_OVERWRITE           0x1400
#define IQS7225A_MM_CHANNEL_1_LTA_OVERWRITE           0x1401
#define IQS7225A_MM_CHANNEL_2_LTA_OVERWRITE           0x1402
#define IQS7225A_MM_CHANNEL_3_LTA_OVERWRITE           0x1403
#define IQS7225A_MM_CHANNEL_4_LTA_OVERWRITE           0x1404
#define IQS7225A_MM_CHANNEL_5_LTA_OVERWRITE           0x1405

/* PMU & SYSTEM SETTINGS: 0x2000 - 0x2003 */
#define IQS7225A_MM_SYSTEM_CONTROL_SETTINGS           0x2000
#define IQS7225A_MM_EVENT_MASK                        0x2001
#define IQS7225A_MM_I2C_TIMEOUT                       0x2002
#define IQS7225A_MM_I2C_CONFIG       	                0x2003

/* REPORT RATES & TIMEOUTS: 0x2100 - 0x2107 */
#define IQS7225A_MM_ATI_ERROR_TIMEOUT                 0x2100
#define IQS7225A_MM_ATI_REPORT_RATE        	          0x2101
#define IQS7225A_MM_NP_TIMEOUT                        0x2102
#define IQS7225A_MM_NP_REPORT_RATE                    0x2103
#define IQS7225A_MM_LP_TIMEOUT                        0x2104
#define IQS7225A_MM_LP_REPORT_RATE       	            0x2105
#define IQS7225A_MM_ULP_TIMEOUT   		                0x2106
#define IQS7225A_MM_ULP_REPORT_RATE         	        0x2107

/* CHANNEL RESEED COMMAND: 0x2200 - 0x2205 */
#define IQS7225A_MM_CH0_RESEED                        0x2200
#define IQS7225A_MM_CH1_RESEED            	          0x2201
#define IQS7225A_MM_CH2_RESEED                        0x2202
#define IQS7225A_MM_CH3_RESEED                        0x2203
#define IQS7225A_MM_CH4_RESEED                        0x2204
#define IQS7225A_MM_CH5_RESEED          	            0x2205

/* CHANNEL MAX DELTA: 0x2300 - 0x2305 */
#define IQS7225A_MM_CH0_MAX_DELTA                     0x2300
#define IQS7225A_MM_CH1_MAX_DELTA            	        0x2301
#define IQS7225A_MM_CH2_MAX_DELTA                     0x2302
#define IQS7225A_MM_CH3_MAX_DELTA                     0x2303
#define IQS7225A_MM_CH4_MAX_DELTA                     0x2304
#define IQS7225A_MM_CH5_MAX_DELTA          	          0x2305

/* RESEED COMMAND: 0x2400 - 0x2405 */
#define IQS7225A_MM_CH0_NO_OF_THRESHOLDS              0x2400
#define IQS7225A_MM_CH1_NO_OF_THRESHOLDS              0x2401
#define IQS7225A_MM_CH2_NO_OF_THRESHOLDS              0x2402
#define IQS7225A_MM_CH3_NO_OF_THRESHOLDS              0x2403
#define IQS7225A_MM_CH4_NO_OF_THRESHOLDS              0x2404
#define IQS7225A_MM_CH5_NO_OF_THRESHOLDS              0x2405

/* CYCLE SETUP 0: 0x3000 - 0x3500 */
#define IQS7225A_MM_CYCLE_0_SETUP_0                   0x3000
#define IQS7225A_MM_CYCLE_1_SETUP_0                   0x3100
#define IQS7225A_MM_CYCLE_2_SETUP_0                   0x3200
#define IQS7225A_MM_CYCLE_3_SETUP_0                   0x3300
#define IQS7225A_MM_CYCLE_4_SETUP_0                   0x3400
#define IQS7225A_MM_CYCLE_5_SETUP_0                   0x3500

/* CYCLE SETUP 1: 0x3001 - 0x3501 */
#define IQS7225A_MM_CYCLE_0_SETUP_1                   0x3001
#define IQS7225A_MM_CYCLE_1_SETUP_1                   0x3101
#define IQS7225A_MM_CYCLE_2_SETUP_1                   0x3201
#define IQS7225A_MM_CYCLE_3_SETUP_1                   0x3301
#define IQS7225A_MM_CYCLE_4_SETUP_1                   0x3401
#define IQS7225A_MM_CYCLE_5_SETUP_1                   0x3501

/* CYCLE SETUP 2: 0x3002 - 0x3502 */
#define IQS7225A_MM_CYCLE_0_SETUP_2                   0x3002
#define IQS7225A_MM_CYCLE_1_SETUP_2                   0x3102
#define IQS7225A_MM_CYCLE_2_SETUP_2                   0x3202
#define IQS7225A_MM_CYCLE_3_SETUP_2                   0x3302
#define IQS7225A_MM_CYCLE_4_SETUP_2                   0x3402
#define IQS7225A_MM_CYCLE_5_SETUP_2                   0x3502

/* ENGINE CHANNEL SELECT: 0x3600 - 0x3605 */
#define IQS7225A_MM_CYCLE_0_ENGINE_CHANNEL_SELECT     0x3600
#define IQS7225A_MM_CYCLE_1_ENGINE_CHANNEL_SELECT     0x3601
#define IQS7225A_MM_CYCLE_2_ENGINE_CHANNEL_SELECT     0x3602
#define IQS7225A_MM_CYCLE_3_ENGINE_CHANNEL_SELECT     0x3603
#define IQS7225A_MM_CYCLE_4_ENGINE_CHANNEL_SELECT     0x3604
#define IQS7225A_MM_CYCLE_5_ENGINE_CHANNEL_SELECT     0x3605

/* BUTTON SETUP */
/* CHANNEL 0: 0x4000 - 0x4006 */
#define IQS7225A_MM_CH_0_TIER_0_DETECTION             0x4000
#define IQS7225A_MM_CH_0_TIER_1_DETECTION             0x4001
#define IQS7225A_MM_CH_0_TIER_2_DETECTION             0x4002
#define IQS7225A_MM_CH_0_TIER_TIMEOUTS                0x4003
#define IQS7225A_MM_CH_0_GENERAL_BUTTON_SETTINGS      0x4004
#define IQS7225A_MM_CH_0_BETA_FILTERS                 0x4005
#define IQS7225A_MM_CH_0_FAST_BETA_FILTERS            0x4006

/* CHANNEL 1: 0x4100 - 0x4106 */
#define IQS7225A_MM_CH_1_TIER_0_DETECTION             0x4100
#define IQS7225A_MM_CH_1_TIER_1_DETECTION             0x4101
#define IQS7225A_MM_CH_1_TIER_2_DETECTION             0x4102
#define IQS7225A_MM_CH_1_TIER_TIMEOUTS                0x4103
#define IQS7225A_MM_CH_1_GENERAL_BUTTON_SETTINGS      0x4104
#define IQS7225A_MM_CH_1_BETA_FILTERS                 0x4105
#define IQS7225A_MM_CH_1_FAST_BETA_FILTERS            0x4106

/* CHANNEL 2: 0x4200 - 0x4206 */
#define IQS7225A_MM_CH_2_TIER_0_DETECTION             0x4200
#define IQS7225A_MM_CH_2_TIER_1_DETECTION             0x4201
#define IQS7225A_MM_CH_2_TIER_2_DETECTION             0x4202
#define IQS7225A_MM_CH_2_TIER_TIMEOUTS                0x4203
#define IQS7225A_MM_CH_2_GENERAL_BUTTON_SETTINGS      0x4204
#define IQS7225A_MM_CH_2_BETA_FILTERS                 0x4205
#define IQS7225A_MM_CH_2_FAST_BETA_FILTERS            0x4206

/* CHANNEL 3: 0x4300 - 0x4306 */
#define IQS7225A_MM_CH_3_TIER_0_DETECTION             0x4300
#define IQS7225A_MM_CH_3_TIER_1_DETECTION             0x4301
#define IQS7225A_MM_CH_3_TIER_2_DETECTION             0x4302
#define IQS7225A_MM_CH_3_TIER_TIMEOUTS                0x4303
#define IQS7225A_MM_CH_3_GENERAL_BUTTON_SETTINGS      0x4304
#define IQS7225A_MM_CH_3_BETA_FILTERS                 0x4305
#define IQS7225A_MM_CH_3_FAST_BETA_FILTERS            0x4306

/* CHANNEL 4: 0x4400 - 0x4406 */
#define IQS7225A_MM_CH_4_TIER_0_DETECTION             0x4400
#define IQS7225A_MM_CH_4_TIER_1_DETECTION             0x4401
#define IQS7225A_MM_CH_4_TIER_2_DETECTION             0x4402
#define IQS7225A_MM_CH_4_TIER_TIMEOUTS                0x4403
#define IQS7225A_MM_CH_4_GENERAL_BUTTON_SETTINGS      0x4404
#define IQS7225A_MM_CH_4_BETA_FILTERS                 0x4405
#define IQS7225A_MM_CH_4_FAST_BETA_FILTERS            0x4406

/* CHANNEL 5: 0x4500 - 0x4506 */
#define IQS7225A_MM_CH_5_TIER_1_DETECTION             0x4501
#define IQS7225A_MM_CH_5_TIER_2_DETECTION             0x4502
#define IQS7225A_MM_CH_5_TIER_TIMEOUTS                0x4503
#define IQS7225A_MM_CH_5_GENERAL_BUTTON_SETTINGS      0x4504
#define IQS7225A_MM_CH_5_BETA_FILTERS                 0x4505
#define IQS7225A_MM_CH_5_FAST_BETA_FILTERS            0x4506

/* SENSOR SETUP */
/* CHANNEL 0: 0x5000 - 0x5003 */
#define IQS7225A_MM_CH_0_GENERAL_CHANNEL_SETTINGS     0x5000
#define IQS7225A_MM_CH_0_ATI_BASE_TARGET              0x5001
#define IQS7225A_MM_CH_0_ATI_MIRRORS                  0x5002
#define IQS7225A_MM_CH_0_ATI_COMPENSATION             0x5003

/* CHANNEL 1: 0x5100 - 0x5103 */
#define IQS7225A_MM_CH_1_GENERAL_CHANNEL_SETTINGS     0x5100
#define IQS7225A_MM_CH_1_ATI_BASE_TARGET              0x5101
#define IQS7225A_MM_CH_1_ATI_MIRRORS                  0x5102
#define IQS7225A_MM_CH_1_ATI_COMPENSATION             0x5103

/* CHANNEL 2: 0x5200 - 0x5203 */
#define IQS7225A_MM_CH_2_GENERAL_CHANNEL_SETTINGS     0x5200
#define IQS7225A_MM_CH_2_ATI_BASE_TARGET              0x5201
#define IQS7225A_MM_CH_2_ATI_MIRRORS                  0x5202
#define IQS7225A_MM_CH_2_ATI_COMPENSATION             0x5203

/* CHANNEL 3: 0x5300 - 0x5303 */
#define IQS7225A_MM_CH_3_GENERAL_CHANNEL_SETTINGS     0x5300
#define IQS7225A_MM_CH_3_ATI_BASE_TARGET              0x5301
#define IQS7225A_MM_CH_3_ATI_MIRRORS                  0x5302
#define IQS7225A_MM_CH_3_ATI_COMPENSATION             0x5303

/* CHANNEL 4: 0x5400 - 0x5403 */
#define IQS7225A_MM_CH_4_GENERAL_CHANNEL_SETTINGS     0x5400
#define IQS7225A_MM_CH_4_ATI_BASE_TARGET              0x5401
#define IQS7225A_MM_CH_4_ATI_MIRRORS                  0x5402
#define IQS7225A_MM_CH_4_ATI_COMPENSATION             0x5403

/* CHANNEL 5: 0x5500 - 0x5503 */
#define IQS7225A_MM_CH_5_GENERAL_CHANNEL_SETTINGS     0x5500
#define IQS7225A_MM_CH_5_ATI_BASE_TARGET              0x5501
#define IQS7225A_MM_CH_5_ATI_MIRRORS                  0x5502
#define IQS7225A_MM_CH_5_ATI_COMPENSATION             0x5503

/* ROTATIONAL ENCODER SETUP: 0x6000 - 0x6202 */
#define IQS7225A_MM_NUMBER_OF_METAL_SEGMENTS          0x6000
#define IQS7225A_MM_COIL_A_CHANNEL_FIXED_REF          0x6100
#define IQS7225A_MM_COIL_A_ENTER_THRESHOLD            0x6102
#define IQS7225A_MM_COIL_A_EXIT_THRESHOLD             0x6103
#define IQS7225A_MM_COIL_B_CHANNEL_FIXED_REF          0x6200
#define IQS7225A_MM_COIL_B_ENTER_THRESHOLD            0x6201
#define IQS7225A_MM_COIL_B_EXIT_THRESHOLD             0x6202

/* TRIM: 0x6300 */
#define IQS7225A_MM_TRIMS                             0x6300
  
#endif /* __IQS7225A_ADDRESSES_H */
