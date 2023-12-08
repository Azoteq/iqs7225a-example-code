/*
* This file contains all the necessary settings for the IQS7225A and this file can
* be changed from the GUI or edited here
* File:   IQS7225A_init.h
* Author: Azoteq PTY Ltd
*/ 

#ifndef IQS7225A_INIT_H
#define IQS7225A_INIT_H

/* Change the Power Mode & System Settings */
/* Memory Map Position 0x2000 - 0x2003 */
#define CONTROL0                                 0x10
#define CONTROL1                                 0x25
#define EVENT_MASK0                              0x07
#define EVENT_MASK1                              0x03
#define I2C_WINDOW_TIMEOUT0                      0x0A
#define I2C_WINDOW_TIMEOUT1                      0x00
#define I2C_COMMS0                               0x00
#define I2C_COMMS1                               0x00

/* Change the Report Rates & Mode Timeouts */
/* Memory Map Position 0x2100 - 0x2107 */
#define ATI_MODE_TIMEOUT0                        0x02
#define ATI_MODE_TIMEOUT1                        0x00
#define ATI_MODE_REPORT_RATE0                    0x00
#define ATI_MODE_REPORT_RATE1                    0x00
#define NP_TIMEOUT0                              0x88
#define NP_TIMEOUT1                              0x13
#define NP_REPORT_RATE0                          0x00
#define NP_REPORT_RATE1                          0x00
#define LP_TIMEOUT0                              0x88
#define LP_TIMEOUT1                              0x13
#define LP_REPORT_RATE0                          0x14
#define LP_REPORT_RATE1                          0x00
#define ULP_TIMEOUT0                             0x10
#define ULP_TIMEOUT1                             0x27
#define ULP_REPORT_RATE0                         0xF4
#define ULP_REPORT_RATE1                         0x01

/* Change the LTA Overwrite */
/* Memory Map Position 0x1400 - 0x1405 */
#define LTA0_WRITE0                              0x8E
#define LTA0_WRITE1                              0x01
#define LTA1_WRITE0                              0x8F
#define LTA1_WRITE1                              0x01
#define LTA2_WRITE0                              0xFE
#define LTA2_WRITE1                              0x00
#define LTA3_WRITE0                              0xFE
#define LTA3_WRITE1                              0x00
#define LTA4_WRITE0                              0xFB
#define LTA4_WRITE1                              0x00
#define LTA5_WRITE0                              0xFD
#define LTA5_WRITE1                              0x00

/* Change the Count & LTA Filter Reseed */
/* Memory Map Position 0x2200 - 0x2205 */
#define CH0_COEFF_RESEED0                        0x00
#define CH0_COEFF_RESEED1                        0x00
#define CH1_COEFF_RESEED0                        0x00
#define CH1_COEFF_RESEED1                        0x00
#define CH2_COEFF_RESEED0                        0x00
#define CH2_COEFF_RESEED1                        0x00
#define CH3_COEFF_RESEED0                        0x00
#define CH3_COEFF_RESEED1                        0x00
#define CH4_COEFF_RESEED0                        0x00
#define CH4_COEFF_RESEED1                        0x00
#define CH5_COEFF_RESEED0                        0x00
#define CH5_COEFF_RESEED1                        0x00

/* Change the Cycle 0 Data */
/* Memory Map Position 0x3000 - 0x3002 */
#define CYCLE0_CONV_FREQ_FRAC                    0x7F
#define CYCLE0_CONV_FREQ_PERIOD                  0x01
#define CYCLE0_GEN_SET                           0x0A
#define CYCLE0_ENGINE_MODE                       0x03
#define CYCLE0_TX_SEL0                           0x01
#define CYCLE0_TX_SEL1                           0x00

/* Change the Cycle 1 Data */
/* Memory Map Position 0x3100 - 0x3102 */
#define CYCLE1_CONV_FREQ_FRAC                    0x7F
#define CYCLE1_CONV_FREQ_PERIOD                  0x01
#define CYCLE1_GEN_SET                           0x0A
#define CYCLE1_ENGINE_MODE                       0x03
#define CYCLE1_TX_SEL0                           0x00
#define CYCLE1_TX_SEL1                           0x02

/* Change the Cycle 2 Data */
/* Memory Map Position 0x3200 - 0x3202 */
#define CYCLE2_CONV_FREQ_FRAC                    0x7F
#define CYCLE2_CONV_FREQ_PERIOD                  0x01
#define CYCLE2_GEN_SET                           0x0A
#define CYCLE2_ENGINE_MODE                       0x03
#define CYCLE2_TX_SEL0                           0x08
#define CYCLE2_TX_SEL1                           0x00

/* Change the Cycle 3 Data */
/* Memory Map Position 0x3300 - 0x3302 */
#define CYCLE3_CONV_FREQ_FRAC                    0x7F
#define CYCLE3_CONV_FREQ_PERIOD                  0x01
#define CYCLE3_GEN_SET                           0x0A
#define CYCLE3_ENGINE_MODE                       0x03
#define CYCLE3_TX_SEL0                           0x00
#define CYCLE3_TX_SEL1                           0x01

/* Change the Cycle 4 Data */
/* Memory Map Position 0x3400 - 0x3402 */
#define CYCLE4_CONV_FREQ_FRAC                    0x7F
#define CYCLE4_CONV_FREQ_PERIOD                  0x01
#define CYCLE4_GEN_SET                           0x0A
#define CYCLE4_ENGINE_MODE                       0x03
#define CYCLE4_TX_SEL0                           0x00
#define CYCLE4_TX_SEL1                           0x08

/* Change the Cycle 5 Data */
/* Memory Map Position 0x3500 - 0x3502 */
#define CYCLE5_CONV_FREQ_FRAC                    0x7F
#define CYCLE5_CONV_FREQ_PERIOD                  0x01
#define CYCLE5_GEN_SET                           0x0A
#define CYCLE5_ENGINE_MODE                       0x03
#define CYCLE5_TX_SEL0                           0x00
#define CYCLE5_TX_SEL1                           0x04

/* Change the Cycle Channel Selection */
/* Memory Map Position 0x3600 - 0x3605 */
#define CYCLE0_ENGINE0                           0x00
#define CYCLE0_ENGINE1                           0xFF
#define CYCLE1_ENGINE0                           0xFF
#define CYCLE1_ENGINE1                           0x01
#define CYCLE2_ENGINE0                           0x02
#define CYCLE2_ENGINE1                           0xFF
#define CYCLE3_ENGINE0                           0xFF
#define CYCLE3_ENGINE1                           0x03
#define CYCLE4_ENGINE0                           0xFF
#define CYCLE4_ENGINE1                           0x04
#define CYCLE5_ENGINE0                           0xFF
#define CYCLE5_ENGINE1                           0x05

/* Change the Button 0 Settings */
/* Memory Map Position 0x4000 - 0x4008 */
#define BUTTON0_PROX_THRESHOLD                   0x0C
#define BUTTON0_PROX_HYSTERESIS                  0x12
#define BUTTON0_TOUCH_THRESHOLD                  0x19
#define BUTTON0_TOUCH_HYSTERESIS                 0x00
#define BUTTON0_DEEP_TOUCH_THRESHOLD             0x33
#define BUTTON0_DEEP_TOUCH_HYSTERESIS            0x00
#define BUTTON0_FILTER_HALT_TIMEOUT              0x08
#define BUTTON0_OTHER_TIMEOUT                    0x00
#define BUTTON0_GENERALS                         0x08
#define BUTTON0_BLOCK                            0xFF
#define BUTTON0_COUNT_BETA                       0x12
#define BUTTON0_LTA_BETA                         0x78
#define BUTTON0_LTA_BETA_FAST                    0x34
#define BUTTON0_LTA_BETA_FAST_BOUND              0x20
#define PROX_DELAY_0                             0x0C

/* Change the Button 1 Settings */
/* Memory Map Position 0x4100 - 0x4106 */
#define BUTTON1_PROX_THRESHOLD                   0x0C
#define BUTTON1_PROX_HYSTERESIS                  0x12
#define BUTTON1_TOUCH_THRESHOLD                  0x19
#define BUTTON1_TOUCH_HYSTERESIS                 0x00
#define BUTTON1_DEEP_TOUCH_THRESHOLD             0x33
#define BUTTON1_DEEP_TOUCH_HYSTERESIS            0x00
#define BUTTON1_FILTER_HALT_TIMEOUT              0x08
#define BUTTON1_OTHER_TIMEOUT                    0x00
#define BUTTON1_GENERALS                         0x08
#define BUTTON1_BLOCK                            0xFF
#define BUTTON1_COUNT_BETA                       0x12
#define BUTTON1_LTA_BETA                         0x78
#define BUTTON1_LTA_BETA_FAST                    0x34
#define BUTTON1_LTA_BETA_FAST_BOUND              0x20

/* Change the Button 2 Settings */
/* Memory Map Position 0x4200 - 0x4206 */
#define BUTTON2_PROX_THRESHOLD                   0x0F
#define BUTTON2_PROX_HYSTERESIS                  0x12
#define BUTTON2_TOUCH_THRESHOLD                  0x32
#define BUTTON2_TOUCH_HYSTERESIS                 0x00
#define BUTTON2_DEEP_TOUCH_THRESHOLD             0x96
#define BUTTON2_DEEP_TOUCH_HYSTERESIS            0x00
#define BUTTON2_FILTER_HALT_TIMEOUT              0x08
#define BUTTON2_OTHER_TIMEOUT                    0x28
#define BUTTON2_GENERALS                         0x0C
#define BUTTON2_BLOCK                            0xFF
#define BUTTON2_COUNT_BETA                       0x12
#define BUTTON2_LTA_BETA                         0x78
#define BUTTON2_LTA_BETA_FAST                    0x34
#define BUTTON2_LTA_BETA_FAST_BOUND              0x20

/* Change the Button 3 Settings */
/* Memory Map Position 0x4300 - 0x4306 */
#define BUTTON3_PROX_THRESHOLD                   0x0F
#define BUTTON3_PROX_HYSTERESIS                  0x12
#define BUTTON3_TOUCH_THRESHOLD                  0x32
#define BUTTON3_TOUCH_HYSTERESIS                 0x00
#define BUTTON3_DEEP_TOUCH_THRESHOLD             0x96
#define BUTTON3_DEEP_TOUCH_HYSTERESIS            0x00
#define BUTTON3_FILTER_HALT_TIMEOUT              0x08
#define BUTTON3_OTHER_TIMEOUT                    0x28
#define BUTTON3_GENERALS                         0x0C
#define BUTTON3_BLOCK                            0xFF
#define BUTTON3_COUNT_BETA                       0x12
#define BUTTON3_LTA_BETA                         0x78
#define BUTTON3_LTA_BETA_FAST                    0x34
#define BUTTON3_LTA_BETA_FAST_BOUND              0x20

/* Change the Button 4 Settings */
/* Memory Map Position 0x4400 - 0x4406 */
#define BUTTON4_PROX_THRESHOLD                   0x0F
#define BUTTON4_PROX_HYSTERESIS                  0x12
#define BUTTON4_TOUCH_THRESHOLD                  0x32
#define BUTTON4_TOUCH_HYSTERESIS                 0x00
#define BUTTON4_DEEP_TOUCH_THRESHOLD             0x96
#define BUTTON4_DEEP_TOUCH_HYSTERESIS            0x00
#define BUTTON4_FILTER_HALT_TIMEOUT              0x08
#define BUTTON4_OTHER_TIMEOUT                    0x28
#define BUTTON4_GENERALS                         0x0C
#define BUTTON4_BLOCK                            0xFF
#define BUTTON4_COUNT_BETA                       0x12
#define BUTTON4_LTA_BETA                         0x78
#define BUTTON4_LTA_BETA_FAST                    0x34
#define BUTTON4_LTA_BETA_FAST_BOUND              0x20

/* Change the Button 5 Settings */
/* Memory Map Position 0x4500 - 0x4506 */
#define BUTTON5_PROX_THRESHOLD                   0x0F
#define BUTTON5_PROX_HYSTERESIS                  0x12
#define BUTTON5_TOUCH_THRESHOLD                  0x32
#define BUTTON5_TOUCH_HYSTERESIS                 0x00
#define BUTTON5_DEEP_TOUCH_THRESHOLD             0x96
#define BUTTON5_DEEP_TOUCH_HYSTERESIS            0x00
#define BUTTON5_FILTER_HALT_TIMEOUT              0x08
#define BUTTON5_OTHER_TIMEOUT                    0x28
#define BUTTON5_GENERALS                         0x0C
#define BUTTON5_BLOCK                            0xFF
#define BUTTON5_COUNT_BETA                       0x12
#define BUTTON5_LTA_BETA                         0x78
#define BUTTON5_LTA_BETA_FAST                    0x34
#define BUTTON5_LTA_BETA_FAST_BOUND              0x20

/* Change the Sensor 0 Settings */
/* Memory Map Position 0x5000 - 0x5003 */
#define SENSOR0_CONVERSIONS                      0x43
#define SENSOR0_GENERALS                         0x15
#define SENSOR0_ATI_MODE_BASE                    0xCD
#define SENSOR0_ATI_TARGET                       0x32
#define SENSOR0_MULTIPLIERS_0                    0x48
#define SENSOR0_MULTIPLIERS_1                    0x20
#define SENSOR0_COMPENSATION_0                   0x15
#define SENSOR0_COMPENSATION_1                   0x78

/* Change the Sensor 1 Settings */
/* Memory Map Position 0x5100 - 0x5103 */
#define SENSOR1_CONVERSIONS                      0x13
#define SENSOR1_GENERALS                         0x15
#define SENSOR1_ATI_MODE_BASE                    0xCD
#define SENSOR1_ATI_TARGET                       0x32
#define SENSOR1_MULTIPLIERS_0                    0x48
#define SENSOR1_MULTIPLIERS_1                    0x20
#define SENSOR1_COMPENSATION_0                   0x06
#define SENSOR1_COMPENSATION_1                   0x78

/* Change the Sensor 2 Settings */
/* Memory Map Position 0x5200 - 0x5203 */
#define SENSOR2_CONVERSIONS                      0x23
#define SENSOR2_GENERALS                         0x15
#define SENSOR2_ATI_MODE_BASE                    0x85
#define SENSOR2_ATI_TARGET                       0x20
#define SENSOR2_MULTIPLIERS_0                    0xE3
#define SENSOR2_MULTIPLIERS_1                    0x20
#define SENSOR2_COMPENSATION_0                   0x14
#define SENSOR2_COMPENSATION_1                   0x78

/* Change the Sensor 3 Settings */
/* Memory Map Position 0x5300 - 0x5303 */
#define SENSOR3_CONVERSIONS                      0x23
#define SENSOR3_GENERALS                         0x15
#define SENSOR3_ATI_MODE_BASE                    0x85
#define SENSOR3_ATI_TARGET                       0x20
#define SENSOR3_MULTIPLIERS_0                    0xE2
#define SENSOR3_MULTIPLIERS_1                    0x08
#define SENSOR3_COMPENSATION_0                   0x1A
#define SENSOR3_COMPENSATION_1                   0x78

/* Change the Sensor 4 Settings */
/* Memory Map Position 0x5400 - 0x5403 */
#define SENSOR4_CONVERSIONS                      0x43
#define SENSOR4_GENERALS                         0x15
#define SENSOR4_ATI_MODE_BASE                    0x85
#define SENSOR4_ATI_TARGET                       0x20
#define SENSOR4_MULTIPLIERS_0                    0xE2
#define SENSOR4_MULTIPLIERS_1                    0x20
#define SENSOR4_COMPENSATION_0                   0x28
#define SENSOR4_COMPENSATION_1                   0x78

/* Change the Sensor 5 Settings */
/* Memory Map Position 0x5500 - 0x5503 */
#define SENSOR5_CONVERSIONS                      0x83
#define SENSOR5_GENERALS                         0x15
#define SENSOR5_ATI_MODE_BASE                    0x85
#define SENSOR5_ATI_TARGET                       0x20
#define SENSOR5_MULTIPLIERS_0                    0xE4
#define SENSOR5_MULTIPLIERS_1                    0x08
#define SENSOR5_COMPENSATION_0                   0x23
#define SENSOR5_COMPENSATION_1                   0x78

/* Change the Encoder Settings */
/* Memory Map Position 0x6000 - 0x6202 */
#define ROTATION_SEGMENTS_0                      0x08
#define ROTATION_SEGMENTS_1                      0x00
#define ROTATION_COIL_INDEX_A                    0x00
#define ROTATION_COIL_INDEX_A_BAR                0x32
#define ROTATION_COIL_THESHOLD_ENTER_A0          0x96
#define ROTATION_COIL_THESHOLD_ENTER_A1          0x00
#define ROTATION_COIL_THESHOLD_EXIT_A0           0x4B
#define ROTATION_COIL_THESHOLD_EXIT_A1           0x00
#define ROTATION_COIL_INDEX_B                    0x01
#define ROTATION_COIL_INDEX_B_BAR                0x32
#define ROTATION_COIL_THESHOLD_ENTER_B0          0x96
#define ROTATION_COIL_THESHOLD_ENTER_B1          0x00
#define ROTATION_COIL_THESHOLD_EXIT_B0           0x4B
#define ROTATION_COIL_THESHOLD_EXIT_B1           0x00

/* Change the Trigger Settings: Max Delta */
/* Memory Map Position 0x2300 - 0x2305 */
#define CH0_TRIGGER_SETTINGS_0_0                 0xE8
#define CH0_TRIGGER_SETTINGS_0_1                 0x03
#define CH1_TRIGGER_SETTINGS_0_0                 0xE8
#define CH1_TRIGGER_SETTINGS_0_1                 0x03
#define CH2_TRIGGER_SETTINGS_0_0                 0xE8
#define CH2_TRIGGER_SETTINGS_0_1                 0x03
#define CH3_TRIGGER_SETTINGS_0_0                 0xE8
#define CH3_TRIGGER_SETTINGS_0_1                 0x03
#define CH4_TRIGGER_SETTINGS_0_0                 0xE8
#define CH4_TRIGGER_SETTINGS_0_1                 0x03
#define CH5_TRIGGER_SETTINGS_0_0                 0xE8
#define CH5_TRIGGER_SETTINGS_0_1                 0x03

/* Change the Trigger Settings: Number of Threshold */
/* Memory Map Position 0x2400 - 0x2405 */
#define CH0_TRIGGER_SETTINGS_1_0                 0x0A
#define CH0_TRIGGER_SETTINGS_1_1                 0x00
#define CH1_TRIGGER_SETTINGS_1_0                 0x0A
#define CH1_TRIGGER_SETTINGS_1_1                 0x00
#define CH2_TRIGGER_SETTINGS_1_0                 0x0A
#define CH2_TRIGGER_SETTINGS_1_1                 0x00
#define CH3_TRIGGER_SETTINGS_1_0                 0x0A
#define CH3_TRIGGER_SETTINGS_1_1                 0x00
#define CH4_TRIGGER_SETTINGS_1_0                 0x0A
#define CH4_TRIGGER_SETTINGS_1_1                 0x00
#define CH5_TRIGGER_SETTINGS_1_0                 0x0A
#define CH5_TRIGGER_SETTINGS_1_1                 0x00

#endif	/* IQS7225A_INIT_H */