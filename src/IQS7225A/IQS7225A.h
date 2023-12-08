/******************************************************************************
  * @file    IQS7225A.h
  * @brief   This file contains the header information for an IQS7225A Arduino 
  *          library. The goal of the library is to provide easy functionality 
  *          for initializing and using the Azoteq IQS7225A capacitive touch 
  *          device.
  * @author  Azoteq PTY Ltd
  * @version V1.1
  * @date    2023
  *****************************************************************************
  *****************************************************************************
  * @attention  Makes use of the following standard Arduino libraries:
  *       - Arduino.h   -> included in IQS7225A.h, comes standard with Arduino
  *       - Wire.h      -> Included in IQS7225A.h, comes standard with Arduino
  ****************************************************************************/

#ifndef IQS7225A_h
#define IQS7225A_h

// Include Files
#include "Arduino.h"
#include "Wire.h"
#include "./inc/IQS7225a_addresses.h"

// Public Global Definitions
/* For use with Wire.h library. True argument with some functions closes the 
I2C communication window. */
#define STOP    true    
/* For use with Wire.h library. False argument with some functions keeps the 
I2C communication window open. */
#define RESTART false   

// Misc Defines
#define IQS7225A_ROT_ENC_PCB_CH_NUM     6 // The amount of channels used on this EV-KIT PCB

// Device Info
#define IQS7225A_PRODUCT_NUM            0x0317

// System Status Bits.
#define IQS7225A_ATI_ACTIVE_BIT		0
#define IQS7225A_ATI_ERROR_BIT		1
#define IQS7225A_PROX_DEBOUNCE_BIT	2
#define IQS7225A_SHOW_RESET_BIT		3
#define IQS7225A_POWER_EVENT_BIT_0      4
#define IQS7225A_POWER_EVENT_BIT_1      5
#define IQS7225A_NORMAL_POWER_BITS      0b00
#define IQS7225A_LOW_POWER_BITS         0b01
#define IQS7225A_ULP_BITS               0b10
#define IQS7225A_GLOBAL_HALT_BIT	7

// Event Bits 
#define PROX_EVENT_BIT                  0
#define TOUCH_EVENT_BIT                 1
#define DEEP_TOUCH_EVENT_BIT            2
#define ATI_EVENT_BIT                   0       // 8      
#define CH0_TRIGGER_EVENT_BIT           1       // 9  
#define CH1_TRIGGER_EVENT_BIT           2       // 10 
#define CH2_TRIGGER_EVENT_BIT           3       // 11 
#define CH3_TRIGGER_EVENT_BIT           4       // 12 
#define CH4_TRIGGER_EVENT_BIT           5       // 13 
#define CH5_TRIGGER_EVENT_BIT           6       // 14 
#define CH6_TRIGGER_EVENT_BIT           7       // 15 

// Touch and Proximity Bits 
#define IQS7225A_CH0_PROX_BIT           0
#define IQS7225A_CH1_PROX_BIT           1
#define IQS7225A_CH2_PROX_BIT           2
#define IQS7225A_CH3_PROX_BIT           3
#define IQS7225A_CH4_PROX_BIT           4
#define IQS7225A_CH5_PROX_BIT           5
#define IQS7225A_CH0_TOUCH_BIT          0 // 8
#define IQS7225A_CH1_TOUCH_BIT          1 // 9
#define IQS7225A_CH2_TOUCH_BIT          2 // 10
#define IQS7225A_CH3_TOUCH_BIT          3 // 11
#define IQS7225A_CH4_TOUCH_BIT          4 // 12
#define IQS7225A_CH5_TOUCH_BIT          5 // 13

// Deep Touch Bits
#define IQS7225A_CH0_DEEP_TOUCH_BIT      0 
#define IQS7225A_CH1_DEEP_TOUCH_BIT      1 
#define IQS7225A_CH2_DEEP_TOUCH_BIT      2
#define IQS7225A_CH3_DEEP_TOUCH_BIT      3
#define IQS7225A_CH4_DEEP_TOUCH_BIT      4
#define IQS7225A_CH5_DEEP_TOUCH_BIT      5

// Encoder Gray States bits
#define GRAY_STATE_BIT_0                0
#define GRAY_STATE_BIT_1                1

// System Control Bits
#define IQS7225A_ACK_RESET_BIT		0
#define IQS7225A_SW_RESET_BIT		1
#define IQS7225A_RE_ATI_BIT             2
#define IQS7225A_RESEED_BIT             3  
#define IQS7225A_I2C_INTERFACE_BIT_0    6       // 14
#define IQS7225A_I2C_INTERFACE_BIT_1    7       // 15
#define IQS7225A_STREAMING_BITS         0b00
#define IQS7225A_EVENT_BITS             0b01
#define IQS7225A_STREAM_IN_TOUCH_BITS   0b10

/* Defines and structs for IQS7225A states */
/**
* @brief  iqs7225a Init Enumeration.
*/
typedef enum {
        IQS7225A_INIT_NONE = (uint8_t) 0x00,
        IQS7225A_INIT_VERIFY_PRODUCT,
        IQS7225A_INIT_READ_RESET,
	IQS7225A_INIT_CHIP_RESET,
	IQS7225A_INIT_UPDATE_SETTINGS,
	IQS7225A_INIT_CHECK_RESET,
	IQS7225A_INIT_ACK_RESET,
	IQS7225A_INIT_ATI,
        IQS7225A_INIT_WAIT_FOR_ATI,
        IQS7225A_INIT_READ_DATA,
	IQS7225A_INIT_ACTIVATE_EVENT_MODE,
	IQS7225A_INIT_DONE
} iqs7225a_init_e;

typedef enum {
        IQS7225A_STATE_NONE = (uint8_t) 0x00,
        IQS7225A_STATE_START,
        IQS7225A_STATE_INIT,
        IQS7225A_STATE_SW_RESET,
        IQS7225A_STATE_CHECK_RESET,
	IQS7225A_STATE_RUN
} iqs7225a_state_e;

typedef enum {
        IQS7225A_CH0 = (uint8_t) 0x00,
        IQS7225A_CH1,
        IQS7225A_CH2,
        IQS7225A_CH3,
        IQS7225A_CH4,
        IQS7225A_CH5
} iqs7225a_channel_e;

typedef enum {
        STATE_NO_COIL_ACTIVE = (uint8_t) 0x00,
        STATE_COIL_A_ACTIVE,
        STATE_COIL_B_ACTIVE,
        STATE_COIL_A_AND_COIL_B_ACTIVE,
        STATE_UNKNOWN
} iqs7225a_encoder_states_e;

typedef enum{
        BUTTON0 = (uint8_t) 0x00,
        BUTTON1 = (uint8_t) 0x01,
        BUTTON2 = (uint8_t) 0x02,
        BUTTON3 = (uint8_t) 0x04,
        BUTTON4 = (uint8_t) 0x08,
        BUTTON5 = (uint8_t) 0x10
} iqs7225a_button;

typedef enum 
{
        IQS7225A_NORMAL_POWER = (uint8_t) 0x00,
        IQS7225A_LOW_POWER,
        IQS7225A_ULP,
        IQS7225A_HALT,
        IQS7225A_POWER_UNKNOWN
} iqs7225a_power_modes;

typedef enum
{
        IQS7225A_CH_NONE = (uint8_t) 0x00,
        IQS7225A_CH_PROX,
        IQS7225A_CH_TOUCH,
        IQS7225A_CH_DEEP_TOUCH,
        IQS7225A_CH_UNKNOWN,
} iqs7225a_ch_states;

/* IQS7225A Memory map data variables, only save the data that might be used 
during program runtime */
#pragma pack(1)
typedef struct
{
	/* READ ONLY */			        //  I2C Addresses:
	uint8_t VERSION_DETAILS[10]; 	        // 	0x0000 -> 0x0004
	
        uint8_t SYSTEM_STATUS[2];               // 	0x1000
	uint8_t EVENTS[2]; 		        // 	0x1001
        uint8_t PROX_AND_TOUCH_EVENTS[2];       // 	0x1002
        uint8_t DEEP_TOUCH_EVENTS[2];           // 	0x1003
        uint8_t ENCODER_GRAY_STATES[2];         // 	0x1004
        uint8_t ENCODER_ANGLE[2];               // 	0x1005
        uint8_t ENCODER_COUNTER[2];             // 	0x1006
        uint8_t CH_TRIG_LEVELS[12]; 	        // 	0x1007 -> 0x100C
        uint8_t CH_COUNTS[12]; 	                // 	0x1100 -> 0x1105
        uint8_t CH_LTA[12]; 	                // 	0x1200 -> 0x1205
        uint8_t CH_DELTAS[12]; 	                // 	0x1300 -> 0x1305

	/* READ WRITE */		        //  I2C Addresses:
	uint8_t SYSTEM_CONTROL[2]; 	        // 	0x2000
        uint8_t EVENT_MASK[2]; 	                // 	0x2001
        uint8_t I2C_WINDOW_TIMEOUT[2]; 	        // 	0x2002
        uint8_t I2C_CONFIG[2]; 	                // 	0x2003
} IQS7225A_MEMORY_MAP;
#pragma pack(4)

#pragma pack(1)
typedef struct {
        iqs7225a_state_e        state;
        iqs7225a_init_e         init_state;
}iqs7225a_s;
#pragma pack(4)

// Class Prototype
class IQS7225A
{
public:
        // Public Constructors
        IQS7225A();

        // Public Device States
        iqs7225a_s iqs7225a_state;

        // Public Variables
        IQS7225A_MEMORY_MAP IQSMemoryMap;
        bool new_data_available;

        // Public Methods
        void begin(uint8_t deviceAddressIn, uint8_t readyPinIn);
        bool init(void);
        void run(void);
        void queueValueUpdates(void);
        bool readATIactive(void);
        uint16_t getProductNum(bool stopOrRestart);
        uint8_t getMajorVersion(bool stopOrRestart);
        uint8_t getMinorVersion(bool stopOrRestart);
        void acknowledgeReset(bool stopOrRestart);
        void ReATI(bool stopOrRestart);
        void SW_Reset(bool stopOrRestart);
        void writeMM(bool stopOrRestart);
        void clearRDY(void);
        bool getRDYStatus(void);

        void setStreamMode(bool stopOrRestart);
        void setEventMode(bool stopOrRestart);
        void setStreamInTouchMode(bool stopOrRestart);

        void updateInfoFlags(bool stopOrRestart);
        iqs7225a_power_modes get_PowerMode(void);
        bool checkReset(void);

        bool channel_deepTouchState(iqs7225a_channel_e channel);
        bool channel_touchState(iqs7225a_channel_e channel);
        bool channel_proxState(iqs7225a_channel_e channel);

        int16_t getEncoderCounter(void);
        uint16_t getEncoderAngle(void);
        iqs7225a_encoder_states_e getEncoderState(void);
        void getChannelCounts(uint16_t *array);
        void force_I2C_communication(void);

private:
        // Private Variables
        uint8_t _deviceAddress;
        uint8_t _readyPin;
        uint8_t transferBytes[16];

        // Private Methods
        void writeRandomBytes16(uint16_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart);
        void readRandomBytes16(uint16_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart);

        bool getBit(uint8_t data, uint8_t bit_number);
        uint8_t setBit(uint8_t data, uint8_t bit_number);
        uint8_t clearBit(uint8_t data, uint8_t bit_number);

        void printArray(uint8_t bytesArray[], uint8_t numBytes);
};
#endif // IQS7225A_h  
