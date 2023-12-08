/**
  *****************************************************************************
  * @file     iqs7225a.cpp
  * @brief    This file contains the constructors and methods which allow ease
  *           of use of an IQS7225A. The IQS7225A is a self-capacitance, 
  *           mutual-capacitance and inductive sensing controller Integrated
  *           Circuit (IC) which provides 6-channel functionality. 
  *           This class provides an easy means of initializing and
  *           interacting with the IQS7225A device from an Arduino.
  * @author   Azoteq PTY Ltd
  * @version  V1.1
  * @date     2023
  *****************************************************************************
  * @attention  Makes use of the following standard Arduino libraries:
  * - Arduino.h -> Included in IQS7225A.h, comes standard with Arduino
  * - Wire.h    -> Included in IQS7225A.h, comes standard with Arduino
  ****************************************************************************/

/* Include Files */
#include "IQS7225A.h"
#include "IQS7225A_AZP1277A1_Demo_Settings.h"

/* Private Global Variables */
bool iqs7225a_deviceRDY = false;
uint8_t iqs7225a_ready_pin;

/* Private Functions */
void iqs7225a_ready_interrupt(void);

/*****************************************************************************/
/*                            CONSTRUCTORS                                   */
/*****************************************************************************/
IQS7225A::IQS7225A(){
}

/*****************************************************************************/
/*                           PUBLIC METHODS                                  */
/*****************************************************************************/

/**
  * @name   begin
  * @param  deviceAddress -> The address of the IQS7225A device.
  * @param  readyPin      -> The Arduino pin which is connected to the ready 
  *                          pin of the IQS7225A device.
  * @retval none
  * @note   - Receiving a true return value does not mean that initialization was 
  *           successful.
  *         - Receiving a true return value only means that the IQS device responded 
  *           to the request for communication.
  *         - Receiving a false return value means that initialization did not take 
  *           place at all.
  *         - If communication is successfully established then it is unlikely 
  *           that initialization will fail.
  */
void IQS7225A::begin(uint8_t deviceAddressIn, uint8_t readyPinIn)
{
  /* Initialize I2C communication here, since this library can't function 
  without it. */
  Wire.begin();
  Wire.setClock(400000);

  _deviceAddress = deviceAddressIn;
  iqs7225a_ready_pin = readyPinIn;
  attachInterrupt(digitalPinToInterrupt(iqs7225a_ready_pin), iqs7225a_ready_interrupt, CHANGE);

  /* Initialize "running" and "init" state machine variables. */
  iqs7225a_state.state = IQS7225A_STATE_START;
  iqs7225a_state.init_state = IQS7225A_INIT_VERIFY_PRODUCT;
}

/**
  * @name   init
  * @brief  A method that runs through a normal start-up routine to set up the  
  *         IQS7225A with the desired settings from the IQS7225A_init.h file.
  * @retval Returns true if the full start-up routine has been completed, returns 
  *         false if not.
  * @note   - No false return will be given, the program will thus be stuck when  
  *           one of the cases is not able to finish. 
  *         - See serial communication to find the ERROR case
  */
bool IQS7225A::init(void)
{
  uint16_t prod_num;
  uint8_t ver_maj, ver_min;

  switch (iqs7225a_state.init_state)
  {
    /* Verifies product number to determine if the correct device is connected for 
    this example */
    case IQS7225A_INIT_VERIFY_PRODUCT:
      if(iqs7225a_deviceRDY)
      {
        Serial.println("\tIQS7225A_INIT_VERIFY_PRODUCT");
        prod_num = getProductNum(RESTART);
        ver_maj = getMajorVersion(RESTART);
        ver_min = getMinorVersion(STOP);
        Serial.print("\t\tProduct number is: ");
        Serial.print(prod_num);
        Serial.print(" v");
        Serial.print(ver_maj);
        Serial.print(".");
        Serial.println(ver_min);
        if(prod_num == IQS7225A_PRODUCT_NUM)
        {
          Serial.println("\t\tIQS7225A Release UI Confirmed!");
          iqs7225a_state.init_state = IQS7225A_INIT_READ_RESET;
        }
        else
        {
          Serial.println("\t\tDevice is not a IQS7225A!");
          iqs7225a_state.init_state = IQS7225A_INIT_NONE;
        }
      }
    break;

    /* Verify if a reset has occurred */
    case IQS7225A_INIT_READ_RESET:
      if(iqs7225a_deviceRDY)
      {
        Serial.println("\tIQS7225A_INIT_READ_RESET");
        updateInfoFlags(STOP);
        if (checkReset())
        {
          Serial.println("\t\tReset event occurred.");
          iqs7225a_state.init_state = IQS7225A_INIT_UPDATE_SETTINGS;
        }
        else
        {
          Serial.println("\t\tNo Reset Event Detected - Request SW Reset");
          iqs7225a_state.init_state = IQS7225A_INIT_CHIP_RESET;
        }
      }
    break;

    /* Perform SW Reset */
    case IQS7225A_INIT_CHIP_RESET:
      if(iqs7225a_deviceRDY)
      {
        Serial.println("\tIQS7225A_INIT_CHIP_RESET");

        //Perform SW Reset
        SW_Reset(STOP);
        Serial.println("\t\tSoftware Reset Bit Set.");
        delay(100);
        iqs7225a_state.init_state = IQS7225A_INIT_READ_RESET;
      }
    break;

    /* Write all settings to IQS7225A from .h file */
    case IQS7225A_INIT_UPDATE_SETTINGS:
      if(iqs7225a_deviceRDY)
      {
        Serial.println("\tIQS7225A_INIT_UPDATE_SETTINGS");
        writeMM(STOP);
        iqs7225a_state.init_state = IQS7225A_INIT_ACK_RESET;
      }
    break;

    /* Acknowledge that the device went through a reset */
    case IQS7225A_INIT_ACK_RESET:
      if(iqs7225a_deviceRDY)
      {
        Serial.println("\tIQS7225A_INIT_ACK_RESET");
        acknowledgeReset(STOP);
        iqs7225a_state.init_state = IQS7225A_INIT_ATI;
      }
      break;

    /* Run the ATI algorithm to recalibrate the device with newly added settings */
    case IQS7225A_INIT_ATI:
      if(iqs7225a_deviceRDY)
      {
        Serial.println("\tIQS7225A_INIT_ATI");
        ReATI(STOP);
        iqs7225a_state.init_state = IQS7225A_INIT_WAIT_FOR_ATI;
        Serial.println("\tIQS7225A_INIT_WAIT_FOR_ATI");
      }
    break;

    /* Read the ATI Active bit to see if the rest of the program can continue */
    case IQS7225A_INIT_WAIT_FOR_ATI:
      if(iqs7225a_deviceRDY)
      {
        if(!readATIactive())
        {
          Serial.println("\t\tDONE");
          iqs7225a_state.init_state = IQS7225A_INIT_READ_DATA;
        }
      }
    break;

    /* Read the latest data from the iqs7225a */
    case IQS7225A_INIT_READ_DATA:
      if(iqs7225a_deviceRDY)
      {
        Serial.println("\tIQS7225A_INIT_READ_DATA");
        queueValueUpdates();
        iqs7225a_state.init_state = IQS7225A_INIT_ACTIVATE_EVENT_MODE;
      }
    break;

    /* Turn on I2C event mode */
    case IQS7225A_INIT_ACTIVATE_EVENT_MODE:
      if(iqs7225a_deviceRDY)
      {
        Serial.println("\tIQS7225A_INIT_ACTIVATE_EVENT_MODE");
        setEventMode(STOP);
        iqs7225a_state.init_state = IQS7225A_INIT_DONE;
      }
    break;

    /* If all operations have been completed correctly, the RDY pin can be set
     * up as an interrupt to indicate when new data is available */
    case IQS7225A_INIT_DONE:
      Serial.println("\tIQS7225A_INIT_DONE");
      new_data_available = true;
      return true;
    break;

    default:
      break;
  }
  return false;
}

/**
  * @name   run
  * @brief  This method is called continuously during runtime, and serves as the
  *         main state machine
  * @param  None
  * @retval None.
  * @note   The state machine continuously checks for specific events and
  *         updates the state machine accordingly. A reset event will cause
  *         the state machine to re-initialize the device.
  *
  *         queueValueUpdates can be edited by the user if other data should be
  *         read every time a RDY window is received.
  */
void IQS7225A::run(void)
{
  switch (iqs7225a_state.state)
  {
    /* After a hardware reset, this is the starting position of the main running 
      state machine */
    case IQS7225A_STATE_START:
      Serial.println("IQS7225A Initialization:");
      iqs7225a_state.state = IQS7225A_STATE_INIT;
    break;

    /* Perform the initialization routine on the IQS7225A */
    case IQS7225A_STATE_INIT:
      if(init())
      {
        Serial.println("IQS7225A Initialization complete!\n");
        iqs7225a_state.state = IQS7225A_STATE_RUN;
      }
    break;

    /* Send an I2C software reset in the next RDY window */
    case IQS7225A_STATE_SW_RESET:
      if(iqs7225a_deviceRDY)
      {
        SW_Reset(STOP);
        iqs7225a_state.state = IQS7225A_STATE_RUN;
      }
    break;

    /* Continuous reset monitoring state, ensure no reset event has occurred 
    for data to be valid */
    case IQS7225A_STATE_CHECK_RESET:
      if(checkReset())
      {
        Serial.println("Reset Occurred!\n");
        new_data_available = false;
        iqs7225a_state.state = IQS7225A_STATE_START;
        iqs7225a_state.init_state = IQS7225A_INIT_VERIFY_PRODUCT;
      }

      /* A reset did not occur, move to the run state and wait for new 
      ready window */
      else
      {
        new_data_available = true; /* No reset, thus data is valid */
        iqs7225a_state.state = IQS7225A_STATE_RUN;
      }
    break;

    /* If a RDY Window is open, read the latest values from the IQS7225A */
    case IQS7225A_STATE_RUN:
      if(iqs7225a_deviceRDY)
      {
        queueValueUpdates();
        iqs7225a_deviceRDY = false;
        new_data_available = false;
        iqs7225a_state.state = IQS7225A_STATE_CHECK_RESET;
      }
    break;
  }
}

/**
  * @name   iqs7225a_ready_interrupt
  * @brief  A method used as an interrupt function. Activated when a High-Low  
  *         and Low-High interrupt is seen on the correct Arduino interrupt pin.
  * @param  None.
  * @retval None.
  * @note   Keep this function as simple as possible to prevent stuck states 
  *         and slow operations.
  */
void iqs7225a_ready_interrupt(void)
{
  if(digitalRead(iqs7225a_ready_pin))
  {
    iqs7225a_deviceRDY = false;
  }
  else
  {
    iqs7225a_deviceRDY = true;
  }
}

/**
  * @name   clearRDY
  * @brief  A method used to clear the ready interrupt bit.
  * @param  None.
  * @retval None.
  */
void IQS7225A::clearRDY(void)
{
  iqs7225a_deviceRDY = false;
}

/**
  * @name   getRDYStatus
  * @brief  A method used to retrieve the device RDY status.
  * @param  None.
  * @retval Returns the boolean IQS7225A RDY state.
  *         - True when RDY line is LOW
  *         - False when RDY line is HIGH
  */
bool IQS7225A::getRDYStatus(void)
{
  return iqs7225a_deviceRDY;
}

/**
  * @name   queueValueUpdates
  * @brief  All I2C read operations in the queueValueUpdates method will be 
  *         performed each time the IQS7225A opens a RDY window.
  * @param  None.
  * @retval None.
  * @note   Any Address in the memory map can be read from here.
  */
void IQS7225A::queueValueUpdates(void)
{
  uint8_t transferBytes[26];	// The array which will hold the bytes to be transferred.

	/* Read the info flags. */
	readRandomBytes16(IQS7225A_MM_SYSTEM_STATUS, 26, transferBytes, RESTART);

	/* Assign the System Status */
  IQSMemoryMap.SYSTEM_STATUS[0] =  transferBytes[0];
  IQSMemoryMap.SYSTEM_STATUS[1] =  transferBytes[1];

  /* Assign the Events */
  IQSMemoryMap.EVENTS[0]  = transferBytes[2];
  IQSMemoryMap.EVENTS[1]  = transferBytes[3];

  /* Assign the Proximity and Touch Events */
  IQSMemoryMap.PROX_AND_TOUCH_EVENTS[0]  = transferBytes[4];
  IQSMemoryMap.PROX_AND_TOUCH_EVENTS[1] = transferBytes[5];

  /* Assign Deep Touch Events */
  IQSMemoryMap.DEEP_TOUCH_EVENTS[0]  = transferBytes[6];
  IQSMemoryMap.DEEP_TOUCH_EVENTS[1]  = transferBytes[7];

  /* Assign Encoder Gray States */
  IQSMemoryMap.ENCODER_GRAY_STATES[0]  = transferBytes[8];
  IQSMemoryMap.ENCODER_GRAY_STATES[1]  = transferBytes[9];

  /* Assign Encoder Angle */
  IQSMemoryMap.ENCODER_ANGLE[0]  = transferBytes[10];
  IQSMemoryMap.ENCODER_ANGLE[1]  = transferBytes[11];

  /* Assign Encoder Counter */
  IQSMemoryMap.ENCODER_COUNTER[0]  = transferBytes[12];
  IQSMemoryMap.ENCODER_COUNTER[1]  = transferBytes[13];

  /* Assign Channel 0 Trigger Level */
  IQSMemoryMap.CH_TRIG_LEVELS[0]  = transferBytes[14];
  IQSMemoryMap.CH_TRIG_LEVELS[1]  = transferBytes[15];

  /* Assign Channel 1 Trigger Level */
  IQSMemoryMap.CH_TRIG_LEVELS[2]  = transferBytes[16];
  IQSMemoryMap.CH_TRIG_LEVELS[3]  = transferBytes[17];

  /* Assign Channel 2 Trigger Level */
  IQSMemoryMap.CH_TRIG_LEVELS[4]  = transferBytes[18];
  IQSMemoryMap.CH_TRIG_LEVELS[5]  = transferBytes[19];

  /* Assign Channel 3 Trigger Level */
  IQSMemoryMap.CH_TRIG_LEVELS[6]  = transferBytes[20];
  IQSMemoryMap.CH_TRIG_LEVELS[7]  = transferBytes[21];

  /* Assign Channel 4 Trigger Level */
  IQSMemoryMap.CH_TRIG_LEVELS[8]  = transferBytes[22];
  IQSMemoryMap.CH_TRIG_LEVELS[9]  = transferBytes[23];

  /* Assign Channel 5 Trigger Level */
  IQSMemoryMap.CH_TRIG_LEVELS[10]  = transferBytes[24];
  IQSMemoryMap.CH_TRIG_LEVELS[11]  = transferBytes[25];

  readRandomBytes16(IQS7225A_MM_CHANNEL_0_COUNTS, 12, transferBytes, STOP);

  /* Assign Channel 0 Counts */
  IQSMemoryMap.CH_COUNTS[0]  = transferBytes[0];
  IQSMemoryMap.CH_COUNTS[1]  = transferBytes[1];

  /* Assign Channel 1 Counts */
  IQSMemoryMap.CH_COUNTS[2]  = transferBytes[2];
  IQSMemoryMap.CH_COUNTS[3]  = transferBytes[3];

  /* Assign Channel 2 Counts */
  IQSMemoryMap.CH_COUNTS[4]  = transferBytes[4];
  IQSMemoryMap.CH_COUNTS[5]  = transferBytes[5];

  /* Assign Channel 3 Counts */
  IQSMemoryMap.CH_COUNTS[6]  = transferBytes[6];
  IQSMemoryMap.CH_COUNTS[7]  = transferBytes[7];

  /* Assign Channel 4 Counts */
  IQSMemoryMap.CH_COUNTS[8]  = transferBytes[8];
  IQSMemoryMap.CH_COUNTS[9]  = transferBytes[9];

  /* Assign Channel 5 Counts */
  IQSMemoryMap.CH_COUNTS[10]  = transferBytes[10];
  IQSMemoryMap.CH_COUNTS[11]  = transferBytes[11];
}

/**
  * @name	  readATIactive
  * @brief  A method that checks if the ATI routine is still active
  * @param  None.
  * @retval Returns true if the ATI_ACTIVE_BIT is cleared, false if the 
  *         ATI_ACTIVE_BIT is set.
  * @note   If the ATI routine is active the channel states (NONE, PROX, TOUCH) 
  *         might exhibit unwanted behaviour. Thus it is advised to wait for 
  *         the routine to complete before continuing.
  */
bool IQS7225A::readATIactive(void)
{
  /* Read the Info flags */
  updateInfoFlags(STOP);

  /* Return the ATI Active status */
  return getBit(IQSMemoryMap.SYSTEM_STATUS[0], IQS7225A_ATI_ACTIVE_BIT);
}

/**
  * @name	  checkReset
  * @brief  A method that checks if the device has reset and returns the reset 
  *         status.
  * @param  None.
  * @retval Returns true if a reset has occurred, false if no reset has occurred.
  * @note   If a reset has occurred the device settings should be reloaded using 
  *         the begin function. After new device settings have been reloaded the 
  *         acknowledge reset function can be used to clear the reset flag.
  */
bool IQS7225A::checkReset(void)
{
	/* Perform a bitwise AND operation with the SHOW_RESET_BIT to return the reset status */
  return getBit(IQSMemoryMap.SYSTEM_STATUS[0], IQS7225A_SHOW_RESET_BIT);
}

/**
  * @name	  getProductNum
  * @brief  A method that checks the device product number and compares the 
  *         result to the defined value to return a boolean result.
  * @param  stopOrRestart ->  Specifies whether the communications window must be 
  *                           kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval Returns the product number as a 16-bit unsigned integer value.
  * @note   If the product is not correctly identified an appropriate messages 
  *         should be displayed.
  */
uint16_t IQS7225A::getProductNum(bool stopOrRestart)
{
	uint8_t transferBytes[2];	
  uint8_t prodNumLow = 0;         
  uint8_t prodNumHigh = 0;        
  uint16_t prodNumReturn = 0;     

	readRandomBytes16(IQS7225A_MM_PROD_NUM, 2, transferBytes, stopOrRestart);
  prodNumLow = transferBytes[0];
  prodNumHigh = transferBytes[1];
  prodNumReturn = (uint16_t)(prodNumLow);
  prodNumReturn |= (uint16_t)(prodNumHigh<<8);
  
  return prodNumReturn;
}

/**
  * @name	  getMajorVersion
  * @brief  A method that checks the device firmware version major number. 
  * @param  stopOrRestart ->  Specifies whether the communications window must  
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval Returns major version number as a uint8_t value.
  */
uint8_t IQS7225A::getMajorVersion(bool stopOrRestart)
{
	uint8_t transferBytes[2];	
  uint8_t ver_maj = 0;         
  
	readRandomBytes16(IQS7225A_MM_MAJOR_VERSION_NUM, 2, transferBytes, stopOrRestart);
  ver_maj = transferBytes[0];

  return ver_maj;
}

/**
  * @name	  getMinorVersion
  * @brief  A method that checks the device firmware version minor number. 
  * @param  stopOrRestart ->  Specifies whether the communications window must  
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval Returns minor version number as a unit8_t value.
  */
uint8_t IQS7225A::getMinorVersion(bool stopOrRestart)
{
	uint8_t transferBytes[2];	
  uint8_t ver_min = 0;         
  
	readRandomBytes16(IQS7225A_MM_MINOR_VERSION_NUM, 2, transferBytes, stopOrRestart); 
  ver_min = transferBytes[0];

  return ver_min;
}

/**
  * @name	  acknowledgeReset
  * @brief  A method that clears the Reset Event bit by writing to the ACK_RESET
  *         bit in the SYSTEM_CONTROL_SETTINGS register.
  * @param  stopOrRestart ->  Specifies whether the communications window must 
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval None.
  * @note   If a reset has occurred the device settings should be reloaded using 
  *         the begin function. After new device settings have been reloaded this 
  *         method should be used to clear the reset bit.
  */
void IQS7225A::acknowledgeReset(bool stopOrRestart)
{
	uint8_t transferBytes[2]; // The array which will hold the bytes which are transferred.
  /* First read the full register to preserve the bytes */
	readRandomBytes16(IQS7225A_MM_SYSTEM_CONTROL_SETTINGS, 2, transferBytes, RESTART);
  /* Set the ACK_RESET_BIT in SYSTEM_CONTROL_SETTINGS register */
  transferBytes[0] = setBit(transferBytes[0], IQS7225A_ACK_RESET_BIT);  
  /* Write the bytes back to the device */
	writeRandomBytes16(IQS7225A_MM_SYSTEM_CONTROL_SETTINGS, 2, transferBytes, stopOrRestart);
}

/**
  * @name   ReATI
  * @brief  A method which sets the REDO_ATI_BIT in order to force the IQS7225A 
  *         device to run the Automatic Tuning Implementation (ATI) routine.
  * @param  stopOrRestart ->  Specifies whether the communications window must 
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval None.
  */
void IQS7225A::ReATI(bool stopOrRestart)
{
  uint8_t transferByte[2]; // The array which will hold the bytes which are transferred.
  /* First read the full register to preserve the bytes */
  readRandomBytes16(IQS7225A_MM_SYSTEM_CONTROL_SETTINGS, 2, transferByte, RESTART);
  /* Set the RE_ATI_BIT in the SYSTEM_CONTROL_SETTINGS register */
  transferByte[0] = setBit(transferByte[0], IQS7225A_RE_ATI_BIT);  
  /* Write the new byte to the required device. */
  writeRandomBytes16(IQS7225A_MM_SYSTEM_CONTROL_SETTINGS, 2, transferByte, stopOrRestart);
}

/**
  * @name   SW_Reset
  * @brief  A method that sets the SW RESET bit to force the IQS7225A 
  *         device to do a Software reset.
  * @param  stopOrRestart ->  Specifies whether the communications window must 
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval None.
  */
void IQS7225A::SW_Reset(bool stopOrRestart)
{
  uint8_t transferByte[2]; // The array which will hold the bytes which are transferred.
  /* First read the full register to preserve the bytes */
  readRandomBytes16(IQS7225A_MM_SYSTEM_CONTROL_SETTINGS, 2, transferByte, RESTART);
  /* Set the SW_RESET_BIT in the SYSTEM_CONTROL_SETTINGS register */
  transferByte[0] = setBit(transferByte[0], IQS7225A_SW_RESET_BIT); 
  /* Write the new byte to the required device. */
  writeRandomBytes16(IQS7225A_MM_SYSTEM_CONTROL_SETTINGS, 2, transferByte, stopOrRestart);
}

/**
  * @name   setStreamMode
  * @brief  A method to set the IQS7225A device  streaming mode.
  * @param  stopOrRestart ->  Specifies whether the communications window must 
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval None. 
  * @note   All other bits at the IQS7225A_MM_SYSTEM_CONTROL_SETTINGS register 
  *         are preserved.
  */
void IQS7225A::setStreamMode(bool stopOrRestart)
{
  uint8_t transferBytes[2]; // The array which will hold the bytes which are transferred.
  /* First read the full register to preserve the bytes */
  readRandomBytes16(IQS7225A_MM_SYSTEM_CONTROL_SETTINGS, 2, transferBytes, RESTART);
  /* Set/Clear the I2C_INTERFACE_BITS in the SYSTEM_CONTROL_SETTINGS register */
  transferBytes[1] = clearBit(transferBytes[1], IQS7225A_I2C_INTERFACE_BIT_0);
  transferBytes[1] = setBit(transferBytes[1], IQS7225A_I2C_INTERFACE_BIT_1);
  /* Write the bytes back to the device */
  writeRandomBytes16(IQS7225A_MM_SYSTEM_CONTROL_SETTINGS, 2, transferBytes, stopOrRestart);
}

/**
  * @name   setEventMode
  * @brief  A method to set the IQS7225A device into event mode.
  * @param  stopOrRestart ->  Specifies whether the communications window must 
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval None. 
  */
void IQS7225A::setEventMode(bool stopOrRestart)
{
  uint8_t transferBytes[2]; // The array which will hold the bytes which are transferred.
  /* First read the full register to preserve the bytes */
  readRandomBytes16(IQS7225A_MM_SYSTEM_CONTROL_SETTINGS, 2, transferBytes, RESTART);
  /* Set/Clear the I2C_INTERFACE_BITS in the SYSTEM_CONTROL_SETTINGS register */
  transferBytes[1] = setBit(transferBytes[1], IQS7225A_I2C_INTERFACE_BIT_0);
  transferBytes[1] = clearBit(transferBytes[1], IQS7225A_I2C_INTERFACE_BIT_1);
  /* Write the bytes back to the device */
  writeRandomBytes16(IQS7225A_MM_SYSTEM_CONTROL_SETTINGS, 2, transferBytes, stopOrRestart);
}

/**
  * @name   setStreamInTouchMode
  * @brief  A method to set the IQS7225A device into streaming when in touch mode.
  * @param  stopOrRestart ->  Specifies whether the communications window must 
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval None. 
  * @note   All other bits at the IQS7225A_MM_SYSTEM_CONTROL_SETTINGS register 
  *         are preserved.
  */
void IQS7225A::setStreamInTouchMode(bool stopOrRestart)
{
  uint8_t transferBytes[2]; // The array which will hold the bytes which are transferred.
  //First read the full register to preserve the bytes
  readRandomBytes16(IQS7225A_MM_SYSTEM_CONTROL_SETTINGS, 2, transferBytes, RESTART);
  /* Set/Clear the I2C_INTERFACE_BITS in the SYSTEM_CONTROL_SETTINGS register */
  transferBytes[1] = clearBit(transferBytes[1], IQS7225A_I2C_INTERFACE_BIT_0);
  transferBytes[1] = setBit(transferBytes[1], IQS7225A_I2C_INTERFACE_BIT_1);
  /* Write the bytes back to the device */
  writeRandomBytes16(IQS7225A_MM_SYSTEM_CONTROL_SETTINGS, 2, transferBytes, stopOrRestart);
}

/**
  * @name   updateInfoFlags
  * @brief  A method which reads the IQS7225A info flags and assigns them to the 
  *         local IQSMemoryMap variables.
  * @param  stopOrRestart ->  Specifies whether the communications window must 
  *                           be kept open or must be closed after this action. 
  *                           Use the STOP and RESTART definitions.
  * @retval None.
  * @note   The local infoFlags SYSTEM_STATUS and EVENTS registers are altered 
  *         with the new value of the info flags register retrieved from the 
  *         IQS7225A.
  */
void IQS7225A::updateInfoFlags(bool stopOrRestart)
{
	uint8_t transferBytes[4];
	readRandomBytes16(IQS7225A_MM_SYSTEM_STATUS, 4, transferBytes, stopOrRestart);
  IQSMemoryMap.SYSTEM_STATUS[0] =  transferBytes[0];
  IQSMemoryMap.SYSTEM_STATUS[1] =  transferBytes[1];
  IQSMemoryMap.EVENTS[0]        =  transferBytes[2];
  IQSMemoryMap.EVENTS[1]        =  transferBytes[3];
}

/**
  * @name   get_PowerMode
  * @brief  A method that reads the local SYSTEM_STATUS variable and returns 
  *         the current power mode. 
  * @param  void
  * @retval None
  * @note   - See Datasheet on power mode options and timeouts. 
  *         - Normal Power, Low Power and Ultra Low Power (ULP).
  */
iqs7225a_power_modes IQS7225A::get_PowerMode(void)
{
  uint8_t buffer = getBit(IQSMemoryMap.SYSTEM_STATUS[0], IQS7225A_POWER_EVENT_BIT_0);
  buffer += getBit(IQSMemoryMap.SYSTEM_STATUS[0], IQS7225A_POWER_EVENT_BIT_1) << 1;

  if(buffer == IQS7225A_NORMAL_POWER_BITS)
  {
    return IQS7225A_NORMAL_POWER;
  }
  else if(buffer == IQS7225A_LOW_POWER_BITS)
  {
    return IQS7225A_LOW_POWER;
  }
  else if(buffer == IQS7225A_ULP_BITS)
  {
    return IQS7225A_ULP;
  }
    else
  {
    return IQS7225A_POWER_UNKNOWN;
  }
}
/**
  * @name   channel_deepTouchState
  * @brief  A method that reads the local DEEP_TOUCH_EVENTS for a given 
  *         channel and determines if the channel is in a deep touch state.
  * @param  channel ->  The channel name on the IQS7225A (CH0-CH5) for which a 
  *                     deep touch state needs to be determined.
  * @retval Returns true if a deep touch is active and false if there is no 
  *         deep touch. 
  * @note   See the IQS7225A_Channel_e typedef for all possible channel names.
  */
bool IQS7225A::channel_deepTouchState(iqs7225a_channel_e channel)
{
  switch(channel)
	{
		case IQS7225A_CH0:
			return getBit(IQSMemoryMap.DEEP_TOUCH_EVENTS[0], IQS7225A_CH0_DEEP_TOUCH_BIT);
		break;

    case IQS7225A_CH1:
			return getBit(IQSMemoryMap.DEEP_TOUCH_EVENTS[0], IQS7225A_CH1_DEEP_TOUCH_BIT);
		break;

    case IQS7225A_CH2:
			return getBit(IQSMemoryMap.DEEP_TOUCH_EVENTS[0], IQS7225A_CH2_DEEP_TOUCH_BIT);
		break;

    case IQS7225A_CH3:
			return getBit(IQSMemoryMap.DEEP_TOUCH_EVENTS[0], IQS7225A_CH3_DEEP_TOUCH_BIT);
		break;

    case IQS7225A_CH4:
			return getBit(IQSMemoryMap.DEEP_TOUCH_EVENTS[0], IQS7225A_CH4_DEEP_TOUCH_BIT);
		break;

    case IQS7225A_CH5:
			return getBit(IQSMemoryMap.DEEP_TOUCH_EVENTS[0], IQS7225A_CH5_DEEP_TOUCH_BIT);
		break;

		default:
			return false;
		break;
	}
}

/**
  * @name   channel_touchState
  * @brief  A method that reads the local PROX_AND_TOUCH_EVENTS for a given 
  *         channel and determines if the channel is in a touch state.
  * @param  channel ->  The channel name on the IQS7225A (CH0-CH5) for which a 
  *                     touch state needs to be determined.
  * @retval Returns true if a touch is active and false if there is no touch. 
  * @note   See the IQS7225A_Channel_e typedef for all possible channel names.
  */
bool IQS7225A::channel_touchState(iqs7225a_channel_e channel)
{
  switch(channel)
	{
		case IQS7225A_CH0:
			return getBit(IQSMemoryMap.PROX_AND_TOUCH_EVENTS[1], IQS7225A_CH0_TOUCH_BIT);
		break;

    case IQS7225A_CH1:
			return getBit(IQSMemoryMap.PROX_AND_TOUCH_EVENTS[1], IQS7225A_CH1_TOUCH_BIT);
		break;

    case IQS7225A_CH2:
			return getBit(IQSMemoryMap.PROX_AND_TOUCH_EVENTS[1], IQS7225A_CH2_TOUCH_BIT);
		break;

    case IQS7225A_CH3:
			return getBit(IQSMemoryMap.PROX_AND_TOUCH_EVENTS[1], IQS7225A_CH3_TOUCH_BIT);
		break;

    case IQS7225A_CH4:
			return getBit(IQSMemoryMap.PROX_AND_TOUCH_EVENTS[1], IQS7225A_CH4_TOUCH_BIT);
		break;

    case IQS7225A_CH5:
			return getBit(IQSMemoryMap.PROX_AND_TOUCH_EVENTS[1], IQS7225A_CH5_TOUCH_BIT);
		break;

		default:
			return false;
		break;
	}
}

/**
  * @name   channel_proxState
  * @brief  A method that reads the local PROX_AND_TOUCH_EVENTS for a given 
  *         channel and determines if the channel is in a proximity state.
  * @param  channel ->  The channel name on the IQS7225A (CH0-CH5) for which a 
  *                     proximity state needs to be determined.
  * @retval Returns true if proximity is active and false if there is no proximity. 
  * @note   See the iqs7225a_channel_e typedef for all possible channel names.
  */
bool IQS7225A::channel_proxState(iqs7225a_channel_e channel)
{
  switch(channel)
	{
		case IQS7225A_CH0:
			return getBit(IQSMemoryMap.PROX_AND_TOUCH_EVENTS[0], IQS7225A_CH0_PROX_BIT);
		break;

    case IQS7225A_CH1:
			return getBit(IQSMemoryMap.PROX_AND_TOUCH_EVENTS[0], IQS7225A_CH1_PROX_BIT);
		break;

    case IQS7225A_CH2:
			return getBit(IQSMemoryMap.PROX_AND_TOUCH_EVENTS[0], IQS7225A_CH2_PROX_BIT);
		break;

    case IQS7225A_CH3:
			return getBit(IQSMemoryMap.PROX_AND_TOUCH_EVENTS[0], IQS7225A_CH3_PROX_BIT);
		break;

    case IQS7225A_CH4:
			return getBit(IQSMemoryMap.PROX_AND_TOUCH_EVENTS[0], IQS7225A_CH4_PROX_BIT);
		break;

    case IQS7225A_CH5:
			return getBit(IQSMemoryMap.PROX_AND_TOUCH_EVENTS[0], IQS7225A_CH5_PROX_BIT);
		break;

		default:
			return false;
		break;

	}
}

/**
  * @name   getChannelCounts
  * @brief  A method that reads all the local channel counts registers and
  *         stores it in an array.
  * @param  array -> Pointer to the array where the channel counts will be stored
  * @retval None 
  * @note   No return value but the user supplied array is updated. 
  */
void IQS7225A::getChannelCounts(uint16_t *array)
{
  array[0] = IQSMemoryMap.CH_COUNTS[0] + (IQSMemoryMap.CH_COUNTS[1] << 8); 
  array[1] = IQSMemoryMap.CH_COUNTS[2] + (IQSMemoryMap.CH_COUNTS[3] << 8); 
  array[2] = IQSMemoryMap.CH_COUNTS[4] + (IQSMemoryMap.CH_COUNTS[5] << 8); 
  array[3] = IQSMemoryMap.CH_COUNTS[6] + (IQSMemoryMap.CH_COUNTS[7] << 8); 
  array[4] = IQSMemoryMap.CH_COUNTS[8] + (IQSMemoryMap.CH_COUNTS[9] << 8); 
  array[5] = IQSMemoryMap.CH_COUNTS[10] + (IQSMemoryMap.CH_COUNTS[11] << 8); 
}
/*****************************************************************************/
/*							  		   		ADVANCED PUBLIC METHODS					    		 			   */
/*****************************************************************************/

/**
  * @name   writeMM
  * @brief  Function to write the whole memory map to the device (writable) 
  *         registers 
  * @param  stopOrRestart ->  Specifies whether the communications window must
  *                           be kept open or must be closed after this action.
  *                           Use the STOP and RESTART definitions.
  * @retval None.
  * @note   IQS7225A_init.h -> exported GUI init.h file
  */
void IQS7225A::writeMM(bool stopOrRestart)
{
    uint8_t transferBytes[16];

    /* Channel LTA Overwrite */
    /* Memory Map Position 0x1400 - 0x1405 */
    transferBytes[0] = LTA0_WRITE0;
    transferBytes[1] = LTA0_WRITE1;
    transferBytes[2] = LTA1_WRITE0;
    transferBytes[3] = LTA1_WRITE1;
    transferBytes[4] = LTA2_WRITE0;
    transferBytes[5] = LTA2_WRITE1;
    transferBytes[6] = LTA3_WRITE0;
    transferBytes[7] = LTA3_WRITE1;
    transferBytes[8] = LTA4_WRITE0;
    transferBytes[9] = LTA4_WRITE1;
    transferBytes[10] = LTA5_WRITE0;
    transferBytes[11] = LTA5_WRITE1;
    writeRandomBytes16(0x1400, 12, transferBytes, RESTART);
    Serial.println("\t\t1. Write LTA Overwrite");

    /* System Settings */
    /* Memory Map Position 0x2000 - 0x2003 */
    transferBytes[0] = CONTROL0;
    transferBytes[1] = CONTROL1;
    transferBytes[2] = EVENT_MASK0;
    transferBytes[3] = EVENT_MASK1;
    transferBytes[4] = I2C_WINDOW_TIMEOUT0;
    transferBytes[5] = I2C_WINDOW_TIMEOUT1;
    transferBytes[6] = I2C_COMMS0;
    transferBytes[7] = I2C_COMMS1;

    writeRandomBytes16(0x2000, 8, transferBytes, RESTART);
    Serial.println("\t\t2. Write System Settings");

    /* Report Rates and Mode Timeouts *
    /* Memory Map Position 0x2100 - 0x2107 */
    transferBytes[0] = ATI_MODE_TIMEOUT0;
    transferBytes[1] = ATI_MODE_TIMEOUT1;
    transferBytes[2] = ATI_MODE_REPORT_RATE0;
    transferBytes[3] = ATI_MODE_REPORT_RATE1;
    transferBytes[4] = NP_TIMEOUT0;
    transferBytes[5] = NP_TIMEOUT1;
    transferBytes[6] = NP_REPORT_RATE0;
    transferBytes[7] = NP_REPORT_RATE1;
    transferBytes[8] = LP_TIMEOUT0;
    transferBytes[9] = LP_TIMEOUT1;
    transferBytes[10] = LP_REPORT_RATE0;
    transferBytes[11] = LP_REPORT_RATE1;
    transferBytes[12] = ULP_TIMEOUT0;
    transferBytes[13] = ULP_TIMEOUT1;
    transferBytes[14] = ULP_REPORT_RATE0;
    transferBytes[15] = ULP_REPORT_RATE1;
                        
    writeRandomBytes16(0x8100, 16, transferBytes, RESTART);
    Serial.println("\t\t3. Write Report Rates and Timeout Settings");

     /* Channel Counts & LTA Filter Coefficient Reseed */
    /* Memory Map Position 0x2200 - 0x2205 */
    transferBytes[0] = CH0_COEFF_RESEED0;
    transferBytes[1] = CH0_COEFF_RESEED1;
    transferBytes[2] = CH1_COEFF_RESEED0;
    transferBytes[3] = CH1_COEFF_RESEED1;
    transferBytes[4] = CH2_COEFF_RESEED0;
    transferBytes[5] = CH2_COEFF_RESEED1;
    transferBytes[6] = CH3_COEFF_RESEED0;
    transferBytes[7] = CH3_COEFF_RESEED1;
    transferBytes[8] = CH4_COEFF_RESEED0;
    transferBytes[9] = CH4_COEFF_RESEED1;
    transferBytes[10] = CH5_COEFF_RESEED0;
    transferBytes[11] = CH5_COEFF_RESEED1;
                        
    writeRandomBytes16(0x2200, 12, transferBytes, RESTART);
    Serial.println("\t\t4. Write Channel Counts & LTA Filter Coefficient Reseed");

    /* Change the Trigger Settings: Max Delta */
    /* Memory Map Position 0x2300 - 0x2305 */
    transferBytes[0] = CH0_TRIGGER_SETTINGS_0_0;
    transferBytes[1] = CH0_TRIGGER_SETTINGS_0_1;
    transferBytes[2] = CH1_TRIGGER_SETTINGS_0_0;
    transferBytes[3] = CH1_TRIGGER_SETTINGS_0_1;
    transferBytes[4] = CH2_TRIGGER_SETTINGS_0_0;
    transferBytes[5] = CH2_TRIGGER_SETTINGS_0_1;
    transferBytes[6] = CH3_TRIGGER_SETTINGS_0_0;
    transferBytes[7] = CH3_TRIGGER_SETTINGS_0_1;
    transferBytes[8] = CH4_TRIGGER_SETTINGS_0_0;
    transferBytes[9] = CH4_TRIGGER_SETTINGS_0_1;
    transferBytes[10] = CH5_TRIGGER_SETTINGS_0_0;
    transferBytes[11] = CH5_TRIGGER_SETTINGS_0_1;

    writeRandomBytes16(0x2300, 12, transferBytes, RESTART);
    Serial.println("\t\t5. Write Channel Max Delta");

    /* Change the Trigger Settings: Number of Threshold */
    /* Memory Map Position 0x2400 - 0x2405 */
    transferBytes[0] = CH0_TRIGGER_SETTINGS_1_0;
    transferBytes[1] = CH0_TRIGGER_SETTINGS_1_1;
    transferBytes[2] = CH1_TRIGGER_SETTINGS_1_0;
    transferBytes[3] = CH1_TRIGGER_SETTINGS_1_1;
    transferBytes[4] = CH2_TRIGGER_SETTINGS_1_0;
    transferBytes[5] = CH2_TRIGGER_SETTINGS_1_1;
    transferBytes[6] = CH3_TRIGGER_SETTINGS_1_0;
    transferBytes[7] = CH3_TRIGGER_SETTINGS_1_1;
    transferBytes[8] = CH4_TRIGGER_SETTINGS_1_0;
    transferBytes[9] = CH4_TRIGGER_SETTINGS_1_1;
    transferBytes[10] = CH5_TRIGGER_SETTINGS_1_0;
    transferBytes[11] = CH5_TRIGGER_SETTINGS_1_1;

    writeRandomBytes16(0x2400, 12, transferBytes, RESTART);
    Serial.println("\t\t6. Write Channel Number of Thresholds");

    /*Cycle 0 */
    /* Memory Map Position 0x3000 - 0x3002 */
    transferBytes[0] =  CYCLE0_CONV_FREQ_FRAC;
    transferBytes[1] =  CYCLE0_CONV_FREQ_PERIOD;
    transferBytes[2] =  CYCLE0_GEN_SET;
    transferBytes[3] =  CYCLE0_ENGINE_MODE;
    transferBytes[4] =  CYCLE0_TX_SEL0;
    transferBytes[5] =  CYCLE0_TX_SEL1;

    writeRandomBytes16(0x3000, 6, transferBytes, RESTART);  

    /*Cycle 1 */
    /* Memory Map Position 0x3100 - 0x3102 */
    transferBytes[0] =  CYCLE1_CONV_FREQ_FRAC;
    transferBytes[1] =  CYCLE1_CONV_FREQ_PERIOD;
    transferBytes[2] =  CYCLE1_GEN_SET;
    transferBytes[3] =  CYCLE1_ENGINE_MODE;
    transferBytes[4] =  CYCLE1_TX_SEL0;
    transferBytes[5] =  CYCLE1_TX_SEL1;

    writeRandomBytes16(0x3100, 6, transferBytes, RESTART);  

    /*Cycle 2 */
    /* Memory Map Position 0x3200 - 0x3202 */
    transferBytes[0] =  CYCLE2_CONV_FREQ_FRAC;
    transferBytes[1] =  CYCLE2_CONV_FREQ_PERIOD;
    transferBytes[2] =  CYCLE2_GEN_SET;
    transferBytes[3] =  CYCLE2_ENGINE_MODE;
    transferBytes[4] =  CYCLE2_TX_SEL0;
    transferBytes[5] =  CYCLE2_TX_SEL1;

    writeRandomBytes16(0x3200, 6, transferBytes, RESTART);  

    /*Cycle 3 */
    transferBytes[0] =  CYCLE3_CONV_FREQ_FRAC;
    transferBytes[1] =  CYCLE3_CONV_FREQ_PERIOD;
    transferBytes[2] =  CYCLE3_GEN_SET;
    transferBytes[3] =  CYCLE3_ENGINE_MODE;
    transferBytes[4] =  CYCLE3_TX_SEL0;
    transferBytes[5] =  CYCLE3_TX_SEL1;

    writeRandomBytes16(0x3300, 6, transferBytes, RESTART);   

    /*Cycle 4 */
    /* Memory Map Position 0x3400 - 0x3402 */
    transferBytes[0] =  CYCLE4_CONV_FREQ_FRAC;
    transferBytes[1] =  CYCLE4_CONV_FREQ_PERIOD;
    transferBytes[2] =  CYCLE4_GEN_SET;
    transferBytes[3] =  CYCLE4_ENGINE_MODE;
    transferBytes[4] =  CYCLE4_TX_SEL0;
    transferBytes[5] =  CYCLE4_TX_SEL1;

    writeRandomBytes16(0x3400, 6, transferBytes, RESTART); 

    /*Cycle 5 */
    /* Memory Map Position 0x3500 - 0x3502 */
    transferBytes[0] =  CYCLE5_CONV_FREQ_FRAC;
    transferBytes[1] =  CYCLE5_CONV_FREQ_PERIOD;
    transferBytes[2] =  CYCLE5_GEN_SET;
    transferBytes[3] =  CYCLE5_ENGINE_MODE;
    transferBytes[4] =  CYCLE5_TX_SEL0;
    transferBytes[5] =  CYCLE5_TX_SEL1;
    
    writeRandomBytes16(0x3500, 6, transferBytes, RESTART);
    Serial.println("\t\t7. Write Cycle Settings");

    /* Cycle Channel Select */
    /* Memory Map Position 0x3600 - 0x3605 */
    transferBytes[0] = CYCLE0_ENGINE0;
    transferBytes[1] = CYCLE0_ENGINE1;
    transferBytes[2] = CYCLE1_ENGINE0;
    transferBytes[3] = CYCLE1_ENGINE1;
    transferBytes[4] = CYCLE2_ENGINE0;
    transferBytes[5] = CYCLE2_ENGINE1;
    transferBytes[6] = CYCLE3_ENGINE0;
    transferBytes[7] = CYCLE3_ENGINE1;
    transferBytes[8] = CYCLE4_ENGINE0;
    transferBytes[9] = CYCLE4_ENGINE1;
    transferBytes[10] = CYCLE5_ENGINE0;
    transferBytes[11] = CYCLE5_ENGINE1;
    
    writeRandomBytes16(0x3600, 12, transferBytes, RESTART);
    Serial.println("\t\t8. Write Cycle Channel Select Settings");

    /* Button 0  */
    /* Memory Map Position 0x4000 - 0x4006 */
    transferBytes[0] = BUTTON0_PROX_THRESHOLD;
    transferBytes[1] = BUTTON0_PROX_HYSTERESIS;
    transferBytes[2] = BUTTON0_TOUCH_THRESHOLD;
    transferBytes[3] = BUTTON0_TOUCH_HYSTERESIS;
    transferBytes[4] = BUTTON0_DEEP_TOUCH_THRESHOLD;
    transferBytes[5] = BUTTON0_DEEP_TOUCH_HYSTERESIS;
    transferBytes[6] = BUTTON0_FILTER_HALT_TIMEOUT;
    transferBytes[7] = BUTTON0_OTHER_TIMEOUT;
    transferBytes[8] = BUTTON0_GENERALS;
    transferBytes[9] = BUTTON0_BLOCK;
    transferBytes[10] = BUTTON0_COUNT_BETA;
    transferBytes[11] = BUTTON0_LTA_BETA;
    transferBytes[12] = BUTTON0_LTA_BETA_FAST;
    transferBytes[13] = BUTTON0_LTA_BETA_FAST_BOUND;

    writeRandomBytes16(0x4000, 14, transferBytes, RESTART);

    /* Button 1  */
    /* Memory Map Position 0x4100 - 0x4106 */
    transferBytes[0] = BUTTON1_PROX_THRESHOLD;
    transferBytes[1] = BUTTON1_PROX_HYSTERESIS;
    transferBytes[2] = BUTTON1_TOUCH_THRESHOLD;
    transferBytes[3] = BUTTON1_TOUCH_HYSTERESIS;
    transferBytes[4] = BUTTON1_DEEP_TOUCH_THRESHOLD;
    transferBytes[5] = BUTTON1_DEEP_TOUCH_HYSTERESIS;
    transferBytes[6] = BUTTON1_FILTER_HALT_TIMEOUT;
    transferBytes[7] = BUTTON1_OTHER_TIMEOUT;
    transferBytes[8] = BUTTON1_GENERALS;
    transferBytes[9] = BUTTON1_BLOCK;
    transferBytes[10] = BUTTON1_COUNT_BETA;
    transferBytes[11] = BUTTON1_LTA_BETA;
    transferBytes[12] = BUTTON1_LTA_BETA_FAST;
    transferBytes[13] = BUTTON1_LTA_BETA_FAST_BOUND;

    writeRandomBytes16(0x4100, 14, transferBytes, RESTART);

    /* Button 2  */
    /* Memory Map Position 0x4200 - 0x4206 */
    transferBytes[0] = BUTTON2_PROX_THRESHOLD;
    transferBytes[1] = BUTTON2_PROX_HYSTERESIS;
    transferBytes[2] = BUTTON2_TOUCH_THRESHOLD;
    transferBytes[3] = BUTTON2_TOUCH_HYSTERESIS;
    transferBytes[4] = BUTTON2_DEEP_TOUCH_THRESHOLD;
    transferBytes[5] = BUTTON2_DEEP_TOUCH_HYSTERESIS;
    transferBytes[6] = BUTTON2_FILTER_HALT_TIMEOUT;
    transferBytes[7] = BUTTON2_OTHER_TIMEOUT;
    transferBytes[8] = BUTTON2_GENERALS;
    transferBytes[9] = BUTTON2_BLOCK;
    transferBytes[10] = BUTTON2_COUNT_BETA;     
    transferBytes[11] = BUTTON2_LTA_BETA;
    transferBytes[12] = BUTTON2_LTA_BETA_FAST;
    transferBytes[13] = BUTTON2_LTA_BETA_FAST_BOUND;

    writeRandomBytes16(0x4200, 14, transferBytes, RESTART);

    /* Button 3  */
    /* Memory Map Position 0x4300 - 0x4306 */
    transferBytes[0] = BUTTON3_PROX_THRESHOLD;
    transferBytes[1] = BUTTON3_PROX_HYSTERESIS;
    transferBytes[2] = BUTTON3_TOUCH_THRESHOLD;
    transferBytes[3] = BUTTON3_TOUCH_HYSTERESIS;
    transferBytes[4] = BUTTON3_DEEP_TOUCH_THRESHOLD;
    transferBytes[5] = BUTTON3_DEEP_TOUCH_HYSTERESIS;
    transferBytes[6] = BUTTON3_FILTER_HALT_TIMEOUT;
    transferBytes[7] = BUTTON3_OTHER_TIMEOUT;
    transferBytes[8] = BUTTON3_GENERALS;                    
    transferBytes[9] = BUTTON3_BLOCK;
    transferBytes[10] = BUTTON3_COUNT_BETA;
    transferBytes[11] = BUTTON3_LTA_BETA;
    transferBytes[12] = BUTTON3_LTA_BETA_FAST;
    transferBytes[13] = BUTTON3_LTA_BETA_FAST_BOUND;

    writeRandomBytes16(0x4300, 14, transferBytes, RESTART);

    /* Button 4  */
    /* Memory Map Position 0x4400 - 0x4406 */
    transferBytes[0] = BUTTON4_PROX_THRESHOLD;
    transferBytes[1] = BUTTON4_PROX_HYSTERESIS;
    transferBytes[2] = BUTTON4_TOUCH_THRESHOLD;
    transferBytes[3] = BUTTON4_TOUCH_HYSTERESIS;
    transferBytes[4] = BUTTON4_DEEP_TOUCH_THRESHOLD;
    transferBytes[5] = BUTTON4_DEEP_TOUCH_HYSTERESIS;
    transferBytes[6] = BUTTON4_FILTER_HALT_TIMEOUT;
    transferBytes[7] = BUTTON4_OTHER_TIMEOUT;
    transferBytes[8] = BUTTON4_GENERALS;
    transferBytes[9] = BUTTON4_BLOCK;
    transferBytes[10] = BUTTON4_COUNT_BETA;
    transferBytes[11] = BUTTON4_LTA_BETA;
    transferBytes[12] = BUTTON4_LTA_BETA_FAST;
    transferBytes[13] = BUTTON4_LTA_BETA_FAST_BOUND;

    writeRandomBytes16(0x4400, 14, transferBytes, RESTART);

    /* Button 5  */
    /* Memory Map Position 0x4500 - 0x4506 */
    transferBytes[0] = BUTTON5_PROX_THRESHOLD;
    transferBytes[1] = BUTTON5_PROX_HYSTERESIS;
    transferBytes[2] = BUTTON5_TOUCH_THRESHOLD;
    transferBytes[3] = BUTTON5_TOUCH_HYSTERESIS;
    transferBytes[4] = BUTTON5_DEEP_TOUCH_THRESHOLD;
    transferBytes[5] = BUTTON5_DEEP_TOUCH_HYSTERESIS;
    transferBytes[6] = BUTTON5_FILTER_HALT_TIMEOUT;
    transferBytes[7] = BUTTON5_OTHER_TIMEOUT;
    transferBytes[8] = BUTTON5_GENERALS;
    transferBytes[9] = BUTTON5_BLOCK;
    transferBytes[10] = BUTTON5_COUNT_BETA;
    transferBytes[11] = BUTTON5_LTA_BETA;
    transferBytes[12] = BUTTON5_LTA_BETA_FAST;
    transferBytes[13] = BUTTON5_LTA_BETA_FAST_BOUND;

    writeRandomBytes16(0x4500, 14, transferBytes, RESTART);
    Serial.println("\t\t9. Write Button Settings");

    /* Sensor 0 */
    /* Memory Map Position 0x5000 - 0x5003  */
    transferBytes[0] = SENSOR0_CONVERSIONS;
    transferBytes[1] = SENSOR0_GENERALS;
    transferBytes[2] = SENSOR0_ATI_MODE_BASE;
    transferBytes[3] = SENSOR0_ATI_TARGET;
    transferBytes[4] = SENSOR0_MULTIPLIERS_0;
    transferBytes[5] = SENSOR0_MULTIPLIERS_1;
    transferBytes[6] = SENSOR0_COMPENSATION_0;
    transferBytes[7] = SENSOR0_COMPENSATION_1;

    writeRandomBytes16(0x5000, 8, transferBytes, RESTART);

    /* Sensor 1 */
    /* Memory Map Position 0x5100 - 0x5103  */
    transferBytes[0] = SENSOR1_CONVERSIONS;
    transferBytes[1] = SENSOR1_GENERALS;
    transferBytes[2] = SENSOR1_ATI_MODE_BASE;
    transferBytes[3] = SENSOR1_ATI_TARGET;
    transferBytes[4] = SENSOR1_MULTIPLIERS_0;
    transferBytes[5] = SENSOR1_MULTIPLIERS_1;
    transferBytes[6] = SENSOR1_COMPENSATION_0;
    transferBytes[7] = SENSOR1_COMPENSATION_1;

    writeRandomBytes16(0x5100, 8, transferBytes, RESTART);

    /* Sensor 2 */
    /* Memory Map Position 0x5200 - 0x5203  */
    transferBytes[0] = SENSOR2_CONVERSIONS;
    transferBytes[1] = SENSOR2_GENERALS;
    transferBytes[2] = SENSOR2_ATI_MODE_BASE;
    transferBytes[3] = SENSOR2_ATI_TARGET;
    transferBytes[4] = SENSOR2_MULTIPLIERS_0;
    transferBytes[5] = SENSOR2_MULTIPLIERS_1;
    transferBytes[6] = SENSOR2_COMPENSATION_0;          
    transferBytes[7] = SENSOR2_COMPENSATION_1;

    writeRandomBytes16(0x5200, 8, transferBytes, RESTART);

    /* Sensor 3 */
    /* Memory Map Position 0x5300 - 0x5303  */
    transferBytes[0] = SENSOR3_CONVERSIONS;
    transferBytes[1] = SENSOR3_GENERALS;
    transferBytes[2] = SENSOR3_ATI_MODE_BASE;
    transferBytes[3] = SENSOR3_ATI_TARGET;
    transferBytes[4] = SENSOR3_MULTIPLIERS_0;
    transferBytes[5] = SENSOR3_MULTIPLIERS_1;
    transferBytes[6] = SENSOR3_COMPENSATION_0;
    transferBytes[7] = SENSOR3_COMPENSATION_1;

    writeRandomBytes16(0x5300, 8, transferBytes, RESTART);

    /* Sensor 4 */
    /* Memory Map Position 0x5400 - 0x5403  */
    transferBytes[0] = SENSOR4_CONVERSIONS;
    transferBytes[1] = SENSOR4_GENERALS;
    transferBytes[2] = SENSOR4_ATI_MODE_BASE;
    transferBytes[3] = SENSOR4_ATI_TARGET;
    transferBytes[4] = SENSOR4_MULTIPLIERS_0;
    transferBytes[5] = SENSOR4_MULTIPLIERS_1;      
    transferBytes[6] = SENSOR4_COMPENSATION_0;
    transferBytes[7] = SENSOR4_COMPENSATION_1;

    writeRandomBytes16(0x5400, 8, transferBytes, RESTART);

    /* Sensor 5 */
    /* Memory Map Position 0x5500 - 0x5503  */
    transferBytes[0] = SENSOR5_CONVERSIONS;
    transferBytes[1] = SENSOR5_GENERALS;
    transferBytes[2] = SENSOR5_ATI_MODE_BASE;
    transferBytes[3] = SENSOR5_ATI_TARGET;
    transferBytes[4] = SENSOR5_MULTIPLIERS_0;
    transferBytes[5] = SENSOR5_MULTIPLIERS_1;
    transferBytes[6] = SENSOR5_COMPENSATION_0;
    transferBytes[7] = SENSOR5_COMPENSATION_1;

    writeRandomBytes16(0x5500, 8, transferBytes, RESTART);
    Serial.println("\t\t10. Write Sensor Settings");
    
    /* Change the Rotation UI Settings */
    /* Memory Map Position 0x6000 - 0x6202 */
    transferBytes[0] = ROTATION_SEGMENTS_0;
    transferBytes[1] = ROTATION_SEGMENTS_1;
    transferBytes[2] = ROTATION_COIL_INDEX_A;
    transferBytes[3] = ROTATION_COIL_INDEX_A_BAR;
    transferBytes[4] = ROTATION_COIL_THESHOLD_ENTER_A0;
    transferBytes[5] = ROTATION_COIL_THESHOLD_ENTER_A1;
    transferBytes[6] = ROTATION_COIL_THESHOLD_EXIT_A0;
    transferBytes[7] = ROTATION_COIL_THESHOLD_EXIT_A1;
    transferBytes[8] = ROTATION_COIL_INDEX_B;
    transferBytes[9] = ROTATION_COIL_INDEX_B_BAR;
    transferBytes[10] = ROTATION_COIL_THESHOLD_ENTER_B0;
    transferBytes[11] = ROTATION_COIL_THESHOLD_ENTER_B1;
    transferBytes[12] = ROTATION_COIL_THESHOLD_EXIT_B0;
    transferBytes[13] = ROTATION_COIL_THESHOLD_EXIT_B1;

    writeRandomBytes16(0x6000, 14, transferBytes, stopOrRestart);
    Serial.println("\t\t11. Write Rotation UI Settings");
}

/*****************************************************************************/
/*                           PRIVATE METHODS                                 */
/*****************************************************************************/

/**
  * @name   writeRandomBytes16
  * @brief  A method that writes a specified number of bytes to a specified 
  *         address, the bytes to write are supplied using an array 
  *         pointer. This method is used by all other methods of this class  
  *         which writes data to the IQS7225A device.
  * @param  memoryAddress ->  The memory address at which to start writing the 
  *                           bytes to. See the "IQS7225A_addresses.h" file.
  * @param  numBytes      ->  The number of bytes that must be written.
  * @param  bytesArray    ->  The array which stores the bytes which will be 
  *                           written to the memory location.
  * @param  stopOrRestart ->  A boolean that specifies whether the communication 
  *                           window should remain open or be closed of transfer.
  *                           False keeps it open, true closes it. Use the STOP 
  *                           and RESTART definitions.
  * @retval No value is returned, only the IQS device registers are altered.
  * @note   Uses standard Arduino "Wire" library which is for I2C communication.
  *         Take note that a full array cannot be passed to a function in C++.
  *         Pass an array to the function by using only its name, e.g. "bytesArray", 
  *         without the square brackets, this passes a pointer to the array. 
  *         The values to be written must be loaded into the array prior 
  *         to passing it to the function.
  */
void IQS7225A::writeRandomBytes16(uint16_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart)
{
  // Select the device with the address of "_deviceAddress" and start communication.
  Wire.beginTransmission(_deviceAddress);
  /* Specify the memory address where the IQS7225A must start saving the data, 
  as designated by the "memoryAddress" variable. */
  uint8_t addr_h, addr_l;
  addr_h = memoryAddress >> 8;
  addr_l = memoryAddress;
  Wire.write(addr_h);
  Wire.write(addr_l);
  // Write the bytes as specified in the array that "arrayAddress" pointer points to.
  for(int i=0; i<numBytes; i++)
  {
    Wire.write(bytesArray[i]);
  }
  // End the transmission, user decides to STOP or RESTART.
  Wire.endTransmission(stopOrRestart);

  /* Always manually close the RDY window after a STOP is sent to prevent 
  writing while the RDY window closes */
  if(stopOrRestart == STOP)
  {
    iqs7225a_deviceRDY = false;
  }
}

/**
 * @name    readRandomBytes16
 * @brief   A method that reads a specified number of bytes from a specified 
 *          address and saves it into a user-supplied array. This method is used 
 *          by all other methods in this class which read data from the IQS7225A 
 *          device.
 * @param   memoryAddress ->  The memory address at which to start reading bytes 
 *                            from.  See the "IQS7225A_addresses.h" file.
 * @param   numBytes      ->  The number of bytes that must be read.
 * @param   bytesArray    ->  The array which will store the bytes to be read, 
 *                            this array will be overwritten.
 * @param   stopOrRestart ->  A boolean that specifies whether the communication 
 *                            window should remain open or be closed after transfer.
 *                            False keeps it open, true closes it. Use the STOP 
 *                            and RESTART definitions. 
 * @retval  No value is returned, however, the user-supplied array is overwritten.
 * @note    Uses standard Arduino "Wire" library which is for I2C communication.
 *          Take note that C++ cannot return an array, therefore, the array which 
 *          is passed as an argument is overwritten with the required values.
 *          Pass an array to the method by using only its name, e.g. "bytesArray", 
 *          without the brackets, this passes a pointer to the array.
 */
void IQS7225A::readRandomBytes16(uint16_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[], bool stopOrRestart)
{
  // A simple counter to assist with loading bytes into the user-supplied array.
  uint8_t i = 0;  

  /* Select the device with the address "_deviceAddress" and start communication. */
  Wire.beginTransmission(_deviceAddress);

  /* Specify the memory address where the IQS7225A must start saving the data, 
  as designated by the "memoryAddress" variable. */
  uint8_t addr_h, addr_l;
  addr_h = memoryAddress >> 8;
  addr_l = memoryAddress;
  uint8_t buffer[2]={addr_h, addr_l};
  Wire.write( buffer , 2);
  // Complete the selection and communication initialization.
  Wire.endTransmission(RESTART);  // Restart transmission for reading that follows.
  /* The required device has now been selected and it has been told which 
  register to send information from. */

  // Request "numBytes" bytes from the device which has the address "_deviceAddress"
  do
  {
    Wire.requestFrom((int)_deviceAddress, (int)numBytes, (int)stopOrRestart);
  }while(Wire.available() == 0);  // Wait for response, this sometimes takes a few attempts

  // Load the received bytes into the array until there are no more 
  while(Wire.available())
  {
    // Load the received bytes into the user-supplied array
    bytesArray[i] = Wire.read();
    i++;
  }

  /* Always manually close the RDY window after a STOP is sent to prevent 
  writing while the RDY window closes */
  if(stopOrRestart == STOP)
  {
    iqs7225a_deviceRDY = false;
  }
}

/**
  * @name   getEncoderCounter
  * @brief  Return the encoder counter from the local ENCODER_COUNTER variable. 
  * @param  None
  * @retval Return the 16-bit integer value of the encoder counter
  */
int16_t IQS7225A::getEncoderCounter(void)
{
  uint16_t buffer = IQSMemoryMap.ENCODER_COUNTER[0];
  buffer += IQSMemoryMap.ENCODER_COUNTER[1]<<8;
  return buffer;
}

/**
  * @name   getEncoderAngle
  * @brief  Return the encoder angle from the local ENCODER_ANGLE variable.
  * @param  None
  * @retval Return the 16-bit unsigned integer value of the encoder angle
  */
uint16_t IQS7225A::getEncoderAngle(void)
{
  uint16_t buffer = IQSMemoryMap.ENCODER_ANGLE[0];
  buffer += IQSMemoryMap.ENCODER_ANGLE[1]<<8;
  return buffer;
}

/**
  * @name   getEncoderState
  * @brief  Return the encoder state from the local ENCODER_GRAY_STATES variable.
  * @param  None
  * @retval Return the iqs7225a_encoder_states_e value based on the current 
  *         state of the encoder
  */
iqs7225a_encoder_states_e IQS7225A::getEncoderState(void)
{
  uint8_t buffer = getBit(IQSMemoryMap.ENCODER_GRAY_STATES[0], GRAY_STATE_BIT_0);
  buffer += getBit(IQSMemoryMap.ENCODER_GRAY_STATES[0], GRAY_STATE_BIT_1) << 1;

  switch(buffer)
  {
    case STATE_NO_COIL_ACTIVE:
      return STATE_NO_COIL_ACTIVE;
      break;

    case STATE_COIL_A_ACTIVE:
      return STATE_COIL_A_ACTIVE;
      break;
      
    case STATE_COIL_B_ACTIVE:
      return STATE_COIL_B_ACTIVE;
      break;

    case STATE_COIL_A_AND_COIL_B_ACTIVE:
      return STATE_COIL_A_AND_COIL_B_ACTIVE;
      break;

    default:
      return STATE_UNKNOWN;
      break;
  }

  return STATE_UNKNOWN;
}

/**
  * @name   printArray
  * @brief  Print the specified array to the serial terminal
  * @param  bytesArray -> the array that stores the register bytes
  * @param  numBytes -> number of bytes to print
  * @retval None
  */
void IQS7225A::printArray(uint8_t bytesArray[], uint8_t numBytes)
{
  for (int i = 0; i < numBytes; i++) { 
    if (bytesArray[i] < 0x10) 
      {Serial.print("0");
    } 
    Serial.print(bytesArray[i], HEX); 
    Serial.print(" "); 
  }
  Serial.println("");
}

/**
  * @name   getBit
  * @brief  A method that returns the chosen bit value of the provided byte.
  * @param  data       -> byte of which a given bit value needs to be calculated.
  * @param  bit_number -> a number between 0 and 7 representing the bit in question.
  * @retval The boolean value of the specific bit requested. 
  */
bool IQS7225A::getBit(uint8_t data, uint8_t bit_number)
{
  return (data & ( 1 << bit_number )) >> bit_number;
}

/**
  * @name   setBit
  * @brief  A method that returns the chosen bit value of the provided byte.
  * @param  data       -> byte of which a given bit value needs to be calculated.
  * @param  bit_number -> a number between 0 and 7 representing the bit in question.
  * @retval Returns an 8-bit unsigned integer value of the given data byte with 
  *         the requested bit set.
  */
uint8_t IQS7225A::setBit(uint8_t data, uint8_t bit_number)
{
	return (data |= 1UL << bit_number);
}

/**
  * @name   clearBit
  * @brief  A method that returns the chosen bit value of the provided byte.
  * @param  data       -> byte of which a given bit value needs to be calculated.
  * @param  bit_number -> a number between 0 and 7 representing the bit in question.
  * @retval Returns an 8-bit unsigned integer value of the given data byte with 
  *         the requested bit cleared.
  */
uint8_t IQS7225A::clearBit(uint8_t data, uint8_t bit_number)
{
	return (data &= ~(1UL << bit_number));
}

/**
  * @name   force_I2C_communication
  * @brief  A method which writes to memory address 0xFF to open a 
  *         communication window on the IQS7225A.
  * @param  None.
  * @retval None.
  */
void IQS7225A::force_I2C_communication(void)
{
  /*Ensure RDY is HIGH at the moment*/
  if (!iqs7225a_deviceRDY)
  {
    /* Select the device with the address "DEMO_IQS7225A_ADDR" and start 
    communication. */
    Wire.beginTransmission(_deviceAddress);

    /* Write to memory address 0xFF that will prompt the IQS7225A to open a
    communication window.*/
    Wire.write(0xFF);

    /* End the transmission and the user decides to STOP or RESTART. */
    Wire.endTransmission(STOP);
  }
}