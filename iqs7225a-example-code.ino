/******************************************************************************
 *                                                                            *
 *                                Copyright by                                *
 *                                                                            *
 *                              Azoteq (Pty) Ltd                              *
 *                          Republic of South Africa                          *
 *                                                                            *
 *                           Tel: +27(0)21 863 0033                           *
 *                           E-mail: info@azoteq.com                          *
 *                                                                            *
 * ========================================================================== *
 * @file        iqs7225a-example-code.ino                                     *
 * @brief       IQS7225A Rotary Encoder EV KIT Example Code                   *
 *              (PCB Version - AZP1277A1)                                     *
 *              This example demonstrates how to write the desired settings   *
 *              to the IQS7225A in order to use the IQS7225A EV-Kit variant   *
 *              with 4 Inductive buttons and an inductive rotary encoder.     *
 *                                                                            *
 *              All data is displayed over serial communication with the      *
 *              following outputs:                                            *
 *              - Encoder Counts                                              *
 *              - Encoder State                                               *
 *              - Encoder Angle                                               *
 *              - Button States                                               *
 *              - Power Mode Changing (If enabled)                            *
 *              - Channel Counts (Force Communication)                        *
 *                                                                            *
 * @author      Azoteq PTY Ltd                                                *
 * @version     v1.0.1                                                        *
 * @date        2023-12-08                                                    *
 ******************************************************************************/

#include <Arduino.h>
#include "src\IQS7225A\IQS7225A.h"

/*** Defines ***/
#define DEMO_IQS7225A_ADDR            0x44
#define DEMO_IQS7225A_POWER_PIN       4
#define DEMO_IQS7225A_RDY_PIN         7

/*** Instances ***/
IQS7225A iqs7225a;

/*** Global Variables ***/
bool show_data = false;
uint8_t running_display_counter = 0;
iqs7225a_encoder_states_e running_encoder_state =  STATE_NO_COIL_ACTIVE;
iqs7225a_power_modes running_power_mode = IQS7225A_NORMAL_POWER;
iqs7225a_ch_states running_channel_states[IQS7225A_ROT_ENC_PCB_CH_NUM] = {IQS7225A_CH_NONE};
int16_t running_encoder_angle     = 0;
int16_t running_encoder_interval  = 0;

void setup() {
  /* Start Serial Communication */
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Start Serial communication");
  delay(200);

  /* Power On IQS7225A */
  pinMode(DEMO_IQS7225A_POWER_PIN, OUTPUT);
  delay(200);
  digitalWrite(DEMO_IQS7225A_POWER_PIN, LOW);
  delay(200);
  digitalWrite(DEMO_IQS7225A_POWER_PIN, HIGH);

  /* Initialize the IQS7225A with input parameters device address and RDY pin */
  iqs7225a.begin(DEMO_IQS7225A_ADDR, DEMO_IQS7225A_RDY_PIN);
  Serial.println("IQS7225A Ready");
  delay(1);
}

void loop() {
  
  iqs7225a.run(); // Runs the IQS7225A program loop
  
  force_comms_and_reset(); // function to initialize a force communication window.

  /* Process data read from IQS7225A when new data is available (RDY Line Low) */
  if (iqs7225a.new_data_available) 
  {
    if(show_data)
    {
      show_device_data();       // Display device data
    }
    else
    {
      if (printData())
      {
        printHeading();
        show_encoder_counter();   // Display encoder counter
        show_encoder_angle();     // Display encoder angle
        check_channel_states();   // Display the channel states
        show_encoder_states();    // Display encoder states
        check_power_mode();       // Display the current power mode
      }
    }

    iqs7225a.new_data_available = false;
  }
}

/* Show channel counts in the serial terminal */
void show_channel_counts(void) {
  uint16_t ch_array[6];
  iqs7225a.getChannelCounts(ch_array); //Read the channel counts from the memory map and stores them in an array
  Serial.println("| \tCH0\t | \tCH1\t | \tCH2\t | \tCH3\t | \tCH4\t | \tCH5\t |");
  Serial.print("\t");
  Serial.print(ch_array[0]);
  Serial.print("\t\t");
  Serial.print(ch_array[1]);
  Serial.print("\t\t");
  Serial.print(ch_array[2]);
  Serial.print("\t\t");
  Serial.print(ch_array[3]);
  Serial.print("\t\t");
  Serial.print(ch_array[4]);
  Serial.print("\t\t");
  Serial.print(ch_array[5]);
  Serial.println("\t");
}

/* Function to print the heading on the initial and after every 20th event*/
void printHeading(void)
{
  if (running_display_counter > 20 || running_display_counter == 0)
  {
    Serial.println("\nENC Counter\tENC Angle\tCH0\t\tCH1\t\tCH2\t\tCH3\t\tCH4\t\tCH5\t\tENC State\tPower Mode");

    running_display_counter = 1;
  }

  running_display_counter++;
}

/* Function to determine if a change occurred since the last event of the 
different channels and states*/
bool printData(void)
{
  bool bChange = false;

  /* Loop through all active channels to see if a change occurred */
  for(uint8_t i = 0; i < IQS7225A_ROT_ENC_PCB_CH_NUM; i++)
  {
    /* Check if the deep touch state bit is currently set */
    if(iqs7225a.channel_deepTouchState((iqs7225a_channel_e)(i)))
    {
      if(running_channel_states[i] != IQS7225A_CH_DEEP_TOUCH)
      {
        bChange = true;
        i = IQS7225A_ROT_ENC_PCB_CH_NUM;
      }
    }
    else if(iqs7225a.channel_touchState((iqs7225a_channel_e)(i)))
    {
      if(running_channel_states[i] != IQS7225A_CH_TOUCH)
      {
        bChange = true;
        i = IQS7225A_ROT_ENC_PCB_CH_NUM;
      }
    }
    else if (iqs7225a.channel_proxState((iqs7225a_channel_e)(i)))
    {
      if(running_channel_states[i] != IQS7225A_CH_PROX)
      {
        bChange = true;
        i = IQS7225A_ROT_ENC_PCB_CH_NUM;
      }
    }
    else
    {
      if(running_channel_states[i] != IQS7225A_CH_NONE)
      {
        bChange = true;
        i = IQS7225A_ROT_ENC_PCB_CH_NUM;
      }   
    }
  }

  /* Determine if a change of state or value occurred of the data this Arduino 
    example monitors */
  if (    bChange 
      || (running_power_mode        != iqs7225a.get_PowerMode()     ) 
      || (running_encoder_state     != iqs7225a.getEncoderState()   )
      || (running_encoder_angle     != iqs7225a.getEncoderAngle()   )  
      || (running_encoder_interval  != iqs7225a.getEncoderCounter() )
    )
  {
    return true; // Let the main loop know to display the changes that occurred.
  }
  else
  {
    return false; // No changes occurred of the data being monitored in this demo.
  }
}

/* Show the encoder counter in the serial terminal */
void show_encoder_counter(void) 
{
  int16_t buffer = iqs7225a.getEncoderCounter();

  Serial.print(buffer);
  Serial.print("\t\t");

  /* Update the running encoder interval value */ 
  running_encoder_interval = buffer;
}

/* Show the encoder angle in the serial terminal */
void show_encoder_angle(void) 
{
  int16_t buffer = iqs7225a.getEncoderAngle();

  Serial.print(buffer);
  Serial.print("\t\t");

  /* Update the running encoder angle value */ 
  running_encoder_angle = buffer;
}

/* Show the encoder counter in the serial terminal */
void show_encoder_states(void) 
{
  iqs7225a_encoder_states_e buffer  = iqs7225a.getEncoderState();

  switch (buffer)
  {
    case  STATE_NO_COIL_ACTIVE:
      Serial.print("No Coil\t");
    break;

    case  STATE_COIL_A_ACTIVE:
      Serial.print("Coil A\t");
    break;

    case  STATE_COIL_B_ACTIVE:
      Serial.print("Coil B\t");
    break;

    case  STATE_COIL_A_AND_COIL_B_ACTIVE:
      Serial.print("Coil A and B");
    break;

    default:
    break; 
  }

  /* Update the running encoder state value. */
  running_encoder_state = buffer;
}

/* Function to check when the current power mode of the IQS7225A changed */
void check_power_mode(void)
{
  iqs7225a_power_modes buffer = iqs7225a.get_PowerMode();

    switch(buffer)
    {
      case IQS7225A_NORMAL_POWER:
        Serial.println("\tNORMAL");
        break;

      case IQS7225A_LOW_POWER:
        Serial.println("\tLOW");
        break;

      case IQS7225A_ULP:
        Serial.println("\tULP");
        break;

      default:
        break;
    }

    /* Update the running power mode value */
    running_power_mode = buffer;
}

/* Function to check the proximity and touch states of the two inductive sensors */
void check_channel_states(void)
{
  /* Loop through all the active channels */
  for(uint8_t i = 0; i < IQS7225A_ROT_ENC_PCB_CH_NUM; i++)
  {
    /* Check if the deep touch state bit is set */
    if(iqs7225a.channel_deepTouchState((iqs7225a_channel_e)(i)))
    {
      running_channel_states[i] = IQS7225A_CH_DEEP_TOUCH;
      Serial.print("Deep Touch\t");
    }
    /* Check if the touch state bit is set */
    else if(iqs7225a.channel_touchState((iqs7225a_channel_e)(i)))
    {
      running_channel_states[i] = IQS7225A_CH_TOUCH;
      Serial.print("Touch\t\t");
    }
    /* Check if the proximity state bit is set */
    else if (iqs7225a.channel_proxState((iqs7225a_channel_e)(i)))
    {
      running_channel_states[i] = IQS7225A_CH_PROX;
      Serial.print("Prox\t\t");
    }
    /* None state if neither the touch nor proximity bit was set to 1 */
    else
    {
        running_channel_states[i] = IQS7225A_CH_NONE;
        Serial.print("-\t\t");
    }
  }
}

/* Force the IQS7225A to open a RDY window and read the current state of the device */
void force_comms_and_reset(void) {

 char message = read_message();

  /* If an 'f' was received over serial, open a forced communication window and
   * print the new data received */
  if(message == 'f')
  {
    iqs7225a.force_I2C_communication(); // Prompt the IQS7225A
    show_data = true;
  }

  /* If an 'r' was received over serial, request a software (SW) reset */
  if(message == 'r')
  {
    Serial.println("Software Reset Requested!");
    iqs7225a.force_I2C_communication(); // Request a RDY window
    iqs7225a.iqs7225a_state.state = IQS7225A_STATE_SW_RESET;
    running_display_counter = 0; // Ensure a new heading is printed in the terminal
    running_power_mode = IQS7225A_NORMAL_POWER;
  }
}

/* Read message sent over serial communication */
char read_message(void) {
  while (Serial.available()) {

    if (Serial.available() > 0) {
      return Serial.read();
    }
  }

  return '\n';
}

/* Show device data in the serial terminal when a force comms event is detected */
void show_device_data(void)
{
    Serial.println("******************************************IQS7225A DATA*******************************************");
    Serial.print("1. Encoder Counter: \t");
    Serial.println(iqs7225a.getEncoderCounter());
    Serial.print("2. Encoder Angle: \t");
    Serial.println(iqs7225a.getEncoderAngle());
    Serial.print("3. Encoder State: \t");
    Serial.println(iqs7225a.getEncoderState());
    Serial.print("4. Power Mode: \t");
    check_power_mode();
    Serial.println("5. Channel Counts:");
    show_channel_counts(); // Display counts in serial terminal

    Serial.println("**************************************************************************************************");

    show_data = false;
}

