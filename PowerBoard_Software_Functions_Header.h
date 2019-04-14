//Program: PowerBoard_Software_Functions.h
//Programmer: Evan Hite
//Purpose: To hold all constants and function prototypes for 
//         PowerBoard_Software.ino and PowerBoard_Software_Functions.cpp

#ifndef POWERBOARD_SOFTWARE_FUNCTIONS_HEADER_H
#define POWERBOARD_SOFTWARE_FUNCTIONS_HEADER_H
#include "RoveComm.h"
#include <Energia.h>
using namespace std ;

RoveCommEthernetUdp RoveComm;

//Global Constants

//Delays
#define ROVER_POWER_RESET_DELAY  3000 //Delay to reset rover power
#define ROVECOMM_DELAY  10 //Delay to send Rovecomm a package
#define DEBOUNCE_DELAY  10 //Delay after current or voltage trip
#define ROVECOMM_UPDATE_DELAY 1000 //Delay so that Rovecomm is not overloaded

//////////////////////////////////////////////Pinmap
// Control Pins for Busses
#define ACT_CTL_PIN              PN_3
#define LOGIC_CTL_PIN            PD_1
#define COM_CTL_PIN              PH_2
#define COM_LOGIC_CTL_PIN        PP_2
#define AUX_CTL_PIN              PK_5
#define M1_CTL_PIN               PK_7
#define M2_CTL_PIN               PK_6
#define M3_CTL_PIN               PH_1
#define M4_CTL_PIN               PH_0
#define M5_CTL_PIN               PM_2
#define M6_CTL_PIN               PM_1
#define M7_CTL_PIN               PM_0
#define FAN_CTL_PIN              PM_3

// Sensor Volts/Amps Readings Pins
#define ACT_I_MEAS_PIN           PE_2
#define LOGIC_I_MEAS_PIN         PE_1
#define COM_I_MEAS_PIN           PE_0
#define AUX_I_MEAS_PIN           PD_0
#define M1_I_MEAS_PIN            PK_3
#define M2_I_MEAS_PIN            PK_2
#define M3_I_MEAS_PIN            PK_1 
#define M4_I_MEAS_PIN            PK_0
#define M5_I_MEAS_PIN            PB_5
#define M6_I_MEAS_PIN            PB_4
#define M7_I_MEAS_PIN            PE_3
#define PACK_VOLTAGE_PIN         PE_5

//////////////////////////////////////////////RoveBoard
// Tiva1294C RoveBoard Specs
#define VCC                  3300       //volts
#define ADC_MAX              4096      //bits
#define ADC_MIN              420         //meme bits

//////////////////////////////////////////////Sensor
// ACS722LLCTR-40AU-T IC Sensor Specs 
#define SENSOR_SENSITIVITY    0.066    //volts/amp
#define SENSOR_SCALE          0.1      //volts/amp
#define SENSOR_BIAS           VCC * SENSOR_SCALE

#define CURRENT_MAX           40000//((VCC - SENSOR_BIAS) / SENSOR_SENSITIVITY)
#define CURRENT_MIN           0//-SENSOR_BIAS / SENSOR_SENSITIVITY

#define VOLTS_MIN             0
#define VOLTS_MAX             33600

//Safest Test pin
#define ESTOP_12V_COM_LOGIC_MAX_AMPS_THRESHOLD  2000 //5
#define ESTOP_12V_ACT_MAX_AMPS_THRESHOLD        2000 //15  
#define ESTOP_AUX_MAX_AMPS_THRESHOLD            19000 //17  (360W/21.6V) = 16.6 A
#define ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD      22000 //22

//Tuning Variables
#define LOGIC_COMM_TUNER  1085 //Tuner for ADC values for Logic and Comm busses
#define ACT_TUNER         1132 //Tuner for ADC values for Actuation bus
#define AUX_TUNER         1136 //Tuner for ADC values for Auxilliary bus
#define MOTOR_TUNER       1144 //Tuner for ADC values for motor busses

//Functions/////////////////////////////////////////////////////////////////////////

//Description: Checks the pin for bouncing voltages to avoid false positive
//Pre: Bouncing_pin is a pin on the tiva that reads amperage, max_amps_threshold
// is the maximum aperage the device can tolerate
//Post: Returns a bool of true if the pin has too much current
bool singleDebounce(const int & bouncing_pin,const int & max_amps_threshold, const int & Tuner);

//Description: Sets the pins on the tiva to certain buses on the powerboard
//Pre: None
//Post: All pins on the tiva are set to correct values.
void Configure_Pins ();

//Description: Turns all pins to on after setting all to low
//Pre: Pins must be in correct position from Configure_Pins
//Post: All pins are set high
void Pin_Initialization ();

//Description: Begins communication with RoveComm and sends a packet to rovecomm
//Pre: Bus[] must have a size of RC_POWERBOARD_BUSENABLED_DATACOUNT to work.
//Post: Sends to RoveComm a packet that the powerboard is live.
void Communication_Begin (uint8_t Bus []) ;

//Description: Checks overcurrent on the Bus and shuts off in need be
//Pre: BUS_I_MEAS_PIN must be a pin for current measurement, Bus[] must be the same array from Commuincation_Begin.
// BUS_CTL_PIN must be the relevent pin that controls the bus. and ESTOP_AMP_THRESHOLD must also be the relevent estop threshold.
//Post: Shuts off the bus if there is an overcurrent situation and writes this to Bus[]
bool Shut_Off( const int & BUS_I_MEAS_PIN, uint8_t Bus[], const int & BUS_CTL_PIN, const int & ESTOP_AMP_THRESHOLD, const int & Tuner) ;

//Description: Takes a packet from RoveComm and turn on or off a bus
//Pre: Enable_Disable should be a packet recieved from RoveComm, and Bus should be from Shut_Off
//Post: Turns on or off all buses that the packet says should be on or off and writes this to Bus[].
void Bus_Enable (const rovecomm_packet & Enable_Disable, uint8_t Bus[]) ;

//Description: Reads a current sensing pin and saves this value to current_reading
//Pre: Current_Reading should be an array part, for RoveComm reasons. BUS_I_MEAS_PIN should be a current measuring pin
//that corresponds to the spot on the Current_Reading array.
//Post: Returns a value to current_reading of the current reading value from the BUS_I_MEAS_PIN in mA.
void Pin_Read (uint16_t & current_reading, const int & BUS_I_MEAS_PIN) ;

#endif
