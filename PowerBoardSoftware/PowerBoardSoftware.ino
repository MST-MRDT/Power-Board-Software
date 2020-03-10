/*
*   POWERBOARD SOFTWARE
*   Patrick Sanchez
*   Missouri University of Science and Technology
*   Mars Rover Design Team
*   2020, Revision 1
*/

//C types
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "Energia.h"

//RoveComm
#include <RoveComm.h>

RoveCommEthernet RoveComm;
rovecomm_packet packet;

const uint16_t ROVECOMM_TELEM_RATE         = 1000;
const uint16_t NO_ROVECOMM_MESSAGE          = 0;

//Current readings
const uint16_t MOTOR_CURRENT_READING      = 3105; //M1,M2,M3,M4,M5,M6,M7,Spare (last 2 unused atm)
const uint16_t ACT_LOG_CURRENT_READING    = 3106; //Actuation, Logic
const uint16_t MISC_30V_CURRENT_READING   = 3107; //12V Board, Rockets, Auxiliary
const uint16_t VAC_CURRENT_READING        = 3108; //Vacuum

//Bus control
//Disable = 0, Enable = 1
const uint16_t MOTOR_ENABLE      = 3100; //M1,M2,M3,M4,M5,M6,M7,Spare (last 2 unused atm)
const uint16_t ACT_LOG_ENABLE    = 3101; //Actuation,Logic
const uint16_t MISC_30V_ENABLE   = 3102; //12V Board, Rockets, Auxiliary
const uint16_t VAC_ENABLE        = 3103; //Vacuum
const uint16_t PATCH_ENABLE      = 3104; //PB1,PB2,PB3,PB4,PB5,PB6,PB7,PB8


//TIVA PIN MAPPINGS
//ANALOG PINS
//30V
#define RKT_ISENSE PE_4
#define TWV_ISENSE PE_5
#define AUX_ISENSE PD_3
#define VAC_ISENSE PE_2
#define M1_ISENSE PD_4
#define M2_ISENSE PK_1
#define M3_ISENSE PK_3
#define M4_ISENSE PD_2
#define M5_ISENSE PB_4
#define M6_ISENSE PE_1
//12V
#define LOG_ISENSE PD_7
#define ACT_ISENSE PE_3

//TEMPORARY MAPPINGS BECAUSE I DID BAD
#define LOG_EN PA_6
#define ACT_EN PD_7
//end temp stuff

//DIGITAL PINS
//Error LED
#define ERROR PB_3
//30V
#define RKT_EN PB_2
#define TWV_EN PC_6
#define AUX_EN PC_7 
#define VAC_EN PC_5
#define M1_EN PP_1
#define M2_EN PD_5
#define M3_EN PQ_0
#define M4_EN PN_5
#define M5_EN PP_0
#define M6_EN PE_0
//12V
//#define LOG_EN PM_4
//#define ACT_EN PA_6
#define VOUT_EN PM_5
//Patch Panel
#define PB1_EN PL_3
#define PB2_EN PF_3
#define PB3_EN PL_2
#define PB4_EN PG_0
#define PB5_EN PL_1
#define PB6_EN PL_4
#define PB7_EN PL_0
#define PB8_EN PL_5

//Current sense mapping values
const uint16_t ADCMin = 330;
const uint16_t ADCMax = 4095;
const uint16_t ISENSEMin = 0;
const uint16_t ISENSEMax = 40000;
const float CURR_SCALER = 1.15;

//Overcurrent constants (amps)
const uint8_t OCP_MOTOR = 180; //temp fix due to inaccurate current readings on Motor busses
const uint8_t OCP_12V = 18;
const uint8_t OCP_30V = 18;
const uint8_t OCP_RKT = 4;
const uint8_t OCP_VAC = 13;
const uint16_t RESET_TIME = 5000;  //ms between tripping OCP and turn on


struct Bus
{
  int EN_PIN;
  int ISENSE_PIN = -1;//default for busses without ISENSE pins
  bool enabled = false;

  Bus(const int en, const int isense)
  {
    EN_PIN = en;
    ISENSE_PIN = isense;
    pinMode(ISENSE_PIN,INPUT);
    pinMode(EN_PIN,OUTPUT);
  }

  Bus(const int en)
  {
    EN_PIN = en;
    pinMode(EN_PIN,OUTPUT);
  }

};

Bus busses_Motor[6] = {Bus(M1_EN,M1_ISENSE),Bus(M2_EN,M2_ISENSE),Bus(M3_EN,M3_ISENSE),Bus(M4_EN,M4_ISENSE),Bus(M5_EN,M5_ISENSE),Bus(M6_EN,M6_ISENSE)};
Bus busses_12V[3] = {Bus(ACT_EN,ACT_ISENSE),Bus(LOG_EN,LOG_ISENSE),Bus(VOUT_EN)};
Bus busses_30V[3] = {Bus(TWV_EN,TWV_ISENSE),Bus(RKT_EN,RKT_ISENSE),Bus(AUX_EN,AUX_ISENSE)};
Bus vac = Bus(VAC_EN,VAC_ISENSE);
//the last time in millis() when telemetry was sent
unsigned long lastTelemTime = 0;


void setup() {
  Serial.begin(9600);
  RoveComm.begin(RC_POWERBOARD_FOURTHOCTET, RC_ROVECOMM_ETHERNET_POWERBOARD_PORT);
  delay(100);
  Serial.println("Started");

  //turning on all 30V busses
  for(int i = 0; i < 3; i++)
  {
    digitalWrite(busses_30V[i].EN_PIN,HIGH);
    busses_30V[i].enabled = true;
  }

  //turning on all 12V busses
  for(int i = 0; i < 3; i++)
  {
    digitalWrite(busses_12V[i].EN_PIN,HIGH);
    busses_12V[i].enabled = true;
  }

  //turning on the ACT bus as well
  pinMode(PD_7, OUTPUT);
  digitalWrite(PD_7, HIGH);
  
  delay(3000);

  //turning on all motor busses
  for(int i = 0; i < 6; i++)
  {
    digitalWrite(busses_Motor[i].EN_PIN,HIGH);
    busses_Motor[i].enabled = true;
  }
  delay(500);
}

void loop()
{
  //read
  packet = RoveComm.read();
  //Serial.println(packet.data_id);

  //Motor bus switches
  switch(packet.data_id)
  {
    case RC_POWERBOARD_MOTOR_BUSENABLE_DATAID:
    {
      //expects data to be an array with a single uint8
      //checks each bit. Bit 0 is M1, bit 1 is M2, etc.
      for(int i = 0; i < 6; i++)
      {
        if(packet.data[0] & 1<<i) 
        {
          //enable motor
          digitalWrite(busses_Motor[i].EN_PIN,HIGH);
          busses_Motor[i].enabled = true;
        }
        else
        {
          //disable motor
          digitalWrite(busses_Motor[i].EN_PIN,LOW);
          busses_Motor[i].enabled = false;
        }
      }
      break;
    }
    case RC_POWERBOARD_12V_BUSENABLE_DATAID:
    {      
      //expects data to be an array with a single uint8
      //checks each bit. Bit 0 is Actuation, bit 1 is Logic, bit 2 is VOUT.
      Serial.println("Enable/Disable 12V");
      for(int i = 0; i < 3; i++)
      {
        if(packet.data[0] & 1<<i) 
        {
          //enable bus
          digitalWrite(busses_12V[i].EN_PIN,HIGH);
          busses_12V[i].enabled = true;
        }
        else
        {
          //disable bus
          Serial.println("Disabling Bus:");
          Serial.println(i);
          digitalWrite(busses_12V[i].EN_PIN,LOW);
          busses_12V[i].enabled = false;
        }
      }
      break;
    }
    case RC_POWERBOARD_30V_BUSENABLE_DATAID:
    {
      //expects data to be an array with a single uint8
      //checks each bit. Bit 0 is 12V, bit 1 is Rocket, bit 2 is Aux.
      for(int i = 0; i < 3; i++)
      {
        if(packet.data[0] & 1<<i) 
        {
          //enable bus
          digitalWrite(busses_30V[i].EN_PIN,HIGH);
          busses_30V[i].enabled = true;
        }
        else
        {
          //disable bus
          digitalWrite(busses_30V[i].EN_PIN,LOW);
          busses_30V[i].enabled = false;
        }
      }
      break;
    }
    case RC_POWERBOARD_VACUUM_ENABLE_DATAID:
    {
      //expects data to be an array with a single uint8
      //checks first bit.
      if(packet.data[0] & 1) 
      {
        //enable bus
        digitalWrite(vac.EN_PIN,HIGH);
        vac.enabled = true;
      }
      else
      {
        //disable bus
        digitalWrite(vac.EN_PIN,LOW);
        vac.enabled = false;
      }
      break;
    }
    default:
    {
      Serial.println("Unfamiliar code");
    }
  }

  


  //Motor current output & bus status telemetry
  //motor currents & related variables
  float curr_motor[6] = {0,0,0,0,0,0};
  float temp;
  float real;
  //Bus status telemetry
  uint8_t en_motor = 0;
  for(int i = 0; i < 6; i++)
  {
    //current sensing and calculations
    temp = analogRead(busses_Motor[i].ISENSE_PIN);
    real = map(temp,ADCMin,ADCMax,ISENSEMin,ISENSEMax);
    real = (real*CURR_SCALER)/1000;
    curr_motor[i] = real;
    //motor status telemetry checks
    if(busses_Motor[i].enabled)
    {
      en_motor |= 1<<i;
    }
  }


  /*
  //NEEDS WORK
  //12V current output
  float curr_12V[2] = {0,0};
  for(int i = 0; i < 2; i++)
  {
    temp = analogRead(busses_12V[i].ISENSE_PIN);
    real = map(temp,ADCMin,ADCMax,ISENSEMin,ISENSEMax);
    real = (real*CURR_SCALER)/1000;
    curr_12V[i] = real;
    //12V status telemetry checks
    if(busses_Motor[i].enabled)
    {
      en_12V |= 1<<i;
    }
  }
  RoveComm.write(RC_POWERBOARD_12V_BUS_CURRENT_DATAID, RC_POWERBOARD_12V_BUS_CURRENT_DATACOUNT, curr_12V);
  */

  //30V current output
  float curr_30V[3] = {0,0,0};
  uint8_t en_30V = 0;
  for(int i = 0; i < 3; i++)
  {
    //current sensing and calculations
    temp = analogRead(busses_30V[i].ISENSE_PIN);
    real = map(temp,ADCMin,ADCMax,ISENSEMin,ISENSEMax);
    real = (real*CURR_SCALER)/1000;
    curr_30V[i] = real;
    //30V status telemetry checks
    if(busses_30V[i].enabled)
    {
      en_30V |= 1<<i;
    }
  }


  //Vac current output
  float curr_vac = analogRead(vac.ISENSE_PIN);
  uint8_t en_vac = (vac.enabled ? 1 : 0);
  curr_vac = map(curr_vac,ADCMin,ADCMax,ISENSEMin,ISENSEMax);
  curr_vac = (curr_vac*CURR_SCALER)/1000;


  //Overcurrent Protection - motors
  uint8_t error_motor = 0;
  for(int i = 0; i < 6; i++)
  {
    if(curr_motor[i] >= OCP_MOTOR)
    {
      //disable bus
      digitalWrite(busses_Motor[i].EN_PIN,LOW);
      busses_Motor[i].enabled = false;
      //save error
      error_motor |= 1<<i;
    }
  }
  /*ACTUATION AND LOGIC CURRENT SENSE DISABLED UNTIL REV 2
  uint8_t error_12V = 0;
  for(int i = 0; i < 3; i++)
  {
    if(curr_motor[i] >= OCP_MOTOR)
    {
      //disable bus
      digitalWrite(busses_12V[i].EN_PIN,LOW);
      busses_12V[i].enabled = false;
      //save error
      error_12V |= 1<<i;
    }
  }
  */
  uint8_t error_30V = 0;
  for(int i = 0; i < 6; i++)
  {
    if(curr_30V[i] >= OCP_MOTOR || (i==1 && curr_30V[i] >= OCP_RKT))
    {
      //disable bus
      digitalWrite(busses_30V[i].EN_PIN,LOW);
      busses_30V[i].enabled = false;
      //save error
      error_30V |= 1<<i;
    }
  }
  bool error_vac = false;
  if(curr_vac >= OCP_VAC)
  {
    //disable bus
    digitalWrite(vac.EN_PIN,LOW);
    vac.enabled = false;
    //save error
    error_vac = true;
  }

  
  if(error_motor /*|| error_12V*/ || error_30V || error_vac)
  {
    //NEED ERROR LED ACTIVATION ONCE THE LED WORKS
    if(error_motor)
    {
      RoveComm.write(RC_POWERBOARD_MOTOR_BUS_OVERCURRENT_DATAID, RC_POWERBOARD_MOTOR_BUS_OVERCURRENT_DATACOUNT, error_motor);
    }
    /*
    if(error_12V)
    {
      RoveComm.write(RC_POWERBOARD_12V_BUS_OVERCURRENT_DATAID, RC_POWERBOARD_12V_BUS_OVERCURRENT_DATACOUNT, error_12V);
    }
    */
    if(error_30V)
    {
      RoveComm.write(RC_POWERBOARD_30V_BUS_OVERCURRENT_DATAID, RC_POWERBOARD_30V_BUS_OVERCURRENT_DATACOUNT, error_30V);
      //check if the error is on Rockets
      if(error_30V & 1<<1)
      {
        //wait, then reenable
        delay(RESET_TIME);
        digitalWrite(busses_30V[1].EN_PIN,HIGH);
        busses_30V[1].enabled = true;
      }
    }
    if(error_vac)
    {
      RoveComm.write(RC_POWERBOARD_VACUUM_OVERCURRENT_DATAID, RC_POWERBOARD_VACUUM_OVERCURRENT_DATACOUNT, error_motor);
    }
  }

  //send the telemetry at the specified rate
  if((millis() - lastTelemTime) >= ROVECOMM_TELEM_RATE)
  {
    //write the telemetry as a batch
    Serial.println("Sending telemetry");
    RoveComm.write(RC_POWERBOARD_MOTOR_BUS_CURRENT_DATAID, RC_POWERBOARD_MOTOR_BUS_CURRENT_DATACOUNT, curr_motor);
    RoveComm.write(RC_POWERBOARD_MOTOR_BUSENABLED_DATAID, RC_POWERBOARD_MOTOR_BUSENABLED_DATACOUNT, en_motor);
    RoveComm.write(RC_POWERBOARD_VACUUM_CURRENT_DATAID, RC_POWERBOARD_VACUUM_CURRENT_DATACOUNT, curr_vac);
    RoveComm.write(RC_POWERBOARD_VACUUM_ENABLED_DATAID, RC_POWERBOARD_VACUUM_ENABLED_DATACOUNT, en_vac);
    lastTelemTime = millis();
  }
}