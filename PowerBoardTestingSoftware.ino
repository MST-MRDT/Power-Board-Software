//RoveWare Powerboard ACS_722 Interface
//
// Created for Zenith by: Judah Schad, jrs6w7
// Altered for Gryphon by: Jacob Lipina, jrlwd5
//
// Using http://www.digikey.com/product-detail/en/allegro-microsystems-llc/ACS722LLCTR-40AU-T/620-1640-1-ND/4948876
//
// Standard C
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

//Energia
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

//RoveWare
#include <RoveBoard.h>
#include <RoveEthernet.h>
#include <RoveComm.h>

//RED can toggle the bus by bool
const uint16_t NO_ROVECOMM_MESSAGE          = 0;

const uint16_t M1_CURRENT_READING           = 1104;
const uint16_t M2_CURRENT_READING           = 1105;
const uint16_t M3_CURRENT_READING           = 1106;
const uint16_t M4_CURRENT_READING           = 1107;
const uint16_t M5_CURRENT_READING           = 1108;
const uint16_t M6_CURRENT_READING           = 1109;
const uint16_t M7_CURRENT_READING           = 1110;
const uint16_t EXTRA_12V_CURRENT_READING    = 1114; 
const uint16_t ACT_12V_CURRENT_READING      = 1115; 
const uint16_t LOGIC_12V_CURRENT_READING    = 1116; 
const uint16_t COM_12V_CURRENT_READING      = 1117;
const uint16_t PACK_VOLTAGE_READING         = 1120;

//const uint16_t ROVER_POWER_RESET            = 1041; //reset on power board buses no longer necessary since id 1041 now tells bms to turn off all power to rover. This accomplishes the same thing.
const uint16_t POWER_BUS_ENABLE             = 1088;
const uint16_t POWER_BUS_DISABLE            = 1089; 
const uint16_t POWER_BUS_OVER_CURRENT       = 1090;   

const uint8_t BUS_M1_ON_OFF                 = 0;
const uint8_t BUS_M2_ON_OFF                 = 1;
const uint8_t BUS_M3_ON_OFF                 = 2;
const uint8_t BUS_M4_ON_OFF                 = 3;
const uint8_t BUS_M5_ON_OFF                 = 4;
const uint8_t BUS_M6_ON_OFF                 = 5;
const uint8_t BUS_M7_ON_OFF                 = 6;
const uint8_t BUS_12V_EXTRA_ON_OFF          = 7;
const uint8_t BUS_12V_ACT_ON_OFF            = 8;
const uint8_t BUS_12V_LOGIC_ON_OFF          = 9;
const uint8_t BUS_12V_COM_ON_OFF            = 10;
const uint8_t BUS_12V_COM_LOGIC_ON_OFF      = 11;
const uint8_t FAN_ON_OFF                    = 12; //power board fan

const uint16_t BMS_PACK_CURRENT             = 1072;
const uint16_t BMS_PACK_OVER_CURRENT        = 1076;  //notification sent if pack overcurrents and shuts down
const uint16_t BMS_V_CHECK_ARRAY            = 1073;  //Total pack voltage
const uint16_t BMS_TEMP1                    = 1074;  
//const uint16_t BMS_TEMP2                    = 1075; //Not yet recieveing 2nd temp sensing from BMS
const uint16_t BMS_UNDER_VOLTAGE            = 1077;  //notification sent if a cell drops below a specified voltage. Value indicates cell
const uint16_t CELL_1_VOLTAGE               = 1056;
const uint16_t CELL_2_VOLTAGE               = 1057;
const uint16_t CELL_3_VOLTAGE               = 1058;
const uint16_t CELL_4_VOLTAGE               = 1059;
const uint16_t CELL_5_VOLTAGE               = 1060;
const uint16_t CELL_6_VOLTAGE               = 1061;
const uint16_t CELL_7_VOLTAGE               = 1062;
const uint16_t CELL_8_VOLTAGE               = 1063;

const uint16_t BMS_COMMAND                  = 1042;
const uint16_t BATT_PACK_OFF                = 1040;
const uint16_t BATT_PACK_RESET              = 1041;
const uint16_t BATT_FANS_ON_OFF             = 1078;
const uint16_t SOUND_BUZZER                 = 1079; //Might be needed if we want to honk at other rovers.

const int ROVER_POWER_RESET_DELAY           = 3000;


//Rovecomm :: RED packet :: data_id and data_value with number of data bytes size
uint16_t data_id       = 0;
size_t   data_size     = 0; 
uint8_t  data_value    = 0;

const int ROVECOMM_DELAY = 10;

//////////////////////////////////////////////Pinmap
// Control Pins

//0 MIN VOLT    3.036 MAX_VOLT  RESISTOR DIVIDER = 11;      
//const int BATTERYPACK_CNTRL  = 11;
const int EXTRA_CNTRL         = 17;   
const int ACT_CNTRL           = 13;
const int LOGIC_CNTRL         = 18; //created new
const int COM_CNTRL           = 34; //created new
const int COM_LOGIC_CNTRL     = 11; //created new
const int M1_CNTRL            = 58;
const int M2_CNTRL            = 57;
const int M3_CNTRL            = 74;
const int M4_CNTRL            = 53;
const int M5_CNTRL            = 73;
const int M6_CNTRL            = 72;
const int M7_CNTRL            = 71;
const int FAN_CNTRL           = 42;


// Sensor Volts/Amps Readings Pins
const int EXTRA_AMPS          = 26;
const int ACT_AMPS            = 25;
const int LOGIC_AMPS          = 24;
const int COM_AMPS            = 23;
const int M1_AMPS             = 68;
const int M2_AMPS             = 67;
const int M3_AMPS             = 66; 
const int M4_AMPS             = 45;
const int M5_AMPS             = 65;
const int M6_AMPS             = 64;
const int M7_AMPS             = 63;
const int PACK_VOLTAGE        = 6;

//////////////////////////////////////////////RoveBoard
// Tiva1294C RoveBoard Specs
const float VCC                 = 3.3;       //volts
const float ADC_MAX             = 4096;      //bits
const float ADC_MIN             = 0;         //bits
float adc_reading = 0;

//////////////////////////////////////////////Sensor
// ACS722LLCTR-40AU-T IC Sensor Specs 
const float SENSOR_SENSITIVITY   = 0.066;    //volts/amp
const float SENSOR_SCALE         = 0.1;      //volts/amp
const float SENSOR_BIAS          = VCC * SENSOR_SCALE;

const float CURRENT_MAX          = (VCC - SENSOR_BIAS) / SENSOR_SENSITIVITY;
const float CURRENT_MIN          = -SENSOR_BIAS / SENSOR_SENSITIVITY;
float current_reading            = 0;
bool com_over_current            = 0;
float time1                      = 0;

const float VOLTS_MIN            = 0;
const float VOLTS_MAX            = 40;
float voltage_reading            = 0;

const int DEBOUNCE_DELAY = 10;

//Safest Test pin
const int ESTOP_12V_COM_LOGIC_MAX_AMPS_THRESHOLD = 5;
const int ESTOP_12V_EXTRA_ACT_MAX_AMPS_THRESHOLD = 15;   
const int ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD = 22;

// Checks the pin for bouncing voltages to avoid false positives
bool singleDebounce(int bouncing_pin, int max_amps_threshold)
{
  int adc_threshhold = map(max_amps_threshold, CURRENT_MIN, CURRENT_MAX, ADC_MIN, ADC_MAX);
  
  if( analogRead(bouncing_pin) > adc_threshhold)
  {  
    delay(DEBOUNCE_DELAY);
    
    if( analogRead(bouncing_pin) > adc_threshhold)
    {
       return true;
    }//end if
  }// end if 
  return false;
}//end fntcn

/////////////////////////////////////////Data from BMS
uint8_t cell_voltages_byte[12];
float cell_voltages[8]; //each space in the array is a cell voltage

union txable_float {        
    float f;
    unsigned char ch[4];
};

union txable_float v_check_array;
union txable_float bms_temp;      //may need to add another temp variable if we use two temp sensors.
union txable_float pack_current;

int num_loops = 0; //used to track number of times the main loop has completed a cycle.

///////////////////////////////////////////////Implementation
float mapFloats(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}//end fnctn

///scales the input value x from analog input range (0 to 3.3) to actual values (Pack voltage or current)
float scale(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//////////////////////////////////////////////Powerboard Begin
// the setup routine runs once when you press reset
void setup() 
{
  // Control Pins are outputs
  pinMode(EXTRA_CNTRL, OUTPUT);
  pinMode(ACT_CNTRL, OUTPUT);
  pinMode(COM_LOGIC_CNTRL, OUTPUT);
  pinMode(COM_CNTRL, OUTPUT);
  pinMode(LOGIC_CNTRL, OUTPUT);
  pinMode(M1_CNTRL, OUTPUT);
  pinMode(M2_CNTRL, OUTPUT);
  pinMode(M3_CNTRL, OUTPUT); 
  pinMode(M4_CNTRL, OUTPUT);
  pinMode(M5_CNTRL, OUTPUT);
  pinMode(M6_CNTRL, OUTPUT);
  pinMode(M7_CNTRL, OUTPUT);
  pinMode(FAN_CNTRL, OUTPUT);

  pinMode(EXTRA_AMPS, INPUT);
  pinMode(ACT_AMPS, INPUT);
  pinMode(COM_AMPS, INPUT);
  pinMode(LOGIC_AMPS, INPUT);
  pinMode(M1_AMPS, INPUT);
  pinMode(M2_AMPS, INPUT);
  pinMode(M3_AMPS, INPUT);
  pinMode(M4_AMPS, INPUT);
  pinMode(M5_AMPS, INPUT);
  pinMode(M6_AMPS, INPUT);
  pinMode(M7_AMPS, INPUT);
  pinMode(PACK_VOLTAGE, INPUT);
  
  digitalWrite(EXTRA_CNTRL, LOW);
  digitalWrite(ACT_CNTRL, LOW);
  digitalWrite(COM_LOGIC_CNTRL, LOW);
  digitalWrite(COM_CNTRL, LOW);
  digitalWrite(LOGIC_CNTRL, LOW);
  digitalWrite(M1_CNTRL, LOW);
  digitalWrite(M2_CNTRL, LOW);
  digitalWrite(M3_CNTRL, LOW);
  digitalWrite(M4_CNTRL, LOW);
  digitalWrite(M5_CNTRL, LOW);
  digitalWrite(M6_CNTRL, LOW);
  digitalWrite(M7_CNTRL, LOW);
  digitalWrite(FAN_CNTRL, LOW);
  
  // Turn on everything when we begin
  delay(ROVER_POWER_RESET_DELAY);
  
  digitalWrite(EXTRA_CNTRL, HIGH);
  digitalWrite(ACT_CNTRL, HIGH);
  digitalWrite(COM_LOGIC_CNTRL, HIGH);
  digitalWrite(COM_CNTRL, HIGH);
  digitalWrite(LOGIC_CNTRL, HIGH);
  digitalWrite(M1_CNTRL, HIGH);
  digitalWrite(M2_CNTRL, HIGH);
  digitalWrite(M3_CNTRL, HIGH);
  digitalWrite(M4_CNTRL, HIGH);
  digitalWrite(M5_CNTRL, HIGH);
  digitalWrite(M6_CNTRL, HIGH);
  digitalWrite(M7_CNTRL, HIGH);
  digitalWrite(FAN_CNTRL, HIGH);
  
  roveComm_Begin(192, 168, 1, 132);
  Serial7.begin(115200); //corresponds to pair of Rx and Tx pins on the Tiva that pb uses to communicate with bms.
  Serial.begin(9600);
  delay(500);
}//end setup

//Loop
//
/////////////////////////////////////////////Powerboard Loop Forever
void loop() 
{ 
  if( singleDebounce(EXTRA_AMPS, ESTOP_12V_EXTRA_ACT_MAX_AMPS_THRESHOLD) ) //checks current reading and if too high, sends error msg to base
  {                                                                        //station then turns off the bus.      
    (POWER_BUS_OVER_CURRENT, sizeof(BUS_12V_EXTRA_ON_OFF), &BUS_12V_EXTRA_ON_OFF);
    digitalWrite(EXTRA_CNTRL, LOW);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if( singleDebounce(ACT_AMPS, ESTOP_12V_EXTRA_ACT_MAX_AMPS_THRESHOLD) )
  {
    (POWER_BUS_OVER_CURRENT, sizeof(BUS_12V_ACT_ON_OFF), &BUS_12V_ACT_ON_OFF);
    digitalWrite(ACT_CNTRL, LOW);
    delay(ROVECOMM_DELAY);
  }//end if

  if( singleDebounce(LOGIC_AMPS, ESTOP_12V_COM_LOGIC_MAX_AMPS_THRESHOLD) )
  {
    (POWER_BUS_OVER_CURRENT, sizeof(BUS_12V_LOGIC_ON_OFF), &BUS_12V_LOGIC_ON_OFF);                                                                         
    digitalWrite(LOGIC_CNTRL, LOW);                                            
    delay(ROVECOMM_DELAY);                                                    
  }//end if

  if( singleDebounce(COM_AMPS, ESTOP_12V_COM_LOGIC_MAX_AMPS_THRESHOLD) )
  {
    (POWER_BUS_OVER_CURRENT, sizeof(BUS_12V_COM_ON_OFF), &BUS_12V_COM_ON_OFF);
    digitalWrite(COM_CNTRL, LOW);
    time1 = millis();
    com_over_current = 1; //sets com_over_current to true
    delay(ROVECOMM_DELAY);
  }//end if
  
  if(com_over_current = 1)                 //When the communication bus overcurrents, the bus is turned off and our communication with the rover is severed. We would not
  {                                        //be able to control the rover from base station in any way. This if statement is here to turn the com bus back on 10seconds after 
    if(millis()>=(time1+10000))            //it is turned off in case the overcurrent was just a random spike. If there actually is a short in the bus, the bus will turn itself 
      {                                    //off again.
        digitalWrite(COM_CNTRL,HIGH);
        com_over_current = 0;
      }
  }//end if

  if( singleDebounce(M1_AMPS, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) ) 
  {
    (POWER_BUS_OVER_CURRENT, sizeof(BUS_M1_ON_OFF), &BUS_M1_ON_OFF);
    digitalWrite(M1_CNTRL, LOW);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if( singleDebounce(M2_AMPS, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) )
  {
    (POWER_BUS_OVER_CURRENT, sizeof(BUS_M2_ON_OFF), &BUS_M2_ON_OFF);
    digitalWrite(M2_CNTRL, LOW);
    delay(ROVECOMM_DELAY);
  }//end if
  
   if(singleDebounce(M3_AMPS, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) )
  {
    (POWER_BUS_OVER_CURRENT, sizeof(BUS_M3_ON_OFF), &BUS_M3_ON_OFF);
    digitalWrite(M3_CNTRL, LOW);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if(singleDebounce(M4_AMPS, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) )
  {
    (POWER_BUS_OVER_CURRENT, sizeof(BUS_M4_ON_OFF), &BUS_M4_ON_OFF);
    digitalWrite(M4_CNTRL, LOW);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if( singleDebounce(M5_AMPS, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) )
  {
    (POWER_BUS_OVER_CURRENT, sizeof(BUS_M5_ON_OFF), &BUS_M5_ON_OFF);
    digitalWrite(M5_CNTRL, LOW);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if( singleDebounce(M6_AMPS, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) )
  {
    (POWER_BUS_OVER_CURRENT, sizeof(BUS_M6_ON_OFF), &BUS_M6_ON_OFF);
    digitalWrite(M6_CNTRL, LOW);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if(singleDebounce(M7_AMPS, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) )
  {
    (POWER_BUS_OVER_CURRENT, sizeof(BUS_M7_ON_OFF), &BUS_M7_ON_OFF);
    digitalWrite(M7_CNTRL, LOW);
    delay(ROVECOMM_DELAY);
  }//end if
  
  /////////////////////////////////////////////RED Control and Telem RoveComm
  
  //If there is no message, data_id gets set to zero
  roveComm_GetMsg(&data_id, &data_size, &data_value);
  switch (data_id) 
  {   
    case NO_ROVECOMM_MESSAGE: //data_id is 0; do nothing
      break; 
      
    case POWER_BUS_ENABLE: //data_id is 1088
      switch (data_value)
      { 
        case BUS_12V_EXTRA_ON_OFF:
          digitalWrite(EXTRA_CNTRL, HIGH);
          break;

        case BUS_12V_ACT_ON_OFF:
          digitalWrite(ACT_CNTRL, HIGH);
          break;

        case BUS_12V_COM_LOGIC_ON_OFF:
          digitalWrite(COM_LOGIC_CNTRL, HIGH);
          break;
          
        case BUS_12V_LOGIC_ON_OFF:
          digitalWrite(LOGIC_CNTRL, HIGH); 
          break;                                     

        case BUS_12V_COM_ON_OFF:
          digitalWrite(COM_CNTRL, HIGH);
          break;
          
        case BUS_M1_ON_OFF:
          digitalWrite(M1_CNTRL, HIGH);
          break;
          
        case BUS_M2_ON_OFF:
          digitalWrite(M2_CNTRL, HIGH);
          break;
          
        case BUS_M3_ON_OFF:
          digitalWrite(M3_CNTRL, HIGH);
          break;
          
        case BUS_M4_ON_OFF:
          digitalWrite(M4_CNTRL, HIGH);
          break;
          
        case BUS_M5_ON_OFF:
          digitalWrite(M5_CNTRL, HIGH);
          break;
          
        case BUS_M6_ON_OFF:
          digitalWrite(M6_CNTRL, HIGH);
          break;
          
        case BUS_M7_ON_OFF:
          digitalWrite(M7_CNTRL, HIGH);
          break;

        case FAN_ON_OFF:
          digitalWrite(FAN_CNTRL, HIGH);
          break;
          
        default:
          //Serial.println("Unrecognized data : 2");
          //Serial.println(data);
          break; 
     }//endswitch 
     break;  
 
    case POWER_BUS_DISABLE: //data_id id 1089
          switch (data_value)
          { 
            case BUS_12V_EXTRA_ON_OFF:
              digitalWrite(EXTRA_CNTRL, LOW);
              break;
            case BUS_12V_ACT_ON_OFF:
              digitalWrite(ACT_CNTRL, LOW);
              break;
            case BUS_12V_COM_LOGIC_ON_OFF:
              digitalWrite(COM_LOGIC_CNTRL, LOW);
              break;
              
            case BUS_12V_LOGIC_ON_OFF:
              digitalWrite(LOGIC_CNTRL, LOW);
              break;                                 
              
            case BUS_12V_COM_ON_OFF:
              digitalWrite(COM_CNTRL, LOW);
              break;
              
            case BUS_M1_ON_OFF:
              digitalWrite(M1_CNTRL, LOW);
              break;
              
            case BUS_M2_ON_OFF:
              digitalWrite(M2_CNTRL, LOW);
              break;
              
            case BUS_M3_ON_OFF:
              digitalWrite(M3_CNTRL, LOW);
              break;
              
            case BUS_M4_ON_OFF:
              digitalWrite(M4_CNTRL, LOW);
              break;
              
            case BUS_M5_ON_OFF:
              digitalWrite(M5_CNTRL, LOW);
              break;
              
            case BUS_M6_ON_OFF:
              digitalWrite(M6_CNTRL, LOW);
              break;
              
            case BUS_M7_ON_OFF:
              digitalWrite(M7_CNTRL, LOW);
              break;

            case FAN_ON_OFF:
              digitalWrite(FAN_CNTRL, LOW);
              break;
              
            default:
              //Serial.println("Unrecognized data : 3");
              //Serial.println(data);
              break; 
         }//endswitch 
         break;

    /*case ROVER_POWER_RESET: //data_id is 1041                 Reset not needed since BMS command to reset pack accomplishes the same thing.
                                                                This would be necessary if BMS communication was not implemented.
      digitalWrite(M1_CNTRL, LOW);
      digitalWrite(M2_CNTRL, LOW);
      digitalWrite(M3_CNTRL, LOW);
      digitalWrite(M4_CNTRL, LOW);
      digitalWrite(M5_CNTRL, LOW);
      digitalWrite(M6_CNTRL, LOW);
      digitalWrite(M7_CNTRL, LOW);  
                  
      digitalWrite(ACT_CNTRL, LOW);
      digitalWrite(EXTRA_CNTRL, LOW);
      digitalWrite(LOGIC_CNTRL, LOW);
      digitalWrite(COM_CNTRL, LOW);
      digitalWrite(COM_LOGIC_CNTRL, LOW);
      
      digitalWrite(FAN_CNTRL, LOW);
     
      delay(ROVER_POWER_RESET_DELAY);

      digitalWrite(EXTRA_CNTRL, HIGH);
      digitalWrite(ACT_CNTRL, HIGH);
      digitalWrite(COM_LOGIC_CNTRL, HIGH);
      digitalWrite(COM_CNTRL, HIGH);
      digitalWrite(LOGIC_CNTRL, HIGH);
    
      digitalWrite(M1_CNTRL, HIGH);
      digitalWrite(M2_CNTRL, HIGH);
      digitalWrite(M3_CNTRL, HIGH);
      digitalWrite(M4_CNTRL, HIGH);
      digitalWrite(M5_CNTRL, HIGH);
      digitalWrite(M6_CNTRL, HIGH);
      digitalWrite(M7_CNTRL, HIGH);

      digitalWrite(FAN_CNTRL, HIGH);
      break;*/

    case BATT_PACK_OFF: //data_id is 1040
        Serial7.write(1);
        //Serial.println("BMS shutdown");
        break;

    case BATT_PACK_RESET: //data_id is 1041
        Serial7.write(2);
        break;

    case BATT_FANS_ON_OFF: //data_id is 1078
          switch (data_value)
          {
            case '1':
              Serial7.write(3); //fans on
              break;

            case '0':
              Serial7.write(4); //fans off
              break;
          }

    case SOUND_BUZZER: //data_id is 1079            //here in case base station wants to honk at other rovers. May not end up using this case.
        switch (data_value)
          {
            case '1':
              Serial7.write(6); //buzzer on
              break;

            case '0':
              Serial7.write(7); //buzzer off
              break;
          }

    default:
      //Serial.println("Unrecognized data_id: 5");
      //Serial.println(data_id);
      break;
  }//endswitch

  adc_reading = analogRead(EXTRA_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(EXTRA_12V_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(ACT_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(ACT_12V_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);

  adc_reading = analogRead(LOGIC_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(LOGIC_12V_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);

  adc_reading = analogRead(COM_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(COM_12V_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M1_AMPS); 
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);  
  roveComm_SendMsg(M1_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);

  adc_reading = analogRead(M2_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M2_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M3_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M3_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M4_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M4_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M5_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M5_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M6_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M6_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M7_AMPS);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M7_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);

  adc_reading = analogRead(PACK_VOLTAGE);
  voltage_reading = scale(adc_reading, ADC_MIN, ADC_MAX, VOLTS_MIN, VOLTS_MAX);
  roveComm_SendMsg(PACK_VOLTAGE_READING, sizeof(voltage_reading), &voltage_reading);
  delay(ROVECOMM_DELAY);

  ////////////// BMS Communication /////////////////////////////////////////////////////////
  
  if (num_loops > 18 ) 
  {
    num_loops = 0;    //resets loops to zero to start count to 18 over again
    Serial7.write(5); //tells bms that I am ready to recieve data
  }

  if (Serial7.available() >= 24) //number of bytes I expect to recieve from bms
  { 
    for (int i=0; i < 4; i++)
    {
      pack_current.ch[i] = Serial7.read(); 
    }

    /*Serial.println(pack_current.ch[0]);           //used when testing to check if i recieve the correct bytes in the right order
    Serial.println(pack_current.ch[1],);
    Serial.println(pack_current.ch[2], HEX);
    Serial.println(pack_current.ch[3], HEX);*/
    
            //assuming least significant byte first
    //pack_current = ((pack_current_byte[0]) | (pack_current_byte[1] << 8) | (pack_current_byte[2] << 16) | (pack_current_byte[3] <<24));
    roveComm_SendMsg(BMS_PACK_CURRENT, sizeof(pack_current.f), &pack_current.f);
    delay(ROVECOMM_DELAY);

    for (int i=0; i < 4; i++)           
    {
      v_check_array.ch[i] = Serial7.read();
    }       
    
    //v_check_array = ((v_check_array_byte[0]) | (v_check_array_byte[1] << 8) | (v_check_array_byte[2] << 16) | (v_check_array_byte[3] <<24));
    roveComm_SendMsg(BMS_V_CHECK_ARRAY, sizeof(v_check_array.f), &v_check_array.f);
    delay(ROVECOMM_DELAY);

    for (int i=0; i < 4; i++)
    {
      bms_temp.ch[i] = Serial7.read();
    }  

    roveComm_SendMsg(BMS_TEMP1, sizeof(bms_temp.f), &bms_temp.f);
    delay(ROVECOMM_DELAY);

       
    /*Serial.println(pack_current.f);         //used when testing
    Serial.println(pack_current.ch[0], HEX);
    Serial.println(v_check_array.f);
    Serial.println(bms_temp.f);*/
     
  
    for (int i=0; i < 12; i++)
    {
      cell_voltages_byte[i] = Serial7.read();
    }   

    //The CVR0x registers, as read in from SPI, are 8 bits, but the actual voltages are 12-bit floats
    // spread out across multiple registers. Consult the LTC6803 datasheet (table 8, p. 23)
    //
    //The "split" occurs at the most significant nybble, then at the least significant, then most again, etc.
    cell_voltages[0] = cell_voltages_byte[0] | ((cell_voltages_byte[1] & 0x0F) << 8); //Cell 1
    roveComm_SendMsg(CELL_1_VOLTAGE, sizeof(cell_voltages[0]), &cell_voltages[0]);
    delay(ROVECOMM_DELAY);

    cell_voltages[1] = ((cell_voltages_byte[1] & 0xF0) >> 4) | (cell_voltages_byte[2] << 4); //Cell 2
    roveComm_SendMsg(CELL_2_VOLTAGE, sizeof(cell_voltages[1]), &cell_voltages[1]);
    delay(ROVECOMM_DELAY);

    cell_voltages[2] = cell_voltages_byte[3] | ((cell_voltages_byte[4] & 0x0F) << 8); //Cell 3
    roveComm_SendMsg(CELL_3_VOLTAGE, sizeof(cell_voltages[2]), &cell_voltages[2]);
    delay(ROVECOMM_DELAY);

    cell_voltages[3] = ((cell_voltages_byte[4] & 0xF0) >> 4) | (cell_voltages_byte[5] << 4); //Cell 4
    roveComm_SendMsg(CELL_4_VOLTAGE, sizeof(cell_voltages[3]), &cell_voltages[3]);
    delay(ROVECOMM_DELAY);

    cell_voltages[4] = cell_voltages_byte[6] | ((cell_voltages_byte[7] & 0x0F) << 8); //Cell 5
    roveComm_SendMsg(CELL_5_VOLTAGE, sizeof(cell_voltages[4]), &cell_voltages[4]);
    delay(ROVECOMM_DELAY);

    cell_voltages[5] = ((cell_voltages_byte[7] & 0xF0) >> 4) | (cell_voltages_byte[8] << 4);//Cell 6
    roveComm_SendMsg(CELL_6_VOLTAGE, sizeof(cell_voltages[5]), &cell_voltages[5]);
    delay(ROVECOMM_DELAY);

    cell_voltages[6] = cell_voltages_byte[9] | ((cell_voltages_byte[10] & 0x0F) << 8); //Cell 7
    roveComm_SendMsg(CELL_7_VOLTAGE, sizeof(cell_voltages[6]), &cell_voltages[6]);
    delay(ROVECOMM_DELAY);

    cell_voltages[7] = ((cell_voltages_byte[10] & 0xF0) >> 4) | (cell_voltages_byte[11] << 4);//Cell 8
    roveComm_SendMsg(CELL_8_VOLTAGE, sizeof(cell_voltages[7]), &cell_voltages[7]);
    delay(ROVECOMM_DELAY);

    

    //These still aren't normal voltages after putting them in floats; you need to do some extra processing.
    for(int k = 0; k < 8; k++)
      {
        cell_voltages[k] -= 512;
        cell_voltages[k] *= 1.5 * .001;//I don't yet know if this is correct; taken wholesale from solar car*/
      } 
  }
    /*Serial.println(cell_voltages[0]);
    Serial.println(cell_voltages[1]);
    Serial.println(cell_voltages[2]);
    Serial.println(cell_voltages[3]);
    Serial.println(cell_voltages[4]);
    Serial.println(cell_voltages[5]);
    Serial.println(cell_voltages[6]);
    Serial.println(cell_voltages[7]);

    
    

  }
  char incomingByte;
    if(Serial.available() > 0)
    {
      incomingByte = Serial.read();
      Serial.print(incomingByte);
      switch (incomingByte)                
      {   
        case '1':
          Serial7.write(3); //fans on
          Serial.println("fans on");
          break; 

        case '2':
          Serial7.write(4); //fans off
          Serial.println("fans off");
          break;

        case '3':
          digitalWrite(M7_CNTRL, LOW);
          delay(2000);
          digitalWrite(M7_CNTRL, HIGH);
          break;
      }
    }*/
  num_loops++;
}//end loop



