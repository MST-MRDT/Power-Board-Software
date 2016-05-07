//RoveWare Powerboard ACS_722 Interface
//
// Judah jrs6w7
//
// Using http://www.allegromicro.com/en/Products/Current-Sensor-ICs/Zero-To-Fifty-Amp-Integrated-Conductor-Sensor-ICs/ACS722.aspx
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

// RED udp device id by fourth octet
const int POWERBOARD_IP_DEVICE_ID   = 51;

// RED can toggle the bus by bool
const uint16_t NO_ROVECOMM_MESSAGE   = 0;

const uint16_t M1_CURRENT_READING        = 1104;
const uint16_t M2_CURRENT_READING        = 1105;
const uint16_t M3_CURRENT_READING        = 1106;
const uint16_t M4_CURRENT_READING        = 1107;
const uint16_t M5_CURRENT_READING        = 1108;
const uint16_t M6_CURRENT_READING        = 1109;
const uint16_t M7_CURRENT_READING        = 1110;
const uint16_t M8_CURRENT_READING        = 1111;
const uint16_t BUS_5V_CURRENT_READING    = 1112;
const uint16_t BUS_12V_CURRENT_READING   = 1113;

const uint16_t POWER_BUS_ENABLE      = 1088;
const uint16_t POWER_BUS_DISABLE     = 1089;
const uint16_t POWER_BUS_DISABLE     = 1090;

const uint8_t BUS_M1_ON_OFF         = 0;
const uint8_t BUS_M2_ON_OFF         = 1;
const uint8_t BUS_M3_ON_OFF         = 2;
const uint8_t BUS_M4_ON_OFF         = 3;
const uint8_t BUS_M5_ON_OFF         = 4;
const uint8_t BUS_M6_ON_OFF         = 5;
const uint8_t BUS_M7_ON_OFF         = 6;
const uint8_t BUS_M8_ON_OFF         = 7;
const uint8_t BUS_5V_ON_OFF         = 8;
const uint8_t BUS_12V_ON_OFF        = 9;

//Rovecomm :: RED packet :: data_id and data_value with number of data bytes size
uint16_t data_id       = 0;
size_t   data_size     = 0; 
uint16_t data_value    = 0;

const int ROVECOMM_DELAY = 10;

//////////////////////////////////////////////Pinmap
// Control Pins
//Todo const int BATTERYPACK_CNTRL?
const int BUS_5V_CNTRL_PP_2  = 11;
const int BUS_12V_CNTRL_PN_3 = 12;
const int M1_CNTRL_PK_7      = 71;
const int M2_CNTRL_PQ_1      = 52;
const int M3_CNTRL_PK_6      = 72;
const int M4_CNTRL_PP_3      = 53;
const int M5_CNTRL_PH_1      = 73;
const int M6_CNTRL_PH_0      = 74;
const int M7_CNTRL_PA_7      = 57;
const int M8_CNTRL_PP_5      = 58;

// Sensor Volts/Amps Readings Pins
//Todo const int BATTERYPACK_VOLTS_PE3?
const int BUS_5V_AMPS_PE_2   = 25;
const int BUS_12V_AMPS_PD_7  = 27;
const int M1_AMPS_PK_3       = 68;
const int M2_AMPS_PK_2       = 67 ;
const int M3_AMPS_PK_1       = 66; 
const int M4_AMPS_PD_4       = 45;
const int M5_AMPS_PK_0       = 65;
const int M6_AMPS_PB_5       = 64;
const int M7_AMPS_PB_4       = 63;
const int M8_AMPS_PD_2       = 42;



//////////////////////////////////////////////RoveBoard
// Tiva1294C RoveBoard Specs
const float VCC                 = 3.3;       //volts
const float ADC_MAX             = 4096;      //bits
const float ADC_MIN             = 0;         //bits
float adc_reading = 0;

//////////////////////////////////////////////Sensor
// ACS_722 IC Sensor Specs 
const float SENSOR_SENSITIVITY   = 0.125;    //volts/amp
const float SENSOR_SCALE         = 0.1;      //volts/amp
const float SENSOR_BIAS          = VCC * SENSOR_SCALE;

const float CURRENT_MAX          = (VCC - SENSOR_BIAS) / SENSOR_SENSITIVITY;
const float CURRENT_MIN          = -SENSOR_BIAS / SENSOR_SENSITIVITY;
float current_reading            = 0;

//Safest Test pin
const int ESTOP_5V_BUS_MAX_AMPS_THRESHOLD = 1;
const int ESTOP_12V_BUS_MAX_AMPS_THRESHOLD = 1;
const int ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD = 1;

// Checks the pin for bouncing voltages to avoid false positives
bool singleDebounce(int bouncing_pin, int max_threshold)
{
  int bouncing_reading = analogRead(bouncing_pin);
  if(bouncing_reading > max_threshold)
  }
    bouncing_reading = analogRead(bouncing_pin);
    if(bouncing_reading > max_threshold)
       return true;
    }//
  }//  
  return false;
}//end if


///////////////////////////////////////////////Implementation
float mapFloats(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}//end fnctn


//////////////////////////////////////////////Powerboard Begin
// the setup routine runs once when you press reset
void setup() 
{  
  // Control Pins are outputs
  pinMode(BUS_5V_CNTRL_PP_2, OUTPUT);
  pinMode(BUS_12V_CNTRL_PN_3, OUTPUT);
  
  pinMode(M1_CNTRL_PK_7, OUTPUT);
  pinMode(M2_CNTRL_PQ_1, OUTPUT);
  pinMode(M3_CNTRL_PK_6, OUTPUT); 
  pinMode(M4_CNTRL_PP_3, OUTPUT);
  pinMode(M5_CNTRL_PH_1, OUTPUT);
  pinMode(M6_CNTRL_PH_0, OUTPUT);
  pinMode(M7_CNTRL_PA_7, OUTPUT);
  pinMode(M8_CNTRL_PP_5, OUTPUT);
  
  // Turn on everything when we begin
  digitalWrite(BUS_5V_CNTRL_PP_2, HIGH);
  digitalWrite(BUS_12V_CNTRL_PN_3, HIGH);
  
  digitalWrite(M1_CNTRL_PK_7, HIGH);
  digitalWrite(M2_CNTRL_PQ_1, HIGH);
  digitalWrite(M3_CNTRL_PK_6, HIGH);
  digitalWrite(M4_CNTRL_PP_3, HIGH);
  digitalWrite(M5_CNTRL_PH_1, HIGH);
  digitalWrite(M6_CNTRL_PH_0, HIGH);
  digitalWrite(M7_CNTRL_PA_7, HIGH);
  digitalWrite(M8_CNTRL_PP_5, HIGH);
  
  Serial.begin(9600);
 
  roveComm_Begin(192, 168, 1, POWERBOARD_IP_DEVICE_ID);
  
}//end setup

//Loop
//
/////////////////////////////////////////////Powerboard Loop Forever
void loop() 
{ 
  if( singleDebounce(BUS_5V_AMPS_PE_2, ESTOP_5V_BUS_MAX_AMPS_THRESHOLD)) 
  {
    digitalWrite(BUS_5V_CNTRL_PP_2, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_5V_ON_OFF), &BUS_5V_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if( singleDebounce(BUS_12V_AMPS_PD_7, ESTOP_12V_BUS_MAX_AMPS_THRESHOLD))
  {
    digitalWrite(BUS_5V_CNTRL_PP_2, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_12V_ON_OFF), &BUS_12V_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if( singleDebounce(M1_AMPS_PK_3), ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) 
  {
    digitalWrite(M1_CNTRL_PK_7, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M1_ON_OFF), &BUS_M1_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if( singleDebounce(M2_AMPS_PK_2, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) 
  {
    digitalWrite(M2_CNTRL_PQ_1, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M2_ON_OFF), &BUS_M2_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
   if(singleDebounce(M3_AMPS_PK_1, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) 
  {
    digitalWrite(M3_AMPS_PK_1, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M3_ON_OFF), &BUS_M3_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if(singleDebounce(M4_AMPS_PD_4, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) 
  {
    digitalWrite(M4_CNTRL_PP_3, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M4_ON_OFF), &BUS_M4_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if( singleDebounce(M5_AMPS_PK_0, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD))
  {
    digitalWrite(M5_CNTRL_PH_1, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M5_ON_OFF), &BUS_M5_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if( singleDebounce(M6_AMPS_PB_5, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD) 
  {
    digitalWrite(M6_CNTRL_PH_0, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M6_ON_OFF), &BUS_M6_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if(singleDebounce(M7_AMPS_PB_4, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD))
  {
    digitalWrite(M7_CNTRL_PA_7, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M7_ON_OFF), &BUS_M7_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  if(singleDebounce(M8_AMPS_PD_2, ESTOP_MOTOR_BUS_MAX_AMPS_THRESHOLD))
  {
    digitalWrite(M8_CNTRL_PP_5, LOW);
    roveComm_SendMsg(POWER_BUS_OVER_CURRENT, sizeof(BUS_M8_ON_OFF), &BUS_M8_ON_OFF);
    delay(ROVECOMM_DELAY);
  }//end if
  
  /////////////////////////////////////////////RED Control and Telem RoveComm

  //If there is no message data_id gets set to zero
  roveComm_GetMsg(&data_id, &data_size, &data_value);
  
  switch (data_id) 
  {   
    //Don't do anything for data_id zero 
    case NO_ROVECOMM_MESSAGE:
      break;
      
    case BUS_5V_ON_OFF:
      digitalWrite(BUS_5V_CNTRL_PP_2, (bool)data_value);
      break;
    case BUS_12V_ON_OFF:
      digitalWrite(BUS_12V_CNTRL_PN_3, (bool)data_value);
      break;
    case BUS_M1_ON_OFF:
      digitalWrite(M1_CNTRL_PK_7, (bool)data_value);
      break;
    case BUS_M2_ON_OFF:
      digitalWrite(M2_CNTRL_PQ_1, (bool)data_value);
      break;
    case BUS_M3_ON_OFF:
      digitalWrite(M3_CNTRL_PK_6, (bool)data_value);
      break;
    case BUS_M4_ON_OFF:
      digitalWrite(M4_CNTRL_PP_3, (bool)data_value);
      break;
    case BUS_M5_ON_OFF:
      digitalWrite(M5_CNTRL_PH_1, (bool)data_value);
      break;
    case BUS_M6_ON_OFF:
      digitalWrite(M6_CNTRL_PH_0, (bool)data_value);
      break;
    case BUS_M7_ON_OFF:
      digitalWrite(M7_CNTRL_PA_7, (bool)data_value);
      break;
    case BUS_M8_ON_OFF:
      digitalWrite(M8_CNTRL_PP_5, (bool)data_value);
      break;
      
    default:
      //Serial.print("Unrecognized data_id :");
      //Serial.println(data_id);
      break; 
  }//endswitch 
 
  adc_reading = analogRead(BUS_5V_AMPS_PE_2);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX); 
  roveComm_SendMsg(GPS_LAT_LON_DATA_ID, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(BUS_12V_AMPS_PD_7);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(GPS_LAT_LON_DATA_ID, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M1_AMPS_PK_3);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);  
  roveComm_SendMsg(M1_CURRENT_READING, sizeof(current_reading), &current_reading);
  
  adc_reading = analogRead(M2_AMPS_PK_2);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M1_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M3_AMPS_PK_1);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M1_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M4_AMPS_PD_4);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M1_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M5_AMPS_PK_0);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M1_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M6_AMPS_PB_5);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M1_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M7_AMPS_PB_4);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M1_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  adc_reading = analogRead(M8_AMPS_PD_2);
  current_reading = mapFloats(adc_reading, ADC_MIN, ADC_MAX, CURRENT_MIN, CURRENT_MAX);
  roveComm_SendMsg(M1_CURRENT_READING, sizeof(current_reading), &current_reading);
  delay(ROVECOMM_DELAY);
  
  
}//end loop


/*Developing
//Todo Cameron and Mike 
//////////////////////////////////////////////Hardware Calibration
const int ANALOG_DEBOUNCE_TIME_MICROS = 250;
const int ANALOG_TRY_COUNT = 250;

const int ANALOG_ACCEPTABLE_DRIFT = 250;

const int DIGITAL_DEBOUNCE_TIME_MICROS = 250;
const int DIGITAL_TRY_COUNT = 250; 

const int PIN_TOO_NOISY = -1;

///////////////////////////////////////////////Implementation
int analogDebounce(int bouncing_pin)
{    
  // Count the bounces
  int analog_trend_count = 0;
  bool analog_reading = analogRead(bouncing_pin);  
  
  // Read a bouncing pin and save the state
  bool last_analog_reading = analog_reading;   
  
  // Get timestamp from the system clock counter
  unsigned long system_time_micros = micros(); 
 
 // Spin for a max of millisec
  while(system_time_micros != ( micros()  + ANALOG_DEBOUNCE_TIME_MICROS) )
  {
    analog_reading = analogRead(bouncing_pin);
    
    if( analog_trend_count && (abs(analog_reading - last_analog_reading) < ANALOG_ACCEPTABLE_DRIFT)  )    
    {
      analog_trend_count++;
    }//end if
    
    if( analog_trend_count && (abs(analog_reading - last_analog_reading) > ANALOG_ACCEPTABLE_DRIFT)  )    
    {
       analog_trend_count--; 
       last_analog_reading = last_analog_reading;
    }//end if
  
    if(analog_trend_count > ANALOG_TRY_COUNT)
    {   
      
      return analog_reading;   
    }else{         
      
      last_analog_reading = analog_reading;
    }//end else
  }//end while
  
  return PIN_TOO_NOISY;
}//end functn*/
