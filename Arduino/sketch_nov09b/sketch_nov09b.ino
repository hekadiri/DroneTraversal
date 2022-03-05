/* VL53L1X_W20.ino  code for E100-400 Pololu 32U4 Arduini LIDAR ranging  E100-400 W20 1/2/2020
 * 
 */

#define ALPHA 1.0f                            // Tunable IIR filter on data to mavlink
const uint8_t Skips = 30;                     // Skips = Number of readings to skip between printed outputs
                                              // Skips = 0 will turn off printing to console monitor
unsigned long const LoopTime = 47;            // loop time minimum (set to sum of SAMPLE + BUDGET)


/*
*********  no user servicable parts below here  ***************
*/
 
// Libraries and headers
#include <Wire.h>
#include <SoftwareSerial.h>
#include <VL53L1X.h>
#include "mavlink/common/mavlink.h"

// Communication parameters for I2C and serial
#define BAUD_RATE 57600
#define CLOCK_HZ 400000
SoftwareSerial ardupilotSerial( 10, 11 ); 

// Sensor configuration
#define N_SENSORS 4
#define XSHUT 4                            // XSHUT connects to arduino pin 4
#define START_ADDRESS 50
#define DISTANCE_MODE VL53L1X::Medium      // range to 2.6 meters on Medium
#define TIMEOUT_MS 50
#define TIME_BUDGET 33000                // VL53L1X time budget in microsec (20000 for short, 33000 for med, 50000 for long)
#define SAMPLE_TIME 14                   // intersample sample time - start at 11 (not what docs say but they appear to be wrong

// MAVLink configuration values
#define SYSTEM_ID 1
#define COMPONENT_ID 10
#define MIN_DISTANCE 0
#define MAX_DISTANCE 2600                 // Short: 1360, Medium: 2900, Long: 3600 in mm, change to match DISTANCE_MODE
#define COVARIANCE 0

// Mavlink sensor orientation enumerated type array
uint8_t orientations[N_SENSORS] = 
{
  MAV_SENSOR_ROTATION_NONE,           // forward-facing
  MAV_SENSOR_ROTATION_YAW_90,         // right-facing
  MAV_SENSOR_ROTATION_YAW_180,        // backward-facing
  MAV_SENSOR_ROTATION_YAW_270         // left-facing
}; 

// Globals

//VL53LX1 variables 
static const uint8_t AddressDefault = 0b0101001;
VL53L1X sensor[N_SENSORS];                    // vl53l1x is a library defined type, create an array of 4 of them
uint16_t distance[N_SENSORS] = {0};           // distance value set

// Loop Timing variables
unsigned long lastlooptime_ms = 0;
float frequency = 0.0;                        // output sampling frequency
unsigned long lastReadTime[N_SENSORS] = {0};       // time stamp array of last good report for each sensor

// Local serial monitor print control
uint16_t pcounter = 0;                        // print counter for skipping outputs
boolean printFlag = true;                     // console serial printer control flag

//FUNCTIONS

void send_distance(uint16_t lightSensor, uint8_t channel )           // Send ranging measurement to arduPilot
{
  mavlink_message_t msg;
  mavlink_msg_distance_sensor_pack( 
    SYSTEM_ID,
    COMPONENT_ID,
    &msg,
    (uint32_t)millis(),
    MIN_DISTANCE,
    MAX_DISTANCE,
    dist,
    MAV_DISTANCE_SENSOR_INFRARED,
    channel,
    MAV_SENSOR_PITCH_90,
    COVARIANCE
  );
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  ardupilotSerial.write(buf, len);
}

void printGoodData( uint8_t channel, uint16_t lightSensor, float freq  )
{
  Serial.print( "Channel ");
  Serial.print( channel + 1 );
  Serial.print( " distance = " );
  Serial.print( lightSensor );
  Serial.print(" mm, Freq = ");
  Serial.print( freq );
  Serial.print(" Hz");
}

void printBadData(String errMsg, uint8_t channel )
{
  Serial.print( "Channel ");
  Serial.print( channel + 1 );
  Serial.print( " " );
  Serial.print( errMsg );
  Serial.print( ", Reporting dist =  ");
  Serial.print( MAX_DISTANCE );
  Serial.print( " mm."); 
}

void printBadI2C( uint8_t channel )
{
  Serial.print( "had to reset I2C for channel: ");
  Serial.println( channel );
}

void printFilteredDistance( uint16_t lightSensor )
{
  Serial.print( ", filtered dist = " );
  Serial.println( lightSensor );
}

void printAlpha()
{
  Serial.print("ALPHA is set to: ");
  Serial.print( ALPHA );
  Serial.println('\n');    
}

void initializeI2CAddresses(uint8_t channel) 
{
  digitalWrite( XSHUT + channel, LOW );
  delay( 10 );

  digitalWrite( XSHUT + channel, HIGH );
  delay( 10 );

  sensor[channel].setAddress(AddressDefault);
  sensor[channel].setTimeout( TIMEOUT_MS );                   // if a sensor fails, punt
  if ( !sensor[channel].init() ) 
  {
    Serial.print( "Failed to initialize sensor " );
    Serial.print( channel + 1 );
    Serial.println();
    Serial.print( "Retrying..." );
    Serial.println();
    return;
  }
  Serial.print("Connected!");
  Serial.println();    
  sensor[channel].setAddress( START_ADDRESS + channel );      // Set sensor's address and ranging mode
  sensor[channel].setDistanceMode( DISTANCE_MODE );
  sensor[channel].setMeasurementTimingBudget( TIME_BUDGET );
  sensor[channel].startContinuous( SAMPLE_TIME );
  Serial.print("Made it here\n");
}

/**** Arduino Setup and Loop  *****************************************/

void setup()
{     
  if ( Skips )                                                  // Initialize UART and SW serial
  {
  Serial.begin( BAUD_RATE );                                    // don't setup "Serial" if print is off
  }
  
  ardupilotSerial.begin( BAUD_RATE );
  
  Wire.begin();                                                  // Initialize I2C
  Wire.setClock( CLOCK_HZ );
 
  for ( uint8_t channel = 0; channel < N_SENSORS; channel++  ) pinMode( XSHUT + channel, OUTPUT );   // Disable all sensors
  for ( uint8_t channel = 0; channel < N_SENSORS; channel++  ) digitalWrite( XSHUT + channel, LOW ); // Reset
  for ( uint8_t channel = 0; channel < N_SENSORS; channel++  ) initializeI2CAddresses( channel );
}

void loop()
{
 // Serial.println( LoopTime - lastlooptime_ms );                      // diagnostic print
  if ( LoopTime > lastlooptime_ms ) delay( LoopTime - lastlooptime_ms );
  unsigned long time1 = millis();

  if ( !Skips )
  {
    printFlag = false;                          // will never print
  }
  else
  {
    printFlag = !( Skips - ++pcounter );         // will print once every Skips loops
  }
  if ( printFlag ) pcounter = 0;                 // reset counter 
  
  for( uint8_t channel = 0; channel < N_SENSORS; channel++ )
  {
    Wire.beginTransmission(START_ADDRESS + channel);
    if(Wire.endTransmission() != 0)
    {
      if ( printFlag ) printBadI2C( channel );  
      initializeI2CAddresses( channel );
      continue;
    }
    
  if (!sensor[channel].dataReady())                  // skip sensor if not ready
  {
    Serial.print( "sensor not ready channel: " );     // diagnostic print
    Serial.println( channel );                        // diagnositic print
    continue;
  }
  uint16_t lightSensor = sensor[channel].read();           // get a value
    
  if (sensor[channel].timeoutOccurred() ) 
  { 
   Serial.print(" TIMEOUT");                   // if a sensor times out, ADJUST TIME_BUDGET or SAMPLE_TIME 
  }
  
  if (sensor[channel].ranging_data.range_status == VL53L1X::RangeValid)    // got good data :)
  {
    uint32_t currentReadTime = millis();
    frequency = ( 1000.0 / ( currentReadTime - lastReadTime[channel] ) );
    lastReadTime[channel] = currentReadTime;
        
    if ( printFlag ) printGoodData( channel, lightSensor, frequency );
  }
  else                                            // don't got good data :(
  {
    lightSensor = MAX_DISTANCE;                          //if bad data always set to MAX   
    if ( printFlag )
    {   
      String errorMessage = VL53L1X::rangeStatusToString(sensor[channel].ranging_data.range_status);
      printBadData( errorMessage, channel );  
    }           
  }                                                                                                                  // Calculate Filtered result and send data   
  distance[channel] = (uint16_t)(ALPHA * (float)lightSensor + ( 1.0f - ALPHA ) * (float)distance[channel] );
  send_distance( distance[channel], channel );
  if ( printFlag ) printFilteredDistance( distance[ channel ] );
}    
  if ( printFlag ) printAlpha();                           // display ALPHA with data  

  lastlooptime_ms = millis() - time1;                      // how much time this loop took 
}
