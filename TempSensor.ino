/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Aaron Andrus (kb3gzo)
 * 
 * DESCRIPTION
 * This sketch powers the Battery Powered Temperature Sensor Board (v1.0) designed by kb3gzo
 * The sensor board can be powered by either 1 CR2032 (3V) or 2xCR2016 (6V) it's using a ATMEGA328P-AU running at 8MHz
 * at either 3.3V or 5V (selectable by soldering jumpers)
 *
 * Includes provisions for DHT11/22, DS18B20, Motion and I2C breakout
 * There is also a 4M spi flash, FTDI and ISCP headers for easy programming along with a NRF24L01 connector
 * The board can be programmed with the standard Arduino bootolader or one of the supported MySensors bootloaders
 *******************************/
 
#include <MySensor.h>  
#include <DHT.h>  
#include <Wire.h>
#include <SPI.h>
#include "utility/SPIFlash.h"
#include <EEPROM.h>  
#include <RunningAverage.h>
#include <avr/power.h>
#include <DallasTemperature.h>
#include <OneWire.h>

//Define Sleep Time
unsigned long SLEEP_TIME = 30000; // Sleep time between reads (in milliseconds)

// Uncomment the line belows to activate various sensors
#define BATT_SENSOR    199
#define MOTION_SENSOR 1
#define DHT_SENSOR 1
#define DALLAS_SENSOR 1


#define RELEASE "1.2"
#define AVERAGES 2

//Define DHT Sensor
#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define HUMIDITY_SENSOR_DIGITAL_PIN 3

#ifdef DALLAS_SENSOR
#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No
#define ONE_WIRE_BUS 3 // Pin where dallase sensor is connected 
#define MAX_ATTACHED_DS18B20 16
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 

#endif

#ifdef MOTION_SENSOR
#define DIGITAL_INPUT_SENSOR 3   // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
#define INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)
#define CHILD_ID 3   // Id of the sensor child
#endif


// How many milli seconds between each measurement
#define MEASURE_INTERVAL 60000

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to the controller
#define FORCE_TRANSMIT_INTERVAL 30 

// When MEASURE_INTERVAL is 60000 and FORCE_TRANSMIT_INTERVAL is 30, we force a transmission every 30 minutes.
// Between the forced transmissions a tranmission will only occur if the measured value differs from the previous measurement

// HUMI_TRANSMIT_THRESHOLD tells how much the humidity should have changed since last time it was transmitted. Likewise with
// TEMP_TRANSMIT_THRESHOLD for temperature threshold.
#define HUMI_TRANSMIT_THRESHOLD 0.5
#define TEMP_TRANSMIT_THRESHOLD 0.5

SPIFlash flash(8, 0x1F65);


MySensor gw;

// Sensor messages
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

#ifdef BATT_SENSOR
MyMessage msgBatt(BATT_SENSOR, V_VOLTAGE);
#endif

#ifdef DALLAS_SENSOR
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors=0;
boolean receivedConfig = false;
MyMessage msg(0,V_TEMP);
#endif

#ifdef MOTION_SENSOR
MyMessage msg(CHILD_ID, V_TRIPPED);
#endif

// Global settings
int measureCount = 0;
int sendBattery = 0;
boolean isMetric = true;
boolean highfreq = true;

// Storage of old measurements
float lastTemperature = -100;
int lastHumidity = -100;
long lastBattery = -100;

RunningAverage raHum(AVERAGES);

//Start DHT sensor
DHT dht;

/****************************************************
 *
 * Setup code 
 *
 ****************************************************/
void setup()  
{ 
  gw.begin();
  dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN); 

  // Send the Sketch Version Information to the Gateway
  gw.sendSketchInfo("SensorBoard", "1.0");

  pinMode(DIGITAL_INPUT_SENSOR, INPUT);      // sets the motion sensor digital pin as input

  // Register all sensors to gw (they will be created as child devices)
  gw.present(CHILD_ID_HUM, S_HUM);
  gw.present(CHILD_ID_TEMP, S_TEMP);

#ifdef DALLAS_SENSOR
  sensors.begin();      // Startup up the OneWire library
  sensors.setWaitForConversion(false);    // requestTemperatures() will not block current thread
  numSensors = sensors.getDeviceCount();    // Fetch the number of attached temperature sensors  

  // Present all sensors to controller
     #ifdef DHT_SENSOR
  for (int i=1; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {   
     gw.present(i, S_TEMP);
  }
#else
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {   
     gw.present(i, S_TEMP);
#endif

 #ifdef MOTION_SENSOR
  gw.present(CHILD_ID, S_MOTION);
#endif
 
  metric = gw.getConfig().isMetric;
}

void loop()      
{  

   measureCount ++;
  sendBattery ++;
  bool forceTransmit = false;
  
  if ((measureCount == 5) && highfreq) 
  {
    clock_prescale_set(clock_div_8); // Switch to 1Mhz for the reminder of the sketch, save power.
    highfreq = false;
  } 
  
  if (measureCount > FORCE_TRANSMIT_INTERVAL) { // force a transmission
    forceTransmit = true; 
    measureCount = 0;
  }
    
  gw.process();

  #ifdef DHT_SENSOR
  sendTempHumidityMeasurements(forceTransmit);
  #endif
  
  #ifdef DALLAS_SENSOR
  sendDallasTemp
  #endif
  
  if (sendBattery > 60) 
  {
     sendBattLevel(forceTransmit); // Not needed to send battery info that often
     sendBattery = 0;
  }

  // Sleep until interrupt comes in on motion sensor. Send update every two minute. 
  gw.sleep(INTERRUPT,CHANGE, SLEEP_TIME);
//  gw.sleep(SLEEP_TIME); //sleep a bit
}
}


void sendDallasTemp
{
#ifdef DALLAS_SENSOR
  sensors.requestTemperatures();      // Fetch temperatures from Dallas sensors
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());      // query conversion time and sleep until conversion completed
  gw.sleep(conversionTime);       // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)

#ifdef DHT_SENSOR
  // Read temperatures and send them to controller 
  for (int i=1; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
 
    // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;
 
    #if COMPARE_TEMP == 1    // Only send data if temperature has changed and no error
    if (lastTemperature[i] != temperature && temperature != -127.00 && temperature != 85.00) {
    #else
    if (temperature != -127.00 && temperature != 85.00) {
    #endif
 
      gw.send(msg.setSensor(i).set(temperature,1));         // Send in the new temperature
      lastTemperature[i]=temperature;      // Save new temperatures for next compare
    }
  }
#else
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {      // Read temperatures and send them to controller 

    // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;
 
    #if COMPARE_TEMP == 1    // Only send data if temperature has changed and no error
    if (lastTemperature[i] != temperature && temperature != -127.00 && temperature != 85.00) {
    #else
    if (temperature != -127.00 && temperature != 85.00) {
    #endif
 
      gw.send(msg.setSensor(i).set(temperature,1));      // Send in the new temperature
      lastTemperature[i]=temperature;      // Save new temperatures for next compare
    }
  }
  }
#endif

 #ifdef MOTION_SENSOR
  boolean tripped = digitalRead(DIGITAL_INPUT_SENSOR) == HIGH; 
  gw.send(msg.set(tripped?"1":"0"));  // Send tripped value to gw 
#endif
}


void sendTempHumidityMeasurements(bool force)
{
  bool tx = force;

  delay(dht.getMinimumSamplingPeriod());

  float oldAvgHum = raHum.getAverage();
  raHum.addValue(dht.getHumidity());

  float diffTemp = abs(lastTemperature - (isMetric ? dht.getTemperature() : dht.toFahrenheit(temperature)));
  float diffHum = abs(oldAvgHum - raHum.getAverage());

  if (isnan(diffHum)) tx = true; 
  if (diffTemp > TEMP_TRANSMIT_THRESHOLD) tx = true;
  if (diffHum >= HUMI_TRANSMIT_THRESHOLD) tx = true;

  if (tx) {
    measureCount = 0;
    float temperature = (isMetric ? data.celsiusHundredths : data.fahrenheitHundredths) / 100.0;
     
    int humidity = data.humidityPercent;

    gw.send(msgTemp.set(temperature,1));
    gw.send(msgHum.set(humidity));
    lastTemperature = temperature;
    lastHumidity = humidity;
  }
}

/********************************************
 *
 * Sends battery information (battery percentage)
 *
 * Parameters
 * - force : Forces transmission of a value
 *
 *******************************************/
void sendBattLevel(bool force)
{
  if (force) lastBattery = -1;
  long vcc = readVcc();
  if (vcc != lastBattery) {
    lastBattery = vcc;

#ifdef BATT_SENSOR
    gw.send(msgBatt.set(vcc));
#endif

    // Calculate percentage

    vcc = vcc - 1900; // subtract 1.9V from vcc, as this is the lowest voltage we will operate at
    
    long percent = vcc / 14.0;
    gw.sendBatteryLevel(percent);
  }
}

/*******************************************
 *
 * Internal battery ADC measuring 
 *
 *******************************************/
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADcdMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
