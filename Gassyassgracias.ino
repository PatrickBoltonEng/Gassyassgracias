/*
 * Project Gassyassgracias
 * Description:  Gas Sensors Seeed Multichanel v2 & MH-Z16
 * Author:  PJB
 * Date:  05/20/21
 */

#include "Particle.h"
#include "math.h"
#include "Multichannel_Gas_GMXXX.h"
#include <Wire.h>
#include "JsonParserGeneratorRK.h"    //install it JsonParserGeneratorRK

SYSTEM_THREAD(ENABLED);

GAS_GMXXX<TwoWire> gas_sensor;

#define UPDATE_INTERVAL 10000  //1 sec = 1000 millis

int min_time, min_last, CO2, CO2TC;
float NO2, C2H5OH, VOC, CO;
uint32_t gas_val = 0; 

unsigned long UpdateInterval;

const unsigned char cmd_get_sensor[] = {0xff,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};

const unsigned char cmd_calibratezero[] = {0xff,0x01,0x87,0x00,0x00,0x00,0x00,0x00,0x78};

const unsigned char cmd_calibratespan[] = {0xff,0x01,0x88,0x07,0xD0,0x00,0x00,0x00,0xA0};

SerialLogHandler logHandler(LOG_LEVEL_INFO);

void setup()
{ 
  Serial.begin(9600);
  delay(100);

  Serial1.begin(9600,SERIAL_8N1);
  Serial1.setTimeout(500);             // Wait up to 500ms for the data to return in full
  delay(100);

  gas_sensor.begin(Wire, 0x08);

  //Serial1.write(cmd_calibratezero, sizeof(cmd_calibratezero));
  //Serial1.write(cmd_calibratespan, sizeof(cmd_calibratespan));
  
  Log.info("System version: %s", (const char*)System.version());
  Log.info("Setup Complete");

  UpdateInterval = millis();
  min_last=-1;
  min_time=0;
}

void loop()
{
  Time.zone(-7);

  if(Particle.disconnected()){return;}
  if(millis() - UpdateInterval > UPDATE_INTERVAL)
  {
    getGas(NO2, C2H5OH, VOC, CO);

    GetCO2(CO2, CO2TC);
            
    min_time=Time.minute();
    if((min_time!=min_last)&&(min_time==0||min_time==5||min_time==10||min_time==15||min_time==20||min_time==25||min_time==30||
        min_time==35||min_time==40||min_time==45||min_time==50||min_time==55))
    {
      createEventPayload(NO2, C2H5OH, VOC, CO, CO2, CO2TC);
      min_last = min_time;
      Log.info("Last Update: %d", min_last);
      Log.info(Time.timeStr());
    }
    UpdateInterval = millis();
  }
}

void getGas(float &NO2, float &C2H5OH, float &VOC, float &CO)
{
    gas_val = gas_sensor.measure_NO2(); 
    NO2 = gas_sensor.calcVol(gas_val);
    Log.info("NO2: %d %f", gas_val, NO2);

    gas_val = gas_sensor.measure_C2H5OH(); 
    C2H5OH = gas_sensor.calcVol(gas_val);
    Log.info("C2H5OH: %d %f", gas_val, C2H5OH);
    
    gas_val = gas_sensor.measure_VOC(); 
    VOC = gas_sensor.calcVol(gas_val);
    Log.info("VOC: %d %f", gas_val, VOC);
    
    gas_val = gas_sensor.measure_CO(); 
    CO = gas_sensor.calcVol(gas_val);
    Log.info("CO: %d %f", gas_val, CO);
}

bool GetCO2(int& CO2, int& CO2TC)
{
  byte data[9];
  char chksum = 0x00;
  int len = -1, ntry=0, nmtry=10, i =0;
  while((len < 9 && ntry < nmtry))
    {
    Serial1.flush();                                                             // Flush TX buffer
    while(Serial1.read() >= 0);                                                  // Flush RX buffer
    Serial1.write(cmd_get_sensor, 9);
    for(uint32_t msTimeout=millis(); (len=Serial1.available())<9 && (millis()-msTimeout<1000); Particle.process());
    Log.trace("[MHZ] Available: %d", len);
    Log.trace("[MHZ] Num. try-: %d", ntry);
    ntry++; 
    }
  if (len != 9) return false;
  for(i = 0; i < len; i++) data[i] = Serial1.read();
  for(i = 1; i < 9; i++) chksum += data[i];
  if(chksum==0) CO2 = (256*(int)data[2]) + (int)data[3];
  Log.info("CO2: %d", CO2);
  if(chksum==0) CO2TC = (int)data[4] - 40;
  Log.info("CO2_TC: %d", CO2TC);
  return true;
}

void createEventPayload(float NO2, float C2H5OH, float VOC, float CO, int CO2, int CO2TC)
{
  JsonWriterStatic<512> jw;
  {
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("NO2ppm", NO2);
    jw.insertKeyValue("C2H5OHppm", C2H5OH);
    jw.insertKeyValue("VOCppm", VOC);
    jw.insertKeyValue("COppm", CO);
    jw.insertKeyValue("CO2ppm", CO2);
    jw.insertKeyValue("CO2_TC", CO2TC);
  }
  Particle.publish("GasesGracias", jw.getBuffer(), PRIVATE);
}