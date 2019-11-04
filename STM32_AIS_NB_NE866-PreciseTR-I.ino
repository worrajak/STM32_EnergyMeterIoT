#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include "TSL2561.h"

String serverIP = "34.92.53.75"; // Your Server IP
String serverPort = "2100"; // Your Server Port
#include <ModbusMaster.h>

#define MAX485_DE      PA1

ModbusMaster node;
bool state = true;
void preTransmission()
{
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_DE, 0);
}


float reform_uint16_2_float32(uint16_t u1, uint16_t u2)
{  
  uint32_t num = ((uint32_t)u1 & 0xFFFF) << 16 | ((uint32_t)u2 & 0xFFFF);
    float numf;
    memcpy(&numf, &num, 4);
    return numf;
}

float getRTU(uint16_t m_startAddress){
  uint8_t m_length =2;
  uint16_t result;
  float x;
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  result = node.readInputRegisters(m_startAddress, m_length);  //readInputRegisters
  if (result == node.ku8MBSuccess) {
     return reform_uint16_2_float32(node.getResponseBuffer(0),node.getResponseBuffer(1));
  }
}  

float x,V,A,W,Wh,F,PF = 0; 
uint16_t V_int=0,A_int=0,W_int=0,Wh_int=0;

#include "AIS_NB_NE866.h"
String apnName = "devkit.nb";
String udpData = "HelloWorld";

AIS_NB_NE866 AISnb;

const long interval = 30000;  //millisecond
unsigned long previousMillis = 0;

long cnt = 0;
#define SEALEVELPRESSURE_HPA (1013.25)

float temperature = 0;
float humidity = 0;
uint16_t tempC_int = 0;
uint8_t hum_int = 0;
uint16_t lux_int = 0;
int vbat_int;

int32_t lat32;
int32_t lon32;
int32_t alt32;
byte gpsb[10];
float flat, flon;
float lat=0.0, lon=0.0, alt=0.0;
unsigned long age=0;

const int pingPin = PA0;
int inPin = PA1;
 long duration, cm;

void setup_vdd_sensor() {
    adc_reg_map *regs = ADC1->regs;
    regs->CR2 |= ADC_CR2_TSVREFE; // enable VREFINT and temp sensor
    regs->SMPR1 = (ADC_SMPR1_SMP17 /* | ADC_SMPR1_SMP16 */); // sample rate for VREFINT ADC channel
}

void readData()
{
    V = getRTU(0x0000);  
    delay(100);
    A = getRTU(0x0006);
    delay(100); 
    W = getRTU(0x000C);
    delay(100);      
    Wh = getRTU(0x0156);
    delay(100); 
    adc_enable(ADC1);
    vbat_int = 120 * 4096 / adc_read(ADC1, 17);
    adc_disable(ADC1);    
  
    Serial.print(V);Serial.print(" ");
    Serial.print(A);Serial.print(" ");
    Serial.print(W);Serial.print(" ");
    Serial.println(Wh);
}


void setup()
{ 
  delay(15000);
  Serial.begin(9600);
  delay(1000);
  
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, 0); 

  Serial1.begin(2400);
  Serial2.begin(9600);
  node.begin(1,Serial1);
  
  AISnb.debug = true;
  AISnb.setupDevice(serverPort,serverIP);
  String ip1 = AISnb.getDeviceIP();  
  delay(1000);
  //pingRESP pingR = AISnb.pingIP(serverIP);
  previousMillis = millis();
}

void loop()
{ 
  unsigned long currentMillis = millis();
      readData();     
      // Send data in String 
      String DataSend ="{\"id\":\"NB-IoT-1\",\"volt\":"+String(V)+",\"Current\":"+String(A)+",\"Watt\":"+String(W)+",\"Kwh\":"+String(Wh)+"}";
      UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, DataSend);
      previousMillis = currentMillis;
      UDPReceive resp = AISnb.waitResponse();     
      delay(30000);
}



