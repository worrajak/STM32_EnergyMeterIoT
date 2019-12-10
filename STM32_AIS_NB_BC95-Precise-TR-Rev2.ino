#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <Adafruit_MAX31865.h>

#include <libmaple/pwr.h>
#include <libmaple/scb.h>
#include <RTClock.h>
RTClock rtclock(RTCSEL_LSE);

#define SPI_NSS_PIN PA4
SPIClass SPI_2(2);

Adafruit_MAX31865 max = Adafruit_MAX31865(PA4);

#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

#include "TSL2561.h"
#include <ModbusMaster.h>

TSL2561 tsl(TSL2561_ADDR_FLOAT); 

#define MAX485_DE      PA6

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

#include "AIS_NB_BC95.h"

long cnt = 0;
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C #define BME280_ADDRESS 

#define I2C_ADDRESS 0x3C
#define RST_PIN -1
SSD1306AsciiWire oled;

float temperature = 0;
float humidity = 0;
uint16_t tempC_int = 0;
uint8_t hum_int = 0;
uint16_t lux_int = 0;
int vbat_int;

int timezone = 7;
int dst = 0;

String apnName = "devkit.nb";
String serverIP = "34.87.25.25"; // Your Server IP
String serverPort = "2232"; // Your Server Port

String udpData = "HelloWorld";

float x,V,A,W,Wh,F,PF = 0; 
uint16_t V_int=0,A_int=0,W_int=0,Wh_int=0;

AIS_NB_BC95 AISnb;

const long interval = 20000;  //millisecond
unsigned long previousMillis = 0;

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

void setup_vdd_sensor() {
    adc_reg_map *regs = ADC1->regs;
    regs->CR2 |= ADC_CR2_TSVREFE; // enable VREFINT and temp sensor
    regs->SMPR1 = (ADC_SMPR1_SMP17 /* | ADC_SMPR1_SMP16 */); // sample rate for VREFINT ADC channel
}

static void int_fun() {};

// standby=true for deep sleep
void sleepMode(bool standby, uint8_t seconds)
{ 
  rtclock.createAlarm(&int_fun, rtclock.getTime()+seconds);  // wakeup int
  PWR_BASE->CR &= PWR_CR_LPDS | PWR_CR_PDDS | PWR_CR_CWUF;
  PWR_BASE->CR |= PWR_CR_CWUF;
  PWR_BASE->CR |= PWR_CSR_EWUP;
  SCB_BASE->SCR |= SCB_SCR_SLEEPDEEP;
  if(standby) {
    PWR_BASE->CR |= PWR_CR_PDDS;
    PWR_BASE->CR &= ~PWR_CR_LPDS;
  } else {
    adc_disable(ADC1);
    adc_disable(ADC2);
    PWR_BASE->CR &= ~PWR_CR_PDDS;
    PWR_BASE->CR |= PWR_CR_LPDS;
  }
  asm("    wfi");
  SCB_BASE->SCR &= ~SCB_SCR_SLEEPDEEP;
}

void setPLL(rcc_pll_multiplier mult) 
{
  rcc_switch_sysclk(RCC_CLKSRC_HSI);
  rcc_turn_off_clk(RCC_CLK_PLL);
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , mult);
}

float extT;

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

    Serial.print(vbat_int*0.01);Serial.print(" "); 
    Serial.print(V);Serial.print(" ");
    Serial.print(A);Serial.print(" ");
    Serial.print(W);Serial.print(" ");
    Serial.println(Wh);
}

void setup(){ 

  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_DE, 0); 
 
  Serial.begin(9600);
  delay(2000);
  Serial1.begin(2400);
  Serial2.begin(9600);
  node.begin(1,Serial1);
  
  Serial.println("CONNECTING.......");
  
  AISnb.debug = true;

  AISnb.setupDevice(serverPort);
  //AISnb.setupDevice(serverPort,serverIP);

  String ip1 = AISnb.getDeviceIP();  
  //pingRESP pingR = AISnb.pingIP(serverIP);

  delay(2000); 
  previousMillis = millis();
}

unsigned long delTime = 10000;

void loop()
{ 
  unsigned long currentMillis = millis();
      cnt++;     
      readData();     
      // Send data in String 
      String DataSend ="{\"id\":\"NB-IoT-2230\",\"volt\":"+String(V)+",\"Current\":"+String(A)+",\"Watt\":"+String(W)+",\"Kwh\":"+String(Wh)+"}";
      UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, DataSend);
      UDPReceive resp = AISnb.waitResponse(); 
      delay(60000); 
}

