// **********************************************************************************
#include <RFM69.h>         //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://github.com/lowpowerlab/rfm69
#include <RFM69_OTA.h>     //get it here: https://github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: https://github.com/lowpowerlab/spiflash
#include <SPI.h>           //included in Arduino IDE (www.arduino.cc)
#include <Wire.h>          //included in Arduino IDE (www.arduino.cc)
#include <SparkFunBME280.h>//get it here: https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/tree/master/src
#include <LowPower.h>      //get it here: https://github.com/lowpowerlab/lowpower
//writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define RFM69_RST_PIN 4
#define LED 7
#define GATEWAYID   1
int NODEID = 0;
#define NETWORKID   50
//#define FREQUENCY     RF69_433MHZ
#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY       RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY      "MeinSmartHomeKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -75
//*********************************************************************************************
#define SEND_LOOPS   70 //send data this many sleep loops (15 loops of 8sec cycles = 120sec ~ 2 minutes)  //70
#define SLEEP_FASTEST SLEEP_15MS
#define SLEEP_FAST SLEEP_250MS
#define SLEEP_SEC SLEEP_1S
#define SLEEP_LONG SLEEP_2S
#define SLEEP_LONGER SLEEP_4S
#define SLEEP_LONGEST SLEEP_8S
period_t sleepTime = SLEEP_LONGEST; //period_t is an enum type defined in the LowPower library (LowPower.h)
//*********************************************************************************************
#define BATT_MONITOR_EN A3 //enables battery voltage divider to get a reading from a battery, 
//disable it to save power
#define BATT_MONITOR  A1   //through 1Meg+470Kohm and 0.1uF cap from battery VCC - 
//this ratio divides the voltage to bring it below 3.3V where it is scaled to a readable range
#define BATT_CYCLES   1    //read and report battery voltage every this many sleep cycles 
//(ex 30cycles * 8sec sleep = 240sec/4min). 
//For 450 cyclesyou would get ~1 hour intervals
#define BATT_FORMULA(reading) reading * 0.003223 //* 1.475  // >>> fine tune this parameter to 
//match your voltage when fully charged
#define BATT_LOW      2.0  //(volts)
#define BATT_READ_LOOPS  SEND_LOOPS*10  // read and report battery voltage every this many sleep cycles 
//(ex 30cycles * 8sec sleep = 240sec/4min). 
//For 450 cycles you would get ~1 hour intervals between readings
//*************************************************************************************************************
//****************** PIN CONFIG NODE ADRESS *******************************************************************
#define BINARY_1  6
#define BINARY_2  5
#define BINARY_4  A7
#define BINARY_8  A0
#define SET_PIN_READ 9
int BINARY_1_STATE = 0;
int BINARY_2_STATE = 0;
int BINARY_4_STATE = 0;
int BINARY_8_STATE = 0;
//*************************************************************************************************************

#define BLINK_EN                 //Auskommentieren wenn kein blinken der LED gewünscht ist
//#define SERIAL_EN              //Auskommentieren wenn keine Serielle Übertragung gewünscht ist

#ifdef SERIAL_EN
  #define SERIAL_BAUD   115200
  #define DEBUG(input)   {Serial.print(input);}
  #define DEBUGln(input) {Serial.println(input);}
  #define SERIALFLUSH() {Serial.flush();}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
  #define SERIALFLUSH();
#endif
//*****************************************************************************************************

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif


BME280 bme280;
char Pstr[10];
char Fstr[10];
char Hstr[10];
double F,P,H;
char buffer[50];

typedef struct {
  uint32_t      nodeId;
  uint32_t      packages;
  float         temp;
  float         humidy;
  float         pressure;
  float         batt_voltage;
} Payload;
Payload theData;

void setup(void)
{
#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
  delay(10);
  Serial.println("START");
#endif

  pinMode(LED, OUTPUT);

  pinMode(BINARY_1, INPUT);
  pinMode(BINARY_2, INPUT);
  pinMode(BINARY_4, INPUT);
  pinMode(BINARY_8, INPUT);
  pinMode(SET_PIN_READ, OUTPUT); digitalWrite(SET_PIN_READ, LOW);

  pinMode(RFM69_RST_PIN, OUTPUT);
  digitalWrite(RFM69_RST_PIN, HIGH); 
  delay(100);
  digitalWrite(RFM69_RST_PIN, LOW);
  delay(100);

  readAdress();
  radio.initialize(FREQUENCY,NODEID,NETWORKID);

#ifdef IS_RFM69HW_HCW
  //radio.setHighPower(); //must include this only for RFM69HW/HCW!
  radio.setHighPower(true); // Always use this for RFM69HCW
  radio.setPowerLevel(8); //0-32 - power level, for close range just use 1 to make sure there's no power supply problems //10
#endif
  radio.encrypt(ENCRYPTKEY);

// Automatische Sendeleistungseinstellung aktivieren
#ifdef ENABLE_ATC
  //radio.enableAutoPower(ATC_RSSI);
#endif

  sprintf(buffer, "Transmitting at: %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  DEBUGln(buffer);

  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!

  //initialize weather shield BME280 sensor
  bme280.setI2CAddress(0x77); //0x76,0x77 is valid.
  bme280.beginI2C();
  bme280.setMode(MODE_FORCED); //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid. See 3.3
  bme280.setStandbyTime(0); //0 to 7 valid. Time between readings. See table 27.
  bme280.setFilter(0); //0 to 4 is valid. Filter coefficient. See 3.4.4
  bme280.setTempOverSample(1); //0 to 16 are valid. 0 disables temp sensing. See table 24.
  bme280.setPressureOverSample(1); //0 to 16 are valid. 0 disables pressure sensing. See table 23.
  bme280.setHumidityOverSample(1); //0 to 16 are valid. 0 disables humidity sensing. See table 19.
  theData.temp = bme280.readTempC();
  theData.pressure = bme280.readFloatPressure() / 100; // read Pa und convert to hPa
  theData.humidy = bme280.readFloatHumidity();
  bme280.setMode(MODE_SLEEP);

  //radio.sendWithRetry(GATEWAYID, (const void*)(&theData), sizeof(theData));
  Blink(LED, 200);Blink(LED, 200);Blink(LED, 200);
  
  SERIALFLUSH();
  readBattery();

  theData.packages=1;
}

unsigned long doorPulseCount = 0;
char input=0;
byte sendLoops=0;
byte battReadLoops=0;
float batteryVolts = 5;
char* BATstr="BAT:5.00v"; //longest battery voltage reading message = 9chars
byte sendLen;

void loop()
{
  if (battReadLoops--<=0) //only read battery every BATT_READ_LOOPS cycles
  {
    readBattery();
    battReadLoops = BATT_READ_LOOPS-1;
  }
  
  if (sendLoops--<=0)   //send readings every SEND_LOOPS
  {
    sendLoops = SEND_LOOPS-1;
    
    //read BME sensor
    bme280.setMode(MODE_FORCED); //Wake up sensor and take reading
    theData.temp = bme280.readTempC();
    theData.pressure = bme280.readFloatPressure() / 100;// * 0.0002953; //read Pa and convert to inHg
    theData.humidy = bme280.readFloatHumidity();
    bme280.setMode(MODE_SLEEP);

    theData.nodeId        = NODEID;
    theData.batt_voltage  = batteryVolts;

    Serial.print("Temp = "); Serial.println(F);
    Serial.print("Bat = "); Serial.println(batteryVolts);
    
    Serial.print("\nSENDE: (");
    Serial.print(sizeof(theData));
    Serial.print(" bytes) ... ");
    
    if (radio.sendWithRetry(GATEWAYID, (const void*)(&theData), sizeof(theData)))
    {
      Serial.println("ACK gesendet ...");
      Serial.print("ACT von Node ["); Serial.print(radio.SENDERID, DEC);Serial.println("]");
      Serial.print("RX_RSSI: [");Serial.print(radio.readRSSI());Serial.println("]");
    }
    else
    { 
      Serial.print(" WARTE...");
    }

    theData.packages++;
    
    #ifdef BLINK_EN
      Blink(LED, 100);
    #endif
  }
  
  
  SERIALFLUSH();
  radio.sleep(); //you can comment out this line if you want this node to listen for wireless programming requests
  LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF);
  DEBUGln("WAKEUP");
}

void readBattery()
{
  unsigned int readings=0;

  for (byte i=0; i<5; i++){
    readings+=analogRead(BATT_MONITOR);    
  }

  batteryVolts = BATT_FORMULA(readings / 5.0);
  //if (batteryVolts <= BATT_LOW) BATstr = "LOW";
}

void Blink(byte PIN, byte DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS/2);
  digitalWrite(PIN,LOW);
  delay(DELAY_MS/2);  
}

void readAdress()
{
  digitalWrite(SET_PIN_READ, HIGH);
  delay(5);
  if(digitalRead(BINARY_1) == 1) BINARY_1_STATE=0; else BINARY_1_STATE=8;
  if(digitalRead(BINARY_2) == 1) BINARY_2_STATE=0; else BINARY_2_STATE=4;
  if(analogRead(BINARY_4) > 5)   BINARY_4_STATE=0; else BINARY_4_STATE=2;
  if(analogRead(BINARY_8) > 5)   BINARY_8_STATE=0; else BINARY_8_STATE=1;
  digitalWrite(SET_PIN_READ, LOW);

  NODEID = BINARY_1_STATE + BINARY_2_STATE + BINARY_4_STATE + BINARY_8_STATE + 1;
}
