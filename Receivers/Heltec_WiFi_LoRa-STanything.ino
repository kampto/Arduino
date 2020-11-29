  /*  LoRa-WiFi AP Station, Version-1.0,   by TK kampto April-2017
BOARD INFO: "Heltec_WiFi_LoRa_32"  CPU freq 80Mhz, Flash 4M,  Upload Speed 921600 or 115200 if packet issue
  Heltec Board Downloads: https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
  Using RFM9x with Ardunio: https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/using-the-rfm-9x-radio
  Radio Head info and latest Librarys http://www.airspayce.com/mikem/arduino/RadioHead/
ST_ANYTHING INFO:
  Github set up info  https://github.com/DanielOgorchock/ST_Anything/blob/master/README.md
  ST_Anything info:  https://community.smartthings.com/t/release-st-anything-v2-8-arduino-esp8266-esp32-to-st-via-thingshield-ethernet-or-wifi/77707
REVISIONS:
  Apr20-2017: Test LoRa
  May25-2018: First use as wifi and RF reciever
  July02-2018: Define data structure. Add Wifi connected on board LED
  Aug27-2018: Incude SmartThingsESP32WiFi libray into local folder and modify to not reset board if wifi disconnect whihc cuases lockUps
  Apr11-2019: Update data structure datatypes to be comapatibe with 32u4 boards
  Nov02-2019: Revamp data structure, data parse and ST send into arrays, change display from scroll to blink
  Mar24-2020: Add Server - Client Communication so Board can pass data over wifi
*/

//******************************** ST_Anything Librarys **********************************************************
#include <SmartThingsESP32WiFi.h>  // SmartThings Library for ESP32 boards.  Use " " if library is custom and local
#include <Constants.h>       //Constants.h is designed to be modified by the end user to adjust behavior of the ST_Anything library
#include <Device.h>          //Generic Device Class, inherited by Sensor and Executor classes
#include <Sensor.h>          //Generic Sensor Class, typically provides data to ST Cloud (e.g. Temperature, Motion, etc...)
#include <Executor.h>        //Generic Executor Class, typically receives data from ST Cloud (e.g. Switch)
#include <InterruptSensor.h> //Generic Interrupt "Sensor" Class, waits for change of state on digital input 
#include <PollingSensor.h>   //Generic Polling "Sensor" Class, polls Arduino pins periodically
#include <Everything.h>      //Master Brain of ST_Anything library that ties everything together and performs ST Shield communications

//#include <PS_DS18B20_Temperature.h>  //Implements a Polling Sesnor (PS) to measure Temperature via DS18B20 libraries 
//#include <PS_Voltage.h>      //Implements a Polling Sensor (PS) to measure voltage on an analog input pin 
//#include <PS_AdafruitThermocouple.h> //Implements a Polling Sensor (PS) to measure Temperature via Adafruit_MAX31855 library
//#include <PS_Ultrasonic.h>   //Ultrasonic Distance Measurement Sensor, being used to monitor water level in cylindrical tank
//#include <PS_Illuminance.h>  //Implements a Polling Sensor (PS) to measure light levels via a photo resistor
//#include <PS_TemperatureHumidity.h>  //Implements a Polling Sensor (PS) to measure Temperature and Humidity via DHT library // wont compile ith this!!
//#include <IS_Motion.h>       //Implements an Interrupt Sensor (IS) to detect motion via a PIR sensor
//#include <IS_Contact.h>      //Implements an Interrupt Sensor (IS) to monitor the status of a digital input pin
//#include <IS_Smoke.h>        //Implements an Interrupt Sensor (IS) to monitor the status of a digital input pin
//#include <IS_DoorControl.h>  //Implements an Interrupt Sensor (IS) and Executor to monitor the status of a digital input pin and control a digital output pin
//#include <IS_Button.h>       //Implements an Interrupt Sensor (IS) to monitor the status of a digital input pin for button presses
//#include <EX_Switch.h>       //Implements an Executor (EX) via a digital output to a relay
//#include <EX_Alarm.h>        //Implements Executor (EX)as an Alarm Siren capability via a digital output to a relay
#include <S_TimedRelay.h>    //Implements a Sensor to control a digital output pin with timing capabilities
//#include <PS_Water.h>        //Implements a Polling Sensor (PS) to measure presence of water (i.e. leak detector)

#include <SPI.h>
#include <RH_RF95.h> // RadioHead Lora
#include <SSD1306.h> // OLeD
//#include "WiFi.h" // Use ST wifi library instead

//// ESP Sleep Stuff and stored variables
//#include "esp_deep_sleep.h" // Library Needed to put into sleep mode
//#define sleepTime  60  // Time to sleep in Seconds
RTC_DATA_ATTR unsigned int WifiBootCounter = 0; // Store wifi connection reboots

//// Number of devices of each type, *** Put Number of + 1, becuase [10] is really 0 to 9
  const byte NumberOfVoltage = 15;  
  const byte NumberOfTemp = 13; 
  const byte NumberOfGeneric = 12; 
  const byte NumberOfHumidity = 2;
  const byte NumberOfUltrasonic = 2; 
  const byte NumberOfContact = 6; 
  const byte NumberOfRFrssi = 7; 
  //const byte NumberOfMotion = 1;
  byte STbatch = 0; //ST send interval selector
  unsigned long STsendCount = 0;
  int TankLevel; // for display only
  const float bucketAmount = 0.01650157;   // inches equivalent tipping rain bucket
  
  int Voltage[NumberOfVoltage];  
  float Temp[NumberOfTemp];  
  float Generic[NumberOfGeneric];  
  float Humid[NumberOfHumidity];  
  float Ultrasonic[NumberOfUltrasonic];  
  bool Contact[NumberOfContact];  
  int rfRssi[NumberOfRFrssi];  
  //unsigned int Motion[NumberOfMotion];
  //String STmotion1 = "motion1 inactive";

//// Timers
unsigned long TempReadTimer = 0; 
unsigned long TempReadDelay = 65000; // Wait Time for local Temp reads
unsigned long STsendTimer = 0;
unsigned long STsendDelay = 200000;
unsigned long SerialPrintTimer = 0; 
unsigned long SerialPrintDelay = 5000; // Time between serial prints
unsigned long DisplayTimer1 = 0;
unsigned int DisplayTimer3 = 0;
unsigned long ResetTimer =0;
unsigned long ResetTimerDelay = 86400000; // 24hr day scheduled board reset
unsigned long WifiCheckTimer = 0;
unsigned long WifiCheckDelay = 240000; // Wait this time in between WiFi connection checks
unsigned long OledResetTimer = 0;
unsigned long OledResetDelay = 600000; // Wait 10min inbetween Oled resets
//unsigned long MotionActiveTimer = 0;
//unsigned long MotionActiveDelay = 5000; // Wait time to make inactive
unsigned int DisplayDataTime;  // Time to display data before switching to next
unsigned int ConstDisplayTime = 1200; // Time to display data before switching to next
unsigned long ClientSendTimer = 0; // For wifi data sends, non-ST
unsigned long ClientSendDelay = 180000;

//***************************************** Helec ESP32 with Lora & OLed Pins ***********************************************
//#define xxxxxx      A1 // GPIO 0 or ADC2-1
//#define xxxxxx      23 // GPIO 23
//SDA                 21 // GPIO 21 or SDA I2C (Wire) data pin
//SCL                 22 // GPIO 22 or SCL I2C (Wire) clock pin
//RX                  3  // RX receive (input) pin, GPIO 3
//TX                  1  // TX transmit (output) pin, GPIO 1 
#define LED           25 // GPIO 25 or ADC2-8, AI & AO an analog output DAC2, On Board LED
#define Button2       12 // GPIO 12 or ADC2-5
//#define xxxxxx      A5 // GPIO 12 or ADC2-5
#define DS18B20       13 // GPIO 13 or ADC2-4
//#define xxxxxx      36 // GPI 36 or ADC1-0, Input Only!
//#define xxxxxx      37 // GPI 37 or ADC1-1, Input Only!
//#define xxxxxx      38 // GPI 38 or ADC1-2, Input Only!
//#define xxxxxx      39 // GPI 39 or ADC1-3, Input Only!
//#define xxxxxx      34 // GPI 34 or ADC1-6, Input Only!
#define BatVolt       A7 // GPI 35 or ADC1-7, Input Only!  V1 Working AI
//#define xxxxxx      32 // GPI 32 or ADC1-4, Input Only!
//#define xxxxxx      33 // GPI 33 or ADC1-5, Input Only!
#define Button1       2  // GPIO 2 or ADC2-2
//RST                  // this is the 3.3V regulator's enable pin. It's pulled up, so ground to disable the 3.3V regulator

//// Reserved Oled Pins  
#define Oled_SDA      4  // SDA I2C (Wire) data pin, Reserved for Oled, GPIO 4 or ADC2-0
#define Oled_SCL      15 // SCL I2C (Wire) clock pin, Reserved for Oled, GPIO 15 or ADC2-3
#define Oled_RST      16 // Reserved for Oled, GPIO 16, Use sketch to control
SSD1306  display(0x3c, Oled_SDA, Oled_SCL, Oled_RST ); // Oled, V1/V2 board new library needs pins defined

//// Reserved Lora Pins
//#define SCK     5    // GPIO5  -- SX127x's SCK, SCK SPI Clock
//#define MISO    19   // GPIO19 -- SX127x's MISO, MISO SPI Master-In-Slave-Out
//#define MOSI    27   // GPIO27 -- SX127x's MOSI, MOSI SPI Master-Out-Slave-In

//// RFM95 Lora Library definitions
#define RFM95_RST     14   // GPIO14 -- SX127x's RESET
#define RFM95_CS      18   // GPIO18 -- SX127x's CS
#define RFM95_INT     26   // GPIO26 -- SX127x's IRQ(Interrupt Request)
#define RF95_FREQ 915.0  // Change to 434.0 or other frequency, must match RX's freq!
RH_RF95 rf95(RFM95_CS, RFM95_INT); // Singleton instance of the radio driver

//// Required If Using DS18B20's
#include <OneWire.h> // Dallas temp sensor
#include <DallasTemperature.h> // temp sensor
OneWire oneWire(DS18B20); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.

//****************** Send array structure. Sender and Reciever structure must match exactly, Datatypes are not the same for some boards!! **************
struct dataStruct{
    unsigned short SenderID; // 0 to 65,535
    unsigned short bat_volt; // 0 to 65,535 
    unsigned short sol_volt; // 0 to 65,535
    short volt1; // -32,768 to 32,767
    short temp1; // -32,768 to 32,767; Send as 10x, on Reciever side /10 to float to get decimals 
    short temp2; // -32,768 to 32,767
    short temp3; // -32,768 to 32,767
    short temp4; // -32,768 to 32,767 
    short humid1; // -32,768 to 32,767
    short contact1; // 32,768 to 32,767, true, false
    short contact2; // 32,768 to 32,767, true, false
    short contact3; // 32,768 to 32,767, true, false
    short contact4; // 32,768 to 32,767, true, false
    short contact5; // 32,768 to 32,767, true, false
    short generic1; // -32,768 to 32,767
    short generic2; // -32,768 to 32,767
    short generic3; // -32,768 to 32,767
    short generic4; // -32,768 to 32,767
    short ultrasonic1; // -32,768 to 32,767
    unsigned short MotionCount; // 0 to 65,535
    unsigned short SleepCount; // 0 to 65,535
    short RFrssi; // -32,768 to 32,767
    }SensorReadings;
  byte buf[sizeof(SensorReadings)] = {0}; // RF communication, Dont put this on the stack

//******************************^^^***** ESP32 WiFi Information *************************************************
String str_ssid     = "XXXXXXXX";   //                      <------You must edit this line!
String str_password = "XXXXXXXX";   //                                   <------You must edit this line!
IPAddress ip(192, 168, XX, XXX);       //Device IP Address                   <------You must edit this line!
IPAddress gateway(192, 168, 0, 1);    //Router gateway                      <------You must edit this line!
IPAddress subnet(255, 255, 255, 0);   //LAN subnet mask                     <------You must edit this line!
IPAddress dnsserver(192, 168, 0, 1);  //DNS server                          <------You must edit this line!
const unsigned int serverPort = 8090; // port to run the http server on
// Smartthings Hub Information
IPAddress hubIp(192, 168, XX, XXX);    // smartthings hub ip                 <-----You must edit this line!
const unsigned int hubPort = 39500;   // smartthings hub port
// Server info, where the non ST data goes
IPAddress server(192,168,XX,XXX);      // (optional) fixed IP address of arduino server     <------You must edit this line!
WiFiClient client; // to connect to server (optional)
int SenderID = 100; // Assign initial of this station             <------You must edit this line!

//*****************************************  Arduino Setup() routine *************************************************
void setup(){ 
  Serial.begin(115200); 
  Serial.println("Setting Up!!!!!!!!!");
  DisplayDataTime = ConstDisplayTime;
  
//// ESP32 ADC Set-up
  analogReadResolution(11); // Set pin Resolution. Default of 12 is not very linear. 12 = 0-4095, 11 = 0-2047, 10 = 0-1024, 9 = 0-512 
  analogSetAttenuation(ADC_11db);  // Sets attenuation for all pins. 11db=0-3.3v, 6dB=0-2.2v, 2.5db=0-1.5v, 0db= 0-1v, Default is 11db which is noisy. 
  //analogSetPinAttenuation(A7, ADC_11db) // sets the attenuation for individual pins.
  
//SPECIAL sensors/executors (uses portions of both polling and executor classes)
   static st::S_TimedRelay            sensor1(F("relaySwitch1"), Button1, LOW, false, 1000, 0, 1); // Low = init stae, false = no inverted logic, 30000, on time
   static st::S_TimedRelay            sensor2(F("relaySwitch2"), Button2, LOW, false, 1000, 0, 1); // Low = init stae, false = no inverted logic, 30000, on time
    
//************************************** Configure debug print output from each main class ***************************************
  st::Everything::debug=true;
  st::Executor::debug=true;
  st::Device::debug=true;
  st::PollingSensor::debug=true;
  st::InterruptSensor::debug=true;

//********************************** Initialize the "Everything" Class and WiFi Connect *******************************************
   //Create the SmartThings ESP32 Communications Object,  STATIC IP Assignment - Recommended
   //st::Everything::SmartThing = new st::SmartThingsESP32WiFi(str_ssid, str_password, ip, gateway, subnet, dnsserver, serverPort, hubIp, hubPort, st::receiveSmartString);
  
   //DHCP IP Assigment - Must set your router's DHCP server to provice a static IP address for this device's MAC address
   st::Everything::SmartThing = new st::SmartThingsESP32WiFi(str_ssid, str_password, serverPort, hubIp, hubPort, st::receiveSmartString, "esp32Station");
 
   st::Everything::init();   //Run the Everything class' init() routine which establishes WiFi communications with SmartThings Hub

//************************ Add each sensor to the "Everything" Class, Comment out sensor # not used *******************************
  st::Everything::addSensor(&sensor1);  //Button
  st::Everything::addSensor(&sensor2);  //Button
  //st::Everything::addSensor(&sensor3);  //Thermocouple
          
//************************* Add each executor to the "Everything" Class,  Enable if using executors *******************************
  //st::Everything::addExecutor(&executor1);
  //st::Everything::addExecutor(&executor2);
    
  st::Everything::initDevices();  //Initialize each of the devices which were added to the Everything Class

pinMode(LED, OUTPUT); digitalWrite(LED, LOW);// WiFi indicator
  
//// Required If Using DS18B20  
  sensors.begin(); // OneWire IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement
  sensors.setResolution(10); // 9bit +/-0.5deg 94msec read, 10bit +/-0.25deg 187msec read, 11bit +/-0.125deg 3754msec read, 12bit +/-.06deg 750msec read time

//// Oled start
  pinMode(Oled_RST,OUTPUT); digitalWrite(Oled_RST, LOW); delay(100); digitalWrite(Oled_RST, HIGH); // while OLED is running, must set GPIO16 in high
  display.init(); // Initialising the UI will init the display too.
  display.flipScreenVertically(); display.setFont(ArialMT_Plain_16);
  display.clear();

//// Set Up Lora 
  pinMode(RFM95_RST, OUTPUT); digitalWrite(RFM95_RST, HIGH);
  while (!Serial) { delay(1);} delay(100);

  Serial.println("Feather LoRa TX Test!"); display.drawString(0, 0, "LoRa TX Test!"); display.display();// Oled
  digitalWrite(RFM95_RST, LOW); delay(10); digitalWrite(RFM95_RST, HIGH); delay(10); //// Manual LoRa reset
 
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed"); display.drawString(0, 16, "LoRa radio init failed"); display.display();// Oled
    while (1);
    }
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed"); display.drawString(0, 32, "Set Freq failed"); display.display();// Oled
    while (1);
   }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ); display.drawString(0, 32, "Set Freq to: " + String(RF95_FREQ)); display.display();// Oled
  rf95.setTxPower(23, false); // If using RFM95/96/97/98 modules which use PA_BOOST transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
  delay(2000);  // delay to read Oled

  WIFIScan(1);  // FUNCTION Oled
  WIFISetUp();  // FUNCTION Oled
  
  } /// End Set-Up Routine

//****************************************** Arduino Loop() routine *******************************************
void loop(){ 
  RF95x_Recieve(); // FUNCTION
  //ServerRecieve(); 
  ClientSend();
  LocalSensors(); // FUNCTION
  LocalDisplay(); // FUNCTION
  ParseData(); // FUNCTION 
  SendSTData(); // FUNCTION
  SerialPrint(); // FUNCTION
  WiFiwatchdeg(); // FUNCTION
  OledReset(); // FUNCTION,  reset OLED after a set time
  RF95x_Send(); // FUNCTION
  if (millis() - ResetTimer > ResetTimerDelay) {ESP.restart();} // Reset board periodically
  if (digitalRead(Button1) == HIGH) {digitalWrite(LED, HIGH); delay(1000); digitalWrite(LED, LOW); }
    
  st::Everything::run();  //FUNCTION ST Lib Internal, Required: Execute the Everything run method which takes care of "Everything"
  }

//****************************************** LoRa Recieve ********************************************
void RF95x_Recieve(){ 
 if (rf95.available()) {
 // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
 
    if (rf95.recv(buf, &len)){
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: "); Serial.println((char*)buf); Serial.print("RF RSSI: "); Serial.println(rf95.lastRssi(), DEC);
 
      // Send a reply
      uint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      
      memcpy(&SensorReadings, buf, sizeof(SensorReadings)); // Parse out the readings
      Serial.println("");
      }
    else {Serial.println("Receive failed");}
  }
}
//****************************************** LoRa Send ********************************************
void RF95x_Send(){ 
  return;  // ***************** TEST
  if ((digitalRead(Button1) == HIGH) ||  (digitalRead(Button2) == HIGH)) {
    display.clear(); display.setFont(ArialMT_Plain_16);
    Serial.println("Transmitting...");  display.drawString(0, 0, "Transmitting..."); display.display();// Oled
  
    // Send a message to rf95_server
    if (digitalRead(Button1) == HIGH) {uint8_t data[] = "Hello World!"; rf95.send(data, sizeof(data));}
    else if (digitalRead(Button2) == HIGH) {uint8_t data[] = "Yep"; rf95.send(data, sizeof(data));}
    delay(100);
    Serial.println("Waiting for packet to complete..."); 
    rf95.waitPacketSent();
    
//// Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply..."); display.drawString(0, 16, "Waiting for reply..."); display.display();// Oled
  if (rf95.waitAvailableTimeout(2000)) { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len)) {
      Serial.print("Got reply: "); Serial.println((char*)buf); Serial.print("RF RSSI: ");Serial.println(rf95.lastRssi(), DEC); 
      display.drawString(0, 32, "Got reply OK!" ); 
      display.drawString(0, 48, "RF RSSI= " + String(rf95.lastRssi(), DEC)); 
      display.display(); delay(1500); // Oled and time to display  
      //LastRFrssi = (rf95.lastRssi()); Serial.println(LastRFrssi);// Store rssi before sleep
      }
    else {Serial.println("Receive failed"); display.drawString(0, 48, "Receive failed"); display.display(); delay(1000);} // Serial and Oled
    }
  else {Serial.println("No reply, is there a listener around?"); display.drawString(0, 48, "No Reciever?"); display.display(); delay(1000);} // Serial and Oled
 }
}
//****************************************** Parse data from Senders ********************************************
void ParseData(){
//// Put value with ST Object name and send, ST objects Temperature1,2 & Voltage1,2 are reserved for the reciever board
    if (SensorReadings.SenderID == 0) {return;} // get out if no new data
    if (SensorReadings.SenderID == 1){  //// Water Tank
      Serial.println("..............Recieved Water Tank Values......... ");
      Voltage[1] = SensorReadings.bat_volt;
      Voltage[2] = SensorReadings.sol_volt;
      Temp[1] = (SensorReadings.temp1 / 10.0); 
      Temp[2] = (SensorReadings.temp2 / 10.0); // Air Temp
      Ultrasonic[1] = SensorReadings.ultrasonic1;  // Tank Level
      Generic[1] = SensorReadings.SleepCount; 
      rfRssi[1] = SensorReadings.RFrssi;
      }
    else if (SensorReadings.SenderID == 2){  //// Shooting Bench
      Serial.println("..............Recieved Shooting Bench Values......... "); 
      Voltage[3] = SensorReadings.bat_volt;
      Voltage[4] = SensorReadings.sol_volt;
      Temp[3] = (SensorReadings.temp1 / 10.0); 
      Temp[4] = (SensorReadings.temp2 / 10.0); // Air Temp
      Generic[2] = SensorReadings.SleepCount; 
      rfRssi[2] = SensorReadings.RFrssi;
      }
    else if (SensorReadings.SenderID == 3){  //// Deadmans Curve
      Serial.println("..............Recieved Deadmans Curve Values......... "); 
      Voltage[5] = SensorReadings.bat_volt;
      Voltage[6] = SensorReadings.sol_volt;
      Temp[5] = (SensorReadings.temp1 / 10.0); 
      Temp[6] = (SensorReadings.temp2 / 10.0); // Air Temp
      Generic[3] = SensorReadings.SleepCount;  // Sleeps
      rfRssi[3] = SensorReadings.RFrssi;
      }
    else if (SensorReadings.SenderID == 4){  //// Red Shed
      Serial.println("..............Recieved Red Shed Values......... "); 
      Voltage[7] = SensorReadings.bat_volt;  // Shed LiPo Battery
      Voltage[8] = SensorReadings.sol_volt;  // Shed Solar Panel
      Voltage[9] = SensorReadings.volt1;  // Shed 12V Bat
      Temp[7] = (SensorReadings.temp1 / 10.0); // inside
      Temp[8] = (SensorReadings.temp2 / 10.0);
      Generic[4] = SensorReadings.SleepCount; // Sleeps
      Generic[11] = (SensorReadings.generic1 * bucketAmount);  //  Rain amount
      Contact[1] = SensorReadings.contact1; // Shed Door
      rfRssi[4] = SensorReadings.RFrssi;
      }
    else if (SensorReadings.SenderID == 5){  // Pump Box
      Serial.println("..............Recieved Pump Box Values......... "); 
      Voltage[10] = SensorReadings.bat_volt;
      Voltage[11] = SensorReadings.sol_volt;
      Voltage[12] = SensorReadings.volt1;  // Pump voltage
      Temp[9] = SensorReadings.temp1 / 10.0; // Board temp
      Temp[10] = SensorReadings.temp2 / 10.0; // Air temp
      Generic[5] = SensorReadings.generic2 / 10;  //  Total Tank1 in gallons
      Generic[6] = SensorReadings.generic3;  //  Pump On time in minutes
      Generic[10] = SensorReadings.generic4 / 10;  //  Total Tank2 in gallons
      Generic[7] = SensorReadings.SleepCount;
      Contact[2] = SensorReadings.contact1; // Pump, Low equal On
      Contact[3] = SensorReadings.contact2; // Spring, Low equal empty
      Contact[4] = SensorReadings.contact3; // Tank1, Low equal Full
      Contact[5] = SensorReadings.contact4; // Tank2, Low equal Full
      rfRssi[5] = SensorReadings.RFrssi;
      }
    else if (SensorReadings.SenderID == 6){  // Turtle Pond
      Serial.println("..............Recieved Turtle Pond Values......... "); 
      Voltage[13] = SensorReadings.bat_volt;  // LiPo Battery
      Voltage[14] = SensorReadings.sol_volt;  // Solar Panel
      Temp[11] = (SensorReadings.temp1 / 10.0); 
      Temp[12] = (SensorReadings.temp2 / 10.0);
      Generic[8] = SensorReadings.SleepCount; // Sleeps
      Generic[9] = SensorReadings.MotionCount; // PIR counts
      rfRssi[6] = SensorReadings.RFrssi;
      }
    //else if (SensorReadings.SenderID == 9){  // Concrete Shed via wifi
      //Voltage[15] = SensorReadings.bat_volt;  // Battery bank
      //Temp[13] = (SensorReadings.temp1 / 10.0); // DHT room temp
      //}
  SensorReadings.SenderID = 0; // Reset ID so not to continously write
}
    
//********************************** Send all Data to ST in batches of 10 max ********************************
void SendSTData() {
 if (millis() - STsendTimer > STsendDelay) { 
    if (STsendCount < 2) {++STsendCount; STsendTimer = millis(); return;} // get out so not to send zero values after board reboot
    ++STbatch; Serial.print(F("Begin ST Sends, STbatch = ")); Serial.println(STbatch);
    
  switch (STbatch) {
  case 1: Serial.println(F("Case 1!!!")); for (int i=0; i <= (NumberOfVoltage -1); i++) {
          if (Voltage[i] >= 0) {String string("voltage"); string.concat(i); string.concat(" "); string.concat(Voltage[i]); st::Everything::sendSmartString(string); delay(20);}
          if (i == 9) {st::Everything::run();} // flush out data if >9 sends que'd
          } break;
              
  case 2: Serial.println(F("Case 2!!!")); for (int i=0; i <= (NumberOfTemp - 1); i++)  {
          if (Temp[i] > -50) {String string("temperature"); string.concat(i); string.concat(" "); string.concat(Temp[i]); st::Everything::sendSmartString(string);}
          if (i == 9) {st::Everything::run();} // flush out data if >9 sends que'd
          } break;
       
  case 3: Serial.print(F("Case 3!!!")); for (int i=1; i <= (NumberOfGeneric - 1); i++) {
          String string("generic"); string.concat(i); string.concat(" "); string.concat(Generic[i]); st::Everything::sendSmartString(string);
          if (i == 9) {st::Everything::run();} // flush out data if >9 sends que'd
          } break;

  case 4: Serial.println(F("Case 4!!!")); {String string{"ultrasonic"}; string.concat(1); string.concat(" "); string.concat(Ultrasonic[1]); st::Everything::sendSmartString(string);
          //for (int i=1; i <= NumberOfUltrasonic - 1; i++) {
          //String string{"ultrasonic"}; string.concat(i); string.concat(" "); string.concat(Ultrasonic[i]); st::Everything::sendSmartString(string);
          // if (i == 9) {st::Everything::run();} // flush out data if >9 sends que'd
          }  break;
          
  case 5: Serial.println(F("Case 5!!!")); for (int i=1; i <= (NumberOfContact - 1); i++) {
          if (Contact[i]) {String string("contact"); string.concat(i); string.concat(" "); string.concat("open"); st::Everything::sendSmartString(string);}
            else {String string("contact"); string.concat(i); string.concat(" "); string.concat("closed"); st::Everything::sendSmartString(string);}
          if (i == 9) {st::Everything::run();} // flush out data if >9 sends que'd
          } break;

  //case 6: for (int i=1; i <= NumberOfHumidity; i++) {
         // if (Humid[i] >= -200) {String string{"humidity"}; string.concat(i); string.concat(" "); string.concat(Humid[i]); st::Everything::sendSmartString(string);}
          //} break;
     
  default: Serial.println(F("Case Default!!!")); STsendTimer = millis(); STbatch = 0; ++STsendCount; // Reset Send timer and incriment counter
           Serial.println(F("Sending Data to ST Completed .......... "));  
           Serial.print(F("ST Send Counts = ")); Serial.println(STsendCount);
           break;  
     }
     
  }
}

//****************************************** IDE Serial Print Display ********************************************
void SerialPrint(){
  if (millis() - SerialPrintTimer > 30000) {
  Serial.println(F("Local Values............................"));
  Serial.println("Local Temp... " + (String)Temp[0]); 
  Serial.println("Local Supply V... " + (String)Voltage[0]);
  Serial.println("Local Wifi RSSI... " + (String)WiFi.RSSI()); 
  Serial.print("Local RF95 915mhz RSSI:"); Serial.println(rf95.lastRssi(), DEC);
  Serial.print(F("ST Send Counts = ")); Serial.println(STsendCount);
  Serial.println(F(""));

  //// Remote Serial Prints   
  Serial.println("Remote Values..........................."); 
     Serial.print("Temp1: "); Serial.println(SensorReadings.temp1 / 10); 
     Serial.print("Temp2: "); Serial.println(SensorReadings.temp2 / 10); 
     Serial.print("Temp3: "); Serial.println(SensorReadings.temp3 / 10); 
     Serial.print("Temp4: "); Serial.println(SensorReadings.temp4 / 10); 
     Serial.print("Battery Voltage: "); Serial.println(SensorReadings.bat_volt);
     Serial.print("Solar Voltage: "); Serial.println(SensorReadings.sol_volt);
     Serial.print("Ultrasonice: "); Serial.println(SensorReadings.ultrasonic1);
     Serial.print("Sleep Counts: "); Serial.println(SensorReadings.SleepCount); 
     Serial.print("RF RSSI: "); Serial.println(SensorReadings.RFrssi);
     Serial.println(F(""));
  SerialPrintTimer = millis();
  }
}

 
//****************************************** Serial and Oled Display ********************************************
void LocalDisplay(){ 
if (millis() - DisplayTimer3 >1200){
  if (millis() - DisplayTimer1 > 10800) { DisplayTimer1 = millis();}
  display.clear(); display.setFont(ArialMT_Plain_24);
  if (millis() - DisplayTimer1 <1200) { 
    display.drawString(0, 0, "Room Temp");
    display.drawString(0, 25, (String)Temp[0] + " F");
    }
  
  else if (millis() - DisplayTimer1 <2400) { //// All Local Values
    display.drawString(0, 0, "Water Tank");
    display.drawString(0, 25, (String)Temp[2] + " F");
    }

  else if (millis() - DisplayTimer1 <3600) { //// All Local Values
    display.drawString(0, 0, "S-Bench");
    display.drawString(0, 25, (String)Temp[4] + " F");
    }

  else if (millis() - DisplayTimer1 <4800) { //// All Local Values
    display.drawString(0, 0, "Deadmans");
    display.drawString(0, 25, (String)Temp[6] + " F");
    }

  else if (millis() - DisplayTimer1 <6000) { //// All Local Values
    display.drawString(0, 0, "Red Shed");
    display.drawString(0, 25, (String)Temp[8] + " F");
    }

  else if (millis() - DisplayTimer1 <7200) { //// All Local Values
    display.drawString(0, 0, "Pump Box");
    display.drawString(0, 25, (String)Temp[10] + " F");
    }

  else if (millis() - DisplayTimer1 <8400) { //// All Local Values
    display.drawString(0, 0, "TurtlePond");
    display.drawString(0, 25, (String)Temp[12] + " F");
    }

  else if (millis() - DisplayTimer1 <9600) { //// All Local Values
    display.drawString(0, 0, "Rain Amount");
    display.drawString(0, 25, (String)Generic[11] + " in");
    }
    
  else if (millis() - DisplayTimer1 <10800) { //// All Local Values
    if ( Ultrasonic[1] != 0) {TankLevel = ((85 - (Ultrasonic[1] / 2.54) - 9)) / 85 * 100;}
    display.drawString(0, 0, "Tank Level");
    display.drawString(0, 25, (String)TankLevel + " %");
    }
  
  if (WiFi.status() == WL_CONNECTED){display.setFont(ArialMT_Plain_10); display.drawString(0, 55, "WiFi RSSI = " + (String)WiFi.RSSI());}
    else {display.drawString(0, 50, "NO WiFi CONNECTION");}
   
   display.display();
   DisplayTimer3 = millis();
  }
}

//****************************************** Local Sensors ********************************************
void LocalSensors(){
 if (millis() - TempReadTimer > TempReadDelay) {
    Voltage[0] = map(analogRead(BatVolt), 0, 2047, 0, 7500); //int mVolts, V Divider R1=680Kohm, R2=680Kohm  
    
    sensors.requestTemperatures(); // DS18B20, Send the command to get temperatures on all sensors on onewire Bus
    Temp[0] = sensors.getTempCByIndex(0);  delay(100); // Give it time to read DS18B20
    Temp[0] = (Temp[0] * 1.8 + 32.0); // convert to DegF
    TempReadTimer = millis();
  }
}

//****************************************** Send Data to Arduino Server ****************************************
void ClientSend() {
  if (millis() - ClientSendTimer > ClientSendDelay) {
  client.connect(server, 23); 
  String strSenderID("SenderID"); strSenderID.concat(" "); strSenderID.concat(SenderID);
  String strcontact1("contact1"); strcontact1.concat(" "); strcontact1.concat(Contact[2]); // Water Pump on/off
  String strcontact2("contact2"); strcontact2.concat(" "); strcontact2.concat(Contact[4]); // Main Tank Full
  //String strtemp1("temp1"); strtemp1.concat(" "); strtemp1.concat(temp1);
  //String strhumid1("humid1"); strhumid1.concat(" "); strhumid1.concat(humid1);
  
  Serial.println(""); Serial.println("..........Sending Variables to Arduino Server");
  client.println("Starting Data Stream");
  client.println(strSenderID); Serial.println(strSenderID); // Need to send ID first
  client.println(strcontact1); Serial.println(strcontact1); 
  client.println(strcontact2); Serial.println(strcontact2); 
  //client.println(strhumid1); Serial.println(strhumid1); 
  
  client.flush();
  client.stop(); 
  Serial.println("..........Disconnecting"); Serial.println("");
  delay(5); // add appropriate delay here before sending next data element
  ClientSendTimer = millis();
  }
}

//************************************ Check WiFi Connectivity **********************************
void WiFiwatchdeg(){
  //if (WiFi.status() == WL_CONNECTED) {dacWrite(LED, 210); } // OnBoard LED On if wifi is connected. dacWrite to lower intensity
  //else dacWrite(LED, 0);  // OnBoard LED Off if wifi lost
  
  if (millis() - WifiCheckTimer > WifiCheckDelay) {
    WifiCheckTimer = millis(); 
    if (WiFi.status() != WL_CONNECTED) {++WifiBootCounter;  ESP.restart(); }
    }
  }

//**************************************** OLED Reset  ***************************************
void OledReset(){
  if (millis() - OledResetTimer > OledResetDelay) {
//// Reset OLED incase it got hung up
    pinMode(Oled_RST,OUTPUT); digitalWrite(Oled_RST, LOW); delay(100); digitalWrite(Oled_RST, HIGH); // while OLED is running, must set GPIO16 in high
    display.init(); display.flipScreenVertically(); display.setFont(ArialMT_Plain_24); // Initialising the UI will init the display too.
    //display.drawString(0, 0, "Reset");
    //display.drawString(0, 24, "OLED...");
    //display.display(); delay(100); display.clear();
    OledResetTimer = millis(); 
    }
 }
 
//************************************ Wifi SCAN Oled ********************************************
void WIFIScan(unsigned int value) {
  for(int i=0;i<value;i++){
    display.clear(); display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Scan start..."); display.display();

    int n = WiFi.scanNetworks();
    display.drawString(0, 20, "Scan done");
    display.display(); delay(500); display.clear();

    if (n == 0) {display.clear(); display.drawString(0, 0, "no network found"); display.display(); delay(2000);}
    else {
      display.drawString(0, 0, (String)n);
      display.drawString(14, 0, "networks found:");
      display.display(); delay(500);
  
      for (int i = 0; i < n; ++i) {
        // Print SSID and RSSI for each network found
        display.drawString(0, (i+1)*9,(String)(i + 1));
        display.drawString(6, (i+1)*9, ":");
        display.drawString(12,(i+1)*9, (String)(WiFi.SSID(i)));
        display.drawString(90,(i+1)*9, " (");
        display.drawString(98,(i+1)*9, (String)(WiFi.RSSI(i)));
        display.drawString(114,(i+1)*9, ")");
        delay(10);
        }
      }
    display.display(); delay(3000); display.clear();
    }
}

//************************************ Wifi SET-UP Oled ********************************************
void WIFISetUp(void) {
  byte count = 0;
  while(WiFi.status() != WL_CONNECTED && count < 10)  {count ++; delay(500); display.drawString(0, 0, "Connecting..."); display.display();}
  display.clear();
  
  if(WiFi.status() == WL_CONNECTED){
    display.drawString(0, 0, "Connected To........");
    display.drawString(0, 10, (String)WiFi.SSID());
    display.drawString(0, 20, "RSSI = "); display.drawString(45, 20, (String)WiFi.RSSI());
    display.drawString(0, 30, "WIFI Setup done");
    display.display(); delay(3000); display.clear();
    }
  else {display.clear(); display.drawString(0, 0, "Connecting...Failed"); display.display(); delay(2000);}
}

