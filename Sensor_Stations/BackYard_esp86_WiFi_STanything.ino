/*  WiFi Station, Version-1.1, Dec2015  by TK kampto
BOARD INFO: Board "LOLIN(WEMOS D1 mini Pro)" esp8266,  CPU freq 80Mhz, Flash 16M,  
  Board Manager load:  "esp8266 community" Load from Boards Manager
  Board Learn: https://wiki.wemos.cc/doku.php
  For esp8266, DISCONNECT RST PIN WHEN UPLAODING CODE if useing Sleep function!!!
  if upload issues. 1. Slow upload speed, 2. select Generic esp8266 Board, jmp GPIO0 to gnd, then upload right board GPIO with disconnected
  On wake Up set-up is run, variables reset
ST_Anything Info:
  Example:  File: ST_Anything_Multiples_ESP8266WiFi.ino
  Github set up info  https://github.com/DanielOgorchock/ST_Anything/blob/master/README.md
  ST_Anything info:  https://community.smartthings.com/t/release-st-anything-v2-8-arduino-esp8266-esp32-to-st-via-thingshield-ethernet-or-wifi/77707
REVISIONS:
  June07-2015: Set up esp8266 with sleep
  Aug07-2017: First Use of WEMOS esp8266, Bat Voltage and Temp
  Nov27-2018: Set up DS18B20 temp sensor 
  Sept18-2020: Add client send and change to Hubitat
*/

//******************************** ST_Anything Librarys *********************************************
#include <SmartThingsESP8266WiFi.h>  // SmartThings Library for ESP8266WiFi
#include <Constants.h>       //Constants.h is designed to be modified by the end user to adjust behavior of the ST_Anything library
#include <Device.h>          //Generic Device Class, inherited by Sensor and Executor classes
#include <Sensor.h>          //Generic Sensor Class, typically provides data to ST Cloud (e.g. Temperature, Motion, etc...)
#include <PollingSensor.h>   //Generic Polling "Sensor" Class, polls Arduino pins periodically
#include <Everything.h>      //Master Brain of ST_Anything library that ties everything together and performs ST Shield communications

//*******************^^^^^^**** WEMOS D1 mini Pro Pin Definitions ***********************************
#define BatVolt       A0 //AI max 1.0V
//#define xxxxx       4 // IO, SDA
//#define xxxxx       16 // IO, Reserved for Sleep Reset, tied to RST
//#define xxxxx       5 // IO, SCL
//#define DHTpin      2 // IO, 10k Pull-up, BUILTIN_LED,  must not be pulled low during power on/reset, toggles value during boot, 
//#define xxxxx       13 // IO, MOSI
//#define xxxxx       0 // IO, 10k Pull-up, must not be pulled low during power on/reset, toggles value during boot
//#define xxxxx       14 // IO, SCK
#define DS18B20       12 // IO, MISO,  D6
//#define xxxxx       15 // IO, 10k Pull-down, SS, must not be pulled high during power on/reset*/,

//// DS18B20 Set-Up
#include <OneWire.h> // Dallas temp sensor
#include <DallasTemperature.h> // temp sensor
OneWire oneWire(DS18B20); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
float temp1; // DS18B20

//************************ Other **************************
#define DefaultSleepTime 600 // Time to sleep in Seconds in Normal operation
int sleepTime; // Adjustable Sleep Time
int bat_volt;
int RFrssi;
const byte SenderID = 11; //  (Optional) ID of this station                                 <------You must edit this line!

//******************************** ESP8266 WiFi Information **********************************************
String str_ssid     = "XXXXXXXXXX";                           //  <---You must edit this line!
String str_password = "XXXXXXXXX";                   //  <---You must edit this line!
IPAddress ip(192, 168, XXXXXX);       //Device IP Address       //  <---You must edit this line!
IPAddress gateway(192, 168, 0, 1);    //Router gateway          //  <---You must edit this line!
IPAddress subnet(255, 255, 255, 0);   //LAN subnet mask         //  <---You must edit this line!
IPAddress dnsserver(192, 168, 0, 1);  //DNS server              //  <---You must edit this line!
const unsigned int serverPort = 8090; // port to run the http server on
// Smartthings Hub Information
//IPAddress hubIp(192, 168, XXXXX);    // smartthings hub ip     //  <------You must edit this line!
//unsigned int hubPort = 39500;   // smartthings hub port
// Hubitat Hub Information
IPAddress hubIp(192, 168, XXX);    // hubitat hub ip         //  <---You must edit this line!
unsigned int hubPort = 39501;   // hubitat hub port

// Server info, where the non ST data goes, Board to Board Communication (optional)
IPAddress server(192,168,XXXXX);      // fixed IP address of arduino server     <------You must edit this line!
WiFiClient client; // to connect to server


//************************************** Set-Up *******************************************
void setup() {
  delay(100); // Test power up delay crash
  Serial.begin(115200);
   
  st::Everything::debug=true; // Needed for serial print debug
  //st::Executor::debug=true;
  st::Device::debug=true;
  st::PollingSensor::debug=true;
  //st::InterruptSensor::debug=true;

//// Initialize the "Everything" Class, Create the SmartThings ESP8266WiFi Communications Object, STATIC IP Assignment - Recommended
  // st::Everything::SmartThing = new st::SmartThingsESP8266WiFi(str_ssid, str_password, ip, gateway, subnet, dnsserver, serverPort, hubIp, hubPort, st::receiveSmartString, "BackYard1"); //Static
  st::Everything::SmartThing = new st::SmartThingsESP8266WiFi(str_ssid, str_password, serverPort, hubIp, hubPort, st::receiveSmartString,"BackYard1"); //DHCP
  st::Everything::init();   //Run the Everything class' init() routine which establishes WiFi communications with SmartThings Hub
  st::Everything::initDevices();  //Initialize each of the devices which were added to the Everything Class
}

//********************************* Arduino Loop() routine ************************************************
void loop(){
  sensorReads(); //FUNCTION
  STsend(); // FUNCTION
  ClientSend();  // (Optoinal)
  st::Everything::run();  //Execute the Everything run method which takes care of "Everything"

//// Sleep 
  delay(100); // a short delay so it can finish befoe sleep. Must connect esp8266 pin16 to RST to wake, disconnect during coade upload
  if (bat_volt < 3600) {sleepTime = 3600;} // Increase sleep time to 60min if battery low
  else sleepTime = DefaultSleepTime; // Default Sleep if Battery OK
  Serial.print(F("Zzzzzz..... Time to sleep for ")); Serial.print(sleepTime); Serial.println(F("sec")); Serial.println(F(""));
  ESP.deepSleep(sleepTime * 1000000, WAKE_RF_DEFAULT); // deepSleep time is defined in microseconds. Multiply seconds by 1e6
  delay(500); // a short delay to keep out of loop while Deep Sleep is being implemented.
  
}

//****************************************** Read Sensors ********************************************  
void sensorReads() {
  RFrssi = WiFi.RSSI();
////Read and Average, 
    sensors.requestTemperatures(); // DS18B20, Send the command to get temperatures on all sensors on onewire Bus
    int MeasurementsToAverage1 = 3;
    for(int i = 1; i <= MeasurementsToAverage1; ++i) {
      bat_volt = bat_volt + map(analogRead(BatVolt), 0, 1023, 0, 4320); // int mVolts, Internal V-divider R1=130K+220K, R2=100K, 4.5v Bat = 1v ADC 
      temp1 = temp1 + sensors.getTempCByIndex(0); delay(100); // Read DS18B20's, needs delay
      }
    temp1 = temp1 / MeasurementsToAverage1; temp1 = (temp1 * 1.8 + 32.0); // convert to DegF
    bat_volt = bat_volt / MeasurementsToAverage1;
    Serial.print("Avergae bat_volt = "); Serial.println(bat_volt);
    Serial.print("Avergae temp1 = "); Serial.println(temp1); 
}

//****************************************** Send Data to ST ********************************************  
void STsend() {
  String STvoltage1("voltage1 " + String(bat_volt)); st::Everything::sendSmartString(STvoltage1); // Send Bat Volt to ST
  if ((temp1 > -30) && (temp1 < 160)) {String STtemperature1("temperature1 " + String(temp1)); st::Everything::sendSmartString(STtemperature1);} // DS18B20
  }

//****************************************** Send Data to Arduino Server ****************************************
void ClientSend() {
  client.connect(server, 23); 
  String strSenderID("SenderID"); strSenderID.concat(" "); strSenderID.concat(SenderID);
  String strbat_volt("bat_volt"); strbat_volt.concat(" "); strbat_volt.concat(bat_volt);
  String strtemp1("temp1"); strtemp1.concat(" "); strtemp1.concat(temp1);
  String strRFrssi("RFrssi"); strRFrssi.concat(" "); strRFrssi.concat(RFrssi);

  Serial.println(""); Serial.println("..........Sending Variables to Server");
  client.println("Starting Data Stream");
  client.println(strSenderID); Serial.println(strSenderID); // Need to send ID first
  client.println(strbat_volt); Serial.println(strbat_volt); 
  client.println(strtemp1); Serial.println(strtemp1); 
  client.println(strRFrssi); Serial.println(strRFrssi); 

  client.flush();
  client.stop(); 
  Serial.println("..........Disconnecting"); Serial.println("");
  delay(5); // add appropriate delay here before sending next data element

}

//********* END
