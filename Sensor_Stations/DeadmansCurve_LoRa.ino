/*  LoRa Station, Version-1.0,   by TK kampto June-2017
BOARD INFO: "Adafruit Feather 32u4"  or BSfrance "LoRa32u4II" Knock off board
  Adafruit Board infos: https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/pinouts
  Using RFM9x with Ardunio: https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/using-the-rfm-9x-radio
  Radio Head info and latest Librarys http://www.airspayce.com/mikem/arduino/RadioHead/
  AVR 32u4 Sleep info:  https://learn.adafruit.com/adafruit-feather-32u4-radio-with-rfm69hcw-module/power-management
  BSfrance Board Files https://github.com/BSFrance/BSFrance-avr  
BOARD OPERATION:
  This board does not deep sleep, so variables carry over, set-up does not run after wake up.
  UPLOADING ISSUE for 32u4, if the COM doesnt show up, double click reset when uplad starts after complile is done, or once, Use TestMode for troubleshooting
  "LowPower.h" 0.13ma poweer down sleep mode, 10.5ma awake idle, 35ma RF send, also support interupts, use this one
REVISIONS:
  Aug18-2017: First use as Heltec v1 board LoRa 
  Dec18-2018: First use as 32u4 RF board
  Apr07-2019: Updated data structure datatypes to be compatable with other board processors
  Nov02-2019: Updated Data structure
*/

#include <SPI.h>
#include <RH_RF95.h> // RadioHead Lora
#include "LowPower.h" // lower sleep draw then <Adafruit_SleepyDog.h> that is 0.3ma

//// 32u4 Sleep Stuff and stored variables
bool TestMode = false; // ********* false = normal operation, true = no sleep, no RF send, with serial com.
#define DefaultSleepCycles 75 // Sleep time is Cycles x 8sec, 8sec is max sleep for 32u4 so cycle it,  Use 75 x 8 = 10min,  
int sleepCycles; // Adjustable Sleep Time
int LastRFrssi = 0; // Store rssi for next wake becuase rssi happen at end of loop
unsigned int SleepCounter = 0; // Store boot count even during sleep

//*************************** Fethaer 32u4 LoRa Pin-Out ****************************************
//#define XXXXX   0 // RX - GPIO #0, also receive (input) pin for Serial1 and Interrupt #2
//#define XXXXX   1 // TX - GPIO #1, also transmit (output) pin for Serial1 and Interrupt #3
//#define XXXXX   2 // SDA - GPIO #2, also the I2C (Wire) data pin. no pull up, when using with I2C, may need a 2.2K-10K pullup. Also Interrupt #1
//#define XXXXX   3 // SCL - GPIO #3, also the I2C (Wire) clock pin. no pull up, when using with I2C, may need a 2.2K-10K pullup. Can also do PWM output and act as Interrupt #0.
//#define XXXXX   5 // GPIO #5, can also do PWM output
//#define XXXXX   6 // GPIO #6, can also do PWM output and analog input A7
#define BatVolt   A9 // GPIO #9, also analog input A9 and can do PWM output. analog input is connected to a BAT voltage divider,this pin naturally 'sits' at around 2VDC
//#define XXXXX   10 // GPIO #10, also analog input A10 and can do PWM output.
//#define XXXXX   11 // GPIO #11, can do PWM output.
//#define XXXXX   12 // GPIO #12, also analog input A11 and can do PWM output.
#define LEDboard  13 // GPIO #13, can do PWM output and is on board White LED
//#define XXXXX   A0 //  Analog input as well as digital I/O pins
//#define XXXXX   A1 //  Analog input as well as digital I/O pins
//#define XXXXX   A2 //  Analog input as well as digital I/O pins
//#define XXXXX   A3 //  Analog input as well as digital I/O pins
#define DS18B20   A4 //  Analog input as well as digital I/O pins
#define SolVolt   A5 //  Analog input as well as digital I/O pins
// There are also breakouts for 3 of the RFM's GPIO pins (IO1, IO2 and IO3). 
// 15-SCK, 16-MOSI, 14-MISO - These are the hardware SPI pins, used by the RFM radio module 

//// RFM95 Lora Library definitions for 32u4 Board
#define RFM95_RST     4  // SX127x's RESET Pin
#define RFM95_CS      8  // SX127x's CS, used as the radio CS (chip select) pin
#define RFM95_INT     7  // SX127x's IRQ(Interrupt Request), used as the radio GPIO0 / IRQ (interrupt request) pin.
#define RF95_FREQ 915.0  // Change to 434.0 or other frequency, must match RX's freq!
RH_RF95 rf95(RFM95_CS, RFM95_INT); // Singleton instance of the radio driver

//// Required If Using DS18B20 temp IC's
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
    short RFrssi; //-32,768 to 32,767
    }SensorReadings;
  byte buf[sizeof(SensorReadings)] = {0}; // RF communication, Dont put this on the stack

float temperature1; // Set up as float for serial prints
float temperature2;

//*****************************************  Arduino Setup() routine *************************************************
void setup(){   
  SensorReadings.SenderID = 3; // < ----------- You must set the ID of this station
  Serial.begin(115200);
  if (TestMode) {while (!Serial) { delay(1);}} // to use serial monitor Only needed for 32u4 boards
  
//// Required If Using DS18B20 temperature sensors
  sensors.begin(); // OneWire IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement
  sensors.setResolution(10); // 9bit +/-0.5deg 94msec read, 10bit +/-0.25deg 187msec read, 11bit +/-0.125deg 3754msec read, 12bit +/-.06deg 750msec read time
  
//// Set Up Lora 
  pinMode(RFM95_RST, OUTPUT); digitalWrite(RFM95_RST, HIGH); delay(100);
  digitalWrite(RFM95_RST, LOW); delay(10); digitalWrite(RFM95_RST, HIGH); delay(10); //// Manual reset LoRa
  Serial.println("LoRa TX Reset Completed!");
 
  while (!rf95.init()) {Serial.println("LoRa radio init failed"); while (1); }
  Serial.println("LoRa init OK!,");
   
  if (!rf95.setFrequency(RF95_FREQ)) {Serial.println("setFrequency failed"); while (1);} // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(22, false); // If you are using RFM95/96/97/98 modules whith PA_BOOST transmitter pin, you can set transmitter powers from 5 to 23 dBm:

} /// End Set-Up Routine

//****************************************** Arduino Loop() routine *******************************************
void loop(){ 
    Temperature(); // FUNCTION
    Voltages(); // FUNCTION
    RF95x_Send(); // FUNCTION
    Sleep(); // FUNCTION, put to sleep after loop
  }
  
//****************************************** Sleep Logic ********************************************  
void Sleep() {
    if (TestMode) {delay(2000); return;} // for serial troubleshooting
    digitalWrite(LEDboard, LOW); delay(100); // force LED off and Finish up stuff before sleeping
    if (SensorReadings.bat_volt < 3700) {sleepCycles = DefaultSleepCycles * 6;} // Increase sleep time to 1hr
    else sleepCycles = DefaultSleepCycles; // Default Sleep Cylces x 8sec
    Serial.println("Going to Sleep for " + String(sleepCycles*8) + " Seconds, Zzzzzzzzzzzzzzz......... ");
    Serial.println("Sleep Counts " + String(SleepCounter));
    rf95.sleep(); // Put Radio to Sleep
    int i; for (i = 0; i < sleepCycles; i++) {LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);}   // 8sec is max for ATmega32u4, 8*i = total sleep time
    SensorReadings.SleepCount = SleepCounter++; // Incriment sleep counts
   }

//******************************************** LoRa Send ********************************************
void RF95x_Send(){ 
  if (TestMode) {return;} // dont send if in test mode
  digitalWrite(LEDboard, HIGH); // Visual indicator
  Serial.println("Transmitting..."); // Send a message to rf95_server
   
  byte zize=sizeof(SensorReadings);
  memcpy (buf, &SensorReadings, zize); // copy array into buf
  
  Serial.println("Sending to ask server");
  rf95.send(buf, zize); delay(200);
  Serial.println("Waiting for packet to complete..."); 
  rf95.waitPacketSent();
  
//// Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
 
  Serial.println("Waiting for reply..."); 
  if (rf95.waitAvailableTimeout(1000)) { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len)) {
      Serial.print("Got reply: "); Serial.println((char*)buf); Serial.print("RF RSSI: ");Serial.println(rf95.lastRssi(), DEC); 
      LastRFrssi = (rf95.lastRssi()); Serial.println(LastRFrssi);// Store rssi before sleep
      SensorReadings.RFrssi = LastRFrssi; // Load stored variable after sleep
      }
    else {Serial.println("Receive failed");} 
    }
  else {Serial.println("No reply, is there a listener around?");}
  digitalWrite(LEDboard, LOW);
}

//****************************************** Voltage Reads ********************************************
void Voltages(){ 
 SensorReadings.bat_volt = 0;
 SensorReadings.sol_volt = 0;
 int MeasurementsToAverage1 = 3;
 for(int i = 1; i <= MeasurementsToAverage1; ++i) {
 SensorReadings.bat_volt = SensorReadings.bat_volt + map(analogRead(BatVolt), 0, 1023, 0, 4187); //Bat int mVolts, internal V Divider 
 SensorReadings.sol_volt = SensorReadings.sol_volt + map(analogRead(SolVolt), 0, 1023, 0, 6645); //Solar-USB int mVolts, V Divider R1=680Kohm, R2=680Kohm 
 delay(50);}
 SensorReadings.bat_volt = SensorReadings.bat_volt / MeasurementsToAverage1;
 SensorReadings.sol_volt = SensorReadings.sol_volt / MeasurementsToAverage1;
 if (SensorReadings.sol_volt < SensorReadings.bat_volt) {SensorReadings.sol_volt = 0;} // Make 0 becuause sun is not out
    Serial.print(F("Averaged Battery Voltage = ")); Serial.print(SensorReadings.bat_volt); Serial.println(F(" Volts"));
    Serial.print(F("Averaged Solar Voltage = ")); Serial.print(SensorReadings.sol_volt); Serial.println(F(" Volts"));
 }

//****************************************** Temperature ********************************************
void Temperature(){
    sensors.requestTemperatures(); // DS18B20, Send the command to get temperatures on all sensors on onewire Bus
    temperature1 =sensors.getTempCByIndex(0); delay(75); // Give it time to read DS18B20's
    temperature2 =sensors.getTempCByIndex(1); delay(75);
    temperature1 = (temperature1 * 1.8 + 32.0); // convert to DegF
    temperature2 = (temperature2 * 1.8 + 32.0); // convert to DegF
    Serial.print(F("Temp1 = ")); Serial.print(temperature1); Serial.println(" DegF");
    Serial.print(F("Temp2 = ")); Serial.print(temperature2); Serial.println(" DegF");
    SensorReadings.temp1 = (temperature1 *10); // Send as short int
    SensorReadings.temp2 = (temperature2 *10); // Send as short int
    
  }

/*
//****************************************** LoRa Recieve ********************************************
void RF95x_Recieve(){ 
 return; // ********** Comment out to Recieve
 if (rf95.available()) {
 // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
 
   if (rf95.recv(buf, &len)){
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: "); Serial.println((char*)buf); Serial.print("RF RSSI: "); Serial.println(rf95.lastRssi(), DEC);

      // Send a reply
      uint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data)); delay(100);
      rf95.waitPacketSent();
      Serial.println("Sent a reply"); Serial.println("");
      
///Perform task from Recieved message and Display
   String Recieved((char*)buf);
   if (Recieved == "Hello World!") {Serial.println(F("Recieved - Hello World !!!!!!!!!!!!!!!"));}
   else if (Recieved == "Yep") {Serial.println(F("Recieved - Yep !!!!!!!!!!!!!!!"));}
   delay(100);
   }
  else {Serial.println("Receive failed");}
  }
}
*/
