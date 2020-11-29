/*  LoRa Station, Version-1.0,   by TK kampto June-2017
BOARD INFO: "Adafruit Feather 32u4" (using this)  or BSfrance "LoRa32u4II" Knock off board
  Adafruit Board infos: https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/pinouts
  Using RFM9x with Ardunio: https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/using-the-rfm-9x-radio
  Radio Head info and latest Librarys http://www.airspayce.com/mikem/arduino/RadioHead/
BOARD OPERATION:
  Wake up Collect data and RF send sensor data, door open will trigger interupt, wake up and RF send immediatly once
  This board does run set-up after wake up, variables are maintaned
  UPLOADING ISSUE for 32u4, Dont select COM Port, click reset ounce when upload starts after complile is done, or twice, Use TestMode for troubleshooting
  "LowPower.h" 0.13ma poweer down sleep mode, 10.5ma awake idle, 35ma RF send, also support interupts, use this one
REVISIONS:
  Aug18-2017: First use as Heltec v1 board LoRa
  Aug18-2018: First use as 32u4 RF board
  Apr07-2019: Updated data structure datatypes to be compatable with other board processors
  May20-2019: Add sensor interupt
  Jul28-2019: Set up for RedShed sensors
  May16-2020: Add rain gauge
*/

#include <SPI.h> // used for LoRa chip
#include <RH_RF95.h> // RadioHead Lora
#include "LowPower.h" // lower sleep draw then <Adafruit_SleepyDog.h> that is 0.3ma

//// ATmeag32u4 Sleep and Sketch Variables
bool TestMode = false; // ********* false = normal operation, true = no sleep, no RF send, with serial com.
#define DefaultSleepCycles 75 // Sleep time is Cycles x 8sec, 8sec is max sleep for 32u4 so cycle it,  Use 75 x 8s = 10min,  
int sleepCycles; // Adjustable Sleep Time cycles
unsigned int SleepCounter = 0; 
float temperature1; // Set up as float for serial prints
float temperature2;
const float bucketAmount = 0.01650157;   // inches equivalent of ml to trip tipping-bucket
float bucketTips = 0; // incrimenting number of bucket tips
float rainTotal = 0;

//*************************** Fethaer ATmega32u4+LoRa Pin-Out ****************************************
#define rainBucket 0 // RX - GPIO #0, also receive (input) pin for Serial1 and Interrupt #2
//#define XXXXX   1 // TX - GPIO #1, also transmit (output) pin for Serial and Interrupt #3
//#define XXXXX   2 // SDA - GPIO #2, also the I2C (Wire) data pin. no pull up, when using with I2C, may need a 2.2K-10K pullup. Also Interrupt #1
//#define XXXXX   3 // SCL - GPIO #3, also the I2C (Wire) clock pin. no pull up, when using with I2C, may need a 2.2K-10K pullup. Can also do PWM output and act as Interrupt #0.
//#define XXXXX   5 // GPIO #5, can also do PWM output
//#define XXXXX   6 // GPIO #6, can also do PWM output and analog input A7
#define BatVolt   A9 // GPIO #9, also analog input A9 and can do PWM output. analog input is connected to a BAT voltage divider,this pin naturally 'sits' at around 2VDC
#define Bat12Volt A10 // GPIO #10, also analog input A10 and can do PWM output.
//#define XXXXX   11 // GPIO #11, can do PWM output.
//#define XXXXX   12 // GPIO #12, also analog input A11 and can do PWM output.
#define LEDboard  13 // GPIO #13, can do PWM output and is on board White LED
//#define XXXXX   A0 //  Analog input as well as digital I/O pins
//#define XXXXX   A1 //  Analog input as well as digital I/O pins
//#define XXXXX   A2 //  Analog input as well as digital I/O pins
#define Door      A3 //  Analog input as well as digital I/O pins
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

//*****************************************  Arduino Setup() routine *************************************************
void setup(){   
  SensorReadings.SenderID =  4; // < ----------- You must set the ID of this station
  Serial.begin(115200);
  if (TestMode) {while (!Serial) { delay(1);}} // to use serial monitor Only needed for 32u4 boards

  pinMode(rainBucket, INPUT_PULLUP); 
  pinMode(Door, INPUT_PULLUP);  // Switch output, closed = LOW,  Open = HIGH
  pinMode(LEDboard, OUTPUT); digitalWrite(LEDboard, HIGH); delay(500); digitalWrite(LEDboard, LOW); // TEST

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
    if (digitalRead(Door) == HIGH)  {SensorReadings.contact1 = true; Serial.println(F("DOOR OPEN"));}// Door open
    else if (digitalRead(Door) == LOW) {SensorReadings.contact1 = false; Serial.println(F("DOOR CLOSED"));} // Door Closed
    Temperature(); // FUNCTION
    Voltages(); // FUNCTION
    RF95x_Send(); // FUNCTION
    Sleep(); // FUNCTION, put to sleep after loop
  }
  
//****************************************** Sleep Logic ********************************************  
void Sleep() {
   if (TestMode) {delay(2000); return;} // for serial troubleshooting
   digitalWrite(LEDboard, LOW); delay(100); // force LED off and Finish up stuff before sleeping
   if (SensorReadings.bat_volt < 3700) {sleepCycles = DefaultSleepCycles * 3;} // Increase sleep time to 30min
    else sleepCycles = DefaultSleepCycles; // Default Sleep Cylces x 8sec
    Serial.println("Going to Sleep for " + String(sleepCycles*8) + " Seconds, Zzzzzzzzzzzzzzz......... ");
    Serial.println("Sleep Counts " + String(SleepCounter));
    rf95.sleep(); // Put Radio to Sleep, this will save you 2ma on ATmega32u4 boards
    SensorReadings.SleepCount = SleepCounter++; // Incriment sleep counts
    
    attachInterrupt(2, wakeUp, FALLING); // Allow wake up pin RISING to trigger interrupt on HIGH begin only. HIGH, LOW, RISING, CHANGE, FALLING
    int i; for (i = 0; i < sleepCycles; i++) {LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);}   // 8sec is max for ATmega32u4, 8*i = total sleep time
    detachInterrupt(2); // Disable external pin interrupt on wake up pin so it doesnt mess up other loop functions.
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
      SensorReadings.RFrssi = (rf95.lastRssi()); Serial.println(rf95.lastRssi());
      }
    else {Serial.println("Receive failed");} 
    }
  else {Serial.println("No reply, is there a listener around?");}
  digitalWrite(LEDboard, LOW);  
}

//****************************************** Voltage Reads ********************************************
void Voltages(){ 
   
    SensorReadings.bat_volt = map(analogRead(BatVolt), 0, 1023, 0, 4170); // 4.5v max in, LiPo Bat int mVolts, internal V Divider  
    SensorReadings.sol_volt = map(analogRead(SolVolt), 0, 1023, 0, 25990); //25v max in, Solar in mVolts, V Divider R1=680Kohm, R2=100Kohm 
    SensorReadings.volt1 = map(analogRead(Bat12Volt), 0, 1023, 0, 25815); //25v max in, 12v Bat int mVolts, V Divider R1=680Kohm, R2=100Kohm 
    
    if (SensorReadings.sol_volt < 2000) {SensorReadings.sol_volt = 0;} // In mV, Make 0 becuause sun is not out, 12V
    Serial.print(F("Averaged LiPo Battery Voltage = ")); Serial.print(SensorReadings.bat_volt); Serial.println(F(" milli Volts"));
    Serial.print(F("Averaged Solar Voltage = ")); Serial.print(SensorReadings.sol_volt); Serial.println(F(" milli Volts"));
    Serial.print(F("Averaged 12V Battery Voltage = ")); Serial.print(SensorReadings.volt1); Serial.println(F(" milli Volts"));
 }

//****************************************** Temperature ********************************************
void Temperature(){
    sensors.requestTemperatures(); // DS18B20, Send the command to get temperatures on all sensors on onewire Bus
      temperature1 =sensors.getTempCByIndex(0); delay(75); // Give delay time to read DS18B20's
      temperature2 =sensors.getTempCByIndex(1); delay(75);
      temperature1 = (temperature1 * 1.8 + 32.0); // convert to DegF
      temperature2 = (temperature2 * 1.8 + 32.0); // convert to DegF
      Serial.print(F("Temp1 = ")); Serial.print(temperature1); Serial.println(F(" DegF"));
      Serial.print(F("Temp2 = ")); Serial.print(temperature2); Serial.println(F(" DegF"));
    SensorReadings.temp1 = (temperature1 *10); // Send as short int
    SensorReadings.temp2 = (temperature2 *10); // Send as short int
  }


//****************************************** ISR ********************************************
void wakeUp(){    // Interrupt Service Routine (ISR), after run will return to place that interupt started
    bucketTips++; 
    rainTotal = (bucketTips * bucketAmount); 
    //SensorReadings.generic1 = (rainTotal * 1000); // multpily by 100 before sending as a short
    SensorReadings.generic1 = bucketTips; 
    digitalWrite(LEDboard, HIGH); delay(150); digitalWrite(LEDboard, LOW); 
    }
