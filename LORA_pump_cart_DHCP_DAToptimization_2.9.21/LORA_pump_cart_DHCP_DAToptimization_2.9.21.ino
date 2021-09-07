//cart transponder for 3+ pumps, tested with 3, scalable to 8
//this revision attempts to allow
//a more optimized flow of code, for transistion to library org
///
//Coded by Brandon Mosburg for use with arduino IDE on ESP32
//3-9-21
#include <RHReliableDatagram.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <esp_sleep.h>
#include <esp_attr.h>
//////////////////////////////////////////////////////////////////
//addresses for lora radio comms
#define CART_ADDRESS1 1
#define CART_ADDRESS2 2
#define CART_ADDRESS3 3
#define CART_ADDRESS4 4
#define PUMP1_ADDRESS 5
#define PUMP2_ADDRESS 6
#define PUMP3_ADDRESS 7
#define PUMP4_ADDRESS 8
#define PUMP5_ADDRESS 9
#define PUMP6_ADDRESS 10
#define PUMP7_ADDRESS 11
#define PUMP8_ADDRESS 12
//registers for pump configuration
#define PUMP1 0b00000001
#define PUMP2 0b00000010
#define PUMP3 0b00000100
#define PUMP4 0b00001000
#define PUMP5 0b00010000
#define PUMP6 0b00100000
#define PUMP7 0b01000000
#define PUMP8 0b10000000
///////////////////////////////////////////////////////////////////
//pin def for lora radio
#define RFM95_CS 25
#define RFM95_INT 26
//pin def for leds
#define CUTOFF 13 // latch lock
#define NOTEone 5 // latch IO1
#define NOTEtwo 2 // latch IO2
#define NOTEthree 21 // latch IO3
//untested  pin allocation
//#define NOTEfour 16 // latch IO4
//#define NOTEfive 17 // latch IO5
//#define NOTEsix 18 // latch IO6
//#define NOTEseven 19 // latch IO7
//#define NOTEeight 20 // latch IO8
//pin def for buttons
#define PUMPone 12
#define PUMPtwo 14
#define PUMPthree 4
//#define PUMPfour 27
//#define PUMPfive 13
//#define PUMPsix 32
//#define PUMPseven 33
//#define PUMPeight 35
//def for wakeup pins
#define BUTTON_PIN_BITMASK 0xB0C007010
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
/////////////////////////////////////////////////////////////////////
RTC_DATA_ATTR uint8_t pumps; //state space of pumps
RTC_DATA_ATTR uint8_t CLOSED_ADDRESS; //archive for used Addresses
/////////////////////////////////////////////////////////////////////
// Singleton instance of the radio driver
RH_RF95 driver(RFM95_CS, RFM95_INT);
// Singleton instance of the addressed radio driver
RHReliableDatagram rf95(driver, CART_ADDRESS1);// --change for each cart max 4 together
//var for Button check
uint8_t Bootons;
////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);

  Pin_Setup ();
  Startup_light();
  Radio_Startup ();
  Wakeup_Protocol ();
  the_big_sleep();
}
/////////////////////////////////////////////////////////////////
void loop() {}
/////////////////////////////////////////////////////////////
void  Pin_Setup () {
  //led pin setup
  pinMode(CUTOFF, OUTPUT);
  pinMode(NOTEone, OUTPUT);
  pinMode(NOTEtwo, OUTPUT);
  pinMode(NOTEthree, OUTPUT);
  //  pinMode(NOTEfour, OUTPUT);
  //  pinMode(NOTEfive, OUTPUT);
  //  pinMode(NOTEsix, OUTPUT);
  //  pinMode(NOTEseven, OUTPUT);
  //  pinMode(NOTEeight, OUTPUT);
  //button pin setup
  pinMode(PUMPone, INPUT);
  pinMode(PUMPtwo, INPUT);
  pinMode(PUMPthree, INPUT);
  //  pinMode(PUMPfour, INPUT);
  //  pinMode(PUMPfive, INPUT);
  //  pinMode(PUMPsix, INPUT);
  //  pinMode(PUMPseven, INPUT);
  //  pinMode(PUMPeight, INPUT);
}
//////////////////////////////////////////////////////////////////////
void  Startup_light() {// sets the lights that need to be on based on the state space of the pumps
  Serial.println("---------------------------------------------------------------------");
  low_NOTES ();
  if (pumps & PUMP1) digitalWrite(NOTEone, HIGH);
  if (pumps & PUMP2) digitalWrite(NOTEtwo, HIGH);
  if (pumps & PUMP3) digitalWrite(NOTEthree, HIGH);
  //if (pumps & PUMP4) digitalWrite(NOTEfour, HIGH);
  //if (pumps & PUMP5) digitalWrite(NOTEfive, HIGH);
  //if (pumps & PUMP6) digitalWrite(NOTEsix, HIGH);
  //if (pumps & PUMP7) digitalWrite(NOTEseven, HIGH);
  //if (pumps & PUMP8) digitalWrite(NOTEeight, HIGH);
  digitalWrite(CUTOFF, HIGH);
}
/////////////////////////////////////////////////////////////
void  low_NOTES () {//makes all lights low
  digitalWrite(NOTEone, LOW);
  digitalWrite(NOTEtwo, LOW);
  digitalWrite(NOTEthree, LOW);
  //digitalWrite(NOTEfour, LOW);
  //digitalWrite(NOTEfive, LOW);
  //digitalWrite(NOTEsix, LOW);
  //digitalWrite(NOTEseven, LOW);
  //digitalWrite(NOTEeight, LOW);
}
//////////////////////////////////////////////////////////////////////
void  Radio_Startup () { // startup sequence for the LORA radio
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  if (!driver.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  driver.setTxPower(23, false);
}
//////////////////////////////////////////////////////////////////////
void Wakeup_Protocol () { // finds the reason for waking up and responds accordingly
  uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status();
  uint8_t Wakeup_reason = log(GPIO_reason) / log(2);
  uint8_t TO;
  if (Wakeup_reason == 26) Recieve_Reply ();
  else {
    if (Wakeup_reason == 12) {
      TO = 5;
      while (digitalRead(PUMPone) == 1);
      Send_On_To_Pump (TO);
    }
    else if (Wakeup_reason == 14) {
      TO = 6;
      while (digitalRead(PUMPtwo) == 1);
      Send_On_To_Pump (TO);
    }
    else if (Wakeup_reason == 4) {
      TO = 7;
      while (digitalRead(PUMPthree) == 1);
      Send_On_To_Pump (TO);
    }
    //  else if (Wakeup_reason == 27) {
    //TO = 8;
    //    while (digitalRead(PUMPfour) == 1);
    //Send_On_To_Pump (TO);
    //  }
    //  else if (Wakeup_reason == 13) {
    //TO = 9;
    //    while (digitalRead(PUMPfive) == 1);
    //Send_On_To_Pump (TO);
    //  }
    //  else if (Wakeup_reason == 32) {
    //TO = 10;
    //    while (digitalRead(PUMPsix) == 1);
    //Send_On_To_Pump (TO);
    //  }
    //  else if (Wakeup_reason == 33) {
    //TO = 11;
    //    while (digitalRead(PUMPseven) == 1);
    //Send_On_To_Pump (TO);
    //  }
    //  else if (Wakeup_reason == 35) {
    //TO = 12;
    //    while (digitalRead(PUMPeight) == 1);
    //Send_On_To_Pump (TO);
    //  }
    //Send_On_To_Pump (TO);
  }
}
//////////////////////////////////////////////////////////////////////
void  Send_On_To_Pump (uint8_t TO) { //tells the pump to turn on
  uint8_t dat[250];
  strcpy((char*)dat, "Turn On");
  Serial.print("Attempting to Send Turn On to ");
  Serial.println(TO, HEX);
  rf95.sendtoWait(dat, sizeof(dat), TO);
  Serial.print("Sent : ");
  Serial.println((char*)dat);
  Recieve_Reply ();
}
//////////////////////////////////////////////////////////////////////
void  Recieve_Reply () { // checks for a message then receives and reacts accordingly
  Serial.println("Possible message incoming...");
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (rf95.recvfromAckTimeout(buf, &len, 2000, &from)) {
    uint8_t dat[250];
    uint8_t PUMP = Pump_Config(from);

    Serial.print("Got MSG from : ");
    Serial.print(from, HEX);
    Serial.print(" : ");
    Serial.println((char*)buf);
    Serial.print("RSSI: ");
    Serial.println(driver.lastRssi(), DEC);
    delay(100);

    if (strcmp((char*)buf, "Pump Off") == 0) {
      strcpy((char*)dat, "LED Off");
      Serial.print("Attempting to Send LED off to ");
      Serial.println(from, HEX);
      rf95.sendtoWait(dat, sizeof(dat), from);
      Signal_off(PUMP);
    }
    else if (strcmp((char*)buf, "Pump On") == 0) {
      strcpy((char*)dat, "LED On");
      Serial.print("Attempting to Send LED on to ");
      Serial.println(from, HEX);
      rf95.sendtoWait(dat, sizeof(dat), from);
      Signal_on(PUMP);
    }
    else if (strcmp((char*)buf, "Pump turning On") == 0) Signal_on(PUMP);
    else if (strcmp((char*)buf, "Still On") == 0) {
      Address_Archive(from);
      Signal_on(PUMP);
    }
    else if ((!(strcmp((char*)buf, "Im On")) || !(strcmp((char*)buf, "IM ADDRESSED"))) == 1) {
      Address_Archive(from);
      strcpy((char*)dat, "Noted");
      Serial.print("Attempting to Send Noted to ");
      Serial.println(from, HEX);
      rf95.sendtoWait(dat, sizeof(dat), from);
    }
    else if (strcmp((char*)buf, "Address Me") == 0) {
      uint8_t addy = Allocate_Address();
      Serial.println(addy);
      Serial.print("Attempting to Send Address to ");
      Serial.println(from, HEX);
      sprintf((char*)dat, "Add %d", addy);
      rf95.sendtoWait(dat, sizeof(dat), from);
    }
    Serial.print("Sent : ");
    Serial.println((char*)dat);
  }
  else Serial.println("Receive failed");
}
/////////////////////////////////////////////////////////////
uint8_t Pump_Config(uint8_t from) { //returns which pump binary a message was recieved from
  if (from == 5) return (PUMP1);
  else if (from == 6) return (PUMP2);
  else if (from == 7) return (PUMP3);
  else if (from == 8) return (PUMP4);
  else if (from == 9) return (PUMP5);
  else if (from == 10) return (PUMP6);
  else if (from == 11) return (PUMP7);
  else if (from == 12) return (PUMP8);
}
/////////////////////////////////////////////////////////////
void  Signal_on(uint8_t pump_num) { //turns on the light based on which pump messaged
  pumps = pumps | pump_num;
  if (pump_num == PUMP1) digitalWrite(NOTEone, HIGH);
  else if (pump_num == PUMP2) digitalWrite(NOTEtwo, HIGH);
  else if (pump_num == PUMP3) digitalWrite(NOTEthree, HIGH);
  //else if (pump_num == PUMP4) digitalWrite(NOTEfour, HIGH);
  //else if (pump_num == PUMP5) digitalWrite(NOTEfive, HIGH);
  //else if (pump_num == PUMP6) digitalWrite(NOTEsix, HIGH);
  //else if (pump_num == PUMP7) digitalWrite(NOTEseven, HIGH);
  //else if (pump_num == PUMP8) digitalWrite(NOTEeight, HIGH);
}
/////////////////////////////////////////////////////////////
void  Signal_off(uint8_t pump_num) {//turns off the light based on which pump messaged
  pumps = pumps ^ pump_num;
  if (pump_num == PUMP1) digitalWrite(NOTEone, LOW);
  else if (pump_num == PUMP2) digitalWrite(NOTEtwo, LOW);
  else if (pump_num == PUMP3) digitalWrite(NOTEthree, LOW);
  //else if (pump_num == PUMP4) digitalWrite(NOTEfour, LOW);
  //else if (pump_num == PUMP5) digitalWrite(NOTEfive, LOW);
  //else if (pump_num == PUMP6) digitalWrite(NOTEsix, LOW);
  //else if (pump_num == PUMP7) digitalWrite(NOTEseven, LOW);
  //else if (pump_num == PUMP8) digitalWrite(NOTEeight, LOW);
}
//////////////////////////////////////////////////////////////////////
void Address_Archive(uint8_t p) { //archives addresses that are in use
  if (p == 5) CLOSED_ADDRESS = CLOSED_ADDRESS | PUMP1;
  else if (p == 6) CLOSED_ADDRESS = CLOSED_ADDRESS | PUMP2;
  else if (p == 7) CLOSED_ADDRESS = CLOSED_ADDRESS | PUMP3;
  else if (p == 8) CLOSED_ADDRESS = CLOSED_ADDRESS | PUMP4;
  else if (p == 9) CLOSED_ADDRESS = CLOSED_ADDRESS | PUMP5;
  else if (p == 10) CLOSED_ADDRESS = CLOSED_ADDRESS | PUMP6;
  else if (p == 11) CLOSED_ADDRESS = CLOSED_ADDRESS | PUMP7;
  else if (p == 12) CLOSED_ADDRESS = CLOSED_ADDRESS | PUMP8;
}
//////////////////////////////////////////////////////////////////////
uint8_t Allocate_Address() { //returns the next address that should be used by a pump
  if (CLOSED_ADDRESS & PUMP8) return 0;
  else if (CLOSED_ADDRESS & PUMP7) return 12;
  else if (CLOSED_ADDRESS & PUMP6) return 11;
  else if (CLOSED_ADDRESS & PUMP5) return 10;
  else if (CLOSED_ADDRESS & PUMP4) return 9;
  else if (CLOSED_ADDRESS & PUMP3) return 8;
  else if (CLOSED_ADDRESS & PUMP2) return 7;
  else if (CLOSED_ADDRESS & PUMP1) return 6;
  else return 5;
}
//////////////////////////////////////////////////////////////////////
void  the_big_sleep () {//sends the device into low power sleep
  driver.setModeRx();
  digitalWrite(CUTOFF, LOW);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
  Serial.println("sleep");
  esp_deep_sleep_start();
}
