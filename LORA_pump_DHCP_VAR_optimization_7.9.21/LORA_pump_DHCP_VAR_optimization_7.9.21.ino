// PUMP out reciever (hard connect)
// revision 10
//7-9-21
//
//uses two switch/relay/button
//this revision attempts to allow
//a more optimized flow of code, for transistion to library org
//
//Coded by Brandon Mosburg for use with arduino IDE on ESP32
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <esp_attr.h>
#include <Preferences.h>
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define CART_ADDRESS1 1 //starts at 1 -> 4
#define CART_ADDRESS2 2 //starts at 1 -> 4
#define CART_ADDRESS3 3 //starts at 1 -> 4
#define CART_ADDRESS4 4 //starts at 1 -> 4
//defining pins
#define RFM95_CS   25
#define RFM95_INT  26
#define Pump_out   2 //this pin tells the Pump if the MCU wants it to turn on (pump ->MCU)
#define Pump_in    5 //this pin tells the MCU if the pump is currently on (pump ->MCU)
// must match RX's freq!
#define RF95_FREQ 915.0
#define interval  10 * 1000 // set to (secs) * millis
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//instance of Non-Volatile Storage
Preferences Non_Volatile;
//vars for address allocation
uint8_t CART_ADDYS; //The address of the carts
uint8_t PUMP_ADDRESS; //The Pumps Address
bool pump = 0; //var for pump status
// Singleton instance of the radio driver
RH_RF95 driver(RFM95_CS, RFM95_INT);
// Singleton instance of the addressed radio driver manager
RHReliableDatagram rf95(driver);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Allocate_Non_Volatiles ();
  pinMode(Pump_out, OUTPUT);
  pinMode(Pump_in, INPUT);
  Radio_Startup ();
  if (PUMP_ADDRESS == 0) Message_Carts (3);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  unsigned long CurrentMillis;
  unsigned long PrevMillis;
  PrevMillis = millis();
  Receive ();
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // when the pumping relay is closed (pump is on)
  if ((pump == 0) && (digitalRead(Pump_in) == 1)) {
    digitalWrite(Pump_out, HIGH);
    pump = 1;
    Message_Carts (1);
  }
  /////////////////////////////////////////////////////////////////////////////
  //if the pump was on and it is now off
  else if ((pump == 1) && (digitalRead(Pump_in) == 0)) {
    digitalWrite(Pump_out, LOW);
    pump = 0;
    Message_Carts (0);
  }
  //while the pump is chilling on
  while ((pump == 1) && (digitalRead(Pump_in) == 1)) {
    CurrentMillis = millis();
    if (CurrentMillis - PrevMillis > interval) {
      // save the last time we sent to server
      PrevMillis = CurrentMillis;
      Message_Carts (2);
    }
    Non_Volatile.putUChar("pump", pump);
    Non_Volatile.end();
    ESP.restart();
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Allocate_Non_Volatiles () {
  Non_Volatile.begin("my-app", false);
  PUMP_ADDRESS = Non_Volatile.getUChar("PUMP_ADDRESS", 0);
  CART_ADDYS = Non_Volatile.getUChar("CART_ADDRESS", 0);
  pump = Non_Volatile.getUChar("pump", 0);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Receive () {
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;
  uint8_t addy;
  if (rf95.recvfromAckTimeout(buf, &len, 1000, &from)) {
    Cart_Archive (from);

    Serial.print("Got reply from: ");
    Serial.print(from, DEC);
    Serial.print(" : ");
    Serial.println((char*)buf);
    Serial.print("RSSI: ");
    Serial.println(driver.lastRssi(), DEC);

    if (strcmp((char*)buf, "Turn On") == 0) {
      digitalWrite(Pump_out, HIGH);
      pump = 1;
      Message_Carts (69);
      delay(10000);//testing for relay debouce... needs to be long enough for pump to tell the mcu that iot has tunred on
    }
    else if (strcmp((char*)buf, "LED Off") == 0); //\
    else if (strcmp((char*)buf, "LED On") == 0);  // > just here for formality
    else if (strcmp((char*)buf, "Noted") == 0);   ///
    else if (strncmp((char*)buf, "Add ", 4) == 0) {
      if (buf[5] > 0) addy = (buf[4] - 48) * 10 + (buf[5] - 48);
      else addy = (buf[4] - 48);

      Serial.println("setting addy to");
      Serial.println(addy);
      rf95.setThisAddress(addy);
      PUMP_ADDRESS = addy;
      Serial.println(rf95.thisAddress());
      Message_Carts (4);
      Non_Volatile.putUChar("PUMP_ADDRESS", PUMP_ADDRESS);
    }
  }
  else Serial.println("Message not Received...");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Radio_Startup () {
  rf95.setThisAddress(PUMP_ADDRESS);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!driver.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  driver.setTxPower(23, false);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Cart_Archive (uint8_t from) {
  uint8_t Cart_Num;
  if (from == 1) Cart_Num = 1;
  if (from == 2) Cart_Num = 2;
  if (from == 3) Cart_Num = 4;
  if (from == 4) Cart_Num = 8;
  CART_ADDYS = Cart_Num | CART_ADDYS;
  Non_Volatile.putUChar("CART_ADDRESS", CART_ADDYS);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Message_Carts (uint8_t type) {
  uint8_t dat[250];
  Serial.println("Attempting to Send");
  if (type == 0) {
    strcpy((char*)dat, "Pump Off");
    //tell the cart the pump is off
    rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS1);
    if (CART_ADDYS & 2) rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS2);
    if (CART_ADDYS & 4) rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS3);
    if (CART_ADDYS & 8) rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS4);
    delay(1600); // no fucking clue why this is needed, doesnt work without the delay
    //apparently its needed since the radio wants <1% duty cycle
    Receive ();
  }
  else if (type == 1) {
    strcpy((char*)dat, "Pump On");
    //tell the cart that the pump is on--
    rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS1);
    if (CART_ADDYS & 2) rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS2);
    if (CART_ADDYS & 4) rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS3);
    if (CART_ADDYS & 8) rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS4);
    delay(1600); // no fucking clue why this is needed, doesnt work without the delay
    //apparently its needed since the radio wants <1% duty cycle
    Receive ();
  }
  else if (type == 2) {
    strcpy((char*)dat, "Still On"); //tell the cart the pump is still on
    rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS1);
    if (CART_ADDYS & 2) rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS2);
    if (CART_ADDYS & 4) rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS3);
    if (CART_ADDYS & 8) rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS4);
    Receive ();
  }
  else if (type == 3) {
    strcpy((char*)dat, "Address Me"); //ask for an address
    rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS1);
    delay(1500);//needed to prevent crosstalk
  }
  else if (type == 4) {
    strcpy((char*)dat, "IM ADDRESSED");
    rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS1);
  }
  else {
    strcpy((char*)dat, "Pump turning On"); //tell the cart that the pump is on--
    rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS1);
    if (CART_ADDYS & 2) rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS2);
    if (CART_ADDYS & 4) rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS3);
    if (CART_ADDYS & 8) rf95.sendtoWait(dat, sizeof(dat), CART_ADDRESS4);
  }
  Serial.print("Sent : ");
  Serial.println((char*)dat);
}
