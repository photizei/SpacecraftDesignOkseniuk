/*************************************************************
  This is an examples for the RL01-02-03 Radio Range

  You can buy one on our store!
  -----> https://xinabox.cc/products/RL01/
  -----> https://xinabox.cc/products/RL02/
  -----> https://xinabox.cc/products/RL03/

  This example requests the Alcohol sensor to measure
  the Breath Alcohol Level

  Currently Supported on the following ☒CHIPs:
  - CW01
  - CR01/02/03

  The sensor communicates over the I2C Bus.

  ------------------------TIPS--------------------------
  Change this line ----->Wire.begin(2,14);
  to this      ----->Wire.begin();
  to allow this sensor to communicate other cpus

*************************************************************/

#include <xCore.h>
#include <xRL0x.h>
#include <SPI.h>
#include <SD.h>

#define RL03_FREQ 915.0

#define Serial SerialUSB

#define CW01_RED 12
#define CW01_GREEN 13
#define CW01_BLUE 5

#ifdef ESP8266
  Wire.pins(2,14);
  Wire.setClockStretchLimit(15000);
#endif

int msg_num = 0;
int glb_msg_num = 0;
String dataString ="";
File sensorData;
//File root;
const int chipSelect = 3;


// Bonus 2
//Sd2Card card;
//SdVolume volume;
//SdFile root;

void setup() {
  Wire.begin();
  // Start the Serial Monitor
  Serial.begin(115200);

  // Set the RGB Pin directions
/*  pinMode(CW01_RED, OUTPUT);
  pinMode(CW01_GREEN, OUTPUT);
  pinMode(CW01_BLUE, OUTPUT); */

  // Start the I2C Comunication
  /*Wire.pins(2, 14);
  Wire.begin();
  Wire.setClockStretchLimit(15000);*/

  // Bonus: start SD card comms
  pinMode(chipSelect, OUTPUT);
  SD.begin(chipSelect);

  if (!RL0X.begin()) { // <-- enter radio name here
    Serial.println("Check the connector to CR01");
    while (1) {
      // Flash RED to indicate failure
      digitalWrite(CW01_RED, HIGH);
      delay(100);
      digitalWrite(CW01_RED, LOW);
      delay(100);
    }
  } else {
    // RL0X Initialized correctly
    RL0X.setModemConfig(RL0X.Bw31_25Cr48Sf512);

    RL0X.setFrequency(RL03_FREQ);
    RL0X.setTxPower(23, false);
  }
}

void loop() {
  Serial.println("Sending to RL0X Server -----");

  digitalWrite(CW01_GREEN, HIGH);

  uint8_t data[] = "Howdy! This is Kevin's client replying. You there?";
  delay(100);
  RL0X.send(data, sizeof(data));


  uint8_t buf[195];
  uint8_t len = sizeof(buf);

  if (RL0X.waitAvailableTimeout(3000)) {
    if (RL0X.recv(buf, &len)) {
      Serial.print("got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(RL0X.lastRssi(), DEC);
      Serial.println("-------- Config ID = 3");
    } else {
      Serial.println("recv failed");
    }
    msg_num = msg_num + 1 ;
    glb_msg_num = buf[0];
    //watchModemConfig(glb_msg_num);
  } else {
    digitalWrite(CW01_GREEN, LOW);
    Serial.println("No reply, is the RL01 server running ?");
    // Bonus problem
    //watchModemConfig(10*(random(4)+1));
  }

    Serial.print("Card status:");
//    bool sd_status = SD.init(SPI_HALF_SPEED, chipSelect);
bool sd_status = true;
    Serial.println(sd_status);
    dataString = String(sd_status) + "," + String(msg_num) + "," + String(RL0X.lastRssi()) ;
    saveData(); // save to SD card
}


void watchModemConfig(int mid) {
bool p ;
bool do_switch; do_switch = false;
int cid ;
// Performt triage
if ( mid == 10) {
  do_switch = true ;
  cid = 2; }

else if (mid == 20)
  { do_switch = true;
    cid = 3;}

else if (mid == 30)
  { do_switch = true;
    cid = 4;}

else if (mid == 40) {
  do_switch = true;
  cid = 1;
  glb_msg_num = 0;}



// Switch modem configs
 if (do_switch) {
    if (cid == 1) {
      p = RL0X.setModemConfig(RL0X.Bw125Cr45Sf128);
        if (p) { Serial.println("****** Client config changed to id: 1"); }
        else{Serial.println("Bad config attempted!");}
    }
    else if (cid == 2) {
      p = RL0X.setModemConfig(RL0X.Bw500Cr45Sf128);
        if (p) { Serial.println("****** Client config changed to id: 2"); }
        else{Serial.println("Bad config attempted!");}
    }
    else if (cid == 3) {
      p = RL0X.setModemConfig(RL0X.Bw31_25Cr48Sf512);
        if (p) { Serial.println("******  Client config changed to id: 3"); }
        else{Serial.println("Bad config attempted!");}
    }

    else if (cid == 4) {
      p = RL0X.setModemConfig(RL0X.Bw125Cr48Sf4096);
        if (p) { Serial.println("****** Client config changed to id: 4"); }
        else{Serial.println("Bad config attempted!");}
    }
 }
}

// Bonus 2: function adapted from
// https://rydepier.wordpress.com/2015/08/07/using-an-sd-card-reader-to-store-and-retrieve-data-with-arduino/

void saveData(){

sensorData = SD.open("data_test.csv", FILE_WRITE);
Serial.println(sensorData);
Serial.println(dataString);
if (sensorData){
sensorData.println(dataString);
Serial.println("Wrote to file!");
sensorData.close(); // close the file
}
else{
Serial.println("Error writing to file !");}
}


void printDirectory(File dir, int numTabs) {

  while (true) {

    File entry =  dir.openNextFile();

    if (! entry) {

      // no more files

      break;

    }

    for (uint8_t i = 0; i < numTabs; i++) {

      Serial.print('\t');

    }

    Serial.print(entry.name());

    if (entry.isDirectory()) {

      Serial.println("/");

      printDirectory(entry, numTabs + 1);

    } else {

      // files have sizes, directories do not

      Serial.print("\t\t");

      Serial.println(entry.size(), DEC);

    }

    entry.close();

  }
}
