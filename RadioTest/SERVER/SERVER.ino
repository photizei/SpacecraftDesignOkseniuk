
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

*************************************************************/

#include <xCore.h>
//#include <SC18IS602.h>
#include <xRL0x.h>

#define RL03_FREQ 915.0
#define CW01_RED 12
#define CW01_GREEN 13
#define CW01_BLUE 5

int msg_num = 0;
int glb_msg_num = 0;
float avg_rssi = 0;
float last_rssi ;

void setup() {
  // Start the Serial Monitor
  Serial.begin(115200);

  // Set the RGB Pin directions
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Start the I2C Comunication
  Wire.begin();




  if (!RL0X.begin()) { // <-- enter radio name here
    Serial.println("Check the connector to RL03");
    while (1) {
      // Flash RED to indicate failure
      digitalWrite(LED_RED, HIGH);
      delay(100);
      digitalWrite(LED_RED, LOW);
      delay(100);
    }
  } else {
    // RL0X Initialized correctly
    RL0X.setModemConfig(RL0X.Bw500Cr45Sf128);

    RL0X.setFrequency(RL03_FREQ);
    RL0X.setTxPower(23, false);
  }
  Serial.println("setup done");
}

void loop() {

  digitalWrite(LED_BUILTIN,HIGH);
  Serial.println("Waiting");
  Serial.println("-------- Config ID = 2");
  if (RL0X.waitAvailableTimeout(3000)) {
    uint8_t buf[195];
    uint8_t len = sizeof(buf);
    if (RL0X.recv(buf, &len)) {
      digitalWrite(LED_RED, HIGH);
      msg_num = msg_num + 1 ;
      glb_msg_num = glb_msg_num + 1;
      Serial.print("Got message from client! Msg Id: ");
      Serial.println(msg_num);
      Serial.println((char*)buf);
      last_rssi = RL0X.lastRssi();
      Serial.print("RSSI: ");
      Serial.println(last_rssi, DEC);
      avg_rssi = ((avg_rssi * (msg_num-1)) + last_rssi) / msg_num;
      Serial.print("Average RSSI in this config: ");
      Serial.println(avg_rssi, DEC);
      Serial.println("-------- Config ID = 2");

      // Send a reply
      uint8_t data[] = {glb_msg_num};
      delay(100);
      RL0X.send(data, sizeof(data));
      Serial.println("Sent a reply");
      digitalWrite(LED_RED, LOW);
    } else {
      Serial.println("recv failed");
    }
    // Bonus problem
    // watchModemConfig(glb_msg_num);
  }
  digitalWrite(LED_BUILTIN,LOW);
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
        if (p) { Serial.println("****** Server config changed to id: 1"); }
        else{Serial.println("Bad config attempted!");}
    }
    else if (cid == 2) {
      p = RL0X.setModemConfig(RL0X.Bw500Cr45Sf128);
        if (p) { Serial.println("******  Server config changed to id: 2"); }
        else{Serial.println("Bad config attempted!");}
    }
    else if (cid == 3) {
      p = RL0X.setModemConfig(RL0X.Bw31_25Cr48Sf512);
        if (p) { Serial.println("****** Server config changed to id: 3"); }
        else{Serial.println("Bad config attempted!");}
    }

    else if (cid == 4) {
      p = RL0X.setModemConfig(RL0X.Bw125Cr48Sf4096);
        if (p) { Serial.println("Server config changed to id: 4"); }
        else{Serial.println("Bad config attempted!");}
    }
    avg_rssi = 0;
    msg_num = 0;
 }
}
