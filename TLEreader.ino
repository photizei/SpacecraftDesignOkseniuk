#include <xCore.h>
#include <xVersion.h>


#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <SSD1306init.h>
#include <xOD01.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

const char* ssid = "Stanford";               // your network SSID (name)
const char* pass = "" ;             // your network password
char servername[]="celestrak.com";           // Celestrak Server
int wifi_attempt_counter = 1;

// OD01 instance variable
xOD01 OD01;

WiFiClient client;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Attempting to connect to WiFi");
  wifi_attempt_counter = 1;

 // initialize OLED
  OD01.begin();
  Serial.println("Starting the OD01");
  OD01.clear();

  delay(2000);
  Serial.println("Available WiFi networks:");
  delay(1000);
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++)
  {
    Serial.println(WiFi.SSID(i));
  }
  
  WiFi.begin(ssid, pass);
  while ( WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print("... connection attempt number:");
    Serial.println(wifi_attempt_counter);
    wifi_attempt_counter = wifi_attempt_counter + 1;

    OD01.clear();
    delay(2000);
    OD01.print("OLED test number: ");
    OD01.println(wifi_attempt_counter);
    delay(1000);
  }

    Serial.println("Connected to wifi");
    Serial.println("\nStarting connection with server...");

}

void makeRequest(){
    // if you get a connection, report back via serial:
    if (client.connect(servername, 80)) {
    Serial.println("connected to server");
    Serial.println();
    Serial.print("TLE for: ");
    // Make HTTP request:
    client.println("GET /NORAD/elements/stations.txt HTTP/1.0");     // rest of url for your chosen txt file, i.e extension following celestrak.com , Replace everything EXCEPT: GET HTTP/1.0
    client.println();
    }

   // if there are incoming bytes available
   // from the server, read them and print them:
  char c;
  int lineCounter=0;
 while (!client.available()){
  // while loop runs while waiting for server availability
 }

// Skip HTTP headers
 char endOfHeaders[] = "\r\n\r\n";
  if (!client.find(endOfHeaders))
  {
    Serial.println(F("Invalid response"));
    return;
  }

 while (client.available()) {
    c = client.read();
    Serial.print(c);

    if (c == '\n'){
      lineCounter = lineCounter+1;
    }

    if (lineCounter==3){
      client.stop();
      break;
    }
  }

  // if the server becomes disconnected, stop the client:
  if (!client.connected()) {
    Serial.println();
    Serial.println("disconnecting from server");
    client.stop();
  }
}

void loop() {
  
    makeRequest();
}
