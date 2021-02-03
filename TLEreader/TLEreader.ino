// adding comment for git purposes - test4
#include <xOD01.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

const char* ssid = "<hidden>";               // your network SSID (name)
const char* pass = "<hidden>" ;             // your network password
char servername[]="celestrak.com";           // Celestrak Server
int wifi_attempt_counter = 1;

// OD01 instance variable
xOD01 OD01;

WiFiClient client;



void setup() {

  #ifdef ESP8266
  Wire.pins(2, 14);
  #endif
  Wire.begin();
  Serial.begin(115200);

  OD01.begin();  // initialize OLED
  OD01.clear();
  Serial.println("Starting the OD01");

  delay(1000);
  Serial.println("Attempting to connect to WiFi");
//  OD01.print("OLED test number: ");
  wifi_attempt_counter = 1;

  delay(1000);
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

//    OD01.clear();
//    delay(2000);
//    OD01.print("OLED test number: ");
//    OD01.println(wifi_attempt_counter);
//    delay(1000);
  }

    Serial.println("Connected to wifi");
    OD01.println("Connected to wifi!");
    Serial.println("\nStarting connection with server...");
    OD01.println("\nStarting connection with server...");

}

void makeRequest(){
    // if you get a connection, report back via serial:
    if (client.connect(servername, 80)) {
    Serial.println("connected to server");
    OD01.println("connected to server");
    Serial.println();



    // Make HTTP request:
    client.println("GET /NORAD/elements/stations.txt HTTP/1.0");     // rest of url for your chosen txt file, i.e extension following celestrak.com , Replace everything EXCEPT: GET HTTP/1.0
    client.println();
//    Serial.println(client);
    }

   // if there are incoming bytes available
   // from the server, read them and print them:
  String c;
  int lineCounter=0;
 while (!client.available()){
  // while loop runs while waiting for server availability
 }

    // BONUS QUESTION -- get TLE identifier from user
//    char desired_TLE[] ;
    Serial.println("Which TLE do you want? ... (input here ->)");
    while (!Serial.available()) {
        //     wait until user inputs data in monitor
    }

    String desired_TLE_String; // define String
    desired_TLE_String = Serial.readString();
    Serial.print("You said:");
    Serial.println(desired_TLE_String);


// Skip HTTP headers
 char endOfHeaders[] = "\r\n\r\n";
  if (!client.find(endOfHeaders))
  {
    Serial.println(F("Invalid response"));
    OD01.println(F("Invalid response"));
    return;
  }

      // BONUS QUESTION -- parse HTTP response and isolate desired TLE
int desired_index;
String line;
 while (client.available()) {
    c = client.readString(); // copy HTTP data into string c, so we can manipulate that
 }

desired_index = c.indexOf(desired_TLE_String); // find TLE identifier
c = c.substring(desired_index); // cut string at TLE identifier
Serial.println("------ Response: ------");

char p;
for (int i = 0; i <= c.length(); i++) {
    p = c[i];
    Serial.print(c[i]);
    if (p == '\n'){
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
    OD01.println("disconnecting from server");
    client.stop();
  }
}

void loop() {

    makeRequest();
}
