// Code integrated by Kevin Okseniuk for AA236A
// Significant portions of the code were developed out of examples in Xinabox tutorials.
// see https://github.com/xinabox/ for more details.

#include <xCore.h>
#include <xSI01.h>
#include <xSN01.h>
#include <xSL01.h>
#include <xSW01.h>
#include <xOD01.h>

//Initialize each sensor object
xSI01 SI01;
xSW01 SW01;
xSL01 SL01;
xSN01 SN01;

void setup() {
  // put your setup code here, to run once:
  delay(1500);
  Serial.begin(9600);
  Serial.println("********** xChip is alive! **********");
  Serial.println("Setup: serial initialized!");
  #ifdef ESP8266
    Wire.pins(2, 14);
  #endif
  Wire.begin();
  Serial.println("Setup: I2C initialized!");

  routineCheckout(true);
// Conclude checkout
  Serial.println("====== Setup checkout program completed! =======");

}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println(">> Waiting 5 seconds until next check...");
  delay(5000);
  routineCheckout(false);
}

void routineCheckout(boolean initialFlag) {

// SI01 - call IMU Poll function
  if(initialFlag){
  SI01.begin();
  Serial.println("Initial Checkout: SI01 interface begun");}
  delay(1000);
  Serial.println("Polling SI01...");
  pollIMU();

// SW01 - call Weather Poll function
  if(initialFlag){
  SW01.begin();
  Serial.println("Initial Checkout: SW01 interface begun");
  delay(1000);}
  Serial.println("Polling SW01...");
  pollWeather();

// SL01 - call Light Poll function
  if(initialFlag){
  SL01.begin();
  Serial.println("Initial Checkout: SL01 interface begun");}
  delay(1000);
  Serial.println("Polling SL01...");
  pollLux();

// SN01 - call GPS poll function
  if(initialFlag){SN01.begin();
  Serial.println("Initial Checkout: SN01 interface begun");}
  delay(1000);
  Serial.println("Polling SN01...");
  pollGPS();
}

void pollGPS() {
  // Sample sensor
  SN01.poll();
  Serial.println("----- Start SN01 Poll cycle -----");

  // Get values
  String date = SN01.getDate();
  String gpstime = SN01.getTime();
  long latitude = SN01.getLatitude();
  long longitude = SN01.getLongitude();

  // Print values
  Serial.print("GPS Time: ");
  Serial.println(gpstime);
  Serial.print("GPS Date: ");
  Serial.println(date);
  Serial.print("GPS Latitude: ");
  Serial.println(latitude);
  Serial.print("GPS longitude: ");
  Serial.println(longitude);

  Serial.println("-----End SN01 Poll cycle -----");
}

void pollLux() {
  // Sample sensor
  SL01.poll();
  Serial.println("----- Start SL01 Poll cycle -----");

  // Light
  float lux = SL01.getLUX();
  Serial.print("Ambient Light Level: ");
  Serial.print(lux);
  Serial.println(" LUX");

  // UVA
  float uva = SL01.getUVA();
  Serial.print("UVA Intensity: ");
  Serial.print(uva);
  Serial.println(" uW/m^2");

  // UVB
  float uvb = SL01.getUVB();
  Serial.print("UVB Intensity: ");
  Serial.print(uvb);
  Serial.println(" uW/m^2");

  // UVIndex
  float uvindex = SL01.getUVIndex();
  Serial.print("UV Index: ");
  Serial.println(uvindex);

  Serial.println("-----End SL01 Poll cycle -----");
}

void pollWeather() {
  // Sample sensor
  SI01.poll();
  Serial.println("----- Start SW01 Poll cycle -----");

  // Pressure
  float pressure = SW01.getPressure();
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" Pa");

  // Altitude
  float datum_pressure = 101325;
  float alt = SW01.getAltitude(datum_pressure);
  Serial.print("Altitude: ");
  Serial.print(alt);
  Serial.println(" m");

  // Temperature
  float tempC = SW01.getTempC(); // Temperature in Celcuis
  float tempF = SW01.getTempF(); // Temperature in Farenheit
  Serial.print("Temperature: ");
  Serial.print(tempC);
  Serial.println(" C");
  Serial.print("Temperature: ");
  Serial.print(tempF);
  Serial.println(" F");

  // Humidity
  float humidity = SW01.getHumidity();
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.println("-----End SW01 Poll cycle -----");
}


void pollIMU() {
  // Sample sensor
  SI01.poll();
  Serial.println("----- Start SI01 Poll cycle -----");
  // Gyroscope
  Serial.print("G: ");
  Serial.print(SI01.getGX(), 2);
  Serial.print(", ");
  Serial.print(SI01.getGY(), 2);
  Serial.print(", ");
  Serial.println(SI01.getGZ(), 2);

  // Accelerometer
  Serial.print("A: ");
  Serial.print(SI01.getAX(), 2);
  Serial.print(", ");
  Serial.print(SI01.getAY(), 2);
  Serial.print(", ");
  Serial.println(SI01.getAZ(), 2);

  // Magnitude
  Serial.print("M: ");
  Serial.print(SI01.getMX(), 2);
  Serial.print(", ");
  Serial.print(SI01.getMY(), 2);
  Serial.print(", ");
  Serial.println(SI01.getMZ(), 2);

  // Attitude
  Serial.print("Roll: ");
  Serial.println(SI01.getRoll(), 2);
  Serial.print("Pitch :");
  Serial.println(SI01.getPitch(), 2);
  Serial.print("GForce :");
  Serial.println(SI01.getGForce(), 2);

  Serial.println("-----End SI01 Poll cycle -----");
}
