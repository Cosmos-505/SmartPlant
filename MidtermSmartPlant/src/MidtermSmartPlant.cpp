/* 
 * Project MIDTERM SMART PLANT
 * Author: Chris Cade
 * Date: 16 March 2024
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Air_Quality_Sensor.h"
#include "Math.h"
#include "Adafruit_BME280.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"
#include "IoTTimer.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
int waterButtonPress;
IoTTimer waterTimer; 
IoTTimer moistureTimer;
IoTTimer publishTimer;

/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();

// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Subscribe waterButton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/waterbutton"); 

Adafruit_MQTT_Publish mqtttemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp");
Adafruit_MQTT_Publish mqttpress = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/press");
Adafruit_MQTT_Publish mqttrHumidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/rHumidity");
Adafruit_MQTT_Publish mqttmoisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisture");
Adafruit_MQTT_Publish mqttAQ = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/AirQuality");
Adafruit_MQTT_Publish mqttdust = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dust");

//Declare the BME
Adafruit_BME280 bme;  // the I2C device that measures

float tempC;
float tempF;
float pascals;
float humidRH;
float mercury; 
const char degree = 0xF8;

int status; 

////ESTABLISH AIR QUALITY SENSOR AND DISPLAY
AirQualitySensor airqualitysensor = A2;
int current_quality =-1;

int dustPin = D3;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;//sampe 30s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;
int airQ;


//WATERPIN
const int WATERPIN = D16;

// Declare Global Variables in Header
String DateTime, TimeOnly;

//declare the OLED
const int OLED_RESET =-1;
Adafruit_SSD1306 display(OLED_RESET);

//decalre the moisture sensor and pin
int sensorPin = D11;
float moisture;



void setup() {

  // Connect to Internet but not Particle Cloud
  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
//INITIALIZE THE BME
// Initialize the bme
status = bme.begin(0x76);
  if (status == false) {
    Serial.printf("BME at address 0x%02X failed to start",0x76);
  }

  // Setup MQTT subscription
  mqtt.subscribe(&waterButton);


//AIR QUALITY SENSOR
airqualitysensor.init();
    pinMode(dustPin,INPUT);
    starttime = millis();//get the current time;


//WATERPIN OUTPUT
    pinMode(WATERPIN, OUTPUT);


Time.zone ( -6); // MST = -7, MDT = -6
Particle.syncTime (); // Sync time with Particle Cloud

Serial.begin(9600);
waitFor(Serial.isConnected,10000);

pinMode (sensorPin, INPUT);

display.begin(SSD1306_SWITCHCAPVCC,0x3C);
display.clearDisplay();
display.setRotation(2);
display.setTextSize(1);
display.setTextColor(WHITE);
display.setCursor(0,0);
display.printf ("Hello world!\n Welcome to \n GROWING GREEN!");
display.display();
display.clearDisplay();

publishTimer.startTimer(30000);
}


void loop()  {

  MQTT_connect();
  MQTT_ping();
  

  // this is our 'wait for incoming subscription packets' busy subloop 
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) { 
    if (subscription == &waterButton) {
      waterButtonPress = atoi((char *)waterButton.lastread);
      Serial.printf("Water Button is pressed %i\n", waterButtonPress);
    }

  }

if (waterButtonPress == 1) {
  digitalWrite(WATERPIN,HIGH);
  waterTimer.startTimer(3000);
}
  if (waterTimer.isTimerReady()) {
    digitalWrite (WATERPIN,LOW);
    display.begin(SSD1306_SWITCHCAPVCC,0x3C);
    display.clearDisplay();
  }
  
  

    //use bme to read all values
tempC = bme.readTemperature();
pascals = bme.readPressure();
humidRH = bme.readHumidity();

tempF = map(int(tempC),0,100,32,212);
//mercury = map(pascals,29.92,106500,31.95,108100);
mercury = pascals*0.0003; 


if (publishTimer.isTimerReady()) {
  if(mqtt.Update()) {
//set display parameters for OLED
display.clearDisplay();
display.setTextSize(1);
display.setTextColor(WHITE);
display.setCursor(0,0);  
display.printf ("Temperature is %0.1f\n",tempF);
mqtttemp.publish(tempF);
display.printf("Humidity is %0.1f\n", humidRH);
mqttrHumidity.publish(humidRH);
display.printf("Pressure is %0.1f\n", mercury);
mqttpress.publish(mercury);
mqttmoisture.publish(moisture);
mqttAQ.publish(airQ);
mqttdust.publish(concentration);
display.display();
 
display.clearDisplay();
display.setCursor(0,0);
//print to monitor
Serial.printf ("Temperature is %0.1f %c\n",tempF,degree);
Serial.printf("Humidity is %f\n", humidRH);
Serial.printf("Pressure is %f\n", mercury); 
publishTimer.startTimer(30000);
}
}

// AIR QUALITY AND DUST SENSORS
airQ = airqualitysensor.getValue();

    current_quality = airqualitysensor.slope();
    if (current_quality >= 0) {
        // Print air quality message based on slope
        if (current_quality == 0)
            Serial.printf("High pollution! Force signal active %i\n", airQ);
        else if (current_quality == 1)
            Serial.printf("High pollution! %i\n", airQ);
        else if (current_quality == 2)
            Serial.printf("Low pollution! %i\n", airQ);
        else if (current_quality == 3)
            Serial.printf("Fresh air %i\n", airQ);

// DUST SENSOR
    duration = pulseIn(dustPin, LOW);
    lowpulseoccupancy = lowpulseoccupancy+duration;

    if ((millis()-starttime) > sampletime_ms)//if the sampel time == 30s
    {
    ratio = lowpulseoccupancy / (sampletime_ms * 10.0);  
    concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; 
    Serial.printf(" Pulse is %i,\n Ratio is %.2f,\n Concentration is %.2f\n", lowpulseoccupancy, ratio, concentration);
    lowpulseoccupancy = 0;
    starttime = millis();
    


//WATERPUMP AND RELAY
 if (moisture <= 2000) {
  digitalWrite(WATERPIN,HIGH);
 moistureTimer.startTimer(3000);
 }
  if (moistureTimer.isTimerReady()) {
    digitalWrite (WATERPIN,LOW);
    display.begin(SSD1306_SWITCHCAPVCC,0x3C);
 }
 

//  MOISTURE SENSOR AND CONTROL
DateTime = Time.timeStr (); // Current Date and Time from Particle Time class
TimeOnly = DateTime.substring (11 ,19) ; // Extract the Time from the DateTime String

//%s prints an array of char
// the . c_str () method converts a String to an array of char
Serial.printf (" Date and time is %s\n",DateTime.c_str ());
Serial.printf (" Time is %s\n",TimeOnly.c_str ());


//MOISTURE SENSOR
moisture = analogRead(sensorPin);

Serial.printf("Moisture content is %f\n", moisture);
display.printf("Moisture content is\n %f\n", moisture);
display.printf(" Date and time is %s\n",DateTime.c_str());
display.display();

display.clearDisplay();
display.setCursor(0,0);
}
}
}












bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}