/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

/*
 * Project l14_plantwatering_Final
 * Description: Smart plant watering
 * Author: Daniel Mills
 * Date:03-22-2022
 */

///////////////////////////////
//-----Include and Objects Block
#include "credentials.h"                    //creddential for ada fruit dashboard
#include "math.h"
#include <Adafruit_GFX.h>                   //library for graphics
#include <Adafruit_SSD1306.h>               //library for OLED
#include <Adafruit_BME280.h>                //Library for temp
#include <Adafruit_MQTT.h>                  //Library for publishing to Adafruit
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Air_Quality_Sensor.h"            //Library for Air Quality


//Define OLED
#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET);
String DateTime, TimeOnly;  //needed for getting the current time to send to the Adafruit dashboard 

//BME
#define BME_ADDRESS 0x76 //BME Address
Adafruit_BME280 bme;
float temp;              //var to store temperature values
float press;             //var to store pressure values
float rHumidity;         //var to store the relative humidity values

//Connecting to Adafruit Webservice
/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Publish mqtttemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp");
Adafruit_MQTT_Publish mqttpress = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/press");
Adafruit_MQTT_Publish mqttrHumidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/rHumidity");
Adafruit_MQTT_Publish mqttmoisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisture");
Adafruit_MQTT_Publish mqttAQ = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/AirQuality");
Adafruit_MQTT_Publish mqttdust = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dust");
//Adafruit_MQTT_Publish mqtttime = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/time");  //time object for MQTT
//Adafruit_MQTT_Subscribe mqttSubPumpButton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/pumpButton"); Updated Name
Adafruit_MQTT_Subscribe mqttObj2 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/FeedNamePlantB");  

//Set Sensor Pins
const int PUMP = A4;           //sets the pin to use for the pump
const int DUSTSENSOR = A2;      //sets the pin to use for the dust sensor
const int MOISTURESENSOR = A1;  //pin to use for the moisture sensor
AirQualitySensor sensor(A3);    //pin to use for the AQ sensor

bool status;      //sets for sd card

/************Declare Variables*************/
unsigned long last, lastTime;
int valueB;        //MQTT Button
int moistureValues;  //var to store the moisture probe values
int lastTime2;

//DUST SENSOR Variables (var needed for the dust sensor)
unsigned long duration;             //creates the duration var
unsigned long starttime;
unsigned long sampletime_ms = 5000;//sampe 30s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;            //creates var to store the concentration values


/////////////////////////
//-------------Setup Loop
void setup() {
  Serial.begin(9600);  //Starts the Serial monitor
  bme.begin(0x76);
  //*Setup for OLED
  Time.zone(-7);                              //Sets the mountain time zone
  Particle.syncTime();                        //syncs particle time
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.display();                          //show splashscreen (To make sure that the OLED is working)
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.display();

  //*Setup for Dust Sensor
  pinMode(DUSTSENSOR,INPUT);       //sets the pinMode for the dust sensor
  pinMode(MOISTURESENSOR, INPUT);  //sets the pinMode for the moisture probe
  pinMode(PUMP, OUTPUT);           //sets the pinMode the the motor pump

  //Connect to WiFi without going to Particle Cloud
  WiFi.connect();               //connect to wifi
  while(WiFi.connecting()) {    //for connecting to another broker
    Serial.printf(".");
  }

  mqtt.subscribe(&mqttObj2);  //subscribe to pump button from Adafruit  --changed
  starttime = millis();  //stores current time to the starttime var
}

//////////////////////////////
//--------Start of Main Loop
void loop() { 
  //*Loop for OLED
  DateTime = Time.timeStr();              // Current Date and Time from Particle Time class
  TimeOnly = DateTime.substring(11 ,19) ; // Extract the Time from the DateTime String
  

  //=====Main Functions=======
  //*MQTT Start
  MQTT_connect();
  // Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis()-last)>120000) {               //function to ping the MQTT broker
      Serial.printf("Pinging MQTT \n");             
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
    last = millis();
  }

  //*MQTT Subscription
  Adafruit_MQTT_Subscribe *subscription;                                             //looks for MQTT subscriptions for button input to turn on motor pump
  while ((subscription = mqtt.readSubscription(100))) {                              //looks for receiving signal
     if (subscription == &mqttObj2) {
        valueB = atoi((char *)mqttObj2.lastread);                                    //takes last data and converts it char and converts it to a float
        Serial.printf("Received %i from Adafruit.io feed FeedNameB \n",valueB);   //prints to screen
     }
  }

  //*Pump  
  if (valueB == 1){  //if the the soil is dry(less than value), pump water
    digitalWrite(PUMP, HIGH);                 //turns pump on
    Serial.printf("Pump is ON \n");
    delay(500);
    digitalWrite(PUMP, LOW);                  //turns pump off
    Serial.printf("Pump is OFF \n");
    delay(500);
  }
  if ((millis()-lastTime2)>(60000*30)) {   //
    if (moistureValues > 2300 ){  //if the the soil is dry(less than value), pump water
      digitalWrite(PUMP, HIGH);                 //turns pump on
      Serial.printf("Pump is ON \n");
      delay(500);
      digitalWrite(PUMP, LOW);                  //turns pump off
      Serial.printf("Pump is OFF \n");
      delay(500);
    }
    lastTime2 = millis();
  }
    //*Dust
    duration = pulseIn(DUSTSENSOR, LOW);
    lowpulseoccupancy = lowpulseoccupancy+duration;
    if ((millis()-starttime) > sampletime_ms)//if the sampel time == 30s
    {
      ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
      concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
      Serial.println(concentration);
      lowpulseoccupancy = 0;
      starttime = millis();
    }
  
  showDisplayValues(); //function to print sensor values to OLED
  
  //////////////////////////////////////////
  //*Reads and Publishes Values to Adafruit 
  if((millis()-lastTime > 20000)) {
  
    //*Air Quality Check Function
    int quality = sensor.slope();
    Serial.print("Sensor value: ");
    Serial.println(sensor.getValue());  
    //Checks how fresh the air is
    if (quality == AirQualitySensor::FORCE_SIGNAL) {
      Serial.println("High pollution! Force signal active.");
    }
    else if (quality == AirQualitySensor::HIGH_POLLUTION) {
      Serial.println("High pollution!");
    }
    else if (quality == AirQualitySensor::LOW_POLLUTION) {
      Serial.println("Low pollution!");
    }
    else if (quality == AirQualitySensor::FRESH_AIR) {
      Serial.println("Fresh air.");
    }
    delay(500);

    //*Moisture
    moistureValues = analogRead(MOISTURESENSOR);         //gets the values from the moisture sensor

    //*BME
    temp = (bme.readTemperature()*1.8)+32;  //gets the temp values  and converts them to F
    press = bme.readPressure()*0.00030;     //converts the press values to inches of murcury
    rHumidity = bme.readHumidity();         //gets RH values from the BME sensor

    if(mqtt.Update()) {  //starts MQTT updats
      mqtttemp.publish(temp);                                      //publishes the temp values  Adafruit
      Serial.printf("Publishing Temp %0.2f \n", temp);             //prints temp values to serial monitor
      mqttpress.publish(press);                                    //pub the pressure values
      Serial.printf("Publishing Pressure %0.2f \n", press);        //prints pressure to serial monitor
      mqttrHumidity.publish(rHumidity);                            //publishes rHumidity values
      Serial.printf("Publishing RH %0.2f \n", rHumidity);          //prints the RH values
      mqttmoisture.publish(moistureValues);                        //publishes the moisture values
      Serial.printf("Publishing Moisture %i \n", moistureValues);  //prints the moisture values
      mqttAQ.publish(sensor.getValue());                           //gets the Air Quality values
      Serial.printf("Publishing AQ %i \n", sensor.getValue());     //publishes the AQ Values
      mqttdust.publish(concentration);                             //gets the Dust
      Serial.printf("Publishing Dust Values %0.2f \n", concentration);
      mqttdust.publish(DateTime.c_str());                             //gets the Dust
      //Serial.printf(" Date and time is %s\n", DateTime.c_str());
    }
   lastTime = millis();
  }
}

//}

/////////////////////////////////
//-----------------Void Functions
// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect() {  //this function is important to include for connecting to MQTT
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
      Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
      Serial.printf("Retrying MQTT connection in 5 seconds..\n");
      mqtt.disconnect();
      delay(5000);  // wait 5 seconds
  }
  Serial.printf("MQTT Connected!\n");   //output for if connection was successful
}

//*Prints values to the OLED
void showDisplayValues(){
  display.setCursor(0,0);
  display.clearDisplay();                             //clears the previous values
  display.printf("Temp: %0.1f\n", temp);              //shows the temp values
  display.printf("Pressure: %0.1f\n", press);         //shows the pressure values
  display.printf("Humidity:  %0.1f\n", rHumidity);    //shows the rel humidity values
  display.printf("AQ: %i \n", sensor.getValue());     //gets the air quality
  display.printf("Moisture: %i \n", moistureValues);  //gets the moisture values
  display.printf("Dust: %0.2f \n", concentration);    //gets the dust values
  display.printf("Time is %s\n", TimeOnly.c_str());   //gets time
  display.display();
}