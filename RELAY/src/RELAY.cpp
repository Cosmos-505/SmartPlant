/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);
const int WATERPIN = (D16);



// setup() runs once, when the device is first turned on
void setup() {
  pinMode(WATERPIN, OUTPUT);
  // Put initialization like pinMode and begin functions here
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {

  digitalWrite(WATERPIN,HIGH);
  delay (2000);

  digitalWrite(WATERPIN,LOW);
  delay(10000);

  // The core of your code will likely live here.

  // Example: Publish event to cloud every 10 seconds. Uncomment the next 3 lines to try it!
  // Log.info("Sending Hello World to the cloud!");
  // Particle.publish("Hello world!");
  // delay( 10 * 1000 ); // milliseconds and blocking - see docs for more info!
}
