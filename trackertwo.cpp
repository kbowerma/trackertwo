/* -----------------------------------------------------------
* Kyle Bowerman 11.28.2016
* taken mostly from: https://github.com/spark/AssetTracker
* Wishlist

---------------------------------------------------------------*/

/*  These includes are setup for local compiling looking for a lib folder      *
* if you are compiling on the web IDE you sould comment out this whole         *
* include block and add:  AssetTracker, Steaming, and HTTPClient using the web *
* IDE
*  TODO:
*  [ ] make sitance threshold configurable via function
*  [ ] make move function take in full float
*  [ ] Display speed for 5 seconds
*  [ ] Change Previous location to a last sent location, that way I dont have to worry about micro moves

 */
#include "application.h"
 #include"lib/AssetTracker/firmware/AssetTracker.h"
 #include"lib/streaming/firmware/spark-streaming.h"
 #include "lib/HttpClient/firmware/HttpClient.h"
 #include "lib/SparkJson/firmware/SparkJson.h"
 #include "lib/Adafruit_SSD1306/Adafruit_SSD1306.h"
 #include "regal.h"


void setup() {
    t.begin();
    t.gpsOn();

    Serial.begin(9600);

    Particle.function("tmode", transmitMode);
    Particle.function("gps", gpsPublish);
    Particle.variable("currLat", currLat);
    Particle.variable("currLon", currLon);
    Particle.variable("pubdLon",startLon);
    Particle.variable("pubdLat",startLat);
    Particle.variable("startLat",startLat);
    Particle.variable("startLon",startLon);
    Particle.variable("debugLevel", mydebug);
    Particle.variable("lastPublish", lastPublish);
    Particle.variable("pubdCounter", publishCounter);
    Particle.variable("gpsloctime", gpsloctime);
    Particle.variable("minDelta", minDelta);
    Particle.variable("maxDelta", maxDelta);
    Particle.variable("distance", distance);
    Particle.variable("lstDistTime", lastDistanceTime);
    Particle.variable("stalness", staleness);
    Particle.variable("hdop", hdop);
    Particle.variable("ssid",ssid);

    String ssid = String(WiFi.SSID());

    Particle.variable("readyToPub", readyToPub);

    pinMode(D7, OUTPUT);
    digitalWrite(D7, HIGH);  //turn on built in led

    request.port = 80;
    request.hostname = "kb-dsp-server-dev.herokuapp.com";
    request.path = DSPPATH;

   display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
   display.clearDisplay();   // clears the screen and buffer
   display.setTextSize(2);           // from 1-9 sensible actually can be bigger but useless
   display.setTextColor(WHITE, BLACK); // 'normal' text
   display.setCursor(20,20);       // 128,64 pixels
   display.clearDisplay();
   display.println(MYBUILD);
   display.println(MYBUILD);
   display.display();
   delay(4000);
}
void loop() {
    // You'll need to run this every loop to capture the GPS output
  t.updateGPS();

  oldLat=currLat; //previous Lat
  oldLon=currLon; // previous Lon
  currLat = t.readLatDeg();
  currLon = t.readLonDeg();
  if ( oldLat==currLat && oldLon==currLon ) {
     staleness++; //stroke th stalness if nothing changes
   } else { staleness = 0; }
  hdop = t.readHDOP();

  //FIRST GPS LOC
  if(t.gpsFix() && gpsloctime == 0 ) {
    gpsloctime = millis()/1000;
    digitalWrite(D7, LOW);
  }

    //FIRST PUB Logic
  if (pubdLat == 0.0 && pubdLon == 0.0 && currLat != 0.0 && currLon ) {
    pubdLat = currLat;
    pubdLon = currLon;
    startLat = currLat;
    startLon = currLon;
    readyToPub = true;
  }

  if( !t.gpsFix() ) readyToPub = false;  // make note ready if we loose the gpsFix

  /*  2 second belt */
  if(millis()%(delaySeconds*1000) == 0  ) {
    if(mydebug < 2 ) {
       Serial << endl << MYBUILD << " " << millis()/1000 << "  GPS FIX TIME: " << gpsloctime  << "  " << t.readLatLon();
       Serial  << " pubdLat: " << String(pubdLat) << " pubdLon " <<  String(pubdLon) << " currLat: " << String(currLat) << " currLon " <<  String(currLon) << " distance ";
       //Serial << String(distance) << " speed " << String(speed) << endl;
       //Serial << "SSID: " << String(WiFi.SSID()) << endl;
       //Serial << "HDOP: " << t.readHDOP() << "gpsTimestamp: " << t.getGpsTimestamp() << endl;
    }
   myoled();
   if(gpsloctime > 0 ) { // CHECK DISTANCE IF WE HAVE A LOC
     display << "dist " << checkDistance() << endl;  // this check distance call will call the pbulish too
     display << MYBUILD << endl;
    }// only call
   display.display();
  }

   while (Serial1.available()  ){
        Serial.print(char(Serial1.read()));

    }
}

// *********** functions *****************
// runs the oled functions
void myoled() {
  display.setTextSize(1);
  display.clearDisplay();
  display.setCursor(0,0);
  display << gpsloctime <<"  "  << (String(WiFi.SSID()))  << " " << t.readHDOP() << endl;
    int seconds =  ( millis()/1000 ) % 60;
    int minutes = ( millis()/(1000 * 60) ) % 60;
    int hours  = ( millis()/(1000 * 60 * 60) ) % 24;
   display << hours << ":" << minutes<< ":" << seconds << "|" << publishCounter << "|" << staleness << endl ;

   if (gpsloctime > 0 ) {
    display  <<  String(currLat)  << endl;
    display  << String(currLon) << endl;
  }

  display.display();
}
 /* Allows you to remotely change whether a device is publishing to the cloud
   or is only reporting data over Serial. Saves data when using only Serial!
   Change the default at the top of the code.
 */
int transmitMode(String command){
    transmittingData = atoi(command);
    return 1;
}
 /* Actively ask for a GPS reading if you're impatient. Only publishes if there's
  a GPS fix, otherwise returns '0'
 */
int gpsPublish(String command){
    if(t.gpsFix()){
        if ( command == "1"  ) {
        request.body = generateRequestBody();
        http.put(request, response, headers);
        Serial << "Fnc call: http body: " << request.body << endl;
        }
        return 1;
    }
    else { return 0; }
}
//Function to assembly JSON body payload
String generateRequestBody() {

     // A Dynamic Json buffer
     DynamicJsonBuffer jsonBuffer;
     JsonObject& obj = jsonBuffer.createObject();
     char buf[500];
     JsonVariant lat;
     lat.set(currLat, 6);
     obj["lat"] = lat;
     JsonVariant lng;
     lng.set(currLon, 6);
     obj["lng"] = lng;
     obj.printTo(buf, sizeof(buf));
       return String(buf);

 }
// Convert degree to radians
float deg2rad(float deg) {
    return deg * (PI / 180);
}
// Get distance between two coordinates in kilometers
float getDistanceFromLatLong(float lat1, float lon1, float lat2, float lon2) {
    int R = 6371; // Radius of the earth in km
    float dLat = deg2rad(lat2 - lat1);
    float dLon = deg2rad(lon2 - lon1);
    float a = sin(dLat / 2) * sin(dLat / 2) +
        cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
        sin(dLon/2) * sin(dLon / 2);

    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float d = R * c;
    return d;
}
// checkDistance
float checkDistance() {
  if(currLat > 0 ) { // only check the distance is we have previous distance
    distance = getDistanceFromLatLong(pubdLat, pubdLon, currLat, currLon) * 1000 ;
  }
  if(distance > minDelta && distance < maxDelta)  {
    Serial << endl << "DISTANCE over threshold   : " << distance;
    speed = distance / (millis()/1000 - lastDistanceTime);
    Serial << "   Speed: " <<  speed  << "m/s" << endl;
    // BEGIN PUBLISH SEQUENCE
      gpsPublish("1");

      lastDistanceTime = millis()/1000;
      Particle.publish("DELTA", String("delta  " + String(distance) + " : " + String(currLat) + "," + String(currLat) +" | "+ String(pubdLat) + "," + String(pubdLat)), 60, PRIVATE);
      pubdLon = currLon;
      pubdLat = currLat;
      publishCounter++;
      delay(500);  // make a little hold down
    // END PUBLISH SEQUENCE

  }
  return distance;
}
