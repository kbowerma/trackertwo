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
 #include "bobcat.h"


void setup() {
    t.begin();
    t.gpsOn();
    // Opens up a Serial port so you can listen over USB
    Serial.begin(9600);
    // Serial1.begin(9600);  //NOTE:   it looks like I dont need this anymore

    // These three functions are useful for remote diagnostics. Read more below.
    Particle.function("tmode", transmitMode);
    Particle.function("gps", gpsPublish);
    Particle.function("move", fakeMove);
    Particle.variable("currLat", currLat);
    Particle.variable("currLon", currLon);
    Particle.variable("debugLevel", mydebug);
    Particle.variable("lastPublish", lastPublish);
    Particle.variable("pubdCounter", publishCounter);
    Particle.variable("pubdRate", pubRate );
    Particle.variable("gpsloctime", gpsloctime);
    Particle.variable("DIST_THRESHO", DIST_THRESHOLD);
    Particle.variable("distance", distance);
    Particle.variable("lstDistTime", lastDistanceTime);
    Particle.variable("speed", speed);

    String thisDeviceName = "bobcat_hunter";


    pinMode(D7, OUTPUT);
    digitalWrite(D7, HIGH);  //turn on built in led

    request.port = 80;
    request.hostname = "kb-dsp-server-dev.herokuapp.com";
    request.path = DSPPATH;

    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
   display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)

   display.clearDisplay();   // clears the screen and buffer

   display.setTextSize(2);           // from 1-9 sensible actually can be bigger but useless
   display.setTextColor(WHITE, BLACK); // 'normal' text
   display.setCursor(20,20);       // 128,64 pixels
   display.clearDisplay();
   display.println(MYBUILD);
   display.println(thisDeviceName);
   display.display();
   delay(4000);
}
void loop() {
    // You'll need to run this every loop to capture the GPS output
    t.updateGPS();

    currLat = t.readLatDeg();
    currLon = t.readLonDeg();

    //FIRST GPS LOC
    if(t.gpsFix() && gpsloctime == 0 ) {
      gpsloctime = millis()/1000;
      digitalWrite(D7, LOW);
    }

   // 2 minutes - BELT JOB
    //if the current time - the last time we published is greater than your set delay...
    if(millis()-lastPublish > delayMinutes*60*1000){
        lastPublish = millis();  // Remember when we published
        Serial << millis()/1000 << "  ";  //prints uptime in seconds
        Serial.println(t.preNMEA()); // Dumps the full NMEA sentence to serial in case you're curious
        // GPS requires a "fix" on the satellites to give good data,
          // so we should only publish data if there's a fix
        if(t.gpsFix()){ // Only publish if we're in transmittingData mode 1;
            if(transmittingData){
                gpsPublish("1");  // call the gpsPublish Function
            }
            digitalWrite(D7, LOW);   // turn off the led on the fix
        }
    }

    // 5 seconds debug logger
    if(millis()%(delaySeconds*1000) == 0  ) {
      if(mydebug < 2 ) {
         Serial << endl << MYBUILD << " " << millis()/1000 << "  GPS FIX TIME: " << gpsloctime  << "  " << t.readLatLon();
         Serial  << " pubdLat: " << String(pubdLat) << " pubdLon " <<  String(pubdLon) << " currLat: " << String(currLat) << " currLon " <<  String(currLon) << " distance ";
         Serial << String(distance) << " speed " << String(speed) << endl;
         Serial << "SSID: " << String(WiFi.SSID()) << endl;
         Serial << "HDOP: " << t.readHDOP() << "gpsTimestamp: " << t.getGpsTimestamp() << endl;
      }
     myoled();
     if(gpsloctime > 0 ) { // CHECK DISTANCE IF WE HAVE A LOC
       display << "dist " << checkDistance() << endl;
       display << MYBUILD << endl;
      }// only call
     display.display();
    }





   //debug the gps serial Note: this has to be turned on or it wont update the gps location
   while (Serial1.available() && gpsserialdebug ){
        Serial.print(char(Serial1.read()));

    }


  //------------- debug thoughts -------------------- //

  /*  dont do this debug thoughts now

  if (currLat != prevLat  ) {
    publishCounter = publishCounter+1;  //TODO Move this to actaul published
    Serial << "precheckdistance A" << String(prevLat) << "  " <<  String(prevLon) << " " << String(currLat) << "  " <<  String(currLon) << "  " << String(distance) << endl;
    checkDistance();
    Serial << endl << endl << "THERE IS A CHANGE in LAT,  deltaLon "  <<  currLat - prevLat << " distance: " << distance << endl;

    //delay(6000);
  }

  if (currLon != prevLon  ) {
    publishCounter = publishCounter+1;  //TODO Move this to actaul published
    Serial << "precheckdistance  B" << String(prevLat) << "  " <<  String(prevLon) << " " << String(currLat) << "  " <<  String(currLon) << "  " << String(distance) << endl;
    checkDistance();
    Serial << endl << endl << "THERE IS A CHANGE in LON, deltaLon "  <<  currLon - prevLon << " distance: " << distance << endl;
    //delay(6000);
  }

  */
  //------------- end debug thoughts -------------------- //

  if (currLat != pubdLat || currLon != pubdLon)  { // movement from last pubbed
    if ( checkDistance() > DIST_THRESHOLD ) {  //we have movment but we dont know if it is enough
        Serial << " We have movement d="  <<  distance << endl;
        if (currLat != pubdLat  ) {
          Serial << "currlat: " << currLat <<  "pubdLat: " << pubdLat  << endl;
        }
        if (currLon != pubdLon ) {
          Serial << "currLon: " << currLon <<  "pubdLon: " << pubdLon  << endl;
        }

      publishCounter = publishCounter+1;
      pubRate = publishCounter *60000 / millis();
      gpsPublish("1");
      pubdLat= currLat;
      pubdLon = currLon;
      myoled();

    }
  }

    //lastly set old to current as long as you get areading
    //if(currLat > 0 )
//    prevLat = currLat;
    //if(currLon > 0 )
//    prevLon = currLon;

}


// runs the oled functions
void myoled() {
  display.setTextSize(1);
  display.clearDisplay();
  display.setCursor(0,0);
 // display.clearDisplay();
  //display.print("SSID:");
   //display.setCursor(30,0);
  display << gpsloctime <<"  "  << (String(WiFi.SSID()))  << " " << t.readHDOP() << endl;
    int seconds =  ( millis()/1000 ) % 60;
    int minutes = ( millis()/(1000 * 60) ) % 60;
    int hours  = ( millis()/(1000 * 60 * 60) ) % 24;
   display << hours << ":" << minutes<< ":" << seconds << " pud " << publishCounter << endl ;
  // display << "GPS: "  << gpsloctime << endl;
   if (gpsloctime > 0 ) {
    //display.setCursor(0,25);
    display  <<  String(currLat)  << endl;
    display  << String(currLon) << endl;
  }

  display.display();
}
// Allows you to remotely change whether a device is publishing to the cloud
  // or is only reporting data over Serial. Saves data when using only Serial!
  // Change the default at the top of the code.
int transmitMode(String command){
    transmittingData = atoi(command);
    return 1;
}
// Actively ask for a GPS reading if you're impatient. Only publishes if there's
 // a GPS fix, otherwise returns '0'
int gpsPublish(String command){
    if(t.gpsFix()){

        //static float prevLat, prevLon; // Store previous values of latitude and longitude
        //float mydistance = checkDistance();


        // Serial << " calling checkDistance() distance: "  << mydistance << endl;

        // WE HAVE MOVEMENT
        if ( command == "1"  ) {
        request.body = generateRequestBody();
        http.put(request, response, headers);
        Serial << "Fnc call: http body: " << request.body << endl;

        Particle.publish("G", t.readLatLon(), 60, PRIVATE);
        Particle.publish("GLAT", String(t.readLatDeg()), 60, PRIVATE);
        Particle.publish("GLON", String(t.readLonDeg()), 60, PRIVATE);
        //Particle.publish("Dist", String(mydistance), 60, PRIVATE);

        }

        // uncomment next line if you want a manual publish to reset delay counter
        // lastPublish = millis();
        return 1;
    }
    else { return 0; }
}
//Function to assembly JSON body payload
int fakeMove(String command){
  int n = atof(command);
  prevLon = prevLon + n;
  return 1;

}
String generateRequestBody() {

     // A Dynamic Json buffer
     DynamicJsonBuffer jsonBuffer;
     JsonObject& obj = jsonBuffer.createObject();
     char buf[500];
     JsonVariant lat;
     lat.set(t.readLatDeg(), 6);
     obj["lat"] = lat;
     JsonVariant lng;
     lng.set(t.readLonDeg(), 6);
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
  if(currLat > 0 ) {
    // only check the distance is we have previous distance
    distance = getDistanceFromLatLong(pubdLat, pubdLon, currLat, currLon);
  }
  if(distance < DIST_THRESHOLD)  {
    Serial << endl << "DISTANCE over threshold   : " << distance;
    speed = distance / (millis()/1000 - lastDistanceTime);
    Serial << "   Speed: " <<  speed  << "m/s" << endl;
  }
  lastDistanceTime = millis()/1000;
  return distance;
}
