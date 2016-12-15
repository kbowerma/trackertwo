//defines
#define FILENAME "trackertwo.cpp"
#define MYVERSION "0.05.1214"
#define MYBUILD "oled"
#define PI 3.14159265

#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16



#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

//declarations
// Set whether you want the device to publish data to the internet by default here.
  // 1 will Particle.publish AND Serial.print, 0 will just Serial.print
  // Extremely useful for saving data while developing close enough to have a cable plugged in.
  // You can also change this remotely using the Particle.function "tmode" defined in setup()
int transmittingData = 1;
// Used to keep track of the last time we published data
long lastPublish = 0;
long publishCounter = 0;
// How many minutes minimum between publishes? 10+ recommended!
int delayMinutes = 2;
int delaySeconds = 3;
// Threshold to trigger a publish
// 9000 is VERY sensitive, 12000 will still detect small bumps
int accelThreshold = 12000;
unsigned long lastTime = 0;
bool gpsserialdebug = true;
int mydebug = 1;
  /* 1 - info
     2 - just distance
    */
int serial1Avail = 0;
int gpsloctime = 0;
int DIST_THRESHOLD = 0; // Distance threshold in km for publish gps data
double prevLat = 0.000000;
double prevLon = 0.000000;
double currLat = 0.000000;
double currLon = 0.000000;
double distance = 0.000000;
int lastDistanceTime = 0;
double speed = 0;


http_header_t headers[] = {
      { "Content-Type", "application/json" },
    { NULL, NULL } // NOTE: Always terminate headers will NULL
};
http_request_t request;
http_response_t response;

// Creating an AssetTracker named 't' for us to reference
AssetTracker t = AssetTracker();

 HttpClient http;

// A FuelGauge named 'fuel' for checking on the battery state
FuelGauge fuel;

// Prototypes
 int transmitMode(String command);
 int gpsPublish(String command);
 int fakeMove(String command);
 String generateRequestBody();
 float getDistanceFromLatLong(float lat1, float lon1, float lat2, float lon2);
 float checkDistance();
 void myoled();
