/*
  ESP SmartBeacon APRS-IS Client and Traccar
  v0.3  2023-10-2   增加 Traccar 上传功能
  v0.4  2023-10-10  修改 Traccar 上传方式与 APRS 相同
  v0.5  2023-10-14  增加 Web Configuration 功能
*/

#include <Arduino.h>

// GPS
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// NodeMCU ESP8266 Wifi Server
#include <ESP8266WiFiMulti.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>

// Filesystem
#include <LittleFS.h>

// OTA
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

/* ------------------------------------------------------------------------------- */
/* These are the pins for all ESP8266 boards */
//      Name   GPIO    Function     My function
#define PIN_D0  16  // WAKE         Onboard LED
#define PIN_D1   5  // User purpose I2C_SCL
#define PIN_D2   4  // User purpose I2C_SDA
#define PIN_D3   0  // (Low on boot means enter FLASH mode)
#define PIN_D4   2  // TXD1         Onboard LED (must be high on boot to go to UART0 FLASH mode)
#define PIN_D5  14  // HSCLK        
#define PIN_D6  12  // HMISO        GPS TX
#define PIN_D7  13  // HMOSI  RXD2  GPS RX
#define PIN_D8  15  // HCS    TXD2  (must be low on boot to enter UART0 FLASH mode)
#define PIN_D9   3  //        RXD0              (Same as USB Serial)
#define PIN_D10  1  //        TXD0              (Same as USB Serial)

#define PIN_MOSI 8  // SD1
#define PIN_MISO 7  // SD0
#define PIN_SCLK 6  // CLK
#define PIN_HWCS 0  // D3

#define PIN_D11  9  // SD2
#define PIN_D12 10  // SD4

/* ------------------------------------------------------------------------------- */
#define TOCALL "APEST1"
char ver[] = "v0.5";

// Use Serial port on IO12/IO13 for GPS
//static const int RXPin = PIN_D6, TXPin = PIN_D7;
static const int TXPin = PIN_D6, RXPin = PIN_D7;
static const uint32_t GPSBaud = 9600;

// Wifi configuration
char ssid1[32], pass1[32], ssid2[32], pass2[32];

// APRS configuration
// char mycall[] = "BH9FXK-5";                // Radioamateur callsign
// char aprspass[] = "20XXXX";                 // APRS-IS aprspass for callsign
// char comment[] = "ESP APRS Tracker & Traccar.";    // Comment string added to position
// char custominfo[] = "不负韶华 为梦想奋斗";
// char aprshost[] = "asia.aprs2.net";        // APRS-IS host
// char symbol_str[] = "/X";                  // APRS Symbol

char mycall[10];                // Radioamateur callsign
char aprspass[8];               // APRS-IS aprspass for callsign
char comment[32];               // Comment string added to position
char custominfo[32];            // Information
char aprshost[255];             // APRS-IS host
char symbol_str[8];             // APRS Symbol
uint16_t aprsport = 14580;      // Port is fixed to TCP/14580

// APRS SmartBeacon configuration
// int low_speed = 3;    //km/h
// long unsigned int low_rate = 300;   //s
// int high_speed = 60;  //km/h
// long unsigned int high_rate = 60;   //s
// long unsigned int turn_time = 5;    //s
// int turn_min = 8;     //degree
// int turn_slope = 255;  //degree

unsigned long lastBeaconMillis;
bool send_now = true;
int prev_heading;

char low_speed_str[8], low_rate_str[8], high_speed_str[8], high_rate_str[8];
char turn_time_str[8], turn_min_str[8], turn_slope_str[8];
int low_speed, high_speed, turn_min, turn_slope; 
long unsigned int low_rate, high_rate, turn_time;

// Traccar configuration
char DEVICENUM[32], TRACCARHOST[255], TRACCARPORT[8];
double CurrentLati, CurrentLogi;
String FINALLATI, FINALLOGI, FINALSPEED, FINALALTI, FINALCOURSE = "0";
bool post_now = false;

// The TinyGPS++ object
TinyGPSPlus gps;
TinyGPSCustom gpsFix(gps, "GPGSA", 2); // 1= No fix, 2=2D, 3=3D

// The serial connection to the GPS device
SoftwareSerial gpsSerial(RXPin, TXPin);

ESP8266WiFiMulti WiFiMulti;
WiFiClient wificlient;
HTTPClient httpclient;
ESP8266WebServer server(80);
File file;

// -------------------------------------------------------------------------------
// APRS position with without timestamp (no APRS messaging)
//   !lati.xxN/long.xxEvCRS/SPD/comment
// -------------------------------------------------------------------------------
char* positionReportWithAltitude() {
  static char report [64];
  memset (report, '\0' , sizeof(report));
  String symbolStr = String(symbol_str);

  if (gps.location.isValid()) {
    sprintf(report, "%s>%s,TCPIP*:!%02.0f%05.2f%s%c%03.0f%05.2f%s%c%03.0f/%03.0f/A=%06.0f",
            mycall, TOCALL,
            (float)gps.location.rawLat().deg, (float)gps.location.rawLat().billionths / 1000000000 * 60,
            (gps.location.rawLat().negative ? "S" : "N"), symbol_str[0],
            (float)gps.location.rawLng().deg, (float)gps.location.rawLng().billionths / 1000000000 * 60,
            (gps.location.rawLng().negative ? "W" : "E"), symbol_str[1],
            (float)gps.course.deg(), (float)gps.speed.knots(), (float)gps.altitude.feet());
  }
  return (report);
}

/* ------------------------------------------------------------------------------- */
// Function to read APRS configuration from file.
static void readCfgAPRS()
{
  if (LittleFS.exists("/aprs.txt")) {
    file = LittleFS.open("/aprs.txt", "r");
    file.readBytesUntil('\n', mycall, 10);
    if (mycall[strlen(mycall) - 1] == 13) {
      mycall[strlen(mycall) - 1] = 0;
    }

    file.readBytesUntil('\n', aprspass, 7);
    if (aprspass[strlen(aprspass) - 1] == 13) {
      aprspass[strlen(aprspass) - 1] = 0;
    }

    file.readBytesUntil('\n', comment, 32);
    if (comment[strlen(comment) - 1] == 13) {
      comment[strlen(comment) - 1] = 0;
    }

    file.readBytesUntil('\n', custominfo, 32);
    if (custominfo[strlen(custominfo) - 1] == 13) {
      custominfo[strlen(custominfo) - 1] = 0;
    }

    file.readBytesUntil('\n', aprshost, 255);
    if (aprshost[strlen(aprshost) - 1] == 13) {
      aprshost[strlen(aprshost) - 1] = 0;
    }

    file.readBytesUntil('\n', symbol_str, 8);
    if (symbol_str[strlen(symbol_str) - 1] == 13) {
      symbol_str[strlen(symbol_str) - 1] = 0;
    }

    file.readBytesUntil('\n', low_speed_str, 8);
    if (low_speed_str[strlen(low_speed_str) - 1] == 13) {
      low_speed_str[strlen(low_speed_str) - 1] = 0;
    }
    low_speed = atoi(low_speed_str);

    file.readBytesUntil('\n', low_rate_str, 8);
    if (low_rate_str[strlen(low_rate_str) - 1] == 13) {
      low_rate_str[strlen(low_rate_str) - 1] = 0;
    }
    low_rate = atoi(low_rate_str);

    file.readBytesUntil('\n', high_speed_str, 8);
    if (high_speed_str[strlen(high_speed_str) - 1] == 13) {
      high_speed_str[strlen(high_speed_str) - 1] = 0;
    }
    high_speed = atoi(high_speed_str);

    file.readBytesUntil('\n', high_rate_str, 8);
    if (high_rate_str[strlen(high_rate_str) - 1] == 13) {
      high_rate_str[strlen(high_rate_str) - 1] = 0;
    }
    high_rate = atoi(high_rate_str);

    file.readBytesUntil('\n', turn_min_str, 8);
    if (turn_min_str[strlen(turn_min_str) - 1] == 13) {
      turn_min_str[strlen(turn_min_str) - 1] = 0;
    }
    turn_min = atoi(turn_min_str);

    file.readBytesUntil('\n', turn_slope_str, 8);
    if (turn_slope_str[strlen(turn_slope_str) - 1] == 13) {
      turn_slope_str[strlen(turn_slope_str) - 1] = 0;
    }
    turn_slope = atoi(turn_slope_str);

    file.readBytesUntil('\n', turn_time_str, 8);
    if (turn_time_str[strlen(turn_time_str) - 1] == 13) {
      turn_time_str[strlen(turn_time_str) - 1] = 0;
    }
    turn_time = atoi(turn_time_str);

    file.close();
  }
}

/* ------------------------------------------------------------------------------- */
// Function to read TRACCAR configuration from file.
static void readCfgTRACCAR()
{
  if (LittleFS.exists("/traccar.txt")) {
    file = LittleFS.open("/traccar.txt", "r");
    file.readBytesUntil('\n', DEVICENUM, 32);
    if (DEVICENUM[strlen(DEVICENUM) - 1] == 13) {
      DEVICENUM[strlen(DEVICENUM) - 1] = 0;
    }

    file.readBytesUntil('\n', TRACCARHOST, 255);
    if (TRACCARHOST[strlen(TRACCARHOST) - 1] == 13) {
      TRACCARHOST[strlen(TRACCARHOST) - 1] = 0;
    }

    file.readBytesUntil('\n', TRACCARPORT, 8);
    if (TRACCARPORT[strlen(TRACCARPORT) - 1] == 13) {
      TRACCARPORT[strlen(TRACCARPORT) - 1] = 0;
    }
    file.close();
  }
}

/* ------------------------------------------------------------------------------- */
// Function to read WiFi configuration from file.
static void readCfgWiFi()
{
  if (LittleFS.exists("/wifis.txt")) {
    file = LittleFS.open("/wifis.txt", "r");
    file.readBytesUntil('\n', ssid1, 32);
    if (ssid1[strlen(ssid1) - 1] == 13) {
      ssid1[strlen(ssid1) - 1] = 0;
    }

    file.readBytesUntil('\n', pass1, 32);
    if (pass1[strlen(pass1) - 1] == 13) {
      pass1[strlen(pass1) - 1] = 0;
    }

    file.readBytesUntil('\n', ssid2, 32);
    if (ssid2[strlen(ssid2) - 1] == 13) {
      ssid2[strlen(ssid2) - 1] = 0;
    }

    file.readBytesUntil('\n', pass2, 32);
    if (pass2[strlen(pass2) - 1] == 13) {
      pass2[strlen(pass2) - 1] = 0;
    }
    file.close();
  }
}

/* ------------------------------------------------------------------------------- */
// This custom version of delay() ensures that the gps object is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}

/* ------------------------------------------------------------------------------- */
void httpRoot() {
  String html;

  file = LittleFS.open("/index.html", "r");
  html = file.readString();
  file.close();

  html.replace("###CURRSSID###", WiFi.SSID());
  html.replace("###CURRIP###", WiFi.localIP().toString());

  server.send(200, "text/html; charset=UTF-8", html);
}

void httpStyle() {
  String css;

  file = LittleFS.open("/style.css", "r");
  css = file.readString();
  file.close();
  server.send(200, "text/css", css);
}

/* ------------------------------------------------------------------------------- */
void httpAPRS() {
  String html;
  String symtab;

  file = LittleFS.open("/aprs.html", "r");
  html = file.readString();
  file.close();

  html.replace("###MYCALL###", String(mycall));
  html.replace("###APRSPASS###", String(aprspass));
  html.replace("###COMMENT###", String(comment));
  html.replace("###CUSTOMINFO###", String(custominfo));
  html.replace("###APRSHOST###", String(aprshost));
  html.replace("###SYMBOL###", String(symbol_str));
  html.replace("###LOWSPEED###", String(low_speed));
  html.replace("###LOWRATE###", String(low_rate));
  html.replace("###HIGHSPEED###", String(high_speed));
  html.replace("###HIGHRATE###", String(high_rate));
  html.replace("###TURNMIN###", String(turn_min));
  html.replace("###TURNSLOPE###", String(turn_slope));
  html.replace("###TURNTIME###", String(turn_time));

  server.send(200, "text/html; charset=UTF-8", html);
}

void httpSaveAPRS() {
  String html;

  file = LittleFS.open("/aprs.txt", "w");
  file.println(server.arg("mycall"));
  file.println(server.arg("aprspass"));
  file.println(server.arg("comment"));
  file.println(server.arg("custominfo"));
  file.println(server.arg("aprshost"));
  file.println(server.arg("symbol"));
  file.println(server.arg("low_speed"));
  file.println(server.arg("low_rate"));
  file.println(server.arg("high_speed"));
  file.println(server.arg("high_rate"));
  file.println(server.arg("turn_min"));
  file.println(server.arg("turn_slope"));
  file.println(server.arg("turn_time"));
  file.close();

  // reread config from file
  readCfgAPRS();

  file = LittleFS.open("/ok.html", "r");
  html = file.readString();
  file.close();

  server.sendHeader("Refresh", "3;url=/");
  server.send(200, "text/html; charset=UTF-8", html);
}

void httpTRACCAR() {
  String html;
  String symtab;

  file = LittleFS.open("/traccar.html", "r");
  html = file.readString();
  file.close();

  html.replace("###DEVICENUM###", String(DEVICENUM));
  html.replace("###TRACCARHOST###", String(TRACCARHOST));
  html.replace("###TRACCARPORT###", String(TRACCARPORT));

  server.send(200, "text/html; charset=UTF-8", html);
}

void httpSaveTRACCAR() {
  String html;

  file = LittleFS.open("/traccar.txt", "w");
  file.println(server.arg("DEVICENUM"));
  file.println(server.arg("TRACCARHOST"));
  file.println(server.arg("TRACCARPORT"));
  file.close();

  // reread config from file
  readCfgTRACCAR();

  file = LittleFS.open("/ok.html", "r");
  html = file.readString();
  file.close();

  server.sendHeader("Refresh", "3;url=/");
  server.send(200, "text/html; charset=UTF-8", html);
}

void httpWiFi() {
  String html;
  String symtab;

  file = LittleFS.open("/wifis.html", "r");
  html = file.readString();
  file.close();

  html.replace("###SSID1###", String(ssid1));
  html.replace("###PASS1###", String(pass1));
  html.replace("###SSID2###", String(ssid2));
  html.replace("###PASS2###", String(pass2));

  server.send(200, "text/html; charset=UTF-8", html);
}

void httpSaveWiFi() {
  String html;

  file = LittleFS.open("/wifis.txt", "w");
  file.println(server.arg("ssid1"));
  file.println(server.arg("pass1"));
  file.println(server.arg("ssid2"));
  file.println(server.arg("pass2"));
  file.close();

  // reread config from file
  readCfgWiFi();

  file = LittleFS.open("/ok.html", "r");
  html = file.readString();
  file.close();

  server.sendHeader("Refresh", "3;url=/");
  server.send(200, "text/html; charset=UTF-8", html);
}

/* ------------------------------------------------------------------------------- */
void httpBoot() {
  String html;

  file = LittleFS.open("/ok.html", "r");
  html = file.readString();
  file.close();

  server.sendHeader("Refresh", "3;url=about:blank");
  server.send(200, "text/html; charset=UTF-8", html);
  delay(1000);
  ESP.restart();
}

void httpDownload() {
  String str = "";
  file = LittleFS.open(server.arg(0), "r");
  if (!file) {
    Serial.println("Can't open LittleFS file !\r\n");
  }
  else {
    char buf[1024];
    int siz = file.size();
    while (siz > 0) {
      size_t len = std::min((int)(sizeof(buf) - 1), siz);
      file.read((uint8_t *)buf, len);
      buf[len] = 0;
      str += buf;
      siz -= sizeof(buf) - 1;
    }
    file.close();
    server.send(200, "text/plain", str);
  }
}

/* ------------------------------------------------------------------------------- */
void startWeberver() {
  server.on("/", httpRoot);
  server.on("/style.css", httpStyle);
  server.on("/aprs.html", httpAPRS);
  server.on("/saveaprs", httpSaveAPRS);
  server.on("/traccar.html", httpTRACCAR);
  server.on("/savetraccar", httpSaveTRACCAR);
  server.on("/wifis.html", httpWiFi);
  server.on("/savewifi", httpSaveWiFi);
  server.on("/boot", httpBoot);
  server.on("/dl", httpDownload);

  server.onNotFound([]() {
    server.sendHeader("Refresh", "1;url=/");
    server.send(404, "text/plain", "QSD QSY");
  });
  server.begin();
  Serial.println("Started web server.");
}

/* ------------------------------------------------------------------------------- */
void setup() {
  pinMode(PIN_D4, OUTPUT);

  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
  Serial.println(F("0 ESP SmartBeacon APRS-IS Tracker."));
  
  // Start AP
  WiFi.mode(WIFI_AP_STA);

  Serial.println("");
  Serial.println("Start AP...");

  WiFi.setHostname("aprs-tracker");
  WiFi.softAP("aprs-tracker", "wsmymmd!");
  
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  
  // Web Configuration
  if (!LittleFS.begin()) {
    Serial.println("Could not mount the filesystem...\n");
    delay(2000);
    ESP.restart();
  }
  startWeberver();
  readCfgAPRS();
  readCfgTRACCAR();

  readCfgWiFi();
  WiFiMulti.addAP(ssid1, pass1);
  WiFiMulti.addAP(ssid2, pass2);
  
  // Wait for connection
  Serial.print("1 Wait for WiFi... ");
  while(WiFiMulti.run() != WL_CONNECTED) {
    server.handleClient();    // Server handle client
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.print("2 Connected to WiFi: ");    // NodeMCU将通过串口监视器输出。
  Serial.println(WiFi.SSID());              // 连接的WiFI名称

  /* ------------------------------------------------------------------------------- */
  // OTA
  // ArduinoOTA.setPort(8266);    // Port defaults to 8266
  ArduinoOTA.setHostname("aprs-tracker");    // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setPassword("admin");    // No authentication by default
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3    // Password can be set with it's md5 value as well
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();

  Serial.println("3 OTA Ready!");
  Serial.print("OTA IP address: ");
  Serial.println(WiFi.localIP());
}

/* ------------------------------------------------------------------------------- */
// Traccar Post
void traccarPOST()
{
  // GPS Data for Traccar
  Serial.println("Traccar is running.");
  CurrentLati = gps.location.lat();
  CurrentLogi = gps.location.lng();
  String SerialData, SerialData1 = "";
  SerialData = String(CurrentLati, 6);
  SerialData1 = String(CurrentLogi, 6);
  FINALLATI = SerialData;
  FINALLOGI = SerialData1;
  FINALSPEED = gps.speed.kmph();
  FINALALTI = gps.altitude.meters();
  FINALCOURSE = gps.course.deg();
    
  // Traccar osmand 上传链接
  String traccarhost = TRACCARHOST;    // 变换类型
  String apiTraccar = "http://" + traccarhost + ":" + TRACCARPORT + "/?id=" + DEVICENUM +
    "&lat=" + FINALLATI + "&lon=" + FINALLOGI + "&altitude=" + FINALALTI +
    "&speed=" + FINALSPEED + "&heading=" + FINALCOURSE + "";

  httpclient.begin(wificlient, apiTraccar);    // POST to Traccar Server
  int httpCode = httpclient.GET();
  if (httpCode == 200) {
    Serial.println("OK: DATA SENT TO THE TRACCAR SERVER.");
    Serial.println(apiTraccar);
    post_now = false;
    // 上传成功闪灯5次
    int i = 0;
    while (i < 5)
    {
      digitalWrite(LED_BUILTIN, LOW);
      delay(150);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(150);
      i++;
    }

    } else {
    Serial.println("FAULT: DATA SENT TO THE TRACCAR SERVER.");
    post_now = true;
  }
}

/* ------------------------------------------------------------------------------- */
void loop() {

  ArduinoOTA.handle();      // OTA handle
  server.handleClient();    // Server handle client

  // APRS SmartBeacon
  int cur_speed, cur_heading, turn_threshold, heading_change_since_beacon = 0;
  long unsigned int beacon_rate = 0;
  unsigned long currentMillis = millis(), secs_since_beacon = (currentMillis - lastBeaconMillis) / 1000;
 
  // Connect to wifi, decode GPS and send APRS & TRACCAR packets.
  if (WiFiMulti.run() == WL_CONNECTED) {    // When connected to WiFi
    Serial.println(F("4 Wifi is OK."));
    digitalWrite(PIN_D4, HIGH);      // led off 当无线网络已连接
    smartDelay(1000);                // initial feeding of the GPS to make sure we have data
    // 如果参数文件中无数据，等待页面输入，停止后续代码执行
    Serial.println(F("5 判断是否有 APRS 或 Traccar 参数."));
    if (strlen(aprshost) == 0  || strlen(TRACCARHOST) == 0)
    {
      Serial.println(F("参数长度为0."));
      return;
    }
    
    // GPS 数据
    if ( atoi(gpsFix.value()) > 1 ) {
      Serial.println(F("6 GPS is working."));
      const char* report = positionReportWithAltitude();
      
      int satellitenumber = gps.satellites.value();    //satellites number
      Serial.print("Satellites in view: ");
      Serial.println(satellitenumber);

      /* ------------------------------------------------------------------------------- */      
      // Traccar上传失败，再次上传
      if (post_now) {
        Serial.println("R: TRACCAR 上传失败，再次上传");   
        traccarPOST();    // POST to Traccar Server
      }
      
      // APRS Report
      if (report[0] != '\0') {
        Serial.println("7 APRS is running.");
        
        // Position Report available, lets transmit to APRS-IS
        cur_speed = gps.speed.kmph();
        cur_heading = gps.course.deg();

        //
        // SmartBeacon
        //
        // Slow Speed = Speed below which I consider myself "stopped" 10 m.p.h.
        // Slow Rate = Beacon rate while speed below stopped threshold (1750s = ~29mins)
        // Fast Speed = Speed that I send out beacons at fast rate 100 m.p.h.
        // Fast Rate = Beacon rate at fastest interval (175s ~ 3 mins)
        // Any speed between these limits, the beacon rate is proportional.
        // Min Turn Time = don't beacon any faster than this interval in a turn (40sec)
        // Min Turn Angle = Minimum turn angle to consider beaconing. (20 degrees)
        // Turn Slope = Number when divided by current speed creates an angle that is added to Min Turn Angle to trigger a beacon.

        // Stopped - slow rate beacon
        if (cur_speed < low_speed) {
          beacon_rate = low_rate;
        } else {
          // Adjust beacon rate according to speed
          if (cur_speed > high_speed) {
            beacon_rate = high_rate;
          } else {
            beacon_rate = high_rate * high_speed / cur_speed;
            if (beacon_rate > low_rate) {
              beacon_rate = low_rate;
            }
            if (beacon_rate < high_rate) {
              beacon_rate = high_rate;
            }
          }

          // Corner pegging - ALWAYS occurs if not "stopped"
          // - turn threshold is speed-dependent
          turn_threshold = turn_min + turn_slope / cur_speed;
          if (prev_heading > cur_heading) {
            heading_change_since_beacon = ((prev_heading - cur_heading + 360) % 360);
          } else {
            heading_change_since_beacon = ((cur_heading - prev_heading + 360) % 360);
          }
          if ((heading_change_since_beacon > turn_threshold) && (secs_since_beacon > turn_time)) {
            send_now = true;
          }
        }

        // Send beacon if SmartBeacon interval (beacon_rate) is reached
        if (secs_since_beacon > beacon_rate || send_now) {
          lastBeaconMillis = currentMillis;
          // APRS-IS
          if (wificlient.connect(aprshost, aprsport)) {
            wificlient.printf("user %s pass %s\r\n", mycall, aprspass);
            smartDelay(100);
            wificlient.printf("%s%s %s\r\n", report, comment, ver);
            smartDelay(100);
            wificlient.printf("%s>%s,TCPIP*:>%s SATs: %d\r\n", mycall, TOCALL, custominfo, satellitenumber);
            wificlient.stop();
            Serial.printf("OK: %s\n", report);
            prev_heading = cur_heading;
            
            // 上传成功闪灯5次
            int i = 0;
            while (i < 5)
            {
              digitalWrite(LED_BUILTIN, LOW);
              delay(150);
              digitalWrite(LED_BUILTIN, HIGH);
              delay(150);
              i++;
            }
          } else {
            Serial.printf("Failed to connect to %s:%u as %s %s\n", aprshost, aprsport, mycall, aprspass);
          }
          send_now = false;

          traccarPOST();    // Traccar 上传
        }
      }
    }

    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println(F("ERROR: No GPS detected: check wiring."));
      while (true);
    }
    delay(1000);
  } else {
    digitalWrite(PIN_D4, LOW);    // 无网络，闪灯 interval led on as a heartbeat  
  }
  delay(1000);
}
