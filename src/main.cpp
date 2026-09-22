/*
  ESP SmartBeacon APRS-IS Client and Traccar
  v0.3  2023-10-2   增加 Traccar 上传功能
  v0.4  2023-10-10  修改 Traccar 上传方式与 APRS 相同
  v0.5  2023-10-14  增加 Web Configuration 功能
  v0.6  2026-09-19  Traccar 改为选配；positionReport 缓冲扩至 160B；
                    无 APRS 参数时加延时避免空转；GPS 失败改为重启；
                    配置读取前清空残留，避免空字段携带旧值
  v0.7  2026-09-21  固件更新改为串口上传：移除 ArduinoOTA（原无密码，存在
                    任意固件刷入风险）；移除无引用的 /dl 下载接口（原可用
                    arg(0) 读取任意 LittleFS 文件含明文凭据）
  v0.8  2026-09-22  APRS 配置页增加默认值：字段为空时预填推荐值，
                    保存时关键字段被清空也回落默认值，避免缺参数失效
  v0.9  2026-09-22  AP 热点自动管理：连上 WiFi 后自动关闭，断开超 30s 自动重开；
                    断线期间限频自动重连；首页显示热点状态
  v0.10 2026-09-22  修正 AP 关闭方式：v0.9 的 softAPdisconnect(false) 在 ESP8266 core
                    里只把 softap_config.ssid 置空（等于隐藏热点），AP 接口、DHCP
                    与射频仍在运行。改用 WiFi.enableAP(false) 从 opmode 摘掉 AP 位
                    真正关闭；并加 AP_OFF_COOLDOWN 冷却，避免切 opmode 抖动
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

/* 固件升级说明：
 * 自 v0.7 起移除 ArduinoOTA，固件一律通过 USB 串口本地烧录
 * （platformio.ini: upload_protocol = esptool）。
 * 不再开放任何网络侧的固件写入通道。
 */

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

// AP（配置热点）：上电开启供首次配置；连上 WiFi 后真正关闭 AP 接口；
// WiFi 断开超过 AP_REOPEN_DELAY 则自动重开，便于现场重新配置。
#define AP_SSID            "aprs-tracker"
#define AP_PASS            "88888888"
#define AP_REOPEN_DELAY    30000UL    // STA 断开多久后重开 AP（ms）
#define AP_OFF_COOLDOWN    60000UL    // 关闭 AP 后至少保持关闭这么久，给 STA 重连留时间（ms）
#define AP_OFF_CHECK_WINDOW 15000UL   // 关 AP 后观察 STA 是否掉线的窗口（ms）
#define AP_OFF_MAX_FAIL    2          // 关 AP 导致掉线累计多少次后放弃关闭
#define WIFI_RETRY_PERIOD  10000UL    // 断线时重连尝试间隔（ms）
char ver[] = "v0.10";

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

// ---------------------------------------------------------------------------
// APRS 参数默认值：Web 配置页在字段为空时预填以下推荐值，
// 用户可直接采用或改成自己的；保存时若关键字段被清空，
// 也会回落到这些值，避免因缺参数导致设备不发信标。
// ---------------------------------------------------------------------------
#define DEF_MYCALL     "BH9FXK-5"                          // 呼号（改成自己的）
#define DEF_APRSPASS   ""                                  // 不预填：必须用呼号对应的 passcode
#define DEF_COMMENT    "ESP APRS Tracker & Traccar."        // 信标注释
#define DEF_CUSTOMINFO "不负韶华 为梦想奋斗"                          // 自定义信息
#define DEF_APRSHOST   "asia.aprs2.net"                    // APRS-IS 服务器
#define DEF_SYMBOL     "/X"                                // APRS 符号（/X = 直升机）
#define DEF_LOWSPEED   "3"                                 // km/h
#define DEF_LOWRATE    "300"                               // s
#define DEF_HIGHSPEED  "60"                                // km/h
#define DEF_HIGHRATE   "60"                                // s
#define DEF_TURNMIN    "8"                                 // deg
#define DEF_TURNSLOPE  "255"
#define DEF_TURNTIME   "5"

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

// Traccar configuration (可选：TRACCARHOST 留空则不启用 Traccar 上报，仅使用 APRS-IS)
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

// AP / WiFi 状态（v0.9）
bool apEnabled = false;                  // 配置热点当前是否开启
unsigned long lastWifiOkMillis = 0;      // 最近一次 WiFi 正常的时刻
unsigned long lastWifiRetryMillis = 0;   // 最近一次重连尝试时刻
unsigned long lastApOffMillis = 0;       // 最近一次关闭 AP 的时刻（v0.10）
// 关 AP 的自适应退避（v0.10）：切 opmode 有把 STA 踢下线的风险，
// 若连续 AP_OFF_MAX_FAIL 次关完就掉线，则判定关闭不安全，之后保持热点常开。
bool apOffSafe = true;                   // 关 AP 是否已知不会导致 STA 掉线
bool apOffChecking = false;              // 是否处于"关 AP 后观察期"
unsigned long apOffCheckAt = 0;          // 观察期结束时刻
uint8_t apOffFailCount = 0;              // 关 AP 后随即掉线的累计次数

// -------------------------------------------------------------------------------
// APRS position with without timestamp (no APRS messaging)
//   !lati.xxN/long.xxEvCRS/SPD/comment
// -------------------------------------------------------------------------------
char* positionReportWithAltitude() {
  static char report [160];
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
  // 清空残留，避免空字段携带上一轮/旧文件残留值
  memset(mycall, 0, sizeof(mycall));
  memset(aprspass, 0, sizeof(aprspass));
  memset(comment, 0, sizeof(comment));
  memset(custominfo, 0, sizeof(custominfo));
  memset(aprshost, 0, sizeof(aprshost));
  memset(symbol_str, 0, sizeof(symbol_str));
  memset(low_speed_str, 0, sizeof(low_speed_str));
  memset(low_rate_str, 0, sizeof(low_rate_str));
  memset(high_speed_str, 0, sizeof(high_speed_str));
  memset(high_rate_str, 0, sizeof(high_rate_str));
  memset(turn_min_str, 0, sizeof(turn_min_str));
  memset(turn_slope_str, 0, sizeof(turn_slope_str));
  memset(turn_time_str, 0, sizeof(turn_time_str));
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
  memset(DEVICENUM, 0, sizeof(DEVICENUM));
  memset(TRACCARHOST, 0, sizeof(TRACCARHOST));
  memset(TRACCARPORT, 0, sizeof(TRACCARPORT));
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
  memset(ssid1, 0, sizeof(ssid1));
  memset(pass1, 0, sizeof(pass1));
  memset(ssid2, 0, sizeof(ssid2));
  memset(pass2, 0, sizeof(pass2));
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
  html.replace("###APSTATUS###", apEnabled ? (apOffSafe ? String("ON (192.168.4.1)")
                                                                   : String("ON (192.168.4.1, forced)"))
                                             : String("OFF"));

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
// 参数为空则回落到默认值（用于 Web 页预填与保存兜底）
static String valOr(const String &v, const char *def) {
  String s = v;
  s.trim();
  return s.length() ? s : String(def);
}
static String valOr(const char *v, const char *def) {
  return valOr(String(v), def);
}

// HTML 属性值转义：防止配置内容中的 " < > & 破坏表单渲染
// （只用于页面显示，保存时仍存原文）
static String valEsc(const char *v, const char *def) {
  String s = valOr(v, def);
  s.replace("&", "&amp;");
  s.replace("\"", "&quot;");
  s.replace("<", "&lt;");
  s.replace(">", "&gt;");
  return s;
}

void httpAPRS() {
  String html;
  String symtab;

  file = LittleFS.open("/aprs.html", "r");
  html = file.readString();
  file.close();

  // 空字段预填默认值，首次配置无需逐项手填
  html.replace("###MYCALL###", valEsc(mycall, DEF_MYCALL));
  html.replace("###APRSPASS###", valEsc(aprspass, DEF_APRSPASS));
  html.replace("###COMMENT###", valEsc(comment, DEF_COMMENT));
  html.replace("###CUSTOMINFO###", valEsc(custominfo, DEF_CUSTOMINFO));
  html.replace("###APRSHOST###", valEsc(aprshost, DEF_APRSHOST));
  html.replace("###SYMBOL###", valEsc(symbol_str, DEF_SYMBOL));
  html.replace("###LOWSPEED###", valEsc(low_speed_str, DEF_LOWSPEED));
  html.replace("###LOWRATE###", valEsc(low_rate_str, DEF_LOWRATE));
  html.replace("###HIGHSPEED###", valEsc(high_speed_str, DEF_HIGHSPEED));
  html.replace("###HIGHRATE###", valEsc(high_rate_str, DEF_HIGHRATE));
  html.replace("###TURNMIN###", valEsc(turn_min_str, DEF_TURNMIN));
  html.replace("###TURNSLOPE###", valEsc(turn_slope_str, DEF_TURNSLOPE));
  html.replace("###TURNTIME###", valEsc(turn_time_str, DEF_TURNTIME));

  server.send(200, "text/html; charset=UTF-8", html);
}

void httpSaveAPRS() {
  String html;

  file = LittleFS.open("/aprs.txt", "w");
  // 关键字段被清空时回落默认值，防止保存后设备因缺参数而失效
  file.println(valOr(server.arg("mycall"), DEF_MYCALL));
  file.println(valOr(server.arg("aprspass"), DEF_APRSPASS));
  file.println(valOr(server.arg("comment"), DEF_COMMENT));
  file.println(valOr(server.arg("custominfo"), DEF_CUSTOMINFO));
  file.println(valOr(server.arg("aprshost"), DEF_APRSHOST));
  file.println(valOr(server.arg("symbol"), DEF_SYMBOL));
  file.println(valOr(server.arg("low_speed"), DEF_LOWSPEED));
  file.println(valOr(server.arg("low_rate"), DEF_LOWRATE));
  file.println(valOr(server.arg("high_speed"), DEF_HIGHSPEED));
  file.println(valOr(server.arg("high_rate"), DEF_HIGHRATE));
  file.println(valOr(server.arg("turn_min"), DEF_TURNMIN));
  file.println(valOr(server.arg("turn_slope"), DEF_TURNSLOPE));
  file.println(valOr(server.arg("turn_time"), DEF_TURNTIME));
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

/* /dl 下载接口已于 v0.7 移除：
 * 原实现 LittleFS.open(server.arg(0)) 未做任何路径校验，攻击者可构造
 * GET /dl?/wifis.txt 或 /aprs.txt 直接下载明文 WiFi 密码与 APRS passcode，
 * 且该接口在所有前端页面中均无引用，属纯风险死代码。
 */

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

  server.onNotFound([]() {
    server.sendHeader("Refresh", "1;url=/");
    server.send(404, "text/plain", "QSD QSY");
  });
  server.begin();
  Serial.println("Started web server.");
}

/* ------------------------------------------------------------------------------- */
// AP（配置热点）管理
//   · 上电开启，供首次配置；
//   · 成功连上 WiFi 后真正关闭 AP 接口，减少暴露面，不必一直占着一个热点；
//   · WiFi 断开超过 AP_REOPEN_DELAY 自动重开，保证还能连上去改配置；
//
//   v0.10 修正：v0.9 用的 WiFi.softAPdisconnect(false) 在 ESP8266 core 中只是把
//   softap_config.ssid 置 0（见 ESP8266WiFiAP.cpp），AP 接口、DHCP 服务器与射频
//   都还在跑 —— 那只是"隐藏 SSID"，并不是关闭。SDK 头文件未导出 wifi_softap_stop()，
//   没有"只停 AP、不动 STA"的接口，所以真正关闭只能改 opmode：
//   WIFI_AP_STA -> WIFI_STA，即 WiFi.enableAP(false)。
//   代价是切换 opmode 会让 WiFi 子系统重启、STA 可能短暂掉线；SDK 会自动重连，
//   manageWifi() 每 WIFI_RETRY_PERIOD 也会补一次，AP_OFF_COOLDOWN 用于吸收这段抖动。
/* ------------------------------------------------------------------------------- */
static void apOn() {
  if (apEnabled) {
    return;
  }
  if (WiFi.softAP(AP_SSID, AP_PASS)) {
    apEnabled = true;
    Serial.print("AP ON  (SSID: ");
    Serial.print(AP_SSID);
    Serial.print(")  IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("AP ON failed!");
  }
}

static void apOff() {
  if (!apEnabled || !apOffSafe) {
    return;                        // 已判定关 AP 不安全时，热点保持常开
  }
  WiFi.enableAP(false);            // 从 opmode 摘掉 AP 位：真正关闭 AP 接口
  apEnabled = false;
  lastApOffMillis = millis();
  // 关 AP 需切 opmode，可能把 STA 踢下线；这次掉线是自己造成的，
  // 重置计时以免立刻触发"断线超时重开"，给它完整的重连窗口
  lastWifiOkMillis = millis();
  apOffChecking = true;            // 进入观察期，核对 STA 是否被带下线
  apOffCheckAt = millis() + AP_OFF_CHECK_WINDOW;
  Serial.println("AP OFF (AP interface disabled)");
}

static void manageWifi() {
  unsigned long now = millis();

  // 关 AP 观察期结束：窗口内 STA 仍在线 => 关闭安全；否则累计失败次数，
  // 达到 AP_OFF_MAX_FAIL 后放弃关闭，避免"关了就掉、掉了又连"反复闪断。
  if (apOffChecking && (long)(now - apOffCheckAt) >= 0) {
    apOffChecking = false;
    if (WiFi.status() == WL_CONNECTED) {
      apOffFailCount = 0;
    } else {
      apOffFailCount++;
      Serial.print("AP OFF dropped STA (fail #");
      Serial.print(apOffFailCount);
      Serial.println(")");
      if (apOffFailCount >= AP_OFF_MAX_FAIL) {
        apOffSafe = false;
        Serial.println("AP OFF is unsafe on this board; config AP stays on.");
      }
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    lastWifiOkMillis = now;
    apOff();                       // 已连上 WiFi，关闭配置热点
    return;
  }

  // WiFi 断开：需同时满足"断开超过 AP_REOPEN_DELAY"且"不在关 AP 冷却期"才重开。
  // 冷却期用来吸收切换 opmode 造成的短暂掉线，避免"关了又开"来回抖动。
  bool apOffCooldown = (lastApOffMillis != 0) && (now - lastApOffMillis < AP_OFF_COOLDOWN);
  if ((now - lastWifiOkMillis > AP_REOPEN_DELAY) && !apOffCooldown) {
    apOn();
  }

  // 限频尝试重连（WiFiMulti.run() 是阻塞调用，不宜每轮 loop 都调）
  if (now - lastWifiRetryMillis > WIFI_RETRY_PERIOD) {
    lastWifiRetryMillis = now;
    Serial.println("WiFi lost, retrying...");
    WiFiMulti.run();
  }
}

/* ------------------------------------------------------------------------------- */
void setup() {
  pinMode(PIN_D4, OUTPUT);

  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
  Serial.println(F("0 ESP SmartBeacon APRS-IS Tracker."));
  
  // Start AP（首次配置用；连上 WiFi 后自动关闭，断线超时自动重开，见 manageWifi()）
  WiFi.mode(WIFI_AP_STA);
  WiFi.setHostname("aprs-tracker");
  Serial.println("");
  Serial.println("Start AP...");
  apOn();
  
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
  lastWifiOkMillis = millis();              // v0.9: 记录连接时刻

  /* ------------------------------------------------------------------------------- */
  // v0.7: 已移除 ArduinoOTA。固件更新只能通过 USB 串口烧录。
  Serial.println("3 Ready! (firmware update: USB serial only)");
  Serial.print("IP address: ");
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

  manageWifi();             // v0.9: AP 开关 + 断线重连管理
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
    // APRS 为必配；Traccar 为选配（TRACCARHOST 为空时仅使用 APRS-IS）
    Serial.println(F("5 判断是否有 APRS 参数."));
    if (strlen(aprshost) == 0)
    {
      Serial.println(F("APRS 参数未配置，等待 Web 配置."));
      delay(1000);
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
      // Traccar上传失败，再次上传（仅当配置了 Traccar）
      if (post_now && strlen(TRACCARHOST) > 0) {
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

          // Traccar 上传（选配：未配置 Traccar 时跳过）
          if (strlen(TRACCARHOST) > 0) {
            traccarPOST();
          }
        }
      }
    }

    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println(F("ERROR: No GPS detected: check wiring. Restarting..."));
      ESP.restart();    // 明确重启，避免不可控看门狗复位
    }
    delay(1000);
  } else {
    digitalWrite(PIN_D4, LOW);    // 无网络，闪灯 interval led on as a heartbeat  
  }
  delay(1000);
}
