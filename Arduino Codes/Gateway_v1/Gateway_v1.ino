/**
 * Forest Guard - Gateway (GA) — Arduino UNO R4 WiFi + ILI9488 TFT
 * -------------------------------------------------------------------------------------------------
 * Responsibilities:
 *  - Parse LoRa frames strictly between '#' and '*' (ignore garbage)
 *  - Handle node registration (#<NODE_ID>*  ->  reply #<NODE_ID>+OK*)
 *  - Show live ENV/LOC and events on TFT (DFRobot style UI)
 *  - Keep local NTP time; only push to Firebase when NTP epoch >= Jan 1, 2025
 *  - Push ENV and LOC only when changed (ENV delta > +/-1.0; LOC changed by ~> 11m)
 *  - On event (#F+id... / #G+id...):
 *      1) set Firebase /nodes/<id>/meta/Event = true
 *      2) non-blocking buzzer while true
 *      3) when false (you clear in dashboard), broadcast #<NODE_ID>+C* to clear NA
 *      4) de-duplicate events (id), and re-ACK CLEAR if NA keeps repeating same id
 */

#include "DFRobot_UI.h"
#include "Arduino.h"
#include "DFRobot_GDL.h"
#include "DFRobot_Touch.h"

// --- Fix for min/max macro clash with <algorithm> on UNO R4 toolchain ---
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#include <WiFiS3.h>
#include <ArduinoHttpClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <math.h>

// ---- Fill these ----
const char* WIFI_SSID = "Makerbrains_2.4G";
const char* WIFI_PASS = "Balaji2830";

// Firebase RTDB host WITHOUT "https://", and DB secret / OAuth token
const char* FB_HOST = "forest-guard-532f5-default-rtdb.asia-southeast1.firebasedatabase.app"; // e.g. "myproj.firebaseio.com"
const char* FB_AUTH = "jrwAbSXtRRO950ZjTTYXhl2Hg9i8Sd3AW20VKaPW";

WiFiSSLClient fbClient;
HttpClient     fbHttp(fbClient, FB_HOST, 443);

WiFiUDP ntpUDP;
NTPClient ntp(ntpUDP, "pool.ntp.org", 0 /*UTC*/, 10 * 1000);

// NTP is considered synced only after this epoch (2025-01-01 UTC)
const unsigned long EPOCH_2025_01_01 = 1735689600UL;

//-------------------------------------
// TFT + Touch
//-------------------------------------
#define TFT_DC  8
#define TFT_CS  10
#define TFT_RST 9

DFRobot_Touch_GT911 touch;
DFRobot_ILI9488_320x480_HW_SPI screen(/*dc=*/TFT_DC,/*cs=*/TFT_CS,/*rst=*/TFT_RST);
DFRobot_UI ui(&screen, &touch);

//-------------------------------------
// LoRa UART
//-------------------------------------
#define LORA_RX_PIN 2
#define LORA_TX_PIN 3
//#define USE_HWSERIAL   // Prefer enabling this (Serial1 on D0/D1) if you can

#ifdef USE_HWSERIAL
  #define LORA_PORT Serial1
#else
  #include <SoftwareSerial.h>
  SoftwareSerial LORA_PORT(LORA_RX_PIN, LORA_TX_PIN);
#endif

//-------------------------------------
// Buzzer
//-------------------------------------
#define BUZZER_PIN 5

//-------------------------------------
// Colors (same style as your UI)
///////////////////////////////////////
#define COLOR_BG        0x0841
#define COLOR_CARD_BG   0x2124
#define COLOR_TITLE     0xFFFF
#define COLOR_VALUE     0x07FF
#define COLOR_UNIT      0x8410
#define COLOR_BORDER    0x39C7
#define COLOR_ALERT     0xF800
#define COLOR_TEMP      0xFD20
#define COLOR_HUMI      0x07E0
#define COLOR_UV        0x780F
#define COLOR_LIGHT     0x07FF
#define COLOR_PRESS     0xFFE0
#define COLOR_ALT       0xF81F
#define COLOR_OK        0x07E0
#define COLOR_WARN      0xFD20

//-------------------------------------
// UI layout
//-------------------------------------
#define CARD_WIDTH      140
#define CARD_HEIGHT     80
#define CARD_MARGIN     10
#define HEADER_HEIGHT   50

//-------------------------------------
// Data models
//-------------------------------------
struct EnvData {
  float t=0, h=0, uv=0, lux=0, p=0, alt=0;
  unsigned long lastUpdateMs = 0;
};
EnvData env;

String lastNodeId = "--";
bool   hasLoc = false;
float  lastLat = 0, lastLon = 0;
unsigned long lastLocUpdateMs = 0;

bool fireActive = false;
bool gunActive  = false;
String fireTimeFromNode = "NT";
String gunTimeFromNode  = "NT";
String gunScoreStr      = "";
int    currentEventId   = -1;     // de-dup
int    lastClearedEventId = -1;

//-------------------------------------
// Firebase helpers
//-------------------------------------
String fbPath(const String &path) {
  String url = "/";
  url += path;
  url += ".json?auth=";
  url += FB_AUTH;
  return url;
}

bool fbPUT(const String &path, const String &json) {
  int r = fbHttp.put(fbPath(path), "application/json", json);
  if (r == 0) {
    int code = fbHttp.responseStatusCode();
    fbHttp.stop();
    return code >= 200 && code < 300;
  }
  fbHttp.stop();
  return false;
}

bool fbPATCH(const String &path, const String &json) {
  int r = fbHttp.patch(fbPath(path), "application/json", json);
  if (r == 0) {
    int code = fbHttp.responseStatusCode();
    fbHttp.stop();
    return code >= 200 && code < 300;
  }
  fbHttp.stop();
  return false;
}

String fbGET(const String &path) {
  String body = "";
  int r = fbHttp.get(fbPath(path));
  if (r == 0) {
    int code = fbHttp.responseStatusCode();
    if (code == 200) body = fbHttp.responseBody();
  }
  fbHttp.stop();
  return body;
}

//-------------------------------------
// Utility: epoch formatting & parsing
//-------------------------------------
String epochToIso(unsigned long epoch) {
  time_t t = (time_t)epoch;
  struct tm *tmv = gmtime(&t);
  char buf[32];
  if (tmv) {
    snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             tmv->tm_year + 1900, tmv->tm_mon + 1, tmv->tm_mday,
             tmv->tm_hour, tmv->tm_min, tmv->tm_sec);
  } else {
    snprintf(buf, sizeof(buf), "1970-01-01T00:00:00Z");
  }
  return String(buf);
}

// days-from-civil (Howard Hinnant) — convert Y/M/D to days since epoch
static long daysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y-399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);           // [0, 399]
  const unsigned doy = (153*(m + (m > 2 ? -3 : 9)) + 2)/5 + d-1; // [0, 365]
  const unsigned doe = yoe * 365 + yoe/4 - yoe/100 + doy;  // [0, 146096]
  return era * 146097L + (long)doe - 719468L;              // days since 1970-01-01
}

static unsigned long ymdToEpochUTC(int y, int mon, int day, int hh, int mm, int ss) {
  long days = daysFromCivil(y, (unsigned)mon, (unsigned)day);
  if (days < 0) return 0;
  unsigned long e = (unsigned long)days * 86400UL + (unsigned long)hh * 3600UL + (unsigned long)mm * 60UL + (unsigned long)ss;
  return e;
}

//-------------------------------------
// Firebase write policies (change gates)
//-------------------------------------
const float ENV_DELTA_MIN = 1.0f;
const float LOC_DELTA_MIN = 0.00010f; // ~11 meters

EnvData lastPostedEnv;
bool    havePostedEnv = false;
bool    havePostedLoc = false;
float   lastPostedLat = 0, lastPostedLon = 0;

//-------------------------------------
// NTP gating
//-------------------------------------
bool ntpReady() { return ntp.getEpochTime() >= EPOCH_2025_01_01; }
unsigned long lastNtpUpdate = 0;

//-------------------------------------
// LoRa parsing helpers
//-------------------------------------
String inBuf;
bool readLoraFrame(String &out) {
  while (LORA_PORT.available()) {
    char c = (char)LORA_PORT.read();
    if (c == '#') {
      inBuf = "#";
    } else if (inBuf.length()) {
      inBuf += c;
      if (c == '*') {
        out = inBuf;
        inBuf = "";
        return true;
      }
      if (inBuf.length() > 160) inBuf.remove(0, 80); // keep buffer bounded
    }
  }
  return false;
}

void loraSend(const String &s) {
  LORA_PORT.println(s);
  Serial.print("[GA->LoRa] ");
  Serial.println(s);
}

//-------------------------------------
// UI helpers (kept same style)
//-------------------------------------
void drawRoundedRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, bool filled = true) {
  if (filled) {
    screen.fillRect(x+2, y, w-4, h, color);
    screen.fillRect(x, y+2, w, h-4, color);
  }
  screen.drawRect(x, y, w, h, color);
}

void drawSensorCard(int16_t x, int16_t y, const char* title, const char* value, const char* unit, uint16_t valueColor, unsigned long /*lastUpdate*/) {
  drawRoundedRect(x, y, CARD_WIDTH, CARD_HEIGHT, COLOR_CARD_BG);
  drawRoundedRect(x, y, CARD_WIDTH, CARD_HEIGHT, COLOR_BORDER, false);
  ui.drawString(x + 10, y + 10, (char*)title, COLOR_TITLE, COLOR_CARD_BG, 1, 0);
  ui.drawString(x + 10, y + 30, (char*)value, valueColor, COLOR_CARD_BG, 2, 0);
  ui.drawString(x + 10, y + 60, (char*)unit, COLOR_UNIT, COLOR_CARD_BG, 1, 0);
}

void drawHeader() {
  screen.fillRect(0, 0, 320, HEADER_HEIGHT, COLOR_BG);
  ui.drawString(15, 20, (char*)"Forest MONITOR", COLOR_TITLE, COLOR_BG, 2, 0);
  ui.drawString(250, 20, (char*)lastNodeId.c_str(), COLOR_VALUE, COLOR_BG, 2, 0);
}

//-------------------------------------
// Non-blocking buzzer
//-------------------------------------
bool buzzerOn = false;
unsigned long buzzerNextToggle = 0;
unsigned long buzzerPeriodMs = 500; // 2 Hz while event active

void buzzerTick(unsigned long now) {
  if (now >= buzzerNextToggle) {
    buzzerOn = !buzzerOn;
    digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);
    buzzerNextToggle = now + buzzerPeriodMs;
  }
}

//-------------------------------------
// CSV mini-parser
//-------------------------------------
bool parseCSV(const String &s, String out[], int max, int &count) {
  count = 0; int last = 0;
  for (int i=0; i<=s.length(); i++) {
    if (i == s.length() || s.charAt(i) == ',') {
      if (count < max) {
        out[count++] = s.substring(last, i); out[count-1].trim();
      }
      last = i+1;
      if (count >= max) break;
    }
  }
  return count > 0;
}

//-------------------------------------
// Firebase writers (using your schema)
//-------------------------------------
bool postEnvIfChanged() {
  if (!ntpReady()) return false;

  bool changed = !havePostedEnv
    || fabs(env.t   - lastPostedEnv.t)   >= ENV_DELTA_MIN
    || fabs(env.h   - lastPostedEnv.h)   >= ENV_DELTA_MIN
    || fabs(env.uv  - lastPostedEnv.uv)  >= ENV_DELTA_MIN
    || fabs(env.lux - lastPostedEnv.lux) >= ENV_DELTA_MIN
    || fabs(env.p   - lastPostedEnv.p)   >= ENV_DELTA_MIN
    || fabs(env.alt - lastPostedEnv.alt) >= ENV_DELTA_MIN;

  if (!changed) return false;

  unsigned long epoch = ntp.getEpochTime();

  // Your schema: nodes/<nodeId>/env/<epoch>  with keys temp,humi,uvi,li,pres,alt
  String json = String("{\"temp\":") + env.t +
                ",\"humi\":" + env.h +
                ",\"uvi\":"  + env.uv +
                ",\"li\":"   + env.lux +
                ",\"pres\":" + env.p +
                ",\"alt\":"  + env.alt + "}";

  fbPUT(String("nodes/") + lastNodeId + "/env/" + epoch, json);

  lastPostedEnv = env; havePostedEnv = true;

  // meta heartbeat: Event stays as-is, only update lastSeenAt
  fbPATCH(String("nodes/") + lastNodeId + "/meta",
          String("{\"lastSeenAt\":") + epoch + "}");

  return true;
}

bool postLocIfChanged() {
  if (!ntpReady() || !hasLoc) return false;

  bool changed = !havePostedLoc
    || fabs(lastLat - lastPostedLat) >= LOC_DELTA_MIN
    || fabs(lastLon - lastPostedLon) >= LOC_DELTA_MIN;

  if (!changed) return false;

  unsigned long epoch = ntp.getEpochTime();

  // Your schema: nodes/<nodeId>/Loc/<epoch>  with keys lat,lon (capital L in "Loc")
  String json = String("{\"lat\":") + String(lastLat, 6) +
                ",\"lon\":" + String(lastLon, 6) + "}";

  fbPUT(String("nodes/") + lastNodeId + "/Loc/" + epoch, json);

  lastPostedLat = lastLat; lastPostedLon = lastLon; havePostedLoc = true;
  return true;
}

void setFirebaseEventFlag(bool value) {
  if (!ntpReady()) return;
  // Your schema: meta.Event (capital E), and update lastSeenAt
  unsigned long epoch = ntp.getEpochTime();
  String json = String("{\"Event\":") + (value ? "true" : "false") +
                ",\"lastSeenAt\":" + epoch + "}";
  fbPATCH(String("nodes/") + lastNodeId + "/meta", json);
}

// Convert "YYYY/MM/DD" + "HH:MM:SS" to epoch (ms). If "NT", return 0.
unsigned long parseNodeTimeMs(const String &datePart, const String &timePart, bool &ok) {
  ok = false;
  if (datePart == "NT") return 0UL;

  // Expected: YYYY/MM/DD  and HH:MM:SS
  int y = 0, m = 0, d = 0, hh = 0, mm = 0, ss = 0;
  if (datePart.length() >= 10) {
    y = datePart.substring(0,4).toInt();
    m = datePart.substring(5,7).toInt();
    d = datePart.substring(8,10).toInt();
  }
  if (timePart.length() >= 8) {
    hh = timePart.substring(0,2).toInt();
    mm = timePart.substring(3,5).toInt();
    ss = timePart.substring(6,8).toInt();
  }
  if (y < 1970 || m < 1 || d < 1) return 0UL;

  unsigned long epoch = ymdToEpochUTC(y, m, d, hh, mm, ss);
  ok = (epoch > 0);
  return epoch * 1000UL; // store ms like your export
}

void logFireToFirebase(int id, const String &smoke, const String &timeFromNode) {
  if (!ntpReady()) return;

  // timeFromNode is "NT" OR "YYYY/MM/DD HH:MM:SS"
  bool ok = false;
  unsigned long nodeTimeMs = 0;
  int sp = timeFromNode.indexOf(' ');
  if (timeFromNode == "NT") {
    nodeTimeMs = 0;
  } else if (sp > 0) {
    String ds = timeFromNode.substring(0, sp);
    String ts = timeFromNode.substring(sp+1);
    nodeTimeMs = parseNodeTimeMs(ds, ts, ok);
    if (!ok) nodeTimeMs = 0;
  }

  unsigned long epoch = ntp.getEpochTime();
  // Your schema: nodes/<nodeId>/fire/<epoch> { value, NodeTime }
  String json = String("{\"value\":") + smoke +
                ",\"NodeTime\":" + nodeTimeMs + "}";
  fbPUT(String("nodes/") + lastNodeId + "/fire/" + epoch, json);
}

void logGunToFirebase(int id, const String &score, const String &timeFromNode) {
  if (!ntpReady()) return;

  bool ok = false;
  unsigned long nodeTimeMs = 0;
  int sp = timeFromNode.indexOf(' ');
  if (timeFromNode == "NT") {
    nodeTimeMs = 0;
  } else if (sp > 0) {
    String ds = timeFromNode.substring(0, sp);
    String ts = timeFromNode.substring(sp+1);
    nodeTimeMs = parseNodeTimeMs(ds, ts, ok);
    if (!ok) nodeTimeMs = 0;
  }

  unsigned long epoch = ntp.getEpochTime();
  // Your schema: nodes/<nodeId>/gun/<epoch> { score, NodeTime }
  String json = String("{\"score\":") + score +
                ",\"NodeTime\":" + nodeTimeMs + "}";
  fbPUT(String("nodes/") + lastNodeId + "/gun/" + epoch, json);
}

//-------------------------------------
// UI rendering
//-------------------------------------
unsigned long lastUiRefresh = 0;

void renderAll() {
  drawHeader();
  // ENV
  char t[16], h[16], uv[16], lx[16], pp[16], al[16];
  snprintf(t,  sizeof(t),  "%.1f", env.t);
  snprintf(h,  sizeof(h),  "%.1f", env.h);
  snprintf(uv, sizeof(uv), "%.2f", env.uv);
  snprintf(lx, sizeof(lx), "%.0f", env.lux);
  snprintf(pp, sizeof(pp), "%.1f", env.p);
  snprintf(al, sizeof(al), "%.1f", env.alt);
  drawSensorCard(15,   80, "TEMPERATURE", t,  "°C",  COLOR_TEMP,  env.lastUpdateMs);
  drawSensorCard(165,  80, "HUMIDITY",    h,  "%",   COLOR_HUMI,  env.lastUpdateMs);
  drawSensorCard(15,  170, "UV INDEX",    uv, "UV",  COLOR_UV,    env.lastUpdateMs);
  drawSensorCard(165, 170, "LIGHT",       lx, "Lux", COLOR_LIGHT, env.lastUpdateMs);
  drawSensorCard(15,  260, "PRESSURE",    pp, "hPa", COLOR_PRESS, env.lastUpdateMs);
  drawSensorCard(165, 260, "ALTITUDE",    al, "m",   COLOR_ALT,   env.lastUpdateMs);

  // Events
  drawSensorCard(15,  350, "Fire",     fireActive ? "Yes" : "No", fireTimeFromNode.c_str(), COLOR_ALERT, env.lastUpdateMs);
  drawSensorCard(165, 350, "Gun Shot", gunScoreStr.length() ? gunScoreStr.c_str() : (gunActive ? "Yes" : "No"),
                 gunTimeFromNode.c_str(), COLOR_ALERT, env.lastUpdateMs);

  // Location line
  char locLine[64];
  if (hasLoc) snprintf(locLine, sizeof(locLine), "Loc: %.5f, %.5f", lastLat, lastLon);
  else        snprintf(locLine, sizeof(locLine), "Loc: --");
  ui.drawString(15, 440, locLine, COLOR_VALUE, COLOR_BG, 2, 0);

  // Status
  const char *status = (WiFi.status() == WL_CONNECTED) ? (ntpReady() ? "NTP ONLINE      " : "NTP OFFLINE") : "WIFI DISCONNECTED";
  uint16_t sc = (WiFi.status() == WL_CONNECTED) ? (ntpReady() ? COLOR_OK : COLOR_WARN) : COLOR_ALERT;
  ui.drawString(15, 50, (char*)status, sc, COLOR_BG, 2, 0);
}

//-------------------------------------
// Registration handler
//-------------------------------------
void handleRegistrationOrClear(const String &body) {
  // body examples: "01" (registration)  |  "01+C" (clear ack back from us)
  if (body.length() && (lastNodeId == "--" || body == lastNodeId)) {
    // Registration from NA -> send ACK
    String ack = String("#") + body + "+OK*";
    loraSend(ack);
    Serial.println("✓ Registration request -> ACK sent");
  }
}

//-------------------------------------
// Setup & Loop
//-------------------------------------
void setup() {
  Serial.begin(115200);
#ifdef USE_HWSERIAL
  LORA_PORT.begin(115200);
#else
  LORA_PORT.begin(115200);
#endif

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  ui.begin();
  screen.fillScreen(COLOR_BG);
  drawHeader();

  // Wi-Fi
  Serial.print("Connecting Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(200);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) Serial.println(" OK");
  else Serial.println(" FAILED");

  // NTP
  ntp.begin();
}

unsigned long lastPollEvent = 0;

void loop() {
  unsigned long now = millis();

  // NTP maintenance (non-blocking)
  if (now - lastNtpUpdate >= 2000) {
    ntp.update();
    lastNtpUpdate = now;
  }

  // Read LoRa frames
  String frame;
  while (readLoraFrame(frame)) {
    int k1 = frame.indexOf('#'); int k2 = frame.lastIndexOf('*');
    if (k1 < 0 || k2 <= k1) continue;
    String body = frame.substring(k1 + 1, k2); body.trim();
    Serial.print("[LoRa] "); Serial.println(body);

    // Registration (#<nodeId>*)
    if (body.indexOf('+') < 0 && body.indexOf(',') < 0) {
      lastNodeId = body;
      handleRegistrationOrClear(body);
      drawHeader();
      continue;
    }

    // ENV  => E,<node>,t,h,uv,lux,p,alt
    if (body.startsWith("E,")) {
      String tok[8]; int n=0; parseCSV(body, tok, 8, n);
      if (n >= 8) {
        lastNodeId = tok[1];
        env.t   = tok[2].toFloat();
        env.h   = tok[3].toFloat();
        env.uv  = tok[4].toFloat();
        env.lux = tok[5].toFloat();
        env.p   = tok[6].toFloat();
        env.alt = tok[7].toFloat();
        env.lastUpdateMs = now;
        if (postEnvIfChanged()) Serial.println("↑ Firebase: ENV updated");
        lastUiRefresh = 0;
      }
      continue;
    }

    // LOC => L,<node>,lat,lon
    if (body.startsWith("L,")) {
      String tok[5]; int n=0; parseCSV(body, tok, 5, n);
      if (n >= 4) {
        lastNodeId = tok[1];
        lastLat = tok[2].toFloat();
        lastLon = tok[3].toFloat();
        hasLoc = true; lastLocUpdateMs = now;
        if (postLocIfChanged()) Serial.println("↑ Firebase: LOC updated");
        lastUiRefresh = 0;
      }
      continue;
    }

    // FIRE => F+id,<node>,smoke,YYYY/MM/DD,HH:MM:SS  or  F+id,<node>,smoke,NT
    if (body.startsWith("F+")) {
      String tok[6]; int n=0; parseCSV(body, tok, 6, n);
      if (n >= 4) {
        int id = tok[0].substring(2).toInt(); // after F+
        String node = tok[1];
        String smoke = tok[2];
        String timeFromNode = (n >= 5) ? (tok[3] + " " + tok[4]) : tok[3]; // NT or date+time

        if (node.length()) lastNodeId = node;

        if (id == lastClearedEventId) {
          // Node missed our CLEAR; re-send CLEAR and skip logging
          String clear = String("#") + node + "+C*"; loraSend(clear);
          Serial.println("Re-ACK CLEAR for stale FIRE event");
        } else if (id != currentEventId) {
          currentEventId = id;
          fireActive = true; gunActive = false;
          fireTimeFromNode = timeFromNode;
          setFirebaseEventFlag(true);
          logFireToFirebase(id, smoke, timeFromNode);
          Serial.print("🔥 FIRE event ");
        } else {
          // duplicate — ignore (we'll CLEAR once dashboard flips Event=false)
          Serial.println("FIRE duplicate (ignored, will re-ACK when cleared).");
        }
        lastUiRefresh = 0;
      }
      continue;
    }

    // GUN => G+id,<node>,score,YYYY/MM/DD,HH:MM:SS  or  G+id,<node>,score,NT
    if (body.startsWith("G+")) {
      String tok[6]; int n=0; parseCSV(body, tok, 6, n);
      if (n >= 4) {
        int id = tok[0].substring(2).toInt();
        String node = tok[1];
        String score = tok[2];
        String timeFromNode = (n >= 5) ? (tok[3] + " " + tok[4]) : tok[3];

        if (node.length()) lastNodeId = node;

        if (id == lastClearedEventId) {
          // Node missed our CLEAR; re-send CLEAR and skip logging
          String clear = String("#") + node + "+C*"; loraSend(clear);
          Serial.println("Re-ACK CLEAR for stale GUN event");
        } else if (id != currentEventId) {
          currentEventId = id;
          gunActive = true; fireActive = false;
          gunScoreStr = score; gunTimeFromNode = timeFromNode;
          setFirebaseEventFlag(true);
          logGunToFirebase(id, score, timeFromNode);
          Serial.print("🔫 GUN event");
        } else {
          Serial.println("GUN duplicate (ignored, will re-ACK when cleared).");
        }
        lastUiRefresh = 0;
      }
      continue;
    }
  }

  // When an event is active => beep and poll Firebase flag. If it becomes false, clear NA.
  if ((fireActive || gunActive) && (now - lastPollEvent >= 2000)) {
    lastPollEvent = now;
    if (ntpReady()) {
      // Your schema: meta/Event
      String body = fbGET(String("nodes/") + lastNodeId + "/meta/Event");
      body.trim();
      bool stillTrueInFB = (body.indexOf("true") >= 0);

      if (!stillTrueInFB) {
        // Send CLEAR to NA (multiple times to be safe)
        String clear = String("#") + lastNodeId + "+C*";
        loraSend(clear); delay(50); loraSend(clear); delay(50); loraSend(clear);

        lastClearedEventId = currentEventId;
        currentEventId = -1;
        fireActive = false; gunActive = false;
        gunScoreStr = ""; fireTimeFromNode = "NT"; gunTimeFromNode = "NT";

        digitalWrite(BUZZER_PIN, LOW); buzzerOn = false;
        Serial.println("✓ Firebase meta.Event=false -> CLEAR sent to NA");
      }
    }
  }

  // Buzzer when event is TRUE (non-blocking)
  if (fireActive || gunActive) buzzerTick(now);
  else { digitalWrite(BUZZER_PIN, LOW); buzzerOn = false; }

  // UI refresh (throttled)
  if (now - lastUiRefresh >= 200) { lastUiRefresh = now; renderAll(); }
}
