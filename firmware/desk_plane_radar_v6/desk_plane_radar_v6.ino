/*
  DESK PLANE RADAR v6  (ESP32-S3 N16R8, twin dial)
  ------------------------------------------------
  v6 port of v5.5.8 (firmware/archive/desk_plane_radar_v5_5.5.8.ino). Stage 1 (6.1.0): every v5 app,
  the Pomodoro / Focus Pro logic, NVS keys and web fields unchanged; only the hardware layer is new:
    - two GC9A01 on one SPI bus (CS by hand, left 41 / right 16), full-frame RGB565 canvases in PSRAM,
      brightness applied per pixel at flush (no backlight pin, no band palette any more)
    - left dial = the v5 app; right dial = companion screen (detail views come in stages 2 to 4)
    - two SSD1306 (both 0x3C) on two I2C buses: left = v5 info pages, right = companion
    - two encoders on PCNT: left turn = app, left click = home, left hold 2 s = menu,
      left double click = Pomodoro start / pause; right turn / click / hold = the v5 knob inside the app
    - two 24-LED rings: left = app position, right = seconds
  Bring-up record and pin changes: docs/V6_FIRMWARE_BRIEF.md, HANDOFF.md, firmware/archive/v6_bringup_6.0.21.ino.

  v5 description follows.
  ESP32 DESK PLANE RADAR  v5.2.1
  ------------------------------
  Based on "Desk Plane Radar - ESP32 with 2 screens" by Noah Thio (MakerWorld
  model 3090203). Same wiring as the original.

  APPS (hold the knob 2 s for the menu, rotate, click):
    RADAR      live ADS-B dial: arrivals / departures / overflights, trails,
               predicted paths, runway funnels, METAR wind + runway in use,
               TAF and SIGMET line, destination tags and destination weather,
               type-based icons, landmarks, follow mode, auto range
    ARRIVALS   airport board: next arrivals by ETA, departures page, for the
               home airport or any airport in the AIRPORTS table
    APPROACH   side view against the 3 degree glideslope, FINAL top-down page
    REPLAY     the last 48 minutes of tracks, knob scrubs time
    CLOCK      10 faces incl. Year Dots with public holidays, alarm clock
    WEATHER    animated icon, now / 3-day / 24 h history, pollutants, monsoon
               rain totals, rain-soon, sunset and golden-hour alerts, quakes
    POMODORO   presets + custom + meeting timer, task label, box breathing on
               breaks, session dots, 7-day history, stand/water nudges
    MARKETS    NIFTY, SENSEX, Bank Nifty, watchlist, USD/INR, gold (INR/10 g),
               BTC with fear & greed, heat map, mutual fund NAVs, portfolio
    NETWORK    latency to the Deco (gateway), internet and DNS, 10 min graph,
               loss, mesh node and roaming, outage log
    STATS      aircraft seen today by hour and class, records, spotter log,
               recent alerts
    NEWS       RSS headlines (also a ticker on the OLED)
    CRICKET    live scores (CricAPI key needed, off otherwise)
    F1         next race countdown and last podium (F1_ENABLED)
    SYSTEM     heap, fps, RSSI, uptime, sources, 24 h heap/fps health, reset
    DISPLAY    both brightnesses, colour theme, radar sweep on/off
    LOCATION   radar home, or the weather location, from the AIRPORTS table
               (66 airports: India, Gulf, SE Asia, Europe, US, Canada, Australia)

  ALERTS (OLED banner + round-display toast + buzzer melody): new traffic in
  the inner ring, emergency squawk 7700/7600/7500, heavy or military aircraft,
  flight watch list, go-around, SIGMET,
  rain soon, sunset / golden hour, market threshold, earthquake, nudges,
  hourly chime, timer, alarm. Each kind can be switched off on the web page;
  quiet hours mute the buzzer (alarm and emergency still sound).

  RAM: one 3.4 KB scratch arena is shared by the big display-only data sets
  (replay tracks, remote arrivals board, market chart + heat map + MF NAVs,
  network history, cricket). The app on screen owns it; alert logic for every
  other feature keeps running on small resident state, so a radar alert still
  pops up while the Pomodoro is running.

  WEB (http://deskradar.local): live mirror, settings, /live (both screens),
  /screen.bmp, /oled.bmp, /log, /config.json export, /import, /update.

  BOARD: ESP32 Dev Module.  PARTITION: Minimal SPIFFS (1.9MB APP with OTA).
  LIBRARIES: Adafruit GFX, Adafruit SSD1306, GFX Library for Arduino
  (moononournation), ArduinoJson 7, WiFiManager (tzapu).

  ENCODER (short click < 0.6 s, hold 0.6-2 s, menu > 2 s; fast turns step x4):
    RADAR     rotate zoom / step aircraft, click ZOOM-SELECT (or select the
              alerting aircraft), hold: filter (ZOOM) or follow (SELECT)
    ARRIVALS  rotate highlight, click arr/dep page, hold next airport
    APPROACH  rotate highlight, click select it on the radar, hold FINAL page
    REPLAY    rotate scrub, click play/pause, hold back to now
    CLOCK     rotate / click face, hold alarm on/off
    WEATHER   rotate pages, hold: pick the weather location
    POMODORO  rotate preset or minutes, click start/pause (break: breathing),
              hold stop
    MARKETS   rotate symbol / page, click refresh
    STATS, NEWS, SYSTEM, CRICKET  rotate pages / scroll
    DISPLAY   rotate adjust, click round/OLED/theme/sweep, hold back
    LOCATION  rotate list (fast turns jump), click set, hold radar/weather
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino_GFX_Library.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <lwip/sockets.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_heap_caps.h>
#include <ESP32Encoder.h>
#include <Adafruit_NeoPixel.h>
#include <driver/spi_master.h>
#include <driver/sdspi_host.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include "vfs_api.h"
#include <esp_core_dump.h>
#include <lwip/dns.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>

// forward declarations for Arduino's automatic prototypes
typedef void (*PaintFn)();   // v6: up here because the generated prototypes now land above the hardware block
void paintMessage(); const char *uiHint(); void uiFrame();
const char *navHint(int side, int what); bool navActive(); void drawNavOled(int side); extern unsigned long navMs;   // 6.14.0
bool dnsLookup(const char *host, IPAddress &ip, uint32_t waitMs = portMAX_DELAY); bool skyActive();   // 6.14.1
struct Plane; struct Weather; struct Quote; struct Net; struct Trail; struct RouteEntry; struct tm;
struct Airport; class PatientStream; struct LogoEntry; struct CalEvent; struct SetItem;
struct MacState { bool mic, cam, muted, playing; int vol; char np[64]; char app[12]; };   // v6 Mac helper state (defined below)
extern MacState mac; bool onAir(); void macQueue(const char *c); void bootStep(const char *what, const char *state);
extern volatile bool bootAnim; void bootTask(void *); void bootStop(); extern unsigned long bootStartMs;
extern const char *const CLKR_NAMES[]; extern int clkRight;   // boot sequence (6.17.0), defined after setup()
extern WiFiServer shotServer; void shotTask(void *);   // screenshot server on port 8080 (6.13.2)
void fxTick(int side, int dir); void fxPulse(int side); void fxWipe(uint16_t col); void fxBoot(); extern float fxHold;
#define MAX_CAL 12   // calendar events from the Mac helper
struct IssPass { bool valid; uint32_t startT, maxT; float maxEl, azMax; bool visible; };   // SKY (6.11.0), used by fetchTask above its definition
extern IssPass issPass; extern unsigned long issFetchMs; bool fetchIss();
int agendaList(int *idx, int maxN);
enum IconKind : uint8_t;
bool marketOpenNow(struct tm &t);
int  landingRunway(const Plane &p);
const char *wmoText(int c);
void drawInfo();
void drawMenu();
void beep(int f, int ms);
void logf(const char *fmt, ...);
void logln(const char *s);
void raiseAlert(const char *l1, const char *l2, uint16_t col, int melody, unsigned long ms, const char *icao);
void oledHead(const char *big, const char *r1, const char *r2); void oledTimeHead(const char *r1, const char *r2); void pgAddf(const char *fmt, ...);
bool pomoFocusActive(); const char *hname(); uint16_t todayKey(); void focusAddToday(uint16_t m); void putFocusPro(); void loadFocusPro();
int heatPage(); int mfPage(); void arenaAcquire(int group); void replayRecord(); bool noteInput(); void nodeStr(char *out, size_t n); void netResult(int code); int httpGet(HTTPClient &http, const char *tag); int locCount();
static const char *OLED_SCALE_NAMES[4] = {"1x", "1.5x", "2x", "3x"};
struct OledScale { const GFXfont *reg, *bold; uint8_t scale, lines; int8_t base[3]; };   // OLED body text size table entry (p6)
// OLED pager (p6): defaults live here because the Arduino prototype generator cannot carry them
void oledLineB(int row, const char *s, bool bold = false); void pgAdd(const char *s, bool bold = false); void pgWrap(const char *s, int maxLines, bool bold = false); int pgDraw(int extraFront = 0, int startRow = 0); int locAirport(int row); int locRowFor(int wxIdx);
String melodyNamesJson(); int replayMaxAge(); void applyPalette(); bool holidayForYday(int year, int yday, int &m, int &d); void nextHoliday(char *out, size_t n);

// =====================================================================
// USER CONFIG
// =====================================================================
#include "secrets.h"   // WiFi, OpenSky and focus log credentials (gitignored; copy secrets.example.h to secrets.h)
#if __has_include("config_local.h")
#include "config_local.h"   // 6.21.0: your place, name, time zone and panel wiring quirks (gitignored; copy config_local.example.h)
#endif

// Airports: the first entry is the default home (radar centre). LOCATION in
// the menu switches the home; ARRIVALS can show any of them.
struct Airport { const char *iata; const char *icao; const char *name; double lat, lon; int rwyHdg; const char *rwyLabel; int elevM; };
const Airport AIRPORTS[] = {
  {"BLR", "VOBL", "Bengaluru",     13.1989, 77.7056,  90, "09/27", 915},
  {"HAL", "VOBG", "HAL Bengaluru", 12.9500, 77.6682,  90, "09/27", 888},
  {"MAA", "VOMM", "Chennai",       12.9941, 80.1709,  70, "07/25",  16},
  {"HYD", "VOHS", "Hyderabad",     17.2403, 78.4294,  90, "09/27", 617},
  {"BOM", "VABB", "Mumbai",        19.0896, 72.8656, 270, "09/27",  11},
  {"DEL", "VIDP", "Delhi",         28.5562, 77.1000, 280, "10/28", 237},
  {"CCU", "VECC", "Kolkata",       22.6547, 88.4467,  10, "01/19",   5},
  {"COK", "VOCI", "Kochi",         10.1520, 76.4019,  90, "09/27",   9},
  {"GOI", "VOGO", "Goa Dabolim",   15.3808, 73.8314,  80, "08/26",  56},
  {"GOX", "VOGA", "Goa Mopa",      15.7440, 73.8600, 100, "10/28", 170},
  {"PNQ", "VAPO", "Pune",          18.5822, 73.9197, 100, "10/28", 592},
  {"AMD", "VAAH", "Ahmedabad",     23.0772, 72.6347,  50, "05/23",  58},
  {"JAI", "VIJP", "Jaipur",        26.8242, 75.8122,  90, "09/27", 385},
  {"LKO", "VILK", "Lucknow",       26.7606, 80.8893,  90, "09/27", 123},
  {"GAU", "VEGT", "Guwahati",      26.1061, 91.5859,  20, "02/20",  50},
  {"TRV", "VOTV", "Thiruvananthapuram", 8.4821, 76.9201, 140, "14/32", 5},
  {"CCJ", "VOCL", "Kozhikode",     11.1368, 75.9553, 100, "10/28", 104},
  {"IXE", "VOML", "Mangaluru",     12.9613, 74.8901,  60, "06/24", 103},
  {"IXM", "VOMD", "Madurai",        9.8345, 78.0934,  90, "09/27", 140},
  {"TRZ", "VOTR", "Tiruchirappalli", 10.7654, 78.7097, 90, "09/27", 88},
  {"CJB", "VOCB", "Coimbatore",    11.0300, 77.0434,  50, "05/23", 400},
  {"VTZ", "VOVZ", "Visakhapatnam", 17.7212, 83.2245, 100, "10/28",   4},
  {"BBI", "VEBS", "Bhubaneswar",   20.2444, 85.8178, 140, "14/32",  42},
  {"NAG", "VANP", "Nagpur",        21.0922, 79.0472, 140, "14/32", 315},
  {"IDR", "VAID", "Indore",        22.7218, 75.8011,  70, "07/25", 562},
  {"PAT", "VEPT", "Patna",         25.5913, 85.0880,  70, "07/25",  52},
  {"ATQ", "VIAR", "Amritsar",      31.7096, 74.7973, 160, "16/34", 230},
  {"IXC", "VICG", "Chandigarh",    30.6735, 76.7885, 110, "11/29", 318},
  {"HBX", "VOHB", "Hubballi",      15.3617, 75.0849,  80, "08/26", 661},
  {"MYQ", "VOMY", "Mysuru",        12.2300, 76.6558,  90, "09/27", 710},
  {"VGA", "VOBZ", "Vijayawada",    16.5304, 80.7968,  80, "08/26",  25},
  {"TIR", "VOTP", "Tirupati",      13.6325, 79.5433,  80, "08/26", 107},
  {"IXZ", "VOPB", "Port Blair",    11.6412, 92.7297,  40, "04/22",   4},
  {"SXR", "VISR", "Srinagar",      33.9871, 74.7742, 130, "13/31", 1655},
  {"IXL", "VILH", "Leh",           34.1359, 77.5465,  70, "07/25", 3256},
  {"UDR", "VAUD", "Udaipur",       24.6177, 73.8961,  80, "08/26", 514},
  {"BHO", "VABP", "Bhopal",        23.2875, 77.3374, 120, "12/30", 523},
  {"RPR", "VARP", "Raipur",        21.1804, 81.7388,  60, "06/24", 317},
  {"VNS", "VEBN", "Varanasi",      25.4524, 82.8593,  90, "09/27",  81},
  {"IXB", "VEBD", "Bagdogra",      26.6812, 88.3286,  90, "09/27", 126},
  // International (true runway headings, so US/EU designators differ from the heading by the local magnetic variation)
  {"DXB", "OMDB", "Dubai",         25.2528, 55.3644, 120, "12/30",  19},
  {"AUH", "OMAA", "Abu Dhabi",     24.4330, 54.6511, 133, "13/31",  27},
  {"DOH", "OTHH", "Doha",          25.2731, 51.6081, 164, "16/34",   4},
  {"SIN", "WSSS", "Singapore",      1.3644, 103.9915, 20, "02/20",   7},
  {"HKG", "VHHH", "Hong Kong",     22.3080, 113.9185, 71, "07/25",   9},
  {"NRT", "RJAA", "Tokyo Narita",  35.7647, 140.3864, 157, "16/34", 43},
  {"HND", "RJTT", "Tokyo Haneda",  35.5494, 139.7798, 157, "16/34", 11},
  {"SYD", "YSSY", "Sydney",       -33.9399, 151.1753, 167, "16/34",  6},
  {"LHR", "EGLL", "London Heathrow", 51.4700, -0.4543, 90, "09/27", 25},
  {"LGW", "EGKK", "London Gatwick",  51.1481, -0.1903, 78, "08/26", 62},
  {"MAN", "EGCC", "Manchester",    53.3537, -2.2750,  51, "05/23",  78},
  {"CDG", "LFPG", "Paris CDG",     49.0097,  2.5479,  86, "09/27", 119},
  {"FRA", "EDDF", "Frankfurt",     50.0379,  8.5622,  70, "07/25", 111},
  {"AMS", "EHAM", "Amsterdam",     52.3086,  4.7639, 183, "18/36",  -3},
  {"JFK", "KJFK", "New York JFK",  40.6413, -73.7781, 31, "04/22",   4},
  {"EWR", "KEWR", "Newark",        40.6895, -74.1745, 26, "04/22",   5},
  {"BOS", "KBOS", "Boston",        42.3656, -71.0096, 21, "04/22",   6},
  {"IAD", "KIAD", "Washington Dulles", 38.9531, -77.4565, 2, "01/19", 95},
  {"ATL", "KATL", "Atlanta",       33.6407, -84.4277, 84, "08/26", 313},
  {"MIA", "KMIA", "Miami",         25.7959, -80.2870, 81, "09/27",   3},
  {"ORD", "KORD", "Chicago O'Hare", 41.9742, -87.9073, 92, "10/28", 205},
  {"DFW", "KDFW", "Dallas Fort Worth", 32.8998, -97.0403, 178, "17/35", 185},
  {"SEA", "KSEA", "Seattle",       47.4502, -122.3088, 177, "16/34", 132},
  {"SFO", "KSFO", "San Francisco", 37.6213, -122.3790, 298, "10/28",  4},
  {"LAX", "KLAX", "Los Angeles",   33.9416, -118.4085, 261, "06/24", 38},
  {"YYZ", "CYYZ", "Toronto",       43.6777, -79.6248,  47, "05/23", 173},
};
#define NUM_AIRPORTS (int)(sizeof(AIRPORTS) / sizeof(AIRPORTS[0]))

// Flight watch: callsign prefix ("AIC504"), route substring ("DEL-", "-DXB") or type prefix ("B77", "A35").
const char *FLIGHT_WATCH[] = {"AIC504", "-DXB"};
#define NUM_WATCH (int)(sizeof(FLIGHT_WATCH) / sizeof(FLIGHT_WATCH[0]))

// Markets (Yahoo Finance, no key). Keep NIFTY first. Add NSE stocks as "TCS.NS" etc.
#define NUM_SYMBOLS 7
#define NUM_HEAT_CFG 12
const char *MARKET_SYMBOLS[NUM_SYMBOLS] = {"^NSEI",    "^BSESN", "^NSEBANK",   "RELIANCE.NS", "USDINR=X", "GC=F",  "BTC-USD"};
const char *MARKET_NAMES[NUM_SYMBOLS]   = {"NIFTY 50", "SENSEX", "BANK NIFTY", "RELIANCE",    "USD / INR", "GOLD",  "BITCOIN"};
const char *MARKET_SHORT[NUM_SYMBOLS]   = {"NIFTY",    "SENSEX", "BANKNFT",    "RELIANC",     "USDINR",   "GOLD",  "BTC"};
const float MARKET_ALERT_PCT[NUM_SYMBOLS] = {1.0, 1.0, 1.5, 2.5, 0.5, 2.0, 5.0};   // day move that triggers an alert, 0 = off
// Heat map page: fetched only while that page is open (price and previous close only)
const char *HEAT_SYMBOLS[NUM_HEAT_CFG] = {"RELIANCE.NS", "TCS.NS", "HDFCBANK.NS", "INFY.NS", "ICICIBANK.NS", "ITC.NS", "LT.NS", "SBIN.NS", "EURINR=X", "GBPINR=X", "AEDINR=X", "ETH-USD"};
const char *HEAT_NAMES[NUM_HEAT_CFG]   = {"RELIANCE", "TCS", "HDFC BK", "INFY", "ICICI", "ITC", "L&T", "SBI", "EUR", "GBP", "AED", "ETH"};
// Portfolio: symbols must also be in MARKET_SYMBOLS. qty 0 disables the page.
struct Holding { const char *symbol; float qty; };
const Holding PORTFOLIO[] = {{"RELIANCE.NS", 0}};
#define NUM_HOLDINGS (int)(sizeof(PORTFOLIO) / sizeof(PORTFOLIO[0]))
// Mutual funds: AMFI scheme codes (look yours up at https://api.mfapi.in/mf/search?q=name). Empty list disables the page.
const char *MF_SCHEMES[] = {"120503", "118989"};
#define NUM_MF (int)(sizeof(MF_SCHEMES) / sizeof(MF_SCHEMES[0]))

// News: RSS feeds (https). Up to 3.
const char *NEWS_FEEDS[] = {"https://feeds.bbci.co.uk/news/world/rss.xml", "https://www.thehindu.com/news/national/feeder/default.rss"};
#define NUM_FEEDS (int)(sizeof(NEWS_FEEDS) / sizeof(NEWS_FEEDS[0]))

// Cricket: free key from https://cricketdata.org (100 requests/day). Empty = app hidden.
#define CRICAPI_KEY ""
#define CRICKET_TEAM_FILTER "India"      // only matches involving this team; "" = all live matches

// Countdowns: name, year, month, day, hour, minute (local time)
struct Countdown { const char *name; int y, m, d, hh, mm; };
const Countdown COUNTDOWNS[] = {{"Diwali", 2026, 11, 8, 0, 0}, {"New Year", 2027, 1, 1, 0, 0}};
#define NUM_COUNTDOWNS (int)(sizeof(COUNTDOWNS) / sizeof(COUNTDOWNS[0]))

// Landmarks drawn on the dial (dim labels) when the range is 50 km or less
struct Landmark { const char *name; double lat, lon; };
const Landmark LANDMARKS[] = {{"Hebbal", 13.0358, 77.5970}, {"Whitefield", 12.9698, 77.7500}, {"E-City", 12.8399, 77.6770}, {"Devanahalli", 13.2437, 77.7120},
                              {"Yelahanka", 13.1007, 77.5963}, {"MG Road", 12.9757, 77.6063}, {"Nandi Hills", 13.3702, 77.6835}, {"Hoskote", 13.0707, 77.7980}};
#define NUM_LANDMARKS (int)(sizeof(LANDMARKS) / sizeof(LANDMARKS[0]))

// Public holidays (India + Karnataka). year 0 = every year; lunar dates are listed per year (approximate for 2027).
struct Holiday { uint16_t y; uint8_t m, d; const char *name; };
const Holiday HOLIDAYS[] = {
  {0, 1, 14, "Makar Sankranti"}, {0, 1, 26, "Republic Day"}, {0, 5, 1, "May Day"}, {0, 8, 15, "Independence Day"}, {0, 10, 2, "Gandhi Jayanti"}, {0, 11, 1, "Kannada Rajyotsava"}, {0, 12, 25, "Christmas"},
  {2026, 3, 3, "Holi"}, {2026, 3, 19, "Ugadi"}, {2026, 3, 20, "Eid ul-Fitr"}, {2026, 4, 3, "Good Friday"}, {2026, 5, 27, "Bakrid"}, {2026, 9, 14, "Ganesh Chaturthi"}, {2026, 10, 20, "Dussehra"}, {2026, 11, 8, "Diwali"},
  {2027, 3, 22, "Holi"}, {2027, 4, 7, "Ugadi"}, {2027, 3, 10, "Eid ul-Fitr"}, {2027, 3, 26, "Good Friday"}, {2027, 5, 17, "Bakrid"}, {2027, 9, 4, "Ganesh Chaturthi"}, {2027, 10, 9, "Dussehra"}, {2027, 10, 29, "Diwali"},
};
#define NUM_HOLIDAYS (int)(sizeof(HOLIDAYS) / sizeof(HOLIDAYS[0]))

// F1 (jolpi.ca Ergast mirror, no key). 0 hides the app.
#define F1_ENABLED 0   // 6.16.0: app retired
// REPLAY app (last 48 min of tracks). 0 hides it and stops the recording.
#define REPLAY_ENABLED 0

// World clock zones (POSIX TZ strings)
struct Zone { const char *name; const char *tz; };
const Zone ZONES[] = {{"SAN FRANCISCO", "PST8PDT,M3.2.0,M11.1.0"}, {"LONDON", "GMT0BST,M3.5.0/1,M10.5.0"}};
#define NUM_ZONES (int)(sizeof(ZONES) / sizeof(ZONES[0]))
// Defaults below are overridden by config_local.h (6.21.0)
#ifndef LOCAL_TZ
#define LOCAL_TZ "IST-5:30"
#endif
#ifndef LOCAL_TZ_IANA
#define LOCAL_TZ_IANA "Asia/Kolkata"       // Open-Meteo hours are requested in this zone, so forecasts line up with the clock wherever the weather location is
#endif
#ifndef LOCAL_CITY
#define LOCAL_CITY "BENGALURU"
#endif
// Weather "Home" entry (first row of the weather picker): your house. Editable on the web page, stored in NVS.
#ifndef WX_HOME_NAME
#define WX_HOME_NAME "Home"
#endif
#ifndef WX_HOME_LAT
#define WX_HOME_LAT 13.1989               // defaults to the first airport in the table; set yours in config_local.h
#endif
#ifndef WX_HOME_LON
#define WX_HOME_LON 77.7056
#endif

// ADS-B
#define DEFAULT_DATA_SOURCE 2            // adsb.lol
#define NUM_SOURCES 5                    // 0 OpenSky, 1 airplanes.live, 2 adsb.lol, 3 adsb.fi, 4 local readsb; 5 = auto survey
#define SURVEY_INTERVAL_MS 1800000
// Focus log: each Pomodoro focus session (completed or stopped early) is sent to a Google Apps Script web app
// (focus_log.gs) that writes a row to a Google Sheet and an event to a "Focus" calendar. Leave the URL empty to turn it off.
#ifndef USER_NAME_DEFAULT
#define USER_NAME_DEFAULT ""         // used in greetings; set it in config_local.h or on the web page (Pomodoro > Your name)
#endif
const unsigned long SRC_MIN_MS[NUM_SOURCES] = {30000, 10000, 15000, 10000, 2000};

#define UNITS_IMPERIAL 0
#define WEATHER_INTERVAL_MS 600000       // 10 min
#define METAR_INTERVAL_MS   1800000      // 30 min
#define NEWS_INTERVAL_MS    1200000      // 20 min
#define QUAKE_INTERVAL_MS   3600000      // 1 h
#define MF_INTERVAL_MS      21600000UL   // 6 h
#define CRICKET_INTERVAL_MS 1200000      // 20 min, 09:00-24:00 only
#define MARKET_POLL_OPEN_MS 60000        // per symbol while NSE is open and the app is showing
#define MARKET_POLL_IDLE_MS 600000
#define NET_PROBE_MS        5000
#define NET_PROBE_ENABLED   0            // 0 = no gateway/internet/DNS probes at all (NETWORK app shows x); test setting while chasing connection failures
#define TZ_OFFSET_SEC  19800
#define NIGHT_START_H  22
#define NIGHT_END_H    7
#define BUZZER_PIN     7                 // v6 piezo (phase 2: I2S DIN of the amp). -1 to disable
#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME   "deskradar6"      // mDNS name: http://deskradar6.local
#endif
#define FW_VERSION     "6.21.0"
#define SUMMARY_HOUR   21                // daily summary toast at this hour (-1 = off)

// Rendering
#define TFT_SPI_HZ          40000000     // 6.6.3: the GC9A01 is rated ~62 MHz; 80 MHz was fine in short bursts (Arduino_GFX) but with continuous DMA frames the panel dropped a byte about once in 8 frames and the whole frame's colours rotated (owner video 19:02; 6.6.2 CS setup clocks did not help). 40 MHz: ~24 ms per frame
#define TFT_ROTATION        2            // both round displays are mounted tab up
#define SWEEP_DEG_PER_FRAME 3.0f
#define SWEEP_TRAIL_LINES   10
#define DEAD_RECKON         1
#define TRAIL_LEN           5
#define NUM_FACES           17           // Bold, Analog, Dashboard, World, 24 Hour, Minimal, Flip, Seconds, Night Sky, Year, Huge, Rings, Segments, Nixie, Words, Weather, Orbit
#define POMO_LONG_BREAK_MIN      (pcfg.longBreak)   // Pomodoro settings (5.5.0)
#define POMO_SESSIONS_PER_CYCLE  (pcfg.cycle)
#define HEAVY_ALERTS        1            // A5 wake category, wide-body types, military

// Pins (v6 phase 1 as built, 12 Oct 2026; docs/images/wiring_v6_phase1.png)
#define TFT_SCK   12
#define TFT_MOSI  11
#define TFT_DC    38      // was 13: GPIO 13 is the microSD slot's MISO (slot is SPI on 10/11/12/13), moved 12 Oct
#define SD_CS     10      // on-board microSD slot, SPI mode, shares SCK 12 / MOSI 11 with the round displays
#define SD_MISO   13
#define TFT_RST   14
// Canonical map (docs/images/wiring_v6.png). PANEL_SWAP (config_local.h) flips left and right for the TFT CS wires,
// the OLED buses and the encoders, for a panel whose harness landed mirrored (6.21.0).
#ifdef PANEL_SWAP
#define TFT_CS_L  16
#define TFT_CS_R  41
#else
#define TFT_CS_L  41
#define TFT_CS_R  16
#endif
#define I2C0_SDA   8      // bus 0: left OLED 0x3C + BMP280 (right OLED with PANEL_SWAP)
#define I2C0_SCL   9
#define I2C1_SDA   1      // bus 1: right OLED 0x3C, alone on its own bus (left OLED with PANEL_SWAP)
#define I2C1_SCL   2
#ifdef PANEL_SWAP
#define ENC_L_A   15
#define ENC_L_B   17
#define ENC_L_SW  18
#define ENC_R_A    4
#define ENC_R_B    5
#define ENC_R_SW   6
#define OLED_L_WIRE Wire1
#define OLED_R_WIRE Wire
#else
#define ENC_L_A    4
#define ENC_L_B    5
#define ENC_L_SW   6
#define ENC_R_A   15
#define ENC_R_B   17
#define ENC_R_SW  18
#define OLED_L_WIRE Wire
#define OLED_R_WIRE Wire1
#endif
#define RING_L_PIN 47
#define RING_R_PIN 21
#define ENC_COUNTS_PER_DETENT 4      // measured on 6.0.9
#define ENC_SIGN (-1)                // clockwise counts down on both knobs as wired

// =====================================================================
// TABLES (flash)
// =====================================================================
struct AirlineRow { const char *icao; const char *iata; const char *name; uint32_t rgb; };
const AirlineRow AIRLINES[] = {
  {"IGO","6E","IndiGo",0x0A2A8F},{"AIC","AI","Air India",0xC8102E},{"AXB","IX","Air India Express",0xF37021},{"VTI","UK","Vistara",0x4B1E5B},{"AKJ","QP","Akasa",0xF26B21},
  {"SEJ","SG","SpiceJet",0xC4161C},{"LLR","9I","Alliance Air",0x1F5FA8},{"FLY","S5","Star Air",0x0B4F8A},{"GOW","G8","Go First",0x5BBCE4},{"TRJ","2T","TruJet",0x00A0A0},
  {"UAE","EK","Emirates",0xD71921},{"QTR","QR","Qatar Airways",0x5C0632},{"ETD","EY","Etihad",0xBD9B5A},{"FDB","FZ","flydubai",0xF58220},{"ABY","G9","Air Arabia",0xD2232A},
  {"SVA","SV","Saudia",0x006C35},{"OMA","WY","Oman Air",0x8A1538},{"GFA","GF","Gulf Air",0xB8860B},{"KAC","KU","Kuwait Airways",0x0072BC},{"JZR","J9","Jazeera",0x2B9EB3},
  {"SIA","SQ","Singapore Airlines",0x1C3F94},{"SLK","MI","SilkAir",0x1E9BD7},{"TGW","TR","Scoot",0xFFCD00},{"MAS","MH","Malaysia Airlines",0x0C2C84},{"AXM","AK","AirAsia",0xE2001A},
  {"THA","TG","Thai Airways",0x5B2C83},{"CPA","CX","Cathay Pacific",0x006564},{"ALK","UL","SriLankan",0x7B1FA2},{"BAW","BA","British Airways",0x075AAA},{"DLH","LH","Lufthansa",0x05164D},
  {"AFR","AF","Air France",0x002157},{"KLM","KL","KLM",0x00A1DE},{"UAL","UA","United",0x002244},{"AAL","AA","American",0x0078D2},{"DAL","DL","Delta",0xE3132C},
  {"JAL","JL","Japan Airlines",0xC8102E},{"ANA","NH","ANA",0x13448F},{"KAL","KE","Korean Air",0x00B2E2},{"CCA","CA","Air China",0xC8102E},{"CES","MU","China Eastern",0x2D3A8C},
  {"ETH","ET","Ethiopian",0x2E8B57},{"THY","TK","Turkish",0xC70A0C},{"VIR","VS","Virgin Atlantic",0xDA0A2D},{"QFA","QF","Qantas",0xE0001B},{"UBD","UB","Myanmar National",0x1F4E79},
  {"BBC","BG","Biman",0x006A4E},{"NPL","RA","Nepal Airlines",0x8B0000},{"GIA","GA","Garuda",0x0A5C9E},{"PAL","PR","Philippine Airlines",0x0038A8},{"HVN","VN","Vietnam Airlines",0xC8A951},
  {"FDX","FX","FedEx",0x4D148C},{"UPS","5X","UPS",0x351C15},{"CLX","CV","Cargolux",0x003366},{"QAC","QR","Qatar Cargo",0x5C0632},{"BDA","BZ","Blue Dart",0x1F6FB2},
  {"IFA","IX","Air India Express",0xF37021},{"CTH","","Chartered",0x555555},
};
#define NUM_AIRLINES (int)(sizeof(AIRLINES) / sizeof(AIRLINES[0]))

// IATA for ICAO airport codes that appear in routes
struct AptCode { const char *icao; const char *iata; };
const AptCode APT[] = {
  {"VOBL","BLR"},{"VIDP","DEL"},{"VABB","BOM"},{"VOMM","MAA"},{"VOHS","HYD"},{"VECC","CCU"},{"VOCI","COK"},{"VAAH","AMD"},{"VAPO","PNQ"},
  {"VOGO","GOI"},{"VOTV","TRV"},{"VOCL","CCJ"},{"VIJP","JAI"},{"VILK","LKO"},{"VEGT","GAU"},{"VOTR","TRZ"},{"VOMD","IXM"},{"VOCB","CJB"},
  {"VANP","NAG"},{"VAID","IDR"},{"VEBS","BBI"},{"VOVZ","VTZ"},{"VEPT","PAT"},{"VIAR","ATQ"},{"VICG","IXC"},{"VOHB","HBX"},{"VOMY","MYQ"},
  {"VOML","IXE"},{"VOBG","HAL"},{"VOPB","IXZ"},{"VEIM","IMF"},{"VOBZ","VGA"},{"VOTP","TIR"},{"VOBM","BEP"},{"VOCP","CDP"},
  {"OMDB","DXB"},{"OMDW","DWC"},{"OMAA","AUH"},{"OMSJ","SHJ"},{"OTHH","DOH"},{"OOMS","MCT"},{"OBBI","BAH"},{"OKBK","KWI"},{"OERK","RUH"},
  {"OEJN","JED"},{"OEDF","DMM"},{"WSSS","SIN"},{"WMKK","KUL"},{"VTBS","BKK"},{"VTBD","DMK"},{"VHHH","HKG"},{"VCBI","CMB"},{"VRMM","MLE"},
  {"VNKT","KTM"},{"VGHS","DAC"},{"EGLL","LHR"},{"EGKK","LGW"},{"EDDF","FRA"},{"EHAM","AMS"},{"LFPG","CDG"},{"LTFM","IST"},{"HAAB","ADD"},
  {"RJTT","HND"},{"RJAA","NRT"},{"RKSI","ICN"},{"ZBAA","PEK"},{"ZSPD","PVG"},{"WIII","CGK"},{"YSSY","SYD"},{"YMML","MEL"},{"KJFK","JFK"},
  {"KEWR","EWR"},{"KSFO","SFO"},{"KORD","ORD"},{"KIAD","IAD"},{"CYYZ","YYZ"},{"OPKC","KHI"},{"OPLA","LHE"},{"VYYY","RGN"},{"VVTS","SGN"},
  {"VVNB","HAN"},{"RPLL","MNL"},{"LIRF","FCO"},{"LEMD","MAD"},{"EDDM","MUC"},{"LSZH","ZRH"},{"EBBR","BRU"},{"LOWW","VIE"},{"HECA","CAI"},
};
#define NUM_APT (int)(sizeof(APT) / sizeof(APT[0]))

// Aircraft types: silhouette kind (0 narrow jet, 1 wide-body, 2 regional jet, 3 turboprop, 4 business jet, 5 helicopter, 6 light prop, 7 military transport),
// wingspan m, engines, wing sweep (0 straight .. 35). Longest matching prefix wins, so "C17" (Globemaster) and "C172" (Cessna) both work.
struct TypeRow { const char *prefix; uint8_t kind, span, engines, sweep; };
const TypeRow TYPES[] = {
  {"A19",0,34,2,25},{"A20",0,36,2,25},{"A21",0,36,2,25},{"B73",0,36,2,25},{"B38",0,36,2,25},{"B39",0,36,2,25},{"B75",0,38,2,25},{"B72",0,33,3,30},{"MD8",0,33,2,30},
  {"BCS",2,35,2,25},{"E19",2,29,2,23},{"E29",2,29,2,23},{"E17",2,26,2,23},{"E75",2,26,2,23},{"CRJ",2,24,2,23},{"E13",2,20,2,22},{"E14",2,20,2,22},{"SU9",2,28,2,25},
  {"A33",1,60,2,30},{"A34",1,63,4,30},{"A35",1,65,2,32},{"A38",1,80,4,33},{"B74",1,65,4,37},{"B77",1,65,2,31},{"B78",1,60,2,32},{"B76",1,48,2,31},{"A30",1,45,2,28},{"A31",1,44,2,28},{"MD1",1,52,3,35},{"DC1",1,50,3,35},{"IL9",1,60,4,35},
  {"AT4",3,25,2,0},{"AT7",3,27,2,0},{"DH8",3,28,2,0},{"Q40",3,28,2,0},{"SF3",3,21,2,0},{"AN2",3,30,2,0},{"AN3",3,30,2,0},{"D32",3,21,2,0},{"JS4",3,18,2,0},{"E12",3,20,2,0},
  {"C30",7,40,4,0},{"C13",7,40,4,0},{"AN1",7,38,4,0},{"IL76",7,50,4,30},{"A124",7,73,4,30},{"C17",7,52,4,25},{"A400",7,42,4,20},{"K35",7,40,4,35},{"P8",0,38,2,25},{"E3",1,45,4,35},
  {"GLF",4,28,2,32},{"GL5",4,28,2,32},{"GL7",4,32,2,32},{"GLE",4,29,2,32},{"CL3",4,21,2,30},{"CL6",4,19,2,30},{"FA7",4,26,3,30},{"F90",4,19,3,30},{"F2T",4,21,2,30},{"FA8",4,26,3,30},
  {"E55",4,16,2,25},{"E54",4,20,2,25},{"C56",4,17,2,20},{"C68",4,21,2,20},{"C25",4,15,2,10},{"C52",4,16,2,20},{"C75",4,22,2,25},{"H25",4,16,2,20},{"LJ",4,12,2,20},{"PRM",4,14,2,20},{"PC24",4,17,2,15},{"HDJ",4,12,2,15},
  {"C172",6,11,1,0},{"C182",6,11,1,0},{"C206",6,11,1,0},{"C208",6,16,1,0},{"C210",6,11,1,0},{"PC12",6,16,1,0},{"B350",6,18,2,0},{"BE20",6,17,2,0},{"BE9",6,15,2,0},{"P28",6,11,1,0},{"DA4",6,13,1,0},{"SR2",6,12,1,0},{"TBM",6,13,1,0},{"PA3",6,12,2,0},{"DHC6",6,20,2,0},
  {"EC3",5,0,0,0},{"EC4",5,0,0,0},{"EC2",5,0,0,0},{"EC55",5,0,0,0},{"A139",5,0,0,0},{"A109",5,0,0,0},{"A169",5,0,0,0},{"B06",5,0,0,0},{"B407",5,0,0,0},{"B412",5,0,0,0},{"B429",5,0,0,0},{"R44",5,0,0,0},{"R66",5,0,0,0},{"S76",5,0,0,0},{"S92",5,0,0,0},{"H60",5,0,0,0},{"AS3",5,0,0,0},{"AS5",5,0,0,0},{"H145",5,0,0,0},{"H175",5,0,0,0},{"MI8",5,0,0,0},{"MI17",5,0,0,0},{"ALH",5,0,0,0},{"DHRV",5,0,0,0},
};
#define NUM_TYPES (int)(sizeof(TYPES) / sizeof(TYPES[0]))

// Wide-body / interesting types for the heavy alert (type code prefixes)
const char *HEAVY_TYPES[] = {"A35", "A38", "A33", "A34", "B74", "B77", "B78", "A124", "A225", "C17", "IL76", "C5", "B52", "P8", "E3"};
#define NUM_HEAVY (int)(sizeof(HEAVY_TYPES) / sizeof(HEAVY_TYPES[0]))

// Buzzer melodies: {frequency Hz, ms}, 0 Hz = rest
struct Note { uint16_t f, ms; };
const Note MEL_CLICK[]    = {{1200, 20}};
const Note MEL_TRAFFIC[]  = {{1800, 80}, {0, 60}, {2400, 80}};
const Note MEL_HEAVY[]    = {{600, 120}, {0, 40}, {600, 120}, {0, 40}, {900, 200}};
const Note MEL_WATCH[]    = {{1500, 80}, {0, 40}, {2000, 80}, {0, 40}, {2500, 160}};
const Note MEL_EMERG[]    = {{2800, 150}, {0, 80}, {2800, 150}, {0, 80}, {2800, 150}, {0, 80}, {2200, 400}};
const Note MEL_OVERHEAD[] = {{1000, 60}, {1300, 60}, {1600, 60}, {2000, 160}};
const Note MEL_RAIN[]     = {{900, 90}, {0, 60}, {700, 90}, {0, 60}, {500, 180}};
const Note MEL_SUN[]      = {{1300, 120}, {0, 40}, {1700, 120}, {0, 40}, {2100, 240}};
const Note MEL_MARKET[]   = {{2200, 60}, {0, 40}, {2200, 60}, {0, 40}, {2200, 60}};
const Note MEL_QUAKE[]    = {{400, 300}, {0, 80}, {400, 300}};
const Note MEL_NUDGE[]    = {{1500, 60}, {0, 300}, {1500, 60}};
const Note MEL_POMO[]     = {{1800, 120}, {0, 60}, {1800, 120}, {0, 60}, {2400, 300}};
const Note MEL_BREAK[]    = {{1200, 150}, {0, 60}, {1600, 250}};
const Note MEL_ALARM[]    = {{2000, 120}, {0, 80}, {2000, 120}, {0, 80}, {2000, 120}, {0, 400}};
const Note MEL_GOAROUND[] = {{1000, 100}, {1400, 100}, {1800, 100}, {2200, 100}, {2600, 250}};
const Note MEL_CHIME[]    = {{1568, 180}, {0, 60}, {1047, 300}};
const Note MEL_TIMER[]    = {{2000, 150}, {0, 100}, {2000, 150}, {0, 100}, {2000, 150}, {0, 100}, {2600, 400}};
enum { M_CLICK, M_TRAFFIC, M_HEAVY, M_WATCH, M_EMERG, M_OVERHEAD, M_RAIN, M_SUN, M_MARKET, M_QUAKE, M_NUDGE, M_POMO, M_BREAK, M_ALARM, M_GOAROUND, M_CHIME, M_TIMER, M_COUNT };
const char *MELODY_NAMES[M_COUNT] = {"click", "traffic", "heavy", "watch", "emergency", "unused", "rain", "sun", "market", "quake", "nudge", "pomodoro", "break", "alarm", "go-around", "chime", "timer"};
const Note *MELODIES[M_COUNT] = {MEL_CLICK, MEL_TRAFFIC, MEL_HEAVY, MEL_WATCH, MEL_EMERG, MEL_OVERHEAD, MEL_RAIN, MEL_SUN, MEL_MARKET, MEL_QUAKE, MEL_NUDGE, MEL_POMO, MEL_BREAK, MEL_ALARM, MEL_GOAROUND, MEL_CHIME, MEL_TIMER};
const uint8_t MELODY_LEN[M_COUNT] = {1, 3, 5, 5, 7, 4, 5, 5, 5, 3, 3, 5, 3, 6, 5, 3, 7};

// ---------------------------------------------------------------------
// FONTS (generated by gen_fonts.py from Poppins / DejaVu, 1-bit GFXfont format)
// ---------------------------------------------------------------------
// F_BIG60: Poppins-Bold.ttf 60px, 18 glyphs, 2265 bytes
const uint8_t F_BIG60_Bitmaps[] PROGMEM = {
  0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFE, 0x00, 0x01, 0xFF, 0x00, 0x1F, 0xFF, 0x80, 0x03, 0xFF, 0x00, 0x3F, 0xFF, 0xC0, 0x03, 0xFE, 0x00,
  0x7F, 0xFF, 0xE0, 0x07, 0xFE, 0x00, 0x7F, 0xFF, 0xE0, 0x07, 0xFC, 0x00, 0x7F, 0xFF, 0xE0, 0x0F, 0xFC, 0x00, 0xFF, 0x0F, 0xF0, 0x0F, 0xF8, 0x00,
  0xFE, 0x07, 0xF0, 0x1F, 0xF0, 0x00, 0xFE, 0x07, 0xF0, 0x3F, 0xF0, 0x00, 0xFE, 0x07, 0xF0, 0x3F, 0xE0, 0x00, 0xFE, 0x07, 0xF0, 0x7F, 0xE0, 0x00,
  0xFF, 0x0F, 0xF0, 0x7F, 0xC0, 0x00, 0xFF, 0x1F, 0xF0, 0xFF, 0xC0, 0x00, 0x7F, 0xFF, 0xE0, 0xFF, 0x80, 0x00, 0x7F, 0xFF, 0xE1, 0xFF, 0x80, 0x00,
  0x3F, 0xFF, 0xC1, 0xFF, 0x00, 0x00, 0x1F, 0xFF, 0x83, 0xFF, 0x00, 0x00, 0x0F, 0xFF, 0x07, 0xFE, 0x00, 0x00, 0x03, 0xFC, 0x07, 0xFC, 0x00, 0x00,
  0x00, 0x00, 0x0F, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xF0, 0x1F, 0xC0,
  0x00, 0x00, 0x3F, 0xF0, 0x7F, 0xF0, 0x00, 0x00, 0x7F, 0xE0, 0xFF, 0xF8, 0x00, 0x00, 0x7F, 0xE1, 0xFF, 0xFC, 0x00, 0x00, 0xFF, 0xC3, 0xFF, 0xFE,
  0x00, 0x00, 0xFF, 0xC3, 0xFF, 0xFE, 0x00, 0x01, 0xFF, 0x87, 0xF8, 0xFF, 0x00, 0x01, 0xFF, 0x87, 0xF0, 0xFF, 0x00, 0x03, 0xFF, 0x07, 0xF0, 0x7F,
  0x00, 0x03, 0xFE, 0x07, 0xF0, 0x7F, 0x00, 0x07, 0xFE, 0x07, 0xF0, 0x7F, 0x00, 0x0F, 0xFC, 0x07, 0xF0, 0x7F, 0x00, 0x0F, 0xFC, 0x07, 0xF0, 0xFF,
  0x00, 0x1F, 0xF8, 0x07, 0xF8, 0xFF, 0x00, 0x1F, 0xF8, 0x03, 0xFF, 0xFE, 0x00, 0x3F, 0xF0, 0x03, 0xFF, 0xFE, 0x00, 0x3F, 0xF0, 0x01, 0xFF, 0xFC,
  0x00, 0x7F, 0xE0, 0x00, 0xFF, 0xF8, 0x00, 0xFF, 0xE0, 0x00, 0x7F, 0xF0, 0x00, 0xFF, 0xC0, 0x00, 0x1F, 0xC0, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x7F,
  0xE0, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x01, 0xFF, 0x80, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x07, 0xFE, 0x00, 0x00, 0x0F, 0xFC, 0x00, 0x00, 0x1F, 0xF8,
  0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x7F, 0xE0, 0x00, 0x00, 0xFF, 0xC0, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0,
  0x03, 0xFF, 0x00, 0x00, 0x07, 0xFE, 0x00, 0x00, 0x0F, 0xFC, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x7F, 0xE0, 0x00, 0x00,
  0xFF, 0xC0, 0x00, 0x01, 0xFF, 0x80, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x07, 0xFE, 0x00, 0x00, 0x0F, 0xFC, 0x00, 0x00, 0x1F, 0xF8, 0xFF, 0x8F, 0xFC,
  0x7F, 0xC3, 0xFE, 0x1F, 0xF0, 0xFF, 0x0F, 0xF8, 0x7F, 0x83, 0xFC, 0x1F, 0xE0, 0xFE, 0x0F, 0xF0, 0x7F, 0x83, 0xF8, 0x1F, 0xC0, 0xFC, 0x00, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x1F, 0x83, 0xFC, 0x7F, 0xEF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xE3, 0xFC, 0x1F, 0x80, 0x00, 0x0F,
  0xFC, 0x00, 0x00, 0x1F, 0xFF, 0xE0, 0x00, 0x1F, 0xFF, 0xFE, 0x00, 0x0F, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFF, 0xF8, 0x03, 0xFF, 0xFF, 0xFF, 0x00,
  0xFF, 0xFF, 0xFF, 0xC0, 0x7F, 0xFF, 0xFF, 0xF8, 0x1F, 0xFF, 0xFF, 0xFE, 0x0F, 0xFF, 0xFF, 0xFF, 0xC3, 0xFF, 0xE1, 0xFF, 0xF1, 0xFF, 0xE0, 0x1F,
  0xFE, 0x7F, 0xF0, 0x03, 0xFF, 0x9F, 0xFC, 0x00, 0xFF, 0xE7, 0xFE, 0x00, 0x1F, 0xFB, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xE0, 0x01, 0xFF, 0xFF, 0xF8,
  0x00, 0x7F, 0xFF, 0xFE, 0x00, 0x1F, 0xFF, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xE0, 0x01, 0xFF, 0xFF, 0xF8, 0x00, 0x7F, 0xFF, 0xFE, 0x00, 0x1F, 0xFF,
  0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xE0, 0x01, 0xFF, 0xFF, 0xF8, 0x00, 0x7F, 0xFF, 0xFE, 0x00, 0x1F, 0xFF, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xE0, 0x01,
  0xFF, 0xFF, 0xF8, 0x00, 0x7F, 0xF7, 0xFE, 0x00, 0x1F, 0xF9, 0xFF, 0xC0, 0x0F, 0xFE, 0x7F, 0xF0, 0x03, 0xFF, 0x9F, 0xFE, 0x01, 0xFF, 0xE3, 0xFF,
  0xE1, 0xFF, 0xF0, 0xFF, 0xFF, 0xFF, 0xFC, 0x1F, 0xFF, 0xFF, 0xFE, 0x07, 0xFF, 0xFF, 0xFF, 0x80, 0xFF, 0xFF, 0xFF, 0xC0, 0x3F, 0xFF, 0xFF, 0xF0,
  0x07, 0xFF, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0xFC, 0x00, 0x1F, 0xFF, 0xFE, 0x00, 0x01, 0xFF, 0xFE, 0x00, 0x00, 0x0F, 0xFC, 0x00, 0x00, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xFF, 0xE0, 0x7F, 0xF0,
  0x3F, 0xF8, 0x1F, 0xFC, 0x0F, 0xFE, 0x07, 0xFF, 0x03, 0xFF, 0x81, 0xFF, 0xC0, 0xFF, 0xE0, 0x7F, 0xF0, 0x3F, 0xF8, 0x1F, 0xFC, 0x0F, 0xFE, 0x07,
  0xFF, 0x03, 0xFF, 0x81, 0xFF, 0xC0, 0xFF, 0xE0, 0x7F, 0xF0, 0x3F, 0xF8, 0x1F, 0xFC, 0x0F, 0xFE, 0x07, 0xFF, 0x03, 0xFF, 0x81, 0xFF, 0xC0, 0xFF,
  0xE0, 0x7F, 0xF0, 0x3F, 0xF8, 0x1F, 0xFC, 0x0F, 0xFE, 0x07, 0xFF, 0x03, 0xFF, 0x81, 0xFF, 0xC0, 0xFF, 0xE0, 0x7F, 0xF0, 0x00, 0x1F, 0xF0, 0x00,
  0x03, 0xFF, 0xF8, 0x00, 0x3F, 0xFF, 0xF0, 0x01, 0xFF, 0xFF, 0xF0, 0x0F, 0xFF, 0xFF, 0xE0, 0x7F, 0xFF, 0xFF, 0x83, 0xFF, 0xFF, 0xFF, 0x0F, 0xFF,
  0xFF, 0xFC, 0x7F, 0xFF, 0xFF, 0xF9, 0xFF, 0xE1, 0xFF, 0xEF, 0xFF, 0x03, 0xFF, 0xBF, 0xF8, 0x07, 0xFE, 0xFF, 0xE0, 0x1F, 0xFB, 0xFF, 0x80, 0x7F,
  0xEF, 0xFE, 0x01, 0xFF, 0x80, 0x00, 0x07, 0xFE, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0xFF, 0xE0, 0x00, 0x03, 0xFF, 0x80, 0x00, 0x1F, 0xFC, 0x00,
  0x00, 0x7F, 0xF0, 0x00, 0x03, 0xFF, 0x80, 0x00, 0x1F, 0xFE, 0x00, 0x00, 0xFF, 0xF0, 0x00, 0x07, 0xFF, 0x80, 0x00, 0x3F, 0xFC, 0x00, 0x01, 0xFF,
  0xF0, 0x00, 0x0F, 0xFF, 0x80, 0x00, 0x7F, 0xFC, 0x00, 0x03, 0xFF, 0xE0, 0x00, 0x1F, 0xFF, 0x00, 0x01, 0xFF, 0xF8, 0x00, 0x0F, 0xFF, 0x80, 0x00,
  0x7F, 0xFC, 0x00, 0x03, 0xFF, 0xE0, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x1F, 0xF8, 0x00, 0x03, 0xFF, 0xF8,
  0x00, 0x3F, 0xFF, 0xF8, 0x03, 0xFF, 0xFF, 0xF0, 0x0F, 0xFF, 0xFF, 0xE0, 0x7F, 0xFF, 0xFF, 0xC3, 0xFF, 0xFF, 0xFF, 0x8F, 0xFF, 0xFF, 0xFE, 0x7F,
  0xFF, 0xFF, 0xFD, 0xFF, 0xE0, 0xFF, 0xF7, 0xFF, 0x01, 0xFF, 0xDF, 0xF8, 0x07, 0xFF, 0x7F, 0xE0, 0x1F, 0xFC, 0x00, 0x00, 0x7F, 0xF0, 0x00, 0x01,
  0xFF, 0xC0, 0x00, 0x07, 0xFE, 0x00, 0x00, 0x7F, 0xF8, 0x00, 0x7F, 0xFF, 0xC0, 0x01, 0xFF, 0xFE, 0x00, 0x07, 0xFF, 0xF0, 0x00, 0x1F, 0xFF, 0x00,
  0x00, 0x7F, 0xF8, 0x00, 0x01, 0xFF, 0xF8, 0x00, 0x07, 0xFF, 0xF8, 0x00, 0x1F, 0xFF, 0xF0, 0x00, 0x7F, 0xFF, 0xE0, 0x00, 0x07, 0xFF, 0x80, 0x00,
  0x0F, 0xFF, 0x00, 0x00, 0x1F, 0xFC, 0x00, 0x00, 0x3F, 0xFF, 0xFC, 0x00, 0xFF, 0xFF, 0xF8, 0x03, 0xFF, 0xFF, 0xE0, 0x0F, 0xFF, 0xFF, 0x80, 0x3F,
  0xFF, 0xFF, 0x01, 0xFF, 0xDF, 0xFE, 0x0F, 0xFF, 0x7F, 0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFF, 0xE3, 0xFF, 0xFF, 0xFF, 0x8F, 0xFF, 0xFF, 0xFC, 0x1F,
  0xFF, 0xFF, 0xF0, 0x3F, 0xFF, 0xFF, 0x80, 0x3F, 0xFF, 0xF8, 0x00, 0x3F, 0xFF, 0xC0, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x00, 0x1F, 0xFC, 0x00, 0x00,
  0x07, 0xFF, 0xC0, 0x00, 0x01, 0xFF, 0xF8, 0x00, 0x00, 0x3F, 0xFF, 0x00, 0x00, 0x0F, 0xFF, 0xE0, 0x00, 0x03, 0xFF, 0xFC, 0x00, 0x00, 0x7F, 0xFF,
  0x80, 0x00, 0x1F, 0xFF, 0xF0, 0x00, 0x07, 0xFF, 0xFE, 0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x00, 0x3F, 0xFF, 0xF8, 0x00, 0x0F, 0xFF, 0xFF, 0x00, 0x03,
  0xFF, 0xFF, 0xE0, 0x00, 0x7F, 0xDF, 0xFC, 0x00, 0x1F, 0xFB, 0xFF, 0x80, 0x07, 0xFE, 0x7F, 0xF0, 0x00, 0xFF, 0xCF, 0xFE, 0x00, 0x3F, 0xF1, 0xFF,
  0xC0, 0x0F, 0xFC, 0x3F, 0xF8, 0x01, 0xFF, 0x87, 0xFF, 0x00, 0x7F, 0xE0, 0xFF, 0xE0, 0x1F, 0xF8, 0x1F, 0xFC, 0x03, 0xFF, 0x03, 0xFF, 0x80, 0xFF,
  0xC0, 0x7F, 0xF0, 0x3F, 0xF0, 0x0F, 0xFE, 0x0F, 0xFE, 0x01, 0xFF, 0xC1, 0xFF, 0x80, 0x3F, 0xF8, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0xFF, 0xE0, 0x00, 0x00, 0x1F, 0xFC, 0x00, 0x00, 0x03, 0xFF, 0x80, 0x00, 0x00, 0x7F,
  0xF0, 0x00, 0x00, 0x0F, 0xFE, 0x00, 0x00, 0x01, 0xFF, 0xC0, 0x00, 0x00, 0x3F, 0xF8, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x7F, 0xFF, 0xFF, 0xF8, 0x7F,
  0xFF, 0xFF, 0xF8, 0x7F, 0xFF, 0xFF, 0xF8, 0x7F, 0xFF, 0xFF, 0xF8, 0x7F, 0xFF, 0xFF, 0xF8, 0x7F, 0xFF, 0xFF, 0xF8, 0x7F, 0xFF, 0xFF, 0xF8, 0x7F,
  0xFF, 0xFF, 0xF8, 0x7F, 0xFF, 0xFF, 0xF8, 0x7F, 0xC0, 0x00, 0x00, 0x7F, 0xC0, 0x00, 0x00, 0x7F, 0xC0, 0x00, 0x00, 0x7F, 0xC0, 0x00, 0x00, 0x7F,
  0xC0, 0x00, 0x00, 0x7F, 0xC3, 0xFE, 0x00, 0x7F, 0xCF, 0xFF, 0x80, 0x7F, 0xFF, 0xFF, 0xE0, 0x7F, 0xFF, 0xFF, 0xF0, 0x7F, 0xFF, 0xFF, 0xF8, 0x7F,
  0xFF, 0xFF, 0xFC, 0x7F, 0xFF, 0xFF, 0xFC, 0x7F, 0xFF, 0xFF, 0xFE, 0x7F, 0xF8, 0x3F, 0xFE, 0x7F, 0xF0, 0x0F, 0xFE, 0x7F, 0xE0, 0x07, 0xFF, 0x7F,
  0xE0, 0x07, 0xFF, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0x07, 0xFF, 0xFF,
  0xE0, 0x07, 0xFF, 0x7F, 0xE0, 0x07, 0xFF, 0x7F, 0xF0, 0x0F, 0xFE, 0x7F, 0xF8, 0x3F, 0xFE, 0x7F, 0xFF, 0xFF, 0xFE, 0x3F, 0xFF, 0xFF, 0xFC, 0x3F,
  0xFF, 0xFF, 0xFC, 0x1F, 0xFF, 0xFF, 0xF8, 0x0F, 0xFF, 0xFF, 0xF0, 0x07, 0xFF, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0x00, 0x00,
  0x1F, 0xF8, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x7F, 0xFF, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x03, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFF, 0xE0, 0x0F,
  0xFF, 0xFF, 0xF0, 0x1F, 0xFF, 0xFF, 0xF8, 0x1F, 0xFF, 0xFF, 0xF8, 0x3F, 0xFF, 0xFF, 0xFC, 0x3F, 0xF8, 0x3F, 0xFC, 0x7F, 0xF0, 0x0F, 0xFC, 0x7F,
  0xE0, 0x0F, 0xFE, 0x7F, 0xC0, 0x07, 0xFE, 0x7F, 0xC0, 0x00, 0x00, 0x7F, 0xC0, 0x00, 0x00, 0xFF, 0x80, 0x00, 0x00, 0xFF, 0x81, 0xFE, 0x00, 0xFF,
  0x8F, 0xFF, 0x80, 0xFF, 0x9F, 0xFF, 0xE0, 0xFF, 0xBF, 0xFF, 0xF0, 0xFF, 0xFF, 0xFF, 0xF8, 0xFF, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0xFF, 0xFC, 0xFF,
  0xFF, 0xFF, 0xFE, 0xFF, 0xFC, 0x1F, 0xFE, 0xFF, 0xF0, 0x0F, 0xFE, 0xFF, 0xE0, 0x07, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFF, 0xC0, 0x03, 0xFF, 0xFF,
  0xC0, 0x03, 0xFF, 0x7F, 0xC0, 0x03, 0xFF, 0x7F, 0xC0, 0x03, 0xFF, 0x7F, 0xC0, 0x03, 0xFF, 0x7F, 0xE0, 0x07, 0xFF, 0x7F, 0xF0, 0x0F, 0xFF, 0x3F,
  0xF8, 0x1F, 0xFE, 0x3F, 0xFF, 0xFF, 0xFE, 0x1F, 0xFF, 0xFF, 0xFC, 0x1F, 0xFF, 0xFF, 0xFC, 0x0F, 0xFF, 0xFF, 0xF8, 0x07, 0xFF, 0xFF, 0xF0, 0x03,
  0xFF, 0xFF, 0xE0, 0x01, 0xFF, 0xFF, 0xC0, 0x00, 0x7F, 0xFF, 0x00, 0x00, 0x0F, 0xFC, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8,
  0x00, 0x01, 0xFF, 0x80, 0x00, 0x0F, 0xFC, 0x00, 0x00, 0x7F, 0xE0, 0x00, 0x07, 0xFE, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x1F,
  0xF8, 0x00, 0x01, 0xFF, 0x80, 0x00, 0x0F, 0xFC, 0x00, 0x00, 0xFF, 0xE0, 0x00, 0x07, 0xFE, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x03, 0xFF, 0x00, 0x00,
  0x1F, 0xF8, 0x00, 0x01, 0xFF, 0xC0, 0x00, 0x0F, 0xFC, 0x00, 0x00, 0xFF, 0xE0, 0x00, 0x07, 0xFE, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x03, 0xFF, 0x00,
  0x00, 0x1F, 0xF8, 0x00, 0x01, 0xFF, 0xC0, 0x00, 0x0F, 0xFC, 0x00, 0x00, 0xFF, 0xE0, 0x00, 0x07, 0xFE, 0x00, 0x00, 0x7F, 0xF0, 0x00, 0x03, 0xFF,
  0x80, 0x00, 0x1F, 0xF8, 0x00, 0x01, 0xFF, 0xC0, 0x00, 0x0F, 0xFC, 0x00, 0x00, 0xFF, 0xE0, 0x00, 0x07, 0xFE, 0x00, 0x00, 0x7F, 0xF0, 0x00, 0x03,
  0xFF, 0x80, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFF, 0xF0, 0x0F,
  0xFF, 0xFF, 0xF8, 0x1F, 0xFF, 0xFF, 0xF8, 0x3F, 0xFF, 0xFF, 0xFC, 0x3F, 0xFF, 0xFF, 0xFE, 0x7F, 0xFC, 0x3F, 0xFE, 0x7F, 0xF8, 0x0F, 0xFE, 0x7F,
  0xF0, 0x0F, 0xFE, 0x7F, 0xF0, 0x07, 0xFE, 0x7F, 0xF0, 0x07, 0xFE, 0x7F, 0xF0, 0x07, 0xFE, 0x7F, 0xF0, 0x07, 0xFE, 0x7F, 0xF0, 0x07, 0xFE, 0x3F,
  0xF0, 0x0F, 0xFE, 0x3F, 0xF8, 0x0F, 0xFC, 0x1F, 0xFC, 0x3F, 0xF8, 0x0F, 0xFF, 0xFF, 0xF0, 0x07, 0xFF, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0xC0, 0x07,
  0xFF, 0xFF, 0xE0, 0x0F, 0xFF, 0xFF, 0xF0, 0x1F, 0xFF, 0xFF, 0xF8, 0x3F, 0xFF, 0xFF, 0xFC, 0x7F, 0xFC, 0x1F, 0xFE, 0x7F, 0xF0, 0x0F, 0xFF, 0x7F,
  0xE0, 0x07, 0xFF, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0xE0, 0x03, 0xFF, 0xFF,
  0xE0, 0x07, 0xFF, 0xFF, 0xF0, 0x0F, 0xFF, 0x7F, 0xFC, 0x1F, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFE, 0x3F, 0xFF, 0xFF, 0xFC, 0x1F,
  0xFF, 0xFF, 0xFC, 0x0F, 0xFF, 0xFF, 0xF8, 0x07, 0xFF, 0xFF, 0xF0, 0x03, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00,
  0x1F, 0xF0, 0x00, 0x01, 0xFF, 0xFC, 0x00, 0x0F, 0xFF, 0xFE, 0x00, 0x3F, 0xFF, 0xFE, 0x00, 0xFF, 0xFF, 0xFE, 0x03, 0xFF, 0xFF, 0xFE, 0x0F, 0xFF,
  0xFF, 0xFC, 0x1F, 0xFF, 0xFF, 0xFC, 0x7F, 0xFF, 0xFF, 0xF8, 0xFF, 0xF0, 0x7F, 0xFB, 0xFF, 0xC0, 0x7F, 0xF7, 0xFF, 0x00, 0x7F, 0xEF, 0xFC, 0x00,
  0x7F, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0xF0, 0x01, 0xFF, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFF, 0x80, 0x0F, 0xFF, 0xFF, 0x80, 0x3F,
  0xFF, 0xFF, 0x80, 0xFF, 0xF7, 0xFF, 0x83, 0xFF, 0xEF, 0xFF, 0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0x9F, 0xFF, 0xFF, 0xFF, 0x1F, 0xFF, 0xFF, 0xFE,
  0x1F, 0xFF, 0xFF, 0xFC, 0x1F, 0xFF, 0xEF, 0xF8, 0x0F, 0xFF, 0x1F, 0xF0, 0x07, 0xF8, 0x3F, 0xE0, 0x00, 0x00, 0x7F, 0xC0, 0x00, 0x01, 0xFF, 0x80,
  0x00, 0x03, 0xFF, 0x7F, 0xE0, 0x07, 0xFE, 0xFF, 0xE0, 0x1F, 0xF9, 0xFF, 0xC0, 0x3F, 0xF1, 0xFF, 0xE0, 0xFF, 0xE3, 0xFF, 0xFF, 0xFF, 0x87, 0xFF,
  0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0xFC, 0x07, 0xFF, 0xFF, 0xF8, 0x07, 0xFF, 0xFF, 0xE0, 0x07, 0xFF, 0xFF, 0x80, 0x07, 0xFF, 0xFE, 0x00, 0x03, 0xFF,
  0xF0, 0x00, 0x01, 0xFF, 0x80, 0x00, 0x1F, 0x83, 0xFC, 0x7F, 0xEF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xE3, 0xFC, 0x1F, 0x80, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x3F, 0xC7, 0xFE,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0xFE, 0x3F, 0xC1, 0xF8, 0x00, 0xFF, 0x80, 0x01, 0xFF, 0xF0, 0x03, 0xFF, 0xFE, 0x03, 0xFF, 0xFF,
  0x83, 0xFF, 0xFF, 0xE3, 0xFF, 0xFF, 0xF9, 0xFF, 0x8F, 0xFD, 0xFF, 0x01, 0xFF, 0xFF, 0x80, 0xFF, 0xFF, 0x80, 0x3F, 0xFF, 0xC0, 0x1F, 0xFF, 0xE0,
  0x0F, 0xFF, 0xF0, 0x07, 0xFF, 0xFC, 0x07, 0xFF, 0xFE, 0x03, 0xFE, 0xFF, 0xC7, 0xFE, 0x7F, 0xFF, 0xFF, 0x1F, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x01,
  0xFF, 0xFF, 0x00, 0x3F, 0xFE, 0x00, 0x07, 0xFC, 0x00,
};
const GFXglyph F_BIG60_Glyphs[] PROGMEM = {
  {    0,   0,   0,  13,    0,    0},   // ' '
  {    0,   0,   0,   0,    0,    0},   // '!'
  {    0,   0,   0,   0,    0,    0},   // '"'
  {    0,   0,   0,   0,    0,    0},   // '#'
  {    0,   0,   0,   0,    0,    0},   // '$'
  {    0,  48,  43,  52,    2,  -43},   // '%'
  {  258,   0,   0,   0,    0,    0},   // '&'
  {  258,   0,   0,   0,    0,    0},   // '''
  {  258,   0,   0,   0,    0,    0},   // '('
  {  258,   0,   0,   0,    0,    0},   // ')'
  {  258,   0,   0,   0,    0,    0},   // '*'
  {  258,  31,  31,  38,    4,  -37},   // '+'
  {  379,  13,  17,  17,    2,   -9},   // ','
  {  407,  26,   9,  35,    3,  -26},   // '-'
  {  437,  12,  11,  17,    2,  -11},   // '.'
  {  454,   0,   0,   0,    0,    0},   // '/'
  {  454,  34,  45,  39,    3,  -45},   // '0'
  {  646,  17,  44,  23,    2,  -44},   // '1'
  {  740,  30,  44,  34,    2,  -45},   // '2'
  {  905,  30,  45,  36,    3,  -45},   // '3'
  { 1074,  35,  44,  41,    3,  -44},   // '4'
  { 1267,  32,  44,  39,    5,  -44},   // '5'
  { 1443,  32,  45,  38,    4,  -45},   // '6'
  { 1623,  29,  44,  32,    2,  -44},   // '7'
  { 1783,  32,  46,  39,    3,  -46},   // '8'
  { 1967,  31,  45,  37,    3,  -45},   // '9'
  { 2142,  12,  36,  17,    2,  -36},   // ':'
  { 2196,   0,   0,   0,    0,    0},   // ';'
  { 2196,   0,   0,   0,    0,    0},   // '<'
  { 2196,   0,   0,   0,    0,    0},   // '='
  { 2196,   0,   0,   0,    0,    0},   // '>'
  { 2196,   0,   0,   0,    0,    0},   // '?'
  { 2196,   0,   0,   0,    0,    0},   // '@'
  { 2196,   0,   0,   0,    0,    0},   // 'A'
  { 2196,   0,   0,   0,    0,    0},   // 'B'
  { 2196,   0,   0,   0,    0,    0},   // 'C'
  { 2196,   0,   0,   0,    0,    0},   // 'D'
  { 2196,   0,   0,   0,    0,    0},   // 'E'
  { 2196,   0,   0,   0,    0,    0},   // 'F'
  { 2196,   0,   0,   0,    0,    0},   // 'G'
  { 2196,   0,   0,   0,    0,    0},   // 'H'
  { 2196,   0,   0,   0,    0,    0},   // 'I'
  { 2196,   0,   0,   0,    0,    0},   // 'J'
  { 2196,   0,   0,   0,    0,    0},   // 'K'
  { 2196,   0,   0,   0,    0,    0},   // 'L'
  { 2196,   0,   0,   0,    0,    0},   // 'M'
  { 2196,   0,   0,   0,    0,    0},   // 'N'
  { 2196,   0,   0,   0,    0,    0},   // 'O'
  { 2196,   0,   0,   0,    0,    0},   // 'P'
  { 2196,   0,   0,   0,    0,    0},   // 'Q'
  { 2196,   0,   0,   0,    0,    0},   // 'R'
  { 2196,   0,   0,   0,    0,    0},   // 'S'
  { 2196,   0,   0,   0,    0,    0},   // 'T'
  { 2196,   0,   0,   0,    0,    0},   // 'U'
  { 2196,   0,   0,   0,    0,    0},   // 'V'
  { 2196,   0,   0,   0,    0,    0},   // 'W'
  { 2196,   0,   0,   0,    0,    0},   // 'X'
  { 2196,   0,   0,   0,    0,    0},   // 'Y'
  { 2196,   0,   0,   0,    0,    0},   // 'Z'
  { 2196,   0,   0,   0,    0,    0},   // '['
  { 2196,   0,   0,   0,    0,    0},   // '?'
  { 2196,   0,   0,   0,    0,    0},   // ']'
  { 2196,   0,   0,   0,    0,    0},   // '^'
  { 2196,   0,   0,   0,    0,    0},   // '_'
  { 2196,   0,   0,   0,    0,    0},   // '`'
  { 2196,   0,   0,   0,    0,    0},   // 'a'
  { 2196,   0,   0,   0,    0,    0},   // 'b'
  { 2196,   0,   0,   0,    0,    0},   // 'c'
  { 2196,   0,   0,   0,    0,    0},   // 'd'
  { 2196,   0,   0,   0,    0,    0},   // 'e'
  { 2196,   0,   0,   0,    0,    0},   // 'f'
  { 2196,   0,   0,   0,    0,    0},   // 'g'
  { 2196,   0,   0,   0,    0,    0},   // 'h'
  { 2196,   0,   0,   0,    0,    0},   // 'i'
  { 2196,   0,   0,   0,    0,    0},   // 'j'
  { 2196,   0,   0,   0,    0,    0},   // 'k'
  { 2196,   0,   0,   0,    0,    0},   // 'l'
  { 2196,   0,   0,   0,    0,    0},   // 'm'
  { 2196,   0,   0,   0,    0,    0},   // 'n'
  { 2196,   0,   0,   0,    0,    0},   // 'o'
  { 2196,   0,   0,   0,    0,    0},   // 'p'
  { 2196,   0,   0,   0,    0,    0},   // 'q'
  { 2196,   0,   0,   0,    0,    0},   // 'r'
  { 2196,   0,   0,   0,    0,    0},   // 's'
  { 2196,   0,   0,   0,    0,    0},   // 't'
  { 2196,   0,   0,   0,    0,    0},   // 'u'
  { 2196,   0,   0,   0,    0,    0},   // 'v'
  { 2196,   0,   0,   0,    0,    0},   // 'w'
  { 2196,   0,   0,   0,    0,    0},   // 'x'
  { 2196,   0,   0,   0,    0,    0},   // 'y'
  { 2196,   0,   0,   0,    0,    0},   // 'z'
  { 2196,   0,   0,   0,    0,    0},   // '{'
  { 2196,   0,   0,   0,    0,    0},   // '|'
  { 2196,   0,   0,   0,    0,    0},   // '}'
  { 2196,   0,   0,   0,    0,    0},   // '~'
  { 2196,  25,  22,  28,    1,  -43},   // 'deg'
};
const GFXfont F_BIG60 PROGMEM = {(uint8_t *)F_BIG60_Bitmaps, (GFXglyph *)F_BIG60_Glyphs, 0x20, 0x7F, 84};

// F_BIG36: Poppins-Bold.ttf 36px, 18 glyphs, 836 bytes
const uint8_t F_BIG36_Bitmaps[] PROGMEM = {
  0x1F, 0x80, 0x3F, 0x01, 0xFE, 0x03, 0xF8, 0x1F, 0xF8, 0x1F, 0x81, 0xFF, 0xE1, 0xFC, 0x0F, 0x9F, 0x0F, 0xC0, 0x78, 0x78, 0xFE, 0x03, 0xC3, 0xC7,
  0xE0, 0x1F, 0x3E, 0x7F, 0x00, 0xFF, 0xF3, 0xF0, 0x03, 0xFF, 0x3F, 0x80, 0x0F, 0xF1, 0xF8, 0x00, 0x3F, 0x1F, 0x80, 0x00, 0x00, 0xFC, 0x00, 0x00,
  0x0F, 0xC7, 0xE0, 0x00, 0x7E, 0x7F, 0x80, 0x07, 0xE7, 0xFE, 0x00, 0x7F, 0x7F, 0xF8, 0x03, 0xF3, 0xE7, 0xC0, 0x3F, 0x9E, 0x1E, 0x01, 0xF8, 0xF0,
  0xF0, 0x1F, 0xC7, 0x87, 0x80, 0xFC, 0x3E, 0x7C, 0x0F, 0xE1, 0xFF, 0xE0, 0x7E, 0x07, 0xFE, 0x07, 0xF0, 0x1F, 0xE0, 0x3F, 0x00, 0x7E, 0x00, 0x03,
  0xF0, 0x00, 0xFC, 0x00, 0x3F, 0x00, 0x0F, 0xC0, 0x03, 0xF0, 0x00, 0xFC, 0x00, 0x3F, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xFC, 0x00, 0x3F, 0x00, 0x0F, 0xC0, 0x03, 0xF0, 0x00, 0xFC, 0x00, 0x3F, 0x00, 0x3F, 0x3E, 0x7E, 0x7E, 0x7C, 0x7C,
  0x7C, 0x78, 0xF8, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x38, 0xFB, 0xFF, 0xFF, 0xEF, 0x8E, 0x00,
  0x00, 0x20, 0x00, 0x7F, 0xE0, 0x0F, 0xFF, 0x01, 0xFF, 0xF8, 0x3F, 0xFF, 0xC3, 0xFF, 0xFE, 0x7F, 0x0F, 0xE7, 0xE0, 0x7E, 0x7E, 0x07, 0xFF, 0xE0,
  0x3F, 0xFE, 0x03, 0xFF, 0xC0, 0x3F, 0xFC, 0x03, 0xFF, 0xC0, 0x3F, 0xFC, 0x03, 0xFF, 0xC0, 0x3F, 0xFC, 0x03, 0xFF, 0xC0, 0x3F, 0xFE, 0x03, 0xFF,
  0xE0, 0x7F, 0x7E, 0x07, 0xF7, 0xF0, 0xFE, 0x7F, 0xFF, 0xE3, 0xFF, 0xFC, 0x3F, 0xFF, 0xC1, 0xFF, 0xF8, 0x07, 0xFF, 0x00, 0x1F, 0xC0, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xDF, 0xF0, 0xFC, 0x3F, 0x0F, 0xC3, 0xF0, 0xFC, 0x3F, 0x0F, 0xC3, 0xF0, 0xFC, 0x3F, 0x0F, 0xC3, 0xF0, 0xFC, 0x3F, 0x0F,
  0xC3, 0xF0, 0xFC, 0x3F, 0x0F, 0xC3, 0xF0, 0xFC, 0x00, 0xC0, 0x01, 0xFF, 0x01, 0xFF, 0xF0, 0xFF, 0xFC, 0x7F, 0xFF, 0x9F, 0xFF, 0xFF, 0xF1, 0xFF,
  0xF8, 0x7F, 0xFE, 0x0F, 0xFF, 0x03, 0xF0, 0x01, 0xFC, 0x00, 0x7F, 0x00, 0x1F, 0x80, 0x0F, 0xE0, 0x07, 0xF0, 0x03, 0xFC, 0x01, 0xFE, 0x00, 0xFF,
  0x00, 0x7F, 0x80, 0x3F, 0xC0, 0x1F, 0xE0, 0x0F, 0xF0, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03,
  0xF8, 0x03, 0xFF, 0x83, 0xFF, 0xF0, 0xFF, 0xFE, 0x7F, 0xFF, 0x9F, 0xFF, 0xFF, 0xE1, 0xFF, 0xF8, 0x3F, 0x00, 0x0F, 0xC0, 0x07, 0xF0, 0x03, 0xF8,
  0x07, 0xFE, 0x01, 0xFF, 0x00, 0x7F, 0x00, 0x1F, 0xE0, 0x07, 0xFE, 0x01, 0xFF, 0x80, 0x07, 0xF0, 0x00, 0xFF, 0xF0, 0x3F, 0xFE, 0x0F, 0xFF, 0x87,
  0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0x7F, 0xFF, 0x8F, 0xFF, 0xC1, 0xFF, 0xE0, 0x0F, 0xE0, 0x00, 0x1F, 0xC0, 0x00, 0xFE, 0x00, 0x0F, 0xF0, 0x00, 0xFF,
  0x80, 0x0F, 0xFC, 0x00, 0x7F, 0xE0, 0x07, 0xFF, 0x00, 0x7F, 0xF8, 0x03, 0xEF, 0xC0, 0x3F, 0x7E, 0x03, 0xF3, 0xF0, 0x1F, 0x9F, 0x81, 0xF8, 0xFC,
  0x1F, 0x87, 0xE0, 0xFC, 0x3F, 0x0F, 0xC1, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC,
  0x00, 0x3F, 0x00, 0x01, 0xF8, 0x00, 0x0F, 0xC0, 0x00, 0x7E, 0x00, 0x03, 0xF0, 0xFF, 0xFF, 0xBF, 0xFF, 0xEF, 0xFF, 0xFB, 0xFF, 0xFE, 0xFF, 0xFF,
  0xBF, 0xFF, 0xCF, 0xC0, 0x03, 0xF0, 0x00, 0xFC, 0x20, 0x3F, 0x7F, 0x8F, 0xFF, 0xF3, 0xFF, 0xFE, 0xFF, 0xFF, 0xBF, 0xFF, 0xFF, 0xF1, 0xFF, 0xF8,
  0x7F, 0x00, 0x0F, 0xC0, 0x03, 0xF0, 0x00, 0xFF, 0xF0, 0x3F, 0xFE, 0x1F, 0xFF, 0xFF, 0xF7, 0xFF, 0xF9, 0xFF, 0xFE, 0x3F, 0xFF, 0x03, 0xFF, 0x80,
  0x3F, 0x80, 0x03, 0xF8, 0x01, 0xFF, 0xC0, 0x7F, 0xFC, 0x1F, 0xFF, 0xC3, 0xFF, 0xF8, 0xFF, 0xFF, 0x9F, 0x87, 0xF3, 0xE0, 0x7E, 0xFC, 0x00, 0x1F,
  0x00, 0x03, 0xE3, 0xF0, 0x7D, 0xFF, 0x8F, 0xFF, 0xF9, 0xFF, 0xFF, 0xBF, 0xFF, 0xF7, 0xFF, 0xFF, 0xFE, 0x0F, 0xFF, 0x81, 0xFF, 0xF0, 0x1F, 0xFE,
  0x03, 0xF7, 0xC0, 0xFE, 0xFC, 0x1F, 0xDF, 0xFF, 0xF1, 0xFF, 0xFE, 0x3F, 0xFF, 0x83, 0xFF, 0xF0, 0x3F, 0xF8, 0x01, 0xFC, 0x00, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0xFC, 0x00, 0x7E, 0x00, 0x3F, 0x00, 0x3F, 0x00, 0x1F, 0x80, 0x1F, 0x80, 0x0F,
  0xC0, 0x07, 0xE0, 0x07, 0xE0, 0x03, 0xF0, 0x03, 0xF0, 0x01, 0xF8, 0x01, 0xFC, 0x00, 0xFC, 0x00, 0x7E, 0x00, 0x7E, 0x00, 0x3F, 0x00, 0x3F, 0x80,
  0x1F, 0x80, 0x0F, 0xC0, 0x0F, 0xC0, 0x00, 0x00, 0x40, 0x00, 0xFF, 0xC0, 0x7F, 0xFE, 0x1F, 0xFF, 0xE7, 0xFF, 0xFC, 0xFF, 0xFF, 0xDF, 0xC7, 0xFB,
  0xF0, 0x7F, 0x7E, 0x07, 0xEF, 0xC0, 0xFD, 0xF8, 0x3F, 0x1F, 0x8F, 0xE1, 0xFF, 0xF8, 0x1F, 0xFE, 0x07, 0xFF, 0xE1, 0xFF, 0xFE, 0x7F, 0x1F, 0xDF,
  0xC1, 0xFF, 0xF8, 0x1F, 0xFE, 0x03, 0xFF, 0xC0, 0x7F, 0xFC, 0x0F, 0xFF, 0x83, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xC7, 0xFF, 0xF8, 0x7F, 0xFE, 0x07,
  0xFF, 0x00, 0x3F, 0x80, 0x03, 0xF8, 0x01, 0xFF, 0xC0, 0xFF, 0xFC, 0x1F, 0xFF, 0xC7, 0xFF, 0xF8, 0xFF, 0xFF, 0xBF, 0x83, 0xF7, 0xF0, 0x7E, 0xFC,
  0x07, 0xFF, 0x80, 0xFF, 0xF8, 0x3F, 0xFF, 0x07, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xDF, 0xFF, 0xF9, 0xFF, 0xFF, 0x1F, 0xF7, 0xE0, 0xFC, 0xFC, 0x00,
  0x1F, 0x80, 0x03, 0xF7, 0xE0, 0x7C, 0xFE, 0x1F, 0x9F, 0xFF, 0xF1, 0xFF, 0xFC, 0x3F, 0xFF, 0x83, 0xFF, 0xE0, 0x3F, 0xF8, 0x01, 0xFC, 0x00, 0x38,
  0xFB, 0xFF, 0xFF, 0xEF, 0x8E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x7D, 0xFF, 0xFF, 0xF7, 0xC7, 0x00, 0x0F, 0xE0, 0x7F, 0xF1, 0xFF,
  0xF3, 0xE3, 0xEF, 0xC7, 0xFF, 0x07, 0xFE, 0x0F, 0xFC, 0x1F, 0xFC, 0x3E, 0xF8, 0xF9, 0xFF, 0xF1, 0xFF, 0xC0, 0xFE, 0x00,
};
const GFXglyph F_BIG36_Glyphs[] PROGMEM = {
  {    0,   0,   0,   8,    0,    0},   // ' '
  {    0,   0,   0,   0,    0,    0},   // '!'
  {    0,   0,   0,   0,    0,    0},   // '"'
  {    0,   0,   0,   0,    0,    0},   // '#'
  {    0,   0,   0,   0,    0,    0},   // '$'
  {    0,  29,  26,  31,    1,  -26},   // '%'
  {   95,   0,   0,   0,    0,    0},   // '&'
  {   95,   0,   0,   0,    0,    0},   // '''
  {   95,   0,   0,   0,    0,    0},   // '('
  {   95,   0,   0,   0,    0,    0},   // ')'
  {   95,   0,   0,   0,    0,    0},   // '*'
  {   95,  18,  19,  23,    2,  -23},   // '+'
  {  138,   8,  11,  10,    1,   -6},   // ','
  {  149,  16,   6,  21,    2,  -16},   // '-'
  {  161,   7,   7,  10,    1,   -7},   // '.'
  {  168,   0,   0,   0,    0,    0},   // '/'
  {  168,  20,  28,  23,    2,  -28},   // '0'
  {  238,  10,  27,  14,    0,  -27},   // '1'
  {  272,  18,  28,  21,    1,  -28},   // '2'
  {  335,  18,  28,  22,    2,  -28},   // '3'
  {  398,  21,  27,  24,    2,  -27},   // '4'
  {  469,  18,  27,  23,    3,  -27},   // '5'
  {  530,  19,  28,  23,    2,  -28},   // '6'
  {  597,  17,  27,  19,    1,  -27},   // '7'
  {  655,  19,  29,  23,    2,  -29},   // '8'
  {  724,  19,  28,  22,    2,  -28},   // '9'
  {  791,   7,  22,  10,    1,  -22},   // ':'
  {  811,   0,   0,   0,    0,    0},   // ';'
  {  811,   0,   0,   0,    0,    0},   // '<'
  {  811,   0,   0,   0,    0,    0},   // '='
  {  811,   0,   0,   0,    0,    0},   // '>'
  {  811,   0,   0,   0,    0,    0},   // '?'
  {  811,   0,   0,   0,    0,    0},   // '@'
  {  811,   0,   0,   0,    0,    0},   // 'A'
  {  811,   0,   0,   0,    0,    0},   // 'B'
  {  811,   0,   0,   0,    0,    0},   // 'C'
  {  811,   0,   0,   0,    0,    0},   // 'D'
  {  811,   0,   0,   0,    0,    0},   // 'E'
  {  811,   0,   0,   0,    0,    0},   // 'F'
  {  811,   0,   0,   0,    0,    0},   // 'G'
  {  811,   0,   0,   0,    0,    0},   // 'H'
  {  811,   0,   0,   0,    0,    0},   // 'I'
  {  811,   0,   0,   0,    0,    0},   // 'J'
  {  811,   0,   0,   0,    0,    0},   // 'K'
  {  811,   0,   0,   0,    0,    0},   // 'L'
  {  811,   0,   0,   0,    0,    0},   // 'M'
  {  811,   0,   0,   0,    0,    0},   // 'N'
  {  811,   0,   0,   0,    0,    0},   // 'O'
  {  811,   0,   0,   0,    0,    0},   // 'P'
  {  811,   0,   0,   0,    0,    0},   // 'Q'
  {  811,   0,   0,   0,    0,    0},   // 'R'
  {  811,   0,   0,   0,    0,    0},   // 'S'
  {  811,   0,   0,   0,    0,    0},   // 'T'
  {  811,   0,   0,   0,    0,    0},   // 'U'
  {  811,   0,   0,   0,    0,    0},   // 'V'
  {  811,   0,   0,   0,    0,    0},   // 'W'
  {  811,   0,   0,   0,    0,    0},   // 'X'
  {  811,   0,   0,   0,    0,    0},   // 'Y'
  {  811,   0,   0,   0,    0,    0},   // 'Z'
  {  811,   0,   0,   0,    0,    0},   // '['
  {  811,   0,   0,   0,    0,    0},   // '?'
  {  811,   0,   0,   0,    0,    0},   // ']'
  {  811,   0,   0,   0,    0,    0},   // '^'
  {  811,   0,   0,   0,    0,    0},   // '_'
  {  811,   0,   0,   0,    0,    0},   // '`'
  {  811,   0,   0,   0,    0,    0},   // 'a'
  {  811,   0,   0,   0,    0,    0},   // 'b'
  {  811,   0,   0,   0,    0,    0},   // 'c'
  {  811,   0,   0,   0,    0,    0},   // 'd'
  {  811,   0,   0,   0,    0,    0},   // 'e'
  {  811,   0,   0,   0,    0,    0},   // 'f'
  {  811,   0,   0,   0,    0,    0},   // 'g'
  {  811,   0,   0,   0,    0,    0},   // 'h'
  {  811,   0,   0,   0,    0,    0},   // 'i'
  {  811,   0,   0,   0,    0,    0},   // 'j'
  {  811,   0,   0,   0,    0,    0},   // 'k'
  {  811,   0,   0,   0,    0,    0},   // 'l'
  {  811,   0,   0,   0,    0,    0},   // 'm'
  {  811,   0,   0,   0,    0,    0},   // 'n'
  {  811,   0,   0,   0,    0,    0},   // 'o'
  {  811,   0,   0,   0,    0,    0},   // 'p'
  {  811,   0,   0,   0,    0,    0},   // 'q'
  {  811,   0,   0,   0,    0,    0},   // 'r'
  {  811,   0,   0,   0,    0,    0},   // 's'
  {  811,   0,   0,   0,    0,    0},   // 't'
  {  811,   0,   0,   0,    0,    0},   // 'u'
  {  811,   0,   0,   0,    0,    0},   // 'v'
  {  811,   0,   0,   0,    0,    0},   // 'w'
  {  811,   0,   0,   0,    0,    0},   // 'x'
  {  811,   0,   0,   0,    0,    0},   // 'y'
  {  811,   0,   0,   0,    0,    0},   // 'z'
  {  811,   0,   0,   0,    0,    0},   // '{'
  {  811,   0,   0,   0,    0,    0},   // '|'
  {  811,   0,   0,   0,    0,    0},   // '}'
  {  811,   0,   0,   0,    0,    0},   // '~'
  {  811,  15,  13,  17,    1,  -26},   // 'deg'
};
const GFXfont F_BIG36 PROGMEM = {(uint8_t *)F_BIG36_Bitmaps, (GFXglyph *)F_BIG36_Glyphs, 0x20, 0x7F, 51};
// 6.20.0: right OLED clock (digits and colon only, 35 px tall, '12:59' = 114 px wide)
// F_BIG46: Poppins-Bold.ttf 46px, 12 glyphs, 1028 bytes
const uint8_t F_BIG46_Bitmaps[] PROGMEM = {
  0x00, 0x7F, 0x80, 0x00, 0x7F, 0xF8, 0x00, 0x7F, 0xFF, 0x80, 0x3F, 0xFF, 0xF0, 0x1F, 0xFF, 0xFE, 0x07, 0xFF, 0xFF, 0x83, 0xFF, 0xFF, 0xF0, 0xFF,
  0x87, 0xFC, 0x7F, 0xC0, 0xFF, 0x9F, 0xE0, 0x1F, 0xE7, 0xF8, 0x07, 0xFB, 0xFE, 0x01, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xC0, 0x0F, 0xFF, 0xF0, 0x03,
  0xFF, 0xFC, 0x00, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xC0, 0x0F, 0xFF, 0xF0, 0x03, 0xFF, 0xFC, 0x00, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xC0, 0x0F, 0xFF,
  0xF0, 0x03, 0xFD, 0xFE, 0x01, 0xFE, 0x7F, 0x80, 0x7F, 0x9F, 0xF0, 0x3F, 0xE7, 0xFE, 0x1F, 0xF8, 0xFF, 0xFF, 0xFC, 0x3F, 0xFF, 0xFF, 0x07, 0xFF,
  0xFF, 0x80, 0xFF, 0xFF, 0xC0, 0x1F, 0xFF, 0xE0, 0x03, 0xFF, 0xF0, 0x00, 0x1F, 0xE0, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xE0, 0xFF, 0x07, 0xF8, 0x3F, 0xC1, 0xFE, 0x0F, 0xF0, 0x7F, 0x83, 0xFC, 0x1F, 0xE0, 0xFF, 0x07, 0xF8, 0x3F, 0xC1, 0xFE, 0x0F, 0xF0,
  0x7F, 0x83, 0xFC, 0x1F, 0xE0, 0xFF, 0x07, 0xF8, 0x3F, 0xC1, 0xFE, 0x0F, 0xF0, 0x7F, 0x83, 0xFC, 0x1F, 0xE0, 0xFF, 0x07, 0xF8, 0x00, 0xFF, 0x00,
  0x0F, 0xFF, 0xC0, 0x3F, 0xFF, 0xC0, 0xFF, 0xFF, 0xC3, 0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0x9F, 0xFF, 0xFF, 0xBF, 0xE1, 0xFF, 0xFF, 0x83, 0xFF, 0xFE,
  0x03, 0xFF, 0xFC, 0x07, 0xFF, 0xF8, 0x0F, 0xF0, 0x00, 0x1F, 0xE0, 0x00, 0x7F, 0xC0, 0x00, 0xFF, 0x00, 0x03, 0xFE, 0x00, 0x0F, 0xF8, 0x00, 0x1F,
  0xF0, 0x00, 0x7F, 0xC0, 0x01, 0xFF, 0x00, 0x07, 0xFC, 0x00, 0x1F, 0xF8, 0x00, 0xFF, 0xE0, 0x03, 0xFF, 0x80, 0x0F, 0xFC, 0x00, 0x3F, 0xF0, 0x00,
  0xFF, 0xC0, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00,
  0x7E, 0x00, 0x07, 0xFF, 0x80, 0x3F, 0xFF, 0xC0, 0xFF, 0xFF, 0xC3, 0xFF, 0xFF, 0xC7, 0xFF, 0xFF, 0x9F, 0xFF, 0xFF, 0xBF, 0xE3, 0xFF, 0x7F, 0x83,
  0xFF, 0xFF, 0x03, 0xFD, 0xFC, 0x07, 0xF8, 0x00, 0x0F, 0xF0, 0x00, 0x3F, 0xE0, 0x00, 0xFF, 0x80, 0x1F, 0xFE, 0x00, 0x3F, 0xF8, 0x00, 0x7F, 0xE0,
  0x00, 0xFF, 0x80, 0x01, 0xFF, 0xC0, 0x03, 0xFF, 0xC0, 0x07, 0xFF, 0xC0, 0x00, 0xFF, 0x80, 0x00, 0xFF, 0x80, 0x00, 0xFF, 0xFF, 0x01, 0xFF, 0xFE,
  0x03, 0xFF, 0xFE, 0x0F, 0xFF, 0xFE, 0x1F, 0xF7, 0xFF, 0xFF, 0xEF, 0xFF, 0xFF, 0x8F, 0xFF, 0xFF, 0x1F, 0xFF, 0xFC, 0x1F, 0xFF, 0xF0, 0x0F, 0xFF,
  0x80, 0x03, 0xFC, 0x00, 0x00, 0x03, 0xFE, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x3F, 0xF8, 0x00, 0x07, 0xFF, 0x00, 0x01, 0xFF, 0xE0, 0x00, 0x7F, 0xFC,
  0x00, 0x0F, 0xFF, 0x80, 0x03, 0xFF, 0xF0, 0x00, 0xFF, 0xFE, 0x00, 0x1F, 0xDF, 0xC0, 0x07, 0xFB, 0xF8, 0x01, 0xFE, 0x7F, 0x00, 0x7F, 0x8F, 0xE0,
  0x0F, 0xF1, 0xFC, 0x03, 0xFC, 0x3F, 0x80, 0xFF, 0x07, 0xF0, 0x1F, 0xE0, 0xFE, 0x07, 0xF8, 0x1F, 0xC1, 0xFE, 0x03, 0xF8, 0x3F, 0xC0, 0x7F, 0x0F,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80,
  0x00, 0xFF, 0x00, 0x00, 0x1F, 0xE0, 0x00, 0x03, 0xFC, 0x00, 0x00, 0x7F, 0x80, 0x00, 0x0F, 0xF0, 0x00, 0x01, 0xFE, 0x00, 0x7F, 0xFF, 0xFE, 0x3F,
  0xFF, 0xFF, 0x1F, 0xFF, 0xFF, 0x8F, 0xFF, 0xFF, 0xC7, 0xFF, 0xFF, 0xE3, 0xFF, 0xFF, 0xF1, 0xFF, 0xFF, 0xF8, 0xFE, 0x00, 0x00, 0x7F, 0x00, 0x00,
  0x3F, 0x80, 0x00, 0x1F, 0xC0, 0x00, 0x0F, 0xE3, 0xF8, 0x07, 0xF7, 0xFF, 0x03, 0xFF, 0xFF, 0xE1, 0xFF, 0xFF, 0xF8, 0xFF, 0xFF, 0xFC, 0x7F, 0xFF,
  0xFF, 0x3F, 0xFF, 0xFF, 0x9F, 0xF0, 0x7F, 0xEF, 0xF0, 0x1F, 0xF7, 0xF0, 0x07, 0xF8, 0x00, 0x03, 0xFC, 0x00, 0x01, 0xFE, 0x00, 0x00, 0xFF, 0xFF,
  0x00, 0x7F, 0xBF, 0xC0, 0x7F, 0xDF, 0xF0, 0x7F, 0xCF, 0xFF, 0xFF, 0xE3, 0xFF, 0xFF, 0xF1, 0xFF, 0xFF, 0xF0, 0x7F, 0xFF, 0xF0, 0x1F, 0xFF, 0xF0,
  0x03, 0xFF, 0xF0, 0x00, 0x7F, 0xC0, 0x00, 0x00, 0x7E, 0x00, 0x03, 0xFF, 0xC0, 0x07, 0xFF, 0xE0, 0x0F, 0xFF, 0xF0, 0x1F, 0xFF, 0xF8, 0x3F, 0xFF,
  0xFC, 0x3F, 0xFF, 0xFC, 0x7F, 0xC7, 0xFC, 0x7F, 0x83, 0xFE, 0x7F, 0x01, 0xFE, 0x7F, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFE, 0x00, 0x00, 0xFE, 0x1F,
  0xC0, 0xFE, 0x7F, 0xF0, 0xFE, 0xFF, 0xF8, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFE, 0xFF, 0xC3, 0xFF, 0xFF, 0x01,
  0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0x7F, 0x00, 0xFF, 0x7F, 0x00, 0xFF, 0x7F, 0x01, 0xFF, 0x7F, 0x83, 0xFF, 0x3F, 0xFF, 0xFE, 0x3F, 0xFF,
  0xFE, 0x1F, 0xFF, 0xFC, 0x1F, 0xFF, 0xF8, 0x0F, 0xFF, 0xF0, 0x03, 0xFF, 0xE0, 0x00, 0xFF, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0xFE, 0x00, 0x07, 0xF8, 0x00, 0x1F, 0xE0, 0x00, 0x7F, 0x00, 0x03,
  0xFC, 0x00, 0x0F, 0xE0, 0x00, 0x7F, 0x80, 0x01, 0xFE, 0x00, 0x0F, 0xF0, 0x00, 0x3F, 0xC0, 0x00, 0xFE, 0x00, 0x07, 0xF8, 0x00, 0x1F, 0xE0, 0x00,
  0xFF, 0x00, 0x03, 0xFC, 0x00, 0x1F, 0xE0, 0x00, 0x7F, 0x80, 0x01, 0xFE, 0x00, 0x0F, 0xF0, 0x00, 0x3F, 0xC0, 0x01, 0xFE, 0x00, 0x07, 0xF8, 0x00,
  0x3F, 0xC0, 0x00, 0xFF, 0x00, 0x03, 0xFC, 0x00, 0x1F, 0xE0, 0x00, 0x7F, 0x80, 0x00, 0x00, 0xFF, 0x80, 0x01, 0xFF, 0xF0, 0x03, 0xFF, 0xFE, 0x03,
  0xFF, 0xFF, 0x83, 0xFF, 0xFF, 0xE1, 0xFF, 0xFF, 0xF1, 0xFF, 0xFF, 0xFC, 0xFF, 0x83, 0xFE, 0x7F, 0xC1, 0xFF, 0x3F, 0xC0, 0x7F, 0x9F, 0xE0, 0x3F,
  0xCF, 0xF0, 0x1F, 0xE3, 0xFC, 0x1F, 0xE1, 0xFF, 0x1F, 0xF0, 0x7F, 0xFF, 0xF0, 0x1F, 0xFF, 0xF0, 0x07, 0xFF, 0xF0, 0x0F, 0xFF, 0xFE, 0x0F, 0xFF,
  0xFF, 0x8F, 0xFF, 0xFF, 0xE7, 0xFC, 0x1F, 0xF7, 0xFC, 0x07, 0xFF, 0xFC, 0x01, 0xFF, 0xFE, 0x00, 0xFF, 0xFF, 0x00, 0x7F, 0xFF, 0x80, 0x3F, 0xFF,
  0xE0, 0x3F, 0xFF, 0xF8, 0x3F, 0xF7, 0xFF, 0xFF, 0xF3, 0xFF, 0xFF, 0xF8, 0xFF, 0xFF, 0xF8, 0x3F, 0xFF, 0xF8, 0x0F, 0xFF, 0xF8, 0x01, 0xFF, 0xF0,
  0x00, 0x3F, 0xE0, 0x00, 0x00, 0xFE, 0x00, 0x03, 0xFF, 0xC0, 0x0F, 0xFF, 0xF0, 0x1F, 0xFF, 0xF8, 0x3F, 0xFF, 0xF8, 0x7F, 0xFF, 0xFC, 0x7F, 0xFF,
  0xFC, 0x7F, 0xC3, 0xFE, 0xFF, 0x81, 0xFE, 0xFF, 0x00, 0xFE, 0xFF, 0x00, 0xFE, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x81,
  0xFF, 0xFF, 0xC3, 0xFF, 0x7F, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0x3F, 0xFF, 0xFF, 0x3F, 0xFF, 0xFF, 0x1F, 0xFF, 0x7F, 0x07, 0xFE, 0x7F, 0x01, 0xF8,
  0xFF, 0x00, 0x00, 0xFF, 0x00, 0x00, 0xFF, 0x7F, 0x80, 0xFE, 0x7F, 0x81, 0xFE, 0x7F, 0xC1, 0xFE, 0x3F, 0xFF, 0xFC, 0x3F, 0xFF, 0xFC, 0x1F, 0xFF,
  0xF8, 0x1F, 0xFF, 0xF8, 0x0F, 0xFF, 0xF0, 0x03, 0xFF, 0xE0, 0x00, 0xFF, 0x00, 0x3E, 0x3F, 0xBF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0x3E, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x8F, 0xEF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBF, 0x8F, 0x80,
};
const GFXglyph F_BIG46_Glyphs[] PROGMEM = {
  {    0,   0,   0,  10,    0,    0},   // ' '
  {    0,   0,   0,   0,    0,    0},   // '!'
  {    0,   0,   0,   0,    0,    0},   // '"'
  {    0,   0,   0,   0,    0,    0},   // '#'
  {    0,   0,   0,   0,    0,    0},   // '$'
  {    0,   0,   0,   0,    0,    0},   // '%'
  {    0,   0,   0,   0,    0,    0},   // '&'
  {    0,   0,   0,   0,    0,    0},   // '''
  {    0,   0,   0,   0,    0,    0},   // '('
  {    0,   0,   0,   0,    0,    0},   // ')'
  {    0,   0,   0,   0,    0,    0},   // '*'
  {    0,   0,   0,   0,    0,    0},   // '+'
  {    0,   0,   0,   0,    0,    0},   // ','
  {    0,   0,   0,   0,    0,    0},   // '-'
  {    0,   0,   0,   0,    0,    0},   // '.'
  {    0,   0,   0,   0,    0,    0},   // '/'
  {    0,  26,  34,  30,    2,  -34},   // '0'
  {  111,  13,  33,  17,    1,  -33},   // '1'
  {  165,  23,  34,  26,    2,  -34},   // '2'
  {  263,  23,  35,  28,    2,  -35},   // '3'
  {  364,  27,  33,  31,    2,  -33},   // '4'
  {  476,  25,  34,  30,    3,  -34},   // '5'
  {  583,  24,  35,  29,    3,  -35},   // '6'
  {  688,  22,  34,  24,    1,  -34},   // '7'
  {  782,  25,  35,  29,    2,  -35},   // '8'
  {  892,  24,  35,  28,    2,  -35},   // '9'
  {  997,   9,  27,  13,    2,  -27},   // ':'
};
const GFXfont F_BIG46 PROGMEM = {(uint8_t *)F_BIG46_Bitmaps, (GFXglyph *)F_BIG46_Glyphs, 0x20, 0x3A, 66};

// F_MID22: Poppins-Medium.ttf 22px, 96 glyphs, 1805 bytes
const uint8_t F_MID22_Bitmaps[] PROGMEM = {
  0xFF, 0xFF, 0xFF, 0xFC, 0x0F, 0xF8, 0xFF, 0xFF, 0xFF, 0x03, 0x06, 0x03, 0x87, 0x01, 0xC3, 0x80, 0xE1, 0xC7, 0xFF, 0xFB, 0xFF, 0xFC, 0x38, 0x70,
  0x1C, 0x38, 0x0C, 0x18, 0x06, 0x1C, 0x3F, 0xFF, 0xDF, 0xFF, 0xE1, 0xC3, 0x00, 0xC1, 0x80, 0x61, 0xC0, 0x70, 0xE0, 0x04, 0x00, 0x80, 0xFC, 0x3F,
  0xEE, 0x5D, 0xC9, 0xB9, 0x07, 0x20, 0x7C, 0x07, 0xF0, 0x1F, 0x02, 0xE0, 0x4F, 0xC9, 0xF9, 0x73, 0xFE, 0x3F, 0x00, 0x80, 0x10, 0x00, 0x7C, 0x18,
  0xEE, 0x30, 0xEE, 0x30, 0xEE, 0x60, 0xEE, 0xE0, 0x7C, 0xC0, 0x01, 0x80, 0x01, 0x80, 0x03, 0x3E, 0x03, 0x77, 0x06, 0xE7, 0x06, 0xE7, 0x0C, 0x67,
  0x18, 0x7E, 0x18, 0x3C, 0x0F, 0x80, 0x3F, 0x80, 0x77, 0x01, 0xC6, 0x01, 0x80, 0x03, 0x80, 0x0F, 0x80, 0x3F, 0x86, 0xE3, 0x9D, 0xC3, 0xF3, 0x83,
  0xE7, 0x03, 0xCF, 0x1F, 0xCF, 0xFB, 0xC7, 0xC3, 0x80, 0xFF, 0xF0, 0x0E, 0x38, 0x71, 0xC3, 0x0E, 0x1C, 0x30, 0xE1, 0xC3, 0x87, 0x0E, 0x1C, 0x38,
  0x70, 0xE0, 0xC1, 0xC3, 0x83, 0x87, 0x07, 0x07, 0xE0, 0x70, 0x38, 0x18, 0x1C, 0x0C, 0x0E, 0x0E, 0x06, 0x06, 0x07, 0x07, 0x07, 0x07, 0x06, 0x06,
  0x0E, 0x0E, 0x0E, 0x1C, 0x1C, 0x38, 0x70, 0x70, 0x19, 0x27, 0x5B, 0xE7, 0xDA, 0xE4, 0x8C, 0x0E, 0x01, 0xC0, 0x38, 0x07, 0x00, 0xE1, 0xFF, 0xFF,
  0xF8, 0x70, 0x0E, 0x01, 0xC0, 0x38, 0x00, 0x77, 0x66, 0xCC, 0xFF, 0xFF, 0xC0, 0xFF, 0x80, 0x03, 0x81, 0x80, 0xC0, 0xE0, 0x70, 0x30, 0x18, 0x1C,
  0x0E, 0x06, 0x03, 0x03, 0x81, 0xC0, 0xE0, 0x60, 0x70, 0x38, 0x1C, 0x0C, 0x0E, 0x07, 0x03, 0x81, 0x81, 0xC0, 0x1F, 0x03, 0xFC, 0x71, 0xE6, 0x0E,
  0xE0, 0x6E, 0x06, 0xE0, 0x7E, 0x07, 0xE0, 0x7E, 0x07, 0xE0, 0x6E, 0x06, 0x60, 0xE7, 0x1C, 0x3F, 0xC1, 0xF0, 0xFF, 0xCE, 0x73, 0x9C, 0xE7, 0x39,
  0xCE, 0x73, 0x9C, 0xE7, 0x1F, 0x0F, 0xF1, 0xC7, 0x70, 0x6E, 0x0E, 0x01, 0xC0, 0x30, 0x0E, 0x03, 0x80, 0xF0, 0x3C, 0x0F, 0x03, 0xC0, 0xF0, 0x3F,
  0xFF, 0xFF, 0x1F, 0x0F, 0xF9, 0xC7, 0x70, 0x70, 0x0E, 0x01, 0x80, 0xF0, 0x7C, 0x0F, 0x80, 0x38, 0x03, 0xE0, 0x7E, 0x0F, 0xE3, 0xDF, 0xF0, 0xFC,
  0x01, 0xE0, 0x3E, 0x03, 0xE0, 0x7E, 0x0E, 0xE0, 0xCE, 0x1C, 0xE3, 0x8E, 0x30, 0xE7, 0x0E, 0xE0, 0xEF, 0xFF, 0xFF, 0xF0, 0x0E, 0x00, 0xE0, 0x0E,
  0xFF, 0xDF, 0xFB, 0x80, 0x70, 0x0E, 0x01, 0xDE, 0x3F, 0xF7, 0x8E, 0xE0, 0xE0, 0x1C, 0x03, 0x80, 0x7E, 0x0F, 0xE3, 0x9F, 0xE1, 0xF8, 0x1F, 0x07,
  0xF1, 0xC7, 0x30, 0x6E, 0x01, 0xC0, 0x3B, 0xE7, 0xFE, 0xF1, 0xFC, 0x1F, 0x83, 0xF0, 0x76, 0x0E, 0xE3, 0x8F, 0xF0, 0xF8, 0xFF, 0xFF, 0xFC, 0x03,
  0x80, 0x60, 0x1C, 0x03, 0x00, 0xE0, 0x1C, 0x07, 0x00, 0xE0, 0x38, 0x07, 0x00, 0xC0, 0x38, 0x06, 0x01, 0xC0, 0x1F, 0x83, 0xFC, 0x71, 0xEE, 0x0E,
  0xE0, 0xE6, 0x0E, 0x71, 0xC3, 0xF8, 0x7F, 0xC7, 0x0E, 0xE0, 0x6E, 0x07, 0xE0, 0x6F, 0x1E, 0x7F, 0xC1, 0xF8, 0x1F, 0x0F, 0xF1, 0xC7, 0x70, 0x6E,
  0x0F, 0xC1, 0xF8, 0x3F, 0x8F, 0x7F, 0xE7, 0xEC, 0x01, 0x80, 0x76, 0x0E, 0xE3, 0x8F, 0xE0, 0xF8, 0xFF, 0x80, 0x00, 0x1F, 0xF0, 0x77, 0x70, 0x00,
  0x00, 0x07, 0x76, 0x6C, 0xC0, 0x0E, 0x38, 0xE3, 0x8E, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF,
  0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE1, 0xC7, 0x1C, 0x71, 0xC0, 0x3F, 0x1F, 0xEF, 0x3F, 0x87, 0x01, 0xC0, 0x70, 0x3C, 0xFE, 0x3F, 0x0E, 0x01, 0x80,
  0x00, 0x38, 0x0E, 0x03, 0x80, 0x01, 0xFE, 0x00, 0x7F, 0xF8, 0x0F, 0x03, 0xC1, 0xC0, 0x1E, 0x38, 0xFE, 0xE7, 0x3F, 0xC6, 0x77, 0x9C, 0x7E, 0x70,
  0xC7, 0xE6, 0x0C, 0x6E, 0xE0, 0xC6, 0xEE, 0x1C, 0xEE, 0x73, 0xCC, 0xE7, 0xEF, 0xCE, 0x3C, 0x70, 0x70, 0x00, 0x03, 0xC1, 0x00, 0x1F, 0xF0, 0x00,
  0xFF, 0x00, 0x07, 0x00, 0x3C, 0x03, 0xE0, 0x1F, 0x00, 0xDC, 0x0E, 0xE0, 0x63, 0x87, 0x1C, 0x38, 0x61, 0x83, 0x9F, 0xFC, 0xFF, 0xE6, 0x03, 0xF0,
  0x1F, 0x80, 0x60, 0xFF, 0x9F, 0xFB, 0x87, 0xF0, 0x7E, 0x0F, 0xC1, 0xFF, 0xF7, 0xFC, 0xE1, 0xDC, 0x1F, 0x83, 0xF0, 0x7E, 0x1F, 0xFF, 0xBF, 0xE0,
  0x07, 0xE0, 0x7F, 0xE3, 0xC7, 0xDC, 0x07, 0x60, 0x03, 0x80, 0x0E, 0x00, 0x38, 0x00, 0xE0, 0x03, 0x80, 0x06, 0x00, 0x1C, 0x07, 0x3C, 0x3C, 0x7F,
  0xE0, 0x7E, 0x00, 0xFF, 0x07, 0xFE, 0x38, 0x79, 0xC0, 0xEE, 0x03, 0x70, 0x1F, 0x80, 0xFC, 0x07, 0xE0, 0x3F, 0x01, 0xF8, 0x0D, 0xC0, 0xEE, 0x1E,
  0x7F, 0xE3, 0xFC, 0x00, 0xFF, 0xFF, 0xF8, 0x1C, 0x0E, 0x07, 0x03, 0xFD, 0xFE, 0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0xFF, 0xFE, 0xFF, 0xFF, 0xF8,
  0x1C, 0x0E, 0x07, 0x03, 0xFD, 0xFE, 0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0x80, 0x07, 0xE0, 0x3F, 0xE0, 0xF1, 0xE3, 0x80, 0xE6, 0x00, 0x1C,
  0x00, 0x38, 0x00, 0x70, 0xFF, 0xE1, 0xFF, 0xC0, 0x1D, 0x80, 0x3B, 0x80, 0xE3, 0xC3, 0x83, 0xFE, 0x01, 0xF8, 0x00, 0xE0, 0x7E, 0x07, 0xE0, 0x7E,
  0x07, 0xE0, 0x7E, 0x07, 0xFF, 0xFF, 0xFF, 0xE0, 0x7E, 0x07, 0xE0, 0x7E, 0x07, 0xE0, 0x7E, 0x07, 0xE0, 0x70, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8,
  0x03, 0x81, 0xC0, 0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x07, 0x03, 0x81, 0xC0, 0xFC, 0x7E, 0x3B, 0xF8, 0xF8, 0xE0, 0xFC, 0x3B, 0x8E, 0x73, 0xCE, 0x71,
  0xDC, 0x3F, 0x07, 0xC0, 0xFC, 0x1D, 0xC3, 0xBC, 0x73, 0xCE, 0x3D, 0xC3, 0xB8, 0x38, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0,
  0xE0, 0xE0, 0xE0, 0xFF, 0xFF, 0xE0, 0x07, 0xE0, 0x07, 0xF0, 0x0F, 0xF0, 0x0F, 0xF8, 0x1F, 0xF8, 0x1F, 0xFC, 0x3F, 0xEC, 0x37, 0xEE, 0x77, 0xE6,
  0x67, 0xE7, 0x67, 0xE3, 0xC7, 0xE3, 0xC7, 0xE3, 0xC7, 0xE1, 0x87, 0xE0, 0x3F, 0x81, 0xFC, 0x0F, 0xF0, 0x7F, 0xC3, 0xF6, 0x1F, 0xB8, 0xFC, 0xE7,
  0xE3, 0x3F, 0x1D, 0xF8, 0x7F, 0xC1, 0xFE, 0x0F, 0xF0, 0x3F, 0x80, 0xE0, 0x07, 0xE0, 0x1F, 0xF8, 0x3C, 0x3C, 0x70, 0x1C, 0x60, 0x0E, 0xE0, 0x06,
  0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x06, 0x60, 0x0E, 0x70, 0x1C, 0x3C, 0x3C, 0x1F, 0xF8, 0x07, 0xE0, 0xFF, 0x1F, 0xF3, 0x87, 0x70, 0x6E,
  0x0F, 0xC1, 0xB8, 0x77, 0xFC, 0xFF, 0x1C, 0x03, 0x80, 0x70, 0x0E, 0x01, 0xC0, 0x38, 0x00, 0x07, 0xE0, 0x1F, 0xF8, 0x3C, 0x3C, 0x70, 0x1C, 0x60,
  0x0E, 0xE0, 0x06, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x06, 0xE0, 0x06, 0x60, 0x0E, 0x70, 0x1C, 0x3C, 0x3C, 0x1F, 0xF8, 0x07, 0xF8, 0x00, 0x1C, 0x00,
  0x0E, 0x00, 0x06, 0xFF, 0x1F, 0xF3, 0x87, 0x70, 0x6E, 0x0F, 0xC1, 0xB8, 0x77, 0xFC, 0xFF, 0x1C, 0xE3, 0x9C, 0x71, 0xCE, 0x39, 0xC3, 0xB8, 0x78,
  0x3F, 0x0F, 0xF3, 0xCF, 0x70, 0xEE, 0x01, 0xC0, 0x1F, 0x01, 0xFC, 0x07, 0xC0, 0x38, 0x03, 0xF0, 0x6F, 0x1C, 0xFF, 0x07, 0xC0, 0xFF, 0xFF, 0xFC,
  0x38, 0x07, 0x00, 0xE0, 0x1C, 0x03, 0x80, 0x70, 0x0E, 0x01, 0xC0, 0x38, 0x07, 0x00, 0xE0, 0x1C, 0x03, 0x80, 0xE0, 0x7E, 0x07, 0xE0, 0x7E, 0x07,
  0xE0, 0x7E, 0x07, 0xE0, 0x7E, 0x07, 0xE0, 0x7E, 0x07, 0xE0, 0x7E, 0x07, 0x70, 0xE3, 0xFC, 0x1F, 0x80, 0xC0, 0x1F, 0x80, 0x6E, 0x03, 0x98, 0x0E,
  0x70, 0x71, 0xC1, 0xC3, 0x86, 0x0E, 0x38, 0x18, 0xE0, 0x73, 0x01, 0xDC, 0x03, 0x70, 0x0F, 0x80, 0x1E, 0x00, 0x70, 0x00, 0xE0, 0x78, 0x1B, 0x83,
  0xC1, 0xDC, 0x1E, 0x0E, 0xE0, 0xF0, 0x73, 0x0F, 0xC3, 0x1C, 0x6E, 0x38, 0xE3, 0x31, 0xC3, 0x19, 0xCC, 0x19, 0xCE, 0x60, 0xEC, 0x37, 0x07, 0x61,
  0xB0, 0x1F, 0x0F, 0x80, 0xF0, 0x7C, 0x07, 0x81, 0xE0, 0x1C, 0x0E, 0x00, 0xE0, 0x77, 0x07, 0x38, 0xE3, 0x9C, 0x1D, 0xC0, 0xF8, 0x0F, 0x00, 0x70,
  0x0F, 0x00, 0xF8, 0x1D, 0xC3, 0x9C, 0x30, 0xE7, 0x07, 0xE0, 0x70, 0xE0, 0x37, 0x07, 0x70, 0x63, 0x0E, 0x38, 0xC1, 0x9C, 0x1D, 0x80, 0xF8, 0x0F,
  0x00, 0x70, 0x07, 0x00, 0x70, 0x07, 0x00, 0x70, 0x07, 0x00, 0xFF, 0xFF, 0xF0, 0x1C, 0x0E, 0x07, 0x01, 0xC0, 0xE0, 0x30, 0x1C, 0x0E, 0x03, 0x81,
  0xC0, 0xE0, 0x3F, 0xFF, 0xFC, 0xFF, 0xF9, 0xCE, 0x73, 0x9C, 0xE7, 0x39, 0xCE, 0x73, 0x9C, 0xE7, 0x39, 0xCE, 0x73, 0xFF, 0xC0, 0x70, 0x38, 0x1C,
  0x06, 0x03, 0x81, 0xC0, 0x60, 0x30, 0x1C, 0x0E, 0x03, 0x01, 0xC0, 0xE0, 0x30, 0x18, 0x0E, 0x07, 0x01, 0x80, 0xE0, 0x70, 0x38, 0x0C, 0x07, 0xFF,
  0xCE, 0x73, 0x9C, 0xE7, 0x39, 0xCE, 0x73, 0x9C, 0xE7, 0x39, 0xCE, 0x73, 0x9F, 0xFF, 0x06, 0x00, 0xF0, 0x0F, 0x00, 0xF8, 0x1D, 0x81, 0x9C, 0x39,
  0xC3, 0x0E, 0x70, 0xE7, 0x07, 0xE0, 0x7E, 0x03, 0xFF, 0xFF, 0xFF, 0x87, 0x1E, 0x30, 0x1F, 0x39, 0xFF, 0xDE, 0x3F, 0xC0, 0xFE, 0x03, 0xF0, 0x1F,
  0x80, 0xFC, 0x07, 0xE0, 0x7B, 0xC7, 0xCF, 0xFE, 0x3E, 0x70, 0xE0, 0x07, 0x00, 0x38, 0x01, 0xC0, 0x0E, 0xF8, 0x7F, 0xF3, 0xE3, 0xDC, 0x0E, 0xE0,
  0x37, 0x01, 0xF8, 0x0F, 0xC0, 0x6E, 0x07, 0x7C, 0x7B, 0xFF, 0x9D, 0xF0, 0x1F, 0x87, 0xF9, 0xC7, 0xF0, 0x7E, 0x01, 0xC0, 0x38, 0x07, 0x00, 0xE0,
  0xEE, 0x3C, 0xFF, 0x0F, 0xC0, 0x00, 0x38, 0x01, 0xC0, 0x0E, 0x00, 0x71, 0xF3, 0x9F, 0xFD, 0xE3, 0xFC, 0x0F, 0xE0, 0x3F, 0x01, 0xF8, 0x0F, 0xC0,
  0x7E, 0x07, 0xBC, 0x7C, 0xFF, 0xE3, 0xE7, 0x0F, 0x81, 0xFF, 0x1E, 0x3C, 0xE0, 0xE6, 0x03, 0x7F, 0xFF, 0xFF, 0xEC, 0x00, 0x70, 0x33, 0xC7, 0x8F,
  0xF8, 0x1F, 0x00, 0x04, 0xF3, 0xDC, 0x73, 0xFF, 0xDC, 0x71, 0xC7, 0x1C, 0x71, 0xC7, 0x1C, 0x70, 0x1F, 0x39, 0xFF, 0xDE, 0x3F, 0xC0, 0xFE, 0x03,
  0xF0, 0x1F, 0x80, 0xFC, 0x07, 0xE0, 0x7B, 0xC7, 0xCF, 0xFE, 0x3E, 0x70, 0x03, 0x80, 0x19, 0xC1, 0xCF, 0x1E, 0x3F, 0xE0, 0x7C, 0x00, 0xE0, 0x1C,
  0x03, 0x80, 0x70, 0x0E, 0xF9, 0xFF, 0xBC, 0x7F, 0x07, 0xE0, 0xFC, 0x1F, 0x83, 0xF0, 0x7E, 0x0F, 0xC1, 0xF8, 0x3F, 0x07, 0x5F, 0xA1, 0xFF, 0xFF,
  0xFF, 0xFF, 0xE0, 0x11, 0xCE, 0x20, 0x1C, 0xE7, 0x39, 0xCE, 0x73, 0x9C, 0xE7, 0x39, 0xCE, 0x73, 0xFF, 0xC0, 0xE0, 0x38, 0x0E, 0x03, 0x80, 0xE3,
  0xF8, 0xEE, 0x73, 0xB8, 0xFC, 0x3E, 0x0F, 0x83, 0xF0, 0xEE, 0x39, 0xCE, 0x3B, 0x87, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEF, 0x0F, 0x8F, 0xFD,
  0xFC, 0xF1, 0xF9, 0xEE, 0x0F, 0x0E, 0xE0, 0xE0, 0x6E, 0x0E, 0x07, 0xE0, 0xE0, 0x7E, 0x0E, 0x07, 0xE0, 0xE0, 0x7E, 0x0E, 0x07, 0xE0, 0xE0, 0x7E,
  0x0E, 0x07, 0xEF, 0x9F, 0xFB, 0xC7, 0xF0, 0x7E, 0x0F, 0xC1, 0xF8, 0x3F, 0x07, 0xE0, 0xFC, 0x1F, 0x83, 0xF0, 0x70, 0x0F, 0x81, 0xFF, 0x1E, 0x39,
  0xC0, 0xEE, 0x03, 0x70, 0x1F, 0x80, 0xFC, 0x06, 0xE0, 0x73, 0xC7, 0x0F, 0xF8, 0x3F, 0x00, 0xEF, 0x87, 0xFF, 0x3E, 0x3D, 0xC0, 0xEE, 0x03, 0x70,
  0x1F, 0x80, 0xFC, 0x06, 0xE0, 0x77, 0xC7, 0xBF, 0xF9, 0xDF, 0x0E, 0x00, 0x70, 0x03, 0x80, 0x1C, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x39, 0xFF,
  0xDE, 0x3F, 0xC0, 0xFE, 0x03, 0xF0, 0x1F, 0x80, 0xFC, 0x07, 0xE0, 0x7B, 0xC7, 0xCF, 0xFE, 0x3E, 0x70, 0x03, 0x80, 0x1C, 0x00, 0xE0, 0x07, 0x00,
  0x38, 0x01, 0xC0, 0xEF, 0xFF, 0x38, 0xE3, 0x8E, 0x38, 0xE3, 0x8E, 0x38, 0x3E, 0x1F, 0xEE, 0x3B, 0x86, 0xF0, 0x1F, 0x81, 0xF8, 0x0E, 0xE1, 0xFC,
  0xE7, 0xF8, 0xFC, 0x70, 0xE1, 0xC7, 0xFF, 0xEE, 0x1C, 0x38, 0x70, 0xE1, 0xC3, 0x83, 0x07, 0xC7, 0x80, 0xE0, 0xFC, 0x1F, 0x83, 0xF0, 0x7E, 0x0F,
  0xC1, 0xF8, 0x3F, 0x07, 0xE0, 0xEE, 0x3D, 0xFF, 0x9F, 0x70, 0xE0, 0x37, 0x07, 0x70, 0x77, 0x0E, 0x38, 0xE3, 0x8C, 0x19, 0xC1, 0xDC, 0x1D, 0x80,
  0xF8, 0x0F, 0x00, 0x70, 0xE0, 0xE1, 0xD8, 0x78, 0x76, 0x1E, 0x19, 0xC7, 0x8E, 0x31, 0xF3, 0x8C, 0xEC, 0xC3, 0xB3, 0x30, 0x6C, 0xDC, 0x1F, 0x3E,
  0x07, 0x87, 0x81, 0xE1, 0xE0, 0x38, 0x70, 0xE0, 0xEE, 0x38, 0xE6, 0x1D, 0xC1, 0xF0, 0x1C, 0x03, 0x80, 0xF8, 0x3B, 0x86, 0x71, 0xC7, 0x70, 0x70,
  0xE0, 0x37, 0x07, 0x70, 0x73, 0x06, 0x38, 0xE3, 0x8C, 0x19, 0xC1, 0xDC, 0x0D, 0x80, 0xF8, 0x0F, 0x00, 0x70, 0x06, 0x00, 0xE0, 0x0E, 0x01, 0xC0,
  0x1C, 0x01, 0x80, 0xFF, 0xFF, 0x07, 0x0E, 0x0C, 0x1C, 0x38, 0x30, 0x70, 0xE0, 0xFF, 0xFF, 0x1E, 0x7C, 0xE1, 0xC3, 0x87, 0x0E, 0x1C, 0x38, 0x71,
  0xE7, 0x8F, 0x07, 0x0E, 0x1C, 0x38, 0x70, 0xE1, 0xC3, 0x87, 0x0F, 0x8F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF1, 0xF0, 0xE1, 0xC3,
  0x87, 0x0E, 0x1C, 0x38, 0x70, 0xE0, 0xF1, 0xE7, 0x0E, 0x1C, 0x38, 0x70, 0xE1, 0xC3, 0x87, 0x3C, 0x70, 0x7C, 0xFF, 0xFC, 0x78, 0x3E, 0x3B, 0xB8,
  0xDC, 0x7E, 0x33, 0x38, 0xF8,
};
const GFXglyph F_MID22_Glyphs[] PROGMEM = {
  {    0,   0,   0,   6,    0,    0},   // ' '
  {    0,   3,  15,   7,    2,  -15},   // '!'
  {    6,   6,   4,   7,    1,  -17},   // '"'
  {    9,  17,  16,  19,    1,  -16},   // '#'
  {   43,  11,  19,  14,    1,  -17},   // '$'
  {   70,  16,  15,  17,    1,  -15},   // '%'
  {  100,  15,  15,  17,    1,  -15},   // '&'
  {  129,   3,   4,   4,    1,  -17},   // '''
  {  131,   7,  24,  11,    3,  -20},   // '('
  {  152,   8,  24,  11,    1,  -20},   // ')'
  {  176,   7,   8,  11,    2,  -16},   // '*'
  {  183,  11,  11,  16,    2,  -13},   // '+'
  {  199,   4,   6,   5,    0,   -3},   // ','
  {  202,   9,   2,  13,    2,   -9},   // '-'
  {  205,   3,   3,   5,    1,   -3},   // '.'
  {  207,   9,  24,  11,    1,  -20},   // '/'
  {  234,  12,  16,  14,    1,  -16},   // '0'
  {  258,   5,  16,   8,    1,  -16},   // '1'
  {  268,  11,  16,  13,    1,  -16},   // '2'
  {  290,  11,  16,  13,    1,  -16},   // '3'
  {  312,  12,  16,  14,    1,  -16},   // '4'
  {  336,  11,  16,  14,    2,  -16},   // '5'
  {  358,  11,  16,  14,    2,  -16},   // '6'
  {  380,  11,  16,  12,    1,  -16},   // '7'
  {  402,  12,  16,  14,    1,  -16},   // '8'
  {  426,  11,  16,  14,    2,  -16},   // '9'
  {  448,   3,  12,   5,    1,  -12},   // ':'
  {  453,   4,  15,   7,    1,  -12},   // ';'
  {  461,   7,  10,  13,    3,  -13},   // '<'
  {  470,  12,   7,  17,    3,  -11},   // '='
  {  481,   7,  10,  13,    3,  -13},   // '>'
  {  490,  10,  15,  12,    1,  -15},   // '?'
  {  509,  20,  18,  23,    2,  -14},   // '@'
  {  554,  13,  15,  15,    1,  -15},   // 'A'
  {  579,  11,  15,  14,    2,  -15},   // 'B'
  {  600,  14,  15,  17,    1,  -15},   // 'C'
  {  627,  13,  15,  16,    2,  -15},   // 'D'
  {  652,   9,  15,  12,    2,  -15},   // 'E'
  {  669,   9,  15,  11,    2,  -15},   // 'F'
  {  686,  15,  15,  17,    1,  -15},   // 'G'
  {  715,  12,  15,  16,    2,  -15},   // 'H'
  {  738,   3,  15,   6,    2,  -15},   // 'I'
  {  744,   9,  15,  12,    1,  -15},   // 'J'
  {  761,  11,  15,  14,    2,  -15},   // 'K'
  {  782,   8,  15,  10,    2,  -15},   // 'L'
  {  797,  16,  15,  19,    2,  -15},   // 'M'
  {  827,  13,  15,  16,    2,  -15},   // 'N'
  {  852,  16,  15,  17,    1,  -15},   // 'O'
  {  882,  11,  15,  13,    2,  -15},   // 'P'
  {  903,  16,  18,  17,    1,  -15},   // 'Q'
  {  939,  11,  15,  14,    2,  -15},   // 'R'
  {  960,  11,  15,  13,    1,  -15},   // 'S'
  {  981,  11,  15,  12,    1,  -15},   // 'T'
  { 1002,  12,  15,  15,    2,  -15},   // 'U'
  { 1025,  14,  15,  15,    1,  -15},   // 'V'
  { 1052,  21,  15,  22,    0,  -15},   // 'W'
  { 1092,  12,  15,  15,    1,  -15},   // 'X'
  { 1115,  12,  15,  13,    0,  -15},   // 'Y'
  { 1138,  10,  15,  12,    1,  -15},   // 'Z'
  { 1157,   5,  24,  11,    3,  -20},   // '['
  { 1172,   9,  24,  16,    4,  -20},   // '?'
  { 1199,   5,  24,  11,    3,  -20},   // ']'
  { 1214,  12,  12,  14,    1,  -16},   // '^'
  { 1232,  12,   2,  18,    3,    1},   // '_'
  { 1235,   5,   4,   6,    1,  -17},   // '`'
  { 1238,  13,  12,  15,    1,  -12},   // 'a'
  { 1258,  13,  16,  15,    2,  -16},   // 'b'
  { 1284,  11,  12,  13,    1,  -12},   // 'c'
  { 1301,  13,  16,  15,    1,  -16},   // 'd'
  { 1327,  13,  12,  14,    0,  -12},   // 'e'
  { 1347,   6,  17,   7,    1,  -17},   // 'f'
  { 1360,  13,  18,  15,    1,  -12},   // 'g'
  { 1390,  11,  16,  14,    2,  -16},   // 'h'
  { 1412,   3,  17,   6,    1,  -17},   // 'i'
  { 1419,   5,  23,   6,    0,  -17},   // 'j'
  { 1434,  10,  16,  12,    2,  -16},   // 'k'
  { 1454,   3,  16,   6,    2,  -16},   // 'l'
  { 1460,  20,  12,  23,    2,  -12},   // 'm'
  { 1490,  11,  12,  14,    2,  -12},   // 'n'
  { 1507,  13,  12,  14,    1,  -12},   // 'o'
  { 1527,  13,  18,  15,    2,  -12},   // 'p'
  { 1557,  13,  18,  15,    1,  -12},   // 'q'
  { 1587,   6,  12,   8,    2,  -12},   // 'r'
  { 1596,  10,  12,  12,    1,  -12},   // 's'
  { 1611,   7,  15,   8,    1,  -15},   // 't'
  { 1625,  11,  12,  14,    2,  -12},   // 'u'
  { 1642,  12,  12,  13,    0,  -12},   // 'v'
  { 1660,  18,  12,  18,    0,  -12},   // 'w'
  { 1687,  11,  12,  11,    0,  -12},   // 'x'
  { 1704,  12,  18,  13,    0,  -12},   // 'y'
  { 1731,   8,  12,  10,    1,  -12},   // 'z'
  { 1743,   7,  24,  11,    3,  -20},   // '{'
  { 1764,   3,  20,   7,    2,  -18},   // '|'
  { 1772,   7,  24,  11,    2,  -20},   // '}'
  { 1793,  10,   3,  12,    1,   -8},   // '~'
  { 1797,   9,   7,   9,    1,  -15},   // 'deg'
};
const GFXfont F_MID22 PROGMEM = {(uint8_t *)F_MID22_Bitmaps, (GFXglyph *)F_MID22_Glyphs, 0x20, 0x7F, 32};

// F_MID22B: Poppins-Bold.ttf 22px, 96 glyphs, 2119 bytes
const uint8_t F_MID22B_Bitmaps[] PROGMEM = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x06, 0xFF, 0x60, 0xEF, 0xDF, 0xBF, 0x7E, 0xE0, 0x03, 0xC7, 0x00, 0x70, 0xE0, 0x0E, 0x1C, 0x03, 0xE7, 0x83,
  0xFF, 0xFE, 0x7F, 0xFF, 0xCF, 0xFF, 0xF8, 0x38, 0x70, 0x07, 0x1E, 0x01, 0xE3, 0xC3, 0xFF, 0xFE, 0x7F, 0xFF, 0xCF, 0xFF, 0xF8, 0x3C, 0x78, 0x07,
  0x8F, 0x00, 0xF1, 0xC0, 0x1E, 0x38, 0x00, 0x02, 0x00, 0x20, 0x1F, 0x87, 0xFE, 0xFF, 0xEF, 0x3F, 0xF2, 0xFF, 0xA0, 0xFE, 0x07, 0xFC, 0x1F, 0xE0,
  0x3F, 0x02, 0xFF, 0x2F, 0xFA, 0xF7, 0xFF, 0x7F, 0xE1, 0xF8, 0x02, 0x00, 0x20, 0x3C, 0x1E, 0x1F, 0x8F, 0x0E, 0x73, 0xC3, 0x9D, 0xE0, 0xE7, 0x78,
  0x1F, 0xBC, 0x03, 0xCF, 0x00, 0x07, 0x80, 0x01, 0xEF, 0x00, 0xF7, 0xE0, 0x3F, 0xFC, 0x1E, 0xE7, 0x07, 0xB9, 0xC3, 0xCF, 0xF0, 0xF1, 0xF8, 0x78,
  0x3C, 0x0F, 0x80, 0x1F, 0xC0, 0x3F, 0xE0, 0x3F, 0xE0, 0x3C, 0xE0, 0x3C, 0x00, 0x1E, 0x00, 0x3F, 0x00, 0x7F, 0x9E, 0xFB, 0xFE, 0xF1, 0xFC, 0xF0,
  0xF8, 0xFF, 0xFC, 0xFF, 0xFE, 0x7F, 0xFE, 0x1F, 0x8F, 0xFF, 0xFE, 0x0F, 0x1F, 0x1E, 0x3C, 0x3C, 0x78, 0x78, 0x78, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
  0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0x78, 0x78, 0x78, 0x3C, 0x3C, 0x1E, 0x1F, 0x0F, 0xE0, 0xF0, 0x78, 0x3C, 0x3C, 0x3E, 0x1E, 0x1E, 0x1E, 0x0F, 0x0F,
  0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x1E, 0x1E, 0x1E, 0x3E, 0x3C, 0x3C, 0x78, 0xF0, 0xF0, 0x18, 0x1A, 0xFF, 0xFF, 0x3C, 0xFF, 0xFF, 0x5A, 0x18,
  0x0F, 0x01, 0xE0, 0x3C, 0x07, 0x80, 0xF1, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x01, 0xE0, 0x3C, 0x07, 0x80, 0x7F, 0xFE, 0xEE, 0xC0, 0xFF, 0xFF, 0xFF,
  0xFC, 0x6F, 0xF6, 0x03, 0xC0, 0xE0, 0x78, 0x1E, 0x07, 0x81, 0xE0, 0x70, 0x3C, 0x0F, 0x03, 0xC0, 0xE0, 0x78, 0x1E, 0x07, 0x81, 0xE0, 0x70, 0x3C,
  0x0F, 0x03, 0xC0, 0xE0, 0x78, 0x1E, 0x07, 0x81, 0xC0, 0xF0, 0x3C, 0x00, 0x0F, 0x81, 0xFF, 0x1F, 0xF8, 0xFF, 0xEF, 0x8F, 0x78, 0x3F, 0xC1, 0xFE,
  0x0F, 0xF0, 0x7F, 0x83, 0xFC, 0x1F, 0xE0, 0xFF, 0x8F, 0x3F, 0xF9, 0xFF, 0x87, 0xFC, 0x0F, 0x80, 0xFF, 0xFF, 0xCF, 0x3C, 0xF3, 0xCF, 0x3C, 0xF3,
  0xCF, 0x3C, 0xF3, 0xCF, 0x1F, 0x07, 0xF9, 0xFF, 0x7F, 0xFF, 0x1F, 0xE3, 0xC0, 0x78, 0x1E, 0x03, 0xC0, 0xF0, 0x3C, 0x0F, 0x83, 0xE1, 0xF8, 0x3F,
  0xFF, 0xFF, 0xFF, 0xE0, 0x3F, 0x8F, 0xFB, 0xFF, 0xFC, 0xFF, 0x1E, 0x03, 0xC1, 0xF0, 0x7C, 0x0F, 0x81, 0xF8, 0x0F, 0x80, 0xFF, 0x1F, 0xFF, 0xDF,
  0xFB, 0xFE, 0x1F, 0x00, 0x03, 0xE0, 0x1F, 0x01, 0xF8, 0x1F, 0xC0, 0xFE, 0x0E, 0xF0, 0xF7, 0x87, 0x3C, 0x79, 0xE7, 0xCF, 0x3F, 0xFF, 0xFF, 0xFF,
  0xFF, 0x80, 0xF0, 0x07, 0x80, 0x3C, 0xFF, 0xEF, 0xFE, 0xFF, 0xEF, 0x00, 0xF0, 0x0F, 0x00, 0xFF, 0xCF, 0xFE, 0xFF, 0xEF, 0x9F, 0x00, 0xF0, 0x0F,
  0xF0, 0xFF, 0xFF, 0x7F, 0xE7, 0xFC, 0x1F, 0x80, 0x1F, 0x83, 0xFC, 0x7F, 0xE7, 0x9E, 0xF0, 0xEF, 0x00, 0xF7, 0x8F, 0xFE, 0xFF, 0xEF, 0xFF, 0xF0,
  0xFF, 0x0F, 0xF0, 0xF7, 0xFF, 0x7F, 0xE3, 0xFC, 0x1F, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0x81, 0xF0, 0x1C, 0x07, 0x80, 0xF0, 0x3C, 0x07, 0x81, 0xE0,
  0x3C, 0x07, 0x81, 0xE0, 0x3C, 0x0F, 0x01, 0xE0, 0x7C, 0x00, 0x3F, 0xC7, 0xFE, 0x7F, 0xEF, 0x9F, 0xF0, 0xFF, 0x0F, 0x79, 0xE3, 0xFC, 0x3F, 0xC7,
  0xFE, 0xF0, 0xFF, 0x0F, 0xF0, 0xFF, 0xFF, 0x7F, 0xE3, 0xFC, 0x1F, 0x80, 0x3F, 0x87, 0xFC, 0x7F, 0xEF, 0x9E, 0xF0, 0xFF, 0x0F, 0xF0, 0xFF, 0xFF,
  0x7F, 0xF7, 0xFF, 0x1E, 0xF0, 0x0F, 0x70, 0xE7, 0xFE, 0x7F, 0xE3, 0xFC, 0x1F, 0x80, 0x6F, 0xF6, 0x00, 0x00, 0x06, 0xFF, 0x60, 0x6F, 0xF6, 0x00,
  0x00, 0x07, 0xFE, 0xEE, 0xCC, 0x1F, 0x3E, 0x7C, 0x78, 0xF0, 0xF0, 0x78, 0x7C, 0x3E, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00,
  0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF8, 0x7C, 0x3C, 0x3E, 0x1F, 0x1F, 0x3E, 0x3C, 0x7C, 0xF8, 0x3F, 0x8F, 0xFB, 0xFF, 0x79, 0xFF, 0x1E, 0x03, 0xC0,
  0xF9, 0xFE, 0x3F, 0x87, 0x80, 0xF0, 0x00, 0x00, 0x00, 0x30, 0x0F, 0x01, 0xE0, 0x18, 0x00, 0x00, 0x7E, 0x00, 0x1F, 0xFC, 0x01, 0xFF, 0xF8, 0x1F,
  0x07, 0xC1, 0xE0, 0x0F, 0x1E, 0x00, 0x39, 0xE7, 0xFC, 0xEE, 0x7F, 0xE7, 0x77, 0xFF, 0x3F, 0x3C, 0xF1, 0xFB, 0xC3, 0x8F, 0xDE, 0x1C, 0xEE, 0xF1,
  0xE7, 0x77, 0xFF, 0xFB, 0x9F, 0xFF, 0x9E, 0x7C, 0xF0, 0x70, 0x00, 0x03, 0xE0, 0x00, 0x0F, 0xFE, 0x00, 0x3F, 0xF0, 0x00, 0x7F, 0x00, 0x00, 0x03,
  0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x0E, 0x78, 0x1E, 0x78, 0x1E, 0x78, 0x1C, 0x3C, 0x3F, 0xFC, 0x3F, 0xFC, 0x3F,
  0xFE, 0x7C, 0x1E, 0x78, 0x1E, 0xF8, 0x0F, 0xFF, 0xCF, 0xFE, 0xFF, 0xFF, 0x3F, 0xF0, 0xFF, 0x0F, 0xFF, 0xEF, 0xFC, 0xFF, 0xEF, 0x3F, 0xF0, 0xFF,
  0x0F, 0xF3, 0xFF, 0xFF, 0xFF, 0xEF, 0xFC, 0x07, 0xE0, 0x3F, 0xF0, 0xFF, 0xF3, 0xFF, 0xE7, 0x83, 0xFE, 0x03, 0xFC, 0x00, 0x78, 0x00, 0xF0, 0x01,
  0xE0, 0x03, 0xC0, 0x7B, 0xC1, 0xF7, 0xFF, 0xC7, 0xFF, 0x87, 0xFE, 0x03, 0xF0, 0xFF, 0x83, 0xFF, 0x8F, 0xFF, 0x3C, 0xFE, 0xF0, 0x7B, 0xC1, 0xFF,
  0x03, 0xFC, 0x0F, 0xF0, 0x3F, 0xC0, 0xFF, 0x07, 0xFC, 0x1E, 0xF3, 0xFB, 0xFF, 0xCF, 0xFE, 0x3F, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xF0, 0x3C,
  0x0F, 0xFB, 0xFE, 0xFF, 0xBC, 0x0F, 0x03, 0xC0, 0xF0, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xF0, 0x3C, 0x0F, 0xFB, 0xFE, 0xFF,
  0xBC, 0x0F, 0x03, 0xC0, 0xF0, 0x3C, 0x0F, 0x03, 0xC0, 0x07, 0xE0, 0x3F, 0xF0, 0xFF, 0xF3, 0xFF, 0xE7, 0x83, 0xFE, 0x00, 0x3C, 0x00, 0x78, 0xFF,
  0xF1, 0xFF, 0xE3, 0xFF, 0xC0, 0x7B, 0xC0, 0xF7, 0xFF, 0xC7, 0xFF, 0x07, 0xFC, 0x03, 0xF0, 0xF0, 0x3F, 0xC0, 0xFF, 0x03, 0xFC, 0x0F, 0xF0, 0x3F,
  0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xFF, 0x03, 0xFC, 0x0F, 0xF0, 0x3F, 0xC0, 0xFF, 0x03, 0xFC, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0x01, 0xE0, 0x3C, 0x07, 0x80, 0xF0, 0x1E, 0x03, 0xC0, 0x78, 0x0F, 0x01, 0xE0, 0x3C, 0x07, 0xF8, 0xFF, 0xFE, 0xFF, 0x9F, 0xF0,
  0xF8, 0xF0, 0x7F, 0x87, 0xBC, 0x7D, 0xE7, 0xCF, 0x3C, 0x7B, 0xC3, 0xFE, 0x1F, 0xE0, 0xFF, 0x07, 0xFC, 0x3D, 0xF1, 0xE7, 0x8F, 0x3E, 0x78, 0xFB,
  0xC3, 0xFE, 0x0F, 0xF0, 0x78, 0x3C, 0x1E, 0x0F, 0x07, 0x83, 0xC1, 0xE0, 0xF0, 0x78, 0x3C, 0x1E, 0x0F, 0x07, 0xFF, 0xFF, 0xFF, 0xF8, 0x07, 0xFE,
  0x01, 0xFF, 0xC0, 0xFF, 0xF0, 0x3F, 0xFC, 0x1F, 0xFF, 0x87, 0xFF, 0xE1, 0xFF, 0xFC, 0xFF, 0xF7, 0x3B, 0xFD, 0xCE, 0xFF, 0x7F, 0xBF, 0xCF, 0xCF,
  0xF3, 0xF3, 0xFC, 0xF8, 0xFF, 0x1E, 0x3F, 0xC7, 0x8F, 0xF0, 0x3F, 0xE0, 0xFF, 0x83, 0xFF, 0x0F, 0xFE, 0x3F, 0xF8, 0xFF, 0xF3, 0xFD, 0xEF, 0xF7,
  0xBF, 0xCF, 0xFF, 0x3F, 0xFC, 0x7F, 0xF0, 0xFF, 0xC3, 0xFF, 0x07, 0xFC, 0x0F, 0x07, 0xE0, 0x1F, 0xF8, 0x3F, 0xFC, 0x7F, 0xFE, 0x78, 0x1E, 0xF0,
  0x1F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x1F, 0x78, 0x1E, 0x7F, 0xFE, 0x3F, 0xFC, 0x1F, 0xF8, 0x07, 0xE0, 0xFF, 0x8F, 0xFC,
  0xFF, 0xEF, 0x3F, 0xF0, 0xFF, 0x0F, 0xF1, 0xFF, 0xFE, 0xFF, 0xEF, 0xFC, 0xF0, 0x0F, 0x00, 0xF0, 0x0F, 0x00, 0xF0, 0x0F, 0x00, 0x07, 0xE0, 0x1F,
  0xF8, 0x3F, 0xFC, 0x7F, 0xFE, 0x78, 0x1E, 0xF0, 0x1F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x1E, 0x78, 0x1E, 0x7F, 0xFE, 0x3F,
  0xFC, 0x1F, 0xF8, 0x07, 0xFC, 0x00, 0x3E, 0x00, 0x1E, 0x00, 0x0F, 0xFF, 0x8F, 0xFE, 0xFF, 0xEF, 0x3F, 0xF0, 0xFF, 0x0F, 0xF0, 0xFF, 0xFE, 0xFF,
  0xEF, 0xF8, 0xF7, 0x8F, 0x7C, 0xF3, 0xCF, 0x3E, 0xF1, 0xEF, 0x0F, 0x1F, 0x07, 0xFC, 0xFF, 0xCF, 0xFE, 0xF1, 0xEF, 0x00, 0xFC, 0x07, 0xF8, 0x1F,
  0xE0, 0x3E, 0x00, 0xFF, 0x0F, 0xFF, 0xF7, 0xFE, 0x3F, 0xC1, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF0, 0x0F, 0x00, 0xF0, 0x0F, 0x00, 0xF0, 0x0F,
  0x00, 0xF0, 0x0F, 0x00, 0xF0, 0x0F, 0x00, 0xF0, 0x0F, 0x00, 0xF0, 0xF0, 0x7F, 0x83, 0xFC, 0x1F, 0xE0, 0xFF, 0x07, 0xF8, 0x3F, 0xC1, 0xFE, 0x0F,
  0xF0, 0x7F, 0x83, 0xFC, 0x1F, 0xE0, 0xF7, 0xFF, 0x3F, 0xF8, 0xFF, 0x81, 0xF0, 0xF0, 0x0F, 0x78, 0x1E, 0x78, 0x1E, 0x78, 0x1E, 0x3C, 0x3C, 0x3C,
  0x3C, 0x3C, 0x3C, 0x1E, 0x78, 0x1E, 0x78, 0x1E, 0x78, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0xF0, 0xF8, 0x7F,
  0xC3, 0xE1, 0xEF, 0x0F, 0x87, 0xBC, 0x3F, 0x1E, 0x79, 0xFC, 0x79, 0xE7, 0xF3, 0xC7, 0x9F, 0xCF, 0x1E, 0x77, 0x3C, 0x3B, 0xDE, 0xF0, 0xEF, 0x7B,
  0x83, 0xF8, 0xFE, 0x0F, 0xE3, 0xF8, 0x1F, 0x8F, 0xE0, 0x7E, 0x3F, 0x01, 0xF0, 0x7C, 0x07, 0xC1, 0xF0, 0xF0, 0x7F, 0xE1, 0xE7, 0xCF, 0x8F, 0x3C,
  0x3F, 0xE0, 0x7F, 0x80, 0xFC, 0x03, 0xF0, 0x0F, 0x80, 0x3F, 0x01, 0xFE, 0x0F, 0xF8, 0x3C, 0xF1, 0xE3, 0xEF, 0x87, 0xBC, 0x1F, 0xF0, 0x3F, 0xE1,
  0xE7, 0x87, 0x9F, 0x3C, 0x3C, 0xF0, 0xF7, 0x81, 0xFE, 0x07, 0xF8, 0x0F, 0xC0, 0x3F, 0x00, 0x78, 0x01, 0xE0, 0x07, 0x80, 0x1E, 0x00, 0x78, 0x01,
  0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0x81, 0xF0, 0x3C, 0x0F, 0x03, 0xE0, 0x78, 0x1E, 0x07, 0xC0, 0xF0, 0x3C, 0x0F, 0x81, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0x8F, 0x1E, 0x3C, 0x78, 0xF1, 0xE3, 0xC7, 0x8F, 0x1E, 0x3C, 0x78, 0xF1, 0xE3, 0xC7, 0x8F, 0x1E, 0x3C, 0x7F, 0xFF, 0xFC, 0xF0, 0x3C,
  0x07, 0x01, 0xE0, 0x78, 0x1E, 0x07, 0x80, 0xE0, 0x3C, 0x0F, 0x03, 0xC0, 0x70, 0x1E, 0x07, 0x81, 0xE0, 0x78, 0x0E, 0x03, 0xC0, 0xF0, 0x3C, 0x07,
  0x01, 0xE0, 0x78, 0x1E, 0x03, 0x80, 0xF0, 0xFF, 0xFF, 0xF8, 0xF1, 0xE3, 0xC7, 0x8F, 0x1E, 0x3C, 0x78, 0xF1, 0xE3, 0xC7, 0x8F, 0x1E, 0x3C, 0x78,
  0xF1, 0xE3, 0xC7, 0xFF, 0xFF, 0xFC, 0x07, 0x00, 0x3E, 0x00, 0xFC, 0x03, 0xF0, 0x1F, 0xC0, 0x7F, 0x83, 0xDE, 0x0F, 0x3C, 0x38, 0xF1, 0xE1, 0xE7,
  0x87, 0xBC, 0x1E, 0xF0, 0x3C, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x87, 0x3E, 0xF1, 0x80, 0x1F, 0x79, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0x8F, 0xF8,
  0x3F, 0xC1, 0xFE, 0x0F, 0xF8, 0xFB, 0xFF, 0xDF, 0xFE, 0x7F, 0xF1, 0xF7, 0x80, 0xF0, 0x07, 0x80, 0x3C, 0x01, 0xE0, 0x0F, 0x7C, 0x7F, 0xF3, 0xFF,
  0xDF, 0xFE, 0xF8, 0xFF, 0x83, 0xFC, 0x1F, 0xE0, 0xFF, 0x8F, 0xFF, 0xFB, 0xFF, 0xDF, 0xFC, 0xF7, 0xC0, 0x0F, 0x83, 0xFC, 0x7F, 0xE7, 0xFF, 0xF0,
  0xFF, 0x00, 0xF0, 0x0F, 0x00, 0xF0, 0xF7, 0xFF, 0x7F, 0xE3, 0xFC, 0x0F, 0x80, 0x00, 0x78, 0x03, 0xC0, 0x1E, 0x00, 0xF1, 0xF7, 0x9F, 0xFD, 0xFF,
  0xFF, 0xFF, 0xF8, 0xFF, 0x83, 0xFC, 0x1F, 0xE0, 0xFF, 0x8F, 0xBF, 0xFD, 0xFF, 0xE7, 0xFF, 0x1F, 0x78, 0x0F, 0x81, 0xFF, 0x0F, 0xFC, 0xFF, 0xE7,
  0x87, 0xBF, 0xFF, 0xFF, 0xEF, 0x00, 0x78, 0x73, 0xFF, 0x8F, 0xFC, 0x3F, 0xC0, 0xF8, 0x00, 0x1E, 0x7C, 0xFB, 0xF7, 0x9F, 0xFF, 0xFF, 0x78, 0xF1,
  0xE3, 0xC7, 0x8F, 0x1E, 0x3C, 0x78, 0xF0, 0x1F, 0x79, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0x8F, 0xF8, 0x3F, 0xC1, 0xFE, 0x0F, 0xF8, 0xFB, 0xFF, 0xDF,
  0xFE, 0x7F, 0xF1, 0xF7, 0x80, 0x3D, 0xC1, 0xEF, 0xFF, 0x7F, 0xF1, 0xFF, 0x03, 0xF0, 0xF0, 0x0F, 0x00, 0xF0, 0x0F, 0x00, 0xF7, 0xCF, 0xFE, 0xFF,
  0xEF, 0xFF, 0xF0, 0xFF, 0x0F, 0xF0, 0xFF, 0x0F, 0xF0, 0xFF, 0x0F, 0xF0, 0xFF, 0x0F, 0xF0, 0xF0, 0x6F, 0xF6, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0x18, 0xF3, 0xC6, 0x00, 0xF3, 0xCF, 0x3C, 0xF3, 0xCF, 0x3C, 0xF3, 0xCF, 0x3C, 0xF3, 0xCF, 0x7F, 0xFF, 0xBC, 0xF0, 0x0F, 0x00, 0xF0, 0x0F,
  0x00, 0xF1, 0xFF, 0x3E, 0xF3, 0xCF, 0x7C, 0xFF, 0x8F, 0xF0, 0xFE, 0x0F, 0xF0, 0xFF, 0x8F, 0x7C, 0xF3, 0xCF, 0x3E, 0xF1, 0xF0, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF7, 0xC7, 0xCF, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF0, 0xFF, 0x0F, 0x0F, 0xF0, 0xF0, 0xFF,
  0x0F, 0x0F, 0xF0, 0xF0, 0xFF, 0x0F, 0x0F, 0xF0, 0xF0, 0xFF, 0x0F, 0x0F, 0xF0, 0xF0, 0xF0, 0xF7, 0xCF, 0xFE, 0xFF, 0xEF, 0xFF, 0xF0, 0xFF, 0x0F,
  0xF0, 0xFF, 0x0F, 0xF0, 0xFF, 0x0F, 0xF0, 0xFF, 0x0F, 0xF0, 0xF0, 0x0F, 0x81, 0xFF, 0x1F, 0xFC, 0xFF, 0xEF, 0x0F, 0xF8, 0x3F, 0xC1, 0xFE, 0x0F,
  0xF0, 0xFB, 0xFF, 0x9F, 0xFC, 0x7F, 0xC0, 0xF8, 0x00, 0xF7, 0xC7, 0xFF, 0x3F, 0xFD, 0xFF, 0xEF, 0x8F, 0xF8, 0x3F, 0xC1, 0xFE, 0x0F, 0xF8, 0xFF,
  0xFF, 0xBF, 0xFD, 0xFF, 0xCF, 0x7C, 0x78, 0x03, 0xC0, 0x1E, 0x00, 0xF0, 0x07, 0x80, 0x3C, 0x00, 0x1F, 0x79, 0xFF, 0xDF, 0xFE, 0xFF, 0xFF, 0x8F,
  0xF8, 0x3F, 0xC1, 0xFE, 0x0F, 0xF8, 0xFB, 0xFF, 0xDF, 0xFE, 0x7F, 0xF1, 0xF7, 0x80, 0x3C, 0x01, 0xE0, 0x0F, 0x00, 0x78, 0x03, 0xC0, 0x1E, 0xF3,
  0xFF, 0xFF, 0xFF, 0xF8, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0x3E, 0x1F, 0xEF, 0xFB, 0xFF, 0xF3, 0xFF, 0x03, 0xF0, 0x3E, 0xF3, 0xFF,
  0xFF, 0xFD, 0xFE, 0x1F, 0x00, 0x78, 0x78, 0x78, 0xFF, 0xFF, 0xFF, 0x78, 0x78, 0x78, 0x78, 0x78, 0x78, 0x7C, 0x7F, 0x3F, 0x1F, 0xF0, 0xFF, 0x0F,
  0xF0, 0xFF, 0x0F, 0xF0, 0xFF, 0x0F, 0xF0, 0xFF, 0x0F, 0xF0, 0xFF, 0xFF, 0x7F, 0xF7, 0xFF, 0x3E, 0xF0, 0xF0, 0x7B, 0xC3, 0xDE, 0x1E, 0xF1, 0xE3,
  0xCF, 0x1E, 0x78, 0xF3, 0x83, 0xBC, 0x1F, 0xE0, 0xFE, 0x03, 0xF0, 0x1F, 0x80, 0xF8, 0x00, 0xF1, 0xF1, 0xFE, 0x3E, 0x3D, 0xC7, 0xC7, 0x3C, 0xFC,
  0xE7, 0x9F, 0x9C, 0xF7, 0xF7, 0x8E, 0xEE, 0xE1, 0xDD, 0xDC, 0x3F, 0xBF, 0x87, 0xE3, 0xF0, 0x7C, 0x7C, 0x0F, 0x8F, 0x81, 0xF1, 0xF0, 0xF8, 0xF3,
  0xC7, 0x8F, 0x78, 0x7F, 0xC1, 0xFC, 0x07, 0xC0, 0x3E, 0x03, 0xF0, 0x1F, 0xC1, 0xFF, 0x0F, 0x78, 0xF1, 0xEF, 0x8F, 0x80, 0xF0, 0x3D, 0xE1, 0xE7,
  0x87, 0x9E, 0x1E, 0x3C, 0xF0, 0xF3, 0xC1, 0xFE, 0x07, 0xF8, 0x1F, 0xE0, 0x3F, 0x00, 0xFC, 0x01, 0xE0, 0x07, 0x80, 0x3E, 0x00, 0xF0, 0x03, 0xC0,
  0x1E, 0x00, 0x78, 0x03, 0xC0, 0x00, 0xFF, 0xFF, 0xFF, 0xFC, 0x1F, 0x0F, 0x83, 0xC1, 0xE0, 0xF0, 0x78, 0x3E, 0x0F, 0xFF, 0xFF, 0xFF, 0xC0, 0x0F,
  0x1F, 0x3F, 0x3E, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x7C, 0xF8, 0xF0, 0xFC, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3E, 0x3F, 0x1F,
  0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF8, 0xFC, 0x7C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3E,
  0x1F, 0x0F, 0x1F, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C, 0x7C, 0xFC, 0xF8, 0xF0, 0x3C, 0x77, 0xFF, 0xFF, 0xFF, 0xFE, 0xE3, 0xC0, 0x3E, 0x3F,
  0xBD, 0xFC, 0x7E, 0x3F, 0xBD, 0xFC, 0x7C,
};
const GFXglyph F_MID22B_Glyphs[] PROGMEM = {
  {    0,   0,   0,   5,    0,    0},   // ' '
  {    0,   4,  17,   9,    2,  -17},   // '!'
  {    9,   7,   5,   9,    1,  -18},   // '"'
  {   14,  19,  17,  20,    1,  -17},   // '#'
  {   55,  12,  20,  14,    1,  -18},   // '$'
  {   85,  18,  16,  19,    1,  -16},   // '%'
  {  121,  16,  16,  18,    1,  -16},   // '&'
  {  153,   3,   5,   5,    1,  -18},   // '''
  {  155,   8,  26,  10,    2,  -21},   // '('
  {  181,   8,  26,  10,    1,  -21},   // ')'
  {  207,   8,   9,  12,    2,  -17},   // '*'
  {  216,  11,  12,  14,    1,  -14},   // '+'
  {  233,   4,   7,   6,    1,   -4},   // ','
  {  237,  10,   3,  13,    1,   -9},   // '-'
  {  241,   4,   4,   6,    1,   -4},   // '.'
  {  243,  10,  26,  10,    0,  -22},   // '/'
  {  276,  13,  17,  14,    1,  -17},   // '0'
  {  304,   6,  16,   8,    1,  -16},   // '1'
  {  316,  11,  17,  13,    1,  -17},   // '2'
  {  340,  11,  17,  13,    1,  -17},   // '3'
  {  364,  13,  16,  15,    1,  -16},   // '4'
  {  390,  12,  17,  14,    2,  -17},   // '5'
  {  416,  12,  17,  14,    1,  -17},   // '6'
  {  442,  11,  17,  12,    1,  -17},   // '7'
  {  466,  12,  17,  14,    1,  -17},   // '8'
  {  492,  12,  17,  14,    1,  -17},   // '9'
  {  518,   4,  13,   6,    1,  -13},   // ':'
  {  525,   4,  16,   8,    2,  -13},   // ';'
  {  533,   8,  10,  12,    2,  -13},   // '<'
  {  543,  12,   9,  15,    1,  -13},   // '='
  {  557,   8,  10,  12,    2,  -13},   // '>'
  {  567,  11,  17,  12,    1,  -17},   // '?'
  {  591,  21,  21,  24,    2,  -16},   // '@'
  {  647,  16,  16,  16,    0,  -16},   // 'A'
  {  679,  12,  16,  14,    1,  -16},   // 'B'
  {  703,  15,  16,  17,    1,  -16},   // 'C'
  {  733,  14,  16,  16,    1,  -16},   // 'D'
  {  761,  10,  16,  12,    1,  -16},   // 'E'
  {  781,  10,  16,  12,    1,  -16},   // 'F'
  {  801,  15,  16,  17,    1,  -16},   // 'G'
  {  831,  14,  16,  16,    1,  -16},   // 'H'
  {  859,   4,  16,   6,    1,  -16},   // 'I'
  {  867,  11,  16,  13,    1,  -16},   // 'J'
  {  889,  13,  16,  15,    1,  -16},   // 'K'
  {  915,   9,  16,  10,    1,  -16},   // 'L'
  {  933,  18,  16,  20,    1,  -16},   // 'M'
  {  969,  14,  16,  17,    1,  -16},   // 'N'
  {  997,  16,  16,  17,    1,  -16},   // 'O'
  { 1029,  12,  16,  14,    1,  -16},   // 'P'
  { 1053,  16,  19,  17,    1,  -16},   // 'Q'
  { 1091,  12,  16,  14,    1,  -16},   // 'R'
  { 1115,  12,  16,  14,    1,  -16},   // 'S'
  { 1139,  12,  16,  13,    1,  -16},   // 'T'
  { 1163,  13,  16,  16,    1,  -16},   // 'U'
  { 1189,  16,  16,  16,    0,  -16},   // 'V'
  { 1221,  22,  16,  23,    1,  -16},   // 'W'
  { 1265,  14,  16,  16,    1,  -16},   // 'X'
  { 1293,  14,  16,  15,    0,  -16},   // 'Y'
  { 1321,  11,  16,  13,    1,  -16},   // 'Z'
  { 1343,   7,  26,  11,    3,  -21},   // '['
  { 1366,  10,  26,  18,    4,  -22},   // '?'
  { 1399,   7,  26,  11,    2,  -21},   // ']'
  { 1422,  14,  13,  16,    1,  -17},   // '^'
  { 1445,  14,   3,  17,    2,    2},   // '_'
  { 1451,   5,   5,   6,    1,  -19},   // '`'
  { 1455,  13,  13,  15,    1,  -13},   // 'a'
  { 1477,  13,  17,  15,    1,  -17},   // 'b'
  { 1505,  12,  13,  13,    1,  -13},   // 'c'
  { 1525,  13,  17,  15,    1,  -17},   // 'd'
  { 1553,  13,  13,  14,    0,  -13},   // 'e'
  { 1575,   7,  18,   8,    1,  -18},   // 'f'
  { 1591,  13,  19,  15,    1,  -13},   // 'g'
  { 1622,  12,  17,  15,    1,  -17},   // 'h'
  { 1648,   4,  18,   6,    1,  -18},   // 'i'
  { 1657,   6,  24,   6,   -1,  -18},   // 'j'
  { 1675,  12,  17,  14,    1,  -17},   // 'k'
  { 1701,   4,  17,   6,    1,  -17},   // 'l'
  { 1710,  20,  13,  23,    1,  -13},   // 'm'
  { 1743,  12,  13,  15,    1,  -13},   // 'n'
  { 1763,  13,  13,  14,    1,  -13},   // 'o'
  { 1785,  13,  19,  15,    1,  -13},   // 'p'
  { 1816,  13,  19,  15,    1,  -13},   // 'q'
  { 1847,   8,  13,   9,    1,  -13},   // 'r'
  { 1860,  10,  13,  12,    1,  -13},   // 's'
  { 1877,   8,  16,   9,    1,  -16},   // 't'
  { 1893,  12,  13,  15,    1,  -13},   // 'u'
  { 1913,  13,  13,  14,    0,  -13},   // 'v'
  { 1935,  19,  13,  19,    0,  -13},   // 'w'
  { 1966,  13,  13,  13,    0,  -13},   // 'x'
  { 1988,  14,  19,  14,    0,  -13},   // 'y'
  { 2022,  10,  13,  11,    1,  -13},   // 'z'
  { 2039,   8,  26,  11,    1,  -21},   // '{'
  { 2065,   4,  22,   6,    1,  -19},   // '|'
  { 2076,   8,  26,  11,    2,  -21},   // '}'
  { 2102,  12,   5,  14,    1,  -10},   // '~'
  { 2110,   9,   8,  10,    1,  -16},   // 'deg'
};
const GFXfont F_MID22B PROGMEM = {(uint8_t *)F_MID22B_Bitmaps, (GFXglyph *)F_MID22B_Glyphs, 0x20, 0x7F, 32};

// F_SM13: DejaVuSansCondensed-Bold.ttf 13px, 96 glyphs, 710 bytes
const uint8_t F_SM13_Bitmaps[] PROGMEM = {
  0xBF, 0xF8, 0xF0, 0x9F, 0xFF, 0x16, 0x36, 0xFF, 0x7F, 0x2C, 0xFE, 0xFE, 0x68, 0x48, 0x10, 0xCF, 0xFD, 0xF3, 0xF3, 0xCF, 0xFF, 0xE3, 0x04, 0x30,
  0x8F, 0x33, 0x64, 0x6D, 0x87, 0xA0, 0x49, 0xC3, 0x78, 0x49, 0x19, 0xA2, 0x3C, 0x38, 0x7C, 0x60, 0x70, 0x71, 0xFB, 0xDF, 0xCE, 0xFF, 0x7F, 0xBF,
  0x6D, 0xED, 0xB6, 0xCD, 0x90, 0xD9, 0xB6, 0xDB, 0x6F, 0x60, 0x31, 0xF3, 0xBF, 0x30, 0x18, 0x18, 0x18, 0xFF, 0xFF, 0x18, 0x18, 0x18, 0x7F, 0x80,
  0xFC, 0x7C, 0x11, 0x33, 0x26, 0x64, 0x4C, 0xC0, 0x18, 0x7E, 0x66, 0x67, 0xE7, 0xE7, 0x67, 0x66, 0x7E, 0x3C, 0x33, 0xEF, 0x8E, 0x38, 0xE3, 0x8E,
  0xFF, 0xF0, 0x73, 0xF9, 0xC3, 0x1C, 0x63, 0x18, 0xFF, 0xF0, 0x73, 0xF1, 0xC3, 0x79, 0xF0, 0xC3, 0xFF, 0xE0, 0x0C, 0x1E, 0x1E, 0x3E, 0x6E, 0x6E,
  0xFE, 0xFF, 0x0E, 0x0E, 0x7B, 0xFC, 0x30, 0xFB, 0x70, 0xC3, 0xFF, 0xE0, 0x38, 0xFB, 0x86, 0x0F, 0xDF, 0xB3, 0xE7, 0xFC, 0xF0, 0xFF, 0xF1, 0xC7,
  0x18, 0xE3, 0x0C, 0x71, 0x80, 0x7B, 0xFC, 0xF3, 0x7F, 0xFC, 0xF3, 0xFD, 0xF0, 0x33, 0xEC, 0xF3, 0xCF, 0xF7, 0xC3, 0xFF, 0xE0, 0xFC, 0x05, 0xF8,
  0xFC, 0x05, 0xF6, 0x80, 0x0F, 0x3C, 0xF0, 0xF0, 0x3C, 0x0F, 0xFF, 0x00, 0x7E, 0xFF, 0x80, 0xF0, 0x3C, 0x0F, 0x0F, 0x7C, 0xE0, 0x77, 0xE6, 0x33,
  0xB9, 0x80, 0x63, 0x00, 0x3F, 0x1C, 0x6C, 0x0A, 0x7B, 0xB2, 0x6C, 0x9B, 0x2E, 0x7E, 0xC2, 0x18, 0x43, 0xF0, 0x1C, 0x0E, 0x0F, 0x87, 0xC3, 0x63,
  0xB9, 0xFC, 0xFE, 0xE3, 0xF0, 0xC0, 0xF1, 0xFB, 0xBE, 0x7F, 0xDF, 0xF1, 0xE3, 0xFF, 0xF8, 0x0E, 0x3F, 0x71, 0x60, 0x60, 0xE0, 0x60, 0x70, 0x7F,
  0x3F, 0xF0, 0xFE, 0xFE, 0xC7, 0xC3, 0xC3, 0xC7, 0xC7, 0xFE, 0xFC, 0xFF, 0xFF, 0xF0, 0xFF, 0xFC, 0x30, 0xFF, 0xF0, 0xFF, 0xFF, 0xF0, 0xFF, 0xFC,
  0x30, 0xC3, 0x00, 0x0E, 0x1F, 0xDC, 0x6C, 0x06, 0x07, 0x3D, 0x8E, 0xE3, 0x7F, 0x9F, 0x80, 0xC2, 0xC7, 0xC7, 0xC7, 0xFF, 0xFF, 0xC7, 0xC7, 0xC7,
  0xC7, 0xFF, 0xFF, 0xF0, 0x33, 0x33, 0x33, 0x33, 0x33, 0x7F, 0xC0, 0xC7, 0xCE, 0xCC, 0xF8, 0xF0, 0xF0, 0xF8, 0xDC, 0xCE, 0xC7, 0xC3, 0x0C, 0x30,
  0xC3, 0x0C, 0x30, 0xFF, 0xF0, 0xE1, 0xB8, 0xFF, 0x3F, 0xDF, 0xFF, 0xF7, 0xFD, 0xDF, 0x37, 0xC1, 0xF0, 0x70, 0xC2, 0xE3, 0xF3, 0xF3, 0xFB, 0xDF,
  0xDF, 0xCF, 0xCF, 0xC7, 0x1E, 0x1F, 0x9C, 0xEC, 0x36, 0x1F, 0x0D, 0x86, 0xE7, 0x7F, 0x9F, 0x80, 0xF1, 0xFB, 0xBE, 0x3C, 0xFF, 0xFE, 0x60, 0xC1,
  0x80, 0x1E, 0x1F, 0x9C, 0xEC, 0x36, 0x1F, 0x0D, 0x86, 0xE7, 0x7F, 0x9F, 0x81, 0xC0, 0x60, 0xF0, 0xFC, 0xEE, 0xCE, 0xEC, 0xFC, 0xFC, 0xCE, 0xC6,
  0xC7, 0x39, 0xFB, 0x16, 0x0F, 0x8F, 0x87, 0x87, 0xFD, 0xF8, 0xFF, 0xFF, 0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xC2, 0xC7, 0xC7, 0xC7,
  0xC7, 0xC7, 0xC7, 0xE6, 0xFE, 0x7C, 0xC1, 0xF1, 0xD8, 0xCC, 0x67, 0x71, 0xB0, 0xD8, 0x7C, 0x1C, 0x0E, 0x00, 0x42, 0x1E, 0x73, 0x67, 0x36, 0x73,
  0x6D, 0x77, 0xDF, 0x7D, 0xE3, 0xDE, 0x38, 0xE3, 0x8E, 0x63, 0x31, 0x9D, 0xC7, 0xC1, 0xC0, 0xE0, 0xF8, 0x6C, 0x77, 0x71, 0xC0, 0xC3, 0xE7, 0x77,
  0x3E, 0x3C, 0x1C, 0x18, 0x18, 0x18, 0x18, 0x7F, 0x7F, 0x7F, 0x0E, 0x1C, 0x18, 0x38, 0x70, 0x7F, 0xFF, 0xFE, 0xCC, 0xCC, 0xCC, 0xCC, 0xFF, 0x8C,
  0xC4, 0x66, 0x22, 0x33, 0x10, 0xFD, 0xB6, 0xDB, 0x6F, 0xF0, 0x31, 0xEF, 0xA3, 0xFC, 0x99, 0x80, 0x7C, 0x9C, 0xFB, 0xFE, 0x7F, 0xDF, 0x80, 0xC1,
  0x83, 0x07, 0xEF, 0xF9, 0xF1, 0xE7, 0xFF, 0xF8, 0x3D, 0xF6, 0x38, 0x61, 0xF3, 0xC0, 0x06, 0x07, 0x07, 0x7F, 0x7F, 0x67, 0xE7, 0x67, 0x7F, 0x7F,
  0x3C, 0xFD, 0x9F, 0xF6, 0x0E, 0xCF, 0x80, 0x3B, 0xDD, 0xFF, 0xB1, 0x8C, 0x63, 0x00, 0x7F, 0x7F, 0x67, 0xE7, 0x67, 0x7F, 0x3E, 0x06, 0x7E, 0x3C,
  0xC1, 0x83, 0x07, 0xEF, 0xD9, 0xF3, 0xE7, 0xCF, 0x9C, 0xF3, 0xFF, 0xF0, 0x6C, 0x36, 0xDB, 0x6D, 0xFC, 0xC1, 0x83, 0x06, 0x6D, 0x9E, 0x3C, 0x7C,
  0xDD, 0x9C, 0xFF, 0xFF, 0xF0, 0xFF, 0xFF, 0xFC, 0xCF, 0x33, 0xCC, 0xF3, 0x3C, 0xCC, 0xFD, 0xFB, 0x3E, 0x7C, 0xF9, 0xF3, 0x80, 0x3C, 0x7E, 0x67,
  0xE7, 0x67, 0x7E, 0x3C, 0xFD, 0xFF, 0x3E, 0x3C, 0xFF, 0xFF, 0x60, 0xC1, 0x80, 0x7F, 0x7F, 0x67, 0xE7, 0x67, 0x7F, 0x7F, 0x07, 0x07, 0x06, 0xFF,
  0xF1, 0x8C, 0x63, 0x00, 0xFE, 0x79, 0xF1, 0xEF, 0xE0, 0x63, 0x3F, 0xF6, 0x31, 0x8F, 0x38, 0xCF, 0x3C, 0xF3, 0xCF, 0xF7, 0xC0, 0xE6, 0xCD, 0xBB,
  0xE3, 0xC7, 0x8E, 0x00, 0xEE, 0xDB, 0xB6, 0xED, 0xFF, 0x7B, 0x8E, 0xE3, 0xB8, 0x66, 0xF8, 0xF1, 0xC3, 0xCD, 0xB9, 0x80, 0xE6, 0xCD, 0xBB, 0xE3,
  0xC7, 0x86, 0x0C, 0x70, 0xE0, 0x7D, 0xF1, 0x8E, 0x73, 0xFF, 0xC0, 0x1C, 0xF3, 0x0C, 0x31, 0xCF, 0x0C, 0x30, 0xC3, 0x87, 0xFF, 0xFF, 0xFF, 0xC0,
  0xE1, 0xC3, 0x0C, 0x30, 0xF1, 0xCC, 0x30, 0xC7, 0x3C, 0xFF, 0xDE, 0x6F, 0x9F, 0x40,
};
const GFXglyph F_SM13_Glyphs[] PROGMEM = {
  {    0,   0,   0,   4,    0,    0},   // ' '
  {    0,   2,  10,   5,    2,  -10},   // '!'
  {    3,   4,   4,   6,    1,  -10},   // '"'
  {    5,   8,   9,  10,    1,   -9},   // '#'
  {   14,   6,  12,   8,    1,  -10},   // '$'
  {   23,  11,  10,  12,    0,  -10},   // '%'
  {   37,   8,  10,  10,    1,  -10},   // '&'
  {   47,   2,   4,   4,    1,  -10},   // '''
  {   48,   3,  12,   5,    1,  -10},   // '('
  {   53,   3,  12,   5,    1,  -10},   // ')'
  {   58,   6,   5,   6,    0,   -9},   // '*'
  {   62,   8,   8,  10,    1,   -8},   // '+'
  {   70,   2,   5,   4,    1,   -3},   // ','
  {   72,   3,   2,   5,    1,   -5},   // '-'
  {   73,   2,   3,   4,    1,   -3},   // '.'
  {   74,   4,  11,   4,    0,  -10},   // '/'
  {   80,   8,  10,   8,    0,  -10},   // '0'
  {   90,   6,  10,   8,    1,  -10},   // '1'
  {   98,   6,  10,   8,    1,  -10},   // '2'
  {  106,   6,  10,   8,    1,  -10},   // '3'
  {  114,   8,  10,   8,    0,  -10},   // '4'
  {  124,   6,  10,   8,    1,  -10},   // '5'
  {  132,   7,  10,   8,    1,  -10},   // '6'
  {  141,   6,  10,   8,    1,  -10},   // '7'
  {  149,   6,  10,   8,    1,  -10},   // '8'
  {  157,   6,  10,   8,    1,  -10},   // '9'
  {  165,   3,   7,   5,    1,   -7},   // ':'
  {  168,   3,   9,   5,    1,   -7},   // ';'
  {  172,   8,   6,  10,    1,   -7},   // '<'
  {  178,   8,   4,  10,    1,   -6},   // '='
  {  182,   8,   7,  10,    1,   -8},   // '>'
  {  189,   5,  10,   7,    1,  -10},   // '?'
  {  196,  10,  11,  12,    1,   -9},   // '@'
  {  210,   9,  10,   9,    0,  -10},   // 'A'
  {  222,   7,  10,   9,    1,  -10},   // 'B'
  {  231,   8,  10,   9,    0,  -10},   // 'C'
  {  241,   8,  10,  10,    1,  -10},   // 'D'
  {  251,   6,  10,   8,    1,  -10},   // 'E'
  {  259,   6,  10,   8,    1,  -10},   // 'F'
  {  267,   9,  10,  10,    0,  -10},   // 'G'
  {  279,   8,  10,  10,    1,  -10},   // 'H'
  {  289,   2,  10,   4,    1,  -10},   // 'I'
  {  292,   4,  13,   4,   -1,  -10},   // 'J'
  {  299,   8,  10,   9,    1,  -10},   // 'K'
  {  309,   6,  10,   7,    1,  -10},   // 'L'
  {  317,  10,  10,  12,    1,  -10},   // 'M'
  {  330,   8,  10,  10,    1,  -10},   // 'N'
  {  340,   9,  10,  10,    0,  -10},   // 'O'
  {  352,   7,  10,   9,    1,  -10},   // 'P'
  {  361,   9,  12,  10,    0,  -10},   // 'Q'
  {  375,   8,  10,   9,    1,  -10},   // 'R'
  {  385,   7,  10,   8,    1,  -10},   // 'S'
  {  394,   8,  10,   8,    0,  -10},   // 'T'
  {  404,   8,  10,  10,    1,  -10},   // 'U'
  {  414,   9,  10,   9,    0,  -10},   // 'V'
  {  426,  12,  10,  13,    0,  -10},   // 'W'
  {  441,   9,  10,   9,    0,  -10},   // 'X'
  {  453,   8,  10,   8,    0,  -10},   // 'Y'
  {  463,   8,  10,   8,    0,  -10},   // 'Z'
  {  473,   4,  12,   5,    1,  -10},   // '['
  {  479,   4,  11,   4,    0,  -10},   // '?'
  {  485,   3,  12,   5,    1,  -10},   // ']'
  {  490,   6,   4,  10,    2,  -10},   // '^'
  {  493,   6,   1,   6,    0,    2},   // '_'
  {  494,   3,   3,   6,    1,  -11},   // '`'
  {  496,   7,   7,   8,    0,   -7},   // 'a'
  {  503,   7,  10,   8,    1,  -10},   // 'b'
  {  512,   6,   7,   7,    0,   -7},   // 'c'
  {  518,   8,  10,   8,    0,  -10},   // 'd'
  {  528,   7,   7,   8,    0,   -7},   // 'e'
  {  535,   5,  10,   5,    0,  -10},   // 'f'
  {  542,   8,  10,   8,    0,   -7},   // 'g'
  {  552,   7,  10,   8,    1,  -10},   // 'h'
  {  561,   2,  10,   4,    1,  -10},   // 'i'
  {  564,   3,  13,   4,    0,  -10},   // 'j'
  {  569,   7,  10,   8,    1,  -10},   // 'k'
  {  578,   2,  10,   4,    1,  -10},   // 'l'
  {  581,  10,   7,  12,    1,   -7},   // 'm'
  {  590,   7,   7,   8,    1,   -7},   // 'n'
  {  597,   8,   7,   8,    0,   -7},   // 'o'
  {  604,   7,  10,   8,    1,   -7},   // 'p'
  {  613,   8,  10,   8,    0,   -7},   // 'q'
  {  623,   5,   7,   6,    1,   -7},   // 'r'
  {  628,   5,   7,   7,    1,   -7},   // 's'
  {  633,   5,   9,   6,    0,   -9},   // 't'
  {  639,   6,   7,   8,    1,   -7},   // 'u'
  {  645,   7,   7,   8,    0,   -7},   // 'v'
  {  652,  10,   7,  11,    0,   -7},   // 'w'
  {  661,   7,   7,   8,    0,   -7},   // 'x'
  {  668,   7,  10,   8,    0,   -7},   // 'y'
  {  677,   6,   7,   7,    0,   -7},   // 'z'
  {  683,   6,  12,   8,    1,  -10},   // '{'
  {  692,   2,  13,   4,    1,  -10},   // '|'
  {  696,   6,  12,   8,    1,  -10},   // '}'
  {  705,   8,   2,  10,    1,   -5},   // '~'
  {  707,   4,   5,   6,    1,  -10},   // 'deg'
};
const GFXfont F_SM13 PROGMEM = {(uint8_t *)F_SM13_Bitmaps, (GFXglyph *)F_SM13_Glyphs, 0x20, 0x7F, 17};

// F_SM13R: DejaVuSansCondensed.ttf 13px, 96 glyphs, 625 bytes
const uint8_t F_SM13R_Bitmaps[] PROGMEM = {
  0xFE, 0xC0, 0x96, 0xD0, 0x16, 0x14, 0x7F, 0x34, 0x24, 0xFE, 0xFE, 0x68, 0x48, 0x20, 0x8F, 0xA8, 0xA1, 0xC3, 0x8B, 0xAB, 0xE2, 0x08, 0x60, 0x28,
  0x8B, 0x62, 0xD0, 0xEC, 0x12, 0xE1, 0x28, 0x5B, 0x22, 0x88, 0xE0, 0x30, 0xF1, 0x02, 0x06, 0x1E, 0x66, 0xC7, 0xCE, 0xF4, 0xF0, 0x6B, 0x69, 0x24,
  0xC9, 0x00, 0x99, 0x26, 0xDB, 0x4B, 0x40, 0x23, 0xCD, 0xF2, 0x00, 0x18, 0x30, 0x67, 0xFF, 0xE3, 0x06, 0x0C, 0xFA, 0xF0, 0xB0, 0x11, 0x32, 0x26,
  0x44, 0x4C, 0x80, 0x33, 0xEC, 0xA3, 0x8E, 0x38, 0xE3, 0xC9, 0xE0, 0x23, 0xC3, 0x0C, 0x30, 0xC3, 0x0C, 0x33, 0xF0, 0x77, 0xC2, 0x11, 0x88, 0xCC,
  0xC7, 0xC0, 0x73, 0xE0, 0x82, 0x78, 0xE0, 0xC3, 0x0B, 0xE0, 0x10, 0xE3, 0x9E, 0x5A, 0x69, 0xBF, 0x18, 0x60, 0xFF, 0xE1, 0x8F, 0x84, 0x21, 0x1F,
  0x80, 0x39, 0xEC, 0x20, 0xFB, 0x38, 0xE3, 0xCD, 0xE0, 0xFF, 0xC2, 0x31, 0x08, 0xC4, 0x23, 0x00, 0x73, 0xE8, 0xE2, 0x7B, 0xE8, 0xE3, 0xCD, 0xE0,
  0x73, 0xE8, 0xA3, 0x8F, 0x77, 0xC2, 0x1B, 0xC0, 0xF0, 0x3C, 0xF0, 0x3E, 0x80, 0x07, 0x3C, 0xE0, 0xE0, 0x3C, 0x07, 0xFF, 0x00, 0x00, 0xFF, 0xE0,
  0x3C, 0x07, 0x0E, 0x38, 0xE0, 0x6F, 0x11, 0x36, 0x60, 0x46, 0x3F, 0x18, 0x6C, 0x0A, 0x79, 0xB2, 0x6C, 0x9B, 0x2E, 0x7E, 0xC8, 0x18, 0x43, 0xF0,
  0x18, 0x18, 0x18, 0x3C, 0x24, 0x66, 0x7E, 0x7E, 0xC3, 0xC3, 0xF3, 0xF8, 0xE3, 0xFB, 0xF8, 0x61, 0x8F, 0xE0, 0x38, 0xFF, 0x04, 0x08, 0x10, 0x20,
  0x40, 0xC2, 0xF8, 0xF1, 0xFA, 0x1C, 0x18, 0x30, 0x60, 0xC3, 0x8F, 0xF0, 0xFB, 0xE8, 0x20, 0xFB, 0xE8, 0x20, 0xC3, 0xF0, 0xFF, 0xE1, 0x0F, 0xFA,
  0x10, 0x84, 0x00, 0x3C, 0xFF, 0x04, 0x08, 0x11, 0xE0, 0xC1, 0xC2, 0xFC, 0x81, 0x0E, 0x1C, 0x3F, 0xFF, 0xE1, 0xC3, 0x87, 0x0C, 0xFF, 0xC0, 0x24,
  0x92, 0x49, 0x24, 0xBC, 0x85, 0x1A, 0x65, 0x8E, 0x1C, 0x2C, 0x4C, 0x8D, 0x0C, 0x82, 0x08, 0x20, 0x82, 0x08, 0x20, 0xC3, 0xF0, 0xC3, 0xC3, 0xE7,
  0xE7, 0xA5, 0xBD, 0x99, 0x99, 0x81, 0x81, 0xC1, 0x8F, 0x9F, 0x3B, 0x72, 0xE7, 0xC7, 0x8F, 0x0C, 0x38, 0x7C, 0xC6, 0x82, 0x83, 0x83, 0x83, 0x82,
  0xC6, 0x7C, 0xF3, 0xE8, 0xE3, 0x8F, 0xEC, 0x20, 0x82, 0x00, 0x38, 0x7C, 0xC6, 0x82, 0x83, 0x83, 0x83, 0x82, 0xC6, 0x7C, 0x0C, 0x04, 0xF1, 0xF2,
  0x34, 0x68, 0xDF, 0x22, 0x46, 0x85, 0x0C, 0x7B, 0xE8, 0x20, 0xE1, 0xE0, 0xC3, 0x8F, 0xE0, 0xFF, 0xFC, 0x40, 0x81, 0x02, 0x04, 0x08, 0x10, 0x20,
  0x81, 0x0E, 0x1C, 0x38, 0x70, 0xE1, 0xC3, 0xC4, 0xF8, 0x81, 0xC3, 0x42, 0x42, 0x66, 0x24, 0x24, 0x3C, 0x18, 0x18, 0x04, 0x38, 0xC5, 0x19, 0xA7,
  0x36, 0xA4, 0xD6, 0x8A, 0x51, 0xCE, 0x31, 0xC6, 0x30, 0x42, 0x66, 0x24, 0x3C, 0x18, 0x18, 0x3C, 0x24, 0x66, 0xC3, 0x83, 0x8D, 0x91, 0x63, 0x82,
  0x04, 0x08, 0x10, 0x20, 0x7E, 0x7E, 0x06, 0x0C, 0x08, 0x18, 0x30, 0x60, 0x60, 0xFF, 0xF2, 0x49, 0x24, 0x92, 0x60, 0x88, 0xC4, 0x44, 0x22, 0x23,
  0x10, 0xED, 0xB6, 0xDB, 0x6D, 0xF0, 0x21, 0xCC, 0xA1, 0xFC, 0xB0, 0xF0, 0x4F, 0xF8, 0xCF, 0xE0, 0x82, 0x08, 0x3E, 0xCE, 0x38, 0x63, 0xCB, 0xE0,
  0x7E, 0x21, 0x08, 0x61, 0xE0, 0x08, 0x30, 0xFF, 0x8E, 0x38, 0xE3, 0x8D, 0xF0, 0x7A, 0x28, 0xFF, 0x83, 0x07, 0x80, 0x36, 0x6F, 0x44, 0x44, 0x44,
  0xFE, 0x38, 0xE3, 0x8F, 0x77, 0x82, 0xD9, 0xC0, 0x82, 0x08, 0x3E, 0xCA, 0x38, 0xE3, 0x8E, 0x30, 0xDF, 0xC0, 0x51, 0x55, 0x57, 0x80, 0x84, 0x21,
  0x3B, 0x73, 0x96, 0x94, 0x40, 0xFF, 0xC0, 0xFF, 0xB3, 0x28, 0x8E, 0x23, 0x88, 0xE2, 0x38, 0x8C, 0xFB, 0x28, 0xE3, 0x8E, 0x38, 0xC0, 0x7A, 0x28,
  0xE3, 0x8E, 0x27, 0x80, 0xFB, 0x38, 0xE1, 0x8F, 0x2F, 0xA0, 0x82, 0x00, 0xFE, 0x38, 0xE3, 0x8E, 0x37, 0xC3, 0x0C, 0x20, 0xFC, 0x88, 0x88, 0x80,
  0xF4, 0x31, 0xE1, 0x8F, 0xC0, 0x44, 0xF4, 0x44, 0x44, 0x30, 0x8E, 0x38, 0xE3, 0x8F, 0x37, 0xC0, 0xC6, 0x89, 0x33, 0x62, 0x87, 0x0C, 0x00, 0xCC,
  0xA6, 0xD7, 0x6E, 0xA7, 0x71, 0x98, 0xCC, 0x8E, 0x9C, 0xC7, 0x6E, 0x20, 0xC6, 0x89, 0xB1, 0x42, 0x87, 0x04, 0x08, 0x30, 0xC0, 0x7C, 0x21, 0x8C,
  0x61, 0x0F, 0xC0, 0x36, 0x44, 0x4C, 0xC4, 0x44, 0x63, 0xFF, 0xFF, 0xFF, 0xC0, 0xE1, 0x08, 0x42, 0x1C, 0x64, 0x21, 0x09, 0xC0, 0x7B, 0x8E, 0x6B,
  0x9F,
};
const GFXglyph F_SM13R_Glyphs[] PROGMEM = {
  {    0,   0,   0,   4,    0,    0},   // ' '
  {    0,   1,  10,   5,    2,  -10},   // '!'
  {    2,   3,   4,   5,    1,  -10},   // '"'
  {    4,   8,   9,  10,    1,   -9},   // '#'
  {   13,   6,  12,   7,    1,  -10},   // '$'
  {   22,  10,  10,  11,    1,  -10},   // '%'
  {   35,   7,  10,   9,    1,  -10},   // '&'
  {   44,   1,   4,   3,    1,  -10},   // '''
  {   45,   3,  11,   5,    1,  -10},   // '('
  {   50,   3,  12,   5,    1,  -10},   // ')'
  {   55,   5,   5,   6,    0,   -9},   // '*'
  {   59,   7,   8,  10,    1,   -8},   // '+'
  {   66,   2,   4,   4,    1,   -2},   // ','
  {   67,   4,   1,   4,    0,   -4},   // '-'
  {   68,   2,   2,   4,    1,   -2},   // '.'
  {   69,   4,  11,   4,    0,  -10},   // '/'
  {   75,   6,  10,   7,    1,  -10},   // '0'
  {   83,   6,  10,   7,    1,  -10},   // '1'
  {   91,   5,  10,   7,    1,  -10},   // '2'
  {   98,   6,  10,   7,    1,  -10},   // '3'
  {  106,   6,  10,   7,    1,  -10},   // '4'
  {  114,   5,  10,   7,    1,  -10},   // '5'
  {  121,   6,  10,   7,    1,  -10},   // '6'
  {  129,   5,  10,   7,    1,  -10},   // '7'
  {  136,   6,  10,   7,    1,  -10},   // '8'
  {  144,   6,  10,   7,    1,  -10},   // '9'
  {  152,   2,   7,   4,    1,   -7},   // ':'
  {  154,   2,   9,   4,    1,   -7},   // ';'
  {  157,   8,   6,  10,    1,   -7},   // '<'
  {  163,   8,   4,  10,    1,   -6},   // '='
  {  167,   8,   6,  10,    1,   -7},   // '>'
  {  173,   4,  10,   6,    1,  -10},   // '?'
  {  178,  10,  11,  12,    1,   -9},   // '@'
  {  192,   8,  10,   8,    0,  -10},   // 'A'
  {  202,   6,  10,   8,    1,  -10},   // 'B'
  {  210,   7,  10,   8,    1,  -10},   // 'C'
  {  219,   7,  10,   9,    1,  -10},   // 'D'
  {  228,   6,  10,   7,    1,  -10},   // 'E'
  {  236,   5,  10,   7,    1,  -10},   // 'F'
  {  243,   7,  10,   9,    1,  -10},   // 'G'
  {  252,   7,  10,   9,    1,  -10},   // 'H'
  {  261,   1,  10,   3,    1,  -10},   // 'I'
  {  263,   3,  13,   3,   -1,  -10},   // 'J'
  {  268,   7,  10,   8,    1,  -10},   // 'K'
  {  277,   6,  10,   7,    1,  -10},   // 'L'
  {  285,   8,  10,  10,    1,  -10},   // 'M'
  {  295,   7,  10,   9,    1,  -10},   // 'N'
  {  304,   8,  10,   9,    1,  -10},   // 'O'
  {  314,   6,  10,   7,    1,  -10},   // 'P'
  {  322,   8,  12,   9,    1,  -10},   // 'Q'
  {  334,   7,  10,   8,    1,  -10},   // 'R'
  {  343,   6,  10,   7,    1,  -10},   // 'S'
  {  351,   7,  10,   7,    0,  -10},   // 'T'
  {  360,   7,  10,   9,    1,  -10},   // 'U'
  {  369,   8,  10,   8,    0,  -10},   // 'V'
  {  379,  11,  10,  12,    0,  -10},   // 'W'
  {  393,   8,  10,   8,    0,  -10},   // 'X'
  {  403,   7,  10,   7,    0,  -10},   // 'Y'
  {  412,   8,  10,   8,    0,  -10},   // 'Z'
  {  422,   3,  12,   5,    1,  -10},   // '['
  {  427,   4,  11,   4,    0,  -10},   // '?'
  {  433,   3,  12,   5,    1,  -10},   // ']'
  {  438,   6,   4,  10,    2,  -10},   // '^'
  {  441,   6,   1,   6,    0,    2},   // '_'
  {  442,   2,   2,   6,    2,  -10},   // '`'
  {  443,   5,   7,   7,    1,   -7},   // 'a'
  {  448,   6,  10,   7,    1,  -10},   // 'b'
  {  456,   5,   7,   6,    1,   -7},   // 'c'
  {  461,   6,  10,   7,    1,  -10},   // 'd'
  {  469,   6,   7,   7,    1,   -7},   // 'e'
  {  475,   4,  10,   4,    0,  -10},   // 'f'
  {  480,   6,  10,   7,    1,   -7},   // 'g'
  {  488,   6,  10,   7,    1,  -10},   // 'h'
  {  496,   1,  10,   3,    1,  -10},   // 'i'
  {  498,   2,  13,   3,    0,  -10},   // 'j'
  {  502,   5,  10,   7,    1,  -10},   // 'k'
  {  509,   1,  10,   3,    1,  -10},   // 'l'
  {  511,  10,   7,  11,    1,   -7},   // 'm'
  {  520,   6,   7,   7,    1,   -7},   // 'n'
  {  526,   6,   7,   7,    1,   -7},   // 'o'
  {  532,   6,  10,   7,    1,   -7},   // 'p'
  {  540,   6,  10,   7,    1,   -7},   // 'q'
  {  548,   4,   7,   5,    1,   -7},   // 'r'
  {  552,   5,   7,   6,    1,   -7},   // 's'
  {  557,   4,   9,   5,    0,   -9},   // 't'
  {  562,   6,   7,   7,    1,   -7},   // 'u'
  {  568,   7,   7,   7,    0,   -7},   // 'v'
  {  575,   9,   7,  10,    0,   -7},   // 'w'
  {  583,   5,   7,   7,    1,   -7},   // 'x'
  {  588,   7,  10,   7,    0,   -7},   // 'y'
  {  597,   6,   7,   6,    0,   -7},   // 'z'
  {  603,   4,  12,   7,    2,  -10},   // '{'
  {  609,   2,  13,   4,    1,  -10},   // '|'
  {  613,   5,  12,   7,    1,  -10},   // '}'
  {  621,   8,   2,  10,    1,   -5},   // '~'
  {  623,   4,   4,   6,    1,  -10},   // 'deg'
};
const GFXfont F_SM13R PROGMEM = {(uint8_t *)F_SM13R_Bitmaps, (GFXglyph *)F_SM13R_Glyphs, 0x20, 0x7F, 17};

// F_BIG104: Poppins-Bold.ttf 104px, 11 glyphs, 5035 bytes
const uint8_t F_BIG104_Bitmaps[] PROGMEM = {
  0x00, 0x00, 0x03, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x0F,
  0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x03, 0xFF, 0xFF, 0xFF,
  0xFF, 0xF0, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8,
  0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x01, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x0F, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFC, 0x07, 0xFF, 0xFF, 0xE0, 0x1F, 0xFF, 0xFF, 0x81, 0xFF, 0xFF, 0xE0, 0x01, 0xFF, 0xFF, 0xE0, 0x7F, 0xFF, 0xF0, 0x00, 0x3F, 0xFF,
  0xF8, 0x3F, 0xFF, 0xF8, 0x00, 0x07, 0xFF, 0xFF, 0x0F, 0xFF, 0xFC, 0x00, 0x00, 0xFF, 0xFF, 0xC3, 0xFF, 0xFE, 0x00, 0x00, 0x3F, 0xFF, 0xF1, 0xFF,
  0xFF, 0x80, 0x00, 0x07, 0xFF, 0xFE, 0x7F, 0xFF, 0xE0, 0x00, 0x01, 0xFF, 0xFF, 0x9F, 0xFF, 0xF0, 0x00, 0x00, 0x3F, 0xFF, 0xE7, 0xFF, 0xFC, 0x00,
  0x00, 0x0F, 0xFF, 0xF9, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0xFF, 0xFE, 0x7F, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 0xFF, 0xBF, 0xFF, 0xF0, 0x00, 0x00, 0x3F,
  0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x7F, 0xFF, 0xFF,
  0xFF, 0xE0, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x80,
  0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x01,
  0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x07, 0xFF, 0xFF,
  0xFF, 0xFE, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xF8,
  0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 0xFF, 0xDF, 0xFF, 0xF0, 0x00, 0x00,
  0x3F, 0xFF, 0xE7, 0xFF, 0xFC, 0x00, 0x00, 0x0F, 0xFF, 0xF9, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0xFF, 0xFE, 0x7F, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 0xFF,
  0x9F, 0xFF, 0xF8, 0x00, 0x00, 0x7F, 0xFF, 0xE7, 0xFF, 0xFE, 0x00, 0x00, 0x1F, 0xFF, 0xF0, 0xFF, 0xFF, 0xC0, 0x00, 0x0F, 0xFF, 0xFC, 0x3F, 0xFF,
  0xF0, 0x00, 0x03, 0xFF, 0xFF, 0x0F, 0xFF, 0xFE, 0x00, 0x01, 0xFF, 0xFF, 0xC1, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xE0, 0x7F, 0xFF, 0xF8, 0x00,
  0x7F, 0xFF, 0xF8, 0x1F, 0xFF, 0xFF, 0xC0, 0xFF, 0xFF, 0xFE, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xC0, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x3F,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x3F, 0xFF, 0xFF,
  0xFF, 0xFF, 0xF0, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFE,
  0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
  0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFE, 0x00, 0x3F, 0xFF, 0xF0, 0x01, 0xFF, 0xFF, 0x80, 0x0F, 0xFF, 0xFC, 0x00, 0x7F,
  0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0x00, 0x1F, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFE, 0x00, 0x3F, 0xFF, 0xF0, 0x01, 0xFF, 0xFF, 0x80,
  0x0F, 0xFF, 0xFC, 0x00, 0x7F, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0x00, 0x1F, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFE, 0x00, 0x3F, 0xFF,
  0xF0, 0x01, 0xFF, 0xFF, 0x80, 0x0F, 0xFF, 0xFC, 0x00, 0x7F, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0x00, 0x1F, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0xC0, 0x07,
  0xFF, 0xFE, 0x00, 0x3F, 0xFF, 0xF0, 0x01, 0xFF, 0xFF, 0x80, 0x0F, 0xFF, 0xFC, 0x00, 0x7F, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0x00, 0x1F, 0xFF, 0xF8,
  0x00, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFE, 0x00, 0x3F, 0xFF, 0xF0, 0x01, 0xFF, 0xFF, 0x80, 0x0F, 0xFF, 0xFC, 0x00, 0x7F, 0xFF, 0xE0, 0x03, 0xFF,
  0xFF, 0x00, 0x1F, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFE, 0x00, 0x3F, 0xFF, 0xF0, 0x01, 0xFF, 0xFF, 0x80, 0x0F, 0xFF, 0xFC, 0x00,
  0x7F, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0x00, 0x1F, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFE, 0x00, 0x3F, 0xFF, 0xF0, 0x01, 0xFF, 0xFF,
  0x80, 0x0F, 0xFF, 0xFC, 0x00, 0x7F, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0x00, 0x1F, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFE, 0x00, 0x3F,
  0xFF, 0xF0, 0x00, 0x00, 0x0F, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0xFF,
  0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00,
  0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x81, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x3F, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xC3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x7F, 0xFF, 0xF8, 0x07, 0xFF, 0xFF, 0xC7, 0xFF, 0xFF, 0x00, 0x1F, 0xFF, 0xFE, 0x7F,
  0xFF, 0xE0, 0x01, 0xFF, 0xFF, 0xE7, 0xFF, 0xFE, 0x00, 0x0F, 0xFF, 0xFE, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xEF, 0xFF, 0xFC, 0x00, 0x07, 0xFF,
  0xFE, 0xFF, 0xFF, 0xC0, 0x00, 0x7F, 0xFF, 0xEF, 0xFF, 0xF8, 0x00, 0x07, 0xFF, 0xFE, 0xFF, 0xFF, 0x80, 0x00, 0x7F, 0xFF, 0xEF, 0xFF, 0xF8, 0x00,
  0x07, 0xFF, 0xFE, 0xFF, 0xFF, 0x80, 0x00, 0x7F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xE0, 0x00,
  0x00, 0x00, 0x0F, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF,
  0xC0, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x07,
  0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xE0, 0x00, 0x00,
  0x00, 0x3F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0x00,
  0x00, 0x00, 0x03, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x01, 0xFF, 0xFF,
  0xF8, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x1F,
  0xFF, 0xFF, 0x80, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xC0, 0x00, 0x00,
  0x03, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFC, 0x00,
  0x00, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF,
  0x80, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00,
  0x00, 0x3F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF,
  0x80, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x7F, 0xFF,
  0xFF, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0,
  0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0xC0, 0xFF, 0xFF, 0xFC, 0x3F, 0xFF, 0xF8, 0x01, 0xFF, 0xFF, 0xE3, 0xFF, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0x1F, 0xFF,
  0xFC, 0x00, 0x1F, 0xFF, 0xF8, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xC7, 0xFF, 0xFE, 0x00, 0x03, 0xFF, 0xFE, 0x3F, 0xFF, 0xE0, 0x00, 0x1F, 0xFF,
  0xF1, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xE0, 0x00, 0x00, 0x00,
  0x01, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xF8, 0x00,
  0x00, 0x00, 0x01, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFC, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x1F, 0xFF, 0xFF,
  0xFE, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x01,
  0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFE, 0x00,
  0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x3F, 0xFF,
  0xFF, 0xFF, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0x80, 0x00,
  0x00, 0x00, 0x0F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x07, 0xFF,
  0xFC, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFE, 0x00,
  0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x7F, 0xFF, 0xFF,
  0xFF, 0xF0, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x3F, 0xFF, 0xF7, 0xFF, 0xFE, 0x00, 0x03, 0xFF, 0xFF, 0xBF, 0xFF, 0xFC, 0x00, 0x3F,
  0xFF, 0xF9, 0xFF, 0xFF, 0xF0, 0x03, 0xFF, 0xFF, 0xC7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF1, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC,
  0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFC, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00,
  0x1F, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x01, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
  0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF,
  0xF0, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xE0,
  0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xC0, 0x00,
  0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00,
  0x00, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x07,
  0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x3F, 0xFF,
  0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x03, 0xFF, 0xFE, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x1F, 0xFF, 0xF7, 0xFF, 0xFC, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x3F,
  0xFF, 0xE0, 0x00, 0x00, 0x1F, 0xFF, 0xF1, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x8F, 0xFF, 0xF8, 0x00, 0x00, 0x0F, 0xFF, 0xF8, 0x7F, 0xFF,
  0xC0, 0x00, 0x00, 0xFF, 0xFF, 0xC3, 0xFF, 0xFE, 0x00, 0x00, 0x07, 0xFF, 0xFC, 0x1F, 0xFF, 0xF0, 0x00, 0x00, 0x7F, 0xFF, 0xC0, 0xFF, 0xFF, 0x80,
  0x00, 0x07, 0xFF, 0xFE, 0x07, 0xFF, 0xFC, 0x00, 0x00, 0x7F, 0xFF, 0xE0, 0x3F, 0xFF, 0xE0, 0x00, 0x03, 0xFF, 0xFE, 0x01, 0xFF, 0xFF, 0x00, 0x00,
  0x3F, 0xFF, 0xF0, 0x0F, 0xFF, 0xF8, 0x00, 0x03, 0xFF, 0xFF, 0x00, 0x7F, 0xFF, 0xC0, 0x00, 0x1F, 0xFF, 0xF0, 0x03, 0xFF, 0xFE, 0x00, 0x01, 0xFF,
  0xFF, 0x80, 0x1F, 0xFF, 0xF0, 0x00, 0x1F, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0x80, 0x00, 0xFF, 0xFF, 0x80, 0x07, 0xFF, 0xFC, 0x00, 0x0F, 0xFF, 0xFC,
  0x00, 0x3F, 0xFF, 0xE0, 0x00, 0xFF, 0xFF, 0xC0, 0x01, 0xFF, 0xFF, 0x00, 0x0F, 0xFF, 0xFC, 0x00, 0x0F, 0xFF, 0xF8, 0x00, 0x7F, 0xFF, 0xE0, 0x00,
  0x7F, 0xFF, 0xC0, 0x07, 0xFF, 0xFE, 0x00, 0x03, 0xFF, 0xFE, 0x00, 0x7F, 0xFF, 0xE0, 0x00, 0x1F, 0xFF, 0xF0, 0x03, 0xFF, 0xFF, 0x00, 0x00, 0xFF,
  0xFF, 0x80, 0x3F, 0xFF, 0xF0, 0x00, 0x07, 0xFF, 0xFC, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x03,
  0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF,
  0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFC,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xF8, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFE, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x87, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFC, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8,
  0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0x87, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x1F, 0xFF, 0xE0, 0x00, 0x00, 0x00,
  0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFE, 0x00,
  0x00, 0x00, 0x00, 0x0F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x1F,
  0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xF8, 0x03, 0xFF, 0x80, 0x00, 0x3F, 0xFF, 0xC0, 0xFF, 0xFF,
  0x80, 0x01, 0xFF, 0xFE, 0x3F, 0xFF, 0xFF, 0x00, 0x0F, 0xFF, 0xF3, 0xFF, 0xFF, 0xFE, 0x00, 0x7F, 0xFF, 0xBF, 0xFF, 0xFF, 0xF8, 0x03, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xF0, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8,
  0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x8F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x7F, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xF3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9F, 0xFF, 0xFF, 0xC0, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0xF7, 0xFF,
  0xFF, 0x00, 0x03, 0xFF, 0xFF, 0xBF, 0xFF, 0xF0, 0x00, 0x0F, 0xFF, 0xFD, 0xFF, 0xFF, 0x80, 0x00, 0x3F, 0xFF, 0xEF, 0xFF, 0xF8, 0x00, 0x01, 0xFF,
  0xFF, 0x80, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x00, 0x00, 0x00,
  0x00, 0x0F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xF0,
  0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x03,
  0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x3F, 0xFF, 0xF7, 0xFF, 0xFF, 0x00, 0x03, 0xFF, 0xFF, 0xBF, 0xFF, 0xFC, 0x00, 0x3F, 0xFF, 0xFD, 0xFF, 0xFF,
  0xF0, 0x03, 0xFF, 0xFF, 0xE7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0x87, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x03, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xC0, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x03,
  0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xF8,
  0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00,
  0x7F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
  0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x01, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFE, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x03, 0xFF, 0xFF, 0xC0, 0xFF, 0xFF, 0xF8, 0x0F, 0xFF,
  0xFE, 0x00, 0x3F, 0xFF, 0xF8, 0x1F, 0xFF, 0xF8, 0x00, 0x3F, 0xFF, 0xF0, 0x3F, 0xFF, 0xE0, 0x00, 0x7F, 0xFF, 0xE0, 0xFF, 0xFF, 0x80, 0x00, 0x7F,
  0xFF, 0xE1, 0xFF, 0xFE, 0x00, 0x00, 0xFF, 0xFF, 0xC3, 0xFF, 0xFC, 0x00, 0x00, 0xFF, 0xFF, 0x87, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF,
  0xE0, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00,
  0x00, 0x01, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xF8, 0x00, 0xFF, 0xE0, 0x00, 0x0F, 0xFF, 0xF0, 0x0F, 0xFF, 0xF8, 0x00, 0x1F, 0xFF,
  0xE0, 0x7F, 0xFF, 0xFE, 0x00, 0x3F, 0xFF, 0xC3, 0xFF, 0xFF, 0xFE, 0x00, 0x7F, 0xFF, 0x8F, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x3F, 0xFF, 0xFF,
  0xFF, 0x01, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x0F, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF9, 0xFF, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0xFB, 0xFF, 0xFF, 0xF0, 0x00, 0xFF, 0xFF, 0xF7, 0xFF,
  0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xEF, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xDF, 0xFF, 0xFC, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x01,
  0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x03, 0xFF, 0xFE, 0xFF, 0xFF, 0x80, 0x00, 0x07, 0xFF, 0xFD, 0xFF,
  0xFF, 0x00, 0x00, 0x0F, 0xFF, 0xFB, 0xFF, 0xFE, 0x00, 0x00, 0x1F, 0xFF, 0xF7, 0xFF, 0xFC, 0x00, 0x00, 0x3F, 0xFF, 0xEF, 0xFF, 0xF8, 0x00, 0x00,
  0x7F, 0xFF, 0xDF, 0xFF, 0xF0, 0x00, 0x00, 0xFF, 0xFF, 0x9F, 0xFF, 0xE0, 0x00, 0x03, 0xFF, 0xFF, 0x3F, 0xFF, 0xE0, 0x00, 0x07, 0xFF, 0xFE, 0x7F,
  0xFF, 0xC0, 0x00, 0x0F, 0xFF, 0xFC, 0xFF, 0xFF, 0xC0, 0x00, 0x3F, 0xFF, 0xF0, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xE1, 0xFF, 0xFF, 0xC0, 0x03,
  0xFF, 0xFF, 0xC3, 0xFF, 0xFF, 0xE0, 0x1F, 0xFF, 0xFF, 0x83, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x07,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x1F, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFE, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00,
  0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x0F, 0xFF, 0xFF,
  0xFF, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00,
  0x00, 0x03, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xF0, 0x00,
  0x00, 0x00, 0x0F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xC0,
  0x00, 0x00, 0x00, 0x1F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF,
  0x80, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x03, 0xFF,
  0xFE, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x0F,
  0xFF, 0xF8, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xC0, 0x00, 0x00, 0x00,
  0x3F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x00, 0x00, 0x00,
  0x00, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFE, 0x00, 0x00,
  0x00, 0x01, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xF8, 0x00,
  0x00, 0x00, 0x07, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xE0,
  0x00, 0x00, 0x00, 0x1F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
  0xC0, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x03, 0xFF,
  0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x0F,
  0xFF, 0xFC, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xE0, 0x00, 0x00, 0x00,
  0x1F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xC0, 0x00, 0x00,
  0x00, 0x7F, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xE0,
  0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00,
  0x3F, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x01, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0x80, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0,
  0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x1F, 0xFF, 0xFF,
  0x81, 0xFF, 0xFF, 0xF8, 0x1F, 0xFF, 0xFE, 0x00, 0x7F, 0xFF, 0xF8, 0x3F, 0xFF, 0xFC, 0x00, 0x1F, 0xFF, 0xFC, 0x3F, 0xFF, 0xF8, 0x00, 0x1F, 0xFF,
  0xFC, 0x3F, 0xFF, 0xF0, 0x00, 0x0F, 0xFF, 0xFC, 0x3F, 0xFF, 0xF0, 0x00, 0x0F, 0xFF, 0xFC, 0x3F, 0xFF, 0xE0, 0x00, 0x07, 0xFF, 0xFC, 0x3F, 0xFF,
  0xE0, 0x00, 0x07, 0xFF, 0xFC, 0x3F, 0xFF, 0xE0, 0x00, 0x07, 0xFF, 0xFC, 0x3F, 0xFF, 0xE0, 0x00, 0x07, 0xFF, 0xFC, 0x3F, 0xFF, 0xE0, 0x00, 0x07,
  0xFF, 0xFC, 0x3F, 0xFF, 0xE0, 0x00, 0x07, 0xFF, 0xF8, 0x1F, 0xFF, 0xF0, 0x00, 0x0F, 0xFF, 0xF8, 0x1F, 0xFF, 0xF0, 0x00, 0x0F, 0xFF, 0xF8, 0x1F,
  0xFF, 0xF8, 0x00, 0x1F, 0xFF, 0xF0, 0x0F, 0xFF, 0xFC, 0x00, 0x3F, 0xFF, 0xF0, 0x0F, 0xFF, 0xFE, 0x00, 0x7F, 0xFF, 0xE0, 0x07, 0xFF, 0xFF, 0x81,
  0xFF, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
  0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x7F, 0xFF,
  0xFF, 0xFF, 0xFE, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xE0, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x3F, 0xFF,
  0xFF, 0x80, 0xFF, 0xFF, 0xFC, 0x3F, 0xFF, 0xFC, 0x00, 0x3F, 0xFF, 0xFC, 0x7F, 0xFF, 0xF0, 0x00, 0x0F, 0xFF, 0xFE, 0x7F, 0xFF, 0xE0, 0x00, 0x07,
  0xFF, 0xFE, 0x7F, 0xFF, 0xC0, 0x00, 0x03, 0xFF, 0xFE, 0xFF, 0xFF, 0xC0, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x03, 0xFF, 0xFF, 0xFF,
  0xFF, 0x80, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00,
  0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x03, 0xFF, 0xFF,
  0xFF, 0xFF, 0xE0, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x07, 0xFF, 0xFF, 0x7F, 0xFF, 0xF0, 0x00, 0x0F, 0xFF, 0xFE, 0x7F, 0xFF, 0xFC,
  0x00, 0x3F, 0xFF, 0xFE, 0x7F, 0xFF, 0xFF, 0x80, 0xFF, 0xFF, 0xFE, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFC, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x07, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00,
  0x03, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF,
  0xE0, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00,
  0x7F, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x07, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFE, 0x1F, 0xFF, 0xFF, 0xC0, 0xFF, 0xFF, 0xF8, 0x7F, 0xFF, 0xF8, 0x00, 0x7F, 0xFF, 0xE1, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xC7, 0xFF, 0xFE,
  0x00, 0x01, 0xFF, 0xFF, 0x3F, 0xFF, 0xF8, 0x00, 0x07, 0xFF, 0xFC, 0xFF, 0xFF, 0xC0, 0x00, 0x0F, 0xFF, 0xF3, 0xFF, 0xFF, 0x00, 0x00, 0x3F, 0xFF,
  0xEF, 0xFF, 0xF8, 0x00, 0x00, 0x7F, 0xFF, 0xBF, 0xFF, 0xE0, 0x00, 0x01, 0xFF, 0xFE, 0xFF, 0xFF, 0x80, 0x00, 0x07, 0xFF, 0xFB, 0xFF, 0xFE, 0x00,
  0x00, 0x1F, 0xFF, 0xEF, 0xFF, 0xF8, 0x00, 0x00, 0x7F, 0xFF, 0xBF, 0xFF, 0xE0, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x07, 0xFF, 0xFF,
  0xFF, 0xFF, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x03, 0xFF, 0xFF, 0x7F, 0xFF, 0xE0, 0x00,
  0x1F, 0xFF, 0xFD, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xF7, 0xFF, 0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xDF, 0xFF, 0xFF, 0xC0, 0xFF, 0xFF, 0xFF, 0x3F,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC7, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0xFF, 0xFF, 0xFF, 0xFB, 0xFF, 0xFC, 0x01, 0xFF, 0xFF, 0xFF, 0xCF, 0xFF, 0xF0, 0x03, 0xFF, 0xFF, 0xFE, 0x3F,
  0xFF, 0xC0, 0x03, 0xFF, 0xFF, 0xF0, 0xFF, 0xFF, 0x00, 0x03, 0xFF, 0xFF, 0x03, 0xFF, 0xFC, 0x00, 0x00, 0xFF, 0xC0, 0x0F, 0xFF, 0xE0, 0x00, 0x00,
  0x00, 0x00, 0x3F, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF,
  0xE0, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFE, 0x3F, 0xFF, 0xE0, 0x00, 0x07, 0xFF, 0xF0, 0xFF, 0xFF, 0x80,
  0x00, 0x1F, 0xFF, 0xC3, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x0F, 0xFF, 0xFC, 0x00, 0x03, 0xFF, 0xFC, 0x1F, 0xFF, 0xF8, 0x00, 0x1F, 0xFF, 0xE0,
  0x7F, 0xFF, 0xF0, 0x00, 0xFF, 0xFF, 0x81, 0xFF, 0xFF, 0xE0, 0x07, 0xFF, 0xFE, 0x03, 0xFF, 0xFF, 0xE0, 0x7F, 0xFF, 0xF0, 0x0F, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xC0, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x03,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x1F, 0xFF, 0xFF, 0xFF,
  0xFE, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00,
  0x7F, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0x00, 0x00, 0x00,
};
const GFXglyph F_BIG104_Glyphs[] PROGMEM = {
  {    0,   0,   0,  22,    0,    0},   // ' '
  {    0,   0,   0,   0,    0,    0},   // '!'
  {    0,   0,   0,   0,    0,    0},   // '"'
  {    0,   0,   0,   0,    0,    0},   // '#'
  {    0,   0,   0,   0,    0,    0},   // '$'
  {    0,   0,   0,   0,    0,    0},   // '%'
  {    0,   0,   0,   0,    0,    0},   // '&'
  {    0,   0,   0,   0,    0,    0},   // '''
  {    0,   0,   0,   0,    0,    0},   // '('
  {    0,   0,   0,   0,    0,    0},   // ')'
  {    0,   0,   0,   0,    0,    0},   // '*'
  {    0,   0,   0,   0,    0,    0},   // '+'
  {    0,   0,   0,   0,    0,    0},   // ','
  {    0,   0,   0,   0,    0,    0},   // '-'
  {    0,   0,   0,   0,    0,    0},   // '.'
  {    0,   0,   0,   0,    0,    0},   // '/'
  {    0,  58,  78,  68,    5,  -78},   // '0'
  {  566,  29,  76,  39,    2,  -76},   // '1'
  {  842,  52,  77,  59,    4,  -78},   // '2'
  { 1343,  53,  79,  63,    5,  -78},   // '3'
  { 1867,  61,  75,  70,    5,  -75},   // '4'
  { 2439,  53,  76,  68,    9,  -76},   // '5'
  { 2943,  55,  78,  66,    6,  -77},   // '6'
  { 3480,  50,  76,  56,    3,  -76},   // '7'
  { 3955,  56,  79,  67,    6,  -78},   // '8'
  { 4508,  54,  78,  64,    5,  -78},   // '9'
};
const GFXfont F_BIG104 PROGMEM = {(uint8_t *)F_BIG104_Bitmaps, (GFXglyph *)F_BIG104_Glyphs, 0x20, 0x39, 147};

// =====================================================================
// HARDWARE
// =====================================================================
WiFiClientSecure secureClient;
Adafruit_SSD1306 infoDisplay(128, 64, &OLED_L_WIRE, -1);   // left OLED: the info pages
Adafruit_SSD1306 infoR(128, 64, &OLED_R_WIRE, -1);         // right OLED: clock / companion
// Both GC9A01 share SCK / MOSI / DC / RST. One bus object with no CS of its own; each panel is selected by
// pulling its CS low around every transaction (tftSelect), so they never see each other's traffic.
Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, GFX_NOT_DEFINED, TFT_SCK, TFT_MOSI, GFX_NOT_DEFINED, FSPI);
Arduino_GFX *tftL = new Arduino_GC9A01(bus, GFX_NOT_DEFINED, TFT_ROTATION, true);
Arduino_GFX *tftR = new Arduino_GC9A01(bus, GFX_NOT_DEFINED, TFT_ROTATION, true);
void oledPush(int side);
void tftSelect(int side) { digitalWrite(TFT_CS_L, side == 0 ? LOW : HIGH); digitalWrite(TFT_CS_R, side == 1 ? LOW : HIGH); }
// Full-frame 240 x 240 RGB565 canvas (115 KB) with its framebuffer in PSRAM: internal heap stays for WiFi / TLS.
class PsCanvas : public Arduino_Canvas {
 public:
  PsCanvas() : Arduino_Canvas(240, 240, nullptr, 0, 0, 0) {}
  bool alloc() {
    if (!_framebuffer) _framebuffer = (uint16_t *)heap_caps_aligned_alloc(16, 240 * 240 * 2, MALLOC_CAP_SPIRAM);
    return _framebuffer != nullptr;
  }
  void setFb(uint16_t *p) { _framebuffer = p; }   // 6.5.0: double buffering, present() swaps buffers per frame
};
PsCanvas *canvasL = new PsCanvas(), *canvasR = new PsCanvas();
// LED rings: logical LED 0 = 12 o'clock under the red pip, logical index grows clockwise from the front.
// physical = (RING_OFFSET + RING_DIR * logical) mod 24 (measured on 6.0.8: left LED 0 at 3 o'clock, right at 9).
Adafruit_NeoPixel ringL(24, RING_L_PIN, NEO_GRB + NEO_KHZ800), ringR(24, RING_R_PIN, NEO_GRB + NEO_KHZ800);
const int RING_OFFSET[2] = { 18, 6 };
const int RING_DIR[2]    = { 1, 1 };
ESP32Encoder encL, encR;
// microSD on the display SPI bus (same FSPI host; the core's spiTransaction sets clock and mode per transfer, so
// the 80 MHz displays and the 20 MHz card can take turns). SD is used from loop() only, never from fetchTask.
// microSD on ESP-IDF sdspi (6.5.0), sharing SPI2 with the panels: the bus driver arbitrates between them.
class SdIdf : public fs::FS {
 public:
  sdmmc_card_t *card = NULL;
  SdIdf() : fs::FS(FSImplPtr(new VFSImpl())) {}
  bool mount() {
    sdmmc_host_t host = SDSPI_HOST_DEFAULT(); host.slot = SPI2_HOST; host.max_freq_khz = 20000;
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT(); slot.host_id = SPI2_HOST; slot.gpio_cs = (gpio_num_t)SD_CS;
    esp_vfs_fat_mount_config_t mc = {}; mc.format_if_mount_failed = false; mc.max_files = 5; mc.allocation_unit_size = 16 * 1024;
    esp_err_t e = esp_vfs_fat_sdspi_mount("/sd", &host, &slot, &mc, &card);
    if (e != ESP_OK) { logf("SD: mount %s\n", esp_err_to_name(e)); card = NULL; return false; }
    _impl->mountpoint("/sd"); return true;
  }
  bool isSDHC() { return card && (card->ocr & (1 << 30)); }
  uint64_t cardSize() { return card ? (uint64_t)card->csd.capacity * card->csd.sector_size : 0; }
  uint64_t totalBytes() { uint64_t t = 0, f = 0; esp_vfs_fat_info("/sd", &t, &f); return t; }
  uint64_t usedBytes() { uint64_t t = 0, f = 0; esp_vfs_fat_info("/sd", &t, &f); return t - f; }
};
SdIdf SD;
bool sdOk = false;
// BMP280 / BME280 on I2C bus 0 (0x76 or 0x77; chip id 0x58 BMP280, 0x60 BME280 adds humidity)
Adafruit_BMP280 bmp(&Wire); Adafruit_BME280 bme;
uint8_t envAddr = 0, envChip = 0; float envT = NAN, envP = NAN, envH = NAN; unsigned long envMs = 0;
WebServer server(80);
Preferences prefs;

#define RADAR_W     240
#define RADAR_H     240
#define RADAR_CX    120
#define RADAR_CY    120
#define RADAR_MAX_R 118

// Drawing wrapper. All screen drawing goes through `radar`, which present() points at the left or the
// right dial's canvas. v6 draws full frames, so dy stays 0 (kept because clock face 2 still reads it).
// Text uses the 1-bit GFX fonts below.
int tftBright = 80, oledBright = 60;      // percent, NVS
float brightK = 0.8f;                     // effective multiplier (brightness x night)
extern float fadeK;                       // transition fade, 1 = none
struct BandCanvas {
  PsCanvas *c; Arduino_GFX *out; int side; int dy = 0;
  BandCanvas(PsCanvas *cv, Arduino_GFX *o, int sd) : c(cv), out(o), side(sd) {}
  bool begin() { return c->alloc() && c->begin(GFX_SKIP_OUTPUT_BEGIN); }
  uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return c->color565(r, g, b); }
  void fillScreen(uint16_t col) { c->fillScreen(col); }
  void drawPixel(int x, int y, uint16_t col) { c->drawPixel(x, y - dy, col); }
  void drawLine(int x0, int y0, int x1, int y1, uint16_t col) { c->drawLine(x0, y0 - dy, x1, y1 - dy, col); }
  void drawCircle(int x, int y, int r, uint16_t col) { c->drawCircle(x, y - dy, r, col); }
  void fillCircle(int x, int y, int r, uint16_t col) { c->fillCircle(x, y - dy, r, col); }
  void fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t col) { c->fillTriangle(x0, y0 - dy, x1, y1 - dy, x2, y2 - dy, col); }
  void fillRect(int x, int y, int w, int h, uint16_t col) { c->fillRect(x, y - dy, w, h, col); }
  void drawRect(int x, int y, int w, int h, uint16_t col) { c->drawRect(x, y - dy, w, h, col); }
  void fillRoundRect(int x, int y, int w, int h, int r, uint16_t col) { c->fillRoundRect(x, y - dy, w, h, r, col); }
  void drawRoundRect(int x, int y, int w, int h, int r, uint16_t col) { c->drawRoundRect(x, y - dy, w, h, r, col); }
  void fillArc(int x, int y, int r1, int r2, float a0, float a1, uint16_t col) { c->fillArc(x, y - dy, r1, r2, a0, a1, col); }
  void drawArc(int x, int y, int r1, int r2, float a0, float a1, uint16_t col) { c->drawArc(x, y - dy, r1, r2, a0, a1, col); }
  // arc with angles like polar(): 0 at 12 o'clock, clockwise (Arduino_GFX counts from 3 o'clock)
  void ringArc(int cx, int cy, int r1, int r2, float a0, float a1, uint16_t col) { c->fillArc(cx, cy - dy, r1, r2, a0 - 90, a1 - 90, col); }
  // built-in 5x7 font (kept for the HUD and tiny labels)
  void setTextSize(int n) { c->setTextSize(n); }
  void setTextColor(uint16_t col) { c->setTextColor(col); }
  void setCursor(int x, int y) { c->setCursor(x, y - dy); }
  template <typename T> void print(T v) { c->print(v); }
  template <typename... A> void printf(const char *f, A... a) { c->printf(f, a...); }
  // GFX (1-bit) fonts, baseline referenced
  int glyphAdv(const GFXfont *f, char ch) {
    uint8_t first = pgm_read_byte(&f->first), last = pgm_read_byte(&f->last);
    if ((uint8_t)ch < first || (uint8_t)ch > last) return 0;
    GFXglyph *g = (GFXglyph *)pgm_read_ptr(&f->glyph) + ((uint8_t)ch - first);
    return pgm_read_byte(&g->xAdvance);
  }
  int textWidth(const GFXfont *f, const char *s) { int w = 0; while (*s) w += glyphAdv(f, *s++); return w; }
  void text(int x, int yb, const char *s, const GFXfont *f, uint16_t col) {
    uint8_t first = pgm_read_byte(&f->first), last = pgm_read_byte(&f->last);
    uint8_t *bmp = (uint8_t *)pgm_read_ptr(&f->bitmap);
    for (; *s; s++) {
      uint8_t ch = (uint8_t)*s;
      if (ch < first || ch > last) continue;
      GFXglyph *g = (GFXglyph *)pgm_read_ptr(&f->glyph) + (ch - first);
      uint16_t bo = pgm_read_word(&g->bitmapOffset);
      uint8_t w = pgm_read_byte(&g->width), h = pgm_read_byte(&g->height), adv = pgm_read_byte(&g->xAdvance);
      int8_t xo = (int8_t)pgm_read_byte(&g->xOffset), yo = (int8_t)pgm_read_byte(&g->yOffset);
      int gx = x + xo, gy = yb + yo - dy;
      if (w && h && gy < 240 && gy + h > 0 && gx < 240 && gx + w > 0) {
        uint8_t bits = 0, bit = 0;
        for (int yy = 0; yy < h; yy++)
          for (int xx = 0; xx < w; xx++) {
            if (!(bit++ & 7)) bits = pgm_read_byte(&bmp[bo++]);
            if (bits & 0x80) { int px = gx + xx, py = gy + yy; if (py >= 0 && py < 240 && px >= 0 && px < 240) c->drawPixel(px, py, col); }
            bits <<= 1;
          }
      }
      x += adv;
    }
  }
  // RGB565 image with an 8-bit alpha channel, blended straight into the framebuffer (airline logos, 6.2.2)
  void imageAlpha(int x, int y, const uint16_t *px, const uint8_t *al, int w, int h) {
    uint16_t *fb = c->getFramebuffer(); if (!fb) return; y -= dy;
    for (int j = 0; j < h; j++) {
      int yy = y + j; if (yy < 0 || yy >= 240) continue;
      for (int i = 0; i < w; i++) {
        int xx = x + i; if (xx < 0 || xx >= 240) continue;
        uint8_t a = al[j * w + i]; if (!a) continue;
        uint16_t s = px[j * w + i], &d = fb[yy * 240 + xx];
        if (a >= 250) { d = s; continue; }
        int na = 255 - a;
        int r = (((s >> 11) & 0x1F) * a + ((d >> 11) & 0x1F) * na) / 255, g = (((s >> 5) & 0x3F) * a + ((d >> 5) & 0x3F) * na) / 255, b = ((s & 0x1F) * a + (d & 0x1F) * na) / 255;
        d = (uint16_t)((r << 11) | (g << 5) | b);
      }
    }
  }
  void textC(int yb, const char *s, const GFXfont *f, uint16_t col) { text(RADAR_CX - textWidth(f, s) / 2, yb, s, f, col); }
  void textR(int xr, int yb, const char *s, const GFXfont *f, uint16_t col) { text(xr - textWidth(f, s), yb, s, f, col); }
};
BandCanvas *radarL = new BandCanvas(canvasL, tftL, 0), *radarR = new BandCanvas(canvasR, tftR, 1);
BandCanvas *radar = radarL;          // drawing target; present() / presentR() switch it
PaintFn currentPaint = NULL, currentPaintR = NULL;
void paintToast();
// ===== display pipeline (6.5.0) =====
// Two framebuffers per dial in PSRAM. present() paints into the back buffer on core 1 and hands it to the output
// task on core 0, which streams it to the panel with DMA (ESP-IDF spi_master, 20-row chunks, the next chunk
// prepared, byte-swapped and brightness-scaled while the current one transmits). Painting the next frame overlaps
// sending the last one. The output task also sends panel commands and the OLED buffers (I2C off the UI loop).
uint16_t *fbuf[2][2] = {{NULL, NULL}, {NULL, NULL}}; int fbBack[2] = {0, 0}; uint16_t *fbFront[2] = {NULL, NULL};
SemaphoreHandle_t fbFree[2][2];
struct GfxJob { uint8_t kind, side, buf; uint8_t cmd; float k; };   // kind 0 frame, 1 panel command
QueueHandle_t gfxQ = NULL; TaskHandle_t gfxTask = NULL, oledTask = NULL;
spi_device_handle_t tftDev[2] = {NULL, NULL};
bool gfxReady = false;
unsigned long tPaintUs[2] = {0, 0}, tFlushUs[2] = {0, 0}, tWaitUs[2] = {0, 0}, tFrames[2] = {0, 0};
void IRAM_ATTR tftPreCb(spi_transaction_t *t) { gpio_set_level((gpio_num_t)TFT_DC, (int)(intptr_t)t->user); }
void tftCmd(int side, uint8_t cmd, const uint8_t *data, int n) {   // output task only
  spi_transaction_t t = {}; t.length = 8; t.flags = SPI_TRANS_USE_TXDATA; t.tx_data[0] = cmd; t.user = (void *)0;
  spi_device_polling_transmit(tftDev[side], &t);
  if (n > 0) { spi_transaction_t d = {}; d.length = n * 8; d.user = (void *)1;
    if (n <= 4) { d.flags = SPI_TRANS_USE_TXDATA; memcpy(d.tx_data, data, n); } else d.tx_buffer = data;
    spi_device_polling_transmit(tftDev[side], &d); }
}
#define GFX_ROWS 20
void tftSendFrame(int side, const uint16_t *fb, float k) {
  static uint16_t *dbuf[2] = {NULL, NULL}; static uint16_t lr[32], lg[64], lb[32]; static float lastK = -1;
  if (!dbuf[0]) for (int i = 0; i < 2; i++) dbuf[i] = (uint16_t *)heap_caps_malloc(240 * GFX_ROWS * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (dbuf[0] && !dbuf[1]) { static bool said = false; if (!said) { said = true; logln("gfx: DMA line buffer 2 failed, retrying"); } heap_caps_free(dbuf[0]); dbuf[0] = NULL; }   // 6.14.1: retry next frame, not blank forever
  if (!dbuf[0] || !dbuf[1] || !fb) return;
  if (k > 1) k = 1; if (k < 0) k = 0;
  if (k != lastK) { lastK = k; for (int i = 0; i < 32; i++) { lr[i] = (uint16_t)((uint8_t)(i * k)) << 11; lb[i] = (uint8_t)(i * k); } for (int i = 0; i < 64; i++) lg[i] = (uint16_t)((uint8_t)(i * k)) << 5; }
  spi_device_acquire_bus(tftDev[side], portMAX_DELAY);              // the SD card waits for the whole frame
  static const uint8_t win[4] = {0, 0, 0, 239};
  tftCmd(side, 0x2A, win, 4); tftCmd(side, 0x2B, win, 4); tftCmd(side, 0x2C, NULL, 0);
  // 6.5.2: polling DMA (polling_start / polling_end), no interrupt-mode transactions. 6.5.0 queued them while
  // holding the bus with the SD card on the same host and crashed in spi_intr -> spi_bus_lock_bg_exit (null
  // pointer, core dump). Overlap kept: chunk c transmits while the CPU prepares chunk c + 1.
  static spi_transaction_t tr[2]; bool busy = false; const int n = 240 * GFX_ROWS;
  for (int c = 0; c < 240 / GFX_ROWS; c++) {
    uint16_t *d = dbuf[c & 1]; const uint16_t *src = fb + c * n;
    if (k >= 0.999f) for (int i = 0; i < n; i++) d[i] = __builtin_bswap16(src[i]);
    else for (int i = 0; i < n; i++) { uint16_t v = src[i]; d[i] = __builtin_bswap16(lr[v >> 11] | lg[(v >> 5) & 0x3F] | lb[v & 0x1F]); }
    // 6.6.4: sleep a tick while the chunk transmits instead of spinning in polling_end. At 40 MHz a chunk takes
    // 1.9 ms and the spin made this task eat ~100 % of core 0, starving the aircraft fetcher, fetchTask and the
    // mDNS responder (no aircraft after the first fetch, name lookups dead). The sleep overlaps the transfer.
    if (busy) { vTaskDelay(1); spi_device_polling_end(tftDev[side], portMAX_DELAY); }
    spi_transaction_t &t = tr[c & 1]; memset(&t, 0, sizeof(t)); t.length = n * 16; t.tx_buffer = d; t.user = (void *)1;
    spi_device_polling_start(tftDev[side], &t, portMAX_DELAY); busy = true;
  }
  if (busy) { vTaskDelay(1); spi_device_polling_end(tftDev[side], portMAX_DELAY); }
  spi_device_release_bus(tftDev[side]);
}
// OLED pushes: the UI copies the Adafruit buffer into a shadow, the output task sends the shadow over I2C
uint8_t oledShadow[2][1024]; volatile bool oledDirty[2] = {false, false}; portMUX_TYPE oledMux = portMUX_INITIALIZER_UNLOCKED;
void oledPush(int side) {
  uint8_t *src = side ? infoR.getBuffer() : infoDisplay.getBuffer(); if (!src) return;
  portENTER_CRITICAL(&oledMux); memcpy(oledShadow[side], src, 1024); oledDirty[side] = true; portEXIT_CRITICAL(&oledMux);
  if (oledTask) xTaskNotifyGive(oledTask);
}
void oledSend(int side) {
  static uint8_t buf[1024];
  portENTER_CRITICAL(&oledMux); if (!oledDirty[side]) { portEXIT_CRITICAL(&oledMux); return; } memcpy(buf, oledShadow[side], 1024); oledDirty[side] = false; portEXIT_CRITICAL(&oledMux);
  TwoWire &w = side ? OLED_R_WIRE : OLED_L_WIRE;
  w.beginTransmission(0x3C); w.write(0x00); w.write(0x22); w.write(0); w.write(7); w.write(0x21); w.write(0); w.write(127); w.endTransmission();
  for (int i = 0; i < 1024;) { w.beginTransmission(0x3C); w.write(0x40); for (int n = 0; n < 120 && i < 1024; n++) w.write(buf[i++]); w.endTransmission(); }
}
void gfxLoop(void *) {
  GfxJob j;
  for (;;) {
    if (xQueueReceive(gfxQ, &j, portMAX_DELAY) == pdTRUE) {
      if (j.kind == 0) { unsigned long t0 = micros(); tftSendFrame(j.side, fbuf[j.side][j.buf], j.k); tFlushUs[j.side] += micros() - t0; xSemaphoreGive(fbFree[j.side][j.buf]); }
      else { spi_device_acquire_bus(tftDev[j.side], portMAX_DELAY); tftCmd(j.side, j.cmd, NULL, 0); spi_device_release_bus(tftDev[j.side]); }
    }
  }
}
// 6.5.4: OLEDs on their own task (I2C pushes take ~25 ms each; in the frame task they delayed round-display frames)
void oledLoop(void *) { for (;;) { ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)); oledSend(0); oledSend(1); } }
void tftCommand(int side, uint8_t cmd) { if (!gfxReady) return; GfxJob j = {1, (uint8_t)side, 0, cmd, 0}; xQueueSend(gfxQ, &j, portMAX_DELAY); }
// After Arduino_GFX has run the panels' init sequence: hand the bus to the ESP-IDF driver, add both panels (hardware
// CS 41 / 16) and the SD card (CS 10), allocate the framebuffers, start the output task.
bool gfxInit() {
  spi_bus_config_t bc = {}; bc.mosi_io_num = TFT_MOSI; bc.miso_io_num = SD_MISO; bc.sclk_io_num = TFT_SCK; bc.quadwp_io_num = -1; bc.quadhd_io_num = -1; bc.max_transfer_sz = 240 * GFX_ROWS * 2 + 64;
  esp_err_t e = spi_bus_initialize(SPI2_HOST, &bc, SPI_DMA_CH_AUTO);
  if (e != ESP_OK) { logf("gfx: spi_bus_initialize %s\n", esp_err_to_name(e)); return false; }
  for (int side = 0; side < 2; side++) {
    spi_device_interface_config_t dc = {}; dc.clock_speed_hz = TFT_SPI_HZ; dc.mode = 0; dc.spics_io_num = side ? TFT_CS_R : TFT_CS_L;
    dc.queue_size = 4; dc.pre_cb = tftPreCb; dc.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;
    e = spi_bus_add_device(SPI2_HOST, &dc, &tftDev[side]);
    if (e != ESP_OK) { logf("gfx: add panel %d %s\n", side, esp_err_to_name(e)); return false; }
    for (int b = 0; b < 2; b++) { fbuf[side][b] = (uint16_t *)heap_caps_aligned_alloc(16, 240 * 240 * 2, MALLOC_CAP_SPIRAM); if (!fbuf[side][b]) { logln("gfx: framebuffer alloc failed"); return false; }
      memset(fbuf[side][b], 0, 240 * 240 * 2); fbFree[side][b] = xSemaphoreCreateBinary(); xSemaphoreGive(fbFree[side][b]); }
    fbFront[side] = fbuf[side][0];
  }
  pinMode(TFT_DC, OUTPUT);
  gfxQ = xQueueCreate(6, sizeof(GfxJob));
  xTaskCreatePinnedToCore(gfxLoop, "gfx", 4096, NULL, 4, &gfxTask, 0);
  xTaskCreatePinnedToCore(oledLoop, "oled", 3072, NULL, 2, &oledTask, 0);
  gfxReady = true; logln("gfx: DMA pipeline up (2 panels + SD on SPI2)");
  return true;
}
// ===== UI frame (6.12.0): one footer line per app from a hint table, drawn on every left-dial screen at the
// same place in the same style; an app-name card for 0.9 s after a left-knob app change. =====
unsigned long appCardMs = 0;
unsigned long navMs = 0;   // 6.14.0: last knob activity; both OLEDs show the knob legend for 3 s after it
unsigned long brightMs = 0; int brightWhat = 0;   // 6.17.0: a brightness gesture is active (0 round displays, 1 rings); right dial + rings show the level
bool brightMode() { return brightMs && millis() - brightMs < 2500; }
// Paint into the back buffer and queue it. Waits only if the output task still owns that buffer.
void presentSide(int side, PaintFn paint) {
  if (!gfxReady) return;
  int b = fbBack[side]; BandCanvas *cv = side ? radarR : radarL;
  unsigned long t0 = micros(); xSemaphoreTake(fbFree[side][b], portMAX_DELAY); unsigned long t1 = micros();
  cv->c->setFb(fbuf[side][b]);
  radar = cv; paint(); if (side == 0 && paint != paintMessage) uiFrame(); paintToast(); radar = radarL;
  tWaitUs[side] += t1 - t0; tPaintUs[side] += micros() - t1; tFrames[side]++;
  GfxJob j = {0, (uint8_t)side, (uint8_t)b, 0, brightK * fadeK}; xQueueSend(gfxQ, &j, portMAX_DELAY);
  fbFront[side] = fbuf[side][b]; fbBack[side] = b ^ 1;
}
void present(PaintFn paint)  { currentPaint = paint;  presentSide(0, paint); }
void presentR(PaintFn paint) { currentPaintR = paint; presentSide(1, paint); }
const char *msgL1 = "", *msgL2 = "", *msgL3 = ""; int msgPct = -1;
void bootStop();
void showMessage(const char *a, const char *b, const char *c, int pct) { bootStop(); msgL1 = a; msgL2 = b; msgL3 = c; msgPct = pct; present(paintMessage); presentR(paintMessage); }

// =====================================================================
// LOG (serial + 2 KB ring served at /log)
// =====================================================================
#define LOG_RING 16384      // v6: internal heap has room (was 3 KB on v5)
char *logRing = nullptr; int logHead = 0;   // v6: 16 KB in PSRAM, allocated first thing in setup()
 bool logWrapped = false; bool logAtLineStart = true;
// fetchTask (core 0) and the UI task both log: the ring needs a lock, and each line is appended in one
// shot so the two tasks cannot interleave inside a line. The old version split the index increment from
// the bounds check, which allowed a one-byte write past logRing under preemption.
static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;
void logAppend(const char *s) {
  if (!logRing) return;
  size_t len = strlen(s); if (!len) return;
  if (len > LOG_RING) { s += len - LOG_RING; len = LOG_RING; }     // keep the tail of an over-long line
  portENTER_CRITICAL(&logMux);
  int h = logHead; if (h < 0 || h >= LOG_RING) h = 0;
  size_t first = (size_t)(LOG_RING - h); if (first > len) first = len;
  memcpy(logRing + h, s, first);
  if (len > first) { memcpy(logRing, s + first, len - first); logWrapped = true; h = (int)(len - first); }
  else { h += (int)len; if (h >= LOG_RING) { h = 0; logWrapped = true; } }
  logHead = h;
  portEXIT_CRITICAL(&logMux);
}
void logStampStr(char *st, size_t n) {   // "[12:34:56] " wall clock when NTP has it, else uptime
  struct tm t; time_t now = time(NULL);
  if (now > 1600000000) { localtime_r(&now, &t); snprintf(st, n, "[%02d:%02d:%02d] ", t.tm_hour, t.tm_min, t.tm_sec); }
  else { unsigned long ms = millis(); snprintf(st, n, "[+%lu.%03lu] ", ms / 1000, ms % 1000); }
}
void logf(const char *fmt, ...) {
  char line[248]; size_t p = 0;
  if (logAtLineStart) { logStampStr(line, 20); p = strlen(line); }
  va_list ap; va_start(ap, fmt); vsnprintf(line + p, sizeof(line) - p, fmt, ap); va_end(ap);
  Serial.print(line); logAppend(line);
  size_t n = strlen(line); logAtLineStart = n && line[n - 1] == '\n';
}
void logln(const char *s) {
  char line[248]; size_t p = 0;
  if (logAtLineStart) { logStampStr(line, 20); p = strlen(line); }
  snprintf(line + p, sizeof(line) - p, "%s\n", s);
  Serial.print(line); logAppend(line); logAtLineStart = true;
}

// =====================================================================
// SETTINGS (NVS)
// =====================================================================
#if UNITS_IMPERIAL
const int   RANGE_PRESETS[] = {5, 10, 25, 50, 100};
const char *RANGE_UNIT      = "mi";
const float KM_PER_UNIT     = 1.60934f;
#else
const int   RANGE_PRESETS[] = {10, 25, 50, 100, 200};
const char *RANGE_UNIT      = "km";
const float KM_PER_UNIT     = 1.0f;
#endif
const int NUM_RANGES = 5;
#define RANGE_AUTO NUM_RANGES            // rangeIndex == RANGE_AUTO: auto range

volatile double homeLat = 0, homeLon = 0;
volatile int rangeIndex   = 3;
volatile int dataSource   = DEFAULT_DATA_SOURCE;
volatile int filterMode   = 0;      // 0 all, 1 arrivals, 2 departures
int radarFocus = 0;                  // v6 RADAR mode: 0 ARRIVALS (right dial = landing flight, OLED = departures), 1 DEPARTURES (the reverse). NVS rfoc
volatile int nightMode    = 0;      // 0 auto, 1 on, 2 off
volatile int clockFace    = 0;
bool secRing = false;               // 6.20.0: seconds sweep on the right ring in every app (sring)
int clkRight = 0;                   // 6.18.0: right dial with the clock: 0 weather, 1 agenda, 2 world, 3 sun & moon, 4 traffic (clkr)
volatile int homeIndex    = 0;      // AIRPORTS index for the radar
volatile int boardAirport = 0;      // AIRPORTS index for the arrivals board
volatile int weatherIndex = -2;     // AIRPORTS index for the weather, -1 = same as the radar home, -2 = WX_HOME (your house)
double wxHomeLat = WX_HOME_LAT, wxHomeLon = WX_HOME_LON;
bool sweepOn = true;                // radar sweep lines
bool clock12 = false;               // 12-hour clock with AM/PM
bool popupsOn = false;              // alert banners over the current screen (emergency, alarm and timer-end always show)
int  oledScale = 0;                 // OLED body text size: 0 = 1x (13 px, 3 lines), 1 = 1.5x (22 px, 2 lines), 2 = 2x (26 px, 1 line), 3 = 3x (39 px, 1 line)
int locTarget = 0;                  // LOCATION app: 0 = radar home, 1 = weather location
int pomoPreset = 0, pomoCustomFocus = 30;
volatile int marketSel = 0;
int alarmMin = 7 * 60, alarmDays = 0x3E; bool alarmOn = false;   // days bitmask: bit0 Sun .. bit6 Sat
int sleepStart = 0, sleepEnd = 0;   // hours; equal = sleep schedule off
int nudgeMin = 45;                  // 0 = off
int quietStart = 22, quietEnd = 7;  // buzzer off between these hours (alarm and emergency still sound); equal = off
uint32_t alertMask = 0xFFFFFFFFu;   // bit per melody id: alert kind enabled
bool chimeOn = false;               // two-note chime on the hour
int slideMin = 0, slideMode = 0;    // idle minutes before slideshow (0) / return to radar (1); 0 = off
int theme = 0;                      // 0 green, 1 amber, 2 ice, 3 white
// v6 settings (6.7.0), all in the SETTINGS app and NVS
bool muteAll = false;               // every sound off except the alarm clock (NVS mute)
bool clickOn = true;                // knob click sound (clk)
bool piezoLoud = true;              // notes below 2.1 kHz played an octave up: a 12 mm piezo is far louder near 3 to 4 kHz (pzl)
int pzf(int f) { return (piezoLoud && f > 0 && f * 2 <= 4200) ? f * 2 : f; }
int  ringBright = 20;               // LED ring brightness cap, percent (ringb)
int  nightStart = 22, nightEnd = 7; // night dimming hours, equal = off (n0 / n1)
bool meetOn = true;                 // meeting reminders (mtg)
int  meetLead = 5;                  // first cue, minutes before (mtgl: 2 / 5 / 10 / 15)
bool meetTitles = true;             // show event titles, else "Meeting" (mtgt)
bool onAirOn = true;                // red rings while the Mac mic / camera is in use (onair)
bool micClick = true;               // on a call, left click mutes the Mac mic (micc)
uint32_t appMask = 0xFFFFFFFFu;     // bit per app: shown in the left-knob list (apps)
int  bootApp = 0;                   // app at power on (bapp)
bool focusSync = false;             // Pomodoro focus <-> Mac Focus mode (fsync)
int  nightRings = 1;                // rings during night dimming: 0 off, 1 nightlight (faint warm glow), 2 normal (nring)
bool lookupOn = true; int lookupKm = 3;   // "look up" alert when an aircraft passes within this distance of the house (lookup / lookkm)
bool issOn = true;                  // ISS pass alerts and the next pass on the Clock dial (iss)
int  setCat = -1, setSel = 0; bool setEdit = false;   // SETTINGS app state
char pomoTask[24] = "";             // what the focus session is for
int pomoTimerMin = 15;              // meeting / speaking timer preset
// runtime copies of the watch list and countdowns (editable on the web page, stored as strings in NVS)
#define MAX_WATCH 6
#define MAX_CDOWN 5
char watchList[MAX_WATCH][12]; int watchCount = 0;
struct CountdownR { char name[16]; int y, m, d, hh, mm; };
CountdownR cdowns[MAX_CDOWN]; int cdownCount = 0;
char watchStr[80] = "", cdownStr[160] = "";
struct DaySummary { uint16_t seen, arr, dep, enr, focusMin, outages; int8_t tmin, tmax; uint8_t aqiMax; };
DaySummary yesterday = {}; int summaryDay = -1;
char localUrl[96] = "";
volatile int activeSource = -1;
volatile bool settingsDirty = false;
unsigned long settingsDirtyMs = 0;
#define NUM_POMO 5

float currentRangeKm();
unsigned long now0() { return millis(); }
// "AIC504, -DXB, B77" -> watchList; empty string = defaults from FLIGHT_WATCH
void parseWatch() {
  watchCount = 0;
  if (!watchStr[0]) { for (int i = 0; i < NUM_WATCH && watchCount < MAX_WATCH; i++) strlcpy(watchList[watchCount++], FLIGHT_WATCH[i], 12); return; }
  char buf[80]; strlcpy(buf, watchStr, sizeof(buf));
  for (char *tok = strtok(buf, ",;"); tok && watchCount < MAX_WATCH; tok = strtok(NULL, ",;")) { while (*tok == ' ') tok++; char *e = tok + strlen(tok); while (e > tok && e[-1] == ' ') *--e = 0; if (*tok) strlcpy(watchList[watchCount++], tok, 12); }
}
// "Diwali=2026-11-08 00:00; New Year=2027-01-01" -> cdowns; empty = defaults from COUNTDOWNS
void parseCountdowns() {
  cdownCount = 0;
  if (!cdownStr[0]) { for (int i = 0; i < NUM_COUNTDOWNS && cdownCount < MAX_CDOWN; i++) { CountdownR &c = cdowns[cdownCount++]; strlcpy(c.name, COUNTDOWNS[i].name, 16); c.y = COUNTDOWNS[i].y; c.m = COUNTDOWNS[i].m; c.d = COUNTDOWNS[i].d; c.hh = COUNTDOWNS[i].hh; c.mm = COUNTDOWNS[i].mm; } return; }
  char buf[160]; strlcpy(buf, cdownStr, sizeof(buf));
  char *save = NULL;
  for (char *tok = strtok_r(buf, ";", &save); tok && cdownCount < MAX_CDOWN; tok = strtok_r(NULL, ";", &save)) {
    while (*tok == ' ') tok++;
    char *eq = strchr(tok, '='); if (!eq) continue; *eq = 0;
    CountdownR &c = cdowns[cdownCount]; strlcpy(c.name, tok, 16); c.hh = 0; c.mm = 0;
    if (sscanf(eq + 1, "%d-%d-%d %d:%d", &c.y, &c.m, &c.d, &c.hh, &c.mm) >= 3) cdownCount++;
  }
}
void applyHome() {
  int h = homeIndex; if (h < 0 || h >= NUM_AIRPORTS) h = 0;
  homeLat = AIRPORTS[h].lat; homeLon = AIRPORTS[h].lon;
}
void loadSettings() {
  prefs.begin("radar", true);
  rangeIndex = prefs.getInt("range", 3);
  dataSource = prefs.getInt("src", DEFAULT_DATA_SOURCE);
  filterMode = prefs.getInt("filter", 0);
  radarFocus = prefs.getInt("rfoc", 0) ? 1 : 0;
  nightMode  = prefs.getInt("night", 0);
  clockFace  = prefs.getInt("face", 0); clkRight = prefs.getInt("clkr", 0); if (clkRight < 0 || clkRight > 4) clkRight = 0; secRing = prefs.getBool("sring", false);
  homeIndex  = prefs.getInt("home", 0);
  boardAirport = prefs.getInt("board", 0);
  weatherIndex = prefs.getInt("wxloc", -2);
  wxHomeLat = prefs.getDouble("wxlat", WX_HOME_LAT); wxHomeLon = prefs.getDouble("wxlon", WX_HOME_LON);
  if (!isfinite(wxHomeLat) || fabs(wxHomeLat) > 90 || !isfinite(wxHomeLon) || fabs(wxHomeLon) > 180) { wxHomeLat = WX_HOME_LAT; wxHomeLon = WX_HOME_LON; }   // 6.14.1
  sweepOn = prefs.getBool("sweep", true);
  popupsOn = prefs.getBool("popup", false);
  clock12 = prefs.getBool("h12", false);
  oledScale = prefs.getInt("osz", 0); if (oledScale < 0 || oledScale > 3) oledScale = 0;
  pomoPreset = prefs.getInt("pomo", 0);
  pomoCustomFocus = prefs.getInt("pcus", 30);
  marketSel  = prefs.getInt("mkt", 0);
  tftBright  = prefs.getInt("tftb", 80);
  oledBright = prefs.getInt("oledb", 60);
  alarmMin   = prefs.getInt("almin", 7 * 60);
  alarmDays  = prefs.getInt("aldays", 0x3E);
  alarmOn    = prefs.getBool("alon", false);
  sleepStart = prefs.getInt("slp0", 0);
  sleepEnd   = prefs.getInt("slp1", 0);
  nudgeMin   = prefs.getInt("nudge", 45);
  quietStart = prefs.getInt("q0", 22); quietEnd = prefs.getInt("q1", 7);
  alertMask  = prefs.getUInt("amask", 0xFFFFFFFFu);
  chimeOn    = prefs.getBool("chime", false);
  slideMin   = prefs.getInt("slide", 0); slideMode = prefs.getInt("slmode", 0);
  theme      = prefs.getInt("theme", 0);
  pomoTimerMin = prefs.getInt("ptmr", 15);
  muteAll = prefs.getBool("mute", false); clickOn = prefs.getBool("clk", true); ringBright = prefs.getInt("ringb", 20); piezoLoud = prefs.getBool("pzl", true);
  nightStart = prefs.getInt("n0", 22); nightEnd = prefs.getInt("n1", 7);
  meetOn = prefs.getBool("mtg", true); meetLead = prefs.getInt("mtgl", 5); meetTitles = prefs.getBool("mtgt", true);
  onAirOn = prefs.getBool("onair", true); micClick = prefs.getBool("micc", true);
  appMask = prefs.getUInt("apps", 0xFFFFFFFFu); bootApp = prefs.getInt("bapp", 0); focusSync = prefs.getBool("fsync", false);
  nightRings = prefs.getInt("nring", 1); if (nightRings < 0 || nightRings > 2) nightRings = 1;
  lookupOn = prefs.getBool("lookup", true); lookupKm = prefs.getInt("lookkm", 3); if (lookupKm < 1 || lookupKm > 5) lookupKm = 3; issOn = prefs.getBool("iss", true);
  if (ringBright < 0 || ringBright > 100) ringBright = 20;
  if (nightStart < 0 || nightStart > 23 || nightEnd < 0 || nightEnd > 23) { nightStart = 22; nightEnd = 7; }
  if (meetLead != 2 && meetLead != 5 && meetLead != 10 && meetLead != 15) meetLead = 5;
  if (bootApp < 0 || bootApp > 31) bootApp = 0;   // clamped against APP_COUNT where it is used
  prefs.getString("ptask", pomoTask, sizeof(pomoTask));
  double lat = prefs.getDouble("lat", 0), lon = prefs.getDouble("lon", 0);
  if (!isfinite(lat) || !isfinite(lon) || fabs(lat) > 90 || fabs(lon) > 180) { lat = 0; lon = 0; }   // 6.14.1
  prefs.getString("lurl", localUrl, sizeof(localUrl));
  prefs.getString("watch", watchStr, sizeof(watchStr));
  prefs.getString("cdown", cdownStr, sizeof(cdownStr));
  size_t yl = prefs.getBytesLength("yday"); if (yl == sizeof(yesterday)) prefs.getBytes("yday", &yesterday, yl);
  prefs.end();
  parseWatch(); parseCountdowns();
  if (rangeIndex < 0 || rangeIndex > RANGE_AUTO) rangeIndex = 3;
  if (dataSource < 0 || dataSource > NUM_SOURCES) dataSource = DEFAULT_DATA_SOURCE;
  if (dataSource == 5) dataSource = 2;
  if (clockFace < 0 || clockFace >= NUM_FACES) clockFace = 0;
  if (homeIndex < 0 || homeIndex >= NUM_AIRPORTS) homeIndex = 0;
  if (boardAirport < 0 || boardAirport >= NUM_AIRPORTS) boardAirport = 0;
  if (weatherIndex < -2 || weatherIndex >= NUM_AIRPORTS) weatherIndex = -2;
  if (wxHomeLat == 0 || wxHomeLon == 0) { wxHomeLat = WX_HOME_LAT; wxHomeLon = WX_HOME_LON; }
  if (pomoPreset < 0 || pomoPreset >= NUM_POMO) pomoPreset = 0;
  if (pomoCustomFocus < 5 || pomoCustomFocus > 180) pomoCustomFocus = 30;
  if (tftBright < 10 || tftBright > 100) tftBright = 80;
  if (oledBright < 0 || oledBright > 100) oledBright = 60;
  if (alarmMin < 0 || alarmMin >= 1440) alarmMin = 7 * 60;
  if (sleepStart < 0 || sleepStart > 23 || sleepEnd < 0 || sleepEnd > 23) { sleepStart = 0; sleepEnd = 0; }
  if (quietStart < 0 || quietStart > 23 || quietEnd < 0 || quietEnd > 23) { quietStart = 22; quietEnd = 7; }
  if (theme < 0 || theme > 3) theme = 0;
  if (slideMin < 0 || slideMin > 240) slideMin = 0;
  if (pomoTimerMin < 1 || pomoTimerMin > 180) pomoTimerMin = 15;
  if (marketSel < 0) marketSel = 0;   // paintMarket clamps the upper end against marketPages(); a negative would index quotes[] and MARKET_SYMBOLS[] out of bounds
  applyHome();
  if (lat != 0 && lon != 0) { homeLat = lat; homeLon = lon; }   // web override of the home position
}
void saveSettings() {
  prefs.begin("radar", false);
  prefs.putInt("range", rangeIndex);
  prefs.putInt("src", dataSource);
  prefs.putInt("filter", filterMode);
  prefs.putInt("night", nightMode);
  prefs.putInt("face", clockFace); prefs.putInt("clkr", clkRight); prefs.putBool("sring", secRing);
  prefs.putInt("home", homeIndex);
  prefs.putInt("board", boardAirport);
  prefs.putInt("wxloc", weatherIndex);
  prefs.putDouble("wxlat", wxHomeLat); prefs.putDouble("wxlon", wxHomeLon);
  prefs.putBool("sweep", sweepOn);
  prefs.putInt("rfoc", radarFocus);
  prefs.putInt("osz", oledScale);
  prefs.putBool("popup", popupsOn);
  prefs.putBool("h12", clock12);
  prefs.putInt("pomo", pomoPreset);
  prefs.putInt("pcus", pomoCustomFocus);
  prefs.putInt("mkt", marketSel);
  prefs.putInt("tftb", tftBright);
  prefs.putInt("oledb", oledBright);
  prefs.putInt("almin", alarmMin);
  prefs.putInt("aldays", alarmDays);
  prefs.putBool("alon", alarmOn);
  prefs.putInt("slp0", sleepStart);
  prefs.putInt("slp1", sleepEnd);
  prefs.putInt("nudge", nudgeMin);
  prefs.putInt("q0", quietStart); prefs.putInt("q1", quietEnd);
  prefs.putUInt("amask", alertMask);
  prefs.putBool("chime", chimeOn);
  prefs.putInt("slide", slideMin); prefs.putInt("slmode", slideMode);
  prefs.putBool("mute", muteAll); prefs.putBool("clk", clickOn); prefs.putInt("ringb", ringBright); prefs.putBool("pzl", piezoLoud);
  prefs.putInt("n0", nightStart); prefs.putInt("n1", nightEnd);
  prefs.putBool("mtg", meetOn); prefs.putInt("mtgl", meetLead); prefs.putBool("mtgt", meetTitles);
  prefs.putBool("onair", onAirOn); prefs.putBool("micc", micClick);
  prefs.putUInt("apps", appMask); prefs.putInt("bapp", bootApp); prefs.putBool("fsync", focusSync); prefs.putInt("nring", nightRings);
  prefs.putBool("lookup", lookupOn); prefs.putInt("lookkm", lookupKm); prefs.putBool("iss", issOn);
  prefs.putInt("theme", theme);
  prefs.putInt("ptmr", pomoTimerMin);
  prefs.putString("ptask", pomoTask);
  putFocusPro();   // Pomodoro settings blob (5.5.0)
  prefs.putDouble("lat", homeLat);
  prefs.putDouble("lon", homeLon);
  prefs.putString("lurl", localUrl);
  prefs.putString("watch", watchStr);
  prefs.putString("cdown", cdownStr);
  prefs.end();
  settingsDirty = false;
}

// =====================================================================
// DATA
// =====================================================================
enum PlaneClass { CLS_ENROUTE = 0, CLS_ARRIVAL = 1, CLS_DEPARTURE = 2 };
enum IconKind : uint8_t { ICON_JET, ICON_PROP, ICON_HEAVY, ICON_HELI };
struct Plane {
  char   icao24[8];
  char   callsign[10];
  char   type[12];        // ICAO type code, e.g. A20N
  char   reg[10];
  char   squawk[5];
  double lat, lon;
  float  altitude_m, velocity_ms, heading_deg, distance_km, bearing_deg, vs_fpm;
  uint8_t cls, cat;       // cat: ADS-B emitter category A1..A7 as 1..7 (0 unknown)
  bool   mil;
};
#define MAX_PLANES 40
#define MAX_BOARD  20       // secondary list for an arrivals board airport other than home

struct Weather {
  bool  valid;
  float temp, hum, wind, feels, uv;
  int   wcode, aqi, pm25, pm10, windDir;
  float o3, no2, so2, co;         // ug/m3 (CO in ug/m3 too)
  int   sunriseMin, sunsetMin;
  float hourly[12];
  uint8_t rainProb[24];     // hourly precipitation probability from now
  float rain15[8];          // next 2 h, 15 min steps, mm
  int8_t dayMin[3], dayMax[3]; uint8_t dayCode[3];
};
struct Metar { bool valid; char raw[96]; int wdir, wspd, wgst, qnh, temp, dewp; char fltCat[6]; char visib[8]; unsigned long ms; };
struct Taf { bool valid; char raw[100]; int nextRwy; uint32_t changeT; int wdir, wspd; unsigned long ms; };
struct Sigmet { bool valid; char hazard[12]; char text[44]; uint32_t validTo; };
struct DestWx { bool valid; char code[4]; float temp; int wcode; double lat, lon; };
struct F1Info { bool valid; char race[30]; char circuit[30]; uint32_t t; char podium[3][14]; unsigned long ms; };

#define MKT_PTS 80
struct Quote { bool valid; float price, prevClose, high, low; unsigned long updatedMs; };   // resident: enough for alerts and the dashboard
float mktPtsCache[NUM_SYMBOLS][MKT_PTS]; volatile uint8_t mktNCache[NUM_SYMBOLS];   // chart points per symbol, kept outside the arena so they survive page changes
// UI-side snapshot of the selected symbol's chart, taken under dataMutex in syncFromFetchTask(). paintMarket()
// runs once per band (3x per frame) and must see the same points each time, so it never reads mktPtsCache directly.
float mktDrawPts[MKT_PTS]; uint8_t mktDrawN = 0; int mktDrawSym = -1;
struct MfNav { bool valid; char name[26]; float nav, prev; char date[12]; };
struct Quake { bool valid; float mag; char place[30]; uint32_t t; float distKm; };
#define MAX_NEWS 10
struct Headline { char text[128]; uint8_t feed; };   // v6: was 72, which cut BBC headlines mid-sentence
struct CricketMatch { bool valid; char name[40]; char status[48]; char score[2][24]; bool live; };

struct RouteEntry { char callsign[10]; char route[16]; float olat, olon, dlat, dlon; unsigned long ms; bool valid; };
#define MAX_ROUTES 24

struct Trail { char icao[8]; uint8_t n; float lat[TRAIL_LEN], lon[TRAIL_LEN], hdg[TRAIL_LEN], alt[TRAIL_LEN]; unsigned long lastMs; };

enum PomoState { POMO_SELECT, POMO_FOCUS, POMO_BREAK, POMO_PAUSED };
struct PomoPreset { uint8_t focus, brk; const char *name; };
const PomoPreset POMO_PRESETS[NUM_POMO] = {{25, 5, "classic"}, {50, 10, "long"}, {90, 20, "deep work"}, {0, 0, "custom"}, {0, 0, "timer"}};
#define POMO_TIMER_PRESET 4

// shared between the fetch task (writer) and the UI (reader): guard with dataMutex
// 6.6.0: the big tables live in PSRAM, bound as array references so sizeof() and all uses stay the same; frees
// ~27 KB of internal RAM for a second TLS session (the aircraft now have their own fetch task)
static inline void *psCalloc(size_t n, size_t sz, size_t align) {
  void *p = heap_caps_aligned_calloc(align, n, sz, MALLOC_CAP_SPIRAM); if (!p) p = heap_caps_aligned_calloc(align, n, sz, MALLOC_CAP_8BIT);
  if (!p) esp_system_abort("psCalloc failed");   // 6.14.1: an attributable panic instead of a null-pointer fault at the first use
  return p;
}
#define PS_ARRAY(T, name, N) T (&name)[N] = *reinterpret_cast<T (*)[N]>(psCalloc((N), sizeof(T), 8))
PS_ARRAY(Plane, sharedPlanes, MAX_PLANES); int sharedCount = 0; unsigned long sharedFetchMs = 0; bool sharedFetchOk = false; uint32_t sharedSeq = 0;
char          metaIcao[8] = ""; char metaType[24] = ""; char metaReg[10] = ""; bool metaDone = false;
Weather       sharedWx = {};
Metar         sharedMetar = {};
Taf           sharedTaf = {}; Sigmet sharedSigmet[2]; volatile bool sigmetDirty = false;
DestWx        sharedDestWx = {}; volatile bool destDirty = false;
F1Info        sharedF1 = {}; volatile bool f1Dirty = false;
int           sharedFng = -1; char sharedFngText[14] = "";
float         sharedRainMonth = -1, sharedRain7d = -1;
Quote         sharedQuotes[NUM_SYMBOLS]; volatile bool quotesDirty = false, marketForce = false;
Quake         sharedQuakes[3]; volatile bool quakeDirty = false;
PS_ARRAY(Headline, sharedNews, MAX_NEWS); int sharedNewsCount = 0; volatile bool newsDirty = false;
PS_ARRAY(RouteEntry, routes, MAX_ROUTES);   // fetch task copy (mutex)
PS_ARRAY(RouteEntry, routesUI, MAX_ROUTES); volatile bool routesDirty = false;   // UI copy
volatile bool netDirty = false;
SemaphoreHandle_t dataMutex;

// UI copies
Plane         planes[MAX_PLANES]; int planeCount = 0; unsigned long lastFetchMs = 0; bool fetchOk = false; uint32_t seenSeq = 0;   // planes: internal, read every radar frame
Weather       wx = {};
Metar         metar = {};
Taf           taf = {}; Sigmet sigmet[2]; DestWx destWx = {}; F1Info f1 = {}; int fng = -1; char fngText[14] = ""; float rainMonth = -1, rain7d = -1;
Quote         quotes[NUM_SYMBOLS];
Quake         quakes[3];
PS_ARRAY(Headline, news, MAX_NEWS); int newsCount = 0; unsigned long newsMs = 0;
Trail         trails[MAX_PLANES];    // trails: internal, read every radar frame

// ---- scratch arena: one static block owned by whichever app group needs big display-only data ----
// Switching groups zeroes it; the fetch task only fills the structure of the current owner.
enum { ARENA_NONE = -1, ARENA_AVIATION = 0, ARENA_BOARD, ARENA_MARKET, ARENA_NETWORK, ARENA_CRICKET };
#define REPLAY_TRACKS 16
#define REPLAY_LEN    48                      // one sample per minute -> 48 minutes of history
struct ReplayPt { int16_t dlat, dlon; };      // 10 m units from home
struct ReplayTrack { char icao[8]; uint8_t n, head; uint16_t lastMin; ReplayPt pts[REPLAY_LEN]; };
struct ArenaAviation { ReplayTrack tr[REPLAY_TRACKS]; unsigned long lastSampleMs; uint16_t minutes; };
struct ArenaBoard { Plane items[MAX_BOARD]; int count; unsigned long ms; uint32_t seq; };
#define NUM_HEAT NUM_HEAT_CFG
struct HeatItem { float price, prev; bool valid; };
struct ArenaMarket { HeatItem heat[NUM_HEAT]; MfNav mf[4]; unsigned long heatMs, mfMs; };
struct ArenaNetwork { uint16_t hist[3][120]; uint8_t idx; bool wrapped; };
struct ArenaCricket { CricketMatch m[2]; unsigned long ms; };
#define ARENA_BYTES 3400
PS_ARRAY(uint8_t, arena, ARENA_BYTES);
volatile int arenaOwner = ARENA_NONE;
#define ARENA_AV  ((ArenaAviation *)arena)
#define ARENA_BD  ((ArenaBoard *)arena)
#define ARENA_MK  ((ArenaMarket *)arena)
#define ARENA_NET ((ArenaNetwork *)arena)
#define ARENA_CR  ((ArenaCricket *)arena)
static_assert(sizeof(ArenaAviation) <= ARENA_BYTES, "arena too small for replay");
static_assert(sizeof(ArenaBoard) <= ARENA_BYTES, "arena too small for board");
static_assert(sizeof(ArenaMarket) <= ARENA_BYTES, "arena too small for market");
static_assert(sizeof(ArenaNetwork) <= ARENA_BYTES, "arena too small for network");
static_assert(sizeof(ArenaCricket) <= ARENA_BYTES, "arena too small for cricket");

void arenaAcquire(int group) {
  if (arenaOwner == group) return;
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  memset(arena, 0, ARENA_BYTES); arenaOwner = group;
  xSemaphoreGive(dataMutex);
  logf("arena -> %d\n", group);
}
// alerts / toast
struct Toast { char l1[22], l2[24]; uint16_t col; unsigned long untilMs; char icao[8]; };
Toast toast = {};
struct AlertLog { uint32_t t; char l1[22]; char l2[24]; };
AlertLog alertLog[10]; int alertLogCount = 0;
int  alertQueueMelody = -1;
char prevInner[MAX_PLANES][8]; int prevInnerCount = 0; bool alertArmed = false;
uint32_t alertedBits[8];      // per-icao one-shot alerts (bloom-style bitset by hash)
unsigned long lastRainAlertMs = 0, lastSunsetAlertDay = 0, lastGoldenAlertDay = 0, lastNudgeMs = 0, lastQuakeAlertT = 0;
uint32_t marketAlertedDay = 0; uint8_t marketAlerted = 0;

// stats (today)
uint8_t  hourArr[24], hourDep[24], hourEnr[24];
uint32_t seenBits[64];        // 2048-bit set of icao hashes seen today
int      seenToday = 0, seenArr = 0, seenDep = 0, seenEnr = 0, statsDay = -1;
float    recHighFt = 0, recFastKt = 0, recFarKm = 0; char recHighCs[10] = "", recFastCs[10] = "", recHeavyCs[10] = "", recHeavyType[8] = "";
uint64_t airlinesSeen = 0; char typesSeen[24][6]; int typesSeenCount = 0; int holdingCount = 0;
int8_t   tempHist[96]; uint8_t aqiHist[96]; int histIdx = 0; bool histWrapped = false; unsigned long lastHistMs = 0;

// pomodoro
PomoState pomoState = POMO_SELECT; bool pomoEditing = false, pomoWasFocus = true;
unsigned long pomoEndMs = 0, pomoTotalMs = 0, pomoRemainAtPause = 0;
int pomoSession = 0, pomoToday = 0, pomoTodayMin = 0, pomoDay = -1, nudgesToday = 0;
struct PomoDay { uint16_t day; uint8_t sessions; uint16_t minutes; };
PomoDay pomoHist[7];
// focus log queue: kept in flash until the web app confirms, so sessions done offline are sent later
struct FocusRec { uint32_t start, end; uint16_t planMin, actMin; uint8_t done; char task[24]; };
#define FOCUS_Q 16
FocusRec focusQ[FOCUS_Q]; uint8_t focusQn = 0; volatile bool focusQDirty = false;
uint32_t focusStartEpoch = 0; uint16_t focusPlanMin = 0;
bool pomoAskLog = false; uint8_t pomoAskSel = 0; unsigned long pomoLastTapMs = 0;   // end-of-session prompt: 0 = yes, 1 = no
#define POMO_DOUBLE_TAP_MS 450
// ---- 5.5.0 Focus Pro: settings, stats from the sheet, celebration overlays ----
struct PomoCfg { uint16_t goalMin; uint8_t longBreak, cycle, autoStart, quiet, early, minLog, streakWk, recapHour, anim, sound, greet; char name[13]; };
PomoCfg pcfg = {180, 15, 4, 1, 3, 0, 1, 0, 19, 0, 0, 1, USER_NAME_DEFAULT};
// quiet 0 off 1 planes 2 planes+market 3 all | early 0 ask 1 always log 2 never | recapHour 0 = off | anim 0 full 1 subtle 2 off | sound 0 chime 1 soft 2 silent
struct FocusStats { bool valid; uint16_t day, todayMin, sess, done, longest, week, prev, best, avg, totalH; uint8_t donePct; uint16_t streak, bestStreak; int8_t hour; char bestDay[4]; uint16_t last7[7]; };
FocusStats fstats = {};
volatile bool statsWanted = true;
struct FocusToday { uint16_t day, mins, sess; };
FocusToday ftoday = {};                                   // everything logged today (done + partial), kept on the device
uint8_t heldPlanes = 0, heldMkt = 0, heldOther = 0;       // alerts held during the current focus session
unsigned long pomoPausedAtMs = 0;
bool pomoSettings = false, psetEdit = false; int psetSel = 0;
enum { OV_NONE, OV_GREET, OV_SESSION, OV_GOAL, OV_STREAK, OV_RECAP };
struct Overlay { uint8_t kind; unsigned long startMs, durMs; char l1[36], l2[28], l3[32]; uint16_t a, b; };
Overlay ov = {}; Overlay ovQ[3]; uint8_t ovQn = 0;
unsigned long animT = 0;                                  // ms into the current animation, fixed for all bands of a frame
extern int clkS, clkFaceDrawn;                            // defined with the clock renderer further down

// apps / ui state
enum Mode { MODE_ZOOM, MODE_SELECT };
enum App  { APP_RADAR = 0, APP_ARRIVALS, APP_APPROACH, APP_REPLAY, APP_CLOCK, APP_WEATHER, APP_POMODORO, APP_MARKET, APP_NETWORK, APP_STATS, APP_NEWS, APP_CRICKET, APP_F1, APP_SYSTEM, APP_BRIGHT, APP_LOCATION, APP_AGENDA, APP_SETTINGS, APP_MEDIA, APP_COUNT };   // v6: AGENDA added at the end (indices stay)
const char *APP_NAMES[APP_COUNT] = {"RADAR", "ARRIVALS", "APPROACH", "REPLAY", "CLOCK", "WEATHER", "POMODORO", "MARKETS", "NETWORK", "STATS", "NEWS", "CRICKET", "F1", "SYSTEM", "DISPLAY", "LOCATION", "AGENDA", "SETTINGS", "MEDIA"};
Mode  currentMode = MODE_ZOOM;
App   currentApp  = APP_RADAR;
bool  menuOpen = false; int menuSel = 0;
int   selectedIndex = 0; char selectedIcao[8] = ""; char followIcao[8] = "";
volatile char wantedIcao[8] = ""; volatile char wantedCallsign[10] = ""; volatile double wantedLat = 0, wantedLon = 0;
volatile double wantedDestLat = 0, wantedDestLon = 0; volatile char wantedDestCode[4] = "";
float sweepAngle = 0; bool staticDirty = true; bool isNight = false; bool sleeping = false; unsigned long wakeUntilMs = 0;
unsigned long lastHudMs = 0, lastFrameMs = 0, lastInfoMs = 0, lastClockMs = 0;
int fpsShown = 0; int subPage = 0; int listSel = 0; int brightSel = 0; int newsScroll = 0;
uint16_t heapHist[48]; uint8_t fpsHist[48]; int heapHistN = 0; unsigned long lastHeapSampleMs = 0;   // 30-min samples, 24 h
struct Roam { uint32_t t; uint8_t mac[3]; }; Roam roamLog[8]; int roamCount = 0; uint8_t lastBssid[6] = {0};
int replayPos = 0; unsigned long replayLastMs = 0; bool replayPlay = true;  // REPLAY app playback (pos = minutes ago)
bool breathing = false; unsigned long breathStartMs = 0;                 // breathing exercise during a break
bool pomoIsTimer = false;                                                // meeting timer run (no session counting)
unsigned long lastInputMs = 0; bool slideshow = false; int slideIdx = 0; unsigned long slideSwitchMs = 0; App slideReturnApp = APP_RADAR;
float fadeK = 1.0f; int lastChimeHour = -1; unsigned long lastRotMs = 0;
int resetReasonCode = 0;
int activeRwy = -1;           // runway number in use (e.g. 9 or 27), -1 unknown
uint8_t rwyVotes[10]; int rwyVoteIdx = 0;
bool alarmRinging = false; unsigned long alarmSnoozeUntil = 0, alarmRingStartMs = 0; int alarmFiredMin = -1;
unsigned long bootMs = 0;

// WiFi
volatile bool wifiUp = false; volatile int lastDiscReason = 0; unsigned long lastReconnectMs = 0; bool haveCreds = false;

// encoders (v6): PCNT hardware counters via ESP32Encoder, 4 counts per detent. Like the v5 ISR, a step is
// emitted once 4 counts have accumulated since the last step, so a half-turned knob does not flicker.
struct Knob { long lastRaw; int acc; bool down; unsigned long downMs; bool longFired; unsigned long lastClickMs; };
Knob knobs[2] = { {0, 0, false, 0, false, 0}, {0, 0, false, 0, false, 0} };   // 0 left, 1 right
volatile int knobInject[2] = {0, 0};   // /knob test aid (6.7.0): detents injected over WiFi
int knobSteps(int k) {
  long raw = ENC_SIGN * (long)(k ? encR.getCount() : encL.getCount());
  knobs[k].acc += (int)(raw - knobs[k].lastRaw) + knobInject[k] * ENC_COUNTS_PER_DETENT; knobs[k].lastRaw = raw; knobInject[k] = 0;
  int steps = 0;
  while (knobs[k].acc >= ENC_COUNTS_PER_DETENT)  { steps++; knobs[k].acc -= ENC_COUNTS_PER_DETENT; }
  while (knobs[k].acc <= -ENC_COUNTS_PER_DETENT) { steps--; knobs[k].acc += ENC_COUNTS_PER_DETENT; }
  return steps;
}

// =====================================================================
// HELPERS: geo, colours, time, text
// =====================================================================
double toRad(double d) { return d * M_PI / 180.0; }
float distanceKm(double lat1, double lon1, double lat2, double lon2) {
  double dLat = toRad(lat2 - lat1), dLon = toRad(lon2 - lon1);
  double a = sin(dLat/2)*sin(dLat/2) + cos(toRad(lat1))*cos(toRad(lat2))*sin(dLon/2)*sin(dLon/2);
  return (float)(6371.0 * 2.0 * atan2(sqrt(a), sqrt(1 - a)));
}
float bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double y = sin(toRad(lon2 - lon1)) * cos(toRad(lat2));
  double x = cos(toRad(lat1))*sin(toRad(lat2)) - sin(toRad(lat1))*cos(toRad(lat2))*cos(toRad(lon2 - lon1));
  return (float)fmod(atan2(y, x) * 180.0 / M_PI + 360.0, 360.0);
}
float angleDiff(float a, float b) { float d = fmodf(fabsf(a - b), 360.0f); return d > 180 ? 360 - d : d; }
float signedAngleDiff(float from, float to) { float d = fmodf(to - from + 540.0f, 360.0f) - 180.0f; return d; }
float currentRangeKm() { return (rangeIndex == RANGE_AUTO ? RANGE_PRESETS[3] : RANGE_PRESETS[rangeIndex]) * KM_PER_UNIT; }
float autoRangeKm = 0;   // chosen by auto range
float displayRangeKm() { return (rangeIndex == RANGE_AUTO && autoRangeKm > 0) ? autoRangeKm : currentRangeKm(); }
int homeElevM() { return AIRPORTS[homeIndex].elevM; }
int rwyHdg() { return AIRPORTS[homeIndex].rwyHdg; }

uint16_t COL_BG, COL_RING, COL_RING_DIM, COL_TEXT, COL_TEXT_DIM, COL_HOME, COL_SWEEP, COL_SELECTED,
         COL_ENROUTE_LOW, COL_ENROUTE_HIGH, COL_ARR, COL_DEP, COL_OK, COL_BAD, COL_MENU_BG, COL_PANEL, COL_LINE;
uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) { return radar->color565(r, g, b); }
uint16_t dim565(uint16_t c, float k) {
  if (k > 1) k = 1; if (k < 0) k = 0;
  uint8_t r = (c >> 11) & 0x1F, g = (c >> 5) & 0x3F, b = c & 0x1F;
  return (uint16_t)(((uint8_t)(r*k) << 11) | ((uint8_t)(g*k) << 5) | (uint8_t)(b*k));
}
void applyBrightness() {
  // night (NIGHT_START_H..NIGHT_END_H, or forced from the web page): round at 60%, OLED contrast capped but never off.
  // Adafruit's dim(true) sets contrast 0, which reads as "OLED switched off", so it is not used.
  brightK = tftBright / 100.0f * (isNight ? 0.6f : 1.0f);
  int v = oledBright * 255 / 100; if (isNight && v > 60) v = 60; if (v < 12) v = 12;
  // 6.17.0: SETCONTRAST alone barely changes these panels; the precharge period and VCOMH level set most of the brightness
  uint8_t pre = (uint8_t)(((1 + v * 14 / 255) << 4) | 1), vcom = v < 86 ? 0x00 : v < 171 ? 0x20 : 0x40;
  Adafruit_SSD1306 *oleds[2] = {&infoDisplay, &infoR};
  for (int i = 0; i < 2; i++) { Adafruit_SSD1306 &o = *oleds[i]; o.ssd1306_command(SSD1306_SETCONTRAST); o.ssd1306_command((uint8_t)v); o.ssd1306_command(SSD1306_SETPRECHARGE); o.ssd1306_command(pre); o.ssd1306_command(SSD1306_SETVCOMDETECT); o.ssd1306_command(vcom); }
}
const char *THEME_NAMES[4] = {"green", "amber", "ice", "white"};
void applyPalette() {
  COL_BG = rgb(0, 0, 0); COL_TEXT = rgb(200, 200, 200); COL_TEXT_DIM = rgb(100, 100, 100); COL_HOME = rgb(70, 150, 255);
  COL_SELECTED = rgb(255, 70, 70); COL_ARR = rgb(255, 150, 0); COL_DEP = rgb(255, 240, 60); COL_OK = rgb(0, 200, 0); COL_BAD = rgb(220, 0, 0);
  COL_PANEL = rgb(45, 45, 45); COL_LINE = rgb(60, 60, 60);
  switch (theme) {
    case 1: COL_RING = rgb(150, 90, 0); COL_RING_DIM = rgb(70, 42, 0); COL_SWEEP = rgb(255, 170, 40); COL_ENROUTE_LOW = rgb(255, 200, 90); COL_ENROUTE_HIGH = rgb(255, 230, 170); COL_MENU_BG = rgb(16, 10, 2); COL_ARR = rgb(255, 110, 40); COL_DEP = rgb(255, 250, 120); break;
    case 2: COL_RING = rgb(0, 90, 150); COL_RING_DIM = rgb(0, 45, 75); COL_SWEEP = rgb(80, 200, 255); COL_ENROUTE_LOW = rgb(120, 220, 255); COL_ENROUTE_HIGH = rgb(200, 240, 255); COL_MENU_BG = rgb(4, 10, 16); break;
    case 3: COL_RING = rgb(120, 120, 120); COL_RING_DIM = rgb(55, 55, 55); COL_SWEEP = rgb(235, 235, 235); COL_ENROUTE_LOW = rgb(200, 200, 200); COL_ENROUTE_HIGH = rgb(255, 255, 255); COL_MENU_BG = rgb(12, 12, 12); break;
    default: COL_RING = rgb(0, 110, 0); COL_RING_DIM = rgb(0, 55, 0); COL_SWEEP = rgb(40, 255, 110); COL_ENROUTE_LOW = rgb(60, 255, 120); COL_ENROUTE_HIGH = rgb(80, 220, 255); COL_MENU_BG = rgb(8, 14, 10); break;
  }
}
uint16_t planeColour(const Plane &p) {
  if (p.cls == CLS_ARRIVAL)   return COL_ARR;
  if (p.cls == CLS_DEPARTURE) return COL_DEP;
  return (p.altitude_m * 3.28084f < 25000) ? COL_ENROUTE_LOW : COL_ENROUTE_HIGH;
}
uint8_t classify(const Plane &p) {
  if (p.altitude_m * 3.28084f >= 10000 || p.velocity_ms < 20) return CLS_ENROUTE;
  float toHome = bearingDeg(p.lat, p.lon, homeLat, homeLon);
  float d = angleDiff(p.heading_deg, toHome);
  if (d < 50)  return CLS_ARRIVAL;
  if (d > 130) return CLS_DEPARTURE;
  return CLS_ENROUTE;
}
const char *sourceName(int s) {
  switch (s) { case 0: return "OpenSky"; case 1: return "airplanes.live"; case 2: return "adsb.lol"; case 3: return "adsb.fi"; case 4: return "local"; case 5: return "auto"; default: return "none"; }
}
bool passesFilter(const Plane &p) {
  if (filterMode == 1) return p.cls == CLS_ARRIVAL;
  if (filterMode == 2) return p.cls == CLS_DEPARTURE;
  return true;
}
SemaphoreHandle_t tzMutex;   // 6.14.1: zoneTick() switches the process TZ for a moment; every local-time read waits for it
bool getTime(struct tm &t) { xSemaphoreTake(tzMutex, portMAX_DELAY); bool ok = getLocalTime(&t, 10); xSemaphoreGive(tzMutex); return ok; }
// UTC civil time -> epoch, no timezone involved (days-from-civil)
uint32_t timegmUtc(int y, int m, int d, int hh, int mm) {
  y -= m <= 2; int era = (y >= 0 ? y : y - 399) / 400; int yoe = y - era * 400;
  int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1; int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = (long)era * 146097 + doe - 719468;
  return (uint32_t)(days * 86400L + hh * 3600L + mm * 60L);
}
// World clock cache: zone times are computed once a minute in the UI task, so the timezone is only ever
// switched in one task and only briefly (the fetch task never touches TZ).
struct ZoneTime { int8_t hour, min, wday; } zoneT[NUM_ZONES]; unsigned long zoneTickMs = 0;
void zoneTick() {
  if (zoneTickMs && millis() - zoneTickMs < 30000UL) return; zoneTickMs = millis();
  time_t now = time(NULL);
  xSemaphoreTake(tzMutex, portMAX_DELAY);
  for (int i = 0; i < NUM_ZONES; i++) { setenv("TZ", ZONES[i].tz, 1); tzset(); struct tm z; localtime_r(&now, &z); zoneT[i].hour = z.tm_hour; zoneT[i].min = z.tm_min; zoneT[i].wday = z.tm_wday; }
  setenv("TZ", LOCAL_TZ, 1); tzset();
  xSemaphoreGive(tzMutex);
}
bool computeNight() {
  if (nightMode == 1) return true;
  if (nightMode == 2) return false;
  struct tm t; if (!getTime(t)) return false;
  if (nightStart == nightEnd) return false;   // 6.7.0: hours from SETTINGS > DISPLAY
  return nightStart > nightEnd ? (t.tm_hour >= nightStart || t.tm_hour < nightEnd) : (t.tm_hour >= nightStart && t.tm_hour < nightEnd);
}
int minuteOfDay() { struct tm t; if (!getTime(t)) return -1; return t.tm_hour * 60 + t.tm_min; }
bool marketOpenNow(struct tm &t) {
  if (t.tm_wday == 0 || t.tm_wday == 6) return false;
  int m = t.tm_hour * 60 + t.tm_min; return m >= 9 * 60 + 15 && m <= 15 * 60 + 30;
}
const char *iataFor(const char *icao) { for (int i = 0; i < NUM_APT; i++) if (!strcmp(APT[i].icao, icao)) return APT[i].iata; return icao; }
void prettyRoute(const char *raw, char *out, size_t n) {   // "VIDP-VOBL" -> "DEL-BLR"
  out[0] = 0; char buf[32]; strlcpy(buf, raw, sizeof(buf));
  char *tok = strtok(buf, "-");
  while (tok) { if (out[0]) strlcat(out, "-", n); strlcat(out, iataFor(tok), n); tok = strtok(NULL, "-"); }
}
const AirlineRow *airlineFor(const char *callsign) {
  if (strlen(callsign) < 4 || !isalpha((unsigned char)callsign[0]) || !isalpha((unsigned char)callsign[1]) || !isalpha((unsigned char)callsign[2])) return NULL;
  for (int i = 0; i < NUM_AIRLINES; i++) if (!strncmp(AIRLINES[i].icao, callsign, 3)) return &AIRLINES[i];
  return NULL;
}
void prettyCallsign(const char *cs, char *out, size_t n) {   // "IGO2134" -> "6E 2134"
  const AirlineRow *a = airlineFor(cs);
  if (a && a->iata[0]) snprintf(out, n, "%s %s", a->iata, cs + 3); else strlcpy(out, cs, n);
}
const char *routeFor(const char *callsign) {   // UI side
  for (int i = 0; i < MAX_ROUTES; i++) if (routesUI[i].valid && !strcmp(routesUI[i].callsign, callsign)) return routesUI[i].route;
  return NULL;
}
void fmtPrice(float v, char *out, size_t n, int dec) {   // 25412.3 -> 25,412.30
  char raw[24]; snprintf(raw, sizeof(raw), "%.*f", dec, v);
  char *dot = strchr(raw, '.'); int ilen = dot ? (int)(dot - raw) : (int)strlen(raw);
  size_t o = 0;
  for (int i = 0; i < ilen && o < n - 1; i++) { out[o++] = raw[i]; int left = ilen - 1 - i; if (left > 0 && left % 3 == 0 && o < n - 1 && raw[i] != '-') out[o++] = ','; }
  if (dot) for (char *p = dot; *p && o < n - 1; p++) out[o++] = *p;
  out[o] = 0;
}
uint32_t icaoHash(const char *icao) { uint32_t h = 2166136261u; while (*icao) { h ^= (uint8_t)*icao++; h *= 16777619u; } return h; }
bool hashBitTest(uint32_t *set, int bits, uint32_t h) { int i = h % bits; return set[i >> 5] & (1u << (i & 31)); }
void hashBitSet(uint32_t *set, int bits, uint32_t h) { int i = h % bits; set[i >> 5] |= (1u << (i & 31)); }

// =====================================================================
// FETCHERS (run on the fetch task, core 0)
// =====================================================================
PS_ARRAY(Plane, stage, MAX_PLANES);
int lastHttpCode = 0;
String openskyToken; unsigned long tokenExpiresAtMs = 0;
SemaphoreHandle_t feedMutex;   // 6.14.1: fetchArea() (stage[], the OpenSky token, the per-source backoff tables) is used by planesTask and fetchTask

// ArduinoJson treats an empty receive buffer as end-of-input. This wrapper
// makes read() wait (up to 4 s) for the next bytes so slow TLS streams parse.
class PatientStream : public Stream {
 public:
  PatientStream(Stream &s) : _s(s) {}
  int available() override { return _s.available(); }
  int peek() override { if (!wait()) return -1; return _s.peek(); }
  int read() override { if (!wait()) return -1; return _s.read(); }
  size_t write(uint8_t) override { return 0; }
 private:
  Stream &_s;
  bool wait() { unsigned long t0 = millis(); while (!_s.available()) { if (millis() - t0 > 4000) return false; delay(1); } return true; }
};
void skipWs(PatientStream &ps) { int pk; while ((pk = ps.peek()) == ' ' || pk == '\n' || pk == '\r' || pk == '\t') ps.read(); }

bool fetchFocusStats() {   // network task: today, week, streaks and the last 7 days, computed by the sheet
  JsonDocument d; d["token"] = FOCUS_LOG_TOKEN; d["action"] = "stats"; d["goal"] = pcfg.goalMin; d["wk"] = pcfg.streakWk;
  String body; serializeJson(d, body);
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http;
  httpBegin(http, tls, FOCUS_LOG_URL); http.setTimeout(15000);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  String resp = code == 200 ? http.getString() : String("");
  http.end();
  JsonDocument r;
  if (code != 200 || deserializeJson(r, resp) || !(r["ok"] | false)) { logf("focus stats HTTP %d %s\n", code, resp.substring(0, 60).c_str()); return false; }
  FocusStats s = {}; s.valid = true; s.day = todayKey();
  s.todayMin = r["today"] | 0; s.sess = r["sess"] | 0; s.done = r["done"] | 0; s.longest = r["longest"] | 0;
  s.week = r["week"] | 0; s.prev = r["prev"] | 0; s.best = r["best"] | 0; s.avg = r["avg"] | 0; s.totalH = r["totalH"] | 0;
  s.donePct = r["donePct"] | 0; s.streak = r["streak"] | 0; s.bestStreak = r["bestStreak"] | 0; s.hour = r["hour"] | -1;
  strlcpy(s.bestDay, r["bestDay"] | "", sizeof(s.bestDay));
  JsonArray l7 = r["last7"]; int k = 0; for (JsonVariant v : l7) { if (k < 7) s.last7[k++] = v.as<uint16_t>(); }
  xSemaphoreTake(dataMutex, portMAX_DELAY); fstats = s; xSemaphoreGive(dataMutex);
  staticDirty = true;
  logf("focus stats: today %u  streak %u  best %u\n", s.todayMin, s.streak, s.bestStreak);
  return true;
}
bool sendFocusLog() {   // network task: send the oldest queued session; true when the web app confirmed it
  FocusRec r; xSemaphoreTake(dataMutex, portMAX_DELAY); bool have = focusQn > 0; if (have) r = focusQ[0]; xSemaphoreGive(dataMutex);
  if (!have) return false;
  String body;
  { JsonDocument d;                                        // scoped so the document is freed before the TLS connect
    d["token"] = FOCUS_LOG_TOKEN; d["id"] = String(r.start); d["start"] = r.start; d["end"] = r.end;
    d["plan"] = r.planMin; d["min"] = r.actMin; d["done"] = r.done ? true : false; d["task"] = r.task;
    serializeJson(d, body); }
  // Apps Script runs doPost to completion, writing the sheet row and the calendar event, and only then
  // answers 302 pointing at script.googleusercontent.com to deliver the return value. That host's TLS
  // handshake fails on this ESP32. Measured 9 Oct at the same instant with the same 31,732 byte block:
  // script.googleusercontent.com FAILs in 213 ms while script.google.com connects in 911 ms, and DNS
  // (8 ms) and raw TCP 443 (14 ms) both succeed, so it is host specific and not a heap problem.
  // A 302 therefore means the session was logged. Accept it and do not chase the unreachable second leg.
  String resp; int code; bool redirected = false;
  {
    WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http;
    httpBegin(http, tls, FOCUS_LOG_URL); http.setTimeout(15000);
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.addHeader("Content-Type", "application/json");
    code = http.POST(body);
    if (code == 200) resp = http.getString();
    else if (code == 301 || code == 302 || code == 303 || code == 307 || code == 308) redirected = true;
    http.end();
  }
  bool ok = resp.indexOf("\"ok\":true") >= 0 || redirected;
  logf("focus log send HTTP %d %s\n", code,
       ok ? (redirected ? "ok (302, script ran)" : "ok") : resp.substring(0, 60).c_str());
  if (!ok) return false;
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  if (focusQn > 0 && focusQ[0].start == r.start) { memmove(&focusQ[0], &focusQ[1], sizeof(FocusRec) * (FOCUS_Q - 1)); focusQn--; focusQDirty = true; }
  statsWanted = true;
  xSemaphoreGive(dataMutex);
  return true;
}
String getOpenSkyToken() {
  if (openskyToken.length() > 0 && (long)(millis() - tokenExpiresAtMs) < 0) return openskyToken;
  if (!strcmp(OPENSKY_CLIENT_ID, "YOUR_OPENSKY_CLIENT_ID")) return "";
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http;
  httpBegin(http, tls, "https://auth.opensky-network.org/auth/realms/opensky-network/protocol/openid-connect/token");
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.setTimeout(8000);
  String body = "grant_type=client_credentials&client_id=" + String(OPENSKY_CLIENT_ID) + "&client_secret=" + String(OPENSKY_CLIENT_SECRET);
  int code = http.POST(body);
  if (code == 200) {
    JsonDocument doc;
    if (!deserializeJson(doc, http.getString())) {
      openskyToken = doc["access_token"].as<String>();
      long expiresIn = doc["expires_in"] | 1800;
      tokenExpiresAtMs = millis() + (unsigned long)(expiresIn - 60) * 1000UL;
    }
  } else { logf("OpenSky token HTTP %d\n", code); openskyToken = ""; }
  http.end();
  return openskyToken;
}

void finishPlane(Plane &p, double clat, double clon, float rangeKm, int &n) {
  for (int i = (int)strlen(p.callsign) - 1; i >= 0 && p.callsign[i] == ' '; i--) p.callsign[i] = 0;
  if (p.callsign[0] == 0) strlcpy(p.callsign, p.icao24, sizeof(p.callsign));
  p.distance_km = distanceKm(clat, clon, p.lat, p.lon);
  p.bearing_deg = bearingDeg(clat, clon, p.lat, p.lon);
  if (p.distance_km > rangeKm) return;
  p.cls = classify(p);
  n++;
}

int fetchOpenSky(double clat, double clon, float rangeKm, Plane *out, int maxN) {
  double latPad = rangeKm / 111.0, lonPad = rangeKm / (111.0 * cos(toRad(clat)) + 0.0001);
  String url = "https://opensky-network.org/api/states/all?lamin=" + String(clat - latPad, 4) + "&lomin=" + String(clon - lonPad, 4) +
               "&lamax=" + String(clat + latPad, 4) + "&lomax=" + String(clon + lonPad, 4);
  String token = getOpenSkyToken();
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http; http.useHTTP10(true); httpBegin(http, tls, url); http.setTimeout(8000);
  if (token.length()) http.addHeader("Authorization", "Bearer " + token);
  int code = httpGet(http, "OpenSky"); netResult(code);
  lastHttpCode = code;
  if (code != 200) { http.end(); return -1; }
  PatientStream ps(http.getStream()); ps.setTimeout(4000);
  if (!ps.find((char*)"\"states\"") || !ps.find((char*)"[")) { http.end(); return 0; }
  int n = 0;
    for (;;) {
    skipWs(ps); int pk = ps.peek();
    if (pk == ']' || pk < 0) break;
    JsonDocument sd;
    if (deserializeJson(sd, ps)) break;
    JsonArray s = sd.as<JsonArray>();
    bool more = ps.findUntil((char*)",", (char*)"]");
    if (n >= maxN) { if (!more) break; else continue; }
    if (s.size() < 11 || s[5].isNull() || s[6].isNull() || (!s[8].isNull() && s[8].as<bool>())) { if (!more) break; else continue; }
    Plane &p = out[n]; memset(&p, 0, sizeof(Plane));
    strlcpy(p.icao24, s[0] | "", sizeof(p.icao24));
    strlcpy(p.callsign, s[1] | "", sizeof(p.callsign));
    p.lon = s[5].as<double>(); p.lat = s[6].as<double>();
    p.altitude_m  = s[7].isNull() ? 0 : s[7].as<float>();
    p.velocity_ms = s[9].isNull() ? 0 : s[9].as<float>();
    p.heading_deg = s[10].isNull() ? 0 : s[10].as<float>();
    p.vs_fpm = (s.size() > 11 && !s[11].isNull()) ? s[11].as<float>() * 196.85f : 0;
    if (s.size() > 14 && !s[14].isNull()) strlcpy(p.squawk, s[14] | "", sizeof(p.squawk));
    finishPlane(p, clat, clon, rangeKm, n);
    if (!more) break;
  }
  http.end();
  return n;
}

// One place that logs every outbound request: tag, result, duration, heap and largest free block on failure
// 6.13.4: the Arduino core's hostByName() calls lwIP dns_clear_cache() from the calling task whenever it sees the
// interface IP state change (first lookup after GOT_IP, every reconnect). With CONFIG_LWIP_CHECK_THREAD_SAFETY that
// asserts in udp_remove() if the other task has a lookup in flight (crash seen: planes task in getOpenSkyToken while
// fetchTask resolved). HTTPClient resolves internally, so instead every request first resolves the host here under
// dnsMutex; the lwIP cache then answers HTTPClient's own lookup without a second query or a cache clear.
SemaphoreHandle_t dnsMutex;
bool dnsLookup(const char *host, IPAddress &ip, uint32_t waitMs) {
  if (dnsMutex && xSemaphoreTake(dnsMutex, waitMs == portMAX_DELAY ? portMAX_DELAY : pdMS_TO_TICKS(waitMs)) != pdTRUE) return false;   // 6.14.1: loop() callers give up instead of blocking
  bool ok = WiFi.hostByName(host, ip) == 1;
  if (dnsMutex) xSemaphoreGive(dnsMutex);
  return ok;
}
bool httpBegin(HTTPClient &http, NetworkClient &client, const String &url) {
  int a = url.indexOf("://"); a = a < 0 ? 0 : a + 3; int e = a;
  while (e < (int)url.length() && url[e] != '/' && url[e] != ':' && url[e] != '?') e++;
  String host = url.substring(a, e); IPAddress ip;
  if (host.length()) dnsLookup(host.c_str(), ip);
  return http.begin(client, url);
}
int httpGet(HTTPClient &http, const char *tag) {
  unsigned long t0 = millis(); int code = http.GET(); unsigned long dt = millis() - t0;
  if (code == 200) logf("%s ok %lu ms heap %u block %u\n", tag, dt, ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  else if (code < 0) logf("%s FAIL %d %s after %lu ms heap %u block %u\n", tag, code, http.errorToString(code).c_str(), dt, ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  else logf("%s HTTP %d %lu ms\n", tag, code, dt);
  return code;
}
// Consecutive-failure tracker for the aircraft feeds: diagnoses, then resets the TLS client, then WiFi, then the board
int netFailRun = 0; volatile bool wantReconnect = false;   // 6.14.1: the reconnect runs in loop() (these tasks have PSRAM stacks)
void netResult(int code) {
  if (code > 0) { netFailRun = 0; return; }
  if ((uint32_t)WiFi.localIP() == 0) return;   // not connected yet, not a real failure
  netFailRun++;
  if (netFailRun == 3 || netFailRun % 10 == 0) {
    IPAddress r; unsigned long t0 = millis(); bool dns = dnsLookup("api.adsb.lol", r);
    logf("net check: wifi %d rssi %d ip %s gw %s dns %s %s %lu ms heap %u largest block %u fails %d\n", (int)WiFi.status(), WiFi.RSSI(), WiFi.localIP().toString().c_str(), WiFi.gatewayIP().toString().c_str(),
         dns ? "ok" : "FAIL", dns ? r.toString().c_str() : "", millis() - t0, ESP.getFreeHeap(), ESP.getMaxAllocHeap(), netFailRun);
  }
  if (netFailRun == 8) { logln("net: 8 failures in a row, asking loop() to reconnect WiFi"); wantReconnect = true; }
  if (netFailRun >= 24 && millis() - bootMs > 300000UL) { logln("net: 24 failures in a row, restarting"); delay(300); ESP.restart(); }
}
// airplanes.live, adsb.lol, adsb.fi and a local readsb all use the readsb JSON format
int fetchReadsb(double clat, double clon, float rangeKm, int src, Plane *out, int maxN) {
  float radiusNm = rangeKm / 1.852f; if (radiusNm > 250) radiusNm = 250;
  String url;
  if (src == 1)      url = "https://api.airplanes.live/v2/point/" + String(clat, 4) + "/" + String(clon, 4) + "/" + String((int)ceilf(radiusNm));
  else if (src == 2) url = "https://api.adsb.lol/v2/point/" + String(clat, 4) + "/" + String(clon, 4) + "/" + String((int)ceilf(radiusNm));
  else if (src == 3) url = "https://opendata.adsb.fi/api/v3/lat/" + String(clat, 4) + "/lon/" + String(clon, 4) + "/dist/" + String((int)ceilf(radiusNm));
  else { if (!localUrl[0]) return -1; url = localUrl; }
  bool https = url.startsWith("https://");
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http;
  http.useHTTP10(true);
  WiFiClient plainClient;   // 6.14.1: per request (a global one was shared by the two network tasks)
  if (https) httpBegin(http, tls, url); else httpBegin(http, plainClient, url);
  http.setUserAgent("DeskRadar/6 (personal desk display; Bengaluru)");   // 6.5.5: addHeader("User-Agent") is silently dropped by HTTPClient; adsb.lol answered the default "ESP32HTTPClient" with 403
  http.setTimeout(15000);
  int code = httpGet(http, sourceName(src)); netResult(code);
  lastHttpCode = code;
  if (code != 200) { http.end(); return -1; }
  int bodyLen = http.getSize();
  // one aircraft object at a time straight from the stream: no large JSON document
  JsonDocument filter;
  filter["hex"] = true; filter["flight"] = true; filter["lat"] = true; filter["lon"] = true; filter["alt_baro"] = true;
  filter["gs"] = true; filter["track"] = true; filter["t"] = true; filter["baro_rate"] = true; filter["squawk"] = true;
  filter["category"] = true; filter["dbFlags"] = true; filter["r"] = true; filter["seen_pos"] = true;
  PatientStream ps(http.getStream()); ps.setTimeout(4000);
  const char *key = (src == 4) ? "\"aircraft\"" : "\"ac\"";
  if (!ps.find((char*)key) || !ps.find((char*)"[")) { logln("JSON: no aircraft array"); http.end(); return -1; }
  int n = 0, seen = 0;
  for (;;) {
    skipWs(ps); int pk = ps.peek();
    if (pk == ']' || pk < 0) break;
    JsonDocument a;
    DeserializationError err = deserializeJson(a, ps, DeserializationOption::Filter(filter));
    if (err) { logf("JSON element %d: %s\n", seen, err.c_str()); break; }
    seen++;
    bool more = ps.findUntil((char*)",", (char*)"]");
    if (n >= maxN) { if (!more) break; else continue; }
    if (a["lat"].isNull() || a["lon"].isNull() || a["alt_baro"].is<const char*>()) { if (!more) break; else continue; }
    if ((a["seen_pos"] | 0.0f) > 20.0f) { if (!more) break; else continue; }   // 6.6.0: last position too old (landed below coverage): drop, do not freeze it on final
    Plane &p = out[n]; memset(&p, 0, sizeof(Plane));
    strlcpy(p.icao24, a["hex"] | "", sizeof(p.icao24));
    strlcpy(p.callsign, a["flight"] | "", sizeof(p.callsign));
    strlcpy(p.type, a["t"] | "", sizeof(p.type));
    strlcpy(p.reg, a["r"] | "", sizeof(p.reg));
    strlcpy(p.squawk, a["squawk"] | "", sizeof(p.squawk));
    const char *cat = a["category"] | ""; if (cat[0] == 'A' && cat[1] >= '1' && cat[1] <= '7') p.cat = cat[1] - '0';
    p.mil = (a["dbFlags"] | 0) & 1;
    p.lat = a["lat"].as<double>(); p.lon = a["lon"].as<double>();
    p.altitude_m  = (a["alt_baro"] | 0.0f) * 0.3048f;
    p.velocity_ms = (a["gs"] | 0.0f) * 0.514444f;
    p.heading_deg = a["track"] | 0.0f;
    p.vs_fpm = a["baro_rate"] | 0.0f;
    finishPlane(p, clat, clon, rangeKm, n);
    if (!more) break;
  }
  http.end();
  logf("%s: %d aircraft in feed, %d in range, heap %u block %u\n", sourceName(src), seen, n, ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  if (seen == 0) logf("  query centre %.4f,%.4f radius %d nm, body %d bytes, filter %d (0 all 1 arr 2 dep)\n", clat, clon, (int)ceilf(radiusNm), bodyLen, filterMode);
  return n;
}

// per-source minimum interval and backoff on 403/429
unsigned long srcLastTry[NUM_SOURCES] = {0}, srcBackoffUntil[NUM_SOURCES] = {0};
uint8_t srcFails[NUM_SOURCES] = {0};
bool fetchForce = false;
uint32_t fetchCount = 0, fetchFailCount = 0;
bool srcBackedOff(int src) { return (long)(srcBackoffUntil[src] - millis()) > 0; }
int fetchFrom(int src, double clat, double clon, float rangeKm, Plane *out, int maxN) {
  if (src == 4 && !localUrl[0]) return -1;
  unsigned long now = millis();
  if (srcBackedOff(src)) return -1;
  if (!fetchForce && srcLastTry[src] && now - srcLastTry[src] < SRC_MIN_MS[src]) return -1;
  srcLastTry[src] = now; lastHttpCode = 0;
  int n = (src == 0) ? fetchOpenSky(clat, clon, rangeKm, out, maxN) : fetchReadsb(clat, clon, rangeKm, src, out, maxN);
  if (n >= 0) { srcFails[src] = 0; return n; }
  if (lastHttpCode == 403 || lastHttpCode == 429) {
    if (srcFails[src] < 5) srcFails[src]++;
    unsigned long back = 60000UL << (srcFails[src] - 1);
    srcBackoffUntil[src] = now + back;
    logf("%s rate limited (HTTP %d), backing off %lu s\n", sourceName(src), lastHttpCode, back / 1000);
  }
  return -1;
}
int fetchArea(double clat, double clon, float rangeKm, Plane *out, int maxN, int &used, int pref) {
  int n = -1; used = -1;
  const int order[NUM_SOURCES] = {pref, 4, 3, 2, 1};
  for (int i = 0; i < NUM_SOURCES && n < 0; i++) {
    int src = order[i]; bool dup = false;
    for (int j = 0; j < i; j++) if (order[j] == src) dup = true;
    if (dup) continue;
    n = fetchFrom(src, clat, clon, rangeKm, out, maxN);
    if (n >= 0) used = src;
  }
  if (n < 0 && dataSource != 0) { n = fetchFrom(0, clat, clon, rangeKm, out, maxN); if (n >= 0) used = 0; }
  return n;
}
void sortByDistance(Plane *arr, int n) {
  for (int i = 1; i < n; i++) { Plane key = arr[i]; int j = i - 1; while (j >= 0 && arr[j].distance_km > key.distance_km) { arr[j + 1] = arr[j]; j--; } arr[j + 1] = key; }
}
bool fetchPlanes() {
  if (!wifiUp) return false;
  xSemaphoreTake(feedMutex, portMAX_DELAY);
  int used; int n = fetchArea(homeLat, homeLon, currentRangeKm(), stage, MAX_PLANES, used, dataSource == 5 ? 2 : dataSource);
  fetchCount++;
  if (n < 0) { activeSource = -1; fetchFailCount++; xSemaphoreGive(feedMutex); return false; }
  activeSource = used;
  sortByDistance(stage, n);
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  memcpy(sharedPlanes, stage, sizeof(Plane) * n);
  sharedCount = n; sharedFetchMs = millis(); sharedFetchOk = true; sharedSeq++;
  xSemaphoreGive(dataMutex);
  xSemaphoreGive(feedMutex);
  logf("Planes in range: %d\n", n);
  return true;
}
bool fetchBoard() {   // arrivals board for an airport other than home (arena: ARENA_BOARD)
  if (!wifiUp || arenaOwner != ARENA_BOARD) return false;
  const Airport &ap = AIRPORTS[boardAirport];
  int used;   // prefer the source the radar is not using, so neither gets rate limited
  xSemaphoreTake(feedMutex, portMAX_DELAY);
  int n = fetchArea(ap.lat, ap.lon, 120, stage, MAX_BOARD, used, activeSource == 3 ? 2 : 3);
  if (n < 0) { xSemaphoreGive(feedMutex); return false; }
  // re-classify relative to the board airport (classify() uses home): heading toward / away from the airport
  for (int i = 0; i < n; i++) {
    Plane &p = stage[i];
    if (p.altitude_m * 3.28084f >= 12000 || p.velocity_ms < 20) { p.cls = CLS_ENROUTE; continue; }
    float d = angleDiff(p.heading_deg, bearingDeg(p.lat, p.lon, ap.lat, ap.lon));
    p.cls = d < 50 ? CLS_ARRIVAL : d > 130 ? CLS_DEPARTURE : CLS_ENROUTE;
  }
  sortByDistance(stage, n);
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  if (arenaOwner == ARENA_BOARD) { memcpy(ARENA_BD->items, stage, sizeof(Plane) * n); ARENA_BD->count = n; ARENA_BD->ms = millis(); ARENA_BD->seq++; staticDirty = true; }
  xSemaphoreGive(dataMutex);
  xSemaphoreGive(feedMutex);
  return true;
}

// routes for every arrival/departure in one adsb.lol routeset call
unsigned long lastRoutesetMs = 0;
unsigned long routesetFailMs = 0, routesetBackoffMs = 0;   // adsb.lol answers 403 persistently; do not burn a TLS handshake every cycle
void fetchRoutes() {
  if (routesetBackoffMs && millis() - routesetFailMs < routesetBackoffMs) return;
  char want[12][10]; double wlat[12], wlon[12]; int nw = 0;
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  for (int pass = 0; pass < 2 && nw < 12; pass++) {
    for (int i = 0; i < sharedCount && nw < 12; i++) {
      const Plane &p = sharedPlanes[i];
      if (pass == 0 && p.cls == CLS_ENROUTE) continue;
      if (!p.callsign[0] || !strcmp(p.callsign, p.icao24) || !isalpha((unsigned char)p.callsign[0])) continue;
      bool have = false; for (int r = 0; r < MAX_ROUTES; r++) if (routes[r].valid && !strcmp(routes[r].callsign, p.callsign)) { have = true; break; }
      for (int k = 0; k < nw && !have; k++) if (!strcmp(want[k], p.callsign)) have = true;
      if (!have) { strlcpy(want[nw], p.callsign, 10); wlat[nw] = p.lat; wlon[nw] = p.lon; nw++; }
    }
    for (int i = 0; arenaOwner == ARENA_BOARD && i < ARENA_BD->count && nw < 12; i++) {
      const Plane &p = ARENA_BD->items[i];
      if (p.cls == CLS_ENROUTE || !p.callsign[0] || !strcmp(p.callsign, p.icao24) || !isalpha((unsigned char)p.callsign[0])) continue;
      bool have = false; for (int r = 0; r < MAX_ROUTES; r++) if (routes[r].valid && !strcmp(routes[r].callsign, p.callsign)) { have = true; break; }
      for (int k = 0; k < nw && !have; k++) if (!strcmp(want[k], p.callsign)) have = true;
      if (!have) { strlcpy(want[nw], p.callsign, 10); wlat[nw] = p.lat; wlon[nw] = p.lon; nw++; }
    }
  }
  xSemaphoreGive(dataMutex);
  if (nw == 0) return;
  // v6 (6.1.7): adsb.lol routeset refused every call with HTTP 403 "User-Agent too generic; include valid contact
  // info" (the ESP32 default UA), and with a proper UA it still had no route for local flights. adsbdb.com
  // answers per callsign (GET, no key), e.g. IGO1076 = HKT-BLR. Up to 3 lookups per pass; a callsign it does not
  // know (404) is stored with an empty route so it is not asked again.
  int got = 0, asked = 0;
  for (int i = 0; i < nw && asked < 3; i++) {
    asked++;
    RouteEntry e = {}; strlcpy(e.callsign, want[i], sizeof(e.callsign));
    int rc;
    {
      WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http; http.useHTTP10(true);
      httpBegin(http, tls, String("https://api.adsbdb.com/v0/callsign/") + want[i]); http.setTimeout(8000);
      http.setUserAgent("DeskRadar/6 (ESP32)");
      rc = http.GET();
      if (rc == 200) {
        JsonDocument filter;
        for (const char *end : {"origin", "destination"}) {
          filter["response"]["flightroute"][end]["iata_code"] = true; filter["response"]["flightroute"][end]["icao_code"] = true;
          filter["response"]["flightroute"][end]["latitude"] = true;  filter["response"]["flightroute"][end]["longitude"] = true;
        }
        JsonDocument d; PatientStream ps(http.getStream()); ps.setTimeout(4000);
        if (!deserializeJson(d, ps, DeserializationOption::Filter(filter))) {
          JsonObject o = d["response"]["flightroute"]["origin"], de = d["response"]["flightroute"]["destination"];
          const char *oc = o["iata_code"] | (const char *)(o["icao_code"] | ""), *dc = de["iata_code"] | (const char *)(de["icao_code"] | "");
          if (oc[0] && dc[0]) { char codes[24]; snprintf(codes, sizeof(codes), "%s-%s", oc, dc); prettyRoute(codes, e.route, sizeof(e.route)); got++; }
          e.olat = o["latitude"] | 0.0f; e.olon = o["longitude"] | 0.0f; e.dlat = de["latitude"] | 0.0f; e.dlon = de["longitude"] | 0.0f;
        }
      }
      http.end();
    }
    lastRoutesetMs = millis();
    if (rc == 429 || rc < 0) {
      routesetFailMs = millis();
      routesetBackoffMs = routesetBackoffMs ? (routesetBackoffMs < 1800000UL ? routesetBackoffMs * 2 : 1800000UL) : 300000UL;
      logf("adsbdb HTTP %d, backing off %lu s\n", rc, routesetBackoffMs / 1000);
      break;
    }
    routesetBackoffMs = 0;
    if (rc != 200 && rc != 404) { logf("adsbdb %s HTTP %d\n", want[i], rc); continue; }   // not stored: asked again later
    e.valid = true; e.ms = millis();
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    int slot = -1, oldest = 0;
    for (int r = 0; r < MAX_ROUTES; r++) { if (routes[r].valid && !strcmp(routes[r].callsign, e.callsign)) { slot = r; break; } if (!routes[r].valid) { if (slot < 0) slot = r; } else if (routes[r].ms < routes[oldest].ms) oldest = r; }
    if (slot < 0) slot = oldest;
    routes[slot] = e; routesDirty = true;
    xSemaphoreGive(dataMutex);
  }
  logf("routes (adsbdb): %d asked, %d known, %d waiting\n", asked, got, nw - asked);
}

// type + registration for the selected aircraft when the feed had none
void lookupMeta(const char *icao, bool needType) {
  char type[24] = "", reg[10] = "";
  if (needType) {
    WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http;
    httpBegin(http, tls, String("https://hexdb.io/api/v1/aircraft/") + icao); http.setTimeout(5000);
    if (httpGet(http, "hexdb") == 200) {
      JsonDocument doc;
      if (!deserializeJson(doc, http.getString())) {
        if (doc["Manufacturer"].is<const char*>()) snprintf(type, sizeof(type), "%s %s", doc["Manufacturer"] | "", doc["Type"] | "");
        strlcpy(reg, doc["Registration"] | "", sizeof(reg));
      }
    }
    http.end();
  }
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  strlcpy(metaIcao, icao, sizeof(metaIcao)); strlcpy(metaType, type, sizeof(metaType)); strlcpy(metaReg, reg, sizeof(metaReg)); metaDone = true;
  xSemaphoreGive(dataMutex);
}

String urlEnc(const char *v) { String o = v; o.replace("/", "%2F"); o.replace(" ", "%20"); return o; }
double weatherLat() { return weatherIndex >= 0 ? AIRPORTS[weatherIndex].lat : weatherIndex == -2 ? wxHomeLat : homeLat; }
double weatherLon() { return weatherIndex >= 0 ? AIRPORTS[weatherIndex].lon : weatherIndex == -2 ? wxHomeLon : homeLon; }
const char *weatherPlace() { return weatherIndex == -2 ? WX_HOME_NAME : AIRPORTS[weatherIndex >= 0 ? weatherIndex : homeIndex].name; }
// LOCATION picker rows: for the weather target row 0 is the home entry, airports follow
int locCount() { return locTarget ? NUM_AIRPORTS + 1 : NUM_AIRPORTS; }
int locAirport(int row) { return locTarget ? row - 1 : row; }          // -1 = the home row
int locRowFor(int wxIdx) { return wxIdx == -2 ? 0 : (wxIdx == -1 ? homeIndex : wxIdx) + 1; }
void fetchWeather() {
  if (!wifiUp) return;
  Weather w = {}; w.sunriseMin = -1; w.sunsetMin = -1;
  double wla = weatherLat(), wlo = weatherLon();
  // v6 (6.3.5): start from the last air quality so a failed air-quality call does not show AQI 0 ("good")
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  w.aqi = sharedWx.aqi; w.pm25 = sharedWx.pm25; w.pm10 = sharedWx.pm10; w.o3 = sharedWx.o3; w.no2 = sharedWx.no2; w.so2 = sharedWx.so2; w.co = sharedWx.co;
  xSemaphoreGive(dataMutex);
  {
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http;   // own TLS session: this request and the air-quality one below are different hosts
  String u1 = "https://api.open-meteo.com/v1/forecast?latitude=" + String(wla, 4) + "&longitude=" + String(wlo, 4) +
              "&current=temperature_2m,relative_humidity_2m,weather_code,wind_speed_10m,wind_direction_10m,apparent_temperature,uv_index" +
              "&hourly=temperature_2m,precipitation_probability&forecast_hours=30&minutely_15=precipitation&forecast_minutely_15=8" +
              "&daily=sunrise,sunset,temperature_2m_max,temperature_2m_min,weather_code&forecast_days=3&timezone=" + urlEnc(LOCAL_TZ_IANA);
  http.useHTTP10(true); httpBegin(http, tls, u1); http.setTimeout(10000);
  logf("weather: %s %.3f %.3f heap %u block %u\n", weatherPlace(), wla, wlo, ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  int code = httpGet(http, "weather");
  if (code == 200) {
    JsonDocument filter;
    filter["current"] = true; filter["daily"] = true; filter["minutely_15"]["precipitation"] = true;
    filter["hourly"]["temperature_2m"] = true; filter["hourly"]["precipitation_probability"] = true;
    JsonDocument doc;   // hourly arrays start at the current hour (forecast_hours), 30 values each
    PatientStream ps(http.getStream()); ps.setTimeout(4000);
    DeserializationError e = deserializeJson(doc, ps, DeserializationOption::Filter(filter));
    if (!e) {
      JsonObject c = doc["current"];
      w.temp = c["temperature_2m"] | 0.0f; w.hum = c["relative_humidity_2m"] | 0.0f; w.wind = c["wind_speed_10m"] | 0.0f;
      w.windDir = c["wind_direction_10m"] | 0; w.wcode = c["weather_code"] | 0; w.feels = c["apparent_temperature"] | w.temp; w.uv = c["uv_index"] | 0.0f; w.valid = true;
      const char *sr = doc["daily"]["sunrise"][0] | ""; const char *ss = doc["daily"]["sunset"][0] | "";
      if (strlen(sr) >= 16) w.sunriseMin = atoi(sr + 11) * 60 + atoi(sr + 14);
      if (strlen(ss) >= 16) w.sunsetMin  = atoi(ss + 11) * 60 + atoi(ss + 14);
      for (int d = 0; d < 3; d++) { w.dayMax[d] = (int8_t)lroundf(doc["daily"]["temperature_2m_max"][d] | 0.0f); w.dayMin[d] = (int8_t)lroundf(doc["daily"]["temperature_2m_min"][d] | 0.0f); w.dayCode[d] = doc["daily"]["weather_code"][d] | 0; }
      JsonArray ht = doc["hourly"]["temperature_2m"].as<JsonArray>(), hp = doc["hourly"]["precipitation_probability"].as<JsonArray>();
      for (int i = 0; i < 12; i++) w.hourly[i] = (i < (int)ht.size()) ? (ht[i] | w.temp) : w.temp;
      for (int i = 0; i < 24; i++) { int v = (i < (int)hp.size()) ? (hp[i] | 0) : 0; w.rainProb[i] = v < 0 ? 0 : v > 100 ? 100 : v; }
      JsonArray m15 = doc["minutely_15"]["precipitation"].as<JsonArray>();
      for (int i = 0; i < 8; i++) w.rain15[i] = (i < (int)m15.size()) ? (m15[i] | 0.0f) : 0;
    } else logf("weather parse: %s (doc overflow %d)\n", e.c_str(), doc.overflowed() ? 1 : 0);
  } else logf("weather HTTP %d (%s)\n", code, code < 0 ? http.errorToString(code).c_str() : "");
  http.end();
  }
  {
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http;
  String u2 = "https://air-quality-api.open-meteo.com/v1/air-quality?latitude=" + String(wla, 4) + "&longitude=" + String(wlo, 4) + "&current=pm2_5,pm10,us_aqi,ozone,nitrogen_dioxide,sulphur_dioxide,carbon_monoxide";
  httpBegin(http, tls, u2); http.setTimeout(8000);
  if (httpGet(http, "weather") == 200) {
    JsonDocument doc;
    if (!deserializeJson(doc, http.getString())) {
      JsonObject c = doc["current"];
      w.pm25 = (int)(c["pm2_5"] | 0.0f); w.pm10 = (int)(c["pm10"] | 0.0f); w.aqi = c["us_aqi"] | 0;
      w.o3 = c["ozone"] | 0.0f; w.no2 = c["nitrogen_dioxide"] | 0.0f; w.so2 = c["sulphur_dioxide"] | 0.0f; w.co = c["carbon_monoxide"] | 0.0f;
    }
  }
  http.end();
  }
  if (!w.valid) { logln("weather: forecast failed, keeping the previous one"); return; }   // v6: a blip no longer blanks the app
  xSemaphoreTake(dataMutex, portMAX_DELAY); sharedWx = w; xSemaphoreGive(dataMutex);
  logf("Weather %.1fC AQI %d rain15 %.1f\n", w.temp, w.aqi, w.rain15[0]);
}

void fetchMetar() {
  if (!wifiUp) return;
  Metar m = {};
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http; http.useHTTP10(true);
  httpBegin(http, tls, String("https://aviationweather.gov/api/data/metar?ids=") + AIRPORTS[homeIndex].icao + "&format=json"); http.setTimeout(8000);
  int code = httpGet(http, "metar");
  if (code == 200) {
    JsonDocument doc;   // stream it: getString() held the whole body on the heap alongside the parsed document
    PatientStream ps(http.getStream()); ps.setTimeout(4000);
    if (!deserializeJson(doc, ps) && doc.is<JsonArray>() && doc.size() > 0) {
      JsonObject o = doc[0];
      strlcpy(m.raw, o["rawOb"] | "", sizeof(m.raw));
      m.wdir = o["wdir"].is<int>() ? (o["wdir"] | 0) : -1;    // "VRB" -> -1
      m.wspd = o["wspd"] | 0; m.wgst = o["wgst"] | 0; m.qnh = (int)lroundf(o["altim"] | 0.0f); m.temp = (int)lroundf(o["temp"] | 0.0f); m.dewp = (int)lroundf(o["dewp"] | 0.0f);
      strlcpy(m.fltCat, o["fltCat"] | "", sizeof(m.fltCat));
      if (o["visib"].is<const char*>()) strlcpy(m.visib, o["visib"] | "", sizeof(m.visib)); else snprintf(m.visib, sizeof(m.visib), "%.0f", o["visib"] | 0.0f);
      m.valid = true; m.ms = millis();
    }
  } else logf("METAR HTTP %d\n", code);
  http.end();
  if (!m.valid) return;   // 6.14.1: keep the last good METAR on a failed fetch
  xSemaphoreTake(dataMutex, portMAX_DELAY); sharedMetar = m; xSemaphoreGive(dataMutex);
  if (m.valid) logf("METAR %s\n", m.raw);
}

const char *firFor(const char *iata) {
  static const char *VOMF[] = {"BLR","HAL","MAA","HYD","COK","TRV","CCJ","IXE","IXM","TRZ","CJB","VTZ","VGA","TIR","MYQ","HBX","IXZ"};
  static const char *VABF[] = {"BOM","PNQ","AMD","GOI","GOX","IDR","NAG","BHO","UDR","RPR"};
  static const char *VIDF[] = {"DEL","JAI","LKO","ATQ","IXC","SXR","IXL","VNS"};
  static const char *VECF[] = {"CCU","GAU","BBI","PAT","IXB"};
  for (auto s : VOMF) if (!strcmp(s, iata)) return "VOMF";
  for (auto s : VABF) if (!strcmp(s, iata)) return "VABF";
  for (auto s : VIDF) if (!strcmp(s, iata)) return "VIDF";
  for (auto s : VECF) if (!strcmp(s, iata)) return "VECF";
  if (!strcmp(iata, "DXB")) return "OMAE";
  if (!strcmp(iata, "SIN")) return "WSJC";
  return "VOMF";
}
void fetchTaf() {   // forecast METAR: first period whose wind points to the other runway
  if (!wifiUp) return;
  Taf t = {};
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http; http.useHTTP10(true);
  httpBegin(http, tls, String("https://aviationweather.gov/api/data/taf?ids=") + AIRPORTS[homeIndex].icao + "&format=json"); http.setTimeout(10000);
  int code = httpGet(http, "taf");
  if (code == 200) {
    JsonDocument filter;
    filter[0]["rawTAF"] = true; filter[0]["fcsts"][0]["timeFrom"] = true; filter[0]["fcsts"][0]["timeTo"] = true; filter[0]["fcsts"][0]["wdir"] = true; filter[0]["fcsts"][0]["wspd"] = true;
    JsonDocument doc;
    PatientStream ps(http.getStream()); ps.setTimeout(4000);
    if (!deserializeJson(doc, ps, DeserializationOption::Filter(filter)) && doc.is<JsonArray>() && doc.size() > 0) {
      strlcpy(t.raw, doc[0]["rawTAF"] | "", sizeof(t.raw)); t.valid = true; t.ms = millis();
      uint32_t nowT = (uint32_t)time(NULL); int rh = rwyHdg(); int cur = activeRwy > 0 ? activeRwy : rh / 10;
      for (JsonObject f : doc[0]["fcsts"].as<JsonArray>()) {
        uint32_t from = f["timeFrom"] | 0u; int wd = f["wdir"].is<int>() ? (f["wdir"] | 0) : -1, ws = f["wspd"] | 0;
        if (from < nowT - 3600 || wd < 0 || ws < 5) continue;
        int r = (angleDiff(wd, rh) < 90) ? rh / 10 : ((rh + 180) % 360) / 10;
        if (r != cur) { t.nextRwy = r; t.changeT = from; t.wdir = wd; t.wspd = ws; break; }
      }
    }
  } else logf("TAF HTTP %d\n", code);
  http.end();
  if (!t.valid) return;   // 6.14.1: keep the last good TAF
  xSemaphoreTake(dataMutex, portMAX_DELAY); sharedTaf = t; xSemaphoreGive(dataMutex);
  if (t.valid) logf("TAF ok, next rwy %d at %lu\n", t.nextRwy, (unsigned long)t.changeT);
}
void fetchSigmet() {   // international SIGMETs for our FIR
  if (!wifiUp) return;
  Sigmet out[2] = {}; int n = 0; const char *fir = firFor(AIRPORTS[homeIndex].iata);
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http; http.useHTTP10(true);
  httpBegin(http, tls, "https://aviationweather.gov/api/data/isigmet?format=json"); http.setTimeout(15000);
  int code = httpGet(http, "sigmet");
  if (code == 200) {
    JsonDocument filter;
    filter["firId"] = true; filter["hazard"] = true; filter["qualifier"] = true; filter["validTimeTo"] = true; filter["rawSigmet"] = true;
    PatientStream ps(http.getStream()); ps.setTimeout(4000);
    if (ps.find((char*)"[")) {
      for (int k = 0; k < 400 && n < 2; k++) {
        skipWs(ps); int pk = ps.peek(); if (pk == ']' || pk < 0) break;
        JsonDocument d;
        if (deserializeJson(d, ps, DeserializationOption::Filter(filter))) break;
        bool more = ps.findUntil((char*)",", (char*)"]");
        const char *f = d["firId"] | "";
        if (!strcmp(f, fir)) {
          Sigmet &sg = out[n++]; sg.valid = true;
          snprintf(sg.hazard, sizeof(sg.hazard), "%s %s", d["qualifier"] | "", d["hazard"] | ""); sg.validTo = d["validTimeTo"] | 0u;
          const char *raw = d["rawSigmet"] | ""; const char *p = strstr(raw, "FIR "); strlcpy(sg.text, p ? p + 4 : raw, sizeof(sg.text));
        }
        if (!more) break;
      }
    }
  } else logf("SIGMET HTTP %d\n", code);
  http.end();
  if (code != 200) return;   // 6.14.1: keep the last SIGMET list on a failed fetch
  xSemaphoreTake(dataMutex, portMAX_DELAY); memcpy(sharedSigmet, out, sizeof(out)); sigmetDirty = true; xSemaphoreGive(dataMutex);
  logf("SIGMET %s: %d active\n", fir, n);
}
void fetchDestWx(double lat, double lon, const char *code) {   // weather at the selected departure's destination
  if (!wifiUp) return;
  DestWx d = {}; d.lat = lat; d.lon = lon; strlcpy(d.code, code, sizeof(d.code));
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http;
  httpBegin(http, tls, "https://api.open-meteo.com/v1/forecast?latitude=" + String(lat, 3) + "&longitude=" + String(lon, 3) + "&current=temperature_2m,weather_code"); http.setTimeout(8000);
  if (httpGet(http, "dest-weather") == 200) { JsonDocument doc; if (!deserializeJson(doc, http.getString())) { d.temp = doc["current"]["temperature_2m"] | 0.0f; d.wcode = doc["current"]["weather_code"] | 0; d.valid = true; } }
  http.end();
  xSemaphoreTake(dataMutex, portMAX_DELAY); sharedDestWx = d; destDirty = true; xSemaphoreGive(dataMutex);
}
void fetchFng() {   // crypto fear & greed index
  if (!wifiUp) return;
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http; httpBegin(http, tls, "https://api.alternative.me/fng/?limit=1"); http.setTimeout(8000);
  if (httpGet(http, "fear-greed") == 200) {
    JsonDocument doc;
    if (!deserializeJson(doc, http.getString())) { xSemaphoreTake(dataMutex, portMAX_DELAY); sharedFng = atoi(doc["data"][0]["value"] | "0"); strlcpy(sharedFngText, doc["data"][0]["value_classification"] | "", sizeof(sharedFngText)); xSemaphoreGive(dataMutex); }
  }
  http.end();
}
void fetchRainStats() {   // last 31 days of daily rain from the forecast model
  if (!wifiUp) return;
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http; http.useHTTP10(true);
  httpBegin(http, tls, "https://api.open-meteo.com/v1/forecast?latitude=" + String(weatherLat(), 3) + "&longitude=" + String(weatherLon(), 3) + "&daily=precipitation_sum&past_days=31&forecast_days=1&timezone=" + urlEnc(LOCAL_TZ_IANA)); http.setTimeout(10000);
  if (httpGet(http, "rain-stats") == 200) {
    JsonDocument filter; filter["daily"]["precipitation_sum"] = true;
    JsonDocument doc;
    PatientStream ps(http.getStream()); ps.setTimeout(4000);
    if (!deserializeJson(doc, ps, DeserializationOption::Filter(filter))) {
      JsonArray a = doc["daily"]["precipitation_sum"].as<JsonArray>(); int n = a.size();
      struct tm t; int mday = getTime(t) ? t.tm_mday : 1;
      float month = 0, week = 0;
      for (int i = 0; i < n - 1; i++) { float v = a[i] | 0.0f; int daysAgo = (n - 1) - i; if (daysAgo < mday) month += v; if (daysAgo <= 7) week += v; }
      xSemaphoreTake(dataMutex, portMAX_DELAY); sharedRainMonth = month; sharedRain7d = week; xSemaphoreGive(dataMutex);
      logf("rain month %.1f mm, 7d %.1f mm\n", month, week);
    }
  }
  http.end();
}
void fetchF1() {   // next race and last podium
  if (!wifiUp) return;
  F1Info f = {};
  {
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http; http.useHTTP10(true);   // fresh TLS session per request
  httpBegin(http, tls, "https://api.jolpi.ca/ergast/f1/current/next.json"); http.setTimeout(10000);
  if (httpGet(http, "f1") == 200) {
    JsonDocument filter;
    JsonObject r = filter["MRData"]["RaceTable"]["Races"][0].to<JsonObject>(); r["raceName"] = true; r["date"] = true; r["time"] = true; r["Circuit"]["circuitName"] = true;
    JsonDocument doc; PatientStream ps(http.getStream()); ps.setTimeout(4000);
    if (!deserializeJson(doc, ps, DeserializationOption::Filter(filter))) {
      JsonObject race = doc["MRData"]["RaceTable"]["Races"][0];
      strlcpy(f.race, race["raceName"] | "", sizeof(f.race)); strlcpy(f.circuit, race["Circuit"]["circuitName"] | "", sizeof(f.circuit));
      const char *date = race["date"] | "", *tm = race["time"] | "14:00:00Z";
      struct tm e = {}; if (sscanf(date, "%d-%d-%d", &e.tm_year, &e.tm_mon, &e.tm_mday) == 3) { int hh = 0, mm = 0; sscanf(tm, "%d:%d", &hh, &mm); e.tm_year -= 1900; e.tm_mon -= 1; e.tm_hour = hh; e.tm_min = mm; f.t = (uint32_t)timegmUtc(e.tm_year + 1900, e.tm_mon + 1, e.tm_mday, e.tm_hour, e.tm_min); }
      f.valid = f.race[0] != 0;
    }
  }
  http.end();
  }
  {
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http; http.useHTTP10(true);
  httpBegin(http, tls, "https://api.jolpi.ca/ergast/f1/current/last/results.json?limit=3"); http.setTimeout(10000);
  if (httpGet(http, "f1") == 200) {
    JsonDocument filter;
    filter["MRData"]["RaceTable"]["Races"][0]["Results"][0]["Driver"]["familyName"] = true; filter["MRData"]["RaceTable"]["Races"][0]["Results"][0]["Driver"]["code"] = true;
    JsonDocument doc; PatientStream ps(http.getStream()); ps.setTimeout(4000);
    if (!deserializeJson(doc, ps, DeserializationOption::Filter(filter))) {
      int k = 0; for (JsonObject res : doc["MRData"]["RaceTable"]["Races"][0]["Results"].as<JsonArray>()) { if (k >= 3) break; strlcpy(f.podium[k++], res["Driver"]["familyName"] | (const char*)(res["Driver"]["code"] | ""), 14); }
    }
  }
  http.end();
  }
  f.ms = millis();
  if (!f.valid) return;   // 6.14.1: keep the last good F1 data
  xSemaphoreTake(dataMutex, portMAX_DELAY); sharedF1 = f; f1Dirty = true; xSemaphoreGive(dataMutex);
  if (f.valid) logf("F1 next: %s\n", f.race);
}
unsigned long quoteBackoffUntil = 0, quoteBackoffMs = 0;   // 6.8.1: Yahoo answers 429 to a household that polls too fast; back off 5 min doubling to 30
// 6.8.2: previous close from the daily series (5 bars). If the last bar is today's session the previous close is
// the bar before it; otherwise (pre-open, holiday) it is the last bar. Yahoo sends no regularMarketPreviousClose
// for these indices and chartPreviousClose belongs to whichever day's chart it serves. Refreshed every 6 h.
float dailyPrev[NUM_SYMBOLS] = {0}; unsigned long dailyPrevMs[NUM_SYMBOLS] = {0};
bool fetchDailyPrev(int i, WiFiClientSecure &tls) {
  String sym = MARKET_SYMBOLS[i]; sym.replace("^", "%5E"); sym.replace("=", "%3D");
  HTTPClient http; httpBegin(http, tls, "https://query1.finance.yahoo.com/v8/finance/chart/" + sym + "?interval=1d&range=5d");
  http.useHTTP10(true); http.setTimeout(10000); http.setUserAgent("Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 Chrome/124 Safari/537.36");
  int code = httpGet(http, "daily");
  if (code != 200) {   // 6.14.1: retry in 10 min, not on every pass; 429 / 403 back off like fetchQuote
    http.end(); dailyPrevMs[i] = millis() - 21000000UL;
    if (code == 429 || code == 403) { quoteBackoffMs = quoteBackoffMs ? (quoteBackoffMs < 1800000UL ? quoteBackoffMs * 2 : 1800000UL) : 300000UL; quoteBackoffUntil = millis() + quoteBackoffMs; }
    return false;
  }
  JsonDocument filter; filter["chart"]["result"][0]["timestamp"] = true; filter["chart"]["result"][0]["indicators"]["quote"][0]["close"] = true;
  JsonDocument doc; PatientStream ps(http.getStream()); ps.setTimeout(4000);
  DeserializationError e = deserializeJson(doc, ps, DeserializationOption::Filter(filter)); http.end();
  if (e) { dailyPrevMs[i] = millis() - 21000000UL; return false; }
  JsonArray ts = doc["chart"]["result"][0]["timestamp"], cl = doc["chart"]["result"][0]["indicators"]["quote"][0]["close"];
  float closes[8]; uint32_t when[8]; int n = 0;
  for (size_t k = 0; k < cl.size() && k < ts.size() && n < 8; k++) if (!cl[k].isNull()) { closes[n] = cl[k].as<float>(); when[n] = ts[k].as<uint32_t>(); n++; }
  if (n < 2) { dailyPrevMs[i] = millis() - 21000000UL; return false; }
  time_t tl = when[n - 1]; struct tm lb; xSemaphoreTake(tzMutex, portMAX_DELAY); localtime_r(&tl, &lb); xSemaphoreGive(tzMutex); struct tm now; bool haveT = getTime(now);
  bool lastIsToday = haveT && lb.tm_yday == now.tm_yday && lb.tm_year == now.tm_year;
  dailyPrev[i] = lastIsToday ? closes[n - 2] : closes[n - 1]; dailyPrevMs[i] = millis();
  logf("Quote %s daily prev close %.2f (%s)\n", MARKET_SYMBOLS[i], dailyPrev[i], lastIsToday ? "bar before today's" : "last completed bar");
  return true;
}
bool fetchQuote(int i) {
  Quote q = {};
  if (quoteBackoffUntil && millis() < quoteBackoffUntil) return false;
  String sym = MARKET_SYMBOLS[i]; sym.replace("^", "%5E"); sym.replace("=", "%3D");
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http;
  httpBegin(http, tls, "https://query1.finance.yahoo.com/v8/finance/chart/" + sym + "?interval=5m&range=1d");
  http.useHTTP10(true); http.setTimeout(10000);
  http.setUserAgent("Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 Chrome/124 Safari/537.36");   // 6.5.5: was addHeader (dropped)
  int code = httpGet(http, "quote");
  if (code != 200) {
    logf("Quote %s HTTP %d\n", MARKET_SYMBOLS[i], code); http.end();
    if (code == 429 || code == 403) { quoteBackoffMs = quoteBackoffMs ? (quoteBackoffMs < 1800000UL ? quoteBackoffMs * 2 : 1800000UL) : 300000UL; quoteBackoffUntil = millis() + quoteBackoffMs; logf("quotes: rate limited, backing off %lu min\n", quoteBackoffMs / 60000); }
    return false;
  }
  quoteBackoffMs = 0; quoteBackoffUntil = 0;
  JsonDocument filter;
  JsonObject m = filter["chart"]["result"][0]["meta"].to<JsonObject>();
  m["regularMarketPrice"] = true; m["chartPreviousClose"] = true; m["previousClose"] = true; m["regularMarketPreviousClose"] = true; m["regularMarketDayHigh"] = true; m["regularMarketDayLow"] = true;
  filter["chart"]["result"][0]["indicators"]["quote"][0]["close"] = true;
  JsonDocument doc;
  PatientStream ps(http.getStream()); ps.setTimeout(4000);
  DeserializationError e = deserializeJson(doc, ps, DeserializationOption::Filter(filter));
  http.end();
  if (e) { logf("Quote %s parse: %s\n", MARKET_SYMBOLS[i], e.c_str()); return false; }
  JsonObject meta = doc["chart"]["result"][0]["meta"];
  q.price = meta["regularMarketPrice"] | 0.0f;
  // 6.8.1: the official previous session close first (chartPreviousClose is the close before the chart's range, which
  // for a 1d chart requested pre-open is two sessions back: the +0.35 % vs Google -0.59 % SENSEX case of 7 Oct)
  float pcReg = meta["regularMarketPreviousClose"] | 0.0f, pcChart = meta["chartPreviousClose"] | 0.0f, pcPrev = meta["previousClose"] | 0.0f;
  q.prevClose = pcReg > 0 ? pcReg : pcChart > 0 ? pcChart : pcPrev > 0 ? pcPrev : q.price;
  if (dailyPrevMs[i] == 0 || millis() - dailyPrevMs[i] > 21600000UL) fetchDailyPrev(i, tls);
  if (dailyPrev[i] > 0) q.prevClose = dailyPrev[i];
  { static bool shown[NUM_SYMBOLS] = {}; if (!shown[i]) { shown[i] = true; logf("Quote %s prev-close fields: regular %.2f chart %.2f previous %.2f\n", MARKET_SYMBOLS[i], pcReg, pcChart, pcPrev); } }
  q.high = meta["regularMarketDayHigh"] | 0.0f; q.low = meta["regularMarketDayLow"] | 0.0f;
  JsonArray cl = doc["chart"]["result"][0]["indicators"]["quote"][0]["close"];
  // 5m bars over range=1d: 75 for an NSE session, up to 288 for a 24h symbol (BTC-USD, GC=F, USDINR=X).
  // Keeping the first MKT_PTS left those three charts showing only the oldest ~6.7 h, so decimate instead.
  static float pts[MKT_PTS]; uint8_t n = 0;
  int total = 0; for (JsonVariant v : cl) if (!v.isNull()) total++;
  int stride = total > MKT_PTS ? (total + MKT_PTS - 1) / MKT_PTS : 1;
  int k = 0; float last = 0;
  for (JsonVariant v : cl) {
    if (v.isNull()) continue;
    float f = v.as<float>(); last = f;
    if (q.high <= 0 || f > q.high) q.high = f; if (q.low <= 0 || f < q.low) q.low = f;
    if (k % stride == 0 && n < MKT_PTS) pts[n++] = f;
    k++;
  }
  if (n > 0) pts[n - 1] = last;   // right edge is the latest print
  if (q.price <= 0) { logf("Quote %s: no price in the reply\n", MARKET_SYMBOLS[i]); return false; }
  q.valid = true; q.updatedMs = millis();
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  sharedQuotes[i] = q; quotesDirty = true;
  memcpy(mktPtsCache[i], pts, sizeof(float) * n); mktNCache[i] = n;   // n < 2 clears it, same as before: "loading chart..."
  xSemaphoreGive(dataMutex);
  logf("Quote %s %.2f prev %.2f pts %d\n", MARKET_SYMBOLS[i], q.price, q.prevClose, n);
  return true;
}

bool fetchHeat(int i) {   // one watchlist symbol, price and previous close only (arena: ARENA_MARKET)
  if (i >= NUM_HEAT) return false;
  String sym = HEAT_SYMBOLS[i]; sym.replace("^", "%5E"); sym.replace("=", "%3D");
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http;
  httpBegin(http, tls, "https://query1.finance.yahoo.com/v8/finance/chart/" + sym + "?interval=1d&range=1d");
  http.useHTTP10(true); http.setTimeout(8000);
  http.setUserAgent("Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 Chrome/124 Safari/537.36");   // 6.5.5: was addHeader (dropped)
  int code = httpGet(http, "heat");
  if (code != 200) { logf("Heat %s HTTP %d\n", HEAT_SYMBOLS[i], code); http.end(); return false; }
  JsonDocument filter;
  filter["chart"]["result"][0]["meta"]["regularMarketPrice"] = true; filter["chart"]["result"][0]["meta"]["chartPreviousClose"] = true; filter["chart"]["result"][0]["meta"]["previousClose"] = true;
  JsonDocument doc;
  PatientStream ps(http.getStream()); ps.setTimeout(4000);
  DeserializationError e = deserializeJson(doc, ps, DeserializationOption::Filter(filter));
  http.end();
  if (e) return false;
  JsonObject meta = doc["chart"]["result"][0]["meta"];
  float price = meta["regularMarketPrice"] | 0.0f, prev = meta["chartPreviousClose"] | (meta["previousClose"] | price);
  if (price <= 0) return false;
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  if (arenaOwner == ARENA_MARKET) { ARENA_MK->heat[i].price = price; ARENA_MK->heat[i].prev = prev; ARENA_MK->heat[i].valid = true; ARENA_MK->heatMs = millis(); staticDirty = true; }
  xSemaphoreGive(dataMutex);
  return true;
}
void fetchMf(int i) {   // mfapi.in: history newest first, read two entries then stop (arena: ARENA_MARKET)
  if (i >= NUM_MF || i >= 4) return;
  MfNav r = {};
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http; http.useHTTP10(true);
  httpBegin(http, tls, String("https://api.mfapi.in/mf/") + MF_SCHEMES[i]); http.setTimeout(10000);
  int code = httpGet(http, "mf");
  if (code == 200) {
    PatientStream ps(http.getStream()); ps.setTimeout(4000);
    if (ps.find((char*)"\"scheme_name\"") && ps.find((char*)"\"")) {
      int k = 0; int ch;
      while ((ch = ps.read()) >= 0 && ch != '"' && k < 25) r.name[k++] = (char)ch;
      r.name[k] = 0;
    }
    if (ps.find((char*)"\"data\"") && ps.find((char*)"[")) {
            for (int e = 0; e < 2; e++) {
        skipWs(ps); if (ps.peek() == ']') break;
        JsonDocument d;
        if (deserializeJson(d, ps)) break;
        float nav = atof(d["nav"] | "0");
        if (e == 0) { r.nav = nav; strlcpy(r.date, d["date"] | "", sizeof(r.date)); } else r.prev = nav;
        if (!ps.findUntil((char*)",", (char*)"]")) break;
      }
      r.valid = r.nav > 0;
    }
  } else logf("MF %s HTTP %d\n", MF_SCHEMES[i], code);
  http.end();
  xSemaphoreTake(dataMutex, portMAX_DELAY); if (arenaOwner == ARENA_MARKET) { ARENA_MK->mf[i] = r; ARENA_MK->mfMs = millis(); staticDirty = true; } xSemaphoreGive(dataMutex);
  if (r.valid) logf("MF %s %.2f (prev %.2f) %s\n", r.name, r.nav, r.prev, r.date);
}

void fetchQuakes() {
  if (!wifiUp) return;
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http; http.useHTTP10(true);
  String u = "https://earthquake.usgs.gov/fdsnws/event/1/query?format=geojson&latitude=" + String(homeLat, 2) + "&longitude=" + String(homeLon, 2) +
             "&maxradiuskm=1500&minmagnitude=4.5&limit=3&orderby=time";
  httpBegin(http, tls, u); http.setTimeout(10000);
  int code = httpGet(http, "quakes");
  Quake qs[3] = {};
  if (code == 200) {
    JsonDocument filter;
    filter["properties"]["mag"] = true; filter["properties"]["place"] = true; filter["properties"]["time"] = true; filter["geometry"]["coordinates"] = true;
    PatientStream ps(http.getStream()); ps.setTimeout(4000);
    if (ps.find((char*)"\"features\"") && ps.find((char*)"[")) {
      for (int k = 0; k < 3; k++) {
        skipWs(ps); if (ps.peek() == ']' || ps.peek() < 0) break;
        JsonDocument d;
        if (deserializeJson(d, ps, DeserializationOption::Filter(filter))) break;
        qs[k].mag = d["properties"]["mag"] | 0.0f;
        strlcpy(qs[k].place, d["properties"]["place"] | "", sizeof(qs[k].place));
        qs[k].t = (uint32_t)((d["properties"]["time"] | 0.0) / 1000.0);
        double qlon = d["geometry"]["coordinates"][0] | 0.0, qlat = d["geometry"]["coordinates"][1] | 0.0;
        qs[k].distKm = distanceKm(homeLat, homeLon, qlat, qlon);
        qs[k].valid = true;
        if (!ps.findUntil((char*)",", (char*)"]")) break;
      }
    }
  } else logf("quakes HTTP %d\n", code);
  http.end();
  if (code != 200) return;   // 6.14.1: keep the last quake list
  xSemaphoreTake(dataMutex, portMAX_DELAY); memcpy(sharedQuakes, qs, sizeof(qs)); quakeDirty = true; xSemaphoreGive(dataMutex);
}

// tiny RSS scanner: <item> ... <title>text</title>, CDATA and the common entities
void decodeEntities(char *s) {
  struct { const char *e; char c; } E[] = {{"&amp;", '&'}, {"&lt;", '<'}, {"&gt;", '>'}, {"&quot;", '"'}, {"&apos;", '\''}, {"&#39;", '\''}, {"&#x27;", '\''}, {"&nbsp;", ' '}};
  for (auto &x : E) { char *p; while ((p = strstr(s, x.e))) { *p = x.c; memmove(p + 1, p + strlen(x.e), strlen(p + strlen(x.e)) + 1); } }
  char *p; while ((p = strstr(s, "&#"))) { char *end = strchr(p, ';'); if (!end) break; int code = atoi(p + 2); *p = (code == 8217 || code == 8216) ? '\'' : (code == 8211 || code == 8212) ? '-' : (code < 128 ? (char)code : '?'); memmove(p + 1, end + 1, strlen(end + 1) + 1); }
}
void fetchNews() {
  if (!wifiUp) return;
  Headline tmp[MAX_NEWS]; int n = 0;
  for (int f = 0; f < NUM_FEEDS && n < MAX_NEWS; f++) {
    WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http; http.useHTTP10(true);
    httpBegin(http, tls, NEWS_FEEDS[f]); http.setTimeout(12000);
    http.setUserAgent("Mozilla/5.0 DeskRadar/6");   // 6.5.5: addHeader("User-Agent") is dropped by HTTPClient
    int code = httpGet(http, "news");
    if (code != 200) { logf("news %d HTTP %d\n", f, code); http.end(); continue; }
    PatientStream ps(http.getStream()); ps.setTimeout(3000);
    int perFeed = 0; unsigned long t0 = millis();
    while (perFeed < MAX_NEWS / NUM_FEEDS + 1 && n < MAX_NEWS && millis() - t0 < 15000) {
      if (!ps.find((char*)"<item>")) break;
      if (!ps.find((char*)"<title>")) break;
      char buf[160]; int k = 0; int ch; bool cdata = false;
      // read until </title>
      while ((ch = ps.read()) >= 0 && k < (int)sizeof(buf) - 1) {
        buf[k++] = (char)ch; buf[k] = 0;
        if (k >= 8 && !strcmp(buf + k - 8, "</title>")) { buf[k - 8] = 0; break; }
      }
      char *s = buf;
      if (!strncmp(s, "<![CDATA[", 9)) { s += 9; cdata = true; char *e = strstr(s, "]]>"); if (e) *e = 0; }
      (void)cdata;
      decodeEntities(s);
      while (*s == ' ') s++;
      if (!*s) continue;
      strlcpy(tmp[n].text, s, sizeof(tmp[n].text)); tmp[n].feed = f; n++; perFeed++;
    }
    http.end();
  }
  xSemaphoreTake(dataMutex, portMAX_DELAY); memcpy(sharedNews, tmp, sizeof(Headline) * n); sharedNewsCount = n; newsDirty = true; xSemaphoreGive(dataMutex);
  logf("news: %d headlines\n", n);
}

void fetchCricket() {
  if (!wifiUp || !CRICAPI_KEY[0]) return;
  CricketMatch out[2] = {}; int n = 0;
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http; http.useHTTP10(true);
  httpBegin(http, tls, String("https://api.cricapi.com/v1/currentMatches?apikey=") + CRICAPI_KEY + "&offset=0"); http.setTimeout(12000);
  int code = httpGet(http, "cricket");
  if (code == 200) {
    JsonDocument filter;
    filter["name"] = true; filter["status"] = true; filter["matchStarted"] = true; filter["matchEnded"] = true; filter["teams"] = true;
    filter["score"][0]["r"] = true; filter["score"][0]["w"] = true; filter["score"][0]["o"] = true; filter["score"][0]["inning"] = true;
    PatientStream ps(http.getStream()); ps.setTimeout(4000);
    if (ps.find((char*)"\"data\"") && ps.find((char*)"[")) {
      for (int k = 0; k < 40 && n < 2; k++) {
        skipWs(ps); if (ps.peek() == ']' || ps.peek() < 0) break;
        JsonDocument d;
        if (deserializeJson(d, ps, DeserializationOption::Filter(filter))) break;
        bool more = ps.findUntil((char*)",", (char*)"]");
        bool started = d["matchStarted"] | false, ended = d["matchEnded"] | false;
        bool mine = !CRICKET_TEAM_FILTER[0];
        for (JsonVariant t : d["teams"].as<JsonArray>()) if (strstr(t | "", CRICKET_TEAM_FILTER)) mine = true;
        if (mine && started) {
          CricketMatch &m = out[n++];
          strlcpy(m.name, d["name"] | "", sizeof(m.name)); strlcpy(m.status, d["status"] | "", sizeof(m.status)); m.live = !ended; m.valid = true;
          int si = 0;
          for (JsonObject s : d["score"].as<JsonArray>()) {
            if (si >= 2) break;
            char inn[20]; strlcpy(inn, s["inning"] | "", sizeof(inn)); char *sp = strchr(inn, ' '); if (sp) *sp = 0;   // team short name
            snprintf(m.score[si++], 24, "%s %d/%d (%.1f)", inn, s["r"] | 0, s["w"] | 0, s["o"] | 0.0f);
          }
        }
        if (!more) break;
      }
    }
  } else logf("cricket HTTP %d\n", code);
  http.end();
  if (code != 200) return;   // 6.14.1: keep the last scores
  xSemaphoreTake(dataMutex, portMAX_DELAY); if (arenaOwner == ARENA_CRICKET) { memcpy(ARENA_CR->m, out, sizeof(out)); ARENA_CR->ms = millis(); staticDirty = true; } xSemaphoreGive(dataMutex);
  logf("cricket: %d matches\n", n);
}

// network probes: ICMP ping to the gateway (Deco) and 1.1.1.1, TCP connect as fallback, DNS lookup time
uint16_t icmpPingMs(IPAddress ip, int timeoutMs) {
  int s = lwip_socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
  if (s < 0) return 0xFFFE;   // raw sockets unavailable
  struct timeval tv = {timeoutMs / 1000, (timeoutMs % 1000) * 1000};
  lwip_setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  uint8_t pkt[32] = {0}; static uint16_t seq = 0; seq++;
  pkt[0] = 8; pkt[4] = 0x12; pkt[5] = 0x34; pkt[6] = seq >> 8; pkt[7] = seq & 0xFF;
  for (int i = 8; i < 32; i++) pkt[i] = (uint8_t)i;
  uint32_t sum = 0; for (int i = 0; i < 32; i += 2) sum += (pkt[i] << 8) | pkt[i + 1];
  while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
  uint16_t ck = ~sum; pkt[2] = ck >> 8; pkt[3] = ck & 0xFF;
  struct sockaddr_in to = {}; to.sin_family = AF_INET; to.sin_addr.s_addr = (uint32_t)ip;
  unsigned long t0 = micros(), t0ms = millis();
  if (lwip_sendto(s, pkt, sizeof(pkt), 0, (struct sockaddr *)&to, sizeof(to)) < 0) { lwip_close(s); return 0xFFFF; }
  uint8_t rx[96]; uint16_t result = 0xFFFF;
  while (millis() - t0ms < (unsigned long)timeoutMs) {   // 6.14.1: was millis() - micros()/1000, always past the timeout after 71 min of uptime
    struct sockaddr_in from; socklen_t fl = sizeof(from);
    int n = lwip_recvfrom(s, rx, sizeof(rx), 0, (struct sockaddr *)&from, &fl);
    if (n <= 0) break;
    int ihl = (rx[0] & 0x0F) * 4;
    if (n >= ihl + 8 && rx[ihl] == 0 && rx[ihl + 4] == 0x12 && rx[ihl + 5] == 0x34 && rx[ihl + 6] == (seq >> 8) && rx[ihl + 7] == (seq & 0xFF)) { unsigned long dt = (micros() - t0) / 1000; result = dt > 1500 ? 1500 : (dt < 1 ? 1 : dt); break; }
  }
  lwip_close(s);
  return result;
}
uint16_t tcpProbeMs(IPAddress ip, uint16_t port) {
  WiFiClient c; unsigned long t0 = micros();
  bool ok = c.connect(ip, port, 1500); unsigned long dt = (micros() - t0) / 1000;
  c.stop(); return ok ? (uint16_t)(dt > 1500 ? 1500 : dt) : 0xFFFF;
}
// DNS latency with a raw UDP query to the router's resolver. Deliberately NOT WiFi.hostByName(): the earlier
// version asked for a random name every 5 s, which filled lwIP's small DNS table with pending entries and
// made every new lookup (weather, markets, the other feeds) fail instantly with "connection refused".
uint16_t dnsProbeMs() {
  IPAddress dns = WiFi.dnsIP(); if ((uint32_t)dns == 0) dns = WiFi.gatewayIP();
  int s = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); if (s < 0) return 0xFFFF;
  struct timeval tv = {1, 500000}; lwip_setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  static uint16_t qid = 0x2A00; qid++;
  // standard query for api.adsb.lol, type A, class IN (also keeps that name warm in the resolver)
  uint8_t q[64]; int n = 0;
  q[n++] = qid >> 8; q[n++] = qid & 0xFF; q[n++] = 0x01; q[n++] = 0x00; q[n++] = 0; q[n++] = 1; q[n++] = 0; q[n++] = 0; q[n++] = 0; q[n++] = 0; q[n++] = 0; q[n++] = 0;
  const char *labels[3] = {"api", "adsb", "lol"};
  for (int i = 0; i < 3; i++) { int l = strlen(labels[i]); q[n++] = l; memcpy(q + n, labels[i], l); n += l; }
  q[n++] = 0; q[n++] = 0; q[n++] = 1; q[n++] = 0; q[n++] = 1;
  struct sockaddr_in to = {}; to.sin_family = AF_INET; to.sin_port = htons(53); to.sin_addr.s_addr = (uint32_t)dns;
  unsigned long t0 = micros(); uint16_t result = 0xFFFF;
  if (lwip_sendto(s, q, n, 0, (struct sockaddr *)&to, sizeof(to)) == n) {
    uint8_t rx[128]; int r = lwip_recvfrom(s, rx, sizeof(rx), 0, NULL, NULL);
    if (r >= 12 && rx[0] == (qid >> 8) && rx[1] == (qid & 0xFF)) { unsigned long dt = (micros() - t0) / 1000; result = dt > 1500 ? 1500 : (dt < 1 ? 1 : dt); }
  }
  lwip_close(s);
  return result;
}
uint32_t outageStart = 0; int wanFailRun = 0; volatile bool wanDown = false;   // wanDown 6.16.0: loop() toasts it
uint16_t netLastGw = 0xFFFF, netLastWan = 0xFFFF, netLastDns = 0xFFFF;   // resident latest probe for the OLED
struct Outage { uint32_t start; uint16_t minutes; };
Outage outages[10]; int outageCount = 0;
Preferences prefsNet;   // separate handle: this runs on the fetch task
void loadOutages() { prefsNet.begin("radarnet", true); size_t n = prefsNet.getBytesLength("outages"); if (n == sizeof(outages)) prefsNet.getBytes("outages", outages, n); outageCount = prefsNet.getInt("outn", 0); prefsNet.end(); if (outageCount < 0 || outageCount > 10) outageCount = 0; }
volatile bool outagesDirty = false;
void saveOutages() {   // loop(): copy under the mutex, then write (no flash write while holding it)
  Outage copy[10]; int n; xSemaphoreTake(dataMutex, portMAX_DELAY); memcpy(copy, outages, sizeof(copy)); n = outageCount; xSemaphoreGive(dataMutex);
  prefsNet.begin("radarnet", false); prefsNet.putBytes("outages", copy, sizeof(copy)); prefsNet.putInt("outn", n); prefsNet.end();
}
void netProbe() {
  if (!wifiUp) return;
  uint16_t gw = icmpPingMs(WiFi.gatewayIP(), 1000); if (gw == 0xFFFE) gw = tcpProbeMs(WiFi.gatewayIP(), 80);
  uint16_t wan = icmpPingMs(IPAddress(1, 1, 1, 1), 1500); if (wan == 0xFFFE) wan = tcpProbeMs(IPAddress(1, 1, 1, 1), 443);
  uint16_t dns = dnsProbeMs();
  netLastGw = gw; netLastWan = wan; netLastDns = dns;
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  if (arenaOwner == ARENA_NETWORK) { ArenaNetwork *a = ARENA_NET; a->hist[0][a->idx] = gw; a->hist[1][a->idx] = wan; a->hist[2][a->idx] = dns; a->idx = (a->idx + 1) % 120; if (a->idx == 0) a->wrapped = true; }
  netDirty = true;
  xSemaphoreGive(dataMutex);
  if (wan == 0xFFFF) { if (++wanFailRun == 3) { outageStart = (uint32_t)time(NULL); wanDown = true; logln("WAN outage start"); } }
  else if (wanFailRun >= 3) {
    uint32_t nowT = (uint32_t)time(NULL); uint16_t mins = outageStart > 1600000000u ? (nowT - outageStart + 30) / 60 : 0;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    for (int i = 9; i > 0; i--) outages[i] = outages[i - 1];
    outages[0].start = outageStart; outages[0].minutes = mins; if (outageCount < 10) outageCount++;
    xSemaphoreGive(dataMutex);
    outagesDirty = true; wanDown = false; logf("WAN outage over, %u min\n", mins); wanFailRun = 0;   // 6.6.2: saved by loop() (this task's stack is in PSRAM: no flash writes here)
  } else wanFailRun = 0;
}

// =====================================================================
// FETCH TASK (core 0): one job per pass, planes first
// =====================================================================
volatile bool homeChangedFlag = false;   // planesTask -> fetchTask: refetch METAR for the new home
// 6.6.0: aircraft fetching on its own task (core 0), so a slow weather / METAR / DNS request can no longer freeze
// the radar. Same scheduling as before (source interval, immediate refetch when range / source / home change).
void planesTask(void *) {
  unsigned long lastPoll = 0, lastMetar = 0; int lastRange = -1, lastSrc = -1, lastHome = -1; double lastLat = 0, lastLon = 0;
  for (;;) {
    unsigned long now = millis();
    // planes
    int act = activeSource >= 0 ? activeSource : (dataSource <= 4 ? dataSource : 2);
    unsigned long interval = SRC_MIN_MS[act];
    if (dataSource <= 4 && !srcBackedOff(dataSource) && SRC_MIN_MS[dataSource] > interval) interval = SRC_MIN_MS[dataSource];
    bool changed = (rangeIndex != lastRange || dataSource != lastSrc || homeLat != lastLat || homeLon != lastLon || homeIndex != lastHome);
    if (wifiUp && (changed || now - lastPoll > interval)) {
      lastRange = rangeIndex; lastSrc = dataSource; lastLat = homeLat; lastLon = homeLon;
      if (homeIndex != lastHome) { lastHome = homeIndex; homeChangedFlag = true; (void)lastMetar; }
      fetchForce = changed;
      if (!fetchPlanes()) { xSemaphoreTake(dataMutex, portMAX_DELAY); sharedFetchOk = false; xSemaphoreGive(dataMutex); }
      fetchForce = false; lastPoll = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
void fetchTask(void *arg) {
  unsigned long lastPoll = 0, lastWx = 0, lastMetar = 0, lastMkt = 0, lastQuake = 0, lastNews = 0, lastCric = 0, lastProbe = 0, lastBoard = 0;
  unsigned long lastTaf = 0, lastSigmet = 0, lastRain = 0, lastFng = 0, lastF1 = 0, lastDest = 0;
  int mktIdx = 0;
  int lastRange = -1, lastSrc = -1, lastHome = -1; double lastLat = 0, lastLon = 0, wxLat = 0, wxLon = 0;
  for (;;) {
    unsigned long now = millis();
    if (NET_PROBE_ENABLED && wifiUp && now - lastProbe > NET_PROBE_MS) { netProbe(); lastProbe = millis(); }

    // 6.6.0: aircraft have their own task (planesTask). Do not open a second TLS session while the aircraft one
    // has the internal heap tight; the aircraft task never waits for this one.
    if (homeChangedFlag) { homeChangedFlag = false; lastMetar = 0; }
    if (ESP.getMaxAllocHeap() < 24000) { vTaskDelay(pdMS_TO_TICKS(150)); continue; }   // 6.6.1: 45 KB starved it (idle block settles ~35 KB and TLS works there)
    // arrivals board for another airport, only while that app owns the arena
    if (wifiUp && arenaOwner == ARENA_BOARD && (lastBoard == 0 || now - lastBoard > 30000 || ARENA_BD->ms == 0)) { if (fetchBoard() || ARENA_BD->ms) lastBoard = millis(); else lastBoard = millis() - 20000; continue; }
    // routes for the current traffic
    if (wifiUp && now - lastRoutesetMs > 20000 && sharedFetchMs) { unsigned long t = lastRoutesetMs; fetchRoutes(); if (lastRoutesetMs == t) lastRoutesetMs = millis(); continue; }
    // selected aircraft type/reg
    {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      char want[8]; strlcpy(want, (const char*)wantedIcao, sizeof(want));
      bool need = wifiUp && want[0] && strcmp(want, metaIcao) != 0;
      bool needType = true;
      for (int i = 0; i < sharedCount; i++) if (!strcmp(sharedPlanes[i].icao24, want)) { needType = sharedPlanes[i].type[0] == 0 || sharedPlanes[i].reg[0] == 0; break; }
      xSemaphoreGive(dataMutex);
      if (need) { lookupMeta(want, needType); continue; }
    }
    if (wifiUp && (lastWx == 0 || now - lastWx > WEATHER_INTERVAL_MS || weatherLat() != wxLat || weatherLon() != wxLon)) { bool moved = weatherLat() != wxLat || weatherLon() != wxLon; fetchWeather(); lastWx = millis(); wxLat = weatherLat(); wxLon = weatherLon(); if (moved) lastRain = 0; continue; }   // 6.14.1: rain stats only refetch on a location change
    if (wifiUp && (lastMetar == 0 || now - lastMetar > METAR_INTERVAL_MS)) { fetchMetar(); lastMetar = millis(); continue; }
    if (wifiUp && (lastTaf == 0 || now - lastTaf > 3600000UL)) { fetchTaf(); lastTaf = millis(); continue; }
    // destination weather for the selected departure
    { double dl = wantedDestLat, dn = wantedDestLon;
      if (wifiUp && dl != 0 && (dl != destWx.lat || dn != destWx.lon) && now - lastDest > 5000) { char c[4]; strlcpy(c, (const char*)wantedDestCode, 4); fetchDestWx(dl, dn, c); lastDest = millis(); continue; } }
    // focus log: one queued session per pass, retry every 2 min after a failure
    { static unsigned long lastFocusTry = 0;
      if (wifiUp && FOCUS_LOG_URL[0] && focusQn > 0 && (lastFocusTry == 0 || now - lastFocusTry > 120000UL)) {
        if (!sendFocusLog()) lastFocusTry = millis(); else lastFocusTry = 0;
        continue; }
      static unsigned long lastStats = 0, lastStatsTry = 0; static uint8_t statsFail = 0;   // stats: at boot, after each upload, every 30 min
      unsigned long statsRetry = 60000UL << (statsFail > 5 ? 5 : statsFail);   // 60 s doubling to 32 min: the redirect host is unreachable, do not hammer it
      if (wifiUp && FOCUS_LOG_URL[0] && focusQn == 0 && (statsWanted || lastStats == 0 || now - lastStats > 1800000UL) && (lastStatsTry == 0 || now - lastStatsTry > statsRetry)) {
        lastStatsTry = millis();
        if (fetchFocusStats()) { statsWanted = false; lastStats = millis(); statsFail = 0; }
        else if (statsFail < 5) statsFail++;
        continue; } }
    // markets: one symbol per pass
    {
      struct tm t; bool open = getTime(t) && marketOpenNow(t);
      bool showing = (currentApp == APP_MARKET || (currentApp == APP_CLOCK && clockFace == 2));
      bool anyInvalid = false; for (int i = 0; i < NUM_SYMBOLS; i++) if (!quotes[i].valid) anyInvalid = true;
      unsigned long per = ((showing && open) ? MARKET_POLL_OPEN_MS : MARKET_POLL_IDLE_MS) / NUM_SYMBOLS;
      if (showing && anyInvalid && quoteBackoffUntil == 0) per = 8000;   // 6.8.1: was 1.5 s for all 7 symbols, which earned the 429s; never while backed off
      if (wifiUp && (marketForce || lastMkt == 0 || now - lastMkt > per)) {
        if (marketForce) { marketForce = false; if (marketSel < NUM_SYMBOLS) mktIdx = marketSel; }
        fetchQuote(mktIdx); mktIdx = (mktIdx + 1) % NUM_SYMBOLS; lastMkt = millis(); continue;
      }
    }
    // market arena pages: heat map and mutual funds, only while those pages are open
    if (wifiUp && arenaOwner == ARENA_MARKET) {
      int nMf = NUM_MF < 4 ? NUM_MF : 4;
      if (marketSel == heatPage() && (ARENA_MK->heatMs == 0 || now - ARENA_MK->heatMs > 60000 || !ARENA_MK->heat[NUM_HEAT - 1].valid)) { int k = 0; while (k < NUM_HEAT && ARENA_MK->heat[k].valid && !(ARENA_MK->heatMs && now - ARENA_MK->heatMs > 60000)) k++; if (k >= NUM_HEAT) { xSemaphoreTake(dataMutex, portMAX_DELAY); if (arenaOwner == ARENA_MARKET) for (int j = 0; j < NUM_HEAT; j++) ARENA_MK->heat[j].valid = false; xSemaphoreGive(dataMutex); k = 0; } fetchHeat(k); continue; }
      if (nMf > 0 && marketSel == mfPage() && (ARENA_MK->mfMs == 0 || now - ARENA_MK->mfMs > MF_INTERVAL_MS)) { int k = 0; while (k < nMf && ARENA_MK->mf[k].valid) k++; if (k < nMf) { fetchMf(k); continue; } }
    }
    if (wifiUp && (lastNews == 0 || now - lastNews > NEWS_INTERVAL_MS) && NUM_FEEDS > 0) { fetchNews(); lastNews = millis(); continue; }
    { bool near = issPass.valid && (uint32_t)time(NULL) + 900 >= issPass.startT && (uint32_t)time(NULL) <= issPass.maxT + 360;
      if (issOn && wifiUp && (issFetchMs == 0 || now - issFetchMs > (near ? 30000UL : 600000UL))) { fetchIss(); issFetchMs = millis(); continue; } }
    if (wifiUp && (lastQuake == 0 || now - lastQuake > QUAKE_INTERVAL_MS)) { fetchQuakes(); lastQuake = millis(); continue; }
    if (wifiUp && (lastSigmet == 0 || now - lastSigmet > 3600000UL)) { fetchSigmet(); lastSigmet = millis(); continue; }
    if (wifiUp && (lastRain == 0 || now - lastRain > 21600000UL)) { fetchRainStats(); lastRain = millis(); continue; }
    if (wifiUp && (lastFng == 0 || now - lastFng > 21600000UL)) { fetchFng(); lastFng = millis(); continue; }
    if (wifiUp && F1_ENABLED && (lastF1 == 0 || now - lastF1 > 21600000UL)) { fetchF1(); lastF1 = millis(); continue; }
    if (wifiUp && CRICAPI_KEY[0] && arenaOwner == ARENA_CRICKET && (ARENA_CR->ms == 0 || now - ARENA_CR->ms > CRICKET_INTERVAL_MS) && now - lastCric > 60000) { fetchCricket(); lastCric = millis(); continue; }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// =====================================================================
// MELODIES (non-blocking) + ALERTS
// =====================================================================
int melId = -1, melPos = 0; unsigned long melNextMs = 0; bool melLoop = false;
void beep(int f, int ms) {
  if (muteAll) return;   // 6.7.0: mute all (SETTINGS > SOUND)
#if BUZZER_PIN >= 0
  tone(BUZZER_PIN, pzf(f), ms);
#endif
}
void playMelody(int id, bool loopIt) { if (id < 0 || id >= M_COUNT) return;
  if (muteAll && id != M_ALARM) return; if (id == M_CLICK && !clickOn) return;   /* 6.7.0: mute all, knob clicks */
  melId = id; melPos = 0; melNextMs = millis(); melLoop = loopIt; }
void stopMelody() {
  melId = -1; melLoop = false;
#if BUZZER_PIN >= 0
  noTone(BUZZER_PIN);
#endif
}
void melodyTick() {
  if (melId < 0 || millis() < melNextMs) return;
  if (melPos >= MELODY_LEN[melId]) { if (melLoop) { melPos = 0; melNextMs = millis() + 1500; return; } melId = -1; return; }
  const Note &n = MELODIES[melId][melPos++];
#if BUZZER_PIN >= 0
  if (n.f) tone(BUZZER_PIN, pzf(n.f), n.ms);
#endif
  melNextMs = millis() + n.ms;
}
bool quietNow() {
  if (quietStart == quietEnd) return false;
  struct tm t; if (!getTime(t)) return false; int h = t.tm_hour;
  return quietStart < quietEnd ? (h >= quietStart && h < quietEnd) : (h >= quietStart || h < quietEnd);
}
bool alertEnabled(int melody) { if (melody < 0 || melody == M_EMERG || melody == M_ALARM || melody == M_CLICK) return true; return alertMask & (1u << melody); }
void raiseAlert(const char *l1, const char *l2, uint16_t col, int melody, unsigned long ms, const char *icao) {
  if (!alertEnabled(melody)) { logf("alert (muted by filter) %s\n", l1); return; }
  if (melody >= 0 && melody != M_EMERG && melody != M_ALARM && quietNow()) melody = -1;   // quiet hours: show it, no sound
  // popups off (default): the alert is logged, chimed and kept in the history, but no banner takes over either screen
  // focus mode is do-not-disturb: nothing shows or sounds except the Pomodoro's own events and the alarm clock
  bool pomoOwn = melody == M_POMO || melody == M_BREAK || melody == M_TIMER || melody == M_ALARM;
  if (pomoFocusActive() && !pomoOwn) {   // quiet mode: hold alerts until the break (Pomodoro settings)
    bool plane = melody == M_TRAFFIC || melody == M_HEAVY || melody == M_WATCH || melody == M_OVERHEAD || melody == M_GOAROUND || melody == M_EMERG;
    bool mkt = melody == M_MARKET;
    if (pcfg.quiet == 3 || (pcfg.quiet >= 1 && plane) || (pcfg.quiet >= 2 && mkt)) {
      uint8_t &c = plane ? heldPlanes : mkt ? heldMkt : heldOther; if (c < 255) c++;
      logf("ALERT (focus, held) %s | %s\n", l1, l2 ? l2 : ""); return;
    }
  }
  bool banner = popupsOn || melody == M_EMERG || melody == M_ALARM || melody == M_TIMER;
  if (banner) { strlcpy(toast.l1, l1, sizeof(toast.l1)); strlcpy(toast.l2, l2 ? l2 : "", sizeof(toast.l2)); toast.col = col; toast.untilMs = millis() + ms; strlcpy(toast.icao, icao ? icao : "", sizeof(toast.icao)); }
  if (melody >= 0) playMelody(melody, false);
  logf("ALERT%s %s | %s\n", banner ? "" : " (no popup)", l1, l2 ? l2 : "");
  if (ms >= 4000) {   // keep a history of the real alerts (not clicks / snooze notes)
    for (int i = 9; i > 0; i--) alertLog[i] = alertLog[i - 1];
    alertLog[0].t = (uint32_t)time(NULL); strlcpy(alertLog[0].l1, l1, 22); strlcpy(alertLog[0].l2, l2 ? l2 : "", 24);
    if (alertLogCount < 10) alertLogCount++;
  }
  if (!banner) return;
  if (sleeping) wakeUntilMs = millis() + ms;   // show the toast even in sleep hours
  staticDirty = true; drawInfo();
}
bool toastActive() { if (toast.untilMs && (long)(millis() - toast.untilMs) >= 0) toast.untilMs = 0; return toast.untilMs != 0; }   // 6.14.1: wrap-safe
bool onceFor(const char *icao, int kind) {   // one-shot per aircraft and alert kind
  uint32_t h = icaoHash(icao) * 31u + kind;
  if (hashBitTest(alertedBits, 256, h)) return false;
  hashBitSet(alertedBits, 256, h); return true;
}

// =====================================================================
// AVIATION ANALYSIS
// =====================================================================
void trailPush(const Plane &p) {
  int slot = -1, oldest = 0;
  for (int i = 0; i < MAX_PLANES; i++) {
    if (trails[i].icao[0] && !strcmp(trails[i].icao, p.icao24)) { slot = i; break; }
    if (!trails[i].icao[0]) { if (slot < 0) slot = i; }
    else if (trails[i].lastMs < trails[oldest].lastMs) oldest = i;
  }
  if (slot < 0) slot = oldest;
  Trail &t = trails[slot];
  if (strcmp(t.icao, p.icao24)) { strlcpy(t.icao, p.icao24, 8); t.n = 0; }
  if (t.n == TRAIL_LEN) { for (int i = 1; i < TRAIL_LEN; i++) { t.lat[i-1] = t.lat[i]; t.lon[i-1] = t.lon[i]; t.hdg[i-1] = t.hdg[i]; t.alt[i-1] = t.alt[i]; } t.n--; }
  t.lat[t.n] = p.lat; t.lon[t.n] = p.lon; t.hdg[t.n] = p.heading_deg; t.alt[t.n] = p.altitude_m; t.n++; t.lastMs = millis();
}
const Trail *trailFor(const char *icao) {
  for (int i = 0; i < MAX_PLANES; i++) if (trails[i].icao[0] && !strcmp(trails[i].icao, icao)) return &trails[i];
  return NULL;
}
bool isHolding(const Plane &p) {   // turned through >= 90 degrees in one direction over the trail window, below FL200
  const Trail *t = trailFor(p.icao24); if (!t || t->n < 4 || p.altitude_m * 3.28084f > 20000) return false;
  float sum = 0; for (int i = 1; i < t->n; i++) sum += signedAngleDiff(t->hdg[i-1], t->hdg[i]);
  return fabsf(sum) >= 90;
}
bool isGoAround(const Plane &p) {   // was low on approach, now climbing hard
  if (p.cls != CLS_ARRIVAL && p.cls != CLS_ENROUTE) return false;
  const Trail *t = trailFor(p.icao24); if (!t || t->n < 3 || p.distance_km > 12) return false;
  float agl = p.altitude_m - homeElevM(); float minAlt = 1e9; for (int i = 0; i < t->n - 1; i++) if (t->alt[i] < minAlt) minAlt = t->alt[i];
  return (minAlt - homeElevM()) < 700 && agl < 1500 && p.vs_fpm > 800 && p.altitude_m > t->alt[t->n - 2] + 60;
}
int elevationDeg(const Plane &p, float distKm) { float agl = p.altitude_m - homeElevM(); if (agl < 0) agl = 0; return (int)(atan2f(agl, distKm * 1000.0f) * 180.0f / M_PI); }
int landingRunway(const Plane &p) {
  int rh = rwyHdg();
  if (p.distance_km > 20 && activeRwy > 0) return activeRwy;
  bool onHdgSide = angleDiff(p.bearing_deg, rh) < 90;
  int hdg = onHdgSide ? (rh + 180) % 360 : rh;
  if (p.cls == CLS_DEPARTURE) hdg = onHdgSide ? rh : (rh + 180) % 360;
  return (hdg == 0 ? 36 : hdg / 10);
}
int etaMin(const Plane &p, float dist) { float kmh = p.velocity_ms * 3.6f; return kmh > 50 ? (int)(dist / kmh * 60 + 0.5f) : 99; }
bool isHeavy(const Plane &p) {
  if (p.mil) return true;
  if (p.cat == 5) return true;
  for (int i = 0; i < NUM_HEAVY; i++) if (!strncmp(p.type, HEAVY_TYPES[i], strlen(HEAVY_TYPES[i]))) return true;
  return false;
}
const char *watchMatch(const Plane &p) {
  const char *route = routeFor(p.callsign);
  for (int i = 0; i < watchCount; i++) {
    const char *w = watchList[i]; size_t L = strlen(w);
    if (!L) continue;
    if (!strncmp(p.callsign, w, L)) return w;
    if (route && strstr(route, w)) return w;
    if (p.type[0] && !strncmp(p.type, w, L)) return w;
  }
  return NULL;
}
void updateActiveRunway() {
  int rh = rwyHdg();
  if (metar.valid && metar.wdir >= 0 && metar.wspd >= 4) { int a = rh / 10, b = ((rh + 180) % 360) / 10; activeRwy = (angleDiff(metar.wdir, rh) < 90) ? (a ? a : 36) : (b ? b : 36); return; }   // 6.14.1: 0 means unknown
  // no wind data: majority of the runways arrivals have lined up with
  int votes[37] = {0}; int best = -1, bestN = 0;
  for (int i = 0; i < 10; i++) if (rwyVotes[i]) { int v = rwyVotes[i]; if (++votes[v] > bestN) { bestN = votes[v]; best = v; } }
  if (best > 0) activeRwy = best;
}
void voteRunway(const Plane &p) {
  if (p.cls != CLS_ARRIVAL || p.distance_km > 15 || p.distance_km < 3) return;
  int rh = rwyHdg(); bool onHdgSide = angleDiff(p.bearing_deg, rh) < 90;
  int hdg = onHdgSide ? (rh + 180) % 360 : rh;
  rwyVotes[rwyVoteIdx] = (hdg == 0 ? 36 : hdg / 10); rwyVoteIdx = (rwyVoteIdx + 1) % 10;
}

void resetDailyStats(int yday) {
  statsDay = yday; memset(hourArr, 0, sizeof(hourArr)); memset(hourDep, 0, sizeof(hourDep)); memset(hourEnr, 0, sizeof(hourEnr));
  memset(seenBits, 0, sizeof(seenBits)); seenToday = seenArr = seenDep = seenEnr = 0; recHighFt = recFastKt = recFarKm = 0;
  recHighCs[0] = recFastCs[0] = recHeavyCs[0] = recHeavyType[0] = 0; airlinesSeen = 0; typesSeenCount = 0;
  memset(alertedBits, 0, sizeof(alertedBits)); marketAlerted = 0;
}
void noteSeen(const Plane &p, int hour) {
  uint32_t h = icaoHash(p.icao24);
  if (!hashBitTest(seenBits, 2048, h)) {
    hashBitSet(seenBits, 2048, h); seenToday++;
    if (hour >= 0 && hour < 24) { if (p.cls == CLS_ARRIVAL) { hourArr[hour]++; seenArr++; } else if (p.cls == CLS_DEPARTURE) { hourDep[hour]++; seenDep++; } else { hourEnr[hour]++; seenEnr++; } }
    const AirlineRow *a = airlineFor(p.callsign);
    if (a) airlinesSeen |= (1ULL << ((a - AIRLINES) & 63));
    if (p.type[0]) { bool have = false; for (int i = 0; i < typesSeenCount; i++) if (!strcmp(typesSeen[i], p.type)) have = true; if (!have && typesSeenCount < 24) strlcpy(typesSeen[typesSeenCount++], p.type, 6); }
  }
  float ft = p.altitude_m * 3.28084f, kt = p.velocity_ms * 1.94384f;
  if (ft > recHighFt) { recHighFt = ft; strlcpy(recHighCs, p.callsign, 10); }
  if (kt > recFastKt) { recFastKt = kt; strlcpy(recFastCs, p.callsign, 10); }
  if (p.distance_km > recFarKm) recFarKm = p.distance_km;
  if (isHeavy(p) && !p.mil && p.type[0] && (!recHeavyCs[0] || p.cat == 5)) { strlcpy(recHeavyCs, p.callsign, 10); strlcpy(recHeavyType, p.type, 8); }
}

void analysePlanes() {
  struct tm t; int hour = getTime(t) ? t.tm_hour : -1;
  if (hour >= 0 && t.tm_yday != statsDay) resetDailyStats(t.tm_yday);
  holdingCount = 0;
  char pcs[12], b1[22], b2[24];
  for (int i = 0; i < planeCount; i++) {
    Plane &p = planes[i];
    trailPush(p); noteSeen(p, hour); voteRunway(p);
    prettyCallsign(p.callsign, pcs, sizeof(pcs));
    if (isHolding(p)) holdingCount++;
    // emergency squawk
    if (!strcmp(p.squawk, "7700") || !strcmp(p.squawk, "7600") || !strcmp(p.squawk, "7500")) {
      if (onceFor(p.icao24, 1)) { snprintf(b1, sizeof(b1), "EMERGENCY %s", pcs); snprintf(b2, sizeof(b2), "squawk %s %s", p.squawk, !strcmp(p.squawk, "7700") ? "general" : !strcmp(p.squawk, "7600") ? "radio fail" : "hijack"); raiseAlert(b1, b2, COL_SELECTED, M_EMERG, 12000, p.icao24); }
    }
    // go-around
    if (isGoAround(p) && onceFor(p.icao24, 2)) { snprintf(b1, sizeof(b1), "GO-AROUND %s", pcs); snprintf(b2, sizeof(b2), "%+d fpm at %.0f km", (int)p.vs_fpm, p.distance_km); raiseAlert(b1, b2, COL_DEP, M_GOAROUND, 8000, p.icao24); }
#if HEAVY_ALERTS
    if (isHeavy(p) && p.distance_km < displayRangeKm() && onceFor(p.icao24, 3)) {
      snprintf(b1, sizeof(b1), "%s %s", p.mil ? "MILITARY" : "HEAVY", pcs); snprintf(b2, sizeof(b2), "%s %s %s", p.type[0] ? p.type : "", p.cls == CLS_ARRIVAL ? "arriving" : p.cls == CLS_DEPARTURE ? "departing" : "overflying", p.reg);
      raiseAlert(b1, b2, p.mil ? COL_SELECTED : COL_ENROUTE_HIGH, M_HEAVY, 6000, p.icao24);
    }
#endif
    const char *w = watchMatch(p);
    if (w && onceFor(p.icao24, 4)) { const char *r = routeFor(p.callsign); snprintf(b1, sizeof(b1), "WATCH %s", pcs); snprintf(b2, sizeof(b2), "%s %s", r ? r : "", p.cls == CLS_ARRIVAL ? "arriving" : p.cls == CLS_DEPARTURE ? "departing" : "in range"); raiseAlert(b1, b2, COL_SWEEP, M_WATCH, 8000, p.icao24); }
  }
  updateActiveRunway();
  // auto range: smallest preset holding at least 4 aircraft
  if (rangeIndex == RANGE_AUTO) {
    int best = NUM_RANGES - 1;
    for (int r = 0; r < NUM_RANGES; r++) { int n = 0; for (int i = 0; i < planeCount; i++) if (planes[i].distance_km <= RANGE_PRESETS[r] * KM_PER_UNIT && passesFilter(planes[i])) n++; if (n >= 4) { best = r; break; } }
    autoRangeKm = RANGE_PRESETS[best] * KM_PER_UNIT;
  }
}

// v6 (6.1.9): v5's classify() looks only at heading vs the airport and calls everything above 10,000 ft en route,
// so departures that had turned or climbed past 10,000 ft (IGO7143 9,600 ft hdg 271, IGO579 10,900 ft) showed as
// en route and the departures list stayed empty. Route first (destination = home: arrival, origin = home:
// departure), then climb / descent near the field, else the v5 heading rule stands.
void refineClasses() {
  const char *home = AIRPORTS[homeIndex].iata; size_t hl = strlen(home);
  for (int i = 0; i < planeCount; i++) {
    Plane &p = planes[i]; float ft = p.altitude_m * 3.28084f;
    const char *r = routeFor(p.callsign);
    if (r && r[0]) {
      const char *d = strrchr(r, '-');
      if (d && !strcmp(d + 1, home)) { p.cls = CLS_ARRIVAL; continue; }
      if (!strncmp(r, home, hl) && r[hl] == '-') { p.cls = CLS_DEPARTURE; continue; }
      if (ft > 10000) { p.cls = CLS_ENROUTE; continue; }
    }
    if (p.velocity_ms >= 20 && ft < 20000) {
      if (p.vs_fpm > 500 && p.distance_km < 60) { p.cls = CLS_DEPARTURE; continue; }
      if (p.vs_fpm < -500 && p.distance_km < 80) { p.cls = CLS_ARRIVAL; continue; }
    }
  }
}
void syncFromFetchTask() {
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  bool fresh = (sharedSeq != seenSeq);
  if (fresh) { memcpy(planes, sharedPlanes, sizeof(Plane) * sharedCount); planeCount = sharedCount; lastFetchMs = sharedFetchMs; seenSeq = sharedSeq; }
  fetchOk = sharedFetchOk; wx = sharedWx; metar = sharedMetar;
  if (quotesDirty) { memcpy(quotes, sharedQuotes, sizeof(quotes)); quotesDirty = false; staticDirty = true; }
  if (marketSel >= 0 && marketSel < NUM_SYMBOLS) { memcpy(mktDrawPts, mktPtsCache[marketSel], sizeof(mktDrawPts)); mktDrawN = mktNCache[marketSel]; mktDrawSym = marketSel; }
  else { mktDrawN = 0; mktDrawSym = -1; }
  if (quakeDirty) { memcpy(quakes, sharedQuakes, sizeof(quakes)); quakeDirty = false; staticDirty = true; }
  if (newsDirty) { memcpy(news, sharedNews, sizeof(Headline) * sharedNewsCount); newsCount = sharedNewsCount; newsDirty = false; newsMs = millis(); staticDirty = true; }
  if (routesDirty) { memcpy(routesUI, routes, sizeof(routesUI)); routesDirty = false; staticDirty = true; }
  taf = sharedTaf; fng = sharedFng; strlcpy(fngText, sharedFngText, sizeof(fngText)); rainMonth = sharedRainMonth; rain7d = sharedRain7d;
  bool newSigmet = false;
  if (sigmetDirty) { newSigmet = sharedSigmet[0].valid && (!sigmet[0].valid || strcmp(sharedSigmet[0].text, sigmet[0].text)); memcpy(sigmet, sharedSigmet, sizeof(sigmet)); sigmetDirty = false; staticDirty = true; }
  if (destDirty) { destWx = sharedDestWx; destDirty = false; staticDirty = true; }
  if (f1Dirty) { f1 = sharedF1; f1Dirty = false; staticDirty = true; }
  xSemaphoreGive(dataMutex);
  if (newSigmet) { char b2[24]; strlcpy(b2, sigmet[0].text, sizeof(b2)); raiseAlert(sigmet[0].hazard[0] ? sigmet[0].hazard : "SIGMET", b2, COL_ARR, M_RAIN, 8000, NULL); }

  if (fresh) {
    int found = -1;
    if (selectedIcao[0]) for (int i = 0; i < planeCount; i++) if (!strcmp(planes[i].icao24, selectedIcao)) { found = i; break; }
    selectedIndex = (found >= 0) ? found : 0;
    if (followIcao[0]) { bool still = false; for (int i = 0; i < planeCount; i++) if (!strcmp(planes[i].icao24, followIcao)) still = true; if (!still) followIcao[0] = 0; }
    staticDirty = true;
    for (int i = 0; i < MAX_PLANES; i++) if (trails[i].icao[0] && millis() - trails[i].lastMs > 180000) trails[i].icao[0] = 0;
    refineClasses();   // 6.14.1: before the stats, alerts and runway votes read p.cls (was after them)
    analysePlanes();
    replayRecord();
    // inner-ring alerts
    float inner = displayRangeKm() / 3.0f;
    if (alertArmed) {
      for (int i = 0; i < planeCount; i++) {
        if (planes[i].distance_km > inner || !passesFilter(planes[i])) continue;
        bool seen = false;
        for (int j = 0; j < prevInnerCount; j++) if (!strcmp(prevInner[j], planes[i].icao24)) { seen = true; break; }
        if (!seen && !toastActive()) {
          char pcs[12], b2[24]; prettyCallsign(planes[i].callsign, pcs, sizeof(pcs));
          if (planes[i].cls == CLS_ARRIVAL) snprintf(b2, sizeof(b2), "arriving rwy %02d ~%dm", landingRunway(planes[i]), etaMin(planes[i], planes[i].distance_km));
          else if (planes[i].cls == CLS_DEPARTURE) snprintf(b2, sizeof(b2), "departing rwy %02d", landingRunway(planes[i]));
          else snprintf(b2, sizeof(b2), "overflying FL%03d", (int)(planes[i].altitude_m * 3.28084f / 100));
          char b1[22]; snprintf(b1, sizeof(b1), "NEW %s", pcs);
          raiseAlert(b1, b2, planeColour(planes[i]), M_TRAFFIC, 4000, planes[i].icao24);
        }
      }
    }
    prevInnerCount = 0;
    for (int i = 0; i < planeCount && prevInnerCount < MAX_PLANES; i++) if (planes[i].distance_km <= inner) strlcpy(prevInner[prevInnerCount++], planes[i].icao24, 8);
    alertArmed = true;
  }
  if (planeCount > 0) {
    if (selectedIndex >= planeCount) selectedIndex = 0;
    strlcpy(selectedIcao, planes[selectedIndex].icao24, sizeof(selectedIcao));
    strlcpy((char*)wantedCallsign, planes[selectedIndex].callsign, 10);
    wantedLat = planes[selectedIndex].lat; wantedLon = planes[selectedIndex].lon;
    // destination weather request for a selected departure with a known route
    wantedDestLat = 0; wantedDestLon = 0;
    if (planes[selectedIndex].cls == CLS_DEPARTURE) for (int k = 0; k < MAX_ROUTES; k++) if (routesUI[k].valid && !strcmp(routesUI[k].callsign, planes[selectedIndex].callsign) && routesUI[k].dlat != 0) {
      wantedDestLat = routesUI[k].dlat; wantedDestLon = routesUI[k].dlon; const char *dash = strrchr(routesUI[k].route, '-'); strlcpy((char*)wantedDestCode, dash ? dash + 1 : "", 4); break;
    }
  } else { selectedIcao[0] = 0; ((char*)wantedCallsign)[0] = 0; wantedDestLat = 0; }
  strlcpy((char*)wantedIcao, selectedIcao, 8);
  refineClasses();
  if (fresh) lookUpTick();
}

// =====================================================================
// REPLAY RECORDING (arena: aviation), CHIME, HEAP HISTORY, ROAMING, SLIDESHOW
// =====================================================================
void replayRecord() {   // one sample per aircraft per minute while the aviation group owns the arena
  if (!REPLAY_ENABLED || arenaOwner != ARENA_AVIATION) return;
  ArenaAviation *a = ARENA_AV;
  if (a->lastSampleMs && millis() - a->lastSampleMs < 60000) return;
  a->lastSampleMs = millis(); if (a->minutes < 65000) a->minutes++;
  for (int i = 0; i < planeCount; i++) {
    const Plane &p = planes[i]; int slot = -1, oldest = -1;
    for (int k = 0; k < REPLAY_TRACKS; k++) {
      if (a->tr[k].icao[0] && !strcmp(a->tr[k].icao, p.icao24)) { slot = k; break; }
      if (!a->tr[k].icao[0]) { if (slot < 0) slot = k; }
      else if (oldest < 0 || a->tr[k].lastMin < a->tr[oldest].lastMin || (a->tr[k].lastMin == a->tr[oldest].lastMin && a->tr[k].n < a->tr[oldest].n)) oldest = k;
    }
    if (slot < 0) { if (oldest < 0 || a->tr[oldest].lastMin == a->minutes) continue; slot = oldest; }   // evict the aircraft unseen the longest, never one sampled this minute
    ReplayTrack &t = a->tr[slot];
    if (strcmp(t.icao, p.icao24)) { strlcpy(t.icao, p.icao24, 8); t.n = 0; t.head = 0; }
    t.lastMin = a->minutes;
    ReplayPt pt; pt.dlat = (int16_t)((p.lat - homeLat) * 11132.0); pt.dlon = (int16_t)((p.lon - homeLon) * 11132.0 * cos(toRad(homeLat)));
    t.pts[t.head] = pt; t.head = (t.head + 1) % REPLAY_LEN; if (t.n < REPLAY_LEN) t.n++;
  }
}
void chimeTick() {
  if (!chimeOn || sleeping) return;
  struct tm t; if (!getTime(t)) return;
  if (t.tm_min == 0 && t.tm_sec < 2 && lastChimeHour != t.tm_hour && !quietNow()) { lastChimeHour = t.tm_hour; playMelody(M_CHIME, false); }
}
// Heap guard: if the largest free block stays below HEAP_GUARD_MIN for 10 minutes the board is crippled
// (every HTTPS request "connection refused"); restart at a quiet moment.
// The old threshold was 40000, a guess. Measured 9 Oct on 5.5.2: the device runs perfectly with every
// endpoint connecting at a block of 31,732 to 34,804, so 40000 was never reached. That left seenGood
// false, which is the only reason it never fired. One transient above 40000 would have latched seenGood
// and then restarted a completely healthy board every 10 minutes, twice per power cycle.
#define HEAP_GUARD_MIN 28000
RTC_DATA_ATTR int heapGuardRestarts = 0;     // survives software resets, cleared by power-on
void heapGuardTick() {
  static unsigned long lastCheck = 0, lowSince = 0; static bool warned = false, seenGood = false;
  if (millis() - lastCheck < 60000UL) return; lastCheck = millis();
  uint32_t blk = ESP.getMaxAllocHeap();
  if (blk >= HEAP_GUARD_MIN) { seenGood = true; if (lowSince) logf("heap guard: largest block back to %u\n", blk); lowSince = 0; warned = false; return; }
  if (!lowSince) lowSince = millis();
  if (!warned) { logf("heap guard: largest block %u < %u (heap %u)%s\n", blk, (unsigned)HEAP_GUARD_MIN, ESP.getFreeHeap(), seenGood ? ", restart in 10 min if it stays low" : " since boot, no restart (structural)"); warned = true; }
  // restart only if the block was healthy earlier in this run (so it degraded), never more than twice per power cycle
  if (seenGood && heapGuardRestarts < 2 && millis() - lowSince > 600000UL && millis() - bootMs > 300000UL && pomoState != POMO_FOCUS && pomoState != POMO_BREAK) {
    heapGuardRestarts++; logf("heap guard: largest block %u for 10 min, restarting (%d)\n", blk, heapGuardRestarts); delay(300); ESP.restart();
  }
}
// clock text honouring the 12/24 h setting: "14:05" or "2:05" (+ ampmStr() = "PM")
int hour12(int h) { int x = h % 12; return x == 0 ? 12 : x; }
const char *ampmStr(int h) { return !clock12 ? "" : (h < 12 ? "AM" : "PM"); }
void fmtHM(char *b, size_t n, int h, int m) { if (clock12) snprintf(b, n, "%d:%02d", hour12(h), m); else snprintf(b, n, "%02d:%02d", h, m); }
void fmtHMA(char *b, size_t n, int h, int m) { if (clock12) snprintf(b, n, "%d:%02d %s", hour12(h), m, ampmStr(h)); else snprintf(b, n, "%02d:%02d", h, m); }
void heapTick() {
  heapGuardTick();
  if (lastHeapSampleMs && millis() - lastHeapSampleMs < 1800000UL) return;
  lastHeapSampleMs = millis();
  uint8_t f = fpsShown > 255 ? 255 : (uint8_t)fpsShown;
  if (heapHistN < 48) { heapHist[heapHistN] = ESP.getFreeHeap() / 1024; fpsHist[heapHistN] = f; heapHistN++; }
  else { memmove(heapHist, heapHist + 1, sizeof(uint16_t) * 47); memmove(fpsHist, fpsHist + 1, 47); heapHist[47] = ESP.getFreeHeap() / 1024; fpsHist[47] = f; }
}
void roamTick() {
  static unsigned long last = 0; if (millis() - last < 5000 || !wifiUp) return; last = millis();
  uint8_t *b = WiFi.BSSID(); if (!b) return;
  if (memcmp(b, lastBssid, 6)) {
    if (lastBssid[0] || lastBssid[1] || lastBssid[2]) { for (int i = 7; i > 0; i--) roamLog[i] = roamLog[i-1]; roamLog[0].t = (uint32_t)time(NULL); memcpy(roamLog[0].mac, b + 3, 3); if (roamCount < 8) roamCount++; logf("roamed to node %02x:%02x:%02x\n", b[3], b[4], b[5]); }
    memcpy(lastBssid, b, 6);
  }
}
void enterApp(int a);
void slideTick() {
  if (slideMin <= 0 || sleeping || menuOpen || alarmRinging || toastActive()) return;
  if (pomoState == POMO_FOCUS || pomoState == POMO_BREAK || pomoState == POMO_PAUSED) { lastInputMs = millis(); return; }   // a running timer counts as activity
  if (lastInputMs == 0) lastInputMs = millis();
  bool idle = millis() - lastInputMs > (unsigned long)slideMin * 60000UL;
  if (!idle) return;
  if (slideMode == 1) { if (currentApp != APP_RADAR) { enterApp(APP_RADAR); logln("idle: back to radar"); } return; }
  static const App ring[4] = {APP_CLOCK, APP_WEATHER, APP_STATS, APP_RADAR};
  if (!slideshow) { slideshow = true; slideReturnApp = currentApp; slideIdx = 0; slideSwitchMs = 0; }
  if (millis() - slideSwitchMs > 20000) { slideSwitchMs = millis(); enterApp(ring[slideIdx]); slideIdx = (slideIdx + 1) % 4; }
}
bool noteInput() { lastInputMs = millis(); if (slideshow) { slideshow = false; enterApp(slideReturnApp); return true; } return false; }   // true: this input only ended the slideshow
void nodeStr(char *out, size_t n) { uint8_t *bs = wifiUp ? WiFi.BSSID() : NULL; if (!bs) strlcpy(out, "--", n); else snprintf(out, n, "%02X:%02X:%02X", bs[3], bs[4], bs[5]); }

// =====================================================================
// WEATHER / MARKET / QUAKE EVENTS, HISTORY
// =====================================================================
void weatherEvents() {
  struct tm t; if (!getTime(t) || !wx.valid) return;
  int nowMin = t.tm_hour * 60 + t.tm_min; uint32_t day = t.tm_yday + 1;
  // rain soon (next 2 h in 15-min steps)
  if (millis() - lastRainAlertMs > 7200000UL || lastRainAlertMs == 0) {
    for (int i = 0; i < 8; i++) if (wx.rain15[i] >= 0.2f) {
      char b1[22], b2[24]; snprintf(b1, sizeof(b1), "RAIN in %d min", i * 15 + 5); snprintf(b2, sizeof(b2), "%.1f mm expected", wx.rain15[i]);
      raiseAlert(b1, b2, COL_ENROUTE_HIGH, M_RAIN, 8000, NULL); lastRainAlertMs = millis(); break;
    }
  }
  if (wx.sunsetMin > 0) {
    if (nowMin == wx.sunsetMin - 60 && lastGoldenAlertDay != day) { lastGoldenAlertDay = day; raiseAlert("GOLDEN HOUR", "best light for the next hour", COL_ARR, M_SUN, 8000, NULL); }
    if (nowMin == wx.sunsetMin - 15 && lastSunsetAlertDay != day) { lastSunsetAlertDay = day; char b2[24]; snprintf(b2, sizeof(b2), "sunset at %02d:%02d", wx.sunsetMin / 60, wx.sunsetMin % 60); raiseAlert("SUNSET in 15 min", b2, COL_ARR, M_SUN, 8000, NULL); }
  }
  // 15-min history samples
  if (millis() - lastHistMs > 900000UL || lastHistMs == 0) {
    lastHistMs = millis(); tempHist[histIdx] = (int8_t)lroundf(wx.temp); aqiHist[histIdx] = wx.aqi > 255 ? 255 : wx.aqi;
    histIdx = (histIdx + 1) % 96; if (histIdx == 0) histWrapped = true;
  }
}
void marketEvents() {
  struct tm t; if (!getTime(t)) return;
  if (marketAlertedDay != (uint32_t)t.tm_yday) { marketAlertedDay = t.tm_yday; marketAlerted = 0; }
  bool sessionOpen = marketOpenNow(t);
  for (int i = 0; i < NUM_SYMBOLS; i++) {
    if (!quotes[i].valid || MARKET_ALERT_PCT[i] <= 0 || (marketAlerted & (1 << i))) continue;
    // Symbols 0-3 trade the NSE session. Before the open the quote still carries yesterday's close, and
    // marketAlerted is one shot per symbol per day, so alerting on it would consume the slot and the real
    // intraday move would then never alert. Symbols 4-6 (USDINR, gold, bitcoin) run around the clock.
    if (i <= 3 && !sessionOpen) continue;
    float chg = quotes[i].prevClose > 0 ? (quotes[i].price - quotes[i].prevClose) / quotes[i].prevClose * 100 : 0;
    if (fabsf(chg) >= MARKET_ALERT_PCT[i]) {
      marketAlerted |= (1 << i); char b1[22], b2[24]; fmtPrice(quotes[i].price, b2, sizeof(b2), quotes[i].price > 9999 ? 0 : 2);
      snprintf(b1, sizeof(b1), "%s %+.2f%%", MARKET_SHORT[i], chg); raiseAlert(b1, b2, chg >= 0 ? COL_OK : COL_SELECTED, M_MARKET, 8000, NULL);
    }
  }
}
float moonPhase() {   // 0 new .. 0.5 full .. 1 new
  time_t now = time(NULL); double days = (now - 947182440.0) / 86400.0;   // 2000-01-06 18:14 UTC new moon
  double ph = fmod(days / 29.530588853, 1.0); if (ph < 0) ph += 1; return (float)ph;
}
void summaryTick() {   // once a day at SUMMARY_HOUR: toast + keep yesterday's numbers for the STATS page
  if (SUMMARY_HOUR < 0) return;
  struct tm t; if (!getTime(t) || t.tm_hour != SUMMARY_HOUR || t.tm_min != 0 || summaryDay == t.tm_yday) return;
  summaryDay = t.tm_yday;
  int n = histWrapped ? 96 : histIdx; int8_t tmin = 127, tmax = -128; uint8_t amax = 0;
  for (int k = 0; k < n; k++) { int i = (histIdx - n + k + 96) % 96; if (tempHist[i] < tmin) tmin = tempHist[i]; if (tempHist[i] > tmax) tmax = tempHist[i]; if (aqiHist[i] > amax) amax = aqiHist[i]; }
  DaySummary d = {}; d.seen = seenToday; d.arr = seenArr; d.dep = seenDep; d.enr = seenEnr; d.focusMin = pomoTodayMin; d.outages = 0;
  uint32_t dayStart = (uint32_t)time(NULL) - (t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec);
  for (int i = 0; i < outageCount; i++) if (outages[i].start >= dayStart) d.outages++;
  d.tmin = n ? tmin : 0; d.tmax = n ? tmax : 0; d.aqiMax = amax;
  yesterday = d;
  prefs.begin("radar", false); prefs.putBytes("yday", &yesterday, sizeof(yesterday)); prefs.end();
  char b1[22], b2[24]; snprintf(b1, sizeof(b1), "TODAY %d aircraft", d.seen);
  snprintf(b2, sizeof(b2), "foc %dm AQI<=%d out %d", d.focusMin, d.aqiMax, d.outages);
  raiseAlert(b1, b2, COL_ENROUTE_HIGH, M_SUN, 10000, NULL);
}
void quakeEvents() {
  if (!quakes[0].valid) return;
  uint32_t nowT = (uint32_t)time(NULL);
  if (quakes[0].t > lastQuakeAlertT && nowT > quakes[0].t && nowT - quakes[0].t < 7200 && quakes[0].mag >= 5.0f) {
    lastQuakeAlertT = quakes[0].t; char b1[22], b2[24]; snprintf(b1, sizeof(b1), "M%.1f EARTHQUAKE", quakes[0].mag); snprintf(b2, sizeof(b2), "%.0f km, %s", quakes[0].distKm, quakes[0].place); b2[23] = 0;
    raiseAlert(b1, b2, COL_SELECTED, M_QUAKE, 10000, NULL);
  }
}

// =====================================================================
// POMODORO, NUDGES, ALARM, SLEEP
// =====================================================================
int pomoFocusMin() { if (pomoPreset == POMO_TIMER_PRESET) return pomoTimerMin; return POMO_PRESETS[pomoPreset].focus ? POMO_PRESETS[pomoPreset].focus : pomoCustomFocus; }
int pomoBreakMin() { if (POMO_PRESETS[pomoPreset].brk) return POMO_PRESETS[pomoPreset].brk; int b = pomoCustomFocus / 5; return b < 3 ? 3 : b; }
void loadPomoHist() { prefs.begin("radar", true); if (prefs.getBytesLength("ph") == sizeof(pomoHist)) prefs.getBytes("ph", pomoHist, sizeof(pomoHist)); prefs.end(); }
void savePomoHist() { prefs.begin("radar", false); prefs.putBytes("ph", pomoHist, sizeof(pomoHist)); prefs.end(); }
void pomoRecordToday() {
  struct tm t; if (!getTime(t)) return;
  uint16_t day = (uint16_t)((t.tm_year - 100) * 400 + t.tm_yday);
  int slot = -1; for (int i = 0; i < 7; i++) if (pomoHist[i].day == day) slot = i;
  if (slot < 0) { for (int i = 6; i > 0; i--) pomoHist[i] = pomoHist[i-1]; slot = 0; pomoHist[0].day = day; }
  pomoHist[slot].sessions = pomoToday; pomoHist[slot].minutes = pomoTodayMin; savePomoHist();
}
void loadFocusQ() { prefs.begin("radar", true); if (prefs.getBytesLength("fq") == sizeof(focusQ)) { prefs.getBytes("fq", focusQ, sizeof(focusQ)); focusQn = prefs.getUChar("fqn", 0); if (focusQn > FOCUS_Q) focusQn = 0; } prefs.end(); }
void saveFocusQ() {   // loop(): snapshot under the mutex (the fetch task pops the queue), then write
  FocusRec copy[FOCUS_Q]; uint8_t n; xSemaphoreTake(dataMutex, portMAX_DELAY); memcpy(copy, focusQ, sizeof(copy)); n = focusQn; xSemaphoreGive(dataMutex);
  prefs.begin("radar", false); prefs.putBytes("fq", copy, sizeof(copy)); prefs.putUChar("fqn", n); prefs.end();
}
unsigned long pomoRemainMs();
void focusLogEnd(bool completed) {   // call when a focus session ends: completed, or stopped early (partial)
  if (focusStartEpoch == 0) return;
  uint32_t nowE = (uint32_t)time(nullptr);
  unsigned long doneMs = completed ? pomoTotalMs : pomoTotalMs - pomoRemainMs();
  uint16_t act = (uint16_t)(doneMs / 60000UL);
  FocusRec r = {}; r.start = focusStartEpoch; r.end = nowE; r.planMin = focusPlanMin; r.actMin = act; r.done = completed ? 1 : 0;
  strlcpy(r.task, pomoTask, sizeof(r.task));
  focusStartEpoch = 0;
  if (act < (completed ? 1 : pcfg.minLog) || nowE < 1700000000UL) return;   // too short, or clock not set: skip
  focusAddToday(act);
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  if (focusQn >= FOCUS_Q) { memmove(&focusQ[0], &focusQ[1], sizeof(FocusRec) * (FOCUS_Q - 1)); focusQn = FOCUS_Q - 1; }   // full: drop the oldest
  focusQ[focusQn++] = r;
  xSemaphoreGive(dataMutex);
  saveFocusQ();
  logf("focus log queued: %d min %s (%d waiting)\n", (int)act, completed ? "done" : "partial", focusQn);
}
// ---------- Focus Pro logic ----------
const char *hname() { return (pcfg.greet && pcfg.name[0]) ? pcfg.name : ""; }
void withName(char *out, size_t n, const char *s, const char *tail) { if (hname()[0]) snprintf(out, n, "%s, %s%s", s, hname(), tail); else snprintf(out, n, "%s%s", s, tail); }
void fmtHMs(char *b, size_t n, unsigned m) { if (m < 60) snprintf(b, n, "%um", m); else if (m % 60 == 0) snprintf(b, n, "%uh", m / 60); else snprintf(b, n, "%uh %02um", m / 60, m % 60); }
uint16_t dayKeyOf(const struct tm &t) { struct tm d = t; d.tm_hour = 12; d.tm_min = d.tm_sec = 0; d.tm_isdst = -1; return (uint16_t)(mktime(&d) / 86400); }   // 6.14.1: consecutive days are consecutive keys across New Year
uint16_t todayKey() { struct tm t; if (!getTime(t)) return 0; return dayKeyOf(t); }
uint16_t todayFocus() { uint16_t k = todayKey(); uint16_t m = (k && ftoday.day == k) ? ftoday.mins : 0; if (fstats.valid && fstats.day == k && fstats.todayMin > m) m = fstats.todayMin; return m; }
uint16_t todaySessions() { uint16_t k = todayKey(); uint16_t n = (k && ftoday.day == k) ? ftoday.sess : 0; if (fstats.valid && fstats.day == k && fstats.sess > n) n = fstats.sess; return n; }
void saveFocusToday() { prefs.begin("radar", false); prefs.putBytes("ftd", &ftoday, sizeof(ftoday)); prefs.end(); }
void focusAddToday(uint16_t m) { uint16_t k = todayKey(); if (!k) return; if (ftoday.day != k) { ftoday.day = k; ftoday.mins = 0; ftoday.sess = 0; } ftoday.mins += m; ftoday.sess++; saveFocusToday(); }
uint16_t histMinutes(uint16_t key) { for (int i = 0; i < 7; i++) if (pomoHist[i].day == key) return pomoHist[i].minutes; return 0; }
uint16_t streakNow() {
  uint16_t k = todayKey(); bool met = todayFocus() >= pcfg.goalMin;
  if (fstats.valid) { uint16_t st = fstats.streak; if (fstats.day == k && met && fstats.todayMin < pcfg.goalMin) st++; return st; }
  uint16_t st = met ? 1 : 0;                                  // offline: last 7 days kept on the device
  for (int d = 1; d < 7 && k > d; d++) { if (histMinutes(k - d) >= pcfg.goalMin) st++; else break; }
  return st;
}
uint16_t last7(int i) {   // i = 0 oldest .. 6 today
  if (i == 6) return todayFocus();
  if (fstats.valid && fstats.day == todayKey()) return fstats.last7[i];
  uint16_t k = todayKey(); return k > 6 - i ? histMinutes(k - (6 - i)) : 0;
}
void pomoSound(int mel) { if (pcfg.sound == 2 || quietNow()) return; if (pcfg.sound == 1) { beep(1500, 50); return; } playMelody(mel, false); }
// overlays: full-screen moments drawn over any app (greetings, session done, goal met, streak, recap)
void ovPush(uint8_t kind, unsigned long dur, const char *l1, const char *l2, const char *l3, uint16_t a, uint16_t b) {
  Overlay o = {}; o.kind = kind; o.durMs = pcfg.anim == 2 ? (dur > 2500 ? 2500 : dur) : dur; o.a = a; o.b = b;
  strlcpy(o.l1, l1 ? l1 : "", sizeof(o.l1)); strlcpy(o.l2, l2 ? l2 : "", sizeof(o.l2)); strlcpy(o.l3, l3 ? l3 : "", sizeof(o.l3));
  if (kind == OV_RECAP) o.durMs = dur;
  if (ov.kind == OV_NONE) { ov = o; ov.startMs = millis(); staticDirty = true; } else if (ovQn < 3) ovQ[ovQn++] = o;
}
bool ovActive() { return ov.kind != OV_NONE; }
void ovEnd() {
  if (ovQn) { ov = ovQ[0]; for (int i = 1; i < ovQn; i++) ovQ[i - 1] = ovQ[i]; ovQn--; ov.startMs = millis(); } else ov.kind = OV_NONE;
  staticDirty = true; clkS = -1; clkFaceDrawn = -1; lastHudMs = 0; drawInfo();
}
void ovTick() { if (ov.kind != OV_NONE && millis() - ov.startMs > ov.durMs) ovEnd(); }
bool streakMilestone(uint16_t n) { return n == 3 || n == 7 || n == 14 || n == 21 || n == 30 || n == 50 || n == 100 || n == 200 || n == 365; }
// after a session is logged: count-up screen, then goal met and streak milestone when crossed
void pomoCelebrate(uint16_t added, uint16_t before, bool done) {
  static const char *NICE[] = {"Nice work", "Well done", "Great focus", "Solid session", "Keep it up"};
  char l1[36]; uint16_t after = todayFocus();
  if (done) withName(l1, sizeof(l1), NICE[(after / 5 + todaySessions()) % 5], ""); else withName(l1, sizeof(l1), "Saved", "");
  ovPush(OV_SESSION, done ? 3500 : 2600, l1, done ? "SESSION DONE" : "SAVED", "", added, before);
  if (before < pcfg.goalMin && after >= pcfg.goalMin) {
    char g[36]; withName(g, sizeof(g), "Congratulations", "!");
    ovPush(OV_GOAL, 6000, g, "", "", after, 0); pomoSound(M_CHIME);
    uint16_t st = streakNow();
    if (streakMilestone(st)) { char m[36]; snprintf(m, sizeof(m), "%u days in a row%s%s", st, hname()[0] ? ", " : "", hname()); ovPush(OV_STREAK, 5000, m, "", "", st, 0); }
  }
}
const char *dayPart(int h) { return h < 5 ? "Hello" : h < 12 ? "Good morning" : h < 17 ? "Good afternoon" : h < 22 ? "Good evening" : "Hello"; }
void greet(int kind) {   // 0 boot, 1 first of the day, 2 welcome back
  struct tm t; if (!getTime(t)) return; char l1[36], l3[32], g[12];
  if (kind == 2) snprintf(l1, sizeof(l1), "Welcome back%s", hname()[0] ? "," : ""); else snprintf(l1, sizeof(l1), "%s%s", dayPart(t.tm_hour), hname()[0] ? "," : "");
  fmtHMs(g, sizeof(g), pcfg.goalMin);
  uint16_t tm = todayFocus();
  if (kind == 1 && histMinutes(todayKey() - 1)) { char y[12]; fmtHMs(y, sizeof(y), histMinutes(todayKey() - 1)); snprintf(l3, sizeof(l3), "yesterday %s, goal %s", y, g); }
  else if (tm) { char d[12]; fmtHMs(d, sizeof(d), tm); snprintf(l3, sizeof(l3), "%s of %s today", d, g); }
  else snprintf(l3, sizeof(l3), "goal today: %s", g);
  ovPush(OV_GREET, kind == 2 ? 2200 : 3500, l1, hname(), l3, kind, 0);
}
void greetTick() {   // boot greeting, first-of-day greeting, welcome back after the screens slept
  static bool booted = false, wasSleeping = false; static int lastDay = -1;
  struct tm t; if (!getTime(t)) return;
  if (!booted) { booted = true; wasSleeping = sleeping; lastDay = t.tm_yday; return; }   // 6.17.0: no greeting at boot (the boot sequence is the welcome)
  if (wasSleeping && !sleeping) { if (t.tm_yday != lastDay && t.tm_hour >= 5) { lastDay = t.tm_yday; greet(1); } else greet(2); }
  else if (!sleeping && t.tm_yday != lastDay && t.tm_hour >= 5 && millis() - lastInputMs < 3000) { lastDay = t.tm_yday; greet(1); }
  wasSleeping = sleeping;
}
void recapTick() {
  static int recapDay = -1;
  if (!pcfg.recapHour) return;
  struct tm t; if (!getTime(t) || t.tm_hour != pcfg.recapHour || t.tm_min != 0 || recapDay == t.tm_yday || sleeping) return;
  recapDay = t.tm_yday; statsWanted = true;
  char l1[36]; bool met = todayFocus() >= pcfg.goalMin;
  if (met) withName(l1, sizeof(l1), "Good day", "!"); else if (todayFocus()) withName(l1, sizeof(l1), "Good effort", ""); else withName(l1, sizeof(l1), "New day tomorrow", "");
  ovPush(OV_RECAP, 30000, l1, "", "", todayFocus(), 0);
}
// Pomodoro settings: 13 options, edited on the device (hold on Home) or the web page
#define PSET_N 13
const char *PSET_NAMES[PSET_N] = {"Daily goal", "Focus/break", "Long break", "Per cycle", "Auto-start", "Quiet mode", "Early end", "Min to log", "Streak days", "Day recap", "Animations", "Sounds", "Greetings"};
const char *PSET_HELP[PSET_N] = {"streak counts days that reach it", "preset used when you tap start", "after a full cycle of sessions", "sessions before a long break", "start focus when a break ends", "alerts held until your break", "what double tap or hold does", "shorter sessions are not logged", "Mon-Fri: weekends never break it", "summary screen once a day", "celebrations and pulses", "Pomodoro sounds only", "name is set on the web page"};
void psetValue(int i, char *b, size_t n) {
  static const char *Q[4] = {"off", "planes", "planes+mkt", "all"}, *E[3] = {"ask", "always log", "never log"}, *A[3] = {"full", "subtle", "off"}, *S[3] = {"chime", "soft", "silent"};
  switch (i) {
    case 0: fmtHMs(b, n, pcfg.goalMin); break;
    case 1: { int p = pomoPreset; if (p == POMO_TIMER_PRESET) p = 0; int f = POMO_PRESETS[p].focus ? POMO_PRESETS[p].focus : pomoCustomFocus; int k = POMO_PRESETS[p].brk ? POMO_PRESETS[p].brk : (pomoCustomFocus / 5 < 3 ? 3 : pomoCustomFocus / 5); snprintf(b, n, "%d / %d", f, k); break; }
    case 2: snprintf(b, n, "%u min", pcfg.longBreak); break;
    case 3: snprintf(b, n, "%u", pcfg.cycle); break;
    case 4: strlcpy(b, pcfg.autoStart ? "on" : "off", n); break;
    case 5: strlcpy(b, Q[pcfg.quiet & 3], n); break;
    case 6: strlcpy(b, E[pcfg.early % 3], n); break;
    case 7: snprintf(b, n, "%u min", pcfg.minLog); break;
    case 8: strlcpy(b, pcfg.streakWk ? "Mon-Fri" : "every day", n); break;
    case 9: if (pcfg.recapHour) snprintf(b, n, "%02u:00", pcfg.recapHour); else strlcpy(b, "off", n); break;
    case 10: strlcpy(b, A[pcfg.anim % 3], n); break;
    case 11: strlcpy(b, S[pcfg.sound % 3], n); break;
    default: strlcpy(b, pcfg.greet ? "on" : "off", n); break;
  }
}
int wrapI(int v, int lo, int hi) { return v < lo ? hi : v > hi ? lo : v; }
void psetStep(int i, int st) {
  switch (i) {
    case 0: { int g = pcfg.goalMin + 15 * st; if (g < 30) g = 30; if (g > 480) g = 480; pcfg.goalMin = g; statsWanted = true; break; }
    case 1: { int p = pomoPreset == POMO_TIMER_PRESET ? 0 : pomoPreset; pomoPreset = wrapI(p + st, 0, POMO_TIMER_PRESET - 1); break; }
    case 2: { int v = pcfg.longBreak + 5 * st; if (v < 10) v = 10; if (v > 30) v = 30; pcfg.longBreak = v; break; }
    case 3: { int v = pcfg.cycle + st; if (v < 2) v = 2; if (v > 6) v = 6; pcfg.cycle = v; break; }
    case 4: pcfg.autoStart ^= 1; break;
    case 5: pcfg.quiet = wrapI(pcfg.quiet + st, 0, 3); break;
    case 6: pcfg.early = wrapI(pcfg.early + st, 0, 2); break;
    case 7: { static const uint8_t M[3] = {1, 5, 10}; int k = pcfg.minLog >= 10 ? 2 : pcfg.minLog >= 5 ? 1 : 0; pcfg.minLog = M[wrapI(k + st, 0, 2)]; break; }
    case 8: pcfg.streakWk ^= 1; statsWanted = true; break;
    case 9: { static const uint8_t H[6] = {0, 17, 18, 19, 20, 21}; int k = 0; for (int j = 0; j < 6; j++) if (H[j] == pcfg.recapHour) k = j; pcfg.recapHour = H[wrapI(k + st, 0, 5)]; break; }
    case 10: pcfg.anim = wrapI(pcfg.anim + st, 0, 2); break;
    case 11: pcfg.sound = wrapI(pcfg.sound + st, 0, 2); break;
    default: pcfg.greet ^= 1; break;
  }
  settingsDirty = true;
}
void pcfgSanitize() {
  if (pcfg.goalMin < 30 || pcfg.goalMin > 480) pcfg.goalMin = 180;
  if (pcfg.longBreak < 10 || pcfg.longBreak > 30) pcfg.longBreak = 15;
  if (pcfg.cycle < 2 || pcfg.cycle > 6) pcfg.cycle = 4;
  if (pcfg.quiet > 3) pcfg.quiet = 3;
  if (pcfg.early > 2) pcfg.early = 0;
  if (pcfg.minLog != 1 && pcfg.minLog != 5 && pcfg.minLog != 10) pcfg.minLog = 1;
  if (pcfg.recapHour != 0 && (pcfg.recapHour < 17 || pcfg.recapHour > 21)) pcfg.recapHour = 19;
  if (pcfg.anim > 2) pcfg.anim = 0;
  if (pcfg.sound > 2) pcfg.sound = 0;
  pcfg.autoStart = pcfg.autoStart ? 1 : 0; pcfg.streakWk = pcfg.streakWk ? 1 : 0; pcfg.greet = pcfg.greet ? 1 : 0;
  pcfg.name[sizeof(pcfg.name) - 1] = 0;
}
void putFocusPro() { prefs.putBytes("pcfg", &pcfg, sizeof(pcfg)); }   // called inside saveSettings (prefs already open)
void loadFocusPro() {
  prefs.begin("radar", true);
  if (prefs.getBytesLength("pcfg") == sizeof(pcfg)) prefs.getBytes("pcfg", &pcfg, sizeof(pcfg));
  if (prefs.getBytesLength("ftd") == sizeof(ftoday)) prefs.getBytes("ftd", &ftoday, sizeof(ftoday));
  prefs.end();
  pcfgSanitize();
}
void pomoResetToSelect() { pomoState = POMO_SELECT; pomoEditing = false; breathing = false; pomoIsTimer = false; pomoAskLog = false; staticDirty = true; }
bool pomoFocusActive() { return !pomoIsTimer && (pomoState == POMO_FOCUS || (pomoState == POMO_PAUSED && pomoWasFocus)); }
void pomoFinishAsk(bool log);
void pomoAskEnd() {   // double tap or hold during focus: pause, then ask whether to log it
  if (pomoState == POMO_FOCUS) { pomoRemainAtPause = pomoRemainMs(); pomoWasFocus = true; pomoState = POMO_PAUSED; }
  pomoPausedAtMs = millis();
  if ((pomoTotalMs - pomoRemainAtPause) < (unsigned long)pcfg.minLog * 60000UL) { char b2[24]; snprintf(b2, sizeof(b2), "<%u min, not logged", pcfg.minLog); focusStartEpoch = 0; pomoResetToSelect(); raiseAlert("SESSION ENDED", b2, COL_TEXT_DIM, -1, 2500, NULL); return; }
  if (pcfg.early == 1) { pomoFinishAsk(true); return; }
  if (pcfg.early == 2) { pomoFinishAsk(false); return; }
  pomoAskLog = true; pomoAskSel = 0; staticDirty = true;
}
void pomoFinishAsk(bool log) {
  uint16_t before = todayFocus(), added = (uint16_t)((pomoTotalMs - pomoRemainAtPause) / 60000UL);
  if (log) focusLogEnd(false); else focusStartEpoch = 0;
  pomoResetToSelect();
  if (log && todayFocus() > before) pomoCelebrate(added, before, false);
  raiseAlert(log ? "SESSION LOGGED" : "NOT LOGGED", log ? "sheet + calendar" : "session discarded", log ? COL_OK : COL_TEXT_DIM, -1, 2500, NULL);
}
void pomoStopFocus() {   // the session is being reset: log what was done so far
  if (!pomoIsTimer && (pomoState == POMO_FOCUS || (pomoState == POMO_PAUSED && pomoWasFocus))) focusLogEnd(false);
  focusStartEpoch = 0;
}
void pomoStart(bool focus) {
  pomoIsTimer = (pomoPreset == POMO_TIMER_PRESET); breathing = false;
  int mins = focus ? pomoFocusMin() : ((pomoSession % POMO_SESSIONS_PER_CYCLE == 0 && pomoSession > 0) ? POMO_LONG_BREAK_MIN : pomoBreakMin());
  pomoTotalMs = (unsigned long)mins * 60000UL; pomoEndMs = millis() + pomoTotalMs;
  pomoState = focus ? POMO_FOCUS : POMO_BREAK; staticDirty = true;
  if (focus && !pomoIsTimer) { time_t e = time(nullptr); focusStartEpoch = e > 1700000000 ? (uint32_t)e : 0; focusPlanMin = mins; } else focusStartEpoch = 0;
  if (focus && !pomoIsTimer) {
    heldPlanes = heldMkt = heldOther = 0;
    static int lateDay = -1; struct tm t; char l[36];
    if (getTime(t) && t.tm_hour >= 22 && lateDay != t.tm_yday) { lateDay = t.tm_yday; withName(l, sizeof(l), "Late one", ""); ovPush(OV_GREET, 2500, l, "", "wrap up soon?", 3, 0); }
    else if (todayFocus() == 0 && todaySessions() == 0 && pomoSession == 0) { withName(l, sizeof(l), "Let's go", ""); ovPush(OV_GREET, 1400, l, "", "", 4, 0); }
  }
}
unsigned long pomoRemainMs() { if (pomoState == POMO_PAUSED) return pomoRemainAtPause; long r = (long)(pomoEndMs - millis()); return r < 0 ? 0 : (unsigned long)r; }
void pomoTick() {
  static unsigned long lastDayChk = 0;
  if (focusQDirty) { focusQDirty = false; saveFocusQ(); }   // flash writes stay on this task
  if (millis() - lastDayChk > 5000) { lastDayChk = millis(); struct tm t; if (getTime(t) && t.tm_yday != pomoDay) { pomoDay = t.tm_yday; pomoToday = 0; pomoTodayMin = 0; pomoSession = 0; nudgesToday = 0; statsWanted = true; staticDirty = true; } }
  if (pomoState != POMO_FOCUS && pomoState != POMO_BREAK) return;
  if ((long)(millis() - pomoEndMs) < 0) return;
  if (pomoState == POMO_FOCUS && pomoIsTimer) { pomoState = POMO_SELECT; raiseAlert("TIME'S UP", "timer finished", COL_ARR, M_TIMER, 8000, NULL); staticDirty = true; drawInfo(); return; }
  if (pomoState == POMO_FOCUS) { uint16_t before = todayFocus(); focusLogEnd(true); pomoSession++; pomoToday++; pomoTodayMin += pomoFocusMin(); pomoRecordToday(); raiseAlert("FOCUS DONE", pomoSession % POMO_SESSIONS_PER_CYCLE == 0 ? "long break, well earned" : "short break", COL_OK, -1, 6000, NULL); pomoSound(M_POMO); pomoCelebrate(pomoFocusMin(), before, true); pomoStart(false); }
  else { raiseAlert("BREAK OVER", pcfg.autoStart ? "back to focus" : "tap to start the next one", COL_SELECTED, -1, 6000, NULL); pomoSound(M_BREAK); if (pcfg.autoStart) pomoStart(true); else pomoResetToSelect(); }
}
void nudgeTick() {
  if (nudgeMin <= 0) return;
  struct tm t; if (!getTime(t)) return;
  bool workHours = t.tm_wday >= 1 && t.tm_wday <= 5 && t.tm_hour >= 9 && t.tm_hour < 18;
  if (!(pomoState == POMO_FOCUS || workHours)) { lastNudgeMs = millis(); return; }
  if (lastNudgeMs == 0) lastNudgeMs = millis();
  if (millis() - lastNudgeMs > (unsigned long)nudgeMin * 60000UL) { lastNudgeMs = millis(); nudgesToday++; raiseAlert("STAND UP", "stretch, water, eyes off", COL_ENROUTE_HIGH, M_NUDGE, 20000, NULL); }
}
void alarmOff();
void alarmTick() {
  struct tm t; if (!getTime(t) || !alarmOn) { if (!alarmOn && alarmRinging) { alarmRinging = false; stopMelody(); } return; }
  int nowMin = t.tm_hour * 60 + t.tm_min;
  bool today = alarmDays & (1 << t.tm_wday);
  if (!alarmRinging && today && nowMin == alarmMin && alarmFiredMin != alarmMin + t.tm_yday * 1440 && (!alarmSnoozeUntil || (long)(millis() - alarmSnoozeUntil) > 0)) {
    alarmFiredMin = alarmMin + t.tm_yday * 1440; alarmRinging = true; alarmRingStartMs = millis(); playMelody(M_ALARM, true);
    char b2[24], a1[22]; snprintf(b2, sizeof(b2), "%02d:%02d  click: snooze", t.tm_hour, t.tm_min); if (hname()[0]) snprintf(a1, sizeof(a1), "Wake up, %s", hname()); else strlcpy(a1, "ALARM", sizeof(a1)); raiseAlert(a1, b2, COL_ARR, -1, 600000, NULL);
    sleeping = false; wakeUntilMs = millis() + 600000;
  }
  if (alarmRinging && millis() - alarmRingStartMs > 600000UL) { alarmOff(); logln("alarm: auto off after 10 min"); }
  if (alarmRinging && alarmSnoozeUntil && (long)(millis() - alarmSnoozeUntil) < 0) { alarmRinging = false; stopMelody(); toast.untilMs = 0; }
  if (alarmSnoozeUntil && (long)(millis() - alarmSnoozeUntil) >= 0 && !alarmRinging && alarmFiredMin >= 0) { alarmSnoozeUntil = 0; alarmRinging = true; alarmRingStartMs = millis(); playMelody(M_ALARM, true); raiseAlert("ALARM", "click: snooze  hold: off", COL_ARR, -1, 600000, NULL); }
}
void alarmSnooze() { alarmSnoozeUntil = millis() + 5 * 60000UL; alarmRinging = false; stopMelody(); toast.untilMs = 0; raiseAlert("SNOOZE", "5 minutes", COL_TEXT_DIM, -1, 2000, NULL); }
void alarmOff() { alarmRinging = false; alarmSnoozeUntil = 0; stopMelody(); toast.untilMs = 0; }
bool inSleepHours() {
  if (sleepStart == sleepEnd) return false;
  struct tm t; if (!getTime(t)) return false; int h = t.tm_hour;
  return sleepStart < sleepEnd ? (h >= sleepStart && h < sleepEnd) : (h >= sleepStart || h < sleepEnd);
}
void setScreens(bool on) {
  tftCommand(0, on ? 0x29 : 0x28); tftCommand(1, on ? 0x29 : 0x28);   // DISPON / DISPOFF through the output task
  infoDisplay.ssd1306_command(on ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
  infoR.ssd1306_command(on ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
}
void sleepTick() {
  if (wakeUntilMs && (long)(millis() - wakeUntilMs) > 0) wakeUntilMs = 0;   // 6.14.1: wrap-safe
  bool want = inSleepHours() && !wakeUntilMs && !alarmRinging;
  if (want != sleeping) { sleeping = want; setScreens(!sleeping); staticDirty = true; logf("screens %s\n", sleeping ? "off (sleep hours)" : "on"); }
}

// =====================================================================
// RENDERING: shared helpers
// =====================================================================
#define DEG "\x7F"
#define FB  (&F_BIG60)
#define FB2 (&F_BIG36)
#define FM  (&F_MID22)
#define FMB (&F_MID22B)
#define FS  (&F_SM13)
#define FSR (&F_SM13R)
static const char *DN[] = {"SUN","MON","TUE","WED","THU","FRI","SAT"};
static const char *DNL[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
static const char *MN[] = {"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"};

void polar(float deg, float r, int &x, int &y) { float a = toRad(deg); x = RADAR_CX + (int)lroundf(r * sinf(a)); y = RADAR_CY - (int)lroundf(r * cosf(a)); }
void polarC(int cx, int cy, float deg, float r, int &x, int &y) { float a = toRad(deg); x = cx + (int)lroundf(r * sinf(a)); y = cy - (int)lroundf(r * cosf(a)); }
void drCenter(double &clat, double &clon) {   // radar centre: home, or the followed aircraft
  clat = homeLat; clon = homeLon;
  if (!followIcao[0]) return;
  for (int i = 0; i < planeCount; i++) if (!strcmp(planes[i].icao24, followIcao)) {
    const Plane &p = planes[i]; clat = p.lat; clon = p.lon;
#if DEAD_RECKON
    float dt = (millis() - lastFetchMs) / 1000.0f; if (dt > 120) dt = 120;
    double d = p.velocity_ms * dt, h = toRad(p.heading_deg);
    clat += (d * cos(h)) / 111320.0; clon += (d * sin(h)) / (111320.0 * cos(toRad(p.lat)));
#endif
    return;
  }
}
void planeScreenPos(const Plane &p, float rangeKm, double clat, double clon, int &x, int &y, float &distNow, float &brgNow) {
  double lat = p.lat, lon = p.lon;
#if DEAD_RECKON
  float dt = (millis() - lastFetchMs) / 1000.0f; if (dt > 120) dt = 120;
  double d = p.velocity_ms * dt, h = toRad(p.heading_deg);
  lat += (d * cos(h)) / 111320.0; lon += (d * sin(h)) / (111320.0 * cos(toRad(p.lat)));
#endif
  distNow = distanceKm(clat, clon, lat, lon); brgNow = bearingDeg(clat, clon, lat, lon);
  float r = (distNow / rangeKm) * RADAR_MAX_R;
  x = RADAR_CX + (int)lroundf(r * sinf(toRad(brgNow))); y = RADAR_CY - (int)lroundf(r * cosf(toRad(brgNow)));
}
const char *wmoText(int c) {
  if (c == 0)  return "Clear";
  if (c <= 2)  return "Partly cloudy";
  if (c == 3)  return "Overcast";
  if (c <= 48) return "Fog";
  if (c <= 57) return "Drizzle";
  if (c <= 67) return "Rain";
  if (c <= 77) return "Snow";
  if (c <= 82) return "Showers";
  if (c <= 86) return "Snow showers";
  return "Thunderstorm";
}
const char *wmoShort(int c) { if (c == 0) return "clear"; if (c <= 2) return "cloudy"; if (c == 3) return "overcast"; if (c <= 48) return "fog"; if (c <= 57) return "drizzle"; if (c <= 67) return "rain"; if (c <= 77) return "snow"; if (c <= 86) return "showers"; return "storm"; }
const char *aqiText(int a) { return a <= 50 ? "good" : a <= 100 ? "moderate" : a <= 150 ? "sensitive" : a <= 200 ? "unhealthy" : a <= 300 ? "very bad" : "hazardous"; }
uint16_t aqiColour(int aqi) {
  if (aqi <= 50)  return rgb(0, 228, 0);
  if (aqi <= 100) return rgb(255, 255, 0);
  if (aqi <= 150) return rgb(255, 126, 0);
  if (aqi <= 200) return rgb(255, 0, 0);
  if (aqi <= 300) return rgb(143, 63, 151);
  return rgb(126, 0, 35);
}
void dashedLine(int x0, int y0, int x1, int y1, uint16_t col) {
  float L = sqrtf((float)(x1-x0)*(x1-x0) + (float)(y1-y0)*(y1-y0)); int n = (int)(L / 4); if (n < 2) return;
  for (int i = 0; i < n; i += 2) {
    float t0 = (float)i / n, t1 = (float)(i + 1) / n;
    radar->drawLine(x0 + (int)((x1-x0)*t0), y0 + (int)((y1-y0)*t0), x0 + (int)((x1-x0)*t1), y0 + (int)((y1-y0)*t1), col);
  }
}
void centreText(int y, const char *s, int size, uint16_t col) {   // built-in 5x7 font, top-left y
  radar->setTextSize(size); radar->setTextColor(col);
  radar->setCursor(RADAR_CX - (int)(strlen(s) * 6 * size) / 2 + size / 2, y); radar->print(s);
}
// word-wrap `s` into up to `maxLines` lines no wider than `w` px; returns count
int wrapText(const char *s, const GFXfont *f, int w, char out[][40], int maxLines) {
  int lines = 0; const char *p = s;
  while (*p && lines < maxLines) {
    char line[40] = ""; int len = 0; const char *lastSpace = NULL; const char *q = p;
    while (*q && len < 39) {
      line[len++] = *q; line[len] = 0;
      if (radar->textWidth(f, line) > w) { if (lastSpace) { len = (int)(lastSpace - p); line[len] = 0; q = lastSpace + 1; } else { len--; line[len] = 0; } break; }
      if (*q == ' ') lastSpace = q;
      q++;
    }
    while (len > 0 && line[len-1] == ' ') line[--len] = 0;
    strlcpy(out[lines++], line, 40); p = q;
    while (*p == ' ') p++;
  }
  return lines;
}

// toast overlay (alerts), drawn on every band after the app
void paintToast() {
  if (!toastActive()) return;
  radar->fillRoundRect(34, 172, 172, 54, 10, rgb(12, 12, 12));
  radar->drawRoundRect(34, 172, 172, 54, 10, toast.col);
  radar->drawRoundRect(35, 173, 170, 52, 10, toast.col);
  radar->textC(194, toast.l1, FS, toast.col);
  radar->textC(214, toast.l2, FSR, COL_TEXT);
}
void paintMessage() {
  radar->fillScreen(COL_BG);
  radar->textC(100, msgL1, FMB, COL_SWEEP);
  radar->textC(128, msgL2, FSR, COL_TEXT);
  radar->textC(148, msgL3, FSR, COL_TEXT_DIM);
  if (msgPct >= 0) { radar->drawRoundRect(38, 164, 164, 10, 3, COL_TEXT_DIM); radar->fillRect(40, 166, msgPct * 160 / 100, 6, COL_SWEEP); }
}

// =====================================================================
// RADAR
// =====================================================================
int rangeLabelKm(int ring) { return (int)(displayRangeKm() / KM_PER_UNIT * ring / 3 + 0.5f); }
void drawFunnels() {
  if (followIcao[0]) return;
  for (int side = 0; side < 2; side++) {
    float a = rwyHdg() + side * 180;
    int x0, y0, x1, y1, x2, y2;
    polar(a, 10, x0, y0); polar(a - 6, 70, x1, y1); polar(a + 6, 70, x2, y2);
    radar->fillTriangle(x0, y0, x1, y1, x2, y2, dim565(rgb(0, 60, 24), 0.55f));
    for (int d = 14; d < 70; d += 8) { int xa, ya, xb, yb; polar(a, d, xa, ya); polar(a, d + 4, xb, yb); radar->drawLine(xa, ya, xb, yb, rgb(0, 90, 40)); }
  }
}
void drawRunwayAt(int cx, int cy, float scale) {
  int x0, y0, x1, y1; polarC(cx, cy, rwyHdg(), 8 * scale, x0, y0); polarC(cx, cy, rwyHdg() + 180, 8 * scale, x1, y1);
  for (int o = -1; o <= 1; o++) radar->drawLine(x0 + o, y0 + o, x1 + o, y1 + o, COL_HOME);
}
void drawTicks() {
  for (int d = 0; d < 360; d += 30) { int x0, y0, x1, y1; polar(d, RADAR_MAX_R, x0, y0); polar(d, RADAR_MAX_R - (d % 90 == 0 ? 8 : 5), x1, y1); radar->drawLine(x0, y0, x1, y1, COL_RING); }
  int ax, ay, bx, by, cx, cy; polar(0, RADAR_MAX_R, ax, ay); polar(-4, RADAR_MAX_R - 10, bx, by); polar(4, RADAR_MAX_R - 10, cx, cy);
  radar->fillTriangle(ax, ay, bx, by, cx, cy, COL_TEXT);
}
void drawStatic() {
  for (int i = 1; i <= 3; i++) {
    int r = (RADAR_MAX_R * i) / 3;
    radar->drawCircle(RADAR_CX, RADAR_CY, r, i == 3 ? COL_RING : COL_RING_DIM);
    if (i < 3) { char b[8]; snprintf(b, sizeof(b), "%d", rangeLabelKm(i)); radar->text(RADAR_CX + 3, RADAR_CY - r + 11, b, FSR, COL_RING); }
  }
  radar->drawLine(RADAR_CX, RADAR_CY - RADAR_MAX_R, RADAR_CX, RADAR_CY + RADAR_MAX_R, COL_RING_DIM);
  radar->drawLine(RADAR_CX - RADAR_MAX_R, RADAR_CY, RADAR_CX + RADAR_MAX_R, RADAR_CY, COL_RING_DIM);
  drawFunnels(); if (!followIcao[0]) drawRunwayAt(RADAR_CX, RADAR_CY, 1); drawTicks();
  // landmarks (dim) when zoomed in
  float rk = displayRangeKm();
  if (rk <= 60 && !followIcao[0]) for (int i = 0; i < NUM_LANDMARKS; i++) {
    float d = distanceKm(homeLat, homeLon, LANDMARKS[i].lat, LANDMARKS[i].lon); if (d > rk * 0.9f) continue;
    int x, y; polar(bearingDeg(homeLat, homeLon, LANDMARKS[i].lat, LANDMARKS[i].lon), d / rk * RADAR_MAX_R, x, y);
    radar->fillRect(x - 1, y - 1, 3, 3, rgb(60, 60, 70));
    radar->text(x + 4, y + 4, LANDMARKS[i].name, FSR, rgb(70, 70, 85));
  }
}
void drawSweep(float angle) {
  for (int i = 0; i <= SWEEP_TRAIL_LINES; i++) {
    float a = angle - i * 2.7f;   // fixed trail spacing (the sweep is time-based now)
    int x, y; polar(a, RADAR_MAX_R - 1, x, y);
    radar->drawLine(RADAR_CX, RADAR_CY, x, y, dim565(COL_SWEEP, 1.0f - (float)i / (SWEEP_TRAIL_LINES + 1)));
  }
}
static void rotPt(int x, int y, float ca, float sa, float sz, float px, float py, int &ox, int &oy) {
  ox = x + (int)lroundf((px*ca - py*sa) * sz); oy = y + (int)lroundf((px*sa + py*ca) * sz);
}
const TypeRow *typeRowFor(const Plane &p) {   // longest matching prefix
  const TypeRow *best = NULL; size_t bestLen = 0;
  if (!p.type[0]) return NULL;
  for (int i = 0; i < NUM_TYPES; i++) { size_t L = strlen(TYPES[i].prefix); if (L > bestLen && !strncmp(p.type, TYPES[i].prefix, L)) { best = &TYPES[i]; bestLen = L; } }
  return best;
}
uint8_t iconKindFor(const Plane &p) {   // 0 narrow .. 7 military, see TypeRow
  const TypeRow *t = typeRowFor(p);
  if (t) return t->kind;
  if (p.cat == 7) return 5;
  if (p.cat == 5) return 1;
  if (p.cat == 1) return 6;
  if (p.cat == 2) return 4;
  return 0;
}
// Parametric silhouette: fuselage along the heading, wingspan and sweep from the type table,
// engines as dots (jets) or small circles (props), helicopters as a rotor cross.
void drawPlaneIcon(int x, int y, float heading, uint16_t col, float sz, const Plane &p) {
  const TypeRow *t = typeRowFor(p); uint8_t kind = iconKindFor(p);
  float a = toRad(heading), ca = cosf(a), sa = sinf(a);
  const float kindScale[8] = {1.0f, 1.25f, 0.9f, 0.9f, 0.75f, 0.8f, 0.7f, 1.2f};
  sz *= kindScale[kind];
  #define R(px, py, ox, oy) rotPt(x, y, ca, sa, sz, px, py, ox, oy)
  int ax, ay, bx, by, cx, cy, dx, dy;
  if (kind == 5) {   // helicopter
    R(0,-6, ax,ay); R(-2.2f,4, bx,by); R(2.2f,4, cx,cy); radar->fillTriangle(ax,ay, bx,by, cx,cy, col);
    R(0,4, ax,ay); R(0,10, bx,by); radar->drawLine(ax,ay, bx,by, col); R(-2,10, ax,ay); R(2,10, bx,by); radar->drawLine(ax,ay, bx,by, col);
    R(-8,-2, ax,ay); R(8,2, bx,by); radar->drawLine(ax,ay, bx,by, col); R(-8,2, ax,ay); R(8,-2, bx,by); radar->drawLine(ax,ay, bx,by, col);
    return;
  }
  float span = t ? t->span : (kind == 1 ? 60 : kind == 6 ? 12 : 36);
  float half = 4.5f + span / 7.5f; if (half > 13) half = 13; if (half < 5) half = 5;      // wing half-span in units
  float sweep = t ? t->sweep / 6.0f : (kind == 3 || kind == 6 ? 0 : 4);                // wing tip offset back
  int engines = t ? t->engines : (kind == 1 ? 2 : kind == 6 ? 1 : 2);
  float len = kind == 1 ? 10 : kind == 6 ? 7 : kind == 4 ? 8 : 9;                        // half fuselage length
  float wingRoot = -1 + (kind == 1 ? 1 : 0);
  // wings
  R(-half, wingRoot + sweep, ax,ay); R(half, wingRoot + sweep, bx,by); R(0, wingRoot - 2, cx,cy); R(0, wingRoot + 3.5f, dx,dy);
  radar->fillTriangle(ax,ay, bx,by, cx,cy, col); radar->fillTriangle(ax,ay, bx,by, dx,dy, col);
  // fuselage
  R(0,-len, ax,ay); R(-2.0f, len * 0.55f, bx,by); R(2.0f, len * 0.55f, cx,cy); radar->fillTriangle(ax,ay, bx,by, cx,cy, col);
  R(-1.8f, -2, ax,ay); R(1.8f, -2, bx,by); R(0, len, cx,cy); radar->fillTriangle(ax,ay, bx,by, cx,cy, col);
  // tail
  float tailHalf = half * 0.42f; if (tailHalf < 3) tailHalf = 3;
  R(-tailHalf, len, ax,ay); R(tailHalf, len, bx,by); R(0, len - 3.5f, cx,cy); radar->fillTriangle(ax,ay, bx,by, cx,cy, col);
  // engines
  uint16_t eng = dim565(col, 0.45f);
  if (kind == 3 || kind == 6 || kind == 7) {   // propellers: small circles on the wings (or the nose for a single)
    if (engines == 1) { R(0, -len - 1, ax,ay); radar->drawCircle(ax, ay, (int)(2.5f * sz), eng); }
    else { for (int side = -1; side <= 1; side += 2) { R(side * half * 0.45f, wingRoot + sweep * 0.45f - 1, ax,ay); radar->drawCircle(ax, ay, (int)(2.2f * sz), eng); if (engines == 4) { R(side * half * 0.8f, wingRoot + sweep * 0.8f - 1, bx,by); radar->drawCircle(bx, by, (int)(2.0f * sz), eng); } } }
  } else {
    for (int side = -1; side <= 1; side += 2) { R(side * half * 0.42f, wingRoot + sweep * 0.42f + 1.2f, ax,ay); radar->fillCircle(ax, ay, (int)(1.3f * sz), eng); if (engines == 4) { R(side * half * 0.78f, wingRoot + sweep * 0.78f + 1.2f, bx,by); radar->fillCircle(bx, by, (int)(1.1f * sz), eng); } }
    if (engines == 3) { R(0, len - 2.5f, ax,ay); radar->fillCircle(ax, ay, (int)(1.3f * sz), eng); }
  }
  #undef R
}
void drawClassGlyph(int x, int y, uint8_t cls, uint16_t col) {
  if (cls == CLS_ARRIVAL)        radar->fillTriangle(x, y + 3, x + 4, y - 3, x - 4, y - 3, col);
  else if (cls == CLS_DEPARTURE) radar->fillTriangle(x, y - 3, x + 4, y + 3, x - 4, y + 3, col);
  else                           radar->fillTriangle(x + 3, y, x - 3, y - 4, x - 3, y + 4, col);
}
// coloured two-letter airline chip at (x, baseline); returns the width used (0 if the callsign has no known airline)
uint16_t rgb24(uint32_t c) { return rgb((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF); }
int drawAirlineChip(int x, int yb, const char *callsign) {
  const AirlineRow *a = airlineFor(callsign); if (!a || !a->iata[0]) return 0;
  uint32_t c = a->rgb; int lum = ((c >> 16) & 0xFF) * 3 + ((c >> 8) & 0xFF) * 6 + (c & 0xFF);
  int w = radar->textWidth(FS, a->iata) + 8;
  radar->fillRoundRect(x, yb - 11, w, 14, 3, rgb24(c));
  radar->text(x + 4, yb, a->iata, FS, lum > 1200 ? rgb(0, 0, 0) : rgb(255, 255, 255));
  return w + 4;
}
const char *className(uint8_t cls) { return cls == CLS_ARRIVAL ? "ARR" : cls == CLS_DEPARTURE ? "DEP" : "ENR"; }
void countClasses(int &na, int &nd, int &ne) {
  na = nd = ne = 0;
  for (int i = 0; i < planeCount; i++) { if (!passesFilter(planes[i])) continue; if (planes[i].cls == CLS_ARRIVAL) na++; else if (planes[i].cls == CLS_DEPARTURE) nd++; else ne++; }
}
void drawLegend() {
  int na, nd, ne; countClasses(na, nd, ne); char b[6];
  drawClassGlyph(76, 20, CLS_ARRIVAL, COL_ARR);           snprintf(b, 6, "%d", na); radar->text(83, 26, b, FS, COL_ARR);
  drawClassGlyph(108, 20, CLS_DEPARTURE, COL_DEP);        snprintf(b, 6, "%d", nd); radar->text(115, 26, b, FS, COL_DEP);
  drawClassGlyph(140, 20, CLS_ENROUTE, COL_ENROUTE_HIGH); snprintf(b, 6, "%d", ne); radar->text(147, 26, b, FS, COL_ENROUTE_HIGH);
  char m[40];
  if (metar.valid) { if (metar.wdir >= 0) snprintf(m, sizeof(m), "%03d/%02dkt  RWY %02d", metar.wdir, metar.wspd, activeRwy > 0 ? activeRwy : rwyHdg() / 10); else snprintf(m, sizeof(m), "VRB/%02dkt  RWY %02d", metar.wspd, activeRwy > 0 ? activeRwy : rwyHdg() / 10); }
  else if (activeRwy > 0) snprintf(m, sizeof(m), "RWY %02d in use", activeRwy);
  else m[0] = 0;
  if (holdingCount) { char h[12]; snprintf(h, sizeof(h), "  %d hold", holdingCount); strlcat(m, h, sizeof(m)); }
  if (taf.valid && taf.nextRwy > 0 && !holdingCount) { time_t ct = taf.changeT; struct tm ctm; localtime_r(&ct, &ctm); char h[16]; snprintf(h, sizeof(h), " >%02d @%02d:%02d", taf.nextRwy, ctm.tm_hour, ctm.tm_min); strlcat(m, h, sizeof(m)); }
  if (m[0]) radar->textC(42, m, FSR, COL_TEXT_DIM);
}
void drawHud() {
  unsigned long age = lastFetchMs ? (millis() - lastFetchMs) / 1000 : 999;
  radar->fillCircle(16, 116, 2, (wifiUp && fetchOk) ? COL_OK : COL_BAD);
  char b[12]; if (age < 999) snprintf(b, sizeof(b), "%lus", age); else strcpy(b, "--");
  radar->text(10, 134, b, FSR, COL_TEXT_DIM);
  if (rangeIndex == RANGE_AUTO) snprintf(b, sizeof(b), "A%d%c", (int)(displayRangeKm() / KM_PER_UNIT), RANGE_UNIT[0]); else snprintf(b, sizeof(b), "%d%c", RANGE_PRESETS[rangeIndex], RANGE_UNIT[0]);
  radar->textR(230, 134, b, FSR, currentMode == MODE_SELECT ? COL_SELECTED : COL_TEXT_DIM);
  snprintf(b, sizeof(b), "%dfps", fpsShown); radar->textC(236, b, FSR, rgb(60, 60, 60));
  if (followIcao[0]) radar->textC(60, "FOLLOW", FS, COL_SELECTED);
}
void drawCard(const Plane &p, float dist) {
  int cx = 44, cy = 178, cw = 152, ch = 50;
  radar->fillRoundRect(cx, cy, cw, ch, 8, rgb(10, 10, 10));
  radar->drawRoundRect(cx, cy, cw, ch, 8, COL_RING);
  char pcs[12]; prettyCallsign(p.callsign, pcs, sizeof(pcs));
  int cw2 = drawAirlineChip(cx + 8, cy + 18, p.callsign);
  radar->text(cx + 8 + cw2, cy + 19, cw2 ? (strchr(pcs, ' ') ? strchr(pcs, ' ') + 1 : pcs) : pcs, FMB, COL_SELECTED);
  const char *route = routeFor(p.callsign);
  if (route && route[0]) radar->textR(cx + cw - 8, cy + 17, route, FS, COL_TEXT);
  else if (p.type[0]) radar->textR(cx + cw - 8, cy + 17, p.type, FS, COL_TEXT_DIM);
  int rwy = landingRunway(p); int fl = (int)(p.altitude_m * 3.28084f / 100); char b[40];
  drawClassGlyph(cx + 12, cy + 30, p.cls, planeColour(p));
  if (p.cls == CLS_ARRIVAL)        snprintf(b, sizeof(b), "LAND %02d ~%dm  FL%03d", rwy, etaMin(p, dist), fl);
  else if (p.cls == CLS_DEPARTURE) { const char *r = routeFor(p.callsign); const char *dash = r ? strrchr(r, '-') : NULL; if (destWx.valid && dash && !strcmp(destWx.code, dash + 1)) snprintf(b, sizeof(b), "DEP %02d FL%03d  %s %.0f" DEG " %s", rwy, fl, destWx.code, destWx.temp, wmoShort(destWx.wcode)); else snprintf(b, sizeof(b), "DEP %02d  FL%03d  %+d", rwy, fl, (int)p.vs_fpm); }
  else                             snprintf(b, sizeof(b), "EN ROUTE  FL%03d", fl);
  radar->text(cx + 20, cy + 34, b, FS, planeColour(p));
  snprintf(b, sizeof(b), "%dkt  %.0f%s  look %03d/%d%s", (int)(p.velocity_ms * 1.94384f), dist / KM_PER_UNIT, RANGE_UNIT, (int)p.bearing_deg, elevationDeg(p, dist), DEG);
  radar->text(cx + 8, cy + 46, b, FSR, COL_TEXT_DIM);
}
// Static radar layer (rings, labels, funnels, runway, ticks, landmarks) cached in PSRAM and copied in per frame
// instead of being redrawn; rebuilt when its inputs change or every 15 s. Not cached while following an aircraft.
uint16_t *radarBg = NULL; uint32_t radarBgKey = 0; unsigned long radarBgMs = 0;
void radarBackground() {
  uint16_t *fb = radar->c->getFramebuffer();
  uint32_t key = (uint32_t)(displayRangeKm() * 10) ^ ((uint32_t)homeIndex << 20) ^ ((uint32_t)theme << 26) ^ ((uint32_t)(activeRwy & 63) << 14) ^ (filterMode << 29);
  if (followIcao[0] || !fb) { radar->fillScreen(COL_BG); drawStatic(); return; }
  if (!radarBg) radarBg = (uint16_t *)heap_caps_malloc(240 * 240 * 2, MALLOC_CAP_SPIRAM);
  if (radarBg && key == radarBgKey && millis() - radarBgMs < 15000) { memcpy(fb, radarBg, 240 * 240 * 2); return; }
  radar->fillScreen(COL_BG); drawStatic();
  if (radarBg) { memcpy(radarBg, fb, 240 * 240 * 2); radarBgKey = key; radarBgMs = millis(); }
}
void paintRadar() {
  float rangeKm = displayRangeKm(); double clat, clon; drCenter(clat, clon);
  radarBackground();
  if (sweepOn) drawSweep(sweepAngle);
  if (followIcao[0]) {   // home marker when following
    float d = distanceKm(clat, clon, homeLat, homeLon), b = bearingDeg(clat, clon, homeLat, homeLon);
    if (d < rangeKm) { int hx, hy; polar(b, d / rangeKm * RADAR_MAX_R, hx, hy); radar->drawRect(hx - 4, hy - 4, 9, 9, COL_HOME); drawRunwayAt(hx, hy, 0.7f); }
  }
  bool showCard = (currentMode == MODE_SELECT) && planeCount > 0;
  int selX = -1, selY = -1; float selDist = 0; const Plane *selP = nullptr; int tags = 0;
  int subject = radarSubject();   // v6: ring the aircraft the right dial describes
  for (int i = 0; i < planeCount; i++) {
    const Plane &p = planes[i];
    if (!passesFilter(p)) continue;
    int x, y; float dist, brg;
    planeScreenPos(p, rangeKm, clat, clon, x, y, dist, brg);
    if (dist > rangeKm) continue;
    bool sel = (i == subject);
    uint16_t col = sel ? COL_SELECTED : planeColour(p);
    const Trail *t = trailFor(p.icao24);
    if (t) for (int k = 0; k < t->n; k++) {
      float dk = distanceKm(clat, clon, t->lat[k], t->lon[k]), bk = bearingDeg(clat, clon, t->lat[k], t->lon[k]);
      if (dk > rangeKm) continue;
      int tx, ty; polar(bk, dk / rangeKm * RADAR_MAX_R, tx, ty);
      radar->fillCircle(tx, ty, 1, dim565(col, 0.35f + 0.5f * k / TRAIL_LEN));
    }
    if (p.cls == CLS_ARRIVAL && !followIcao[0]) { int ex, ey; bool onHdgSide = angleDiff(p.bearing_deg, rwyHdg()) < 90; polar(onHdgSide ? rwyHdg() : rwyHdg() + 180, 8, ex, ey); dashedLine(x, y, ex, ey, dim565(col, 0.7f)); }
    else { float h = toRad(p.heading_deg); int ex = x + (int)(26 * sinf(h)), ey = y - (int)(26 * cosf(h)); dashedLine(x, y, ex, ey, dim565(col, 0.6f)); }
    // destination tag for departures (route known): label at the rim in the true direction of the destination
    if (p.cls == CLS_DEPARTURE && tags < 4 && !followIcao[0]) {
      float dlat = 0, dlon = 0; const char *r = routeFor(p.callsign);
      if (r && r[0]) { for (int k = 0; k < MAX_ROUTES; k++) if (routesUI[k].valid && !strcmp(routesUI[k].callsign, p.callsign)) { dlat = routesUI[k].dlat; dlon = routesUI[k].dlon; break; }
        const char *dash = strrchr(r, '-');
        if (dash && dlat != 0) { float db = bearingDeg(homeLat, homeLon, dlat, dlon); int tx, ty; polar(db, 100, tx, ty); radar->text(tx - radar->textWidth(FSR, dash + 1) / 2, ty + 5, dash + 1, FSR, dim565(COL_DEP, 0.8f)); tags++; } }
    }
    drawPlaneIcon(x, y, p.heading_deg, col, sel ? 1.55f : 1.3f, p);
    if (isHolding(p)) radar->drawCircle(x, y, 11, dim565(col, 0.5f));
    if (sel) { selX = x; selY = y; selDist = dist; selP = &p; }
  }
  if (selP) {
    radar->drawCircle(selX, selY, 16, COL_SELECTED); radar->drawCircle(selX, selY, 17, COL_SELECTED);
    (void)showCard; (void)selDist;   // v6: the flight details are on the right dial, not repeated here
  }
  drawLegend();
  drawHud();
}
void drawRadarFrame() {
  staticDirty = false;
  sweepAngle = (millis() % 4000UL) * 0.09f;   // 6.14.1: integer modulo first (float millis lost ms after days of uptime: stutter)   // 6.5.0: 90 deg/s whatever the frame rate (was 3 deg per frame)
  present(paintRadar);
}

// =====================================================================
// ARRIVALS BOARD
// =====================================================================
int boardList(int *idx, int maxN, bool arrivals, const Plane *src, int n, double alat, double alon) {
  // indices of arrivals sorted by ETA (or departures by distance)
  float key[MAX_PLANES]; int m = 0;
  for (int i = 0; i < n && m < MAX_PLANES; i++) {   // collect every match, then sort, then cap: stopping at maxN first made the sort meaningless
    const Plane &p = src[i];
    if (arrivals ? p.cls != CLS_ARRIVAL : p.cls != CLS_DEPARTURE) continue;
    float d = distanceKm(alat, alon, p.lat, p.lon);
    key[m] = arrivals ? (p.velocity_ms > 20 ? d / (p.velocity_ms * 3.6f) * 60 : 999) : d; idx[m++] = i;
  }
  for (int i = 1; i < m; i++) { int k = idx[i]; float kv = key[i]; int j = i - 1; while (j >= 0 && key[j] > kv) { key[j+1] = key[j]; idx[j+1] = idx[j]; j--; } key[j+1] = kv; idx[j+1] = k; }
  return m < maxN ? m : maxN;
}
void paintArrivals() {
  radar->fillScreen(COL_BG);
  bool arr = (subPage == 0); bool other = (boardAirport != homeIndex);
  const Airport &ap = AIRPORTS[boardAirport];
  bool haveArena = other && arenaOwner == ARENA_BOARD;
  const Plane *src = haveArena ? ARENA_BD->items : planes; int n = haveArena ? ARENA_BD->count : planeCount; unsigned long boardMs = haveArena ? ARENA_BD->ms : 1;
  if (other && !haveArena) n = 0;
  radar->textC(34, arr ? "ARRIVALS" : "DEPARTURES", FMB, arr ? COL_ARR : COL_DEP);
  char b[48];
  if (!other) snprintf(b, sizeof(b), "%s  %s", ap.iata, ap.name);   // v6: runway and wind are on the left OLED
  else snprintf(b, sizeof(b), "%s  %s  %s", ap.iata, ap.name, boardMs ? "" : "loading");
  radar->textC(52, b, FSR, COL_TEXT_DIM);
  int idx[MAX_PLANES]; int m = boardList(idx, 12, arr, src, n, ap.lat, ap.lon);
  if (m == 0) { radar->textC(130, n == 0 && other && !boardMs ? "fetching..." : arr ? "no arrivals in range" : "no departures in range", FSR, COL_TEXT_DIM); }
  if (listSel >= m) listSel = m ? m - 1 : 0;
  int first = listSel > 5 ? listSel - 5 : 0;
  struct tm t; bool haveT = getTime(t); int nowMin = haveT ? t.tm_hour * 60 + t.tm_min : 0;
  for (int r = 0; r < 6 && first + r < m; r++) {
    const Plane &p = src[idx[first + r]]; int y = 76 + r * 22; bool hi = (first + r == listSel);
    if (hi) radar->fillRoundRect(18, y - 15, 204, 20, 4, arr ? rgb(32, 20, 0) : rgb(30, 30, 0));
    char pcs[12]; prettyCallsign(p.callsign, pcs, sizeof(pcs));
    int cw2 = drawAirlineChip(22, y, p.callsign);
    radar->text(22 + cw2, y, cw2 ? (strchr(pcs, ' ') ? strchr(pcs, ' ') + 1 : pcs) : pcs, FS, hi ? (arr ? COL_ARR : COL_DEP) : COL_TEXT);
    const char *route = routeFor(p.callsign); char code[6] = "---";
    if (route && route[0]) { if (arr) { strlcpy(code, route, 4); } else { const char *d = strrchr(route, '-'); strlcpy(code, d ? d + 1 : route, 4); } }
    radar->text(94, y, code, FS, COL_ENROUTE_HIGH);
    float d = distanceKm(ap.lat, ap.lon, p.lat, p.lon);
    if (arr) { int eta = etaMin(p, d); if (haveT && eta < 99) { int em = (nowMin + eta) % 1440; snprintf(b, sizeof(b), "%02d:%02d", em / 60, em % 60); } else snprintf(b, sizeof(b), "~%dm", eta); }
    else snprintf(b, sizeof(b), "FL%03d", (int)(p.altitude_m * 3.28084f / 100));
    radar->text(128, y, b, FS, COL_TEXT);
    snprintf(b, sizeof(b), "%.0f km", d); radar->textR(216, y, b, FSR, COL_TEXT_DIM);
  }
}
void drawArrivalsFrame() { if (!staticDirty && millis() - lastHudMs < 1000) return; lastHudMs = millis(); staticDirty = false; present(paintArrivals); }

// =====================================================================
// APPROACH PROFILE
// =====================================================================
int approachList(int *idx, int maxN) {
  int m = 0;
  for (int i = 0; i < planeCount && m < maxN; i++) if ((planes[i].cls == CLS_ARRIVAL || planes[i].cls == CLS_DEPARTURE) && planes[i].distance_km <= 40 && planes[i].altitude_m * 3.28084f < 12000) idx[m++] = i;
  return m;
}
void paintFinal() {   // top-down view of the final approach to the runway in use
  radar->fillScreen(COL_BG); char b[48];
  int rwy = activeRwy > 0 ? activeRwy : rwyHdg() / 10; float landHdg = rwy * 10.0f; float fromDir = fmodf(landHdg + 180, 360);   // aircraft approach from this bearing
  snprintf(b, sizeof(b), "FINAL %02d", rwy); radar->textC(30, b, FS, COL_ARR);
  const int xr = 196, x0 = 30; const float maxKm = 30; const int yc = 124; const float pxPerKmX = (xr - x0) / maxKm, pxPerKmY = 80 / 5.0f;
  radar->drawLine(x0, yc, xr, yc, rgb(0, 90, 40));
  for (int k = 5; k <= 30; k += 5) { int x = xr - (int)(k * pxPerKmX); radar->drawLine(x, yc - 4, x, yc + 4, COL_LINE); if (k % 10 == 0) { snprintf(b, sizeof(b), "%d", k); radar->text(x - radar->textWidth(FSR, b) / 2, yc + 18, b, FSR, rgb(80, 80, 80)); } }
  for (int dy = -80; dy <= 80; dy += 32) if (dy) radar->drawLine(x0, yc + dy, xr, yc + dy, rgb(25, 25, 25));
  radar->fillRect(xr, yc - 2, 14, 5, COL_HOME);
  radar->text(x0, yc - 70, "+5 km", FSR, rgb(70, 70, 70)); radar->text(x0, yc + 86, "-5 km", FSR, rgb(70, 70, 70));
  int idx[MAX_PLANES]; int m = approachList(idx, MAX_PLANES); int shown = 0;
  for (int i = 0; i < m; i++) {
    const Plane &p = planes[idx[i]]; if (p.cls != CLS_ARRIVAL) continue;
    float rel = toRad(p.bearing_deg - fromDir); float along = p.distance_km * cosf(rel), cross = p.distance_km * sinf(rel);
    if (along < 0 || along > maxKm || fabsf(cross) > 5) continue;
    int x = xr - (int)(along * pxPerKmX), y = yc - (int)(cross * pxPerKmY);
    uint16_t col = fabsf(cross) < 0.5f ? COL_OK : COL_ARR;
    radar->fillCircle(x, y, 4, col); if (i == listSel) { radar->drawCircle(x, y, 8, COL_SELECTED); }
    float h = toRad(p.heading_deg - landHdg); radar->drawLine(x, y, x + (int)(10 * sinf(h) * 1.0f), y - (int)(10 * cosf(h)), col);   // heading tick relative to the runway
    char pcs[12]; prettyCallsign(p.callsign, pcs, sizeof(pcs)); char *sp = strchr(pcs, ' '); if (sp) memmove(sp, sp + 1, strlen(sp));
    radar->text(x - radar->textWidth(FSR, pcs) / 2, y - 8, pcs, FSR, col); shown++;
  }
  if (!shown) radar->textC(100, "nobody on final", FSR, COL_TEXT_DIM);
  if (metar.valid) snprintf(b, sizeof(b), "wind %03d/%02d  %s", metar.wdir < 0 ? 0 : metar.wdir, metar.wspd, metar.fltCat); else strcpy(b, "green = on centreline");
  radar->textC(222, b, FSR, COL_TEXT_DIM);
}
void paintApproach() {
  if (subPage == 1) { paintFinal(); return; }
  radar->fillScreen(COL_BG);
  char b[48]; snprintf(b, sizeof(b), "APPROACH %02d", activeRwy > 0 ? activeRwy : rwyHdg() / 10);
  radar->textC(30, b, FS, COL_ARR);
  const int x0 = 28, x1 = 214, y0 = 48, y1 = 176; const float maxKm = 40, maxFt = 12000; float elevFt = homeElevM() * 3.28084f;
  radar->drawLine(x0, y1, x1, y1, COL_LINE); radar->drawLine(x0, y0, x0, y1, COL_LINE);
  for (int k = 0; k <= 40; k += 10) { int x = x0 + k * (x1 - x0) / 40; radar->drawLine(x, y1, x, y1 + 3, COL_TEXT_DIM); snprintf(b, sizeof(b), "%d", k); radar->text(x - radar->textWidth(FSR, b) / 2, y1 + 15, b, FSR, rgb(80, 80, 80)); }
  for (int ft = 2000; ft < 12000; ft += 2000) { int y = y1 - (int)(ft * (y1 - y0) / maxFt); radar->drawLine(x0, y, x0 + 3, y, COL_TEXT_DIM); snprintf(b, sizeof(b), "%d", ft / 1000); radar->textR(x0 - 3, y + 4, b, FSR, rgb(80, 80, 80)); }
  // 3 degree glideslope from the field elevation, 172 ft per km
  int px = -1, py = -1;
  for (int k = 0; k <= 40; k += 2) { float ft = elevFt + k * 172; int x = x0 + k * (x1 - x0) / 40, y = y1 - (int)(ft * (y1 - y0) / maxFt); if (px >= 0 && (k / 2) % 2 == 0) radar->drawLine(px, py, x, y, rgb(0, 110, 50)); px = x; py = y; }
  int ry = y1 - (int)(elevFt * (y1 - y0) / maxFt); radar->fillRect(x0, ry - 3, 10, 4, COL_HOME);
  int idx[MAX_PLANES]; int m = approachList(idx, MAX_PLANES);
  if (listSel >= m) listSel = m ? m - 1 : 0;
  const Plane *sel = NULL; float selDev = 0;
  for (int i = 0; i < m; i++) {
    const Plane &p = planes[idx[i]]; float ft = p.altitude_m * 3.28084f;
    int x = x0 + (int)(p.distance_km * (x1 - x0) / maxKm), y = y1 - (int)(ft * (y1 - y0) / maxFt);
    uint16_t col = planeColour(p);
    radar->fillCircle(x, y, 4, col);
    if (i == listSel) { radar->drawCircle(x, y, 8, COL_SELECTED); radar->drawCircle(x, y, 9, COL_SELECTED); sel = &p; selDev = ft - (elevFt + p.distance_km * 172); }
    char pcs[12]; prettyCallsign(p.callsign, pcs, sizeof(pcs)); char *sp = strchr(pcs, ' '); if (sp) memmove(sp, sp + 1, strlen(sp));   // "6E2134"
    int lx = x + 8; if (lx + radar->textWidth(FSR, pcs) > x1) lx = x - 8 - radar->textWidth(FSR, pcs);
    radar->text(lx, y - 6, pcs, FSR, col);
  }
  if (sel) {
    char pcs[12]; prettyCallsign(sel->callsign, pcs, sizeof(pcs));
    if (sel->cls == CLS_DEPARTURE) snprintf(b, sizeof(b), "%s  climbing %+d fpm", pcs, (int)sel->vs_fpm);
    else if (fabsf(selDev) < 150) snprintf(b, sizeof(b), "%s  on slope  %+.0f ft", pcs, selDev);
    else snprintf(b, sizeof(b), "%s  %.0f ft %s", pcs, fabsf(selDev), selDev > 0 ? "high" : "low");
    radar->textC(206, b, FS, sel->cls == CLS_DEPARTURE ? COL_DEP : fabsf(selDev) < 150 ? COL_OK : COL_ARR);
  } else radar->textC(206, m ? "" : "no traffic on approach", FS, COL_TEXT_DIM);
  if (sigmet[0].valid) { snprintf(b, sizeof(b), "SIGMET %s", sigmet[0].hazard); radar->textC(224, b, FSR, COL_ARR); }
  // v6: METAR / TAF decoded on the left OLED
}
void drawApproachFrame() { if (!staticDirty && millis() - lastHudMs < 1000) return; lastHudMs = millis(); staticDirty = false; present(paintApproach); }

// =====================================================================
// REPLAY: last 48 minutes of tracks from the aviation arena, knob scrubs time
// =====================================================================
int replayMaxAge() { if (arenaOwner != ARENA_AVIATION) return 0; int m = ARENA_AV->minutes; return m > REPLAY_LEN ? REPLAY_LEN - 1 : (m > 0 ? m - 1 : 0); }
void paintReplay() {
  radar->fillScreen(COL_BG); char b[40];
  for (int i = 1; i <= 3; i++) radar->drawCircle(RADAR_CX, RADAR_CY, (RADAR_MAX_R * i) / 3, i == 3 ? COL_RING : COL_RING_DIM);
  drawRunwayAt(RADAR_CX, RADAR_CY, 1);
  if (arenaOwner != ARENA_AVIATION) { radar->textC(124, "replay needs the radar group", FSR, COL_TEXT_DIM); return; }
  ArenaAviation *a = ARENA_AV; float rk = displayRangeKm(); float pxPerUnit = RADAR_MAX_R / (rk * 100.0f);   // one unit = 10 m
  int maxAge = replayMaxAge(); if (replayPos > maxAge) replayPos = maxAge; int cur = replayPos; int shown = 0;
  for (int k = 0; k < REPLAY_TRACKS; k++) {
    ReplayTrack &t = a->tr[k]; if (!t.icao[0] || t.n == 0) continue;
    int gap = a->minutes - t.lastMin;   // minutes since this aircraft was last seen
    if (gap >= REPLAY_LEN) continue;     // everything it has is older than the window
    int px = -1, py = -1; bool headDrawn = false;
    for (int s = 0; s < t.n; s++) {
      int age = gap + (t.n - 1 - s); if (age < cur) break;
      const ReplayPt &pt = t.pts[(t.head - t.n + s + REPLAY_LEN) % REPLAY_LEN];
      int x = RADAR_CX + (int)(pt.dlon * pxPerUnit), y = RADAR_CY - (int)(pt.dlat * pxPerUnit);
      int dx2 = x - RADAR_CX, dy2 = y - RADAR_CY; bool in = dx2 * dx2 + dy2 * dy2 <= RADAR_MAX_R * RADAR_MAX_R;
      float fade = 1.0f - 0.7f * (age - cur) / (float)(REPLAY_LEN); uint16_t col = dim565(COL_ENROUTE_LOW, fade < 0.25f ? 0.25f : fade);
      if (in && px >= 0) radar->drawLine(px, py, x, y, col);
      if (in && age == cur) { radar->fillCircle(x, y, 3, COL_SWEEP); headDrawn = true; shown++; if (shown <= 6) radar->text(x + 5, y + 4, t.icao, FSR, COL_TEXT_DIM); }
      px = in ? x : -1; py = y;
    }
    (void)headDrawn;
  }
  time_t now = time(NULL) - (time_t)cur * 60; struct tm tt; localtime_r(&now, &tt);
  snprintf(b, sizeof(b), "REPLAY  %02d:%02d", tt.tm_hour, tt.tm_min); radar->textC(24, b, FS, COL_TEXT_DIM);
  if (cur == 0) strcpy(b, "now"); else snprintf(b, sizeof(b), "-%d min", cur); radar->textC(40, b, FMB, COL_TEXT);
  // time bar: oldest on the left, now on the right
  int x0 = 50, x1 = 190, yb = 212; radar->drawLine(x0, yb, x1, yb, rgb(50, 50, 50));
  if (maxAge > 0) { int x = x1 - cur * (x1 - x0) / maxAge; radar->fillCircle(x, yb, 4, replayPlay ? COL_OK : COL_ARR); }
  snprintf(b, sizeof(b), "%d ac  %d min", shown, a->minutes > REPLAY_LEN ? REPLAY_LEN : a->minutes); radar->textC(228, b, FSR, rgb(70, 70, 70));
  if (a->minutes < 2) radar->textC(124, "recording, come back in a few minutes", FSR, COL_TEXT_DIM);
}
void drawReplayFrame() {
  if (replayPlay && millis() - replayLastMs > 400) { replayLastMs = millis(); int m = replayMaxAge(); replayPos = replayPos > 0 ? replayPos - 1 : m; staticDirty = true; }
  if (!staticDirty && millis() - lastHudMs < 2000) return; lastHudMs = millis(); staticDirty = false; present(paintReplay);
}

// =====================================================================
// F1: next race countdown and last podium (jolpi.ca)
// =====================================================================
void paintF1() {
  radar->fillScreen(COL_BG); char b[48];
  radar->textC(30, "FORMULA 1", FS, COL_TEXT_DIM);
  if (!f1.valid) { radar->textC(124, "waiting for race data", FSR, COL_TEXT_DIM); return; }
  { char race[30]; strlcpy(race, f1.race, sizeof(race)); char *gp = strstr(race, " Grand Prix"); if (gp) *gp = 0;   // "Singapore" + GRAND PRIX below
    radar->textC(56, race, radar->textWidth(FMB, race) > 168 ? FS : FMB, COL_TEXT);
    radar->textC(72, gp ? "GRAND PRIX" : "", FSR, COL_TEXT_DIM); }
  time_t now = time(NULL); long dt = (long)f1.t - (long)now;
  uint16_t cc = dt > 0 && dt < 86400 ? COL_ARR : (f1.t && dt <= 0 && dt > -3 * 3600 ? COL_SELECTED : COL_TEXT);
  if (f1.t == 0) radar->textC(124, "date to be announced", FM, COL_TEXT_DIM);
  else if (dt > 0 && dt >= 86400) { snprintf(b, sizeof(b), "%ld", dt / 86400); radar->textC(128, b, FB2, cc); snprintf(b, sizeof(b), "days  %ld h", (dt % 86400) / 3600); radar->textC(146, b, FS, COL_TEXT_DIM); }
  else if (dt > 0) { snprintf(b, sizeof(b), "%ld:%02ld", dt / 3600, (dt % 3600) / 60); radar->textC(128, b, FB2, cc); radar->textC(146, "hours to lights out", FS, COL_TEXT_DIM); }
  else if (dt > -3 * 3600) radar->textC(128, "LIVE", FMB, cc);
  else radar->textC(128, "finished", FM, COL_TEXT_DIM);
  // v6: race date and time are on the left OLED, the podium on the right dial
}
void drawF1Frame() { if (!staticDirty && millis() - lastHudMs < 5000) return; lastHudMs = millis(); staticDirty = false; present(paintF1); }

// =====================================================================
// CLOCK: 0 Bold, 1 Analog, 2 Dashboard, 3 World
// =====================================================================
int clkS = -1, clkFaceDrawn = -1; struct tm clkT; bool clkOk = false;
float clkSub = 0;   // 6.15.0: fraction of the current second, set once per frame (smooth hands on Analog and Rings)
// Rounded bar from radius r0 to r1 along deg (0 = 12 o'clock, clockwise); r0 may be negative for a tail
void handBaton(float deg, float r0, float r1, float wid, uint16_t col) {
  float a = toRad(deg); float ux = sinf(a), uy = -cosf(a), px = -uy * wid * 0.5f, py = ux * wid * 0.5f;
  int x0 = RADAR_CX + (int)lroundf(ux * r0), y0 = RADAR_CY + (int)lroundf(uy * r0), x1 = RADAR_CX + (int)lroundf(ux * r1), y1 = RADAR_CY + (int)lroundf(uy * r1);
  int ax = (int)lroundf(px), ay = (int)lroundf(py);
  radar->fillTriangle(x0 - ax, y0 - ay, x0 + ax, y0 + ay, x1 + ax, y1 + ay, col);
  radar->fillTriangle(x0 - ax, y0 - ay, x1 + ax, y1 + ay, x1 - ax, y1 - ay, col);
  int cr = (int)(wid * 0.5f); if (cr >= 1) { radar->fillCircle(x0, y0, cr, col); radar->fillCircle(x1, y1, cr, col); }
}
// Progress ring with a rounded end: track, then the arc 0..f (f 0..1) in col
void ringProgress(int r1, int r2, float f, uint16_t col, uint16_t track) {
  radar->ringArc(RADAR_CX, RADAR_CY, r1, r2, 0, 359.9f, track);
  if (f <= 0) return; if (f > 1) f = 1;
  float a1 = f * 360.0f; int rm = (r1 + r2) / 2, cr = (r1 - r2) / 2;
  radar->ringArc(RADAR_CX, RADAR_CY, r1, r2, 0, a1, col);
  int x, y; polar(0, rm, x, y); radar->fillCircle(x, y, cr, col); polar(a1, rm, x, y); radar->fillCircle(x, y, cr, col);
}
// Seven-segment digit in a w x h box (d 0..9, anything else blank: only the off segments)
void seg7(int x, int y, int w, int h, int d, uint16_t on, uint16_t off) {
  static const uint8_t M[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};   // bits: a b c d e f g
  uint8_t m = (d >= 0 && d <= 9) ? M[d] : 0; int t = w / 5; if (t < 3) t = 3; int hh = (h - t) / 2, r = t / 2;
  radar->fillRoundRect(x + t, y, w - 2 * t, t, r, (m & 1) ? on : off);                 // a top
  radar->fillRoundRect(x + w - t, y + t, t, hh - t, r, (m & 2) ? on : off);            // b upper right
  radar->fillRoundRect(x + w - t, y + hh + t, t, hh - t, r, (m & 4) ? on : off);       // c lower right
  radar->fillRoundRect(x + t, y + h - t, w - 2 * t, t, r, (m & 8) ? on : off);         // d bottom
  radar->fillRoundRect(x, y + hh + t, t, hh - t, r, (m & 16) ? on : off);              // e lower left
  radar->fillRoundRect(x, y + t, t, hh - t, r, (m & 32) ? on : off);                   // f upper left
  radar->fillRoundRect(x + t, y + hh, w - 2 * t, t, r, (m & 64) ? on : off);           // g middle
}
void handPoly(float deg, float len, float wid, float tail, uint16_t col) {
  float a = toRad(deg); int tx, ty, bx, by; polar(deg, len, tx, ty); polar(deg + 180, tail, bx, by);
  int px = (int)lroundf(cosf(a) * wid), py = (int)lroundf(sinf(a) * wid);
  radar->fillTriangle(bx - px, by - py, bx + px, by + py, tx, ty, col);
  radar->fillTriangle(bx - px, by - py, tx - px/2, ty - py/2, tx + px/2, ty + py/2, col);
  radar->fillTriangle(bx + px, by + py, tx + px/2, ty + py/2, tx - px/2, ty - py/2, col);
}
void drawDayRing(int r, int w, bool withRain) {   // 24 h ring, noon at the top
  radar->ringArc(RADAR_CX, RADAR_CY, r, r - w, 0, 359.9f, rgb(26, 26, 44));
  if (wx.sunriseMin >= 0 && wx.sunsetMin > wx.sunriseMin) radar->ringArc(RADAR_CX, RADAR_CY, r, r - w, wx.sunriseMin / 4.0f + 180, wx.sunsetMin / 4.0f + 180, rgb(255, 170, 60));
  if (withRain && wx.valid && clkOk) {
    for (int i = 0; i < 24; i++) if (wx.rainProb[i] >= 40) {
      float a0 = ((clkT.tm_hour + i) % 24) * 15.0f + 180, a1 = a0 + 14.5f;
      radar->ringArc(RADAR_CX, RADAR_CY, r - w - 2, r - w - 5, a0, a1, wx.rainProb[i] >= 70 ? rgb(60, 140, 255) : rgb(40, 90, 170));
    }
  }
  if (clkOk) { int sx, sy; polar((clkT.tm_hour * 60 + clkT.tm_min) / 4.0f + 180, r - w / 2, sx, sy); radar->fillCircle(sx, sy, 4, rgb(255, 235, 180)); }
}
void nextCountdown(char *out, size_t n) {
  out[0] = 0; if (!clkOk) return;
  time_t now = time(NULL); long best = -1; const CountdownR *bc = NULL;
  for (int i = 0; i < cdownCount; i++) {
    struct tm e = {}; e.tm_year = cdowns[i].y - 1900; e.tm_mon = cdowns[i].m - 1; e.tm_mday = cdowns[i].d; e.tm_hour = cdowns[i].hh; e.tm_min = cdowns[i].mm; e.tm_isdst = -1;
    long dt = (long)(mktime(&e) - now); if (dt > 0 && (best < 0 || dt < best)) { best = dt; bc = &cdowns[i]; }
  }
  if (!bc) return;
  if (best >= 2 * 86400) snprintf(out, n, "%s in %ld days", bc->name, best / 86400);
  else if (best >= 3600) snprintf(out, n, "%s in %ld h %ld m", bc->name, best / 3600, (best % 3600) / 60);
  else snprintf(out, n, "%s in %ld min", bc->name, best / 60);
}
bool holidayForYday(int year, int yday, int &m, int &d) {
  struct tm e = {}; e.tm_year = year - 1900; e.tm_mon = 0; e.tm_mday = 1 + yday; e.tm_hour = 12; e.tm_isdst = -1; mktime(&e);
  m = e.tm_mon + 1; d = e.tm_mday;
  for (int i = 0; i < NUM_HOLIDAYS; i++) if ((HOLIDAYS[i].y == 0 || HOLIDAYS[i].y == year) && HOLIDAYS[i].m == m && HOLIDAYS[i].d == d) return true;
  return false;
}
void nextHoliday(char *out, size_t n) {   // "Diwali in 12 days" / "Diwali today"
  out[0] = 0; if (!clkOk) return;
  int year = clkT.tm_year + 1900; long best = -1; const char *name = NULL;
  for (int i = 0; i < NUM_HOLIDAYS; i++) {
    for (int yy = year; yy <= year + 1; yy++) {
      if (HOLIDAYS[i].y != 0 && HOLIDAYS[i].y != yy) continue;
      struct tm e = {}; e.tm_year = yy - 1900; e.tm_mon = HOLIDAYS[i].m - 1; e.tm_mday = HOLIDAYS[i].d; e.tm_hour = 12; e.tm_isdst = -1;
      struct tm today = clkT; today.tm_hour = 12; today.tm_min = 0; today.tm_sec = 0; today.tm_isdst = -1;
      long diff = (long)(mktime(&e) - mktime(&today)); if (diff < -43200) continue;   // yesterday is not "today"
      long dd = (diff + 43200) / 86400;
      if (best < 0 || dd < best) { best = dd; name = HOLIDAYS[i].name; }
    }
  }
  if (!name) return;
  if (best == 0) snprintf(out, n, "%s today", name); else if (best == 1) snprintf(out, n, "%s tomorrow", name); else snprintf(out, n, "%s in %ldd", name, best);
}
void paintClock() {
  struct tm &t = clkT; char b[48];
  radar->fillScreen(COL_BG);
  if (!clkOk) { radar->textC(124, "waiting for NTP", FSR, COL_TEXT); return; }
  if (clockFace == 0) {
    drawDayRing(117, 5, true);
    snprintf(b, sizeof(b), "%s %02d %s", DN[t.tm_wday], t.tm_mday, MN[t.tm_mon]); radar->textC(64, b, FM, COL_TEXT_DIM);
    fmtHM(b, sizeof(b), t.tm_hour, t.tm_min); radar->textC(136, b, FB, COL_TEXT);
    if (clock12) radar->text(RADAR_CX + radar->textWidth(FB, b) / 2 + 4, 136, ampmStr(t.tm_hour), FS, COL_TEXT_DIM);
    if (wx.valid) { snprintf(b, sizeof(b), "%.0f" DEG "  %s", wx.temp, wmoText(wx.wcode)); radar->textC(170, b, FM, rgb(190, 190, 190)); }
    if (wx.valid) {
      int nowMin = t.tm_hour * 60 + t.tm_min;
      if (wx.sunriseMin >= 0 && nowMin < wx.sunriseMin) snprintf(b, sizeof(b), "AQI %d   sunrise %02d:%02d", wx.aqi, wx.sunriseMin / 60, wx.sunriseMin % 60);
      else if (wx.sunsetMin >= 0 && nowMin < wx.sunsetMin) snprintf(b, sizeof(b), "AQI %d   sunset %02d:%02d", wx.aqi, wx.sunsetMin / 60, wx.sunsetMin % 60);
      else snprintf(b, sizeof(b), "AQI %d   %s", wx.aqi, aqiText(wx.aqi));
      radar->textC(196, b, FS, COL_ARR);
    }
    if (alarmOn) { snprintf(b, sizeof(b), "alarm %02d:%02d", alarmMin / 60, alarmMin % 60); radar->textC(214, b, FSR, COL_TEXT_DIM); }
  } else if (clockFace == 1) {   // 6.15.0: bezel, batons, numerals, outlined hands, smooth seconds
    uint16_t bezel = rgb(36, 36, 44), bezel2 = rgb(16, 16, 20), tick = rgb(84, 84, 96), baton = rgb(215, 215, 225);
    radar->ringArc(RADAR_CX, RADAR_CY, 120, 115, 0, 359.9f, bezel); radar->ringArc(RADAR_CX, RADAR_CY, 115, 113, 0, 359.9f, bezel2);
    for (int m = 0; m < 60; m++) {
      if (m % 5) { int x0, y0, x1, y1; polar(m * 6, 109, x0, y0); polar(m * 6, 105, x1, y1); radar->drawLine(x0, y0, x1, y1, tick); }
      else if (m % 15) handBaton(m * 6, 97, 109, 4, baton);
    }
    radar->textC(RADAR_CY - 68, "12", FMB, COL_TEXT); radar->textC(RADAR_CY + 92, "6", FMB, COL_TEXT);
    radar->textR(RADAR_CX + 92, RADAR_CY + 8, "3", FMB, COL_TEXT); radar->text(RADAR_CX - 92, RADAR_CY + 8, "9", FMB, COL_TEXT);
    if (wx.valid) { snprintf(b, sizeof(b), "%.0f" DEG, wx.temp); radar->textC(RADAR_CY + 72, b, FS, rgb(140, 140, 150)); }   // below the date window, clear of the hands
    radar->drawRoundRect(RADAR_CX - 27, RADAR_CY + 30, 54, 22, 4, rgb(70, 70, 80));
    snprintf(b, sizeof(b), "%s %02d", DN[t.tm_wday], t.tm_mday); radar->textC(RADAR_CY + 46, b, FS, COL_TEXT_DIM);
    float sec = t.tm_sec + clkSub, mn = t.tm_min + sec / 60.0f, hr = (t.tm_hour % 12) + mn / 60.0f;
    handBaton(hr * 30, -10, 56, 10, COL_BG); handBaton(hr * 30, -8, 54, 7, baton);
    handBaton(mn * 6, -12, 86, 8, COL_BG);  handBaton(mn * 6, -10, 84, 5, COL_TEXT);
    handBaton(sec * 6, -22, 98, 2, COL_SELECTED); { int x, y; polar(sec * 6 + 180, 18, x, y); radar->fillCircle(x, y, 4, COL_SELECTED); }
    radar->fillCircle(RADAR_CX, RADAR_CY, 6, COL_TEXT); radar->fillCircle(RADAR_CX, RADAR_CY, 2, COL_SELECTED);
  } else if (clockFace == 2) {
    fmtHM(b, sizeof(b), t.tm_hour, t.tm_min); radar->textC(58, b, FB2, COL_TEXT);
    snprintf(b, sizeof(b), "%s %02d %s", DN[t.tm_wday], t.tm_mday, MN[t.tm_mon]); radar->textC(76, b, FS, COL_TEXT_DIM);
    struct { const char *lab; char val[14]; uint16_t col; } tiles[4];
    tiles[0].lab = "WEATHER"; if (wx.valid) snprintf(tiles[0].val, 14, "%.0f" DEG " %s", wx.temp, wmoShort(wx.wcode)); else strcpy(tiles[0].val, "..."); tiles[0].col = rgb(190, 190, 190);
    tiles[1].lab = "AQI"; if (wx.valid) snprintf(tiles[1].val, 14, "%d %s", wx.aqi, aqiText(wx.aqi)); else strcpy(tiles[1].val, "..."); tiles[1].col = wx.valid ? aqiColour(wx.aqi) : COL_TEXT_DIM;
    tiles[2].lab = "NEXT ARRIVAL";
    { // present() paints 3 bands per frame: do the scan and sort on the first band only, then reuse it (also keeps the 3 bands identical)
      static char nextArr[14] = "none";
      if (radar->dy == 0) {
        int idx[MAX_PLANES]; int m = boardList(idx, 1, true, planes, planeCount, homeLat, homeLon);
        if (m) { char pcs[12]; prettyCallsign(planes[idx[0]].callsign, pcs, sizeof(pcs)); char *sp = strchr(pcs, ' '); if (sp) memmove(sp, sp + 1, strlen(sp)); snprintf(nextArr, sizeof(nextArr), "%s ~%dm", pcs, etaMin(planes[idx[0]], planes[idx[0]].distance_km)); }
        else strlcpy(nextArr, "none", sizeof(nextArr));
      }
      strlcpy(tiles[2].val, nextArr, 14);
    }
    tiles[2].col = COL_ARR;
    tiles[3].lab = MARKET_NAMES[0]; if (quotes[0].valid) { float c = quotes[0].prevClose > 0 ? (quotes[0].price - quotes[0].prevClose) / quotes[0].prevClose * 100 : 0; snprintf(tiles[3].val, 14, "%+.2f%%", c); tiles[3].col = c >= 0 ? COL_OK : COL_SELECTED; } else { strcpy(tiles[3].val, "..."); tiles[3].col = COL_TEXT_DIM; }
    for (int i = 0; i < 4; i++) {
      int x = 28 + (i % 2) * 95, y = 90 + (i / 2) * 48;
      radar->drawRoundRect(x, y, 89, 42, 6, COL_PANEL);
      radar->text(x + 6, y + 15, tiles[i].lab, FSR, COL_TEXT_DIM);
      radar->text(x + 6, y + 33, tiles[i].val, FS, tiles[i].col);
    }
    nextCountdown(b, sizeof(b)); if (b[0]) radar->textC(206, b, FS, COL_ENROUTE_HIGH);
    if (pomoState == POMO_FOCUS || pomoState == POMO_BREAK) { unsigned long rem = pomoRemainMs(); snprintf(b, sizeof(b), "%s %02lu:%02lu", pomoState == POMO_FOCUS ? "focus" : "break", rem / 60000, (rem / 1000) % 60); }
    else snprintf(b, sizeof(b), "focus %d min today", pomoTodayMin);
    radar->textC(224, b, FSR, rgb(80, 80, 80));
  } else if (clockFace == 3) {
    radar->textC(46, LOCAL_CITY, FS, COL_TEXT_DIM);
    fmtHM(b, sizeof(b), t.tm_hour, t.tm_min); radar->textC(104, b, FB, COL_TEXT);
    radar->drawLine(50, 122, 190, 122, COL_PANEL);
    zoneTick();
    for (int i = 0; i < NUM_ZONES && i < 2; i++) {
      struct tm z = {}; z.tm_hour = zoneT[i].hour; z.tm_min = zoneT[i].min; z.tm_wday = zoneT[i].wday;
      int y = 150 + i * 44; bool day = z.tm_hour >= 6 && z.tm_hour < 18;
      radar->text(56, y, ZONES[i].name, FSR, COL_TEXT_DIM);
      snprintf(b, sizeof(b), "%02d:%02d", z.tm_hour, z.tm_min); radar->text(56, y + 24, b, FMB, day ? COL_TEXT : rgb(150, 150, 170));
      radar->text(132, y + 24, DNL[z.tm_wday], FSR, rgb(90, 90, 90));
      int cx = 180, cy = y + 14;
      if (day) radar->fillCircle(cx, cy, 7, COL_DEP); else { radar->fillCircle(cx, cy, 7, rgb(150, 150, 170)); radar->fillCircle(cx + 4, cy - 3, 6, COL_BG); }
    }
  } else if (clockFace == 4) {   // 24-hour dial, noon at the top
    drawDayRing(118, 8, true);
    for (int h = 0; h < 24; h++) { int x0,y0,x1,y1; polar(h * 15 + 180, 104, x0,y0); polar(h * 15 + 180, 104 - (h % 6 ? 4 : 9), x1,y1); radar->drawLine(x0,y0,x1,y1, h % 6 ? rgb(70, 70, 70) : COL_TEXT); }
    radar->textC(42, "12", FS, COL_TEXT); radar->textC(206, "0", FS, COL_TEXT); radar->text(30, 125, "6", FS, COL_TEXT); radar->textR(210, 125, "18", FS, COL_TEXT);
    if (wx.sunriseMin >= 0) { snprintf(b, sizeof(b), "%02d:%02d", wx.sunriseMin / 60, wx.sunriseMin % 60); radar->text(36, 148, b, FSR, COL_ARR); snprintf(b, sizeof(b), "%02d:%02d", wx.sunsetMin / 60, wx.sunsetMin % 60); radar->textR(204, 148, b, FSR, COL_ARR); }
    handPoly((t.tm_hour * 60 + t.tm_min) / 4.0f + 180, 84, 3.5f, 14, COL_TEXT);
    radar->fillCircle(RADAR_CX, RADAR_CY, 5, COL_TEXT);
    fmtHM(b, sizeof(b), t.tm_hour, t.tm_min); radar->textC(96, b, FM, COL_TEXT_DIM);
    snprintf(b, sizeof(b), "%s %02d %s", DN[t.tm_wday], t.tm_mday, MN[t.tm_mon]); radar->textC(176, b, FSR, COL_TEXT_DIM);
  } else if (clockFace == 5) {   // minimal (6.15.0): big time, seconds as lit ticks around the rim
    for (int i = 0; i < 60; i++) {
      int x0, y0, x1, y1; bool lit = i <= t.tm_sec; polar(i * 6, 117, x0, y0); polar(i * 6, lit ? 109 : 113, x1, y1);
      radar->drawLine(x0, y0, x1, y1, i == t.tm_sec ? COL_TEXT : lit ? COL_SWEEP : rgb(44, 44, 50));
    }
    fmtHM(b, sizeof(b), t.tm_hour, t.tm_min); radar->textC(138, b, FB, COL_TEXT);
    if (clock12) radar->text(RADAR_CX + radar->textWidth(FB, b) / 2 + 4, 138, ampmStr(t.tm_hour), FS, COL_TEXT_DIM);
    snprintf(b, sizeof(b), "%s %02d %s", DNL[t.tm_wday], t.tm_mday, MN[t.tm_mon]); radar->textC(170, b, FSR, rgb(110, 110, 120));
  } else if (clockFace == 6) {   // flip clock (6.15.0): two-tone cards with a shadow, hinges and a split line
    uint16_t cardTop = rgb(46, 46, 50), cardBot = rgb(30, 30, 34), edge = rgb(78, 78, 86), shadow = rgb(6, 6, 8);
    for (int i = 0; i < 2; i++) {
      int x = 30 + i * 96;
      radar->fillRoundRect(x + 3, 73, 84, 96, 8, shadow);
      radar->fillRoundRect(x, 70, 84, 56, 8, cardTop); radar->fillRect(x, 100, 84, 18, cardTop);
      radar->fillRoundRect(x, 110, 84, 56, 8, cardBot); radar->fillRect(x, 118, 84, 10, cardBot);
      snprintf(b, sizeof(b), "%02d", i == 0 ? (clock12 ? hour12(t.tm_hour) : t.tm_hour) : t.tm_min);
      radar->text(x + 42 - radar->textWidth(FB, b) / 2, 140, b, FB, COL_TEXT);
      radar->fillRect(x, 117, 84, 2, shadow);
      radar->drawRoundRect(x, 70, 84, 96, 8, edge);
      radar->fillRoundRect(x - 3, 112, 6, 12, 2, edge); radar->fillRoundRect(x + 81, 112, 6, 12, 2, edge);
    }
    radar->fillCircle(RADAR_CX, 104, 3, t.tm_sec % 2 ? COL_TEXT : edge); radar->fillCircle(RADAR_CX, 132, 3, t.tm_sec % 2 ? COL_TEXT : edge);
    radar->fillRoundRect(72, 180, 96, 22, 4, cardBot); radar->drawRoundRect(72, 180, 96, 22, 4, edge);
    snprintf(b, sizeof(b), "%s  %02d %s", DN[t.tm_wday], t.tm_mday, MN[t.tm_mon]); radar->textC(196, b, FS, COL_TEXT_DIM);
    if (clock12) radar->textC(214, ampmStr(t.tm_hour), FSR, rgb(110, 110, 120));
    if (wx.valid) { snprintf(b, sizeof(b), "%.0f" DEG "  %s", wx.temp, wmoShort(wx.wcode)); radar->textC(52, b, FSR, COL_TEXT_DIM); }
  } else if (clockFace == 7) {   // big seconds (6.15.0): 60 segments light up, the last few brighter
    uint16_t dimc = rgb(18, 26, 34), midc = rgb(40, 100, 160), hic = rgb(80, 200, 255);
    for (int i = 0; i < 60; i++) {
      uint16_t c = i > t.tm_sec ? dimc : (t.tm_sec - i < 4 ? hic : midc);
      radar->ringArc(RADAR_CX, RADAR_CY, 118, 110, i * 6 + 0.8f, i * 6 + 5.2f, c);
    }
    fmtHM(b, sizeof(b), t.tm_hour, t.tm_min); radar->textC(80, b, FB2, COL_TEXT_DIM);
    if (clock12) radar->text(RADAR_CX + radar->textWidth(FB2, b) / 2 + 4, 80, ampmStr(t.tm_hour), FSR, COL_TEXT_DIM);
    snprintf(b, sizeof(b), "%02d", t.tm_sec); radar->textC(160, b, FB, COL_TEXT);
    snprintf(b, sizeof(b), "%s %02d %s", DN[t.tm_wday], t.tm_mday, MN[t.tm_mon]); radar->textC(192, b, FSR, COL_TEXT_DIM);
  } else if (clockFace == 8) {   // night sky
    radar->fillScreen(rgb(4, 6, 20));
    uint32_t seed = 12345; for (int i = 0; i < 70; i++) { seed = seed * 1103515245u + 12345u; int sx = (seed >> 8) % 240; seed = seed * 1103515245u + 12345u; int sy = (seed >> 8) % 240; int dx2 = sx - 120, dy2 = sy - 120; if (dx2*dx2 + dy2*dy2 < 112*112) radar->drawPixel(sx, sy, (i % 5) ? rgb(120, 125, 150) : rgb(220, 220, 235)); }
    fmtHM(b, sizeof(b), t.tm_hour, t.tm_min); radar->textC(62, b, FB2, COL_TEXT);
    float ph = moonPhase(); int mx = RADAR_CX, my = 118;
    radar->fillCircle(mx, my, 26, rgb(215, 215, 225));
    // shadow: offset disc sized by the phase (waxing: shadow on the left, waning: on the right)
    float k = ph < 0.5f ? (0.5f - ph) * 2 : (ph - 0.5f) * 2;   // 1 at new, 0 at full
    int off = (int)(52 * k); radar->fillCircle(mx + (ph < 0.5f ? -off : off), my, 26, rgb(4, 6, 20));
    int illum = (int)(100 * (1 - fabsf(ph - 0.5f) * 2));
    float daysToFull = fmodf(0.5f - ph + 1.0f, 1.0f) * 29.53f, daysToNew = fmodf(1.0f - ph, 1.0f) * 29.53f;
    snprintf(b, sizeof(b), "%s %d%%", ph < 0.5f ? "waxing" : "waning", illum); radar->textC(162, b, FS, rgb(170, 175, 200));
    if (daysToFull < daysToNew) snprintf(b, sizeof(b), "full moon in %.0f days", daysToFull); else snprintf(b, sizeof(b), "new moon in %.0f days", daysToNew);
    radar->textC(180, b, FSR, rgb(120, 125, 150));
    if (wx.sunsetMin >= 0) { int dl = wx.sunsetMin - wx.sunriseMin; snprintf(b, sizeof(b), "sun %02d:%02d - %02d:%02d  %dh%02dm", wx.sunriseMin / 60, wx.sunriseMin % 60, wx.sunsetMin / 60, wx.sunsetMin % 60, dl / 60, dl % 60); radar->textC(200, b, FSR, COL_ARR); }
    snprintf(b, sizeof(b), "%s %02d %s", DN[t.tm_wday], t.tm_mday, MN[t.tm_mon]); radar->textC(218, b, FSR, rgb(90, 95, 120));
  }
  if (clockFace == 9) {           // year in dots: one dot per day, holidays in amber
    int year = t.tm_year + 1900; bool leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0; int days = leap ? 366 : 365;
    const int cols = 21, cell = 8, x0 = RADAR_CX - cols * cell / 2 + cell / 2, y0 = 50;
    for (int d = 0; d < days; d++) {
      int r = d / cols, c = d % cols, x = x0 + c * cell, y = y0 + r * cell;
      int dx2 = x - RADAR_CX, dy2 = y - RADAR_CY; if (dx2 * dx2 + dy2 * dy2 > 112 * 112) continue;
      uint16_t col = d < t.tm_yday ? rgb(70, 160, 110) : rgb(32, 32, 40);
      int hm, hd; if (holidayForYday(year, d, hm, hd)) col = d < t.tm_yday ? rgb(140, 100, 40) : rgb(255, 170, 60);
      if (d == t.tm_yday) { radar->fillCircle(x, y, 3, COL_TEXT); continue; }
      radar->fillRect(x - 1, y - 1, 3, 3, col);
    }
    snprintf(b, sizeof(b), "%02d:%02d   %d%%", t.tm_hour, t.tm_min, (t.tm_yday + 1) * 100 / days); radar->textC(38, b, FS, COL_TEXT_DIM);
    nextHoliday(b, sizeof(b)); if (b[0]) radar->textC(204, b, FSR, rgb(255, 170, 60));
    snprintf(b, sizeof(b), "day %d of %d", t.tm_yday + 1, days); radar->textC(222, b, FS, COL_TEXT);
  }
  if (clockFace == 10) {          // Huge: hours over minutes, 78 px digits, date in between
    snprintf(b, sizeof(b), "%02d", clock12 ? hour12(t.tm_hour) : t.tm_hour); radar->textC(100, b, &F_BIG104, COL_TEXT);
    if (clock12) radar->text(RADAR_CX + 72, 44, ampmStr(t.tm_hour), FS, COL_TEXT_DIM);
    snprintf(b, sizeof(b), "%02d", t.tm_min);  radar->textC(218, b, &F_BIG104, (t.tm_sec % 2) ? COL_TEXT : rgb(200, 200, 200));
    radar->drawLine(70, 118, 170, 118, COL_PANEL);
    snprintf(b, sizeof(b), "%s %02d %s", DN[t.tm_wday], t.tm_mday, MN[t.tm_mon]);
    if (wx.valid) { char w[16]; snprintf(w, sizeof(w), "   %.0f" DEG, wx.temp); strlcat(b, w, sizeof(b)); }
    radar->textC(130, b, FSR, COL_TEXT_DIM);
    if (alarmOn) { snprintf(b, sizeof(b), "alarm %02d:%02d", alarmMin / 60, alarmMin % 60); radar->textC(20, b, FSR, COL_TEXT_DIM); }
  }
  if (clockFace == 11) {          // Rings (6.15.0): hours, minutes, seconds as three progress rings, time in the middle
    float sec = t.tm_sec + clkSub, mf = (t.tm_min + sec / 60.0f) / 60.0f, hf = ((t.tm_hour % 12) + (t.tm_min + sec / 60.0f) / 60.0f) / 12.0f;
    ringProgress(118, 104, hf, COL_SWEEP, rgb(18, 34, 24));
    ringProgress(100, 86, mf, rgb(255, 170, 60), rgb(40, 28, 10));
    ringProgress(82, 68, sec / 60.0f, rgb(80, 160, 255), rgb(14, 24, 40));
    fmtHM(b, sizeof(b), t.tm_hour, t.tm_min); radar->textC(RADAR_CY + 8, b, FMB, COL_TEXT);
    if (clock12) radar->textC(RADAR_CY - 14, ampmStr(t.tm_hour), FSR, COL_TEXT_DIM);
    snprintf(b, sizeof(b), "%s %02d", DN[t.tm_wday], t.tm_mday); radar->textC(RADAR_CY + 28, b, FSR, COL_TEXT_DIM);
  }
  if (clockFace == 12) {          // Segments (6.15.0): seven-segment display, red-orange with ghost segments
    uint16_t on = rgb(255, 70, 30), off = rgb(36, 10, 6), onS = rgb(255, 120, 60);
    int hh = clock12 ? hour12(t.tm_hour) : t.tm_hour; int dg[4] = {hh / 10, hh % 10, t.tm_min / 10, t.tm_min % 10};
    if (clock12 && dg[0] == 0) dg[0] = -1;
    const int w = 36, h = 64, y = 80; int xs[4] = {22, 66, 138, 182};
    for (int i = 0; i < 4; i++) seg7(xs[i], y, w, h, dg[i], on, off);
    uint16_t cc = (t.tm_sec % 2) ? on : off; radar->fillRoundRect(116, y + 17, 8, 8, 2, cc); radar->fillRoundRect(116, y + 40, 8, 8, 2, cc);
    seg7(96, 160, 16, 28, t.tm_sec / 10, onS, off); seg7(118, 160, 16, 28, t.tm_sec % 10, onS, off);
    if (clock12) radar->text(142, 186, ampmStr(t.tm_hour), FSR, rgb(160, 70, 40));
    snprintf(b, sizeof(b), "%s %02d %s", DN[t.tm_wday], t.tm_mday, MN[t.tm_mon]); radar->textC(212, b, FSR, rgb(150, 80, 50));
    if (alarmOn) { snprintf(b, sizeof(b), "alarm %02d:%02d", alarmMin / 60, alarmMin % 60); radar->textC(52, b, FSR, rgb(150, 80, 50)); }
  }
  if (clockFace == 13) {          // Nixie (6.18.0): amber digits in glass tubes
    uint16_t glass = rgb(22, 16, 12), rim = rgb(70, 50, 30), amber = rgb(255, 120, 20), glow = rgb(90, 40, 8);
    int hh = clock12 ? hour12(t.tm_hour) : t.tm_hour; int dg[4] = {hh / 10, hh % 10, t.tm_min / 10, t.tm_min % 10}; int xs[4] = {26, 70, 130, 174};
    for (int i = 0; i < 4; i++) {
      int x = xs[i]; radar->fillRoundRect(x, 60, 40, 108, 14, glass); radar->drawRoundRect(x, 60, 40, 108, 14, rim); radar->drawLine(x + 6, 68, x + 6, 150, rgb(50, 38, 28));
      if (i == 0 && clock12 && dg[0] == 0) continue;
      snprintf(b, sizeof(b), "%d", dg[i]); int w = radar->textWidth(FB, b);
      radar->text(x + 20 - w / 2 + 1, 134, b, FB, glow); radar->text(x + 20 - w / 2, 132, b, FB, amber);
    }
    radar->fillCircle(RADAR_CX, 100, 3, t.tm_sec % 2 ? amber : glow); radar->fillCircle(RADAR_CX, 128, 3, t.tm_sec % 2 ? amber : glow);
    if (clock12) radar->textC(190, ampmStr(t.tm_hour), FSR, rgb(160, 90, 40));
    snprintf(b, sizeof(b), "%s %02d %s", DN[t.tm_wday], t.tm_mday, MN[t.tm_mon]); radar->textC(210, b, FSR, rgb(150, 100, 60));
  }
  if (clockFace == 14) {          // Words (6.18.0): the time in words
    static const char *HW[12] = {"twelve", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten", "eleven"};
    int m5 = (t.tm_min + 2) / 5; int hr = t.tm_hour % 12; if (m5 >= 7) hr = (hr + 1) % 12; if (m5 == 12) { m5 = 0; }
    const char *mw = m5 == 0 ? "o'clock" : m5 == 1 ? "five past" : m5 == 2 ? "ten past" : m5 == 3 ? "quarter past" : m5 == 4 ? "twenty past" : m5 == 5 ? "twenty-five past" :
                     m5 == 6 ? "half past" : m5 == 7 ? "twenty-five to" : m5 == 8 ? "twenty to" : m5 == 9 ? "quarter to" : m5 == 10 ? "ten to" : "five to";
    radar->textC(70, "it is", FS, COL_TEXT_DIM);
    if (m5 == 0) { radar->textC(118, HW[hr], FMB, COL_TEXT); radar->textC(150, "o'clock", FMB, COL_TEXT); }
    else { radar->textC(118, mw, FMB, COL_TEXT); radar->textC(150, HW[hr], FMB, COL_TEXT); }
    radar->textC(184, t.tm_hour < 12 ? "in the morning" : t.tm_hour < 17 ? "in the afternoon" : t.tm_hour < 21 ? "in the evening" : "at night", FS, COL_TEXT_DIM);
    fmtHM(b, sizeof(b), t.tm_hour, t.tm_min); radar->textC(212, b, FSR, rgb(80, 80, 80));
  }
  if (clockFace == 15) {          // Weather (6.18.0): time on top, the sky in the middle, temperature below
    fmtHM(b, sizeof(b), t.tm_hour, t.tm_min); radar->textC(62, b, FB2, COL_TEXT);
    if (clock12) radar->text(RADAR_CX + radar->textWidth(FB2, b) / 2 + 4, 62, ampmStr(t.tm_hour), FSR, COL_TEXT_DIM);
    if (wx.valid) {
      drawWeatherIcon(wx.wcode, t.tm_sec / 2, RADAR_CX, 118, 1.3f);
      snprintf(b, sizeof(b), "%.0f" DEG "  %s", wx.temp, wmoShort(wx.wcode)); radar->textC(184, b, FM, COL_TEXT);
      snprintf(b, sizeof(b), "H %d  L %d   rain %d%%", wx.dayMax[0], wx.dayMin[0], wx.rainProb[0]); radar->textC(206, b, FSR, COL_TEXT_DIM);
    } else radar->textC(130, "waiting for weather", FSR, COL_TEXT_DIM);
    snprintf(b, sizeof(b), "%s %02d %s", DN[t.tm_wday], t.tm_mday, MN[t.tm_mon]); radar->textC(226, b, FSR, rgb(80, 80, 80));
  }
  if (clockFace == 16) {          // Orbit (6.18.0): sun and moon circle the rim over 24 h, noon at the top
    drawDayRing(118, 10, false);
    float dayDeg = (t.tm_hour * 60 + t.tm_min) / 4.0f + 180; int sx, sy, mx, my;
    polar(dayDeg, 113, sx, sy); polar(dayDeg + 180, 113, mx, my);
    bool up = wx.sunriseMin >= 0 && (t.tm_hour * 60 + t.tm_min) >= wx.sunriseMin && (t.tm_hour * 60 + t.tm_min) <= wx.sunsetMin;
    radar->fillCircle(sx, sy, 9, up ? rgb(255, 200, 40) : rgb(120, 90, 20)); for (int k = 0; k < 8; k++) { int ex, ey; polarC(sx, sy, k * 45, 13, ex, ey); radar->drawLine(sx, sy, ex, ey, up ? rgb(255, 200, 40) : rgb(90, 70, 20)); }
    radar->fillCircle(mx, my, 8, rgb(200, 200, 215)); radar->fillCircle(mx + 4, my - 3, 7, COL_BG);
    radar->drawLine(RADAR_CX - 90, RADAR_CY, RADAR_CX + 90, RADAR_CY, rgb(36, 36, 44));
    fmtHM(b, sizeof(b), t.tm_hour, t.tm_min); radar->textC(RADAR_CY - 10, b, FB, COL_TEXT);
    if (clock12) radar->text(RADAR_CX + radar->textWidth(FB, b) / 2 + 4, RADAR_CY - 10, ampmStr(t.tm_hour), FS, COL_TEXT_DIM);
    snprintf(b, sizeof(b), "%s %02d %s", DN[t.tm_wday], t.tm_mday, MN[t.tm_mon]); radar->textC(RADAR_CY + 20, b, FSR, COL_TEXT_DIM);
    if (wx.sunriseMin >= 0) { char r1[12], r2[12]; fmtHMA(r1, sizeof(r1), wx.sunriseMin / 60, wx.sunriseMin % 60); fmtHMA(r2, sizeof(r2), wx.sunsetMin / 60, wx.sunsetMin % 60); snprintf(b, sizeof(b), "%s  -  %s", r1, r2); radar->textC(RADAR_CY + 44, b, FSR, rgb(150, 110, 40)); }
  }
}
void drawClockFrame() {
  struct tm t; bool ok = getTime(t);
  static unsigned long secStartMs = 0, lastSmoothMs = 0;
  bool smooth = ok && (clockFace == 1 || clockFace == 11);   // 6.15.0: sweeping second hand at 20 fps (the rings sweep on every face)
  if (ok && t.tm_sec != clkS) secStartMs = millis();
  if (ok && t.tm_sec == clkS && clkFaceDrawn == clockFace && !staticDirty && (!smooth || millis() - lastSmoothMs < 50)) return;
  lastSmoothMs = millis(); clkSub = (ok && smooth) ? fminf(0.999f, (millis() - secStartMs) / 1000.0f) : 0;
  staticDirty = false; clkFaceDrawn = clockFace; clkS = ok ? t.tm_sec : -1; clkT = t; clkOk = ok;
  present(paintClock);
}

// =====================================================================
// WEATHER: pages 0 now, 1 forecast, 2 history
// =====================================================================
int wxPhase = 0;
void drawSun(int x, int y, int r, int phase) {
  radar->fillCircle(x, y, r, rgb(255, 220, 40));
  for (int k = 0; k < 8; k++) { float a = k * M_PI / 4 + phase * 0.13f; int l = r + 6 + ((k + phase) % 2) * 3;
    radar->drawLine(x + (int)(cosf(a) * (r + 5)), y + (int)(sinf(a) * (r + 5)), x + (int)(cosf(a) * l), y + (int)(sinf(a) * l), rgb(255, 220, 40)); }
}
void drawCloud(int x, int y, int w, uint16_t col) {
  radar->fillCircle(x - w / 4, y + 4, w / 5, col); radar->fillCircle(x + w / 4, y + 4, w / 5, col);
  radar->fillCircle(x, y - 2, w / 3, col); radar->fillRect(x - w / 4, y + 2, w / 2, w / 5 + 2, col);
}
void drawMoon(int x, int y, int r) { radar->fillCircle(x, y, r, rgb(225, 225, 235)); radar->fillCircle(x + r / 2, y - r / 3, r - 2, COL_BG); }
bool isNightNow() { int m = minuteOfDay(); if (m < 0 || wx.sunriseMin < 0) return false; return m < wx.sunriseMin || m > wx.sunsetMin; }
void drawWeatherIcon(int code, int phase, int cx, int cy, float s) {
  int R = (int)(24 * s); bool night = (s >= 0.99f) && isNightNow();   // forecast icons stay daytime
  if (code == 0 && night) { drawMoon(cx, cy, R); for (int k = 0; k < 5; k++) { int sx = cx + (int)((k * 23 - 46) * s), sy = cy + (int)((k % 2 ? -30 : 30) * s); if ((phase + k) % 4) radar->drawPixel(sx, sy, COL_TEXT); } return; }
  if (code == 0 && wx.wind >= 25) { drawSun(cx - 6 * s, cy, R - 4, phase); for (int k = 0; k < 3; k++) { int off = (phase * 3 + k * 9) % 18; radar->drawLine(cx + 10 * s + off, cy + (k * 10 - 10) * s, cx + 26 * s + off, cy + (k * 10 - 10) * s, rgb(140, 160, 180)); } return; }
  if (code == 0) { drawSun(cx, cy, R, phase); return; }
  if (code <= 2 && night) { drawMoon(cx - 8 * s, cy - 6 * s, 18 * s); drawCloud(cx + 14 * s + (phase % 4 < 2 ? 1 : 0), cy + 12 * s, 46 * s, rgb(150, 150, 170)); return; }
  if (code <= 2) { drawSun(cx - 8 * s, cy - 6 * s, 20 * s, phase); drawCloud(cx + 14 * s + (phase % 4 < 2 ? 1 : 0), cy + 12 * s, 46 * s, rgb(215, 215, 228)); return; }
  if (code == 3) { drawCloud(cx - 10 * s + (phase % 6) / 3, cy - 6 * s, 44 * s, rgb(150, 150, 165)); drawCloud(cx + 10 * s, cy + 10 * s, 50 * s, rgb(200, 200, 212)); return; }
  if (code <= 48) { drawCloud(cx, cy - 10 * s, 44 * s, rgb(140, 140, 150)); for (int i = 0; i < 4; i++) { int off = ((phase + i * 3) % 8) - 4; radar->drawLine(cx - 26 * s + off, cy + (14 + i * 7) * s, cx + 26 * s + off, cy + (14 + i * 7) * s, rgb(120, 120, 135)); } return; }
  if (code <= 67 || (code >= 80 && code <= 82)) {
    drawCloud(cx, cy - 10 * s, 48 * s, rgb(120, 125, 150));
    for (int i = 0; i < 6; i++) { int dy = (phase * 4 + i * 7) % 28; radar->drawLine(cx - 20 * s + i * 8 * s, cy + 8 * s + dy * s, cx - 22 * s + i * 8 * s, cy + 14 * s + dy * s, rgb(80, 160, 255)); }
    return;
  }
  if (code <= 86) {
    drawCloud(cx, cy - 10 * s, 48 * s, rgb(170, 175, 190));
    for (int i = 0; i < 6; i++) { int dy = (phase * 2 + i * 9) % 28; int sx = cx - 20 * s + i * 8 * s + ((phase + i) % 3) - 1; radar->fillCircle(sx, cy + 8 * s + dy * s, 1, COL_TEXT); }
    return;
  }
  drawCloud(cx, cy - 12 * s, 50 * s, rgb(90, 90, 110));
  if (phase % 8 < 3) { uint16_t y = rgb(255, 230, 60); radar->fillTriangle(cx + 2 * s, cy + 4 * s, cx - 8 * s, cy + 20 * s, cx, cy + 18 * s, y); radar->fillTriangle(cx, cy + 16 * s, cx + 8 * s, cy + 14 * s, cx - 4 * s, cy + 34 * s, y); }
  for (int i = 0; i < 4; i++) { int dy = (phase * 5 + i * 9) % 26; radar->drawLine(cx - 24 * s + i * 10 * s, cy + 10 * s + dy * s, cx - 26 * s + i * 10 * s, cy + 16 * s + dy * s, rgb(80, 160, 255)); }
}
void paintWeather() {
  radar->fillScreen(COL_BG); char b[48];
  if (!wx.valid) { radar->textC(124, "fetching weather", FSR, COL_TEXT); return; }
  if (subPage == 0) {
    radar->textC(24, weatherPlace(), FSR, COL_TEXT_DIM);
    drawWeatherIcon(wx.wcode, wxPhase, 68, 88, 1.0f);
    snprintf(b, sizeof(b), "%.0f" DEG, wx.temp); radar->text(130, 92, b, FB2, COL_TEXT);
    snprintf(b, sizeof(b), "feels %.0f" DEG, wx.feels); radar->text(134, 110, b, FSR, COL_TEXT_DIM);
    radar->textC(140, wmoText(wx.wcode), FM, rgb(190, 190, 190));
    struct { const char *lab; char val[8]; uint16_t col; } tiles[3];
    tiles[0].lab = "HUM";  snprintf(tiles[0].val, 8, "%.0f%%", wx.hum); tiles[0].col = COL_ENROUTE_HIGH;
    tiles[1].lab = "WIND"; snprintf(tiles[1].val, 8, "%.0f", wx.wind);  tiles[1].col = COL_OK;
    tiles[2].lab = "AQI";  snprintf(tiles[2].val, 8, "%d", wx.aqi);     tiles[2].col = aqiColour(wx.aqi);
    for (int i = 0; i < 3; i++) {
      int x = 36 + i * 58;
      radar->drawRoundRect(x, 152, 50, 40, 6, COL_PANEL);
      radar->text(x + 25 - radar->textWidth(FSR, tiles[i].lab) / 2, 166, tiles[i].lab, FSR, COL_TEXT_DIM);
      radar->text(x + 25 - radar->textWidth(FM, tiles[i].val) / 2, 187, tiles[i].val, FM, tiles[i].col);
    }
  } else if (subPage == 1) {
    radar->textC(30, "3-DAY FORECAST", FS, COL_TEXT_DIM);
    struct tm t; bool haveT = getTime(t);
    for (int d = 0; d < 3; d++) {
      int y = 62 + d * 46;
      const char *name = d == 0 ? "Today" : d == 1 ? "Tomorrow" : (haveT ? DNL[(t.tm_wday + 2) % 7] : "Day 3");
      radar->text(28, y + 8, name, FS, COL_TEXT);
      drawWeatherIcon(wx.dayCode[d], wxPhase, 118, y + 4, 0.5f);
      snprintf(b, sizeof(b), "%d" DEG, wx.dayMax[d]); radar->textR(214, y + 2, b, FM, COL_TEXT);
      snprintf(b, sizeof(b), "%d" DEG, wx.dayMin[d]); radar->textR(214, y + 20, b, FSR, COL_TEXT_DIM);
      radar->text(28, y + 24, wmoShort(wx.dayCode[d]), FSR, COL_TEXT_DIM);
      if (d < 2) radar->drawLine(30, y + 34, 210, y + 34, rgb(30, 30, 30));
    }
    uint32_t nowT = (uint32_t)time(NULL);
    if (quakes[0].valid && nowT > quakes[0].t && nowT - quakes[0].t < 86400) { snprintf(b, sizeof(b), "M%.1f quake %.0f km, %luh ago", quakes[0].mag, quakes[0].distKm, (unsigned long)((nowT - quakes[0].t) / 3600)); radar->textC(214, b, FSR, COL_ARR); }
    else { int maxP = 0, at = -1; for (int i = 0; i < 24; i++) if (wx.rainProb[i] > maxP) { maxP = wx.rainProb[i]; at = i; } if (maxP >= 30 && haveT) snprintf(b, sizeof(b), "rain %d%% around %02d:00", maxP, (t.tm_hour + at) % 24); else if (rainMonth >= 0) snprintf(b, sizeof(b), "rain %.0f mm this month, %.0f mm in 7 d", rainMonth, rain7d); else strcpy(b, "no rain expected in 24 h"); radar->textC(214, b, FSR, COL_ENROUTE_HIGH); }
  } else {
    radar->textC(30, "LAST 24 HOURS", FS, COL_TEXT_DIM);
    int n = histWrapped ? 96 : histIdx;
    if (n < 2) { radar->textC(124, "collecting samples...", FSR, COL_TEXT_DIM); return; }
    int8_t tmin = 127, tmax = -128; uint8_t amax = 1;
    for (int k = 0; k < n; k++) { int i = (histIdx - n + k + 96) % 96; if (tempHist[i] < tmin) tmin = tempHist[i]; if (tempHist[i] > tmax) tmax = tempHist[i]; if (aqiHist[i] > amax) amax = aqiHist[i]; }
    if (tmax - tmin < 4) tmax = tmin + 4;
    const int x0 = 30, x1 = 210;
    for (int k = 1; k < n; k++) {
      int i0 = (histIdx - n + k - 1 + 96) % 96, i1 = (histIdx - n + k + 96) % 96;
      int xa = x0 + (k - 1) * (x1 - x0) / 95, xb = x0 + k * (x1 - x0) / 95;
      radar->drawLine(xa, 100 - (tempHist[i0] - tmin) * 40 / (tmax - tmin), xb, 100 - (tempHist[i1] - tmin) * 40 / (tmax - tmin), COL_ARR);
      radar->drawLine(xa, 180 - aqiHist[i0] * 50 / amax, xb, 180 - aqiHist[i1] * 50 / amax, aqiColour(aqiHist[i1]));
    }
    snprintf(b, sizeof(b), "temp %d..%d" DEG, tmin, tmax); radar->text(x0, 118, b, FSR, COL_ARR);
    snprintf(b, sizeof(b), "AQI max %d", amax); radar->text(x0, 198, b, FSR, COL_TEXT_DIM);
    const char *worst = wx.o3 > 100 ? "ozone" : wx.no2 > 40 ? "NO2" : wx.so2 > 40 ? "SO2" : wx.co > 4000 ? "CO" : wx.pm25 > 35 ? "PM2.5" : "none high";
    snprintf(b, sizeof(b), "O3 %.0f  NO2 %.0f  SO2 %.0f  CO %.0f  %s", wx.o3, wx.no2, wx.so2, wx.co / 1000, worst); radar->textC(222, b, FSR, rgb(110, 110, 110));
  }
}
void drawWeatherFrame() {
  if (!staticDirty && millis() - lastHudMs < 250) return;
  lastHudMs = millis(); staticDirty = false; wxPhase++;
  present(paintWeather);
}

// =====================================================================
// POMODORO
// =====================================================================
// ---------- Focus Pro rendering (5.5.0) ----------
uint16_t hsh(uint32_t x) { x ^= x >> 16; x *= 0x7feb352dU; x ^= x >> 15; x *= 0x846ca68bU; x ^= x >> 16; return (uint16_t)x; }
#define C_AMBER rgb(255, 170, 0)
#define C_GREEN rgb(0, 210, 90)
// "2h 05m": big digits (the large font has digits only) with small unit letters, centred
void drawHMc(int yb, unsigned mins, uint16_t col, uint16_t ucol) {
  char h[6], m[4]; snprintf(h, sizeof(h), "%u", mins / 60);
  if (mins >= 60) snprintf(m, sizeof(m), "%02u", mins % 60); else snprintf(m, sizeof(m), "%u", mins % 60);
  int wh = mins >= 60 ? radar->textWidth(FB2, h) + radar->textWidth(FM, "h ") : 0;
  int x = RADAR_CX - (wh + radar->textWidth(FB2, m) + radar->textWidth(FM, "m")) / 2;
  if (mins >= 60) { radar->text(x, yb, h, FB2, col); x += radar->textWidth(FB2, h); radar->text(x, yb, "h ", FM, ucol); x += radar->textWidth(FM, "h "); }
  radar->text(x, yb, m, FB2, col); x += radar->textWidth(FB2, m); radar->text(x, yb, "m", FM, ucol);
}
// today's goal as a ring; ghost = what the running session will add
void goalRing(int rOut, int w, unsigned mins, unsigned ghost) {
  float g = pcfg.goalMin; bool met = mins >= pcfg.goalMin; float f = met ? 1.0f : mins / g;
  radar->ringArc(RADAR_CX, RADAR_CY, rOut, rOut - w, 0, 359.9f, met ? rgb(0, 50, 25) : rgb(45, 32, 8));
  if (ghost && !met) { float f2 = (mins + ghost) >= g ? 1.0f : (mins + ghost) / g; if (f2 > f) radar->ringArc(RADAR_CX, RADAR_CY, rOut, rOut - w, f * 359.9f, f2 * 359.9f, rgb(100, 75, 20)); }
  if (f > 0.004f) radar->ringArc(RADAR_CX, RADAR_CY, rOut, rOut - w, 0, f * 359.9f, met ? C_GREEN : C_AMBER);
  for (int i = 0; i < 12; i++) { float a = i * 0.5236f; radar->drawLine(RADAR_CX + (int)((rOut - w) * sinf(a)), RADAR_CY - (int)((rOut - w) * cosf(a)), RADAR_CX + (int)(rOut * sinf(a)), RADAR_CY - (int)(rOut * cosf(a)), COL_BG); }
}
void flame(int cx, int cy, int sz) {
  uint16_t o = rgb(255, 120, 30), y = rgb(255, 220, 80);
  radar->fillCircle(cx, cy + sz / 4, sz / 2, o); radar->fillTriangle(cx - sz / 2, cy + sz / 4, cx + sz / 2, cy + sz / 4, cx, cy - sz, o);
  radar->fillCircle(cx, cy + sz / 3, sz / 4, y); radar->fillTriangle(cx - sz / 4, cy + sz / 3, cx + sz / 4, cy + sz / 3, cx, cy - sz / 3, y);
}
void streakLine(int yb, unsigned st, const GFXfont *f) {
  char b[24]; snprintf(b, sizeof(b), "%u day streak", st);
  int w = 18 + radar->textWidth(f, b), x = RADAR_CX - w / 2;
  flame(x + 6, yb - 5, 8); radar->text(x + 18, yb, b, f, COL_TEXT);
}
// fixed colours only: the band canvas keeps a 256-entry palette, smooth gradients would exhaust it
void sparks(float t, int seed, int n, int r0, int r1) {
  static const uint8_t P[5][3] = {{255, 70, 70}, {255, 170, 0}, {255, 220, 120}, {220, 220, 220}, {0, 210, 90}};
  for (int i = 0; i < n; i++) {
    uint16_t h = hsh(seed * 977 + i); float a = (h % 628) / 100.0f, sp = 0.6f + ((h >> 4) % 40) / 100.0f;
    int r = r0 + (int)((r1 - r0) * t * sp), x = RADAR_CX + (int)(r * cosf(a)), y = RADAR_CY + (int)(r * sinf(a));
    const uint8_t *c = P[(h >> 9) % 5]; int sz = t > 0.8f ? 1 : 1 + (h >> 12) % 3;
    radar->fillCircle(x, y, sz, rgb(c[0], c[1], c[2]));
  }
}
void firework(int cx, int cy, float t, int seed) {
  static const uint8_t P[5][3] = {{0, 210, 90}, {255, 170, 0}, {120, 200, 255}, {255, 120, 200}, {255, 230, 120}};
  const uint8_t *c = P[seed % 5]; uint16_t col = rgb(c[0], c[1], c[2]);
  float r1 = 4 + 16 * t, r0 = r1 * 0.45f;
  for (int j = 0; j < 12; j++) { float a = j * 0.5236f; radar->drawLine(cx + (int)(r0 * cosf(a)), cy + (int)(r0 * sinf(a)), cx + (int)(r1 * cosf(a)), cy + (int)(r1 * sinf(a)), col); }
}
void paintPomoSettings() {
  char v[16];
  radar->textC(40, "POMODORO SETTINGS", FS, COL_SELECTED);
  int first = psetSel - 2; if (first < 0) first = 0; if (first > PSET_N - 6) first = PSET_N - 6;
  for (int k = 0; k < 6; k++) {
    int i = first + k, y = 68 + k * 25; bool on = i == psetSel; const int x0 = 30, x1 = 210;
    psetValue(i, v, sizeof(v));
    if (on) { uint16_t edge = psetEdit ? C_AMBER : COL_SELECTED; radar->fillRoundRect(x0, y - 17, x1 - x0, 23, 6, psetEdit ? rgb(60, 40, 0) : rgb(50, 20, 20)); radar->drawRoundRect(x0, y - 17, x1 - x0, 23, 6, edge); }
    radar->text(x0 + 8, y, PSET_NAMES[i], on ? FS : FSR, on ? COL_TEXT : COL_TEXT_DIM);
    radar->textR(x1 - 8, y, v, FS, on ? (psetEdit ? C_AMBER : COL_TEXT) : COL_TEXT_DIM);
  }
  radar->textC(222, psetEdit ? "tap: save" : "hold: back", FSR, rgb(90, 90, 90));
}
void paintPomoHome() {
  char b[40], g[12]; unsigned tm = todayFocus(); fmtHMs(g, sizeof(g), pcfg.goalMin); bool met = tm >= pcfg.goalMin;
  goalRing(119, 7, tm, 0);
  radar->textC(52, "TODAY", FSR, COL_TEXT_DIM);
  drawHMc(92, tm, COL_TEXT, COL_TEXT_DIM);
  if (met) snprintf(b, sizeof(b), "goal %s met", g); else snprintf(b, sizeof(b), "of %s goal  %u%%", g, tm * 100 / pcfg.goalMin);
  radar->textC(112, b, FSR, met ? C_GREEN : C_AMBER);
  unsigned st = streakNow(); if (st) streakLine(136, st, FS); else radar->textC(136, "start a streak today", FSR, COL_TEXT_DIM);
  if (pomoPreset == POMO_TIMER_PRESET) snprintf(b, sizeof(b), "%d min timer", pomoTimerMin);
  else if (POMO_PRESETS[pomoPreset].focus) snprintf(b, sizeof(b), "%d / %d %s", POMO_PRESETS[pomoPreset].focus, POMO_PRESETS[pomoPreset].brk, POMO_PRESETS[pomoPreset].name);
  else { int cb = pomoCustomFocus / 5 < 3 ? 3 : pomoCustomFocus / 5; snprintf(b, sizeof(b), "%d / %d custom", pomoCustomFocus, cb); }
  int w = radar->textWidth(FS, b) + 40; if (w > 196) w = 196;
  uint16_t edge = pomoEditing ? C_AMBER : COL_SELECTED;
  radar->fillRoundRect(RADAR_CX - w / 2, 146, w, 24, 8, pomoEditing ? rgb(60, 40, 0) : rgb(50, 20, 20)); radar->drawRoundRect(RADAR_CX - w / 2, 146, w, 24, 8, edge);
  radar->text(RADAR_CX - w / 2 + 8, 163, "<", FS, edge); radar->textR(RADAR_CX + w / 2 - 8, 163, ">", FS, edge);
  radar->textC(163, b, FS, COL_TEXT);
  unsigned mx = pcfg.goalMin; for (int i = 0; i < 7; i++) if (last7(i) > mx) mx = last7(i);
  for (int i = 0; i < 7; i++) {
    unsigned v = last7(i); int h = (int)(v * 24 / mx); if (h < 1) h = 1; int x = 68 + i * 15;
    radar->fillRect(x, 202 - h, 10, h, i == 6 ? COL_SELECTED : v >= pcfg.goalMin ? rgb(0, 190, 80) : rgb(110, 50, 50));
  }
  int gy = 202 - (int)(pcfg.goalMin * 24 / mx); radar->drawLine(64, gy, 176, gy, rgb(200, 160, 40));
  radar->textC(222, pomoEditing ? "turn: minutes" : "hold: settings", FSR, rgb(70, 70, 70));
}
void paintPomodoro() {
  unsigned long rem = pomoRemainMs(); int sec = (int)(rem / 1000); char b[40];
  radar->fillScreen(COL_BG);
  if (pomoSettings) { paintPomoSettings(); return; }
  if (pomoAskLog) {   // end-of-session prompt
    radar->textC(52, "END SESSION", FMB, COL_SELECTED);
    snprintf(b, sizeof(b), "%lu of %d min", (pomoTotalMs - pomoRemainAtPause) / 60000UL, (int)focusPlanMin); radar->textC(80, b, FS, COL_TEXT);
    radar->textC(106, "log to sheet + calendar?", FSR, COL_TEXT_DIM);
    for (int i = 0; i < 2; i++) {
      int x = i == 0 ? 52 : 128; bool sel = (pomoAskSel == i); uint16_t c = i == 0 ? COL_OK : COL_SELECTED;
      if (sel) radar->fillRoundRect(x, 124, 60, 34, 8, i == 0 ? rgb(0, 50, 25) : rgb(60, 15, 15));
      radar->drawRoundRect(x, 124, 60, 34, 8, sel ? c : rgb(70, 70, 70));
      const char *lbl = i == 0 ? "YES" : "NO";
      radar->text(x + (60 - radar->textWidth(FS, lbl)) / 2, 147, lbl, FS, sel ? c : COL_TEXT_DIM);
    }
    radar->textC(184, "turn: choose  tap: OK", FSR, rgb(90, 90, 90));
    radar->textC(202, "hold: back", FSR, rgb(90, 90, 90));
    return;
  }
  if (pomoState == POMO_SELECT) { paintPomoHome(); return; }
  bool focus = (pomoState == POMO_FOCUS) || (pomoState == POMO_PAUSED && pomoWasFocus);
  bool paused = pomoState == POMO_PAUSED;
  uint16_t col = pomoIsTimer ? COL_ARR : (focus ? COL_SELECTED : COL_OK), dimc = pomoIsTimer ? rgb(40, 30, 10) : (focus ? rgb(40, 18, 18) : rgb(14, 40, 24));
  if (paused) { col = focus ? rgb(130, 60, 60) : rgb(50, 110, 70); dimc = focus ? rgb(30, 12, 12) : rgb(10, 30, 18); }
  float frac = pomoTotalMs ? 1.0f - (float)rem / pomoTotalMs : 0;
  unsigned el = (unsigned)((pomoTotalMs - rem) / 60000UL);
  bool last = focus && !paused && !pomoIsTimer && sec < 60;
  if (!pomoIsTimer) goalRing(119, 4, todayFocus() + (focus ? el : 0), (focus && !paused) ? (unsigned)(rem / 60000UL) + 1 : 0);
  if (last && pcfg.anim == 0) {   // last minute: a soft red pulse behind the digits (4 fixed shades)
    static const uint8_t G[4][3][3] = {{{14, 3, 3}, {22, 5, 5}, {30, 7, 7}}, {{20, 4, 4}, {32, 7, 7}, {46, 10, 10}}, {{26, 5, 5}, {42, 9, 9}, {62, 13, 13}}, {{32, 6, 6}, {52, 11, 11}, {78, 16, 16}}};
    int q = (int)((0.5f + 0.5f * sinf(animT * 0.0039f)) * 3.99f);
    radar->fillCircle(RADAR_CX, RADAR_CY, 96, rgb(G[q][0][0], G[q][0][1], G[q][0][2]));
    radar->fillCircle(RADAR_CX, RADAR_CY, 74, rgb(G[q][1][0], G[q][1][1], G[q][1][2]));
    radar->fillCircle(RADAR_CX, RADAR_CY, 50, rgb(G[q][2][0], G[q][2][1], G[q][2][2]));
  }
  radar->ringArc(RADAR_CX, RADAR_CY, 112, 103, 0, 359.9f, dimc);
  if (frac > 0.002f) radar->ringArc(RADAR_CX, RADAR_CY, 112, 103, 0, frac * 359.9f, col);
  if (!paused && frac > 0.002f) { float a = frac * 6.2832f; radar->fillCircle(RADAR_CX + (int)(107.5f * sinf(a)), RADAR_CY - (int)(107.5f * cosf(a)), 5, rgb(255, 210, 210)); }
  if (breathing && !focus && pomoState == POMO_BREAK) {   // box breathing: 4 s in, 4 s hold, 4 s out, 4 s hold
    unsigned long ph = (millis() - breathStartMs) % 16000UL; int stage = ph / 4000; float f = (ph % 4000) / 4000.0f;
    float k = stage == 0 ? f : stage == 1 ? 1.0f : stage == 2 ? 1.0f - f : 0.0f; int r = 28 + (int)(k * 50);
    radar->fillCircle(RADAR_CX, 118, r, rgb(10, 40, 50)); radar->drawCircle(RADAR_CX, 118, r, rgb(80, 200, 220));
    const char *lab[4] = {"breathe in", "hold", "breathe out", "hold"}; radar->textC(124, lab[stage], FS, COL_TEXT);
    snprintf(b, sizeof(b), "break %02d:%02d", sec / 60, sec % 60); radar->textC(196, b, FSR, COL_TEXT_DIM);
    radar->textC(216, "click: stop", FSR, rgb(70, 70, 70));
    return;
  }
  radar->textC(62, paused ? "PAUSED" : pomoIsTimer ? "TIMER" : last ? "LAST MINUTE" : (focus ? "FOCUS" : "BREAK"), FMB, last ? C_AMBER : col);
  snprintf(b, sizeof(b), "%02d:%02d", sec / 60, sec % 60); radar->textC(134, b, FB, paused ? rgb(140, 140, 140) : COL_TEXT);
  if (pomoIsTimer) snprintf(b, sizeof(b), "%d min timer", pomoTimerMin);
  else if (focus && pomoTask[0]) strlcpy(b, pomoTask, sizeof(b));
  else if (focus) snprintf(b, sizeof(b), "%d / %d  %s", pomoFocusMin(), pomoBreakMin(), POMO_PRESETS[pomoPreset].name);
  else snprintf(b, sizeof(b), "stand up, water, eyes");
  radar->textC(158, b, (focus && pomoTask[0] && !pomoIsTimer) ? FS : FSR, (focus && pomoTask[0] && !pomoIsTimer && !paused) ? COL_TEXT : COL_TEXT_DIM);
  if (!pomoIsTimer) {
    for (int i = 0; i < POMO_SESSIONS_PER_CYCLE; i++) { int x = RADAR_CX - 24 + i * 16; bool done = i < (pomoSession % POMO_SESSIONS_PER_CYCLE) || (pomoSession > 0 && pomoSession % POMO_SESSIONS_PER_CYCLE == 0 && !focus); radar->fillCircle(x, 176, 5, done ? COL_SELECTED : rgb(60, 30, 30)); }
  }
  if (paused) {
    unsigned long ps = (millis() - pomoPausedAtMs) / 1000; snprintf(b, sizeof(b), "paused %lu:%02lu", ps / 60, ps % 60);
    radar->textC(196, b, FSR, COL_TEXT_DIM); radar->textC(212, focus ? "2x tap: end" : "tap: resume", FSR, rgb(80, 80, 80));
  } else if (pomoIsTimer) radar->textC(198, "hold: cancel", FSR, rgb(80, 80, 80));
  else if (focus) { char d[12]; fmtHMs(d, sizeof(d), todayFocus() + el); snprintf(b, sizeof(b), "+%um today  %s", el, d); radar->textC(198, b, FSR, C_AMBER); }
  else { if (pcfg.autoStart) snprintf(b, sizeof(b), "next: focus %d", pomoFocusMin()); else strlcpy(b, "then tap to start", sizeof(b)); radar->textC(198, b, FSR, COL_TEXT_DIM); radar->textC(214, "tap: breathe", FSR, rgb(70, 70, 70)); }
}
void drawPomodoroFrame() {
  unsigned long rem = pomoRemainMs(); int sec = (int)(rem / 1000);
  bool pulse = pcfg.anim == 0 && pomoState == POMO_FOCUS && !pomoIsTimer && sec < 60 && !pomoSettings && !pomoAskLog;
  if ((breathing && pomoState == POMO_BREAK) || pulse) { if (!staticDirty && millis() - lastHudMs < 80) return; lastHudMs = millis(); staticDirty = false; clkS = sec; animT = millis(); present(paintPomodoro); return; }
  int key = pomoState == POMO_PAUSED ? (int)((millis() - pomoPausedAtMs) / 1000) : sec;
  if (!staticDirty && pomoState != POMO_SELECT && key == clkS) return;
  if (!staticDirty && pomoState == POMO_SELECT) return;
  staticDirty = false; clkS = key; animT = millis();
  present(paintPomodoro);
}
// ---------- overlays: greetings and celebrations, drawn over any app ----------
void paintOverlay() {
  radar->fillScreen(COL_BG); char b[40], g[12]; fmtHMs(g, sizeof(g), pcfg.goalMin);
  bool full = pcfg.anim == 0;
  switch (ov.kind) {
    case OV_GREET: {
      float f = full ? (animT >= 700 ? 1.0f : animT / 700.0f) : 1.0f;
      if (f > 0.004f) radar->ringArc(RADAR_CX, RADAR_CY, 119, 113, 0, 359.9f * f, C_AMBER);
      bool big = ov.l2[0] != 0;
      radar->textC(big ? 98 : 118, ov.l1, FM, big ? COL_TEXT_DIM : COL_TEXT);
      if (big) radar->textC(134, ov.l2, FMB, COL_TEXT);
      if (ov.l3[0]) radar->textC(big ? 166 : 148, ov.l3, FSR, C_AMBER);
      if (ov.a == 0) { snprintf(b, sizeof(b), "v%s", FW_VERSION); radar->textC(206, b, FSR, rgb(70, 70, 70)); }
      break; }
    case OV_SESSION: {
      unsigned add = ov.a, before = ov.b; float pr = full ? (animT >= 900 ? 1.0f : animT / 900.0f) : 1.0f; unsigned shown = before + (unsigned)(add * pr + 0.5f);
      goalRing(119, 7, shown, 0);
      if (full && animT < 900) { int r = 112 - (int)(60 * animT / 900); radar->ringArc(RADAR_CX, RADAR_CY, r, r - 8, 0, 359.9f * (1.0f - animT / 900.0f), COL_SELECTED); }
      if (full && animT > 400 && animT < 2400) sparks((animT - 400) / 2000.0f, 3, 24, 60, 112);
      radar->textC(84, ov.l2, FS, C_GREEN);
      snprintf(b, sizeof(b), "+%u", (unsigned)(add * pr + 0.5f)); int w = radar->textWidth(FB2, b) + radar->textWidth(FM, " min"), x = RADAR_CX - w / 2;
      radar->text(x, 128, b, FB2, COL_TEXT); radar->text(x + radar->textWidth(FB2, b), 128, " min", FM, COL_TEXT_DIM);
      radar->textC(156, ov.l1, FS, COL_TEXT);
      char d[12]; fmtHMs(d, sizeof(d), shown); snprintf(b, sizeof(b), "%s of %s", d, g); radar->textC(178, b, FSR, C_AMBER);
      break; }
    case OV_GOAL: {
      goalRing(119, 10, pcfg.goalMin, 0);
      if (full) { static const uint8_t PX[5][2] = {{58, 66}, {182, 66}, {120, 36}, {44, 128}, {196, 128}};
        for (int i = 0; i < 5; i++) { unsigned long ph = animT + i * 260; firework(PX[i][0], PX[i][1], (ph % 1300) / 1300.0f, i + (int)(ph / 1300)); } }
      radar->textC(100, "GOAL MET", FMB, C_GREEN);
      drawHMc(142, ov.a, COL_TEXT, COL_TEXT_DIM);
      radar->textC(168, ov.l1, FS, COL_TEXT);
      { unsigned st = streakNow(); if (st) streakLine(192, st, FSR); }
      break; }
    case OV_STREAK: {
      for (int i = 0; i < 7; i++) radar->ringArc(RADAR_CX, RADAR_CY, 118, 112, i * 51.43f + 2, (i + 1) * 51.43f - 2, rgb(255, 120 + i * 15, 30));
      flame(RADAR_CX, 84, full ? 26 + (int)(3 * sinf(animT * 0.006f)) : 28);
      snprintf(b, sizeof(b), "%u", ov.a); radar->textC(158, b, FB2, COL_TEXT);
      radar->textC(180, "DAY STREAK", FS, C_AMBER);
      radar->textC(202, ov.l1, FSR, COL_TEXT_DIM);
      break; }
    case OV_RECAP: {
      unsigned tm = todayFocus(); bool met = tm >= pcfg.goalMin; char v[28], d[12];
      goalRing(118, 5, tm, 0);
      radar->textC(44, "TODAY", FSR, COL_TEXT_DIM); drawHMc(80, tm, COL_TEXT, COL_TEXT_DIM);
      if (met) radar->textC(100, "goal met", FS, C_GREEN); else { fmtHMs(d, sizeof(d), pcfg.goalMin - tm); snprintf(b, sizeof(b), "%s short of %s", d, g); radar->textC(100, b, FS, C_AMBER); }
      bool fs = fstats.valid && fstats.day == todayKey();
      const char *L[5] = {"sessions", "best hour", "longest", "streak", "week"};
      for (int r = 0; r < 5; r++) {
        v[0] = 0;
        if (r == 0) { if (fs) snprintf(v, sizeof(v), "%u  (%u done)", todaySessions(), fstats.done); else snprintf(v, sizeof(v), "%u", todaySessions()); }
        if (r == 1) { if (fs && fstats.hour >= 0) snprintf(v, sizeof(v), "%02d:00-%02d:00", fstats.hour, (fstats.hour + 1) % 24); else strlcpy(v, "-", sizeof(v)); }
        if (r == 2) { if (fs && fstats.longest) snprintf(v, sizeof(v), "%u min", fstats.longest); else strlcpy(v, "-", sizeof(v)); }
        if (r == 3) { unsigned st = streakNow(), bs = fstats.valid && fstats.bestStreak > st ? fstats.bestStreak : st; snprintf(v, sizeof(v), "%u (best %u)", st, bs); }
        if (r == 4) { if (fs) { fmtHMs(d, sizeof(d), fstats.week); if (fstats.prev) snprintf(v, sizeof(v), "%s  %+d%%", d, (int)(((long)fstats.week - (long)fstats.prev) * 100 / (long)fstats.prev)); else strlcpy(v, d, sizeof(v)); } else strlcpy(v, "-", sizeof(v)); }
        int y = 122 + r * 17; radar->text(40, y, L[r], FSR, COL_TEXT_DIM); radar->text(106, y, v, FS, COL_TEXT);
      }
      radar->textC(210, ov.l1, FSR, COL_TEXT);
      break; }
    default: break;
  }
}
void drawOverlayFrame() {
  static unsigned long last = 0; unsigned long iv = pcfg.anim == 0 ? 60 : 300; if (ov.kind == OV_RECAP) iv = 1000;
  if (!staticDirty && millis() - last < iv) return;
  last = millis(); staticDirty = false; animT = millis() - ov.startMs;
  present(paintOverlay);
}
// OLED while an overlay is up
void drawOverlayInfo() {
  char b[40], g[12], d[12]; fmtHMs(g, sizeof(g), pcfg.goalMin); unsigned tm = todayFocus(); fmtHMs(d, sizeof(d), tm);
  switch (ov.kind) {
    case OV_GREET: { snprintf(b, sizeof(b), "v%s", FW_VERSION); oledTimeHead(b, ""); snprintf(b, sizeof(b), "%s %s", ov.l1, ov.l2); pgAdd(b, true);
      pgAddf("goal today %s", g); unsigned st = streakNow(); if (st) pgAddf("streak %u days", st); else pgAdd("start a streak today"); break; }
    case OV_SESSION: { snprintf(b, sizeof(b), "+%u MIN", ov.a); oledTimeHead(b, ""); snprintf(b, sizeof(b), "%s of %s  %u%%", d, g, tm * 100 / pcfg.goalMin); pgAdd(b, true);
      if (tm < pcfg.goalMin) { char lf[12]; fmtHMs(lf, sizeof(lf), pcfg.goalMin - tm); int f = pomoFocusMin() > 0 ? pomoFocusMin() : 25; int n = (int)((pcfg.goalMin - tm + f - 1) / f); pgAddf("%s to goal: %d session%s", lf, n, n == 1 ? "" : "s"); }
      else pgAdd("goal done, bonus time");
      pgAdd(FOCUS_LOG_URL[0] ? "saving to sheet + cal" : "kept on the device"); break; }
    case OV_GOAL: { oledTimeHead("GOAL MET", ""); snprintf(b, sizeof(b), "%s  streak %u", d, streakNow()); pgAdd(b, true);
      if (fstats.valid) { char w[12]; fmtHMs(w, sizeof(w), fstats.week); pgAddf("best %u  week %s", fstats.bestStreak, w); }
      pgAdd("extra time counts too"); break; }
    case OV_STREAK: { snprintf(b, sizeof(b), "%u DAYS", ov.a); oledTimeHead(b, ""); pgAdd(ov.l1, true);
      if (fstats.valid) pgAddf("best streak %u", (unsigned)(fstats.bestStreak > ov.a ? fstats.bestStreak : ov.a));
      pgAdd("keep the chain going"); break; }
    case OV_RECAP: { struct tm t; char dn[6] = "TODAY"; if (getTime(t)) strftime(dn, sizeof(dn), "%a", &t);
      oledHead(dn, d, tm >= pcfg.goalMin ? "goal met" : "");
      bool fs = fstats.valid && fstats.day == todayKey();
      snprintf(b, sizeof(b), "%u sessions  %u done", todaySessions(), fs ? fstats.done : 0); pgAdd(b, true);
      if (fs && fstats.hour >= 0) pgAddf("best hour %02d-%02d  streak %u", fstats.hour, (fstats.hour + 1) % 24, streakNow()); else pgAddf("streak %u", streakNow());
      pgAddf("tomorrow goal %s", g); break; }
    default: break;
  }
  pgDraw();
}

// =====================================================================
// MARKETS (symbol pages, then mutual funds, then portfolio)
// =====================================================================
bool havePortfolio() { for (int i = 0; i < NUM_HOLDINGS; i++) if (PORTFOLIO[i].qty > 0) return true; return false; }
int heatPage() { return NUM_SYMBOLS; }
int mfPage() { return NUM_MF > 0 ? NUM_SYMBOLS + 1 : -1; }
int portfolioPage() { return havePortfolio() ? NUM_SYMBOLS + 1 + (NUM_MF > 0 ? 1 : 0) : -1; }
int marketPages() { return NUM_SYMBOLS + 1 + (NUM_MF > 0 ? 1 : 0) + (havePortfolio() ? 1 : 0); }
int symbolIndex(const char *sym) { for (int i = 0; i < NUM_SYMBOLS; i++) if (!strcmp(MARKET_SYMBOLS[i], sym)) return i; return -1; }
float goldInr10g() { int g = symbolIndex("GC=F"), u = symbolIndex("USDINR=X"); if (g < 0 || u < 0 || !quotes[g].valid || !quotes[u].valid) return 0; return quotes[g].price * quotes[u].price / 31.1035f * 10; }
void paintMarket() {
  radar->fillScreen(COL_BG); char b[32];
  int pages = marketPages(); if (marketSel >= pages) marketSel = 0;
  if (marketSel >= NUM_SYMBOLS) {
    bool mk = (arenaOwner == ARENA_MARKET);
    if (marketSel == heatPage()) {
      radar->textC(34, "WATCHLIST", FMB, COL_TEXT_DIM);
      for (int i = 0; i < NUM_HEAT; i++) {
        int x = 30 + (i % 3) * 62, y = 48 + (i / 3) * 38;
        bool v = mk && ARENA_MK->heat[i].valid; float c = 0;
        if (v) { const HeatItem &h = ARENA_MK->heat[i]; c = h.prev > 0 ? (h.price - h.prev) / h.prev * 100 : 0; }
        float k = fabsf(c) > 2.5f ? 1.0f : fabsf(c) / 2.5f;
        uint16_t fill = !v ? rgb(22, 22, 22) : c >= 0 ? rgb(0, (uint8_t)(40 + 120 * k), (uint8_t)(20 + 40 * k)) : rgb((uint8_t)(50 + 150 * k), (uint8_t)(10 + 20 * k), (uint8_t)(10 + 20 * k));
        radar->fillRoundRect(x, y, 58, 34, 5, fill);
        radar->text(x + 5, y + 14, HEAT_NAMES[i], FSR, v ? COL_TEXT : COL_TEXT_DIM);
        if (v) { snprintf(b, sizeof(b), "%+.1f%%", c); radar->text(x + 5, y + 29, b, FS, COL_TEXT); } else radar->text(x + 5, y + 29, "...", FS, COL_TEXT_DIM);
      }
      radar->textC(222, "day change, fetched while open", FSR, rgb(70, 70, 70));
      return;
    }
    if (marketSel == mfPage()) {
      radar->textC(36, "MUTUAL FUNDS", FMB, COL_TEXT_DIM);
      int shown = 0;
      for (int i = 0; i < NUM_MF && i < 4; i++) {
        int y = 72 + i * 40;
        if (!mk || !ARENA_MK->mf[i].valid) { snprintf(b, sizeof(b), "scheme %s", MF_SCHEMES[i]); radar->text(28, y, b, FSR, COL_TEXT_DIM); radar->textR(212, y, "...", FSR, COL_TEXT_DIM); continue; }
        const MfNav &mfi = ARENA_MK->mf[i];
        char nm[26]; strlcpy(nm, mfi.name, sizeof(nm)); int nlen = (int)strlen(nm); while (nlen > 3 && radar->textWidth(FSR, nm) > 184) nm[--nlen] = 0;
        radar->text(28, y, nm, FSR, COL_TEXT);
        fmtPrice(mfi.nav, b, sizeof(b), 2); radar->text(28, y + 18, b, FS, COL_TEXT);
        float c = mfi.prev > 0 ? (mfi.nav - mfi.prev) / mfi.prev * 100 : 0; snprintf(b, sizeof(b), "%+.2f%%", c); radar->textR(212, y + 18, b, FS, c >= 0 ? COL_OK : COL_SELECTED);
        radar->textR(212, y, mfi.date, FSR, rgb(70, 70, 70)); shown++;
      }
      radar->textC(224, "NAV updates nightly (AMFI)", FSR, rgb(70, 70, 70));
    } else {
      radar->textC(36, "PORTFOLIO", FMB, COL_TEXT_DIM);
      float total = 0; bool any = false; int y = 96;
      for (int i = 0; i < NUM_HOLDINGS; i++) {
        if (PORTFOLIO[i].qty <= 0) continue; int s = symbolIndex(PORTFOLIO[i].symbol); if (s < 0 || !quotes[s].valid) continue;
        float pl = PORTFOLIO[i].qty * (quotes[s].price - quotes[s].prevClose); total += pl; any = true;
        if (y < 200) { radar->text(28, y, MARKET_SHORT[s], FSR, COL_TEXT); fmtPrice(fabsf(pl), b, sizeof(b), 0); char c[20]; snprintf(c, sizeof(c), "%sRs %s", pl < 0 ? "-" : "+", b); radar->textR(212, y, c, FS, pl >= 0 ? COL_OK : COL_SELECTED); y += 22; }
      }
      if (any) { fmtPrice(fabsf(total), b, sizeof(b), 0); char c[24]; snprintf(c, sizeof(c), "%s%s", total < 0 ? "-" : "+", b); radar->textC(72, c, FB2, total >= 0 ? COL_OK : COL_SELECTED); radar->textC(214, "Rs, today's change", FSR, COL_TEXT_DIM); }
      else radar->textC(124, "waiting for quotes", FSR, COL_TEXT_DIM);
    }
    return;
  }
  const Quote &q = quotes[marketSel];
  radar->textC(30, MARKET_NAMES[marketSel], FM, COL_TEXT_DIM);
  if (!q.valid) { radar->textC(118, wifiUp ? "fetching quotes" : "no WiFi", FSR, COL_TEXT); return; }
  int dec = q.price > 9999 ? 0 : 2; if (!strcmp(MARKET_SYMBOLS[marketSel], "USDINR=X")) dec = 2;
  fmtPrice(q.price, b, sizeof(b), dec); radar->textC(68, b, FB2, COL_TEXT);
  float chg = q.prevClose > 0 ? (q.price - q.prevClose) / q.prevClose * 100 : 0; bool up = chg >= 0; uint16_t col = up ? COL_OK : COL_SELECTED;
  snprintf(b, sizeof(b), "%+.2f%%", chg); int pw = radar->textWidth(FS, b) + 30;
  radar->fillRoundRect(RADAR_CX - pw / 2, 78, pw, 20, 10, up ? rgb(0, 50, 25) : rgb(60, 15, 15));
  if (up) radar->fillTriangle(RADAR_CX - pw / 2 + 9, 93, RADAR_CX - pw / 2 + 19, 93, RADAR_CX - pw / 2 + 14, 84, col); else radar->fillTriangle(RADAR_CX - pw / 2 + 9, 84, RADAR_CX - pw / 2 + 19, 84, RADAR_CX - pw / 2 + 14, 93, col);
  radar->text(RADAR_CX - pw / 2 + 24, 93, b, FS, col);
  const int x0 = 28, x1 = 212, yTop = 106, yBot = 164;
  bool index = marketSel <= 3;   // NSE session symbol (9:15-15:30) rather than a 24 h one
  bool havePts = (mktDrawSym == marketSel && mktDrawN >= 2);
  if (havePts) {
    const float *pts = mktDrawPts; int qn = mktDrawN;
    float mn = q.prevClose, mx = q.prevClose;
    for (int i = 0; i < qn; i++) { if (pts[i] < mn) mn = pts[i]; if (pts[i] > mx) mx = pts[i]; }
    if (mx - mn < 1e-3f) mx = mn + 1;
    #define Y(v) (yBot - (int)(((v) - mn) / (mx - mn) * (yBot - yTop)))
    int yp = Y(q.prevClose); for (int x = x0; x < x1; x += 6) radar->drawLine(x, yp, x + 2, yp, rgb(90, 90, 90));
    int span = index ? (qn > 75 ? qn : 75) : (qn > 1 ? qn - 1 : 1);   // partial session grows across the width; a full 24 h range fills it
    uint16_t fill = up ? rgb(0, 35, 18) : rgb(45, 10, 10);
    for (int i = 0; i < qn - 1; i++) {
      int xa = x0 + i * (x1 - x0) / span, xb = x0 + (i + 1) * (x1 - x0) / span, ya = Y(pts[i]), yb = Y(pts[i + 1]);
      for (int x = xa; x <= xb; x++) { int y = ya + (yb - ya) * (x - xa) / (xb - xa ? xb - xa : 1); radar->drawLine(x, y, x, yBot, fill); }
      radar->drawLine(xa, ya, xb, yb, col); radar->drawLine(xa, ya + 1, xb, yb + 1, col);
    }
    #undef Y
  } else radar->textC(140, "loading chart...", FSR, rgb(70, 70, 70));
  radar->text(x0, 176, index ? "9:15" : "24h", FSR, rgb(70, 70, 70)); radar->textR(x1, 176, index ? "15:30" : "", FSR, rgb(70, 70, 70));
  struct tm t; bool open = getTime(t) && marketOpenNow(t);
  (void)open;   // v6: open / closed and the time left are on the left OLED
  if (false) {}
  else if (!strcmp(MARKET_SYMBOLS[marketSel], "GC=F") && goldInr10g() > 0) { fmtPrice(goldInr10g(), b, sizeof(b), 0); char c[32]; snprintf(c, sizeof(c), "Rs %s / 10 g", b); radar->textC(176, c, FS, COL_DEP); }
  else if (!strcmp(MARKET_SYMBOLS[marketSel], "BTC-USD") && fng >= 0) { char c[32]; snprintf(c, sizeof(c), "fear & greed %d  %s", fng, fngText); radar->textC(176, c, FS, fng >= 55 ? COL_OK : fng <= 45 ? COL_SELECTED : COL_DEP); }
}
void drawMarketFrame() { if (!staticDirty && millis() - lastHudMs < 1000) return; lastHudMs = millis(); staticDirty = false; present(paintMarket); }

// =====================================================================
// NETWORK
// =====================================================================
void paintNetwork() {
  radar->fillScreen(COL_BG); char b[40];
  radar->textC(28, "NETWORK", FS, COL_TEXT_DIM);
  bool hv = (arenaOwner == ARENA_NETWORK); int netIdx = hv ? ARENA_NET->idx : 0; bool netWrapped = hv && ARENA_NET->wrapped;
  int n = netWrapped ? 120 : netIdx;
  uint16_t last[3] = {netLastGw, netLastWan, netLastDns}; int loss[3] = {0, 0, 0}; uint32_t avg[3] = {0, 0, 0}; int cnt[3] = {0, 0, 0};
  for (int k = 0; k < n; k++) { int i = (netIdx - 1 - k + 120) % 120; for (int m = 0; m < 3; m++) { uint16_t v = ARENA_NET->hist[m][i]; if (v == 0xFFFF) loss[m]++; else { avg[m] += v; cnt[m]++; } } }
  const char *lab[3] = {"GATEWAY", "INTERNET", "DNS"};
  for (int m = 0; m < 3; m++) {
    int x = 40 + m * 54;
    radar->drawRoundRect(x, 36, 50, 46, 6, COL_PANEL);
    radar->text(x + 25 - radar->textWidth(FSR, lab[m]) / 2, 50, lab[m], FSR, COL_TEXT_DIM);
    uint16_t v = last[m]; uint16_t col = v == 0xFFFF ? COL_SELECTED : v < 30 ? COL_OK : v < 100 ? COL_DEP : COL_ARR;
    if (v == 0xFFFF) strcpy(b, "x"); else snprintf(b, sizeof(b), "%u", v);
    radar->text(x + 25 - radar->textWidth(FMB, b) / 2, 76, b, FMB, col);
  }
  snprintf(b, sizeof(b), "ms   loss gw %d%%  wan %d%%   %d min", n ? loss[0] * 100 / n : 0, n ? loss[1] * 100 / n : 0, n * NET_PROBE_MS / 60000); radar->textC(98, b, FSR, COL_TEXT_DIM);
  const int x0 = 30, x1 = 210, yb = 150, h = 40;
  radar->drawLine(x0, yb, x1, yb, rgb(40, 40, 40));
  for (int m = 1; m >= 0; m--) {
    uint16_t col = m == 1 ? COL_OK : COL_ENROUTE_HIGH; int px = -1, py = -1;
    for (int k = 0; k < n; k++) {
      int i = (netIdx - n + k + 120) % 120; uint16_t v = ARENA_NET->hist[m][i];
      int x = x0 + k * (x1 - x0) / 119;
      if (v == 0xFFFF) { radar->drawLine(x, yb - h, x, yb, dim565(COL_SELECTED, 0.6f)); px = -1; continue; }
      float lv = log10f(v < 1 ? 1 : v) / log10f(1500.0f); int y = yb - (int)(lv * h);
      if (px >= 0) radar->drawLine(px, py, x, y, col); px = x; py = y;
    }
  }
  radar->text(x0, 164, "1 ms", FSR, rgb(60, 60, 60)); radar->textR(x1, 164, "1.5 s", FSR, rgb(60, 60, 60));
  radar->text(x0, 110, "green internet, blue gateway", FSR, rgb(60, 60, 60));
  { char node[10]; nodeStr(node, sizeof(node));
    if (roamCount) { time_t rt = roamLog[0].t; struct tm rtm; localtime_r(&rt, &rtm); snprintf(b, sizeof(b), "%s %ddBm roamed %02d:%02d", node, WiFi.RSSI(), rtm.tm_hour, rtm.tm_min); }
    else snprintf(b, sizeof(b), "node %s  %d dBm  ch %d", node, WiFi.RSSI(), WiFi.channel());
    radar->text(32, 182, b, FSR, COL_TEXT_DIM); }
  if (outageCount > 0) { time_t st = outages[0].start; struct tm ot; localtime_r(&st, &ot); snprintf(b, sizeof(b), "last outage %02d:%02d %s  %u min", ot.tm_hour, ot.tm_min, DNL[ot.tm_wday], outages[0].minutes); radar->text(32, 198, b, FSR, rgb(200, 120, 80)); }
  else radar->text(32, 198, "no internet outages logged", FSR, rgb(80, 80, 80));
  snprintf(b, sizeof(b), "IP %s", WiFi.localIP().toString().c_str()); radar->textC(214, b, FSR, rgb(80, 80, 80));
}
void drawNetworkFrame() { if (!staticDirty && !netDirty && millis() - lastHudMs < 2000) return; netDirty = false; lastHudMs = millis(); staticDirty = false; present(paintNetwork); }

// =====================================================================
// STATS: 0 today, 1 spotter
// =====================================================================
void paintStats() {
  radar->fillScreen(COL_BG); char b[48];
  if (subPage == 0) {
    radar->textC(28, "TRAFFIC TODAY", FS, COL_TEXT_DIM);
    snprintf(b, sizeof(b), "%d", seenToday); radar->textC(66, b, FB2, COL_TEXT);
    snprintf(b, sizeof(b), "%d arr  %d dep  %d over", seenArr, seenDep, seenEnr); radar->textC(84, b, FSR, COL_TEXT_DIM);
    int maxH = 1; for (int h = 0; h < 24; h++) { int s = hourArr[h] + hourDep[h] + hourEnr[h]; if (s > maxH) maxH = s; }
    const int x0 = 30, y1 = 164, H = 60;
    for (int h = 0; h < 24; h++) {
      int x = x0 + h * 7.5f; int y = y1;
      int va = hourArr[h] * H / maxH, vd = hourDep[h] * H / maxH, ve = hourEnr[h] * H / maxH;
      if (va) { radar->fillRect(x, y - va, 6, va, COL_ARR); y -= va; }
      if (vd) { radar->fillRect(x, y - vd, 6, vd, COL_DEP); y -= vd; }
      if (ve) { radar->fillRect(x, y - ve, 6, ve, COL_ENROUTE_HIGH); }
    }
    radar->drawLine(x0, y1 + 1, x0 + 180, y1 + 1, rgb(40, 40, 40));
    for (int h = 0; h <= 23; h += 6) { snprintf(b, sizeof(b), "%02d", h); radar->text(x0 + h * 7.5f, y1 + 14, b, FSR, rgb(70, 70, 70)); }
    int busiest = 0; for (int h = 1; h < 24; h++) if (hourArr[h] + hourDep[h] + hourEnr[h] > hourArr[busiest] + hourDep[busiest] + hourEnr[busiest]) busiest = h;
    if (recHeavyCs[0]) { char pcs[12]; prettyCallsign(recHeavyCs, pcs, sizeof(pcs)); snprintf(b, sizeof(b), "busiest %02d:00   heavy %s %s", busiest, recHeavyType, pcs); } else snprintf(b, sizeof(b), "busiest hour %02d:00", busiest);
    radar->textC(200, b, FSR, COL_TEXT_DIM);
    if (yesterday.seen) snprintf(b, sizeof(b), "yesterday %d   high FL%03d  %.0f kt", yesterday.seen, (int)(recHighFt / 100), recFastKt); else snprintf(b, sizeof(b), "highest FL%03d  fastest %.0f kt", (int)(recHighFt / 100), recFastKt);
    radar->textC(216, b, FSR, rgb(80, 80, 80));
  } else if (subPage == 2) {
    radar->textC(28, "RECENT ALERTS", FS, COL_TEXT_DIM);
    if (alertLogCount == 0) radar->textC(124, "nothing yet", FSR, COL_TEXT_DIM);
    for (int i = 0; i < alertLogCount && i < 6; i++) {
      int y = 54 + i * 28; time_t at = alertLog[i].t; struct tm lt; localtime_r(&at, &lt);
      snprintf(b, sizeof(b), "%02d:%02d", lt.tm_hour, lt.tm_min); radar->text(30, y, b, FSR, rgb(90, 90, 90));
      radar->text(70, y, alertLog[i].l1, FS, COL_TEXT); radar->text(70, y + 13, alertLog[i].l2, FSR, COL_TEXT_DIM);
    }
  } else {
    radar->textC(28, "SPOTTER LOG", FS, COL_TEXT_DIM);
    int na = 0; for (int i = 0; i < 64; i++) if (airlinesSeen & (1ULL << i)) na++;
    snprintf(b, sizeof(b), "%d airlines  %d types today", na, typesSeenCount); radar->textC(50, b, FSR, COL_TEXT);
    int y = 72, x = 30; char line[48] = "";
    for (int i = 0; i < NUM_AIRLINES && y < 120; i++) if (airlinesSeen & (1ULL << (i & 63))) {
      char tag[8]; snprintf(tag, sizeof(tag), "%s ", AIRLINES[i].iata[0] ? AIRLINES[i].iata : AIRLINES[i].icao);
      if (radar->textWidth(FS, line) + radar->textWidth(FS, tag) > 180) { radar->text(x, y, line, FS, COL_ENROUTE_HIGH); y += 16; line[0] = 0; }
      strlcat(line, tag, sizeof(line));
    }
    if (line[0] && y < 136) { radar->text(x, y, line, FS, COL_ENROUTE_HIGH); y += 16; }
    y = (y < 140) ? 140 : y + 6; line[0] = 0;
    for (int i = 0; i < typesSeenCount && y < 216; i++) {
      char tag[8]; snprintf(tag, sizeof(tag), "%s ", typesSeen[i]);
      if (radar->textWidth(FSR, line) + radar->textWidth(FSR, tag) > 180) { radar->text(x, y, line, FSR, COL_TEXT_DIM); y += 15; line[0] = 0; }
      strlcat(line, tag, sizeof(line));
    }
    if (line[0] && y < 222) radar->text(x, y, line, FSR, COL_TEXT_DIM);
  }
}
void drawStatsFrame() { if (!staticDirty && millis() - lastHudMs < 5000) return; lastHudMs = millis(); staticDirty = false; present(paintStats); }

// =====================================================================
// NEWS
// =====================================================================
void paintNews() {   // v6: the current headline, large; the next three are on the right dial
  radar->fillScreen(COL_BG);
  if (newsCount == 0) { radar->textC(124, wifiUp ? "fetching feeds..." : "no WiFi", FSR, COL_TEXT_DIM); return; }
  if (newsScroll >= newsCount) newsScroll = 0; if (newsScroll < 0) newsScroll = newsCount - 1;
  const Headline &h = news[newsScroll];
  const char *src = strstr(NEWS_FEEDS[h.feed], "bbc") ? "BBC" : strstr(NEWS_FEEDS[h.feed], "thehindu") ? "THE HINDU" : "NEWS";
  radar->textC(40, src, FS, COL_ARR);
  bool big = strlen(h.text) <= 80; const GFXfont *f = big ? FM : FS; int lh = big ? 24 : 18;
  char lines[7][40]; int nl = wrapText(h.text, f, 184, lines, big ? 6 : 7);
  int y = 124 - nl * lh / 2 + lh / 2;
  for (int l = 0; l < nl; l++) { radar->textC(y, lines[l], f, COL_TEXT); y += lh; }
}
void drawNewsFrame() { if (!staticDirty && millis() - lastHudMs < 5000) return; lastHudMs = millis(); staticDirty = false; present(paintNews); }

// =====================================================================
// CRICKET
// =====================================================================
void paintCricket() {
  radar->fillScreen(COL_BG); char lines[3][40];
  radar->textC(28, "CRICKET", FS, COL_TEXT_DIM);
  bool hv = (arenaOwner == ARENA_CRICKET); int n = 0; if (hv) for (int i = 0; i < 2; i++) if (ARENA_CR->m[i].valid) n++;
  if (n == 0) { radar->textC(124, !CRICAPI_KEY[0] ? "set CRICAPI_KEY" : (hv && ARENA_CR->ms) ? "no live match" : "fetching...", FSR, COL_TEXT_DIM); return; }
  if (subPage >= n) subPage = 0;
  const CricketMatch &m = ARENA_CR->m[subPage];
  int nl = wrapText(m.name, FS, 180, lines, 2); int y = 56;
  for (int l = 0; l < nl; l++) { radar->textC(y, lines[l], FS, COL_TEXT); y += 16; }
  if (m.live) { radar->fillCircle(40, 50, 4, COL_SELECTED); radar->text(48, 54, "LIVE", FSR, COL_SELECTED); }
  y = 110;
  for (int s = 0; s < 2; s++) if (m.score[s][0]) { radar->textC(y, m.score[s], FM, s == 0 ? COL_TEXT : rgb(170, 170, 170)); y += 30; }
  nl = wrapText(m.status, FSR, 180, lines, 2); y = 186;
  for (int l = 0; l < nl; l++) { radar->textC(y, lines[l], FSR, COL_DEP); y += 15; }
  if (n > 1) { char b[16]; snprintf(b, sizeof(b), "%d/%d  rotate", subPage + 1, n); radar->textC(224, b, FSR, rgb(70, 70, 70)); }
}
void drawCricketFrame() { if (!staticDirty && millis() - lastHudMs < 5000) return; lastHudMs = millis(); staticDirty = false; present(paintCricket); }

// =====================================================================
// SYSTEM
// =====================================================================
void paintSystem() {
  radar->fillScreen(COL_BG); char b[48];
  if (subPage == 0) {
    radar->textC(28, "SYSTEM", FS, COL_TEXT_DIM);
    unsigned long up = (millis() - bootMs) / 1000;
    snprintf(b, sizeof(b), "v%s  %s", FW_VERSION, __DATE__); radar->textC(48, b, FSR, COL_TEXT);
    const char *lab[8] = {"heap free", "heap min", "fps", "uptime", "wifi", "ip", "fetches", "failed"}; char val[8][24];
    snprintf(val[0], 24, "%u KB", ESP.getFreeHeap() / 1024); snprintf(val[1], 24, "%u KB", ESP.getMinFreeHeap() / 1024); snprintf(val[2], 24, "%d", fpsShown);
    snprintf(val[3], 24, "%lud %02lu:%02lu", up / 86400, (up / 3600) % 24, (up / 60) % 60); snprintf(val[4], 24, "%d dBm ch%d", WiFi.RSSI(), WiFi.channel());
    strlcpy(val[5], WiFi.localIP().toString().c_str(), 24); snprintf(val[6], 24, "%lu", (unsigned long)fetchCount); snprintf(val[7], 24, "%lu", (unsigned long)fetchFailCount);
    for (int i = 0; i < 8; i++) { int y = 70 + i * 18; radar->text(44, y, lab[i], FSR, COL_TEXT_DIM); radar->textR(200, y, val[i], FS, COL_TEXT); }
  } else if (subPage == 1) {
    radar->textC(28, "SOURCES", FS, COL_TEXT_DIM);
    for (int s = 0; s < NUM_SOURCES; s++) {
      int y = 56 + s * 20; bool act = (activeSource == s);
      radar->text(36, y, sourceName(s), FS, act ? COL_OK : COL_TEXT);
      if (srcBackedOff(s)) snprintf(b, sizeof(b), "backoff %lus", (srcBackoffUntil[s] - millis()) / 1000);
      else if (s == 4 && !localUrl[0]) strcpy(b, "not set");
      else if (srcLastTry[s]) snprintf(b, sizeof(b), "%lus ago%s", (millis() - srcLastTry[s]) / 1000, act ? "  active" : ""); else strcpy(b, "untried");
      radar->textR(206, y, b, FSR, act ? COL_OK : COL_TEXT_DIM);
    }
    int nr = 0; for (int i = 0; i < MAX_ROUTES; i++) if (routesUI[i].valid) nr++;
    snprintf(b, sizeof(b), "routes cached %d   trails %d", nr, planeCount); radar->textC(176, b, FSR, COL_TEXT_DIM);
    int qok = 0; for (int i = 0; i < NUM_SYMBOLS; i++) if (quotes[i].valid) qok++;
    snprintf(b, sizeof(b), "metar %s   news %d   quotes ok %d/%d", metar.valid ? "ok" : "none", newsCount, qok, NUM_SYMBOLS);
    radar->textC(194, b, FSR, COL_TEXT_DIM);
    radar->textC(224, "deskradar6.local/log for the log", FSR, rgb(70, 70, 70));
  } else {
    radar->textC(28, "HEALTH 24 H", FS, COL_TEXT_DIM);
    const int x0 = 34, x1 = 206;
    int hmin = 9999, hmax = 0, fmax = 1; for (int i = 0; i < heapHistN; i++) { if (heapHist[i] < hmin) hmin = heapHist[i]; if (heapHist[i] > hmax) hmax = heapHist[i]; if (fpsHist[i] > fmax) fmax = fpsHist[i]; }
    if (heapHistN < 2) radar->textC(100, "collecting (30 min samples)", FSR, COL_TEXT_DIM);
    else {
      int yb = 112, h = 54; radar->drawLine(x0, yb, x1, yb, rgb(40, 40, 40)); int lo = hmin > 10 ? hmin - 10 : 0, span = hmax + 10 - lo; if (span < 20) span = 20;
      int px = -1, py = -1; for (int i = 0; i < heapHistN; i++) { int x = x0 + i * (x1 - x0) / 47, y = yb - (heapHist[i] - lo) * h / span; if (px >= 0) radar->drawLine(px, py, x, y, COL_OK); px = x; py = y; }
      radar->text(x0, 50, "heap KB", FSR, rgb(60, 60, 60)); snprintf(b, sizeof(b), "min %d  max %d", hmin, hmax); radar->textR(x1, 50, b, FSR, COL_TEXT_DIM);
      yb = 176; h = 40; radar->drawLine(x0, yb, x1, yb, rgb(40, 40, 40)); px = -1;
      for (int i = 0; i < heapHistN; i++) { int x = x0 + i * (x1 - x0) / 47, y = yb - fpsHist[i] * h / fmax; if (px >= 0) radar->drawLine(px, py, x, y, COL_ENROUTE_HIGH); px = x; py = y; }
      radar->text(x0, 130, "fps", FSR, rgb(60, 60, 60)); snprintf(b, sizeof(b), "now %d", fpsShown); radar->textR(x1, 130, b, FSR, COL_TEXT_DIM);
    }
    const char *rr = resetReasonCode == 1 ? "power on" : resetReasonCode == 3 ? "software" : resetReasonCode == 4 ? "panic" : (resetReasonCode == 5 || resetReasonCode == 6 || resetReasonCode == 7) ? "watchdog" : resetReasonCode == 8 ? "deep sleep" : resetReasonCode == 9 ? "brownout" : "other";
    snprintf(b, sizeof(b), "last reset: %s   outages %d", rr, outageCount); radar->textC(198, b, FSR, resetReasonCode == 4 || resetReasonCode == 9 ? COL_SELECTED : COL_TEXT_DIM);
    snprintf(b, sizeof(b), "min heap %uK  roams %d", ESP.getMinFreeHeap() / 1024, roamCount); radar->textC(216, b, FSR, COL_TEXT_DIM);
  }
}
void drawSystemFrame() { if (!staticDirty && millis() - lastHudMs < 2000) return; lastHudMs = millis(); staticDirty = false; present(paintSystem); }

// =====================================================================
// BRIGHTNESS, LOCATION, MENU
// =====================================================================
void paintBright() {
  radar->fillScreen(COL_BG); char b[12];
  radar->textC(34, "DISPLAY", FS, COL_TEXT_DIM);
  for (int i = 0; i < 2; i++) {
    int y = 62 + i * 46; bool sel = (brightSel == i); int v = i == 0 ? tftBright : oledBright;
    radar->text(40, y, i == 0 ? "ROUND" : "OLED", FM, sel ? COL_TEXT : COL_TEXT_DIM);
    snprintf(b, sizeof(b), "%d%%", v); radar->textR(200, y, b, FM, sel ? COL_TEXT : COL_TEXT_DIM);
    radar->fillRoundRect(40, y + 10, 160, 14, 7, rgb(35, 35, 35));
    radar->fillRoundRect(40, y + 10, 16 + 144 * v / 100, 14, 7, sel ? COL_OK : rgb(80, 80, 80));
  }
  { int y = 154; bool sel = (brightSel == 2);
    radar->text(40, y, "THEME", FM, sel ? COL_TEXT : COL_TEXT_DIM);
    radar->textR(200, y, THEME_NAMES[theme], FM, sel ? COL_SWEEP : COL_TEXT_DIM);
    const uint16_t sw[4] = {rgb(0, 200, 90), rgb(255, 170, 40), rgb(90, 180, 255), rgb(230, 230, 230)};
    for (int k = 0; k < 4; k++) { int x = 52 + k * 36; radar->fillRoundRect(x, y + 8, 28, 10, 4, sw[k]); if (k == theme) radar->drawRoundRect(x - 2, y + 6, 32, 14, 5, COL_TEXT); }
  }
  { int y = 200; bool sel = (brightSel == 3), sel2 = (brightSel == 4);
    radar->text(36, y, "SWEEP", FS, sel ? COL_TEXT : COL_TEXT_DIM);
    radar->text(96, y, sweepOn ? "on" : "off", FS, sel ? (sweepOn ? COL_OK : COL_SELECTED) : COL_TEXT_DIM);
    radar->text(132, y, "TEXT", FS, sel2 ? COL_TEXT : COL_TEXT_DIM);
    radar->textR(206, y, OLED_SCALE_NAMES[oledScale], FS, sel2 ? COL_SWEEP : COL_TEXT_DIM);
  }
}
void drawBrightFrame() { if (!staticDirty) return; staticDirty = false; present(paintBright); }
void paintLocation() {
  radar->fillScreen(COL_BG); char b[40];
  radar->textC(36, locTarget ? "WEATHER LOCATION" : "RADAR HOME", FS, COL_TEXT_DIM);
  if (listSel >= locCount()) listSel = 0;
  int first = listSel > 3 ? listSel - 3 : 0;
  for (int r = 0; r < 5 && first + r < locCount(); r++) {
    int row = first + r, i = locAirport(row), y = 70 + r * 28; bool hi = (row == listSel);
    if (hi) { radar->fillRoundRect(30, y - 19, 180, 26, 6, rgb(10, 30, 20)); radar->drawRoundRect(30, y - 19, 180, 26, 6, COL_SWEEP); }
    if (i < 0) snprintf(b, sizeof(b), "HOME  %s", WX_HOME_NAME); else snprintf(b, sizeof(b), "%s  %s", AIRPORTS[i].iata, AIRPORTS[i].name);
    radar->text(40, y, b, FS, hi ? COL_TEXT : COL_TEXT_DIM);
    bool isRadar = (i >= 0 && i == homeIndex);
    bool isWx = (i < 0 && weatherIndex == -2) || (i >= 0 && (i == weatherIndex || (weatherIndex == -1 && i == homeIndex)));
    if (isRadar || isWx) radar->textR(202, y, isRadar && isWx ? "radar wx" : isRadar ? "radar" : "wx", FSR, isWx ? COL_ENROUTE_HIGH : COL_OK);
  }
}
void drawLocationFrame() { if (!staticDirty) return; staticDirty = false; present(paintLocation); }

// 6.16.0: apps the owner dropped. The enum keeps its numbering (NVS), these never enter the menu or the dials.
bool appRetired(int a) { return a == APP_REPLAY || a == APP_NETWORK || a == APP_STATS || a == APP_CRICKET || a == APP_F1 || a == APP_SYSTEM || a == APP_BRIGHT || a == APP_LOCATION; }
int menuItems[APP_COUNT]; int menuCount = 0;
void buildMenu() {
  menuCount = 0;
  for (int a = 0; a < APP_COUNT; a++) {
    if (a == APP_CRICKET && !CRICAPI_KEY[0]) continue;
    if (a == APP_F1 && !F1_ENABLED) continue;
    if (a == APP_REPLAY && !REPLAY_ENABLED) continue;
    if (a == APP_LOCATION && NUM_AIRPORTS < 2) continue;
    if (appRetired(a)) continue;
    if (a == APP_AGENDA || a == APP_MEDIA || a == APP_SETTINGS) continue;   // v6: placed by hand below
    if (!(appMask & (1u << a))) continue;                                     // hidden in SETTINGS > APPS
    menuItems[menuCount++] = a;
    if (a == APP_CLOCK && (appMask & (1u << APP_AGENDA))) menuItems[menuCount++] = APP_AGENDA;
    if (a == APP_NEWS && (appMask & (1u << APP_MEDIA))) menuItems[menuCount++] = APP_MEDIA;
  }
  menuItems[menuCount++] = APP_SETTINGS;   // 6.14.1: always last, never tied to another app's visibility
  if (menuSel >= menuCount) menuSel = 0;
}
void paintMenu() {
  radar->fillScreen(COL_MENU_BG);
  for (int k = -2; k <= 2; k++) {
    int i = menuSel + k; if (i < 0 || i >= menuCount) continue;
    int y = 128 + k * 32; bool s = (k == 0);
    if (s) { radar->fillRoundRect(44, y - 22, 152, 30, 8, rgb(20, 40, 30)); radar->drawRoundRect(44, y - 22, 152, 30, 8, COL_SWEEP); }
    radar->textC(y, APP_NAMES[menuItems[i]], s ? FMB : FM, s ? COL_TEXT : (abs(k) == 1 ? COL_TEXT_DIM : rgb(60, 60, 60)));
  }
  char b[12]; snprintf(b, sizeof(b), "%d / %d", menuSel + 1, menuCount); radar->textC(226, b, FSR, rgb(70, 70, 70));
}
void drawMenu() { present(paintMenu); }

// =====================================================================
// OLED 128x64. Yellow band (rows 0-15) = big title + two small lines.
// Blue area = 3 lines of 13 px text (F_SM13R, the round display's font), ~18 characters each.
// Screens with more than 3 lines are split into pages that auto-advance every OLED_PAGE_MS;
// a dot column on the right edge shows the page. Lines wider than the screen scroll sideways once.
// =====================================================================
#define OLED_PAGE_MS 4500
#define OLED_MAXL 9
#define OLED_LINE_W 123                       // text width available (dot column at 125..127)
#define OLED_MARQUEE_MS_PER_PX 25             // marquee speed
#define OLED_MARQUEE_REST 1500                // rest before and after a scroll
static char pgL[OLED_MAXL][40]; static int pgN = 0; static uint8_t pgBold[OLED_MAXL];
int tickerPx = 0;
bool tickerApp() { return currentApp == APP_CLOCK || currentApp == APP_WEATHER || currentApp == APP_MARKET || currentApp == APP_STATS || currentApp == APP_NETWORK || currentApp == APP_F1; }

// Body text size table (oledScale): font, GFX scale, lines per page, baselines
static const OledScale OSC[4] = {
  {&F_SM13R,  &F_SM13,   1, 3, {29, 44, 59}},   // 1x   13 px, ~18 chars
  {&F_MID22,  &F_MID22B, 1, 2, {36, 60, 0}},    // 1.5x 22 px, ~12 chars
  {&F_SM13R,  &F_SM13,   2, 1, {50, 0, 0}},     // 2x   26 px, ~9 chars
  {&F_SM13R,  &F_SM13,   3, 1, {56, 0, 0}},     // 3x   39 px, ~6 chars
};
const OledScale &osc() { return OSC[oledScale < 0 || oledScale > 3 ? 0 : oledScale]; }
int oledLines() { return osc().lines; }

void oledRow(int y, const char *s) { infoDisplay.setFont(); infoDisplay.setTextSize(1); infoDisplay.setCursor(0, y); infoDisplay.print(s); }
void oledHead(const char *big, const char *r1, const char *r2) {
  infoDisplay.setFont(); infoDisplay.setTextSize(2); infoDisplay.setCursor(0, 0); infoDisplay.print(big);
  int x = 12 * (int)strlen(big) + 4; int maxc = (128 - x) / 6; if (maxc < 0) maxc = 0;   // never wrap into the body
  infoDisplay.setTextSize(1); char t[22];
  if (r1) { strlcpy(t, r1, sizeof(t)); if ((int)strlen(t) > maxc) t[maxc] = 0; infoDisplay.setCursor(x, 0); infoDisplay.print(t); }
  if (r2) { strlcpy(t, r2, sizeof(t)); if ((int)strlen(t) > maxc) t[maxc] = 0; infoDisplay.setCursor(x, 8); infoDisplay.print(t); }
}
void oledTimeHead(const char *r1, const char *r2) {   // v6: no time here (the right OLED is the clock), two short lines
  oledHead("", r1, r2);
}
void oledUseFont(bool bold) { const OledScale &o = osc(); infoDisplay.setFont(bold ? o.bold : o.reg); infoDisplay.setTextSize(o.scale); }
int oledTextW(const char *s, bool bold) {
  oledUseFont(bold); int16_t x1, y1; uint16_t w, h;
  infoDisplay.getTextBounds(s, 0, 40, &x1, &y1, &w, &h); infoDisplay.setFont(); infoDisplay.setTextSize(1); return (int)w + (int)x1;
}
// how long a page needs so that its widest line can scroll fully once (0 if everything fits)
unsigned long oledMarqueeMs(const char *s, bool bold) {
  int w = oledTextW(s, bold); if (w <= OLED_LINE_W) return 0;
  return OLED_MARQUEE_REST + (unsigned long)(w - OLED_LINE_W) * OLED_MARQUEE_MS_PER_PX + OLED_MARQUEE_REST;
}
// page state: pages advance when the current page has been shown for max(OLED_PAGE_MS, its marquee time)
static int pgCur = 0; static unsigned long pgStart = 0; static int pgLastApp = -99; static bool pgLastMenu = false;
// one body line at row; wider than the screen -> rests, scrolls once to the end (phase from the page start), rests
void oledLineB(int row, const char *s, bool bold) {
  if (!s || !s[0]) return;
  const OledScale &o = osc(); if (row >= o.lines) return;
  int w = oledTextW(s, bold); int off = 0;
  if (w > OLED_LINE_W) {
    int over = w - OLED_LINE_W; unsigned long ph = millis() - pgStart;
    if (ph > OLED_MARQUEE_REST) { off = (int)((ph - OLED_MARQUEE_REST) / OLED_MARQUEE_MS_PER_PX); if (off > over) off = over; }
  }
  oledUseFont(bold); infoDisplay.setTextWrap(false);
  infoDisplay.setCursor(-off, o.base[row]); infoDisplay.print(s);
  infoDisplay.setFont(); infoDisplay.setTextSize(1);
}
void pgReset() { pgN = 0; }
void pgAdd(const char *s, bool bold) { if (pgN < OLED_MAXL && s) { strlcpy(pgL[pgN], s, 40); pgBold[pgN] = bold; pgN++; } }
void pgAddf(const char *fmt, ...) { char b[40]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap); pgAdd(b, false); }
// word-wrap s into body lines (at most maxLines)
void pgWrap(const char *s, int maxLines, bool bold) {
  char line[40] = ""; int lines = 0; const char *p = s;
  while (*p && lines < maxLines) {
    while (*p == ' ') p++;
    const char *e = p; while (*e && *e != ' ') e++;
    char word[40]; int wl = e - p; if (wl > 38) wl = 38; memcpy(word, p, wl); word[wl] = 0;
    char trial[40]; if (line[0]) snprintf(trial, sizeof(trial), "%s %s", line, word); else strlcpy(trial, word, sizeof(trial));
    if (oledTextW(trial, bold) <= OLED_LINE_W || !line[0]) strlcpy(line, trial, sizeof(line));
    else { pgAdd(line, bold); lines++; strlcpy(line, word, sizeof(line)); }
    p = e;
  }
  if (line[0] && lines < maxLines) pgAdd(line, bold);
}
int pgPages() { int L = oledLines(); return (pgN + L - 1) / L; }
void pgDots(int n, int cur) {
  infoDisplay.fillRect(124, 16, 4, 48, SSD1306_BLACK);
  for (int k = 0; k < n && k < 8; k++) { infoDisplay.drawRect(125, 19 + k * 6, 3, 3, SSD1306_WHITE); if (k == cur) infoDisplay.fillRect(125, 19 + k * 6, 3, 3, SSD1306_WHITE); }
}
// draw the current page. extraFront = custom pages the caller draws before the text pages; returns the custom page index
// (0..extraFront-1) when one of those is due, or -1 when a text page was drawn. startRow leaves the first rows free.
int pgDraw(int extraFront, int startRow) {
  int L = oledLines(); int perPage = L - startRow; if (perPage < 1) perPage = 1;
  int tp = (pgN + perPage - 1) / perPage; int n = extraFront + tp; if (n <= 0) return -1;
  int appKey = menuOpen ? -1 : currentApp;
  if (appKey != pgLastApp || menuOpen != pgLastMenu) { pgLastApp = appKey; pgLastMenu = menuOpen; pgCur = 0; pgStart = millis(); }
  if (pgCur >= n) pgCur = 0;
  // page duration: at least OLED_PAGE_MS, longer if a line on this page has to scroll
  unsigned long dur = OLED_PAGE_MS;
  if (pgCur >= extraFront) { int page = pgCur - extraFront; for (int r = 0; r < perPage; r++) { int i = page * perPage + r; if (i >= pgN) break; unsigned long m = oledMarqueeMs(pgL[i], pgBold[i]); if (m > dur) dur = m; } }
  if (n > 1 && millis() - pgStart > dur) { pgCur = (pgCur + 1) % n; pgStart = millis(); }
  if (pgCur < extraFront) { if (n > 1) pgDots(n, pgCur); return pgCur; }
  int page = pgCur - extraFront;
  for (int r = 0; r < perPage; r++) { int i = page * perPage + r; if (i >= pgN) break; oledLineB(startRow + r, pgL[i], pgBold[i]); }
  if (n > 1) pgDots(n, pgCur);
  return -1;
}
// news as pages: one headline wrapped over up to 3 lines, next headline on the next page
void pgNews(int maxHeadlines) {
  if (newsCount == 0) return;
  int base = (int)((millis() / 60000UL) % newsCount);   // headline set rotates every minute
  for (int k = 0; k < maxHeadlines && k < newsCount; k++) {
    int before = pgN; pgWrap(news[(base + k) % newsCount].text, 3);
    while ((pgN - before) % oledLines() && pgN < OLED_MAXL) pgAdd("");      // pad so every headline starts a fresh page
  }
}
const char *APP_DESC[APP_COUNT] = {
      "Live ADS-B traffic, arrivals, departures",
      "Airport board: next arrivals, departures",
      "Side view of the approaches, glideslope",
      "Last 48 min of tracks, knob scrubs time",
      "11 faces, daylight, rain ring, alarm",
      "Now, 3-day, 24 h history, AQI, quakes",
      "Focus and meeting timer, breathing, history",
      "NIFTY, SENSEX, gold, MF NAVs, portfolio",
      "Deco, internet, DNS latency, outages",
      "Traffic by hour, records, spotter log",
      "RSS headlines",
      "Live scores for your team",
      "Next race countdown and last podium",
      "Heap, fps, uptime, sources, health",
      "Both screens and theme from the knob",
      "Radar home, hold: weather location",
      "Your calendar: today, countdown, free time",
      "Sound, meetings, display, radar, apps, system",
      "Now playing on the Mac, volume on the knob"};
// v6 left OLED kit: three lines in the blue area (top rows kept empty), a label line in the yellow band
void o6Head(const char *s) { infoDisplay.setFont(); infoDisplay.setTextSize(1); infoDisplay.setCursor(0, 0); infoDisplay.print(s); }   // 5x7 line at the very top
void o6Line(int row, const char *l, const char *r) {
  int y = 17 + row * 13; infoDisplay.setFont(&F_SM13); infoDisplay.setTextSize(1);   // rows 0-3 are visible (owner, 12 Oct): 17 / 30 / 43
  if (l) { infoDisplay.setCursor(0, y); infoDisplay.print(l); }
  if (r) { int16_t x1, y1; uint16_t w, h; infoDisplay.getTextBounds(r, 0, y, &x1, &y1, &w, &h); infoDisplay.setCursor(127 - (int)w - x1, y); infoDisplay.print(r); }
  infoDisplay.setFont();
}
void o6Band(const char *l, const char *r) { int y = 61; infoDisplay.setFont(&F_SM13);
  if (l) { infoDisplay.setCursor(0, y); infoDisplay.print(l); }
  if (r) { int16_t x1, y1; uint16_t w, h; infoDisplay.getTextBounds(r, 0, y, &x1, &y1, &w, &h); infoDisplay.setCursor(127 - (int)w - x1, y); infoDisplay.print(r); }
  infoDisplay.setFont(); }
bool drawInfoV6();
void drawInfo() {
  if (sleeping) return;
  infoDisplay.clearDisplay(); infoDisplay.setTextColor(SSD1306_WHITE); infoDisplay.setTextSize(1); infoDisplay.setTextWrap(false); infoDisplay.invertDisplay(false);
  char b[40]; struct tm t; bool haveT = getTime(t);
  pgReset();

  if (alarmRinging) {
    oledHead("ALARM", haveT ? (snprintf(b, sizeof(b), "%02d:%02d", t.tm_hour, t.tm_min), b) : "", "");
    pgAdd("click: snooze 5 min", true); pgAdd("hold: off"); pgDraw();
    oledPush(0); return;
  }
  if (ovActive()) { drawOverlayInfo(); oledPush(0); return; }
  if (toastActive()) {   // alert banner
    if (strlen(toast.l1) <= 10) oledHead(toast.l1, haveT ? (snprintf(b, sizeof(b), "%02d:%02d", t.tm_hour, t.tm_min), b) : "", NULL);
    else { oledTimeHead("", ""); pgAdd(toast.l1, true); }
    pgWrap(toast.l2, 2);
    if (toast.icao[0]) pgAdd("click: select it");
    pgDraw(); oledPush(0); return;
  }
  if (navActive()) { drawNavOled(0); oledPush(0); return; }   // 6.14.0: knob legend
  if (menuOpen) {

    snprintf(b, sizeof(b), "%d of %d", menuSel + 1, menuCount);   // v6: name + description are on the right dial
    o6Line(0, "turn: move", NULL); o6Line(1, "left click: open", NULL); o6Band("MENU", b);
    oledPush(0); return;
  }
  if (!wifiUp && currentApp != APP_POMODORO && currentApp != APP_BRIGHT) {
    oledTimeHead("WiFi down", "");
    pgAdd("reconnecting...", true); pgAddf("reason %d", lastDiscReason); pgAdd(WIFI_SSID);
    pgDraw(); oledPush(0); return;
  }

  if (drawInfoV6()) { oledPush(0); return; }
  switch (currentApp) {
    case APP_CLOCK: {
      static const char *DAYL[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
      static const char *MONL[] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
      char d[14] = "", w[12] = "";
      if (haveT) snprintf(d, sizeof(d), "%s %02d %s", DNL[t.tm_wday], t.tm_mday, MN[t.tm_mon]);
      if (clock12 && haveT) strlcpy(w, ampmStr(t.tm_hour), sizeof(w)); else if (wx.valid) { snprintf(w, sizeof(w), "%.0fC %s", wx.temp, wmoShort(wx.wcode)); w[10] = 0; }
      oledTimeHead(d, w);
      if (haveT) { pgAddf("%s %d %s %d", DAYL[t.tm_wday], t.tm_mday, MONL[t.tm_mon], t.tm_year + 1900); pgBold[0] = 1; }
      if (wx.sunriseMin >= 0) { char r[8], st[8]; fmtHM(r, sizeof(r), wx.sunriseMin / 60, wx.sunriseMin % 60); fmtHM(st, sizeof(st), wx.sunsetMin / 60, wx.sunsetMin % 60); pgAddf("Rise %s  Set %s", r, st); }
      if (alarmOn) { char a[10]; fmtHMA(a, sizeof(a), alarmMin / 60, alarmMin % 60); pgAddf("Alarm %s", a); } else pgAdd("no alarm set");
      if (haveT) { int wk = (t.tm_yday + 7 - ((t.tm_wday + 6) % 7)) / 7; pgAddf("week %d  day %d", wk, t.tm_yday + 1); }
      { zoneTick(); char z[40] = ""; int zn = 0;
        for (int i = 0; i < NUM_ZONES && i < 2; i++) {
          char hm[10]; fmtHMA(hm, sizeof(hm), zoneT[i].hour, zoneT[i].min); char nm[6]; strlcpy(nm, ZONES[i].name, 4); nm[3] = 0;
          zn += snprintf(z + zn, sizeof(z) - zn, "%s%s %s", i ? "  " : "", nm, hm);
        }
        if (z[0]) pgAdd(z); }
      { char h[40]; nextHoliday(h, sizeof(h)); if (h[0]) pgAdd(h); }
      pgDraw();
      break; }
    case APP_WEATHER: {
      if (!wx.valid) { oledTimeHead("fetching", "weather..."); pgAdd("fetching weather..."); pgDraw(); break; }
      char c[12]; strlcpy(c, wmoShort(wx.wcode), sizeof(c)); char tb[8] = "--:--"; if (haveT) snprintf(tb, sizeof(tb), "%02d:%02d", t.tm_hour, t.tm_min);
      snprintf(b, sizeof(b), "%.0fC", wx.temp); oledHead(b, tb, c);
      int rainIn = -1; for (int i = 0; i < 8; i++) if (wx.rain15[i] >= 0.2f) { rainIn = i * 15; break; }
      if (rainIn >= 0) { snprintf(b, sizeof(b), "rain in ~%d min", rainIn + 5); pgAdd(b, true); } else pgAddf("Wind %.0f  AQI %d  UV %.0f", wx.wind, wx.aqi, wx.uv);
      pgAddf("feels %.0f  hum %.0f%%", wx.feels, wx.hum);
      if (wx.sunriseMin >= 0) pgAddf("Rise %02d:%02d  Set %02d:%02d", wx.sunriseMin / 60, wx.sunriseMin % 60, wx.sunsetMin / 60, wx.sunsetMin % 60);
      pgNews(1);
      if (pgDraw(oledScale == 0 ? 1 : 0) == 0) {   // custom page 0 (1x only): 12 h temperature bars with the range
        float mn = wx.hourly[0], mx = wx.hourly[0];
        for (int i = 1; i < 12; i++) { if (wx.hourly[i] < mn) mn = wx.hourly[i]; if (wx.hourly[i] > mx) mx = wx.hourly[i]; }
        if (mx - mn < 2) mx = mn + 2;
        snprintf(b, sizeof(b), "feels %.0f   next 12 h", wx.feels); oledLineB(0, b);
        for (int i = 0; i < 12; i++) { int h = 2 + (int)((wx.hourly[i] - mn) / (mx - mn) * 14); infoDisplay.fillRect(2 + i * 7, 50 - h, 5, h, SSD1306_WHITE); }
        snprintf(b, sizeof(b), "%.0f-%.0f", mn, mx); infoDisplay.setFont(&F_SM13R); infoDisplay.setCursor(90, 48); infoDisplay.print(b); infoDisplay.setFont();
        if (rainIn >= 0) snprintf(b, sizeof(b), "rain in ~%d min", rainIn + 5); else snprintf(b, sizeof(b), "Wind %.0f  AQI %d  UV %.0f", wx.wind, wx.aqi, wx.uv);
        oledLineB(2, b);
      }
      break; }
    case APP_POMODORO: {   // 5.5.0: the OLED never repeats the countdown; clock + one key number on top, what's next below
      char g[12], d[12]; fmtHMs(g, sizeof(g), pcfg.goalMin); unsigned tm = todayFocus();
      if (pomoSettings) {
        char v[16]; psetValue(psetSel, v, sizeof(v)); snprintf(b, sizeof(b), "%d/%d", psetSel + 1, PSET_N); oledHead("SET", b, psetEdit ? "editing" : "");
        snprintf(b, sizeof(b), "%s: %s", PSET_NAMES[psetSel], v); pgAdd(b, true); pgWrap(PSET_HELP[psetSel], 2); pgDraw(); break;
      }
      if (pomoAskLog) { oledTimeHead("END?", ""); snprintf(b, sizeof(b), "YES: log %lum partial", (pomoTotalMs - pomoRemainAtPause) / 60000UL); pgAdd(b, true); pgAdd("NO: discard it"); pgAdd("hold: back to session"); pgDraw(); break; }
      if (pomoState == POMO_SELECT) {
        char s2[12]; snprintf(b, sizeof(b), "GOAL %u%%", tm * 100 / pcfg.goalMin > 999 ? 999 : tm * 100 / pcfg.goalMin); snprintf(s2, sizeof(s2), "streak %u", streakNow()); oledTimeHead(b, s2);
        fmtHMs(d, sizeof(d), tm);
        if (tm >= pcfg.goalMin) { snprintf(b, sizeof(b), "goal met: %s", d); pgAdd(b, true); pgAdd("extra time counts too"); }
        else { char lf[12]; fmtHMs(lf, sizeof(lf), pcfg.goalMin - tm); snprintf(b, sizeof(b), "%s / %s  %s left", d, g, lf); pgAdd(b, true); int f = pomoFocusMin() > 0 ? pomoFocusMin() : 25; pgAddf("about %d more sessions", (int)((pcfg.goalMin - tm + f - 1) / f)); }
        pgAdd(pomoEditing ? "turn: mins  tap: go" : "tap: go  hold: setup");
        if (fstats.valid) { unsigned st = streakNow(); pgAddf("best streak %u", fstats.bestStreak > st ? fstats.bestStreak : st); char w[12]; fmtHMs(w, sizeof(w), fstats.week); pgAddf("this week %s", w); pgAddf("all time %uh", fstats.totalH); }
        else pgAdd(FOCUS_LOG_URL[0] ? "stats: syncing..." : "stats: no FOCUS_LOG_URL");
        pgDraw(); break;
      }
      bool focus = (pomoState == POMO_FOCUS) || (pomoState == POMO_PAUSED && pomoWasFocus);
      unsigned long rem = pomoRemainMs(); unsigned el = (unsigned)((pomoTotalMs - rem) / 60000UL);
      int endMin = haveT ? (t.tm_hour * 60 + t.tm_min + (int)(rem / 60000) + 1) % 1440 : 0;
      if (pomoState == POMO_PAUSED) {
        unsigned long ps = (millis() - pomoPausedAtMs) / 1000; snprintf(b, sizeof(b), "PAUSE %lu:%02lu", ps / 60, ps % 60); oledHead(b, NULL, NULL);
        snprintf(b, sizeof(b), "%u min focused so far", el); pgAdd(b, true); pgAdd("tap: resume"); pgAdd(focus && !pomoIsTimer ? "2x tap or hold: end" : "hold: reset"); pgDraw(); break;
      }
      if (pomoIsTimer) { oledTimeHead("TIMER", ""); snprintf(b, sizeof(b), "ends %02d:%02d", endMin / 60, endMin % 60); pgAdd(b, true); pgAdd("hold: cancel"); pgDraw(); break; }
      if (focus) {
        unsigned after = tm + (unsigned)(pomoTotalMs / 60000UL); char s2[12];
        snprintf(b, sizeof(b), "GOAL %u%%", (tm + el) * 100 / pcfg.goalMin); snprintf(s2, sizeof(s2), "streak %u", streakNow()); oledTimeHead(b, s2);
        bool longB = (pomoSession + 1) % POMO_SESSIONS_PER_CYCLE == 0;   // page A: plan
        snprintf(b, sizeof(b), "ends %02d:%02d  break %dm", endMin / 60, endMin % 60, longB ? (int)POMO_LONG_BREAK_MIN : pomoBreakMin()); pgAdd(b, true);
        fmtHMs(d, sizeof(d), after); pgAddf("then %s  %u%% of goal", d, after * 100 / pcfg.goalMin);
        pgAdd(pomoTask[0] ? pomoTask : "task label: web page");
        unsigned held = heldPlanes + heldMkt + heldOther;   // page B: quiet mode
        if (pcfg.quiet) { snprintf(b, sizeof(b), "%u alerts held", held); pgAdd(b, true); pgAddf("%u planes %u mkt %u other", heldPlanes, heldMkt, heldOther); }
        else { pgAdd("quiet mode off", true); pgAdd("alerts show as usual"); }
        pgAdd("2x tap: end session");
        if (fstats.valid) {   // page C: momentum
          char w[12]; fmtHMs(w, sizeof(w), fstats.week + el); snprintf(b, sizeof(b), "week %s", w);
          if (fstats.prev) { char c[10]; snprintf(c, sizeof(c), "  %+d%%", (int)(((long)fstats.week + el - (long)fstats.prev) * 100 / (long)fstats.prev)); strlcat(b, c, sizeof(b)); }
          pgAdd(b, true);
          if (fstats.bestDay[0]) { fmtHMs(w, sizeof(w), fstats.best); pgAddf("best day %s %s", fstats.bestDay, w); }
          pgAddf("avg %um  %u%% done", fstats.avg, fstats.donePct);
        }
        pgDraw(); break;
      }
      snprintf(b, sizeof(b), "BREAK %lu:%02lu", rem / 60000UL, (rem / 1000UL) % 60); oledHead(b, NULL, NULL);
      pgAdd(breathing ? "box breathing 4-4-4-4" : "stand up, water, eyes", true);
      if (pcfg.autoStart) pgAddf("next: focus %d at %02d:%02d", pomoFocusMin(), endMin / 60, endMin % 60); else pgAdd("then tap to start");
      { unsigned held = heldPlanes + heldMkt + heldOther; if (held) pgAddf("%u alerts were held", held); else pgAdd("tap: breathing guide"); }
      pgDraw(); break; }
    case APP_MARKET: {
      char c1[12] = "", v1[12] = "", v2[14] = "";
      bool open = haveT && marketOpenNow(t);
      if (marketSel < NUM_SYMBOLS && quotes[marketSel].valid) { float chg = quotes[marketSel].prevClose > 0 ? (quotes[marketSel].price - quotes[marketSel].prevClose) / quotes[marketSel].prevClose * 100 : 0; strlcpy(c1, MARKET_SHORT[marketSel], sizeof(c1)); fmtPrice(quotes[marketSel].price, v1, sizeof(v1), quotes[marketSel].price > 9999 ? 0 : 2); snprintf(v2, sizeof(v2), "%+.1f%%", chg); }
      else if (marketSel >= NUM_SYMBOLS) strlcpy(c1, marketSel == heatPage() ? "WATCH" : marketSel == mfPage() ? "MF" : "PORTF", sizeof(c1));
      else strlcpy(c1, MARKET_SHORT[marketSel], sizeof(c1));
      oledHead(c1, v1, v2);
      for (int k = 1; k < NUM_SYMBOLS && pgN < 6; k++) {
        int i = ((marketSel < NUM_SYMBOLS ? marketSel : 0) + k) % NUM_SYMBOLS; const Quote &o = quotes[i];
        if (o.valid) { char v[12]; fmtPrice(o.price, v, sizeof(v), o.price > 9999 ? 0 : (o.price > 999 ? 1 : 2)); float c = o.prevClose > 0 ? (o.price - o.prevClose) / o.prevClose * 100 : 0; pgAddf("%s %s %+.1f%%", MARKET_SHORT[i], v, c); }
        else pgAddf("%s ...", MARKET_SHORT[i]);
      }
      pgAdd(open ? "NSE open till 15:30" : "NSE closed, opens 9:15");
      pgNews(1); pgDraw();
      break; }
    case APP_NETWORK: {
      char r1[12] = "", r2[12] = "";
      { uint16_t gw = netLastGw, wan = netLastWan, dns = netLastDns; snprintf(r1, sizeof(r1), gw == 0xFFFF ? "gw x" : "gw %ums", gw); snprintf(r2, sizeof(r2), wan == 0xFFFF ? "wan x" : "wan %ums", wan); if (dns == 0xFFFF) pgAdd("DNS failed", true); else pgAddf("DNS %u ms", dns); }
      oledTimeHead(r1, r2);
      { char node[10]; nodeStr(node, sizeof(node)); pgAddf("%s %ddBm ch%d", node, WiFi.RSSI(), WiFi.channel()); }
      if (outageCount) pgAddf("%d outages, last %um", outageCount, outages[0].minutes); else pgAdd("no outages logged");
      pgNews(1); pgDraw();
      break; }
    case APP_STATS: {
      snprintf(b, sizeof(b), "%d today", seenToday); oledTimeHead(b, "aircraft");
      pgAddf("%d arr  %d dep  %d over", seenArr, seenDep, seenEnr);
      pgAddf("high FL%03d %s", (int)(recHighFt / 100), recHighCs);
      pgAddf("fast %.0fkt %s", recFastKt, recFastCs);
      pgNews(1); pgDraw();
      break; }
    case APP_NEWS: {
      oledTimeHead("NEWS", newsCount ? (snprintf(b, sizeof(b), "%d/%d", newsScroll + 1, newsCount), b) : "");
      if (newsCount) pgWrap(news[newsScroll < newsCount ? newsScroll : 0].text, 6); else pgAdd("fetching feeds...");
      pgDraw();
      break; }
    case APP_CRICKET: {
      bool hv = arenaOwner == ARENA_CRICKET;
      oledTimeHead("CRICKET", hv && ARENA_CR->m[0].valid && ARENA_CR->m[0].live ? "LIVE" : "");
      static CricketMatch none = {};
      const CricketMatch &m = hv ? ARENA_CR->m[subPage < 2 ? subPage : 0] : none;
      if (m.valid) { pgAdd(m.score[0], true); pgAdd(m.score[1], true); pgWrap(m.status, 3); }
      else pgAdd("no live match");
      pgDraw();
      break; }
    case APP_SYSTEM: {
      snprintf(b, sizeof(b), "heap %uK", ESP.getFreeHeap() / 1024); oledTimeHead("SYSTEM", b);
      unsigned long up = (millis() - bootMs) / 1000;
      pgAddf("fps %d  up %lud %02lu:%02lu", fpsShown, up / 86400, (up / 3600) % 24, (up / 60) % 60);
      pgAdd(WiFi.localIP().toString().c_str());
      pgAddf("src %s %s", sourceName(activeSource), fetchOk ? "ok" : "fail");
      pgAddf("block %uK  v%s", ESP.getMaxAllocHeap() / 1024, FW_VERSION);
      pgDraw();
      break; }
    case APP_BRIGHT: {
      oledHead("DISPLAY", "", "");
      pgAddf("round %d%%  oled %d%%", tftBright, oledBright);
      pgAddf("theme %s  sweep %s", THEME_NAMES[theme], sweepOn ? "on" : "off");
      pgAddf("text size %s", OLED_SCALE_NAMES[oledScale]);
      pgAdd(brightSel == 0 ? "adjusting: round" : brightSel == 1 ? "adjusting: OLED" : brightSel == 2 ? "adjusting: theme" : brightSel == 3 ? "adjusting: sweep" : "adjusting: text size", true);
      pgAdd("hold: back to menu");
      pgDraw();
      break; }
    case APP_REPLAY: {
      int cur = replayPos; time_t rt = time(NULL) - (time_t)cur * 60; struct tm rtm; localtime_r(&rt, &rtm);
      char at[12]; snprintf(at, sizeof(at), "at %02d:%02d", rtm.tm_hour, rtm.tm_min);
      if (cur) snprintf(b, sizeof(b), "-%d min", cur); else strcpy(b, "now");
      oledTimeHead(b, at);
      int mins = arenaOwner == ARENA_AVIATION ? ARENA_AV->minutes : 0; if (mins > REPLAY_LEN) mins = REPLAY_LEN;
      pgAddf("%d min recorded", mins); pgAdd(replayPlay ? "playing" : "paused"); pgAdd("rotate: scrub  click: go");
      pgDraw();
      break; }
    case APP_F1: {
      oledTimeHead("F1", f1.valid ? "" : "...");
      if (f1.valid) { pgAdd(f1.race, true); long dt = (long)f1.t - (long)time(NULL); if (f1.t && dt > 0) pgAddf("in %ld d %ld h", dt / 86400, (dt % 86400) / 3600); else if (f1.t && dt > -3 * 3600) pgAdd("race weekend LIVE"); else pgAdd("finished"); if (f1.podium[0][0]) pgAddf("P1 %s", f1.podium[0]); }
      else pgAdd("fetching race data...");
      pgNews(1); pgDraw();
      break; }
    case APP_LOCATION: {
      oledTimeHead(locTarget ? "WEATHER" : "HOME", locTarget ? (weatherIndex >= 0 ? AIRPORTS[weatherIndex].iata : weatherIndex == -2 ? "house" : "=radar") : AIRPORTS[homeIndex].iata);
      { int i = locAirport(listSel < locCount() ? listSel : 0);
        if (i < 0) { pgAddf("HOME %s", WX_HOME_NAME); pgAddf("%.4f %.4f", wxHomeLat, wxHomeLon); }
        else { pgAddf("%s %s", AIRPORTS[i].iata, AIRPORTS[i].name); pgAddf("rwy %s  elev %d m", AIRPORTS[i].rwyLabel, AIRPORTS[i].elevM); } }
      pgBold[0] = 1;
      pgAdd(locTarget ? "click: set  hold: home" : "click: set  hold: wx");
      pgDraw();
      break; }
    case APP_ARRIVALS: {
      bool arr = subPage == 0; bool other = boardAirport != homeIndex; const Airport &ap = AIRPORTS[boardAirport];
      bool hv = other && arenaOwner == ARENA_BOARD;
      const Plane *src = hv ? ARENA_BD->items : planes; int n = hv ? ARENA_BD->count : (other ? 0 : planeCount); unsigned long boardMs = hv ? ARENA_BD->ms : 0;
      snprintf(b, sizeof(b), "%s %s", arr ? "ARR" : "DEP", ap.iata); oledTimeHead(b, metar.valid && !other ? (snprintf(b, sizeof(b), "rwy %02d", activeRwy > 0 ? activeRwy : ap.rwyHdg / 10), b) : "");
      int idx[MAX_PLANES]; int m = boardList(idx, 6, arr, src, n, ap.lat, ap.lon);
      int nowMin = haveT ? t.tm_hour * 60 + t.tm_min : 0;
      for (int r = 0; r < 6 && r < m; r++) {
        const Plane &p = src[idx[r]]; char pcs[12]; prettyCallsign(p.callsign, pcs, sizeof(pcs)); char *sp = strchr(pcs, ' '); if (sp) memmove(sp, sp + 1, strlen(sp));
        const char *route = routeFor(p.callsign); char code[6] = "---"; if (route && route[0]) { if (arr) strlcpy(code, route, 4); else { const char *d = strrchr(route, '-'); strlcpy(code, d ? d + 1 : route, 4); } }
        float d = distanceKm(ap.lat, ap.lon, p.lat, p.lon); char tm[8];
        if (arr) { int eta = etaMin(p, d); int em = (nowMin + eta) % 1440; snprintf(tm, sizeof(tm), "%02d:%02d", em / 60, em % 60); } else snprintf(tm, sizeof(tm), "FL%03d", (int)(p.altitude_m * 3.28084f / 100));
        pgAddf("%s %s %s %.0fkm", pcs, code, tm, d);
      }
      if (m == 0) pgAdd(other && !boardMs ? "fetching..." : "nothing in range");
      pgDraw();
      break; }
    case APP_APPROACH: {
      snprintf(b, sizeof(b), "RWY %02d", activeRwy > 0 ? activeRwy : rwyHdg() / 10); oledTimeHead("APPROACH", b);
      int idx[MAX_PLANES]; int m = approachList(idx, MAX_PLANES);
      if (m == 0) { pgAdd("no traffic on approach"); pgDraw(); break; }
      const Plane &p = planes[idx[listSel < m ? listSel : 0]]; float ft = p.altitude_m * 3.28084f; float dev = ft - (homeElevM() * 3.28084f + p.distance_km * 172);
      char pcs[12]; prettyCallsign(p.callsign, pcs, sizeof(pcs));
      pgAddf("%s  %s", pcs, p.type); pgBold[0] = 1;
      pgAddf("%.0f ft  %+d fpm", ft, (int)p.vs_fpm);
      if (p.cls == CLS_DEPARTURE) pgAddf("departing  %.0f km", p.distance_km); else pgAddf("%.0f km  %s %.0f ft", p.distance_km, fabsf(dev) < 150 ? "on slope" : dev > 0 ? "high" : "low", fabsf(dev));
      if (metar.valid) pgAddf("wind %03d/%02d  QNH %d", metar.wdir < 0 ? 0 : metar.wdir, metar.wspd, metar.qnh);
      pgDraw();
      break; }
    default: {   // RADAR (v6): inbound timeline graphic; flight details are on the right dial
      drawInboundTimeline();
      break; }
  }
  oledPush(0);
}

// =====================================================================
// WEB
// =====================================================================
String v6SettingsJson() {   // appended to /api/state and /config.json (6.7.0)
  return ",\"mute\":" + String(muteAll ? 1 : 0) + ",\"clk\":" + String(clickOn ? 1 : 0) + ",\"pzl\":" + String(piezoLoud ? 1 : 0) + ",\"ringb\":" + String(ringBright) + ",\"n0\":" + String(nightStart) + ",\"n1\":" + String(nightEnd) +
         ",\"mtg\":" + String(meetOn ? 1 : 0) + ",\"mtgl\":" + String(meetLead) + ",\"mtgt\":" + String(meetTitles ? 1 : 0) + ",\"onair\":" + String(onAirOn ? 1 : 0) + ",\"micc\":" + String(micClick ? 1 : 0) +
         ",\"apps\":" + String((unsigned long)appMask) + ",\"bapp\":" + String(bootApp) + ",\"fsync\":" + String(focusSync ? 1 : 0) + ",\"nring\":" + String(nightRings) + ",\"clkr\":" + String(clkRight) + ",\"sring\":" + String(secRing ? 1 : 0) + ",\"rfoc\":" + String(radarFocus) + ",\"lookup\":" + String(lookupOn ? 1 : 0) + ",\"lookkm\":" + String(lookupKm) + ",\"iss\":" + String(issOn ? 1 : 0);
}
// NOTE: keep new functions ABOVE the R"HTML(...)" page strings. The Arduino prototype generator (ctags 5.8) does not
// understand raw strings; the quotes inside the HTML leave it confused and the next function can lose every prototype after it.
// Web page (6.13.0): tools/webpage/index.html, embedded verbatim. Edit the file, then re-embed.
const char PAGE[] PROGMEM = R"HTML(<!doctype html><html lang=en><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1"><title>Desk Radar</title>
<style>
:root{--bg:#0b0d10;--card:#14181d;--line:#232a32;--tx:#d8dde3;--dim:#7d8792;--acc:#3fd07a;--amb:#f0a52a;--red:#ef5350;--blue:#4fb0ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:15px/1.45 -apple-system,"Segoe UI",Roboto,Helvetica,Arial,sans-serif}
header{position:sticky;top:0;z-index:5;background:#0b0d10ee;backdrop-filter:blur(8px);border-bottom:1px solid var(--line);padding:12px 18px;display:flex;flex-wrap:wrap;gap:8px 18px;align-items:center}
header h1{font-size:18px;margin:0 12px 0 0;letter-spacing:.06em}header .chip{font-size:12px;color:var(--dim);background:var(--card);border:1px solid var(--line);border-radius:999px;padding:3px 10px}
header .chip b{color:var(--tx);font-weight:600}nav{margin-left:auto;display:flex;gap:4px;flex-wrap:wrap}nav a{color:var(--dim);text-decoration:none;font-size:13px;padding:4px 8px;border-radius:6px}nav a:hover{color:var(--tx);background:var(--card)}
main{max-width:1180px;margin:0 auto;padding:16px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(330px,1fr));gap:14px}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:14px 16px}
.card h2{font-size:12px;letter-spacing:.14em;text-transform:uppercase;color:var(--dim);margin:0 0 10px;font-weight:600}
.row{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:7px 0;border-top:1px solid var(--line)}.row:first-of-type{border-top:0}
.row label{color:var(--tx)}.row small{display:block;color:var(--dim);font-size:12px;margin-top:2px}
select,input[type=text],input[type=number],input[type=time],textarea{background:#0e1115;color:var(--tx);border:1px solid var(--line);border-radius:8px;padding:6px 8px;font:inherit;font-size:14px;min-width:120px}
input[type=number]{width:90px;min-width:0}textarea{width:100%;min-height:90px;font-family:ui-monospace,Menlo,monospace;font-size:12px}
input[type=range]{width:150px;accent-color:var(--acc)}.val{display:inline-block;width:46px;text-align:right;color:var(--dim);font-variant-numeric:tabular-nums}
.sw{position:relative;width:44px;height:24px;flex:none}.sw input{opacity:0;width:0;height:0}.sw i{position:absolute;inset:0;background:#2a323c;border-radius:999px;transition:.15s}.sw i:before{content:"";position:absolute;left:3px;top:3px;width:18px;height:18px;border-radius:50%;background:#aab3bd;transition:.15s}
.sw input:checked+i{background:var(--acc)}.sw input:checked+i:before{transform:translateX(20px);background:#fff}
.live{display:grid;grid-template-columns:1fr 1fr;gap:10px;align-items:center;justify-items:center}.live img.dial{width:100%;max-width:240px;border-radius:50%;background:#000;image-rendering:pixelated}.live img.oled{width:100%;max-width:256px;image-rendering:pixelated;border:1px solid var(--line);border-radius:6px}
.btn{background:#1b222a;color:var(--tx);border:1px solid var(--line);border-radius:8px;padding:7px 12px;font:inherit;cursor:pointer}.btn:hover{border-color:var(--dim)}.btn.red{color:var(--red)}
pre#log{background:#0e1115;border:1px solid var(--line);border-radius:8px;padding:10px;max-height:380px;overflow:auto;font:12px/1.4 ui-monospace,Menlo,monospace;color:#b9c3cc;white-space:pre-wrap}
.alerts div{padding:5px 0;border-top:1px solid var(--line);font-size:13px}.alerts span{color:var(--dim);margin-right:8px;font-variant-numeric:tabular-nums}
#toast{position:fixed;bottom:18px;left:50%;transform:translateX(-50%);background:#1b222a;border:1px solid var(--line);padding:8px 14px;border-radius:10px;font-size:13px;opacity:0;transition:.2s;pointer-events:none}#toast.on{opacity:1}
.apps{display:grid;grid-template-columns:1fr 1fr;gap:2px 12px}.apps label{display:flex;align-items:center;gap:8px;padding:4px 0;font-size:14px}
.wide{grid-column:1/-1}@media(max-width:700px){.live{grid-template-columns:1fr 1fr}}
</style></head><body>
<header><h1>DESK RADAR</h1><span class=chip>v<b id=fw>?</b></span><span class=chip>up <b id=up>?</b></span><span class=chip>WiFi <b id=rssi>?</b></span><span class=chip><b id=acn>?</b> aircraft via <b id=active>?</b></span><span class=chip><b id=fps>?</b> fps</span><span class=chip>heap <b id=heap>?</b> KB</span><span class=chip>Mac <b id=mac>?</b></span>
<nav><a href=#live>Live</a><a href=#settings>Settings</a><a href=#tools>Tools</a><a href=#log>Log</a><a href=/config.json>Export</a></nav></header>
<main>
<section id=live class=card style="margin-bottom:14px"><h2>Live</h2><div class=live>
<img class=dial id=imL src=/screen.bmp alt="left dial"><img class=dial id=imR src="/screen.bmp?d=1" alt="right dial"><img class=oled id=imOL src=/oled.bmp alt="left OLED"><img class=oled id=imOR src="/oled.bmp?d=1" alt="right OLED"></div>
<div style="margin-top:8px;display:flex;gap:8px;align-items:center;flex-wrap:wrap"><button class=btn onclick="liveOn=!liveOn;this.textContent=liveOn?'pause live':'resume live'">pause live</button>
<span style="color:var(--dim);font-size:12px">app:</span><select id=appsel onchange="fetch('/app?n='+this.value)"></select>
<button class=btn onclick="knob(1,'turn=-1')">◀ right knob</button><button class=btn onclick="knob(1,'turn=1')">right knob ▶</button><button class=btn onclick="knob(1,'click=1')">right click</button><button class=btn onclick="knob(1,'hold=1')">right hold</button><button class=btn onclick="knob(0,'click=1')">left click</button></div></section>
<div id=settings class=grid>
<div class=card><h2>Sound</h2>
<div class=row><label>Mute everything<small>alarm still rings</small></label><label class=sw><input type=checkbox data-k=mute><i></i></label></div>
<div class=row><label>Knob clicks</label><label class=sw><input type=checkbox data-k=clk><i></i></label></div>
<div class=row><label>Louder piezo<small>low notes an octave up</small></label><label class=sw><input type=checkbox data-k=pzl><i></i></label></div>
<div class=row><label>Hour chime</label><label class=sw><input type=checkbox data-k=chime><i></i></label></div>
<div class=row><label>Quiet hours<small>from / until (same = off)</small></label><span><select data-k=q0 class=hours></select> <select data-k=q1 class=hours></select></span></div>
<div class=row><label>Pomodoro sound</label><select data-k=psound><option value=0>chime<option value=1>soft<option value=2>silent</select></div>
<div class=row><label>Alert sounds</label><span id=amask class=apps style="grid-column:auto"></span></div>
</div>
<div class=card><h2>Meetings</h2>
<div class=row><label>Reminders</label><label class=sw><input type=checkbox data-k=mtg><i></i></label></div>
<div class=row><label>First reminder</label><select data-k=mtgl><option value=2>2 min before<option value=5>5 min before<option value=10>10 min before<option value=15>15 min before</select></div>
<div class=row><label>Show titles<small>off: only "Meeting"</small></label><label class=sw><input type=checkbox data-k=mtgt><i></i></label></div>
<div class=row><label>On-air light<small>red dot on the right ring during calls</small></label><label class=sw><input type=checkbox data-k=onair><i></i></label></div>
<div class=row><label>Mic on left click<small>mutes the Mac mic on calls</small></label><label class=sw><input type=checkbox data-k=micc><i></i></label></div>
<div class=row><label>Mac Focus sync<small>Shortcuts "DeskRadar Focus On / Off"</small></label><label class=sw><input type=checkbox data-k=fsync><i></i></label></div>
</div>
<div class=card><h2>Display</h2>
<div class=row><label>Round brightness</label><span><input type=range min=10 max=100 step=5 data-k=tftb><span class=val></span></span></div>
<div class=row><label>OLED brightness</label><span><input type=range min=0 max=100 step=5 data-k=oledb><span class=val></span></span></div>
<div class=row><label>Ring brightness</label><span><input type=range min=0 max=100 step=5 data-k=ringb><span class=val></span></span></div>
<div class=row><label>Theme</label><select data-k=theme><option value=0>green<option value=1>amber<option value=2>ice<option value=3>white</select></div>
<div class=row><label>Night dimming</label><select data-k=night><option value=0>auto<option value=1>always<option value=2>off</select></div>
<div class=row><label>Night hours<small>from / until</small></label><span><select data-k=n0 class=hours></select> <select data-k=n1 class=hours></select></span></div>
<div class=row><label>Rings at night</label><select data-k=nring><option value=0>off<option value=1>nightlight<option value=2>normal</select></div>
<div class=row><label>Screens off<small>from / until (same = off)</small></label><span><select data-k=slp0 class=hours></select> <select data-k=slp1 class=hours></select></span></div>
<div class=row><label>OLED text size</label><select data-k=osz><option value=0>1x<option value=1>1.5x<option value=2>2x<option value=3>3x</select></div>
<div class=row><label>Radar sweep</label><label class=sw><input type=checkbox data-k=sweep><i></i></label></div>
<div class=row><label>Slideshow after<small>idle minutes, 0 = off</small></label><span><input type=number min=0 max=240 step=5 data-k=slide> <select data-k=slmode><option value=0>cycle apps<option value=1>back to radar</select></span></div>
</div>
<div class=card><h2>Radar</h2>
<div class=row><label>Home airport</label><select data-k=home class=airports></select></div>
<div class=row><label>Arrivals board airport</label><select data-k=board class=airports></select></div>
<div class=row><label>Range</label><select data-k=ri><option value=0>10 km<option value=1>25 km<option value=2>50 km<option value=3>100 km<option value=4>200 km<option value=5>auto</select></div>
<div class=row><label>Show</label><select data-k=filter><option value=0>all<option value=1>arrivals<option value=2>departures</select></div>
<div class=row><label>Right dial follows</label><select data-k=rfoc><option value=0>arrivals<option value=1>departures</select></div>
<div class=row><label>Data source</label><select data-k=src><option value=0>OpenSky<option value=1>airplanes.live<option value=2>adsb.lol<option value=3>adsb.fi<option value=4>local receiver</select></div>
<div class=row><label>Local receiver URL</label><input type=text data-k=lurl placeholder="http://192.168.x.x/data/aircraft.json"></div>
<div class=row><label>Alert banners<small>alerts take over the dials</small></label><label class=sw><input type=checkbox data-k=popup><i></i></label></div>
<div class=row><label>Look-up alert<small>aircraft close to the house</small></label><span><label class=sw><input type=checkbox data-k=lookup><i></i></label> <input type=number min=1 max=5 data-k=lookkm> km</span></div>
<div class=row><label>Home position override<small>lat / lon, 0 = airport</small></label><span><input type=number step=0.0001 data-k=lat style="width:110px"> <input type=number step=0.0001 data-k=lon style="width:110px"></span></div>
</div>
<div class=card><h2>Clock</h2>
<div class=row><label>12-hour clock</label><label class=sw><input type=checkbox data-k=h12><i></i></label></div>
<div class=row><label>Face</label><select data-k=face><option value=0>Bold<option value=1>Analog<option value=2>Dashboard<option value=3>World<option value=4>24 Hour<option value=5>Minimal<option value=6>Flip<option value=7>Seconds<option value=8>Night Sky<option value=9>Year Dots<option value=10>Huge<option value=11>Rings<option value=12>Segments<option value=13>Nixie<option value=14>Words<option value=15>Weather<option value=16>Orbit</select></div>
<div class=row><label>Right dial<small>with the clock</small></label><select data-k=clkr><option value=0>weather<option value=1>agenda<option value=2>world<option value=3>sun and moon<option value=4>next arrivals</select></div>
<div class=row><label>Alarm</label><span><label class=sw><input type=checkbox data-k=alon><i></i></label> <input type=time data-k=almin></span></div>
<div class=row><label>Alarm days</label><select data-k=aldays><option value=62>weekdays<option value=127>every day<option value=65>weekends<option value=0>custom (knob)</select></div>
<div class=row><label>ISS passes</label><label class=sw><input type=checkbox data-k=iss><i></i></label></div>
<div class=row><label>Seconds ring<small>right ring, every app</small></label><label class=sw><input type=checkbox data-k=sring><i></i></label></div>
<div class=row><label>Weather location</label><select data-k=wxloc class=wxloc></select></div>
<div class=row><label>House position<small>lat / lon for weather and look-up</small></label><span><input type=number step=0.0001 data-k=wxlat style="width:110px"> <input type=number step=0.0001 data-k=wxlon style="width:110px"></span></div>
</div>
<div class=card><h2>Pomodoro</h2>
<div class=row><label>Your name</label><input type=text data-k=uname maxlength=12></div>
<div class=row><label>Daily goal<small>minutes</small></label><input type=number min=30 max=480 step=15 data-k=pgoal></div>
<div class=row><label>Long break / cycle</label><span><input type=number min=5 max=60 step=5 data-k=plong> min / <input type=number min=2 max=8 data-k=pcyc> sessions</span></div>
<div class=row><label>Auto start next</label><label class=sw><input type=checkbox data-k=pauto><i></i></label></div>
<div class=row><label>Quiet mode in focus</label><select data-k=pquiet><option value=0>off<option value=1>hold plane alerts<option value=2>planes + market<option value=3>all alerts</select></div>
<div class=row><label>Greetings</label><label class=sw><input type=checkbox data-k=pgreet><i></i></label></div>
<div class=row><label>Day recap at</label><select data-k=precap><option value=0>off<option value=17>17:00<option value=18>18:00<option value=19>19:00<option value=20>20:00<option value=21>21:00</select></div>
<div class=row><label>Ending early</label><select data-k=pearly><option value=0>ask<option value=1>always log<option value=2>never log</select></div>
<div class=row><label>Task label</label><input type=text data-k=ptask maxlength=24></div>
<div class=row><label>Meeting timer<small>minutes</small></label><input type=number min=1 max=180 data-k=ptmr></div>
<div class=row><label>Stand-up nudge<small>every N minutes, 0 = off</small></label><input type=number min=0 max=120 step=15 data-k=nudge></div>
</div>
<div class=card><h2>Apps</h2>
<div class=row><label>App at power-on</label><select data-k=bapp id=bapp></select></div>
<div class=row><label>Shown in the knob list</label></div><div id=apps class=apps></div>
</div>
<div class=card><h2>Watch list</h2>
<div class=row><label>Flight watch<small>callsign prefix, route part like -DXB, or type prefix; comma separated</small></label></div><input type=text data-k=watch style="width:100%" placeholder="AIC, -DXB, B78">
<div class=row><label>Countdowns<small>name=YYYY-MM-DD HH:MM; separated by ;</small></label></div><input type=text data-k=cdown style="width:100%" placeholder="Trip=2026-12-20 06:00">
</div>
<div class=card><h2>Recent alerts</h2><div id=alerts class=alerts></div></div>
<div id=tools class=card><h2>Tools</h2>
<div class=row><label>Settings file<small>export, or paste a file to import</small></label><a class=btn href=/config.json download>export</a></div>
<textarea id=imp placeholder='{"range":2, ...}'></textarea><div style="margin:8px 0"><button class=btn onclick=doImport()>import</button></div>
<div class=row><label>Firmware<small>upload a .bin, the device restarts</small></label><form method=POST action=/update enctype=multipart/form-data style="display:flex;gap:8px;align-items:center"><input type=file name=fw accept=.bin style="font-size:12px"><button class=btn>flash</button></form></div>
<div class=row><label>Diagnostics</label><span><a class=btn href=/i2c>i2c</a> <a class=btn href=/dns>dns</a> <a class=btn href=/coredump>crash</a> <a class=btn href="/sdls?path=/deskradar">sd</a> <a class=btn href=/live>live mirror</a></span></div>
<div class=row><label>Restart</label><button class="btn red" onclick="if(confirm('Restart the device?'))fetch('/restart').then(()=>toast('restarting'))">restart</button></div>
</div>
<div id=log class="card wide"><h2>Log <span style="float:right;font-weight:400;text-transform:none;letter-spacing:0"><input type=text id=logf placeholder=filter style="min-width:140px;padding:3px 8px"> <label><input type=checkbox id=logauto checked> auto</label></span></h2><pre id=log></pre></div>
</div></main><div id=toast></div>
<script>
const $=s=>document.querySelector(s),$$=s=>[...document.querySelectorAll(s)];
const APPS=["RADAR","ARRIVALS","APPROACH","REPLAY","CLOCK","WEATHER","POMODORO","MARKETS","NETWORK","STATS","NEWS","CRICKET","F1","SYSTEM","DISPLAY","LOCATION","AGENDA","SETTINGS","MEDIA"],HIDE=[3,8,9,11,12,13,14,15];
let liveOn=true,st=null,tt=null;
function toast(m){const t=$('#toast');t.textContent=m;t.classList.add('on');clearTimeout(tt);tt=setTimeout(()=>t.classList.remove('on'),1400)}
function set(k,v){if(k==='ri')k='range';fetch('/set?'+k+'='+encodeURIComponent(v)).then(r=>r.ok?toast('saved '+k):toast('failed '+k))}
function knob(side,a){fetch('/knob?side='+side+'&'+a)}
$$('.hours').forEach(s=>{for(let h=0;h<24;h++)s.insertAdjacentHTML('beforeend','<option value='+h+'>'+(h%12||12)+(h<12?' am':' pm')+'</option>')});
APPS.forEach((n,i)=>{if(HIDE.includes(i))return;$('#bapp').insertAdjacentHTML('beforeend','<option value='+i+'>'+n+'</option>');if(i!==17)$('#apps').insertAdjacentHTML('beforeend','<label><input type=checkbox class=appbit data-bit='+i+'> '+n+'</label>');$('#appsel').insertAdjacentHTML('beforeend','<option value='+i+'>'+n+'</option>')});
$$('[data-k]').forEach(el=>{el.addEventListener(el.type==='range'?'input':'change',()=>{let v;if(el.type==='checkbox')v=el.checked?1:0;else if(el.type==='time'){const[h,m]=el.value.split(':');v=(+h)*60+(+m)}else v=el.value;if(el.type==='range')el.nextElementSibling.textContent=v+'%';if(el.type==='range'){clearTimeout(el._t);el._t=setTimeout(()=>set(el.dataset.k,v),250)}else set(el.dataset.k,v)})});
function mask(cls,ds){return $$(cls).reduce((m,c)=>c.checked?(m|(1<<+c.dataset[ds]))>>>0:m,0)>>>0}
document.addEventListener('change',e=>{if(e.target.classList.contains('appbit'))set('apps',mask('.appbit','bit'));if(e.target.classList.contains('ambit'))set('amask',mask('.ambit','bit'))});
function fill(){if(!st)return;const a=document.activeElement;$$('[data-k]').forEach(el=>{if(el===a||!(el.dataset.k in st))return;const v=st[el.dataset.k];if(el.type==='checkbox')el.checked=!!+v;else if(el.type==='time'){el.value=String(Math.floor(v/60)).padStart(2,'0')+':'+String(v%60).padStart(2,'0')}else el.value=v;if(el.type==='range')el.nextElementSibling.textContent=v+'%'});
if(st.aldays!==undefined&&![62,127,65].includes(+st.aldays))$('[data-k=aldays]').value=0;
$$('.appbit').forEach(c=>{if(c!==a)c.checked=!!((st.apps>>>+c.dataset.bit)&1)});$$('.ambit').forEach(c=>{if(c!==a)c.checked=!!((st.amask>>>+c.dataset.bit)&1)});
$('#fw').textContent=st.fw||'?';$('#up').textContent=st.up||'?';$('#rssi').textContent=(st.rssi||'?')+' dBm';$('#acn').textContent=(st.ac||[]).length;$('#active').textContent=st.active||'?';$('#fps').textContent=st.fps;$('#heap').textContent=st.heap;$('#mac').textContent=st.mac?'linked':'off';
$('#alerts').innerHTML=(st.alerts||[]).map(x=>'<div><span>'+x.t+'</span>'+x.l1+' <span>'+x.l2+'</span></div>').join('')||'<div style="color:var(--dim)">none yet</div>'}
async function poll(){try{const r=await fetch('/api/state');const j=await r.json();const first=!st;st=j;if(first){const ap=$$('.airports');ap.forEach(s=>s.innerHTML=j.airports.map((n,i)=>'<option value='+i+'>'+n+'</option>').join(''));$('.wxloc').innerHTML='<option value=-2>House</option><option value=-1>Radar home</option>'+j.airports.map((n,i)=>'<option value='+i+'>'+n+'</option>').join('');
$('#amask').innerHTML=(j.mnames||[]).map((n,i)=>[0,4,5,13].includes(i)?'':'<label><input type=checkbox class=ambit data-bit='+i+'> '+n+'</label>').join('')}fill()}catch(e){}}
async function logPoll(){if(!$('#logauto').checked)return;try{const t=await (await fetch('/log')).text();const f=$('#logf').value.toLowerCase();const lines=t.split('\n').filter(l=>!f||l.toLowerCase().includes(f));const p=$('#log');const atEnd=p.scrollTop+p.clientHeight>=p.scrollHeight-20;p.textContent=lines.slice(-400).join('\n');if(atEnd)p.scrollTop=p.scrollHeight}catch(e){}}
const SB='http://'+location.hostname+':8080';function live(){if(!liveOn||document.hidden)return;const n=Date.now();$('#imL').src=SB+'/screen.bmp?t='+n;$('#imR').src=SB+'/screen.bmp?d=1&t='+n;$('#imOL').src=SB+'/oled.bmp?t='+n;$('#imOR').src=SB+'/oled.bmp?d=1&t='+n}
async function doImport(){const r=await fetch('/import',{method:'POST',headers:{'Content-Type':'text/plain'},body:$('#imp').value});toast(await r.text());poll()}
poll();setInterval(poll,5000);logPoll();setInterval(logPoll,10000);setInterval(live,2000);$('#logf').addEventListener('input',logPoll);
</script></body></html>)HTML";

const char LIVE_PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta name=viewport content="width=device-width,initial-scale=1"><title>Live</title>
<style>body{background:#000;color:#ccc;font-family:monospace;text-align:center;padding:12px}img{image-rendering:pixelated;display:block;margin:10px auto}#r{width:480px;height:480px;border-radius:50%}#o{width:512px;height:256px;border:1px solid #333}a{color:#6cf}</style></head><body>
<img id=r src=/screen.bmp><img id=o src=/oled.bmp><p>both screens, refreshed every 2 s &nbsp; <a href=/>back</a></p>
<script>setInterval(()=>{document.getElementById('r').src='/screen.bmp?'+Date.now();document.getElementById('o').src='/oled.bmp?'+Date.now()},2000)</script></body></html>)HTML";

const char IMPORT_PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta name=viewport content="width=device-width,initial-scale=1"><title>Import</title>
<style>body{background:#000;color:#ccc;font-family:monospace;padding:20px}textarea{width:100%;height:240px;background:#111;color:#9f9;border:1px solid #363}button{font-size:16px;margin:8px 0;background:#263;color:#fff;border:0;padding:8px 14px}a{color:#6cf}</style></head><body>
<h3>Import settings</h3><p>Paste the JSON from <a href=/config.json>/config.json</a> (another board, or a backup). Keys not present are left unchanged.</p>
<form method=POST action=/import><textarea name=cfg></textarea><br><button>Apply</button></form><p><a href=/>back</a></p></body></html>)HTML";

const char UPDATE_PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta name=viewport content="width=device-width,initial-scale=1"><title>Firmware</title>
<style>body{background:#000;color:#ccc;font-family:monospace;padding:20px}input,button{font-size:16px;margin:8px 0}a{color:#6cf}</style></head><body>
<h3>Desk Radar firmware upload</h3><p>Arduino IDE: Sketch &gt; Export Compiled Binary, then upload the .bin here. The board restarts when done.</p>
<form method=POST action=/update enctype=multipart/form-data><input type=file name=fw accept=.bin><br><button>Upload</button></form><p><a href=/>back</a></p></body></html>)HTML";

String jsonEsc(const char *s) { String o; for (; *s; s++) { if (*s == '"' || *s == '\\') o += '\\'; if ((uint8_t)*s >= 32) o += *s; } return o; }
void handleRoot() { server.send_P(200, "text/html", PAGE); }
String pcfgJson();
void handleState() {
  String j; j.reserve(900 + planeCount * 160);
  { unsigned long up = (millis() - bootMs) / 1000; char ub[24]; snprintf(ub, sizeof(ub), "%lud %02lu:%02lu", up / 86400, (up / 3600) % 24, (up / 60) % 60);
    j += "{\"fw\":\"" FW_VERSION "\",\"up\":\"" + String(ub) + "\",\"rssi\":" + String(WiFi.RSSI()) + ",\"mac\":" + String(macLinked() ? 1 : 0) + ",\"rfoc\":" + String(radarFocus) + ","; }
  j += "\"lat\":" + String(homeLat, 5) + ",\"lon\":" + String(homeLon, 5) + ",\"rk\":" + String(displayRangeKm(), 1) +
       ",\"range\":" + String((int)(displayRangeKm() / KM_PER_UNIT)) + ",\"unit\":\"" + RANGE_UNIT + "\",\"ri\":" + String((int)rangeIndex) +
       ",\"src\":" + String((int)dataSource) + ",\"night\":" + String((int)nightMode) + ",\"filter\":" + String((int)filterMode) +
       ",\"lurl\":\"" + jsonEsc(localUrl) + "\",\"active\":\"" + sourceName(activeSource) + "\"" +
       ",\"home\":" + String((int)homeIndex) + ",\"board\":" + String((int)boardAirport) + ",\"wxloc\":" + String((int)weatherIndex) + ",\"wxlat\":" + String(wxHomeLat, 5) + ",\"wxlon\":" + String(wxHomeLon, 5) + ",\"sweep\":" + String(sweepOn ? 1 : 0) + ",\"osz\":" + String(oledScale) + ",\"popup\":" + String(popupsOn ? 1 : 0) + ",\"h12\":" + String(clock12 ? 1 : 0) + ",\"face\":" + String((int)clockFace) +
       ",\"tftb\":" + String(tftBright) + ",\"oledb\":" + String(oledBright) + ",\"slp0\":" + String(sleepStart) + ",\"slp1\":" + String(sleepEnd) +
       ",\"alon\":" + String(alarmOn ? 1 : 0) + ",\"aldays\":" + String(alarmDays) + ",\"almin\":" + String(alarmMin) + ",\"nudge\":" + String(nudgeMin) +
       ",\"heap\":" + String(ESP.getFreeHeap() / 1024) + ",\"fps\":" + String(fpsShown) +
       ",\"watch\":\"" + jsonEsc(watchStr) + "\",\"cdown\":\"" + jsonEsc(cdownStr) + "\"" +
       ",\"theme\":" + String(theme) + ",\"slide\":" + String(slideMin) + ",\"slmode\":" + String(slideMode) + ",\"q0\":" + String(quietStart) + ",\"q1\":" + String(quietEnd) + v6SettingsJson() +
       ",\"chime\":" + String(chimeOn ? 1 : 0) + ",\"ptask\":\"" + jsonEsc(pomoTask) + "\",\"ptmr\":" + String(pomoTimerMin) + pcfgJson() + ",\"amask\":" + String((unsigned long)alertMask) + ",\"mnames\":[" + melodyNamesJson() + "]" +
       ",\"t\":" + String((double)(time(NULL)) - (millis() - lastFetchMs) / 1000.0, 1) + ",\"ranges\":[";
  for (int i = 0; i < NUM_RANGES; i++) { if (i) j += ","; j += String(RANGE_PRESETS[i]); }
  j += "],\"alerts\":[";
  for (int i = 0; i < alertLogCount; i++) { time_t at = alertLog[i].t; struct tm lt; localtime_r(&at, &lt); char tb[8]; snprintf(tb, sizeof(tb), "%02d:%02d", lt.tm_hour, lt.tm_min); if (i) j += ","; j += "{\"t\":\"" + String(tb) + "\",\"l1\":\"" + jsonEsc(alertLog[i].l1) + "\",\"l2\":\"" + jsonEsc(alertLog[i].l2) + "\"}"; }
  j += "],\"airports\":[";
  for (int i = 0; i < NUM_AIRPORTS; i++) { if (i) j += ","; j += "\"" + String(AIRPORTS[i].iata) + " " + AIRPORTS[i].name + "\""; }
  j += "],\"ac\":[";
  for (int i = 0; i < planeCount; i++) {
    const Plane &p = planes[i];
    if (!passesFilter(p)) continue;
    if (j.endsWith("}")) j += ",";
    const char *r = routeFor(p.callsign);
    j += "{\"cs\":\"" + jsonEsc(p.callsign) + "\",\"type\":\"" + jsonEsc(p.type) + "\",\"route\":\"" + (r ? jsonEsc(r) : "") + "\",\"lat\":" + String(p.lat, 5) + ",\"lon\":" + String(p.lon, 5) +
         ",\"alt\":" + String(p.altitude_m, 0) + ",\"v\":" + String(p.velocity_ms, 1) + ",\"h\":" + String(p.heading_deg, 0) +
         ",\"cls\":" + String(p.cls) + ",\"sel\":" + String(i == selectedIndex ? 1 : 0) + "}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}
bool applySetting(const char *k, const String &a) {   // one key from /set or /import; true if known
  int v = a.toInt();
  if (!strcmp(k, "home"))        { if (v >= 0 && v < NUM_AIRPORTS && v != homeIndex) { homeIndex = v; applyHome(); followIcao[0] = 0; if (currentApp == APP_ARRIVALS) arenaForApp(); } }
  else if (!strcmp(k, "board"))  { if (v >= 0 && v < NUM_AIRPORTS) { boardAirport = v; if (currentApp == APP_ARRIVALS) arenaForApp(); } }   // 6.14.1: the board arena follows a web change too
  else if (!strcmp(k, "wxloc"))  { if (v >= -2 && v < NUM_AIRPORTS) weatherIndex = v; }
  else if (!strcmp(k, "wxlat"))  { if (a.length()) { double d = a.toDouble(); if (isfinite(d) && d != 0 && fabs(d) <= 90) wxHomeLat = d; } }   // 6.14.1: finite, in range
  else if (!strcmp(k, "wxlon"))  { if (a.length()) { double d = a.toDouble(); if (isfinite(d) && d != 0 && fabs(d) <= 180) wxHomeLon = d; } }
  else if (!strcmp(k, "sweep"))  sweepOn = (v == 1);
  else if (!strcmp(k, "rfoc"))   radarFocus = v ? 1 : 0;
  else if (!strcmp(k, "osz"))    { if (v >= 0 && v <= 3) oledScale = v; }
  else if (!strcmp(k, "popup"))  popupsOn = (v == 1);
  else if (!strcmp(k, "h12"))    { clock12 = (v == 1); staticDirty = true; }
  else if (!strcmp(k, "pstop"))  { pomoStopFocus(); pomoAskLog = false; pomoState = POMO_SELECT; pomoEditing = false; breathing = false; pomoIsTimer = false; staticDirty = true; logln("pomodoro reset from web"); }
  else if (!strcmp(k, "lat"))    { if (a.length()) { double d = a.toDouble(); if (isfinite(d) && d != 0 && fabs(d) <= 90) homeLat = d; } }
  else if (!strcmp(k, "lon"))    { if (a.length()) { double d = a.toDouble(); if (isfinite(d) && d != 0 && fabs(d) <= 180) homeLon = d; } }
  else if (!strcmp(k, "src"))    { if (v >= 0 && v <= 4) dataSource = v; }
  else if (!strcmp(k, "lurl"))   strlcpy(localUrl, a.c_str(), sizeof(localUrl));
  else if (!strcmp(k, "night"))  { if (v >= 0 && v <= 2) nightMode = v; }
  else if (!strcmp(k, "range"))  { if (v >= 0 && v <= RANGE_AUTO) rangeIndex = v; }
  else if (!strcmp(k, "filter")) { if (v >= 0 && v <= 2) filterMode = v; }
  else if (!strcmp(k, "face"))   { if (v >= 0 && v < NUM_FACES) clockFace = v; }
  else if (!strcmp(k, "clkr"))   { if (v >= 0 && v <= 4) clkRight = v; }
  else if (!strcmp(k, "sring"))  secRing = v != 0;
  else if (!strcmp(k, "tftb"))   { if (v >= 10 && v <= 100) tftBright = v; }
  else if (!strcmp(k, "oledb"))  { if (v >= 0 && v <= 100) oledBright = v; }
  else if (!strcmp(k, "slp0"))   { if (v >= 0 && v <= 23) sleepStart = v; }
  else if (!strcmp(k, "slp1"))   { if (v >= 0 && v <= 23) sleepEnd = v; }
  else if (!strcmp(k, "alon"))   alarmOn = (v == 1);
  else if (!strcmp(k, "aldays")) { if (v > 0 && v < 128) alarmDays = v; }
  else if (!strcmp(k, "almin"))  { if (v >= 0 && v < 1440) alarmMin = v; }
  else if (!strcmp(k, "nudge"))  { if (v >= 0 && v <= 180) nudgeMin = v; }
  else if (!strcmp(k, "watch"))  { strlcpy(watchStr, a.c_str(), sizeof(watchStr)); parseWatch(); }
  else if (!strcmp(k, "cdown"))  { strlcpy(cdownStr, a.c_str(), sizeof(cdownStr)); parseCountdowns(); }
  else if (!strcmp(k, "theme"))  { if (v >= 0 && v <= 3 && v != theme) { theme = v; applyPalette(); } }
  else if (!strcmp(k, "slide"))  { if (v >= 0 && v <= 240) slideMin = v; }
  else if (!strcmp(k, "slmode")) { if (v >= 0 && v <= 1) slideMode = v; }
  else if (!strcmp(k, "q0"))     { if (v >= 0 && v <= 23) quietStart = v; }
  else if (!strcmp(k, "q1"))     { if (v >= 0 && v <= 23) quietEnd = v; }
  else if (!strcmp(k, "chime"))  chimeOn = (v == 1);
  else if (!strcmp(k, "mute"))   muteAll = (v == 1);
  else if (!strcmp(k, "clk"))    clickOn = (v == 1);
  else if (!strcmp(k, "pzl"))    piezoLoud = (v == 1);
  else if (!strcmp(k, "ringb"))  { if (v >= 0 && v <= 100) ringBright = v; }
  else if (!strcmp(k, "n0"))     { if (v >= 0 && v <= 23) nightStart = v; }
  else if (!strcmp(k, "n1"))     { if (v >= 0 && v <= 23) nightEnd = v; }
  else if (!strcmp(k, "mtg"))    meetOn = (v == 1);
  else if (!strcmp(k, "mtgl"))   { if (v == 2 || v == 5 || v == 10 || v == 15) meetLead = v; }
  else if (!strcmp(k, "mtgt"))   meetTitles = (v == 1);
  else if (!strcmp(k, "onair"))  onAirOn = (v == 1);
  else if (!strcmp(k, "micc"))   micClick = (v == 1);
  else if (!strcmp(k, "apps"))   { appMask = (uint32_t)strtoul(a.c_str(), NULL, 10) | (1u << APP_SETTINGS) | (1u << APP_RADAR); buildMenu(); }
  else if (!strcmp(k, "bapp"))   { if (v >= 0 && v < APP_COUNT) bootApp = v; }
  else if (!strcmp(k, "fsync"))  focusSync = (v == 1);
  else if (!strcmp(k, "nring"))  { if (v >= 0 && v <= 2) nightRings = v; }
  else if (!strcmp(k, "lookup")) lookupOn = (v == 1);
  else if (!strcmp(k, "lookkm")) { if (v >= 1 && v <= 5) lookupKm = v; }
  else if (!strcmp(k, "iss"))    issOn = (v == 1);
  else if (!strcmp(k, "ptask"))  strlcpy(pomoTask, a.c_str(), sizeof(pomoTask));
  else if (!strcmp(k, "ptmr"))   { if (v >= 1 && v <= 180) pomoTimerMin = v; }
  else if (!strcmp(k, "uname"))  strlcpy(pcfg.name, a.c_str(), sizeof(pcfg.name));
  else if (!strcmp(k, "pgoal"))  { if (v >= 30 && v <= 480) { pcfg.goalMin = v; statsWanted = true; } }
  else if (!strcmp(k, "pquiet")) { if (v >= 0 && v <= 3) pcfg.quiet = v; }
  else if (!strcmp(k, "precap")) { if (v == 0 || (v >= 17 && v <= 21)) pcfg.recapHour = v; }
  else if (!strcmp(k, "pgreet")) pcfg.greet = (v == 1);
  else if (!strcmp(k, "pearly")) { if (v >= 0 && v <= 2) pcfg.early = v; }
  else if (!strcmp(k, "panim"))  { if (v >= 0 && v <= 2) pcfg.anim = v; }
  else if (!strcmp(k, "psound")) { if (v >= 0 && v <= 2) pcfg.sound = v; }
  else if (!strcmp(k, "pauto"))  pcfg.autoStart = (v == 1);
  else if (!strcmp(k, "plong"))  { if (v >= 10 && v <= 30) pcfg.longBreak = v; }
  else if (!strcmp(k, "pcyc"))   { if (v >= 2 && v <= 6) pcfg.cycle = v; }
  else if (!strcmp(k, "pminlog")) { if (v == 1 || v == 5 || v == 10) pcfg.minLog = v; }
  else if (!strcmp(k, "pstreak")) { pcfg.streakWk = (v == 1); statsWanted = true; }
  else if (!strcmp(k, "amask"))  alertMask = (uint32_t)strtoul(a.c_str(), NULL, 10) | (1u << M_EMERG) | (1u << M_ALARM) | (1u << M_CLICK);
  else return false;
  return true;
}
String pcfgJson() {
  return ",\"uname\":\"" + jsonEsc(pcfg.name) + "\",\"pgoal\":" + String(pcfg.goalMin) + ",\"pquiet\":" + String(pcfg.quiet) + ",\"precap\":" + String(pcfg.recapHour) +
         ",\"pgreet\":" + String(pcfg.greet) + ",\"pearly\":" + String(pcfg.early) + ",\"panim\":" + String(pcfg.anim) + ",\"psound\":" + String(pcfg.sound) +
         ",\"pauto\":" + String(pcfg.autoStart) + ",\"plong\":" + String(pcfg.longBreak) + ",\"pcyc\":" + String(pcfg.cycle) + ",\"pminlog\":" + String(pcfg.minLog) + ",\"pstreak\":" + String(pcfg.streakWk);
}
void afterSettingsChange() { saveSettings(); alertArmed = false; staticDirty = true; clkS = -1; isNight = computeNight(); applyBrightness(); }
void handleSet() {
  for (int i = 0; i < server.args(); i++) applySetting(server.argName(i).c_str(), server.arg(i));
  afterSettingsChange();
  server.send(200, "text/plain", "ok");
}
String melodyNamesJson() { String o; for (int i = 0; i < M_COUNT; i++) { if (i) o += ","; o += "\"" + String(MELODY_NAMES[i]) + "\""; } return o; }
String configJsonString();
void handleConfig() {   // settings only (no live data), importable on /import
  server.sendHeader("Content-Disposition", "attachment; filename=deskradar-config.json");
  server.send(200, "application/json", configJsonString());
}
String configJsonString() {
  char alt[8]; snprintf(alt, sizeof(alt), "%d", alarmMin);
  String j = "{\"home\":" + String((int)homeIndex) + ",\"board\":" + String((int)boardAirport) + ",\"wxloc\":" + String((int)weatherIndex) + ",\"wxlat\":" + String(wxHomeLat, 5) + ",\"wxlon\":" + String(wxHomeLon, 5) + ",\"sweep\":" + String(sweepOn ? 1 : 0) + ",\"osz\":" + String(oledScale) + ",\"popup\":" + String(popupsOn ? 1 : 0) + ",\"h12\":" + String(clock12 ? 1 : 0) + ",\"lat\":" + String(homeLat, 5) + ",\"lon\":" + String(homeLon, 5) +
             ",\"src\":" + String((int)dataSource) + ",\"lurl\":\"" + jsonEsc(localUrl) + "\",\"night\":" + String((int)nightMode) + ",\"range\":" + String((int)rangeIndex) + ",\"filter\":" + String((int)filterMode) +
             ",\"face\":" + String((int)clockFace) + ",\"tftb\":" + String(tftBright) + ",\"oledb\":" + String(oledBright) + ",\"slp0\":" + String(sleepStart) + ",\"slp1\":" + String(sleepEnd) +
             ",\"alon\":" + String(alarmOn ? 1 : 0) + ",\"aldays\":" + String(alarmDays) + ",\"almin\":" + String(alt) + ",\"nudge\":" + String(nudgeMin) +
             ",\"watch\":\"" + jsonEsc(watchStr) + "\",\"cdown\":\"" + jsonEsc(cdownStr) + "\",\"theme\":" + String(theme) + ",\"slide\":" + String(slideMin) + ",\"slmode\":" + String(slideMode) + v6SettingsJson() +
             ",\"q0\":" + String(quietStart) + ",\"q1\":" + String(quietEnd) + ",\"chime\":" + String(chimeOn ? 1 : 0) + ",\"ptask\":\"" + jsonEsc(pomoTask) + "\",\"ptmr\":" + String(pomoTimerMin) + pcfgJson() + ",\"amask\":" + String((unsigned long)alertMask) +
             ",\"fw\":\"" FW_VERSION "\"}";
  return j;
}
void handleImportPage() { server.send_P(200, "text/html", IMPORT_PAGE); }
void handleImport() {
  String cfg = server.arg("cfg"); if (!cfg.length()) cfg = server.arg("plain");
  JsonDocument doc; DeserializationError e = deserializeJson(doc, cfg);
  if (e) { server.send(400, "text/plain", String("bad json: ") + e.c_str()); return; }
  int n = 0;
  for (JsonPair kv : doc.as<JsonObject>()) {
    String v;
    if (kv.value().is<const char*>()) v = kv.value().as<const char*>();
    else if (kv.value().is<long>()) v = String(kv.value().as<long>());
    else if (kv.value().is<unsigned long>()) v = String(kv.value().as<unsigned long>());
    else v = String(kv.value().as<double>(), 5);
    if (applySetting(kv.key().c_str(), v)) n++;
  }
  afterSettingsChange(); logf("import: %d settings applied\n", n);
  server.send(200, "text/html", "<meta http-equiv=refresh content='2;url=/'>" + String(n) + " settings applied");
}
void handleLive() { server.send_P(200, "text/html", LIVE_PAGE); }
void handleLog() {   // v6: streamed from the PSRAM ring in two pieces (no 16 KB String on the internal heap)
  if (!logRing) { server.send(503, "text/plain", "no log"); return; }
  portENTER_CRITICAL(&logMux); int h = logHead; bool w = logWrapped; portEXIT_CRITICAL(&logMux);
  server.setContentLength((w ? LOG_RING - h : 0) + h); server.send(200, "text/plain; charset=utf-8", "");
  if (w) server.sendContent(logRing + h, LOG_RING - h);
  if (h) server.sendContent(logRing, h);
}
// 24-bit BMP of the round display, painted band by band straight into the socket
unsigned long lastShotMs = 0;
void handleScreen() {   // v6: last frame straight from the PSRAM framebuffer; ?d=1 for the right dial
  if (millis() - lastShotMs < 1500) { server.send(503, "text/plain", "busy"); return; }   // 6.13.1: the 173 KB stream blocks loop(); never back to back
  lastShotMs = millis();
  uint16_t *fbuf = fbFront[(server.hasArg("d") && server.arg("d") == "1") ? 1 : 0];   // last completed frame
  if (!fbuf || sleeping) { server.send(503, "text/plain", "no frame"); return; }
  const uint32_t rowBytes = 240 * 3, imgBytes = rowBytes * 240, fileBytes = 54 + imgBytes;
  uint8_t hdr[54] = {0};
  hdr[0] = 'B'; hdr[1] = 'M'; memcpy(hdr + 2, &fileBytes, 4); hdr[10] = 54; hdr[14] = 40;
  int32_t w = 240, h = 240; memcpy(hdr + 18, &w, 4); memcpy(hdr + 22, &h, 4); hdr[26] = 1; hdr[28] = 24; memcpy(hdr + 34, &imgBytes, 4);
  WiFiClient client = server.client();
  server.setContentLength(fileBytes); server.send(200, "image/bmp", "");
  client.write(hdr, 54);
  static uint8_t row[240 * 3];
  client.setTimeout(2);   // 6.13.1: a cancelled download (browser swapped the image) left loop() writing into a dead socket for >30 s: task watchdog reset at 21:36
  for (int y = 239; y >= 0; y--) {                         // BMP rows are bottom-up
    if (!client.connected()) break;
    for (int x = 0; x < 240; x++) { uint16_t c = fbuf[y * 240 + x]; row[x * 3 + 0] = (c & 0x1F) << 3; row[x * 3 + 1] = ((c >> 5) & 0x3F) << 2; row[x * 3 + 2] = ((c >> 11) & 0x1F) << 3; }
    if (client.write(row, rowBytes) != rowBytes) break;
    if ((y & 15) == 0) esp_task_wdt_reset();
  }
}
// 1-bit BMP of the OLED from the SSD1306 buffer (pages of 8 vertical pixels)
void handleOled() {
  const uint32_t rowBytes = 16, imgBytes = rowBytes * 64, fileBytes = 62 + imgBytes;
  uint8_t hdr[62] = {0};
  hdr[0] = 'B'; hdr[1] = 'M'; memcpy(hdr + 2, &fileBytes, 4); hdr[10] = 62; hdr[14] = 40;
  int32_t w = 128, h = 64; memcpy(hdr + 18, &w, 4); memcpy(hdr + 22, &h, 4); hdr[26] = 1; hdr[28] = 1; memcpy(hdr + 34, &imgBytes, 4);
  uint32_t ncol = 2; memcpy(hdr + 46, &ncol, 4);
  hdr[54] = 0; hdr[55] = 0; hdr[56] = 0; hdr[57] = 0;               // colour 0 black
  hdr[58] = 255; hdr[59] = 200; hdr[60] = 60; hdr[61] = 0;           // colour 1 OLED-ish (BGR)
  WiFiClient client = server.client();
  server.setContentLength(fileBytes); server.send(200, "image/bmp", "");
  client.write(hdr, 62);
  // ?d=1 = right OLED. Both are drawn with rotation 2, so read the raw buffer mirrored to get what the eye sees.
  uint8_t *buf = (server.hasArg("d") && server.arg("d") == "1") ? infoR.getBuffer() : infoDisplay.getBuffer(); uint8_t row[16];
  client.setTimeout(2);
  for (int y = 63; y >= 0; y--) {
    if (!client.connected()) break;
    memset(row, 0, 16);
    for (int x = 0; x < 128; x++) { int rx = 127 - x, ry = 63 - y; if (buf[(ry / 8) * 128 + rx] & (1 << (ry & 7))) row[x >> 3] |= 0x80 >> (x & 7); }
    client.write(row, 16);
  }
}
void handleUpdatePage() { server.send_P(200, "text/html", UPDATE_PAGE); }
void handleUpdateDone() {
  bool ok = !Update.hasError();
  server.send(200, "text/html", ok ? "<meta http-equiv=refresh content='8;url=/'>Update OK, restarting..." : "Update failed, see /log");
  if (ok) { delay(500); ESP.restart(); }
}
void handleUpdateUpload() {
  HTTPUpload &up = server.upload(); esp_task_wdt_reset();
  // v6 (6.1.4): a reset upload left the Update session open, so the next begin() failed. Abort stale sessions,
  // handle aborts, log the real error, and redraw the progress only every 256 KB on the left dial.
  if (up.status == UPLOAD_FILE_START) {
    logf("firmware upload: %s\n", up.filename.c_str());
    if (Update.isRunning()) { Update.abort(); logln("firmware: aborted a stale update session"); }
    msgL1 = "FIRMWARE"; msgL2 = "uploading..."; msgL3 = ""; msgPct = 0; present(paintMessage);
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) logf("Update.begin failed: %s\n", Update.errorString());
  }
  else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.isRunning() && Update.write(up.buf, up.currentSize) != up.currentSize) logf("Update.write failed: %s\n", Update.errorString());
    static unsigned lastKb = 0; unsigned kb = up.totalSize / 1024;
    if (kb / 256 != lastKb / 256) { lastKb = kb; static char b[16]; snprintf(b, sizeof(b), "%u KB", kb); msgL2 = b; msgPct = (int)(kb * 100 / 1600 % 101); present(paintMessage); }
  }
  else if (up.status == UPLOAD_FILE_END) { if (Update.end(true)) logf("firmware: %u bytes OK\n", up.totalSize); else logf("Update.end failed: %s\n", Update.errorString()); }
  else if (up.status == UPLOAD_FILE_ABORTED) { Update.abort(); logln("firmware upload aborted by the client"); staticDirty = true; }
}

// =====================================================================
// WIFI
// =====================================================================
// 6.5.6: DHCP hands out the ISP resolver (49.205.72.130), which stalled lookups for 70 to 104 s (blocking
// lwip_getaddrinfo froze the whole fetch queue). Use the router first (it caches), then Cloudflare and Google.
void dnsFix() {
  ip_addr_t d0, d1, d2; IPAddress gw = WiFi.gatewayIP();
  IP_ADDR4(&d0, gw[0], gw[1], gw[2], gw[3]); IP_ADDR4(&d1, 1, 1, 1, 1); IP_ADDR4(&d2, 8, 8, 8, 8);
  dns_setserver(0, &d0); dns_setserver(1, &d1); dns_setserver(2, &d2);
}
void handleDns() {   // /dns?h=host : time a lookup (diagnostic)
  String h = server.hasArg("h") ? server.arg("h") : String("api.adsb.lol"); IPAddress ip;
  unsigned long t0 = millis(); int ok = dnsLookup(h.c_str(), ip, 2000); unsigned long dt = millis() - t0;
  const ip_addr_t *s0 = dns_getserver(0), *s1 = dns_getserver(1);
  char b[160]; snprintf(b, sizeof(b), "%s -> %s in %lu ms (%s)\nservers %s, %s\n", h.c_str(), ip.toString().c_str(), dt, ok ? "ok" : "FAILED", ipaddr_ntoa(s0), ipaddr_ntoa(s1));
  server.send(200, "text/plain", b);
}
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP: wifiUp = true; dnsFix(); logf("WiFi up, IP %s\n", WiFi.localIP().toString().c_str()); break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: wifiUp = false; lastDiscReason = info.wifi_sta_disconnected.reason; logf("WiFi disconnected, reason %d\n", lastDiscReason); break;
    default: break;
  }
}
void oledMsg(const char *a, const char *b) {
  infoDisplay.clearDisplay(); infoDisplay.setTextColor(SSD1306_WHITE); infoDisplay.setTextSize(1);
  infoDisplay.setCursor(0, 0); infoDisplay.print(a); infoDisplay.setCursor(0, 16); infoDisplay.print(b); oledPush(0);
}
void connectWiFi() {
  WiFi.mode(WIFI_STA); WiFi.setSleep(false); WiFi.setAutoReconnect(true); WiFi.setHostname(OTA_HOSTNAME); WiFi.onEvent(onWiFiEvent);
  haveCreds = strcmp(WIFI_SSID, "YOUR_WIFI_SSID") != 0 && WIFI_SSID[0];
  if (haveCreds) { oledMsg("Connecting WiFi", WIFI_SSID); WiFi.begin(WIFI_SSID, WIFI_PASSWORD); unsigned long t0 = millis(); while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) delay(200); }
  else { WiFi.begin(); unsigned long t0 = millis(); while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200); }
  if (WiFi.status() != WL_CONNECTED) {
    logln("WiFi failed, starting config portal");
    oledMsg("Join WiFi: DeskRadar6-Setup", "then open 192.168.4.1");
    showMessage("WIFI SETUP", "join DeskRadar6-Setup", "then open 192.168.4.1", -1);
    WiFiManager wm; wm.setConfigPortalTimeout(180);
    if (!wm.startConfigPortal("DeskRadar6-Setup")) { logln("Portal timeout, rebooting"); delay(1000); ESP.restart(); }
  }
  wifiUp = (WiFi.status() == WL_CONNECTED);
}
// 6.16.0: say it when the network goes and comes back. The left OLED shows the WiFi page meanwhile and the left
// ring blinks an amber pip at 6 o'clock (ringsTick). 8 s grace so a roam between mesh nodes stays silent.
void netNoticeTick() {
  static bool wifiWas = true, wanWas = false, told = false; static unsigned long downSince = 0, lastNag = 0; unsigned long now = millis();
  if (wifiUp != wifiWas) {
    wifiWas = wifiUp;
    if (!wifiUp) { downSince = now; told = false; }
    else if (told) { raiseAlert("WIFI BACK", WiFi.SSID().c_str(), COL_OK, -1, 3000, NULL); logln("net: WiFi back"); told = false; }
  }
  if (!wifiUp && !told && now - downSince > 8000) { told = true; lastNag = now; raiseAlert("WIFI DOWN", "reconnecting...", COL_DEP, M_NUDGE, 5000, NULL); logln("net: WiFi down notice"); }
  if (!wifiUp && told && now - lastNag > 600000UL) { lastNag = now; raiseAlert("STILL NO WIFI", "check the router", COL_DEP, M_NUDGE, 5000, NULL); }
  if (wanDown != wanWas) { wanWas = wanDown; if (wanDown) raiseAlert("INTERNET DOWN", "WiFi ok, no way out", COL_DEP, M_NUDGE, 5000, NULL); else raiseAlert("INTERNET BACK", "", COL_OK, -1, 3000, NULL); }
}
void wifiWatchdog() {
  if (wantReconnect) { wantReconnect = false; lastReconnectMs = millis(); wifiUp = false; WiFi.disconnect(); delay(300); if (haveCreds) WiFi.begin(WIFI_SSID, WIFI_PASSWORD); else WiFi.begin(); return; }
  if (wifiUp) return;
  if (WiFi.status() == WL_CONNECTED) return;   // 6.14.1: associated, waiting for DHCP: do not tear it down
  if (millis() - lastReconnectMs > 30000) {
    lastReconnectMs = millis(); logln("WiFi watchdog: reconnecting");
    WiFi.disconnect(); delay(50);
    if (haveCreds) WiFi.begin(WIFI_SSID, WIFI_PASSWORD); else WiFi.begin();
  }
}

// =====================================================================
// INPUT
// =====================================================================
void arenaForApp() {   // apps with big display-only data take the scratch arena; others leave it to its owner
  switch (currentApp) {
    case APP_RADAR: case APP_APPROACH: case APP_REPLAY: arenaAcquire(ARENA_AVIATION); break;
    case APP_ARRIVALS: arenaAcquire(boardAirport != homeIndex ? ARENA_BOARD : ARENA_AVIATION); break;
    case APP_MARKET: arenaAcquire(ARENA_MARKET); break;
    case APP_NETWORK: arenaAcquire(ARENA_NETWORK); break;
    case APP_CRICKET: arenaAcquire(ARENA_CRICKET); break;
    default: break;
  }
}
void enterApp(int a) {
  if (appRetired(a)) a = APP_RADAR;   // 6.16.0
  currentApp = (App)a; menuOpen = false; staticDirty = true; setCat = -1; setSel = 0; setEdit = false; clkS = -1; clkFaceDrawn = -1; lastHudMs = 0; subPage = 0; listSel = 0; pomoSettings = false; psetEdit = false;
  arenaForApp();
  if (currentApp == APP_MARKET) marketForce = true;
  if (currentApp == APP_REPLAY) { replayPos = 0; replayPlay = true; replayLastMs = millis(); }
  fadeK = 0.3f;   // fade in, ramped back to 1 in loop()
  fxWipe(COL_SWEEP);
  if (millis() - bootMs > 3000) appCardMs = millis();
  drawInfo();
}
void wakeScreens() { if (sleeping) { wakeUntilMs = millis() + 120000; sleepTick(); } }
void onShortPress() {
  fxPulse(1);
  wakeScreens(); if (noteInput()) { drawInfo(); return; }   // the click only woke the slideshow
  if (alarmRinging) { alarmSnooze(); drawInfo(); return; }
  if (ovActive()) { ovEnd(); return; }   // a tap dismisses a greeting / celebration
  if (menuOpen) { int a = menuItems[menuSel]; if (a == APP_LOCATION) locTarget = 0; enterApp(a); if (a == APP_LOCATION) listSel = homeIndex; playMelody(M_CLICK, false); return; }
  // an alert about a specific aircraft: click selects it on the radar
  if (toastActive() && toast.icao[0]) {
    for (int i = 0; i < planeCount; i++) if (!strcmp(planes[i].icao24, toast.icao)) { selectedIndex = i; strlcpy(selectedIcao, planes[i].icao24, sizeof(selectedIcao)); currentMode = MODE_SELECT; toast.untilMs = 0; if (currentApp != APP_RADAR) enterApp(APP_RADAR); staticDirty = true; drawInfo(); return; }
  }
  switch (currentApp) {
    case APP_RADAR: currentMode = (currentMode == MODE_ZOOM) ? MODE_SELECT : MODE_ZOOM; staticDirty = true; break;
    case APP_ARRIVALS: subPage = subPage ? 0 : 1; listSel = 0; staticDirty = true; break;
    case APP_APPROACH: { int idx[MAX_PLANES]; int m = approachList(idx, MAX_PLANES); if (m) { int i = idx[listSel < m ? listSel : 0]; selectedIndex = i; strlcpy(selectedIcao, planes[i].icao24, sizeof(selectedIcao)); currentMode = MODE_SELECT; enterApp(APP_RADAR); } break; }
    case APP_CLOCK: clkRight = (clkRight + 1) % 5; settingsDirty = true; staticDirty = true; raiseAlert("RIGHT DIAL", CLKR_NAMES[clkRight], COL_TEXT, -1, 1500, NULL); break;   // 6.18.1: click = right dial content, turn = face
    case APP_WEATHER: subPage = (subPage + 1) % 3; staticDirty = true; break;
    case APP_REPLAY: replayPlay = !replayPlay; staticDirty = true; break;
    case APP_POMODORO:
      if (pomoSettings) { psetEdit = !psetEdit; if (!psetEdit) settingsDirty = true; staticDirty = true; break; }   // settings: tap edits / saves
      if (pomoAskLog) { pomoFinishAsk(pomoAskSel == 0); break; }   // confirm the highlighted choice
      if (pomoFocusActive() && pomoLastTapMs && millis() - pomoLastTapMs < POMO_DOUBLE_TAP_MS) { pomoLastTapMs = 0; pomoAskEnd(); break; }   // double tap: end
      pomoLastTapMs = pomoFocusActive() ? millis() : 0;   // only taps during a focus session count toward a double tap
      if (pomoState == POMO_SELECT) { if (POMO_PRESETS[pomoPreset].focus == 0 && !pomoEditing) pomoEditing = true; else { pomoEditing = false; pomoStart(true); } }
      else if (pomoState == POMO_PAUSED) { pomoEndMs = millis() + pomoRemainAtPause; pomoState = pomoWasFocus ? POMO_FOCUS : POMO_BREAK; }
      else if (pomoState == POMO_FOCUS) { pomoRemainAtPause = pomoRemainMs(); pomoWasFocus = true; pomoState = POMO_PAUSED; pomoPausedAtMs = millis(); }
      else { breathing = !breathing; breathStartMs = millis(); }   // break: box breathing on / off
      staticDirty = true; settingsDirty = true; break;
    case APP_MARKET: marketForce = true; break;
    case APP_STATS: subPage = (subPage + 1) % 3; staticDirty = true; break;
    case APP_NEWS: newsScroll += 3; if (newsScroll >= newsCount) newsScroll = 0; staticDirty = true; break;
    case APP_SYSTEM: subPage = (subPage + 1) % 3; staticDirty = true; break;
    case APP_CRICKET: subPage = (subPage + 1) % 2; staticDirty = true; break;
    case APP_BRIGHT: brightSel = (brightSel + 1) % 5; staticDirty = true; break;
    case APP_SETTINGS: settingsClick(); break;
    case APP_MEDIA: macQueue("play"); mac.playing = !mac.playing; break;
    case APP_LOCATION:
      if (locTarget == 1) { int i = locAirport(listSel); weatherIndex = i < 0 ? -2 : (i == homeIndex ? -1 : i); saveSettings(); enterApp(APP_WEATHER); logf("weather location set: %s\n", i < 0 ? WX_HOME_NAME : AIRPORTS[i].name); raiseAlert("WEATHER FOR", i < 0 ? WX_HOME_NAME : AIRPORTS[i].name, COL_ENROUTE_HIGH, -1, 2500, NULL); }
      else { homeIndex = listSel; applyHome(); followIcao[0] = 0; alertArmed = false; saveSettings(); enterApp(APP_RADAR); logf("radar home set: %s\n", AIRPORTS[homeIndex].name); raiseAlert("RADAR HOME", AIRPORTS[homeIndex].name, COL_OK, -1, 2500, NULL); }   // 6.15.2: say what just changed (home moved to Paris unnoticed)
      break;
    default: break;
  }
  playMelody(M_CLICK, false); drawInfo();
}
void onLongPress() {           // 0.6 - 2 s
  wakeScreens(); if (noteInput()) { drawInfo(); return; }
  if (alarmRinging) { alarmOff(); drawInfo(); return; }
  if (ovActive()) { ovEnd(); return; }
  if (menuOpen) return;
  switch (currentApp) {
    case APP_RADAR:
      if (currentMode == MODE_SELECT && planeCount) { if (followIcao[0]) followIcao[0] = 0; else strlcpy(followIcao, selectedIcao, sizeof(followIcao)); }
      else { radarFocus ^= 1; settingsDirty = true; raiseAlert(radarFocus ? "DEPARTURES" : "ARRIVALS", radarFocus ? "OLED: arrivals" : "OLED: departures", radarFocus ? COL_DEP : COL_ARR, -1, 1500, NULL); }   // v6: hold flips ARRIVALS / DEPARTURES (filter: web page)
      staticDirty = true; break;
    case APP_ARRIVALS: boardAirport = (boardAirport + 1) % NUM_AIRPORTS; listSel = 0; settingsDirty = true; staticDirty = true; if (boardAirport != homeIndex) { arenaOwner = ARENA_NONE; arenaAcquire(ARENA_BOARD); } else arenaAcquire(ARENA_AVIATION); break;
    case APP_CLOCK: alarmOn = !alarmOn; settingsDirty = true; staticDirty = true; raiseAlert(alarmOn ? "ALARM ON" : "ALARM OFF", alarmOn ? (char*)"deskradar6.local to set" : (char*)"", COL_ARR, -1, 2000, NULL); break;
    case APP_POMODORO:
      if (pomoSettings) { if (psetEdit) psetEdit = false; else pomoSettings = false; settingsDirty = true; staticDirty = true; break; }   // hold: leave edit, then leave settings
      if (pomoAskLog) { pomoAskLog = false; staticDirty = true; break; }      // hold on the prompt: cancel, session stays paused
      if (pomoFocusActive()) { pomoAskEnd(); break; }                         // hold during focus: same prompt as double tap
      if (pomoState == POMO_SELECT && !pomoEditing) { pomoSettings = true; psetSel = 0; psetEdit = false; staticDirty = true; break; }   // hold on Home: settings
      pomoStopFocus(); pomoState = POMO_SELECT; pomoEditing = false; breathing = false; pomoIsTimer = false; staticDirty = true; break;
    case APP_APPROACH: subPage = subPage ? 0 : 1; staticDirty = true; break;
    case APP_REPLAY: replayPos = 0; replayPlay = true; staticDirty = true; break;
    case APP_WEATHER: enterApp(APP_SETTINGS); setCat = 3; setSel = 5; staticDirty = true; break;   // 6.16.0: weather location lives in SETTINGS > RADAR
    case APP_LOCATION: locTarget = locTarget ? 0 : 1; listSel = locTarget ? locRowFor(weatherIndex) : homeIndex; staticDirty = true; break;
    case APP_BRIGHT: menuOpen = true; menuSel = 0; for (int i = 0; i < menuCount; i++) if (menuItems[i] == APP_BRIGHT) menuSel = i; drawMenu(); break;
    case APP_SETTINGS: settingsHold(); break;
    case APP_MEDIA: macQueue("next"); raiseAlert("NEXT TRACK", "", COL_TEXT, -1, 1200, NULL); break;
    default: break;
  }
  playMelody(M_CLICK, false); drawInfo();
}
void onVeryLongPress() {       // > 2 s
  wakeScreens(); noteInput();   // the menu opens either way
  if (alarmRinging) { alarmOff(); }
  menuOpen = true; menuSel = 0; for (int i = 0; i < menuCount; i++) if (menuItems[i] == currentApp) menuSel = i;
  drawMenu(); drawInfo(); beep(700, 60);
}
// ---- v6 two-knob layer (the v5 handlers above and below are the right knob, unchanged) ----
// Pomodoro start / pause from any app (left double click). Same state changes as the v5 POMODORO click.
void pomoQuickToggle() {
  const char *what = NULL;
  if (pomoAskLog || pomoSettings) return;
  if (pomoState == POMO_SELECT) { pomoEditing = false; pomoStart(true); what = "started"; }
  else if (pomoState == POMO_PAUSED) { pomoEndMs = millis() + pomoRemainAtPause; pomoState = pomoWasFocus ? POMO_FOCUS : POMO_BREAK; what = "resumed"; }
  else if (pomoState == POMO_FOCUS) { pomoRemainAtPause = pomoRemainMs(); pomoWasFocus = true; pomoState = POMO_PAUSED; pomoPausedAtMs = millis(); what = "paused"; }
  if (!what) return;
  staticDirty = true; settingsDirty = true;
  raiseAlert("POMODORO", what, COL_ARR, -1, 1500, NULL);
}
// Left click: back to the current app's home view (or select in the menu). Double click: Pomodoro.
void onLeftClick() {
  unsigned long now = millis();
  bool dbl = knobs[0].lastClickMs && now - knobs[0].lastClickMs < POMO_DOUBLE_TAP_MS;
  knobs[0].lastClickMs = 0;   // 6.14.1: re-armed below only after a plain click (not menu select / mic mute)
  fxPulse(0);
  wakeScreens(); if (noteInput()) { drawInfo(); return; }
  if (alarmRinging) { alarmSnooze(); drawInfo(); return; }
  if (ovActive()) { ovEnd(); return; }
  if (menuOpen) { onShortPress(); return; }
  if (onAir() && micClick && !dbl) { macQueue(mac.muted ? "unmute" : "mute"); mac.muted = !mac.muted; strlcpy(toast.l1, mac.muted ? "MIC MUTED" : "MIC LIVE", sizeof(toast.l1)); toast.l2[0] = 0; toast.col = mac.muted ? COL_DEP : COL_SELECTED; toast.untilMs = millis() + 1500; toast.icao[0] = 0; staticDirty = true; return; }   // on a call: left click = Mac mic mute
  if (dbl) { pomoQuickToggle(); drawInfo(); return; }
  knobs[0].lastClickMs = now;
  if (currentApp == APP_RADAR) currentMode = MODE_ZOOM;
  enterApp(currentApp); playMelody(M_CLICK, false);
}
// Left turn: next / previous app in menu order (hidden apps are not in the menu); moves the list when open.
void handleLeftRotation() {
  int delta = knobSteps(0);
  if (delta == 0) return;
  fxTick(0, delta > 0 ? 1 : -1); navMs = millis();
  if (knobs[1].down) {   // 6.17.0: right knob held + left turn = round display brightness (the release is swallowed)
    int v = tftBright + 5 * (delta > 0 ? 1 : -1); if (v < 10) v = 10; if (v > 100) v = 100; tftBright = v; applyBrightness();
    knobs[1].longFired = true; brightMs = millis(); brightWhat = 0; settingsDirty = true; staticDirty = true; drawInfo(); return;
  }
  wakeScreens(); if (noteInput()) { staticDirty = true; drawInfo(); return; }
  int step = delta > 0 ? 1 : -1;
  if (alarmRinging) { alarmSnooze(); drawInfo(); return; }
  if (ovActive()) { ovEnd(); return; }
  if (menuOpen) { menuSel = (menuSel + step + menuCount) % menuCount; drawMenu(); drawInfo(); return; }
  if (menuCount <= 0) return;
  int cur = 0; for (int i = 0; i < menuCount; i++) if (menuItems[i] == currentApp) cur = i;
  int a = menuItems[(cur + step + menuCount) % menuCount];
  if (a == APP_LOCATION) locTarget = 0;
  enterApp(a);
  if (a == APP_LOCATION) listSel = homeIndex;
}
// Buttons. Left: click (< 2 s) = home / menu select, hold 2 s = menu. Right: v5 timing, click < 0.6 s,
// hold 0.6 s or more = the v5 long press (released), so the menu stays on the left knob only.
void handleButton() {
  unsigned long now = millis();
  for (int k = 0; k < 2; k++) {
    Knob &n = knobs[k];
    bool down = digitalRead(k ? ENC_R_SW : ENC_L_SW) == LOW;
    if (down && !n.down) { n.down = true; n.downMs = now; n.longFired = false; navMs = now; }
    if (k == 0) fxHold = (down && !n.longFired && now - n.downMs > 300) ? fminf(1.0f, (now - n.downMs - 300) / 1700.0f) : 0;   // ring fills during the 2 s hold
    if (down && n.down && k == 0 && !n.longFired && now - n.downMs > 2000) { n.longFired = true; fxHold = 0; onVeryLongPress(); }
    else if (!down && n.down) {
      n.down = false; navMs = now;
      unsigned long held = now - n.downMs;
      if (n.longFired || held < 40) continue;
      if (k == 0) onLeftClick();
      else if (held < 600) onShortPress(); else onLongPress();
    }
  }
  // 6.16.0: both knobs pressed together = SETTINGS (the releases are swallowed)
  static bool chord = false;
  if (knobs[0].down && knobs[1].down && !chord) {
    chord = true; knobs[0].longFired = knobs[1].longFired = true; fxHold = 0; navMs = now; wakeScreens();
    if (currentApp != APP_SETTINGS || menuOpen) enterApp(APP_SETTINGS); else { setCat = -1; setSel = 0; setEdit = false; staticDirty = true; }
    beep(900, 50); drawInfo();
  }
  if (!knobs[0].down && !knobs[1].down) chord = false;
}
void handleRotation() {   // right knob: the v5 rotation handler
  int delta = knobSteps(1);
  if (delta == 0) return;
  fxTick(1, delta > 0 ? 1 : -1); navMs = millis();
  if (knobs[0].down) {   // 6.17.0: left knob held + right turn = ring brightness (the hold no longer opens the menu)
    int v = ringBright + 5 * (delta > 0 ? 1 : -1); if (v < 0) v = 0; if (v > 100) v = 100; ringBright = v;
    knobs[0].longFired = true; fxHold = 0; brightMs = millis(); brightWhat = 1; settingsDirty = true; drawInfo(); return;
  }
  wakeScreens(); if (noteInput()) { staticDirty = true; drawInfo(); return; }
  int step = (delta > 0) ? 1 : -1; int nd = abs(delta);
  int fast = (nd > 1 || millis() - lastRotMs < 80) ? (nd > 4 ? nd : 4) : 1; lastRotMs = millis();   // knob acceleration for numeric settings
  if (alarmRinging) { alarmSnooze(); drawInfo(); return; }
  if (ovActive()) { ovEnd(); return; }
  if (menuOpen) { menuSel = (menuSel + step + menuCount) % menuCount; drawMenu(); drawInfo(); return; }
  switch (currentApp) {
    case APP_RADAR:
      if (currentMode == MODE_ZOOM) { int r = rangeIndex + step; if (r < 0) r = 0; if (r > RANGE_AUTO) r = RANGE_AUTO; if (r != rangeIndex) { rangeIndex = r; settingsDirty = true; alertArmed = false; } }
      else if (planeCount > 0) { int tries = 0; do { selectedIndex = (selectedIndex + step + planeCount) % planeCount; } while (!passesFilter(planes[selectedIndex]) && ++tries < planeCount); strlcpy(selectedIcao, planes[selectedIndex].icao24, sizeof(selectedIcao)); }
      break;
    case APP_ARRIVALS: case APP_APPROACH: listSel += step; if (listSel < 0) listSel = 0; break;
    case APP_CLOCK: clockFace = (clockFace + step + NUM_FACES) % NUM_FACES; settingsDirty = true; fadeK = 0.3f; break;
    case APP_WEATHER: subPage = (subPage + step + 3) % 3; break;
    case APP_REPLAY: { replayPlay = false; int m = replayMaxAge(); replayPos -= step * fast; if (replayPos < 0) replayPos = 0; if (replayPos > m) replayPos = m; break; }
    case APP_POMODORO:
      if (pomoSettings) { if (psetEdit) psetStep(psetSel, step); else psetSel = (psetSel + step + PSET_N) % PSET_N; break; }
      if (pomoAskLog) { pomoAskSel ^= 1; break; }
      if (pomoState == POMO_SELECT) {
        if (pomoEditing && pomoPreset == POMO_TIMER_PRESET) { pomoTimerMin += step * fast; if (pomoTimerMin < 1) pomoTimerMin = 1; if (pomoTimerMin > 180) pomoTimerMin = 180; }
        else if (pomoEditing) { pomoCustomFocus += 5 * step; if (pomoCustomFocus < 5) pomoCustomFocus = 5; if (pomoCustomFocus > 180) pomoCustomFocus = 180; }
        else pomoPreset = (pomoPreset + step + NUM_POMO) % NUM_POMO;
        settingsDirty = true;
      }
      break;
    case APP_MARKET: { int n = marketPages(); marketSel = (marketSel + step + n) % n; settingsDirty = true; if (marketSel < NUM_SYMBOLS) marketForce = true; break; }
    case APP_STATS: subPage = (subPage + step + 3) % 3; break;
    case APP_SYSTEM: subPage = (subPage + step + 3) % 3; break;
    case APP_NEWS: newsScroll += step; if (newsScroll < 0) newsScroll = newsCount ? newsCount - 1 : 0; if (newsScroll >= newsCount) newsScroll = 0; break;
    case APP_CRICKET: subPage = (subPage + step + 2) % 2; break;
    case APP_BRIGHT:
      if (brightSel == 0) { tftBright += 5 * step * fast; if (tftBright < 10) tftBright = 10; if (tftBright > 100) tftBright = 100; applyBrightness(); }
      else if (brightSel == 1) { oledBright += 5 * step * fast; if (oledBright < 0) oledBright = 0; if (oledBright > 100) oledBright = 100; applyBrightness(); }
      else if (brightSel == 2) { theme = (theme + step + 4) % 4; applyPalette(); }
      else if (brightSel == 4) { oledScale = (oledScale + step + 4) % 4; }   // 6.14.1: settingsDirty saves it (was a full NVS write per detent)
      else sweepOn = !sweepOn;
      settingsDirty = true; break;
    case APP_LOCATION: { int n = locCount(); listSel = ((listSel + step * fast) % n + n) % n; break; }
    case APP_AGENDA: { int n = agendaList(NULL, MAX_CAL); listSel += step; if (listSel >= n) listSel = n ? n - 1 : 0; if (listSel < 0) listSel = 0; break; }
    case APP_SETTINGS: settingsTurn(step, fast); break;
    case APP_MEDIA: { for (int i = 0; i < (fast > 1 ? 2 : 1); i++) macQueue(step > 0 ? "vol+" : "vol-"); if (mac.vol >= 0) { mac.vol += 6 * step * (fast > 1 ? 2 : 1); if (mac.vol < 0) mac.vol = 0; if (mac.vol > 100) mac.vol = 100; } break; }
    default: break;
  }
  staticDirty = true; drawInfo();
}

// =====================================================================
// SETUP / LOOP
// =====================================================================
void setup() {
  logRing = (char *)heap_caps_malloc(LOG_RING, MALLOC_CAP_SPIRAM);
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);   // v6: with USB plugged in but no reader, every Serial write blocked up to 100 ms
  bootMs = millis(); lastInputMs = bootMs;
  resetReasonCode = (int)esp_reset_reason();
  secureClient.setInsecure();
  dataMutex = xSemaphoreCreateMutex(); dnsMutex = xSemaphoreCreateMutex(); feedMutex = xSemaphoreCreateMutex(); tzMutex = xSemaphoreCreateMutex();
  loadSettings(); loadPomoHist(); loadFocusQ(); loadFocusPro(); loadOutages(); setSyncMirrors();
  memset(routes, 0, sizeof(routes)); memset(routesUI, 0, sizeof(routesUI)); memset(trails, 0, sizeof(trails));

  pinMode(TFT_CS_L, OUTPUT); pinMode(TFT_CS_R, OUTPUT); tftSelect(-1);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encL.attachFullQuad(ENC_L_A, ENC_L_B); encL.setFilter(1023); encL.clearCount();
  encR.attachFullQuad(ENC_R_A, ENC_R_B); encR.setFilter(1023); encR.clearCount();
  pinMode(ENC_L_SW, INPUT_PULLUP); pinMode(ENC_R_SW, INPUT_PULLUP);
#if BUZZER_PIN >= 0
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
#endif
  Wire.begin(I2C0_SDA, I2C0_SCL, 400000); Wire1.begin(I2C1_SDA, I2C1_SCL, 400000);
  Wire.setTimeOut(20); Wire1.setTimeOut(20);   // a stuck bus must not stall the UI
  if (!infoDisplay.begin(SSD1306_SWITCHCAPVCC, 0x3C, true, false)) logln("SSD1306 left init failed");
  if (!infoR.begin(SSD1306_SWITCHCAPVCC, 0x3C, true, false)) logln("SSD1306 right init failed");
  infoDisplay.setRotation(2); infoR.setRotation(2);   // both mounted upside down
  infoDisplay.clearDisplay(); oledPush(0); infoR.clearDisplay(); oledPush(1);
  ringL.begin(); ringR.begin(); ringL.clear(); ringR.clear(); ringL.show(); ringR.show();   // 6.17.0: the boot task drives the rings

  // round displays: one shared reset pulse, init both at once, then each on its own (the sequence that
  // brought both up in bring-up), then the PSRAM canvases
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, HIGH); delay(5); digitalWrite(TFT_RST, LOW); delay(20); digitalWrite(TFT_RST, HIGH); delay(150);
  digitalWrite(TFT_CS_L, LOW); digitalWrite(TFT_CS_R, LOW); tftL->begin(TFT_SPI_HZ); tftSelect(-1);
  tftSelect(0); tftL->begin(TFT_SPI_HZ); tftSelect(-1);
  tftSelect(1); tftR->begin(TFT_SPI_HZ); tftSelect(-1);
  if (!radarL->begin() || !radarR->begin()) logln("Canvas alloc failed (PSRAM)");
  gfxInit();   // from here on the panels belong to the ESP-IDF driver (Arduino_GFX only ran the init sequence)
  sdInit();
  envInit();
  applyPalette(); applyBrightness();
  bootAnim = true; bootStartMs = millis(); xTaskCreatePinnedToCore(bootTask, "boot", 8192, NULL, 1, NULL, 1);   // 6.17.0: animated boot on both dials and the rings
  bootStep("display", sdOk ? "card ok" : "no card");
  bootStep("WiFi", "connecting");
  logf("Heap after canvas: %u, largest %u, PSRAM free %u\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getFreePsram());

  connectWiFi();
  bootStep("WiFi", wifiUp ? WiFi.localIP().toString().c_str() : "no network");
  configTzTime(LOCAL_TZ, "pool.ntp.org", "time.google.com");
  bootStep("clock", "syncing");
  if (MDNS.begin(OTA_HOSTNAME)) MDNS.addService("http", "tcp", 80);
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.onStart([]() { oledMsg("OTA update...", ""); showMessage("OTA UPDATE", "", "", 0); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) { esp_task_wdt_reset(); static unsigned last = 0; unsigned pct = (100 * p) / t; if (pct != last) { last = pct; showMessage("OTA UPDATE", "", "", pct); } });
  ArduinoOTA.begin();
  server.on("/", handleRoot); server.on("/api/state", handleState); server.on("/set", handleSet);
  shotServer.begin(); shotServer.setNoDelay(true);
  xTaskCreatePinnedToCore(shotTask, "shot", 6144, NULL, 1, NULL, 0);
  server.on("/restart", []() { server.send(200, "text/plain", "restarting"); delay(300); saveSettings(); ESP.restart(); });
  server.on("/log", handleLog); server.on("/screen.bmp", handleScreen); server.on("/oled.bmp", handleOled);
  server.on("/update", HTTP_GET, handleUpdatePage); server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  server.on("/mac", HTTP_POST, handleMac);
  server.on("/sdput", HTTP_POST, handleSdPutDone, handleSdPutUpload); server.on("/sdls", handleSdList); server.on("/i2c", handleI2c); server.on("/coredump", handleCoredump); server.on("/dns", handleDns);
  // test aid (6.7.0): /knob?side=0|1&turn=n | &click=1 | &hold=1 injects knob input (left 0, right 1)
  server.on("/knob", []() {
    int side = server.arg("side").toInt() ? 1 : 0; navMs = millis();   // 6.14.2: the test hook shows the legend like a real knob
    if (server.hasArg("turn")) knobInject[side] = server.arg("turn").toInt();
    if (server.hasArg("click")) { if (side) onShortPress(); else onLeftClick(); }
    if (server.hasArg("hold")) { if (side) onLongPress(); else onVeryLongPress(); }
    server.send(200, "text/plain", "ok");
  });
  // test aid (6.3.0): switch app like the left knob, so every screen can be checked from the Mac
  server.on("/app", []() { int n = server.arg("n").toInt(); if (n >= 0 && n < APP_COUNT) { enterApp(n); server.send(200, "text/plain", APP_NAMES[n]); } else server.send(400, "text/plain", "bad app"); });
  server.on("/live", handleLive); server.on("/config.json", handleConfig); server.on("/import", HTTP_GET, handleImportPage); server.on("/import", HTTP_POST, handleImport);
  server.begin();

  buildMenu();
  bootStep("services", "web, OTA, feeds");
  if (bootApp > 0 && bootApp < APP_COUNT && (appMask & (1u << bootApp)) && !appRetired(bootApp)) currentApp = (App)bootApp;   // SETTINGS > APPS > boot app
  arenaForApp();
  // main-loop watchdog: if the UI loop ever hangs (SPI/DMA, mutex) the board restarts within 30 s and logs WATCHDOG as the reset reason
  { esp_task_wdt_config_t wcfg = {}; wcfg.timeout_ms = 30000; wcfg.idle_core_mask = 0; wcfg.trigger_panic = true; esp_task_wdt_reconfigure(&wcfg); esp_task_wdt_add(NULL); }
  // 6.6.2: both fetch stacks in PSRAM (44 KB of internal RAM back for two TLS sessions at once). These tasks must
  // never write flash (NVS / OTA): with an external stack ESP-IDF aborts. Flash writes stay on loop().
  if (xTaskCreatePinnedToCoreWithCaps(fetchTask, "fetch", 28672, NULL, 1, NULL, 0, MALLOC_CAP_SPIRAM) != pdPASS) xTaskCreatePinnedToCore(fetchTask, "fetch", 28672, NULL, 1, NULL, 0);
  if (xTaskCreatePinnedToCoreWithCaps(planesTask, "planes", 16384, NULL, 2, NULL, 0, MALLOC_CAP_SPIRAM) != pdPASS) xTaskCreatePinnedToCore(planesTask, "planes", 16384, NULL, 2, NULL, 0);
  isNight = computeNight(); applyBrightness();
  staticDirty = true; alertArmed = false;
  beep(1500, 60);
  drawInfo();
  if (resetReasonCode == 1) heapGuardRestarts = 0;
  logf("Heap after setup: %u  largest block %u  PSRAM free %u  reset reason %d (%s)\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getFreePsram(), resetReasonCode, resetReasonCode == 1 ? "power on" : resetReasonCode == 3 ? "software/OTA" : resetReasonCode == 4 ? "PANIC - crash" : (resetReasonCode >= 5 && resetReasonCode <= 7) ? "WATCHDOG" : resetReasonCode == 9 ? "BROWNOUT - power supply" : "other");
  bootStep("ready", ""); vTaskDelay(pdMS_TO_TICKS(500)); bootStop();   // 6.17.0
}

void loop() {
  esp_task_wdt_reset();
  ArduinoOTA.handle();
  server.handleClient();
  wifiWatchdog(); netNoticeTick();
  handleButton();
  handleLeftRotation();
  handleRotation();
  syncFromFetchTask();
  melodyTick(); pomoTick(); ovTick(); greetTick(); recapTick(); nudgeTick(); alarmTick(); sleepTick(); summaryTick(); chimeTick(); heapTick(); roamTick(); slideTick();
  if (settingsDirty) { if (settingsDirtyMs == 0) settingsDirtyMs = now0(); else if (now0() - settingsDirtyMs > 3000) { saveSettings(); settingsDirtyMs = 0; } }

  unsigned long now = millis();
  static unsigned long lastEvents = 0;
  if (now - lastEvents >= 10000) { lastEvents = now; weatherEvents(); marketEvents(); quakeEvents(); }
  if (now - lastClockMs >= 60000) { lastClockMs = now; bool n = computeNight(); if (n != isNight) { isNight = n; applyBrightness(); staticDirty = true; logf("night mode %s (%02d:00-%02d:00)\n", n ? "on" : "off", nightStart, nightEnd); } }
  static unsigned long lastFadeMs = 0;
  if (fadeK < 1.0f && now - lastFadeMs >= 25) { lastFadeMs = now; fadeK += 0.12f; if (fadeK >= 1.0f) fadeK = 1.0f; staticDirty = true; clkS = -1; }
  if (appCardMs && now - appCardMs < 950) { static unsigned long lastCard = 0; if (now - lastCard >= 40) { lastCard = now; staticDirty = true; } }   // the app card animates on static pages
  static bool toastWas = false; bool toastNow = toastActive();
  if (toastWas && !toastNow) { staticDirty = true; drawInfo(); }
  toastWas = toastNow;

  if (!sleeping && !menuOpen) {
    if (ovActive()) drawOverlayFrame(); else
    switch (currentApp) {
      case APP_RADAR:    drawRadarFrame(); break;
      case APP_ARRIVALS: drawArrivalsFrame(); break;
      case APP_APPROACH: drawApproachFrame(); break;
      case APP_REPLAY:   drawReplayFrame(); break;
      case APP_F1:       drawF1Frame(); break;
      case APP_CLOCK:    drawClockFrame(); break;
      case APP_WEATHER:  drawWeatherFrame(); break;
      case APP_POMODORO: drawPomodoroFrame(); break;
      case APP_MARKET:   drawMarketFrame(); break;
      case APP_NETWORK:  drawNetworkFrame(); break;
      case APP_STATS:    drawStatsFrame(); break;
      case APP_NEWS:     drawNewsFrame(); break;
      case APP_CRICKET:  drawCricketFrame(); break;
      case APP_SYSTEM:   drawSystemFrame(); break;
      case APP_BRIGHT:   drawBrightFrame(); break;
      case APP_LOCATION: drawLocationFrame(); break;
      case APP_AGENDA:   drawAgendaFrame(); break;
      case APP_SETTINGS: drawSettingsFrame(); break;
      case APP_MEDIA:    drawMediaFrame(); break;
      default: break;
    }
    if (toastNow && currentApp != APP_RADAR && now - lastHudMs > 250) { lastHudMs = now; if (currentPaint) present(currentPaint); }   // keep the toast visible on static pages
    static int frames = 0; static unsigned long fpsT0 = 0; frames++;
    if (now - fpsT0 >= 1000) { fpsShown = frames; frames = 0; fpsT0 = now; }
  } else if (sleeping && menuOpen) menuOpen = false;
  bool ticker = tickerApp() && newsCount > 0 && !menuOpen && !toastNow && !sleeping;
  unsigned long infoEvery = ticker ? 90 : 1000;
  if (now - lastInfoMs >= infoEvery) { lastInfoMs = now; if (ticker) tickerPx += 2; drawInfo(); }
  drawCompanionFrame();
  ringsTick();
  if (outagesDirty) { outagesDirty = false; saveOutages(); }
  envPoll();
  calTick();
  issTick();
  static unsigned long lastT = 0;
  if (now - lastT >= 60000) {
    lastT = now;
    for (int d = 0; d < 2; d++) if (tFrames[d]) logf("frames %s: %lu in 60 s (%.1f fps), paint %lu us, send %lu us, wait %lu us avg\n", d ? "R" : "L", tFrames[d], tFrames[d] / 60.0f, tPaintUs[d] / tFrames[d], tFlushUs[d] / tFrames[d], tWaitUs[d] / tFrames[d]);
    tPaintUs[0] = tPaintUs[1] = tFlushUs[0] = tFlushUs[1] = tWaitUs[0] = tWaitUs[1] = tFrames[0] = tFrames[1] = 0;
  }
  delay(1);
}

// =====================================================================
// v6 RIGHT DIAL + RIGHT OLED + RINGS (stage 1: companion; per-app detail views follow in stages 2 to 4)
// =====================================================================
void envInit() {
  for (uint8_t a : {(uint8_t)0x76, (uint8_t)0x77}) {
    Wire.beginTransmission(a); Wire.write(0xD0); if (Wire.endTransmission() != 0) continue;
    if (Wire.requestFrom(a, (uint8_t)1) != 1) continue;
    uint8_t id = Wire.read();
    if (id == 0x58 && bmp.begin(a, 0x58)) { envAddr = a; envChip = id; break; }
    if (id == 0x60 && bme.begin(a, &Wire)) { envAddr = a; envChip = id; break; }
    logf("env: 0x%02X answers with chip id 0x%02X\n", a, id);
  }
  if (envAddr) logf("env: %s at 0x%02X\n", envChip == 0x60 ? "BME280" : "BMP280", envAddr); else logln("env: no BMP280 / BME280 at 0x76 / 0x77");
}
void envPoll() {   // every 5 s; retries detection every 60 s while missing
  static unsigned long last = 0, retry = 0; unsigned long now = millis();
  if (!envAddr) { if (now - retry > 60000) { retry = now; envInit(); } return; }
  if (now - last < 5000) return; last = now;
  if (envChip == 0x60) { envT = bme.readTemperature(); envP = bme.readPressure() / 100.0f; envH = bme.readHumidity(); }
  else { envT = bmp.readTemperature(); envP = bmp.readPressure() / 100.0f; }
  if (envT < -40 || envT > 85 || envP < 300 || envP > 1100) { static unsigned long lw = 0; if (now - lw > 60000) { lw = now; logf("env: implausible %.1f C %.1f hPa\n", envT, envP); } envT = envP = NAN; return; }
  envMs = now;
  static unsigned long lg = 0; if (!lg || now - lg > 600000) { lg = now; logf("env: %.1f C  %.1f hPa%s\n", envT, envP, envChip == 0x60 ? "" : ""); }
}
// Last crash from the core-dump partition (survives the reboot): task, PC, backtrace. Decode the addresses with
// xtensa-esp32s3-elf-addr2line -e build6/desk_plane_radar_v6.ino.elf <addrs>
void handleCoredump() {
  esp_core_dump_summary_t *sm = (esp_core_dump_summary_t *)malloc(sizeof(esp_core_dump_summary_t));
  if (!sm) { server.send(500, "text/plain", "no memory"); return; }
  if (server.hasArg("clear")) { free(sm); esp_core_dump_image_erase(); server.send(200, "text/plain", "core dump erased"); return; }
  esp_err_t e = esp_core_dump_get_summary(sm);
  if (e != ESP_OK) { free(sm); server.send(200, "text/plain", String("no core dump: ") + esp_err_to_name(e)); return; }
  char b[64]; String o;
  snprintf(b, sizeof(b), "task %s  pc 0x%08lx  cause %lu  vaddr 0x%08lx\n", sm->exc_task, (unsigned long)sm->exc_pc, (unsigned long)sm->ex_info.exc_cause, (unsigned long)sm->ex_info.exc_vaddr); o += b;
  o += "backtrace:"; for (uint32_t i = 0; i < sm->exc_bt_info.depth && i < 16; i++) { snprintf(b, sizeof(b), " 0x%08lx", (unsigned long)sm->exc_bt_info.bt[i]); o += b; }
  o += sm->exc_bt_info.corrupted ? "  (corrupted)\n" : "\n";
  o += "elf sha256 "; for (int i = 0; i < 8; i++) { snprintf(b, sizeof(b), "%02x", sm->app_elf_sha256[i]); o += b; } o += "\n";
  free(sm); server.send(200, "text/plain", o);
}
void handleI2c() {   // scan both buses + the sensor reading (diagnostic)
  String o; char b[64];
  for (int bus = 0; bus < 2; bus++) { TwoWire &w = bus ? Wire1 : Wire; o += bus ? "bus 1 (SDA 1, SCL 2):" : "bus 0 (SDA 8, SCL 9):";
    for (uint8_t a = 0x08; a < 0x78; a++) { w.beginTransmission(a); if (w.endTransmission() == 0) { snprintf(b, sizeof(b), " 0x%02X", a); o += b; } } o += "\n"; }
  if (envAddr) { snprintf(b, sizeof(b), "%s at 0x%02X: %.2f C  %.2f hPa", envChip == 0x60 ? "BME280" : "BMP280", envAddr, envT, envP); o += b; if (envChip == 0x60) { snprintf(b, sizeof(b), "  %.1f %%RH", envH); o += b; } o += "\n"; }
  else o += "no BMP280 / BME280\n";
  server.send(200, "text/plain", o);
}
// ===== SKY (6.11.0): look up + ISS =====
// Pointer the rings follow for a while: bearing / elevation from the house, label for the OLED band
struct SkyPoint { float az, el; unsigned long untilMs; char label[24]; bool iss; };
SkyPoint sky = {0, 0, 0, "", false};
bool skyActive() { if (sky.untilMs && (long)(millis() - sky.untilMs) >= 0) sky.untilMs = 0; return sky.untilMs != 0; }   // 6.14.1: wrap-safe
const char *compass8(float brg) { static const char *N[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"}; return N[((int)((brg + 22.5f) / 45.0f)) % 8]; }
float houseElevM() { return AIRPORTS[homeIndex].elevM; }   // the house is at the airport's elevation give or take
// Azimuth / elevation of a point (lat, lon, altitude m) seen from the house
void azElFrom(double lat, double lon, double altM, float &az, float &el) {
  auto ecef = [](double la, double lo, double h, double &x, double &y, double &z) { double a = 6378137.0, e2 = 6.69437999e-3, sl = sin(la * M_PI / 180), cl = cos(la * M_PI / 180); double N = a / sqrt(1 - e2 * sl * sl);
    x = (N + h) * cl * cos(lo * M_PI / 180); y = (N + h) * cl * sin(lo * M_PI / 180); z = (N * (1 - e2) + h) * sl; };
  double ox, oy, oz, px, py, pz; ecef(wxHomeLat, wxHomeLon, houseElevM(), ox, oy, oz); ecef(lat, lon, altM, px, py, pz);
  double dx = px - ox, dy = py - oy, dz = pz - oz, la = wxHomeLat * M_PI / 180, lo = wxHomeLon * M_PI / 180;
  double e = -sin(lo) * dx + cos(lo) * dy, n = -sin(la) * cos(lo) * dx - sin(la) * sin(lo) * dy + cos(la) * dz, u = cos(la) * cos(lo) * dx + cos(la) * sin(lo) * dy + sin(la) * dz;
  az = fmodf((float)(atan2(e, n) * 180 / M_PI) + 360.0f, 360.0f); el = (float)(atan2(u, sqrt(e * e + n * n)) * 180 / M_PI);
}
// Look up: an aircraft within lookupKm of the house, below 12,000 ft, once per aircraft per 20 min
struct Looked { char icao[8]; unsigned long ms; }; Looked looked[8]; int lookedN = 0;
void lookUpTick() {
  if (!lookupOn) return;
  for (int i = 0; i < planeCount; i++) {
    const Plane &p = planes[i]; if (p.altitude_m * 3.28084f > 12000) continue;
    float d = distanceKm(wxHomeLat, wxHomeLon, p.lat, p.lon); if (d > lookupKm) continue;
    bool seen = false; for (int k = 0; k < lookedN; k++) if (!strcmp(looked[k].icao, p.icao24) && millis() - looked[k].ms < 1200000UL) seen = true;
    if (seen) continue;
    static int lookedW = 0; Looked &L = looked[lookedW]; lookedW = (lookedW + 1) % 8; strlcpy(L.icao, p.icao24, 8); L.ms = millis(); if (lookedN < 8) lookedN++;   // 6.14.1: ring cursor (slot 0 was overwritten once full: repeated alerts)
    float az, el; azElFrom(p.lat, p.lon, p.altitude_m, az, el);
    char pcs[12]; prettyCallsign(p.callsign, pcs, sizeof(pcs)); char l2[32]; snprintf(l2, sizeof(l2), "%s  %s  %.0f%s up", pcs, compass8(az), el, DEG);
    raiseAlert("LOOK UP", l2, COL_DEP, M_OVERHEAD, 15000, p.icao24);
    sky.az = az; sky.el = el; sky.untilMs = millis() + 60000; sky.iss = false; snprintf(sky.label, sizeof(sky.label), "LOOK UP %s %.0f%s", compass8(az), el, DEG);
    logf("look up: %s %.1f km %s %.0f deg\n", pcs, d, compass8(az), el);
  }
}
// ISS: positions for the next 50 min (10 timestamps, 5 min apart) every 10 min; every 30 s while a pass is near
IssPass issPass = {};
unsigned long issFetchMs = 0; float issLiveAz = 0, issLiveEl = -90; unsigned long issLiveMs = 0;
bool fetchIss() {
  if (!wifiUp) return false;
  uint32_t now = (uint32_t)time(NULL); if (now < 1700000000) return false;
  bool near = issPass.valid && now + 900 >= issPass.startT && now <= issPass.maxT + 360;
  String ts; int step = near ? 30 : 300; for (int i = 0; i < 10; i++) { if (i) ts += ","; ts += String((unsigned long)(now + i * step)); }
  WiFiClientSecure tls; tls.setInsecure(); tls.setHandshakeTimeout(15); HTTPClient http; http.useHTTP10(true);
  httpBegin(http, tls, "https://api.wheretheiss.at/v1/satellites/25544/positions?timestamps=" + ts + "&units=kilometers"); http.setTimeout(10000);
  http.setUserAgent("DeskRadar/6 (personal desk display)");
  int code = httpGet(http, "iss"); if (code != 200) { http.end(); return false; }
  JsonDocument filter; filter[0]["timestamp"] = true; filter[0]["latitude"] = true; filter[0]["longitude"] = true; filter[0]["altitude"] = true; filter[0]["visibility"] = true;
  JsonDocument doc; PatientStream ps(http.getStream()); ps.setTimeout(4000);
  DeserializationError e = deserializeJson(doc, ps, DeserializationOption::Filter(filter)); http.end();
  if (e) { logf("iss parse: %s\n", e.c_str()); return false; }
  struct tm lt; bool haveT = getTime(lt); int nowMin = haveT ? lt.tm_hour * 60 + lt.tm_min : 0;
  bool observerDark = wx.sunriseMin < 0 || nowMin < wx.sunriseMin - 30 || nowMin > wx.sunsetMin + 30;
  IssPass np = {}; float bestEl = -90; bool first = true;
  for (JsonObject o : doc.as<JsonArray>()) {
    float az, el; azElFrom(o["latitude"] | 0.0, o["longitude"] | 0.0, (o["altitude"] | 420.0) * 1000.0, az, el);
    uint32_t t = o["timestamp"] | 0UL; const char *vis = o["visibility"] | "";
    if (first) { issLiveAz = az; issLiveEl = el; issLiveMs = millis(); first = false; }
    if (el >= 10 && !np.valid) { np.valid = true; np.startT = t; }
    if (el > bestEl) { bestEl = el; if (np.valid) { np.maxT = t; np.maxEl = el; np.azMax = az; np.visible = !strcmp(vis, "visible") || (!strcmp(vis, "daylight") && observerDark); } }
  }
  xSemaphoreTake(dataMutex, portMAX_DELAY); if (np.valid || !near) issPass = np; xSemaphoreGive(dataMutex);
  if (np.valid) { time_t st = np.startT; struct tm pt; localtime_r(&st, &pt); logf("iss: pass %02d:%02d max %.0f deg %s %s\n", pt.tm_hour, pt.tm_min, np.maxEl, compass8(np.azMax), np.visible ? "visible" : "not visible"); }
  return true;
}
void issTick() {   // UI task: during a visible pass, point the rings and announce it once
  if (!issOn || !issPass.valid || !issPass.visible) return;
  uint32_t now = (uint32_t)time(NULL); static uint32_t announced = 0;
  if (now >= issPass.startT && now <= issPass.maxT + 300 && issLiveEl > 5 && millis() - issLiveMs < 120000) {
    sky.az = issLiveAz; sky.el = issLiveEl; sky.untilMs = millis() + 40000; sky.iss = true; snprintf(sky.label, sizeof(sky.label), "ISS %s %.0f%s", compass8(issLiveAz), issLiveEl, DEG);
    if (announced != issPass.startT) { announced = issPass.startT; char l2[32]; snprintf(l2, sizeof(l2), "look %s, up to %.0f%s", compass8(issPass.azMax), issPass.maxEl, DEG); raiseAlert("SPACE STATION", l2, COL_ENROUTE_HIGH, M_SUN, 15000, NULL); }
  }
}
// ===== Mac helper link (6.4.0): tools/mac_helper. The Mac posts its state to /mac about once a second
// (mic / camera in use, mute, volume, now playing) and the calendar when it changes; the reply carries the
// commands queued by the knobs. Everything stays on the home network. =====
struct CalEvent { uint32_t t, e; char title[48]; char cal[20]; bool allday; };
#define MAX_CAL 12
CalEvent calEv[MAX_CAL]; int calN = 0; unsigned long calMs = 0;
MacState mac = {};
unsigned long macMs = 0; String macCmds;
void macQueue(const char *c) { if (macCmds.length() > 200) return; if (macCmds.length()) macCmds += ","; macCmds += "\""; macCmds += c; macCmds += "\""; }
bool macLinked() { return macMs && millis() - macMs < 10000; }
bool onAir() { return onAirOn && macLinked() && (mac.mic || mac.cam); }
const char *calTitle(const CalEvent &c) { return meetTitles ? c.title : "Meeting"; }
void handleMac() {
  JsonDocument d;
  if (deserializeJson(d, server.arg("plain"))) { server.send(400, "text/plain", "bad json"); return; }
  bool wasAir = onAir();
  mac.mic = d["mic"] | false; mac.cam = d["cam"] | false; mac.muted = d["muted"] | false; mac.playing = d["playing"] | false;
  mac.vol = d["vol"] | -1; strlcpy(mac.np, d["np"] | "", sizeof(mac.np)); strlcpy(mac.app, d["app"] | "", sizeof(mac.app));
  if (d["cal"].is<JsonArray>()) {
    calN = 0;
    for (JsonObject e : d["cal"].as<JsonArray>()) {
      if (calN >= MAX_CAL) break;
      CalEvent &c = calEv[calN]; c.t = e["s"] | 0UL; c.e = e["e"] | 0UL; c.allday = e["a"] | false;
      strlcpy(c.title, e["t"] | "", sizeof(c.title)); strlcpy(c.cal, e["c"] | "", sizeof(c.cal));
      bool dup = false; for (int k = 0; k < calN; k++) if (calEv[k].t == c.t && !strcmp(calEv[k].title, c.title)) dup = true;   // same meeting on two calendars
      if (!dup) calN++;
    }
    calMs = millis(); logf("mac: calendar %d events\n", calN);
  }
  if (!macMs) logln("mac: helper connected");
  macMs = millis();
  if (onAir() != wasAir) { logf("mac: %s\n", onAir() ? "ON AIR" : "off air"); staticDirty = true; }
  String r = "{\"cmd\":[" + macCmds + "]}"; macCmds = "";
  server.send(200, "application/json", r);
}
// Next timed event (not all-day) that has not started, and the one in progress (if any).
int calNext() { uint32_t now = time(NULL); int b = -1; for (int i = 0; i < calN; i++) if (!calEv[i].allday && calEv[i].t > now && (b < 0 || calEv[i].t < calEv[b].t)) b = i; return b; }
int calNow()  { uint32_t now = time(NULL); for (int i = 0; i < calN; i++) if (!calEv[i].allday && calEv[i].t <= now && now < calEv[i].e) return i; return -1; }
// Reminder banner + chime (not raiseAlert: meeting cues always show, pop-ups setting or not; quiet hours silence them)
void meetCue(const char *l1, const char *l2, int melody) {
  strlcpy(toast.l1, l1, sizeof(toast.l1)); strlcpy(toast.l2, l2, sizeof(toast.l2)); toast.col = COL_DEP; toast.untilMs = millis() + 8000; toast.icao[0] = 0;
  if (melody >= 0 && !quietNow()) playMelody(melody, false);
  logf("meeting cue: %s | %s\n", l1, l2); staticDirty = true; drawInfo();
}
// Cues at 5 min, 1 min, the start, and 5 min before the end. Keyed by start time + title so a calendar
// refresh does not repeat them.
void calTick() {
  static unsigned long last = 0; if (millis() - last < 1000) return; last = millis();
  { static bool lastFocus = false; bool f = pomoFocusActive();   // Focus sync (SETTINGS > POMODORO)
    if (f != lastFocus) { lastFocus = f; if (focusSync && macLinked()) macQueue(f ? "focus_on" : "focus_off"); } }
  if (!meetOn) return;
  uint32_t now = time(NULL); if (now < 1700000000 || !calN) return;
  static uint32_t done[24]; static uint8_t doneN = 0;
  auto key = [](const CalEvent &c, uint8_t k) { uint32_t h = c.t * 31u + k; for (const char *p = c.title; *p; p++) h = h * 131u + (uint8_t)*p; return h; };
  auto once = [&](uint32_t h) { for (int i = 0; i < doneN; i++) if (done[i] == h) return false; done[doneN % 24] = h; if (doneN < 24) doneN++; else { memmove(done, done + 1, 23 * 4); done[23] = h; } return true; };
  for (int i = 0; i < calN; i++) {
    const CalEvent &c = calEv[i]; if (c.allday) continue;
    long to = (long)c.t - (long)now, left = (long)c.e - (long)now;
    long lead = meetLead * 60L; char l1[16]; snprintf(l1, sizeof(l1), "IN %d MIN", meetLead);
    if (to > lead - 60 && to <= lead && once(key(c, 1))) meetCue(l1, calTitle(c), M_CHIME);
    else if (meetLead > 1 && to > 0 && to <= 60 && once(key(c, 2))) meetCue("IN 1 MIN", calTitle(c), M_NUDGE);
    else if (to <= 0 && to > -60 && once(key(c, 3))) meetCue("STARTING NOW", calTitle(c), M_POMO);
    else if (to < 0 && left > 240 && left <= 300 && c.e - c.t > 900 && once(key(c, 4))) meetCue("5 MIN LEFT", calTitle(c), M_CHIME);
  }
}
// Mount the card and append a boot line. Only /deskradar/ is ever written (the owner's card also holds phone files).
void sdInit() {
  sdOk = gfxReady && SD.mount();
  if (!sdOk) { logln("SD: no card (SPI CS 10 / MISO 13)"); return; }
  if (!SD.exists("/deskradar")) SD.mkdir("/deskradar");
  if (!SD.exists("/deskradar/log")) SD.mkdir("/deskradar/log");
  File f = SD.open("/deskradar/log/boot.txt", FILE_APPEND);
  bool w = f && f.printf("boot v%s reset %d heap %u psram %u\n", FW_VERSION, (int)esp_reset_reason(), ESP.getFreeHeap(), ESP.getFreePsram()) > 0;
  if (f) f.close();
  logf("SD: %s %llu MB, free %llu MB, boot line %s\n", SD.isSDHC() ? "SDHC" : "SD", SD.cardSize() >> 20, (SD.totalBytes() - SD.usedBytes()) >> 20, w ? "written" : "FAILED");
}
// ---- airline logos from the card: /deskradar/logos/<IATA>.lgo (tools/make_airline_logos.py) ----
// "LGO1", uint16 w, uint16 h, w*h RGB565, w*h alpha. Loaded into PSRAM from loop() (never inside a paint
// function, the card shares the display bus), then drawn from memory.
struct LogoEntry { char iata[4]; uint16_t w, h; uint16_t *px; uint8_t *al; bool missing; };
#define MAX_LOGOS 64
LogoEntry logoCache[MAX_LOGOS]; int logoCacheN = 0;
const LogoEntry *logoFind(const char *iata) {
  for (int i = 0; i < logoCacheN; i++) if (!strcmp(logoCache[i].iata, iata)) return logoCache[i].missing ? NULL : &logoCache[i];
  return NULL;
}
void logoLoad(const char *iata) {
  if (!iata || !iata[0]) return;
  for (int i = 0; i < logoCacheN; i++) if (!strcmp(logoCache[i].iata, iata)) return;   // cached, or known missing
  if (logoCacheN >= MAX_LOGOS) return;
  LogoEntry &e = logoCache[logoCacheN]; memset(&e, 0, sizeof(e)); strlcpy(e.iata, iata, sizeof(e.iata)); e.missing = true;
  if (sdOk) {
    char path[40]; snprintf(path, sizeof(path), "/deskradar/logos/%s.lgo", iata);
    File f = SD.open(path, FILE_READ);
    if (f) {
      char mg[4]; uint16_t wh[2];
      if (f.read((uint8_t *)mg, 4) == 4 && !memcmp(mg, "LGO1", 4) && f.read((uint8_t *)wh, 4) == 4 && wh[0] && wh[1] && wh[0] <= 200 && wh[1] <= 64) {
        size_t n = (size_t)wh[0] * wh[1];
        e.px = (uint16_t *)heap_caps_malloc(n * 2, MALLOC_CAP_SPIRAM); e.al = (uint8_t *)heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
        if (e.px && e.al && f.read((uint8_t *)e.px, n * 2) == n * 2 && f.read(e.al, n) == n) { e.w = wh[0]; e.h = wh[1]; e.missing = false; }
        else { free(e.px); free(e.al); e.px = NULL; e.al = NULL; }
      }
      f.close();
    }
  }
  logoCacheN++;
}
void logoForget() { for (int i = 0; i < logoCacheN; i++) { free(logoCache[i].px); free(logoCache[i].al); } logoCacheN = 0; }   // after an upload

// ---- card upload / list over WiFi, limited to /deskradar/ (the owner's card also holds phone files) ----
// curl -4 -F "f=@6E.lgo" "http://deskradar6.local/sdput?path=/deskradar/logos/6E.lgo"   (tools/sd_upload.sh)
File sdUpFile; String sdUpPath; bool sdUpOk = false;
bool sdPathOk(const String &p) { return p.startsWith("/deskradar/") && p.indexOf("..") < 0 && p.length() < 96; }
void handleSdPutUpload() {
  esp_task_wdt_reset();   // 6.14.1: a long upload runs inside one handleClient() call
  HTTPUpload &up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    sdUpPath = server.arg("path"); sdUpOk = false;
    if (!sdOk || !sdPathOk(sdUpPath)) { logf("sdput refused: %s\n", sdUpPath.c_str()); return; }
    for (int k = 1; (k = sdUpPath.indexOf('/', k)) > 0; k++) { String d = sdUpPath.substring(0, k); if (!SD.exists(d)) SD.mkdir(d); }
    sdUpFile = SD.open(sdUpPath, FILE_WRITE); sdUpOk = (bool)sdUpFile;
  } else if (up.status == UPLOAD_FILE_WRITE) { if (sdUpFile && sdUpFile.write(up.buf, up.currentSize) != up.currentSize) sdUpOk = false; }
  else if (up.status == UPLOAD_FILE_END) { if (sdUpFile) sdUpFile.close(); }
  else if (up.status == UPLOAD_FILE_ABORTED) { if (sdUpFile) { sdUpFile.close(); SD.remove(sdUpPath); } sdUpOk = false; }   // 6.14.1: only remove what this upload created
}
void handleSdPutDone() {
  if (sdUpOk && sdUpPath.startsWith("/deskradar/logos/")) logoForget();
  server.send(sdUpOk ? 200 : 400, "text/plain", sdUpOk ? "ok " + sdUpPath : String("failed (card missing or path outside /deskradar/)"));
}
void handleSdList() {
  String p = server.hasArg("path") ? server.arg("path") : String("/deskradar");
  if (!sdOk || !(p == "/deskradar" || sdPathOk(p))) { server.send(400, "text/plain", "card missing or path outside /deskradar"); return; }
  File d = SD.open(p); if (!d || !d.isDirectory()) { server.send(404, "text/plain", "no such folder"); return; }
  String o; int n = 0;
  for (File e = d.openNextFile(); e; e = d.openNextFile()) { o += String(e.name()) + (e.isDirectory() ? "/" : "  " + String((unsigned)e.size())) + "\n"; n++; }
  o += String(n) + " entries, card free " + String((unsigned)((SD.totalBytes() - SD.usedBytes()) >> 20)) + " MB\n";
  server.send(200, "text/plain", o);
}
const char *uiHint() {   // 6.14.0: the right knob's turn / click from navHint(); RADAR and CLOCK keep the bottom free
  static char fb[64];
  if (currentApp == APP_RADAR || currentApp == APP_CLOCK) return "";
  if (currentApp == APP_POMODORO || currentApp == APP_SETTINGS) return "";   // 6.18.2: these pages draw their own state-driven footer (it overlapped)
  if (currentApp == APP_MARKET && marketSel >= NUM_SYMBOLS) return "";   // watchlist / MF / portfolio pages have their own captions
  const char *t = navHint(1, 0), *c = navHint(1, 1);
  if (!t[0] && !c[0]) return "";
  if (t[0] && c[0]) snprintf(fb, sizeof(fb), "turn: %s   click: %s", t, c);
  else snprintf(fb, sizeof(fb), t[0] ? "turn: %s" : "click: %s", t[0] ? t : c);
  return fb;
}
void uiFrame() {   // left dial only, after the app's paint, before the toast
  const char *h = uiHint();
  if (h[0] && !(currentApp == APP_RADAR && currentMode == MODE_SELECT && planeCount == 0)) radar->textC(220, h, FSR, rgb(80, 80, 80));
  if (appCardMs && millis() - appCardMs < 900 && !menuOpen) {
    float t = (millis() - appCardMs) / 900.0f; float a = t < 0.15f ? t / 0.15f : t > 0.7f ? (1.0f - t) / 0.3f : 1.0f;
    uint16_t bg = rgb(8 * a + 4, 12 * a + 4, 10 * a + 4), fg = rgb(200 * a + 20, 200 * a + 20, 200 * a + 20);
    radar->fillRoundRect(28, 94, 184, 52, 10, bg); radar->drawRoundRect(28, 94, 184, 52, 10, dim565(COL_RING, 0.5f + 0.5f * a));
    radar->textC(128, APP_NAMES[currentApp], FMB, fg);
  }
}
// ===== Boot sequence (6.17.0): a task animates both dials and the rings at 20 fps while setup() runs. Left dial:
// radar intro (expanding rings, sweep, name). Right dial: a 24-segment progress circle with the step in the middle.
// Rings: the same progress. setup() only reports milestones through bootStep(); showMessage() stops the animation. =====
volatile bool bootAnim = false; int bootPct = 0; const char *bootNow = "starting"; char bootState[40] = "";
void paintBootLeft() {
  radar->fillScreen(COL_BG); unsigned long now = millis(); float t = (now % 3000) / 3000.0f;
  for (int r = 0; r < 3; r++) { float f = t + r / 3.0f; f -= floorf(f); int rr = (int)(f * 112); if (rr > 2) radar->drawCircle(RADAR_CX, RADAR_CY, rr, dim565(COL_RING, 1.0f - f)); }
  float a = (now % 2000) / 2000.0f * 360.0f;
  for (int k = 0; k < 40; k += 2) { int x, y; polar(a - k, 112, x, y); radar->drawLine(RADAR_CX, RADAR_CY, x, y, dim565(COL_SWEEP, 1.0f - k / 40.0f)); }
  radar->fillCircle(RADAR_CX, RADAR_CY, 46, COL_BG); radar->drawCircle(RADAR_CX, RADAR_CY, 46, COL_RING_DIM);
  radar->textC(RADAR_CY - 2, "DESK", FMB, COL_TEXT); radar->textC(RADAR_CY + 22, "RADAR", FMB, COL_TEXT);
  radar->textC(208, "v" FW_VERSION, FSR, COL_TEXT_DIM);
}
void paintBootRight() {
  radar->fillScreen(COL_BG); char b[16];
  int lit = (bootPct * 24 + 50) / 100; int head = (millis() / 60) % 24;
  for (int i = 0; i < 24; i++) { uint16_t c = i < lit ? COL_SWEEP : (i == head ? COL_RING : rgb(30, 30, 36)); radar->ringArc(RADAR_CX, RADAR_CY, 114, 102, i * 15 + 1.5f, i * 15 + 13.5f, c); }
  snprintf(b, sizeof(b), "%d", bootPct); int w = radar->textWidth(FB2, b);
  radar->text(RADAR_CX - w / 2 - 6, RADAR_CY + 10, b, FB2, COL_TEXT); radar->text(RADAR_CX + w / 2 - 4, RADAR_CY + 10, "%", FS, COL_TEXT_DIM);
  radar->textC(RADAR_CY + 38, bootNow, FS, COL_TEXT);
  if (bootState[0]) radar->textC(RADAR_CY + 56, bootState, FSR, COL_TEXT_DIM);
}
void bootRings() {
  float k = ringBright / 100.0f * tftBright / 100.0f; int lit = (bootPct * 24 + 50) / 100; int head = (millis() / 60) % 24;
  for (int j = 0; j < 24; j++) {
    uint8_t r = 0, g = j < lit ? 190 : (j == head ? 70 : 0), bb = j < lit ? 80 : (j == head ? 30 : 0);
    ringL.setPixelColor(ringPhys(0, j), ringCol(ringL, r, g, bb, k)); ringR.setPixelColor(ringPhys(1, j), ringCol(ringR, r, g, bb, k));
  }
  ringL.show(); ringR.show();
}
void bootTask(void *) {
  while (bootAnim) { present(paintBootLeft); presentR(paintBootRight); bootRings(); vTaskDelay(pdMS_TO_TICKS(50)); }
  vTaskDelete(NULL);
}
void bootStep(const char *what, const char *state) {
  static const struct { const char *w; int p; } BP[] = {{"display", 10}, {"WiFi", 40}, {"clock", 70}, {"services", 85}, {"ready", 100}};
  for (auto &e : BP) if (!strcmp(e.w, what)) bootPct = e.p;
  if (!strcmp(what, "WiFi") && !strcmp(state, "connecting")) bootPct = 25;
  bootNow = what; strlcpy(bootState, state, sizeof(bootState));
  logf("boot %d%%: %s %s\n", bootPct, what, state);
}
unsigned long bootStartMs = 0;
void bootStop() {   // the intro stays up for at least 4 s even when WiFi connects in one
  if (!bootAnim) return;
  while (bootStartMs && millis() - bootStartMs < 4000) vTaskDelay(pdMS_TO_TICKS(50));
  bootAnim = false; vTaskDelay(pdMS_TO_TICKS(150));
}
// ===== screenshot server (6.13.2): port 8080, its own task on core 0, so streaming an image never touches loop().
// Serves /screen.bmp (?d=1 right dial) from the last completed framebuffer and /oled.bmp (?d=1) from the OLED shadows. =====
WiFiServer shotServer(8080);
void shotTask(void *) {
  static uint8_t row[720]; static uint8_t *blk = (uint8_t *)heap_caps_malloc(720 * 16, MALLOC_CAP_SPIRAM);   // 6.13.3: 16 rows per write (was 1: 4.9 s per image)
  for (;;) {
    WiFiClient c = shotServer.accept();
    if (!c) { vTaskDelay(pdMS_TO_TICKS(30)); continue; }
    c.setTimeout(2000); c.setConnectionTimeout(2000);   // 6.14.1: Stream timeout is in ms (2 ms read the request line as empty: random 404s)
    String req = c.readStringUntil('\n'); unsigned long t0 = millis(); while (c.connected() && c.available() && millis() - t0 < 300) c.read();   // drop the headers
    bool right = req.indexOf("d=1") > 0, oled = req.indexOf("/oled") >= 0;
    if (req.indexOf("/screen") < 0 && !oled) { c.print("HTTP/1.0 404 Not Found\r\nConnection: close\r\n\r\n"); c.stop(); continue; }
    if (!oled) {
      uint16_t *fb = fbFront[right ? 1 : 0]; if (!fb) { c.print("HTTP/1.0 503 Service Unavailable\r\nConnection: close\r\n\r\n"); c.stop(); continue; }
      static uint16_t *snap = (uint16_t *)heap_caps_malloc(240 * 240 * 2, MALLOC_CAP_SPIRAM);   // 6.14.1: copy the frame first (the UI repaints that buffer two frames later)
      if (snap) { memcpy(snap, fb, 240 * 240 * 2); fb = snap; }
      const uint32_t rowBytes = 720, imgBytes = rowBytes * 240, fileBytes = 54 + imgBytes; uint8_t hdr[54] = {0};
      hdr[0] = 'B'; hdr[1] = 'M'; memcpy(hdr + 2, &fileBytes, 4); hdr[10] = 54; hdr[14] = 40; int32_t w = 240, h = 240; memcpy(hdr + 18, &w, 4); memcpy(hdr + 22, &h, 4); hdr[26] = 1; hdr[28] = 24; memcpy(hdr + 34, &imgBytes, 4);
      c.printf("HTTP/1.0 200 OK\r\nContent-Type: image/bmp\r\nContent-Length: %lu\r\nCache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n", (unsigned long)fileBytes);
      c.write(hdr, 54);
      for (int y0 = 239; y0 >= 0 && c.connected(); y0 -= 16) {
        int n = 0;
        for (int y = y0; y > y0 - 16 && y >= 0; y--, n++) { uint8_t *r = (blk ? blk + n * 720 : row); for (int x = 0; x < 240; x++) { uint16_t v = fb[y * 240 + x]; r[x * 3] = (v & 0x1F) << 3; r[x * 3 + 1] = ((v >> 5) & 0x3F) << 2; r[x * 3 + 2] = ((v >> 11) & 0x1F) << 3; } if (!blk && c.write(row, 720) != 720) break; }
        if (blk && c.write(blk, n * 720) != (size_t)(n * 720)) break;
      }
    } else {
      static uint8_t buf[1024]; portENTER_CRITICAL(&oledMux); memcpy(buf, oledShadow[right ? 1 : 0], 1024); portEXIT_CRITICAL(&oledMux);
      const uint32_t imgBytes = 16 * 64, fileBytes = 62 + imgBytes; uint8_t hdr[62] = {0};
      hdr[0] = 'B'; hdr[1] = 'M'; memcpy(hdr + 2, &fileBytes, 4); hdr[10] = 62; hdr[14] = 40; int32_t w = 128, h = 64; memcpy(hdr + 18, &w, 4); memcpy(hdr + 22, &h, 4); hdr[26] = 1; hdr[28] = 1; memcpy(hdr + 34, &imgBytes, 4);
      uint32_t ncol = 2; memcpy(hdr + 46, &ncol, 4); hdr[58] = 255; hdr[59] = 200; hdr[60] = 60;
      c.printf("HTTP/1.0 200 OK\r\nContent-Type: image/bmp\r\nContent-Length: %lu\r\nCache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n", (unsigned long)fileBytes);
      c.write(hdr, 62);
      for (int y = 63; y >= 0 && c.connected(); y--) { memset(row, 0, 16); for (int x = 0; x < 128; x++) { int rx = 127 - x, ry = 63 - y; if (buf[(ry / 8) * 128 + rx] & (1 << (ry & 7))) row[x >> 3] |= 0x80 >> (x & 7); } c.write(row, 16); }
    }
    c.stop();
  }
}
void paintCompanion() {   // apps without their own companion: name and position (no clock: the right OLED is the clock)
  radar->fillScreen(COL_BG);
  for (int i = 0; i < menuCount; i++) { int x, y; polar(i * 360.0f / menuCount, 108, x, y); bool cur = menuItems[i] == currentApp; radar->fillCircle(x, y, cur ? 5 : 2, cur ? COL_SWEEP : COL_RING_DIM); }
  radar->textC(112, APP_NAMES[currentApp], FMB, COL_TEXT);
  char lines[3][40]; int nl = wrapText(APP_DESC[currentApp], FS, 176, lines, 3);
  for (int l = 0; l < nl; l++) radar->textC(140 + l * 18, lines[l], FS, COL_TEXT_DIM);
}
// ---- RADAR right dial: flight card and runway view ----
#define RWY_LEN_KM 4.0f                     // drawn runway length; the airport reference point is its midpoint
// Dead-reckoned position, same model as planeScreenPos() on the left dial.
void drLatLon(const Plane &p, double &lat, double &lon) {
  lat = p.lat; lon = p.lon;
#if DEAD_RECKON
  float dt = (millis() - lastFetchMs) / 1000.0f; if (dt > 120) dt = 120;
  double d = p.velocity_ms * dt, h = toRad(p.heading_deg);
  lat += (d * cos(h)) / 111320.0; lon += (d * sin(h)) / (111320.0 * cos(toRad(p.lat)));
#endif
}
// Position of an aircraft in runway coordinates for landing runway rwy: along = km before the threshold
// (negative = over the runway), cross = km right of the centreline seen in the landing direction.
void rwyCoords(const Plane &p, int rwy, float &along, float &cross) {
  double lat, lon; drLatLon(p, lat, lon);
  float d = distanceKm(homeLat, homeLon, lat, lon), a = toRad(bearingDeg(homeLat, homeLon, lat, lon) - rwy * 10.0f);
  along = -d * cosf(a) - RWY_LEN_KM / 2; cross = d * sinf(a);
}
// The arrival closest to touchdown: on final (or rolling out) within 20 km, 3 km of the centreline, below 6000 ft AGL.
int finalPlane(int &rwy, float &along, float &cross) {
  int best = -1; along = 1e9; cross = 0; rwy = activeRwy > 0 ? activeRwy : rwyHdg() / 10;
  for (int i = 0; i < planeCount; i++) {
    const Plane &p = planes[i];
    if (p.cls != CLS_ARRIVAL) continue;
    if ((p.altitude_m - homeElevM()) * 3.28084f > 6000) continue;
    int r = activeRwy > 0 ? activeRwy : landingRunway(p); float al, cr; rwyCoords(p, r, al, cr);
    if (al < -RWY_LEN_KM || al > 20 || fabsf(cr) > 3) continue;
    if (al < along) { along = al; cross = cr; best = i; rwy = r; }
  }
  return best;
}
void fmtThousands(char *out, size_t n, long v) {
  char t[16]; snprintf(t, sizeof(t), "%ld", v < 0 ? -v : v); int len = strlen(t), o = 0;
  if (v < 0 && o < (int)n - 1) out[o++] = '-';
  for (int i = 0; i < len && o < (int)n - 1; i++) { out[o++] = t[i]; if ((len - i - 1) % 3 == 0 && i < len - 1 && o < (int)n - 1) out[o++] = ','; }
  out[o] = 0;
}
void cardTile(int x, int y, const char *label, const char *val, uint16_t col) {
  radar->drawRoundRect(x, y, 86, 34, 6, COL_PANEL);
  radar->text(x + 7, y + 13, label, FSR, COL_TEXT_DIM);
  radar->text(x + 7, y + 29, val, FS, col);
}
// Common ICAO type codes in words (the code itself is shown when it is not listed).
const char *typeName(const char *t) {
  static const char *T[][2] = {
    {"A20N", "Airbus A320neo"}, {"A21N", "Airbus A321neo"}, {"A320", "Airbus A320"}, {"A321", "Airbus A321"}, {"A319", "Airbus A319"},
    {"A332", "Airbus A330-200"}, {"A333", "Airbus A330-300"}, {"A339", "Airbus A330-900"}, {"A359", "Airbus A350-900"}, {"A35K", "Airbus A350-1000"},
    {"A388", "Airbus A380"}, {"B738", "Boeing 737-800"}, {"B38M", "Boeing 737 MAX 8"}, {"B39M", "Boeing 737 MAX 9"}, {"B737", "Boeing 737-700"},
    {"B739", "Boeing 737-900"}, {"B77W", "Boeing 777-300ER"}, {"B772", "Boeing 777-200"}, {"B77L", "Boeing 777-200LR"}, {"B788", "Boeing 787-8"},
    {"B789", "Boeing 787-9"}, {"B78X", "Boeing 787-10"}, {"B744", "Boeing 747-400"}, {"B748", "Boeing 747-8"}, {"B763", "Boeing 767-300"},
    {"AT76", "ATR 72-600"}, {"AT75", "ATR 72-500"}, {"AT72", "ATR 72"}, {"DH8D", "Dash 8 Q400"}, {"E190", "Embraer E190"},
    {"E195", "Embraer E195"}, {"E75L", "Embraer E175"}, {"CRJ9", "Bombardier CRJ900"}, {"C130", "Lockheed C-130"}, {"C17", "Boeing C-17"},
    {"IL76", "Ilyushin Il-76"}, {"A400", "Airbus A400M"}, {"P8", "Boeing P-8"}, {"B350", "King Air 350"}, {"PC12", "Pilatus PC-12"},
    {"C56X", "Citation Excel"}, {"GLF6", "Gulfstream G650"}, {"GLEX", "Global Express"}, {"FA7X", "Falcon 7X"}, {"CL60", "Challenger 600"}};
  for (auto &r : T) if (!strcmp(t, r[0])) return r[1];
  return t;
}
// Right dial subject in RADAR: the selected aircraft, else the one closest to touchdown, else the nearest.
int radarSubject() {
  if (planeCount <= 0) return -1;
  if (currentMode == MODE_SELECT && selectedIndex >= 0 && selectedIndex < planeCount) return selectedIndex;
  int idx[MAX_PLANES];
  if (radarFocus == 0) {                                                // ARRIVALS: on final, else the next by ETA
    int r; float al, cr; int f = finalPlane(r, al, cr); if (f >= 0) return f;
    if (boardList(idx, 1, true, planes, planeCount, homeLat, homeLon)) return idx[0];
  } else if (boardList(idx, 1, false, planes, planeCount, homeLat, homeLon)) return idx[0];   // DEPARTURES: closest to the field
  int best = -1; float bd = 1e9;
  for (int i = 0; i < planeCount; i++) { if (!passesFilter(planes[i])) continue; if (planes[i].distance_km < bd) { bd = planes[i].distance_km; best = i; } }
  return best;
}
// Side profile of an arrival (runway on the right, 3 degree slope) or a departure (runway on the left, climb).
// Distance and height scales zoom together with the aircraft, so the slope keeps the same angle on screen.
// Returns false when the aircraft is not near this airport's approach or climb-out (en route).
bool paintProfile(const Plane &p) {
  int rwy = activeRwy > 0 ? activeRwy : landingRunway(p); float along, cross; rwyCoords(p, rwy, along, cross);
  float agl = (p.altitude_m - homeElevM()) * 3.28084f; if (agl < 0) agl = 0;
  bool arr = p.cls == CLS_ARRIVAL, dep = p.cls == CLS_DEPARTURE;
  float past = fmaxf(p.distance_km - RWY_LEN_KM / 2, 0);                  // departures: km from the runway end (real distance: they turn)
  if (arr && !(along > -RWY_LEN_KM && along < 30 && fabsf(cross) < 4 && agl < 10000)) return false;
  if (dep && !(past < 30 && agl < 12000)) return false;
  if (!arr && !dep) return false;
  char b[48], v[16]; const int yg = 200, ytop = 158;
  float d = arr ? along : past;
  float maxKm = fmaxf(3, fminf(15, fmaxf(d, 0) * 1.5f));
  int xr0 = arr ? 172 : 68;                                              // threshold (arrival) or far end (departure)
  float ppk = (arr ? (172 - 40) : (204 - 68)) / maxKm;
  float ppf = ppk / 3.0f / 172.0f;                                       // the 3 degree slope (172 ft per km) rises 1 px per 3 px
  if (dep) ppf = (yg - ytop) / fmaxf(agl * 1.25f, 1500.0f);             // climbs are far steeper: own height scale (6.3.0)
  // ground, runway, distance marks
  radar->drawLine(32, yg, 208, yg, COL_LINE);
  if (arr) { radar->fillRect(xr0, yg - 1, 34, 3, rgb(150, 150, 150)); radar->drawLine(xr0, yg - 4, xr0, yg + 3, rgb(230, 230, 230)); }
  else     { radar->fillRect(xr0 - 34, yg - 1, 34, 3, rgb(150, 150, 150)); }
  int step = maxKm > 8 ? 2 : 1;
  for (int k = step; k <= (int)maxKm; k += step) {
    int x = arr ? xr0 - (int)(k * ppk) : xr0 + (int)(k * ppk); if (x < 40 || x > 204) continue;
    radar->drawLine(x, yg, x, yg + 3, COL_LINE);
    snprintf(v, sizeof(v), "%d", k); radar->text(x - radar->textWidth(FSR, v) / 2, yg + 14, v, FSR, rgb(80, 80, 80));
  }
  // 3 degree glideslope from 50 ft over the threshold (arrivals)
  if (arr) for (float k = 0; k < maxKm; k += 0.5f) {
    int xa = xr0 - (int)(k * ppk), ya = yg - (int)((50 + k * 172) * ppf), xb = xr0 - (int)((k + 0.25f) * ppk), yb = yg - (int)((50 + (k + 0.25f) * 172) * ppf);
    if (ya < ytop) break;
    radar->drawLine(xa, ya, xb, yb, rgb(0, 110, 50));
  }
  // trail, then the aircraft
  uint16_t col = planeColour(p);
  const Trail *t = trailFor(p.icao24);
  if (t) for (int k = 0; k < t->n; k++) {
    float dd = distanceKm(homeLat, homeLon, t->lat[k], t->lon[k]), aa = toRad(bearingDeg(homeLat, homeLon, t->lat[k], t->lon[k]) - rwy * 10.0f);
    float al2 = -dd * cosf(aa) - RWY_LEN_KM / 2, d2 = arr ? al2 : fmaxf(dd - RWY_LEN_KM / 2, 0);
    float h2 = (t->alt[k] - homeElevM()) * 3.28084f; if (h2 < 0) h2 = 0;
    int x = arr ? xr0 - (int)(d2 * ppk) : xr0 + (int)(d2 * ppk), y = yg - (int)(h2 * ppf); if (y < ytop) y = ytop;
    if (x > 34 && x < 206) radar->fillCircle(x, y, 1, dim565(col, 0.5f));
  }
  int x = arr ? xr0 - (int)(fmaxf(d, -RWY_LEN_KM) * ppk) : xr0 + (int)(d * ppk), y = yg - (int)(agl * ppf);
  bool clipped = y < ytop; if (clipped) y = ytop; if (x < 36) x = 36; if (x > 204) x = 204;
  float dev = arr && along > 0 ? agl - (along * 172 + 50) : 0; bool onSlope = !arr || along <= 0 || fabsf(dev) < 150;
  uint16_t pc = arr ? (onSlope ? COL_OK : COL_ARR) : COL_DEP;
  drawPlaneIcon(x, y, arr ? 90 : 90, pc, 1.2f, p);
  if (clipped) radar->drawLine(x, y - 9, x, y - 14, pc);                  // above the frame: tick upward
  // numbers: line 1 where, line 2 height / speed / slope or climb
  float kmh = p.velocity_ms * 3.6f; int kt = (int)(p.velocity_ms * 1.94384f); char ft[16]; fmtThousands(ft, sizeof(ft), (long)agl);
  if (arr && along <= 0) snprintf(b, sizeof(b), "ON RUNWAY %02d", rwy);
  else if (arr) { int sec = kmh > 50 ? (int)(along / kmh * 3600) : -1; if (sec >= 0) snprintf(b, sizeof(b), "RWY %02d  %.1f km  %d:%02d", rwy, along, sec / 60, sec % 60); else snprintf(b, sizeof(b), "RWY %02d  %.1f km", rwy, along); }
  else snprintf(b, sizeof(b), "OFF %02d  %.1f km out", rwy, fmaxf(past, 0));
  radar->textC(132, b, FS, COL_TEXT);
  if (arr && along > 0 && !onSlope) snprintf(b, sizeof(b), "%s ft  %d kt  %.0f %s", ft, kt, fabsf(dev), dev > 0 ? "high" : "low");
  else if (arr) snprintf(b, sizeof(b), "%s ft  %d kt%s", ft, kt, along > 0 ? "  on slope" : "");
  else { char vs[12]; fmtThousands(vs, sizeof(vs), (long)p.vs_fpm); snprintf(b, sizeof(b), "%s ft  %d kt  %s%s", ft, kt, p.vs_fpm > 0 ? "+" : "", vs); }
  radar->textC(148, b, FSR, pc);
  return true;
}
// Right dial in RADAR. Top half = who (airline badge, flight, airline, route, aircraft type and registration);
// bottom half = where (side profile near the airport, else altitude / speed / climb / distance tiles).
void paintFlightDetail(const Plane &p, const char *label, uint16_t labelCol, bool sel, bool profileOk);
void paintTodayRight() {   // 6.16.0: the old STATS page, on the radar's right dial whenever nothing is in range
  char b[48];
  radar->textC(40, "TRAFFIC TODAY", FS, COL_TEXT_DIM);
  snprintf(b, sizeof(b), "%d", seenToday); radar->textC(86, b, FB2, COL_TEXT);
  snprintf(b, sizeof(b), "%d arr  %d dep  %d over", seenArr, seenDep, seenEnr); radar->textC(106, b, FSR, COL_TEXT_DIM);
  int maxH = 1; for (int h = 0; h < 24; h++) { int sum = hourArr[h] + hourDep[h] + hourEnr[h]; if (sum > maxH) maxH = sum; }
  const int x0 = 36, y1 = 172, H = 46;
  for (int h = 0; h < 24; h++) {
    int x = x0 + h * 7, y = y1; int va = hourArr[h] * H / maxH, vd = hourDep[h] * H / maxH, ve = hourEnr[h] * H / maxH;
    if (va) { radar->fillRect(x, y - va, 5, va, COL_ARR); y -= va; }
    if (vd) { radar->fillRect(x, y - vd, 5, vd, COL_DEP); y -= vd; }
    if (ve) radar->fillRect(x, y - ve, 5, ve, COL_ENROUTE_HIGH);
  }
  radar->drawLine(x0, y1 + 1, x0 + 167, y1 + 1, rgb(40, 40, 40));
  for (int h = 0; h <= 18; h += 6) { snprintf(b, sizeof(b), "%02d", h); radar->text(x0 + h * 7, y1 + 13, b, FSR, rgb(70, 70, 70)); }
  int busiest = 0; for (int h = 1; h < 24; h++) if (hourArr[h] + hourDep[h] + hourEnr[h] > hourArr[busiest] + hourDep[busiest] + hourEnr[busiest]) busiest = h;
  radar->textC(202, "no aircraft in range", FSR, COL_TEXT_DIM);
  snprintf(b, sizeof(b), "busiest hour %02d:00", busiest); radar->textC(218, b, FSR, rgb(80, 80, 80));
}
void paintRadarRight() {
  radar->fillScreen(COL_BG);
  int i = radarSubject();
  if (i < 0) { paintTodayRight(); return; }
  bool sel = currentMode == MODE_SELECT;
  paintFlightDetail(planes[i], sel ? "SELECTED" : radarFocus ? "DEPARTURES" : "ARRIVALS", sel ? COL_SELECTED : radarFocus ? COL_DEP : COL_ARR, sel, true);
}
// Who (logo or badge, flight, route, type) on top, where (side profile near the home airport, else tiles) below.
void paintFlightDetail(const Plane &p, const char *label, uint16_t labelCol, bool sel, bool profileOk) {
  char b[48], v[24];
  radar->textC(16, label, FSR, labelCol);
  radar->textC(16, label, FSR, labelCol);
  // who
  char pcs[12]; prettyCallsign(p.callsign, pcs, sizeof(pcs));
  const AirlineRow *al = airlineFor(p.callsign); bool badge = al && al->iata[0];
  const LogoEntry *lg = badge ? logoFind(al->iata) : NULL;   // loaded from the card by drawCompanionFrame()
  if (lg) {   // transparent logo straight on the dial, flight number under it (the logo names the airline)
    radar->imageAlpha(RADAR_CX - lg->w / 2, 43 - lg->h / 2, lg->px, lg->al, lg->w, lg->h);
    radar->textC(84, pcs, FMB, sel ? COL_SELECTED : COL_TEXT);
    const char *rt = routeFor(p.callsign);
    if (rt && rt[0]) { strlcpy(b, rt, sizeof(b)); char *d = strchr(b, '-'); if (d) { *d = 0; snprintf(v, sizeof(v), "%s  >  %s", b, d + 1); } else strlcpy(v, b, sizeof(v)); radar->textC(100, v, FS, COL_TEXT); }
    else radar->textC(100, "route unknown", FSR, COL_TEXT_DIM);
  } else {
  const char *num = badge && strchr(pcs, ' ') ? strchr(pcs, ' ') + 1 : pcs;
  int bw = badge ? radar->textWidth(FMB, al->iata) + 14 : 0, nw = radar->textWidth(FMB, num), gap = badge ? 6 : 0;
  int x0 = RADAR_CX - (bw + gap + nw) / 2;
  if (badge) {
    uint32_t c = al->rgb; int lum = ((c >> 16) & 0xFF) * 3 + ((c >> 8) & 0xFF) * 6 + (c & 0xFF);
    radar->fillRoundRect(x0, 22, bw, 27, 6, rgb24(c));
    radar->text(x0 + 7, 43, al->iata, FMB, lum > 1200 ? rgb(0, 0, 0) : rgb(255, 255, 255));
  }
  radar->text(x0 + bw + gap, 43, num, FMB, sel ? COL_SELECTED : COL_TEXT);
  radar->textC(64, al && al->name && al->name[0] ? al->name : "airline unknown", FS, COL_TEXT_DIM);
  const char *route = routeFor(p.callsign);
  if (route && route[0]) { strlcpy(b, route, sizeof(b)); char *d = strchr(b, '-'); if (d) { *d = 0; snprintf(v, sizeof(v), "%s  >  %s", b, d + 1); } else strlcpy(v, b, sizeof(v)); radar->textC(88, v, FM, COL_TEXT); }
  else radar->textC(86, "route unknown", FS, COL_TEXT_DIM);
  }
  char type[24] = ""; strlcpy(type, p.type, sizeof(type));
  if (!type[0]) { xSemaphoreTake(dataMutex, portMAX_DELAY); if (!strcmp(metaIcao, p.icao24) && metaDone) strlcpy(type, metaType, sizeof(type)); xSemaphoreGive(dataMutex); }
  snprintf(b, sizeof(b), "%s%s%s", type[0] ? typeName(type) : "type unknown", p.reg[0] ? "  " : "", p.reg);
  radar->textC(lg ? 113 : 108, b, FSR, COL_TEXT_DIM);
  radar->drawLine(36, 118, 204, 118, COL_LINE);
  // where
  if (profileOk && paintProfile(p)) return;
  double lat, lon; drLatLon(p, lat, lon);
  float dist = distanceKm(homeLat, homeLon, lat, lon), brg = bearingDeg(homeLat, homeLon, lat, lon);
  int ft = (int)(p.altitude_m * 3.28084f);
  if (ft >= 18000) snprintf(v, sizeof(v), "FL%03d", ft / 100); else { fmtThousands(b, sizeof(b), ft); snprintf(v, sizeof(v), "%s ft", b); }
  cardTile(30, 124, "ALTITUDE", v, COL_TEXT);
  snprintf(v, sizeof(v), "%d kt", (int)(p.velocity_ms * 1.94384f)); cardTile(124, 124, "SPEED", v, COL_TEXT);
  if (fabsf(p.vs_fpm) < 200) strcpy(v, "level"); else { fmtThousands(b, sizeof(b), (long)p.vs_fpm); snprintf(v, sizeof(v), "%s%s fpm", p.vs_fpm > 0 ? "+" : "", b); }
  cardTile(30, 162, "VERTICAL", v, p.vs_fpm > 200 ? COL_DEP : p.vs_fpm < -200 ? COL_ARR : COL_TEXT);
  snprintf(v, sizeof(v), "%.0f%s %03d" DEG, dist / KM_PER_UNIT, RANGE_UNIT, (int)brg); cardTile(124, 162, "DISTANCE", v, COL_TEXT);
  if (p.cls == CLS_ARRIVAL) snprintf(b, sizeof(b), "ARRIVING  ~%d min", etaMin(p, dist));
  else snprintf(b, sizeof(b), "%s  HDG %03d", p.cls == CLS_DEPARTURE ? "DEPARTED" : "EN ROUTE", (int)p.heading_deg);
  radar->textC(214, b, FS, planeColour(p));
  if (p.squawk[0] && (!strcmp(p.squawk, "7700") || !strcmp(p.squawk, "7600") || !strcmp(p.squawk, "7500"))) { snprintf(b, sizeof(b), "SQUAWK %s", p.squawk); radar->textC(230, b, FSR, COL_BAD); }
}
// Left OLED in RADAR: the side the right dial is not showing. ARRIVALS mode lists departures (flight, destination,
// altitude, nearest the field first); DEPARTURES mode lists arrivals (flight, origin, minutes to landing, by ETA).
// Up to 3 rows in the blue area, title and count in the yellow band; the top rows stay empty.
void drawInboundTimeline() {
  Adafruit_SSD1306 &o = infoDisplay;
  bool listArr = radarFocus == 1;
  int idx[MAX_PLANES]; int m = boardList(idx, MAX_PLANES, listArr, planes, planeCount, homeLat, homeLon);
  o.setFont(&F_SM13); o.setTextSize(1);
  if (!m) { o.setCursor(4, 30); o.print(listArr ? "no arrivals" : "no departures"); }
  for (int r = 0; r < 3 && r < m; r++) {
    const Plane &p = planes[idx[r]]; char pcs[12], code[6] = "---", val[12]; int y = 17 + r * 13;
    prettyCallsign(p.callsign, pcs, sizeof(pcs));
    const char *route = routeFor(p.callsign);
    if (route && route[0]) { if (listArr) strlcpy(code, route, 4); else { const char *d = strrchr(route, '-'); strlcpy(code, d ? d + 1 : route, 4); } }
    if (listArr) { int eta = etaMin(p, distanceKm(homeLat, homeLon, p.lat, p.lon)); if (eta < 99) snprintf(val, sizeof(val), "%d min", eta); else strcpy(val, "--"); }
    else { int ft = (int)(p.altitude_m * 3.28084f); if (ft >= 10000) snprintf(val, sizeof(val), "FL%03d", ft / 100); else { fmtThousands(val, sizeof(val), ft); } }
    o.setCursor(0, y); o.print(pcs);
    o.setCursor(56, y); o.print(code);
    int16_t x1, y1; uint16_t w, h; o.getTextBounds(val, 0, y, &x1, &y1, &w, &h); o.setCursor(127 - (int)w - x1, y); o.print(val);
  }
  char t[24]; snprintf(t, sizeof(t), "%s %d", listArr ? "ARRIVALS" : "DEPARTURES", m);
  o.setCursor(0, 61); o.print(t);
  if (m > 3) { char b[8]; snprintf(b, sizeof(b), "+%d", m - 3); int16_t x1, y1; uint16_t w, h; o.getTextBounds(b, 0, 61, &x1, &y1, &w, &h); o.setCursor(127 - (int)w - x1, 61); o.print(b); }
  o.setFont();
}
// ===== AGENDA app (6.4.1): the calendar from the Mac helper =====
// Timed events still to finish, in start order (the helper sends them sorted). Returns the count, fills idx.
int agendaList(int *idx, int maxN) {
  uint32_t now = time(NULL); int n = 0;
  for (int i = 0; i < calN && n < maxN; i++) if (!calEv[i].allday && calEv[i].e > now) { if (idx) idx[n] = i; n++; }
  return n;
}
void fmtDur(char *b, size_t n, long sec) { long m = (sec + 59) / 60; if (m >= 60) snprintf(b, n, "%ldh %02ldm", m / 60, m % 60); else snprintf(b, n, "%ldm", m); }
float clockAngle(uint32_t epoch) { time_t tt = epoch; struct tm lt; localtime_r(&tt, &lt); return ((lt.tm_hour % 12) * 60 + lt.tm_min) * 0.5f; }   // 12 h dial, degrees from 12
void paintAgenda() {
  radar->fillScreen(COL_BG); char b[48], tm1[16];
  uint32_t now = time(NULL);
  // rim: a 12 h dial; the next 12 hours' events as arcs, the selected one brighter, a dot at the current time
  for (int h = 0; h < 12; h++) { int x0, y0, x1, y1; polar(h * 30, 117, x0, y0); polar(h * 30, h % 3 ? 113 : 109, x1, y1); radar->drawLine(x0, y0, x1, y1, h % 3 ? COL_LINE : COL_TEXT_DIM); }
  int idx[MAX_CAL]; int n = agendaList(idx, MAX_CAL); if (listSel >= n) listSel = n ? n - 1 : 0;
  for (int k = 0; k < n; k++) {
    const CalEvent &c = calEv[idx[k]];
    uint32_t a = c.t < now ? now : c.t, e = c.e > now + 43200 ? now + 43200 : c.e; if (e <= a) continue;
    float a0 = clockAngle(a), span = (e - a) / 120.0f; bool cur = c.t <= now;
    uint16_t col = k == listSel ? (cur ? COL_SELECTED : COL_ARR) : dim565(cur ? COL_SELECTED : COL_DEP, 0.55f);
    radar->ringArc(RADAR_CX, RADAR_CY, 98, k == listSel ? 107 : 104, a0, a0 + (span < 2 ? 2 : span), col);
  }
  { int x, y; polar(clockAngle(now), 102, x, y); radar->fillCircle(x, y, 3, COL_TEXT); }
  // centre: upcoming list, four at a time around the selection
  if (!calMs) { radar->textC(112, "waiting for the Mac", FS, COL_TEXT_DIM); radar->textC(132, "(DeskRadar helper)", FSR, rgb(80, 80, 80)); return; }
  if (!n) { radar->textC(112, "no more meetings", FS, COL_TEXT_DIM); radar->textC(132, "in the next 24 h", FSR, rgb(80, 80, 80)); }
  radar->textC(52, "AGENDA", FSR, COL_TEXT_DIM);
  int first = listSel > 2 ? listSel - 2 : 0;
  for (int r = 0; r < 4 && first + r < n; r++) {
    const CalEvent &c = calEv[idx[first + r]]; int y = 74 + r * 34; bool sel = first + r == listSel, cur = c.t <= now;
    time_t st = c.t; struct tm lt; localtime_r(&st, &lt); fmtHMA(tm1, sizeof(tm1), lt.tm_hour, lt.tm_min);
    if (sel) radar->fillRoundRect(34, y - 13, 172, 32, 6, cur ? rgb(45, 12, 12) : rgb(36, 28, 0));
    radar->text(42, y, cur ? "NOW" : tm1, FSR, cur ? COL_SELECTED : COL_TEXT_DIM);
    char title[40]; strlcpy(title, calTitle(c), sizeof(title)); int L = strlen(title); while (L > 3 && radar->textWidth(FS, title) > 160) title[--L] = 0;
    radar->text(42, y + 15, title, FS, sel ? COL_TEXT : rgb(160, 160, 160));
  }
  // all-day events (first one) at the bottom
  for (int i = 0; i < calN; i++) if (calEv[i].allday) { snprintf(b, sizeof(b), "all day: %s", calEv[i].title); char t2[40]; strlcpy(t2, b, sizeof(t2)); int L = strlen(t2); while (L > 3 && radar->textWidth(FSR, t2) > 150) t2[--L] = 0; radar->textC(214, t2, FSR, COL_DEP); break; }
}
void drawAgendaFrame() { if (!staticDirty && millis() - lastHudMs < 1000) return; lastHudMs = millis(); staticDirty = false; present(paintAgenda); }
// Right dial: the selected event, large
void paintAgendaRight() {
  radar->fillScreen(COL_BG); char b[48], t1[16], t2[16];
  int idx[MAX_CAL]; int n = agendaList(idx, MAX_CAL);
  if (!n) { radar->textC(118, calMs ? "free" : "no calendar yet", FMB, COL_TEXT_DIM); return; }
  const CalEvent &c = calEv[idx[listSel < n ? listSel : 0]]; uint32_t now = time(NULL); bool cur = c.t <= now;
  radar->textC(40, cur ? "NOW" : listSel == 0 ? "NEXT" : "LATER", FS, cur ? COL_SELECTED : COL_ARR);
  char lines[3][40]; int nl = wrapText(calTitle(c), FM, 184, lines, 3);
  int y = 92 - (nl - 1) * 12; for (int l = 0; l < nl; l++) { radar->textC(y, lines[l], FM, COL_TEXT); y += 24; }
  time_t st = c.t, en = c.e; struct tm a, e2; localtime_r(&st, &a); localtime_r(&en, &e2);
  fmtHMA(t1, sizeof(t1), a.tm_hour, a.tm_min); fmtHMA(t2, sizeof(t2), e2.tm_hour, e2.tm_min); snprintf(b, sizeof(b), "%s - %s", t1, t2); radar->textC(150, b, FS, COL_TEXT_DIM);
  if (cur) { fmtDur(t1, sizeof(t1), (long)c.e - (long)now); snprintf(b, sizeof(b), "%s left", t1); }
  else { fmtDur(t1, sizeof(t1), (long)c.t - (long)now); snprintf(b, sizeof(b), "in %s", t1); }
  radar->textC(186, b, FMB, cur ? COL_SELECTED : COL_ARR);
  if (c.cal[0]) radar->textC(212, c.cal, FSR, rgb(90, 90, 90));
}
// Left OLED: what is left of today, busy time, the next free half hour
void drawAgendaInfo() {
  char b[40]; uint32_t now = time(NULL); time_t tn = now; struct tm lt; localtime_r(&tn, &lt);
  struct tm eod = lt; eod.tm_hour = 23; eod.tm_min = 59; eod.tm_sec = 59; uint32_t endDay = (uint32_t)mktime(&eod);
  int left = 0; long busy = 0; for (int i = 0; i < calN; i++) { const CalEvent &c = calEv[i]; if (c.allday || c.e <= now || c.t > endDay) continue; left++; busy += (long)(c.e < endDay ? c.e : endDay) - (long)(c.t > now ? c.t : now); }
  snprintf(b, sizeof(b), "%d", left); o6Line(0, "left today", b);
  fmtDur(b, sizeof(b), busy); o6Line(1, "busy", busy ? b : "0m");
  // next free gap of 30 min or more, from now
  uint32_t t = now; for (int pass = 0; pass < MAX_CAL + 1; pass++) { bool moved = false; for (int i = 0; i < calN; i++) { const CalEvent &c = calEv[i]; if (c.allday) continue; if (c.t <= t + 1800 && c.e > t) { t = c.e; moved = true; } } if (!moved) break; }
  if (t <= now + 60) strcpy(b, "now"); else { time_t ft = t; struct tm f; localtime_r(&ft, &f); fmtHMA(b, sizeof(b), f.tm_hour, f.tm_min); }
  o6Line(2, "free 30m from", b);
  o6Band("AGENDA", macLinked() ? "linked" : "Mac offline");
}
// Rings: left = busy half-hours of the next 12 h at their clock position, current time white; right = countdown
// to the selected event (one LED per 2.5 min over the last hour)
// ===== 6.3.0 companions (right dial) =====
// The flight the right dial describes in RADAR / ARRIVALS / APPROACH, or NULL.
const Plane *rightPlane(bool &home) {
  home = true;
  if (currentApp == APP_RADAR) { int i = radarSubject(); return i >= 0 ? &planes[i] : NULL; }
  if (currentApp == APP_ARRIVALS) {
    bool other = boardAirport != homeIndex, hv = other && arenaOwner == ARENA_BOARD;
    const Plane *src = hv ? ARENA_BD->items : planes; int n = hv ? ARENA_BD->count : (other ? 0 : planeCount);
    const Airport &ap = AIRPORTS[boardAirport]; int idx[MAX_PLANES]; int m = boardList(idx, 12, subPage == 0, src, n, ap.lat, ap.lon);
    home = !other; if (!m) return NULL; return &src[idx[listSel < m ? listSel : m - 1]];
  }
  if (currentApp == APP_APPROACH) { int idx[MAX_PLANES]; int m = approachList(idx, MAX_PLANES); if (!m) return NULL; return &planes[idx[listSel < m ? listSel : 0]]; }
  return NULL;
}
void paintBoardRight() {
  radar->fillScreen(COL_BG); bool home; const Plane *p = rightPlane(home);
  if (!p) { radar->textC(118, currentApp == APP_APPROACH ? "nobody on approach" : "nothing in the list", FS, COL_TEXT_DIM); return; }
  const char *lab = currentApp == APP_APPROACH ? "HIGHLIGHTED" : subPage == 0 ? "ARRIVAL" : "DEPARTURE";
  paintFlightDetail(*p, lab, currentApp == APP_APPROACH ? COL_TEXT_DIM : subPage == 0 ? COL_ARR : COL_DEP, false, home);
}
// CLOCK: second zones as two analog clocks + next holiday; on the World face (which already shows zones) daylight.
void miniClock(int cx, int cy, int r, int h, int m, uint16_t col) {
  radar->drawCircle(cx, cy, r, COL_RING_DIM); radar->drawCircle(cx, cy, r - 1, COL_RING_DIM);
  for (int k = 0; k < 12; k++) { int x0, y0, x1, y1; polarC(cx, cy, k * 30, r - 4, x0, y0); polarC(cx, cy, k * 30, r - (k % 3 ? 7 : 10), x1, y1); radar->drawLine(x0, y0, x1, y1, k % 3 ? COL_LINE : COL_TEXT_DIM); }
  int x, y; polarC(cx, cy, (h % 12) * 30 + m * 0.5f, r * 0.5f, x, y); radar->drawLine(cx, cy, x, y, col); radar->drawLine(cx + 1, cy, x + 1, y, col);
  polarC(cx, cy, m * 6, r * 0.8f, x, y); radar->drawLine(cx, cy, x, y, col);
  radar->fillCircle(cx, cy, 2, col);
}
// 6.18.0: the right dial with the clock. Big type (readable from the desk), chosen in SETTINGS > CLOCK > right dial.
void paintClockRightWeather() {
  char b[40];
  if (!wx.valid) { radar->textC(124, "waiting for weather", FSR, COL_TEXT_DIM); return; }
  radar->textC(38, weatherIndex >= 0 ? AIRPORTS[weatherIndex].name : WX_HOME_NAME, FSR, COL_TEXT_DIM);
  drawWeatherIcon(wx.wcode, (int)(millis() / 500), RADAR_CX, 84, 1.0f);
  snprintf(b, sizeof(b), "%.0f", wx.temp); int w = radar->textWidth(FB2, b);
  radar->text(RADAR_CX - w / 2 - 8, 150, b, FB2, COL_TEXT); radar->text(RADAR_CX + w / 2 - 6, 136, DEG "C", FS, COL_TEXT_DIM);
  radar->textC(172, wmoText(wx.wcode), FS, COL_TEXT);
  snprintf(b, sizeof(b), "H %d  L %d   AQI %d", wx.dayMax[0], wx.dayMin[0], wx.aqi); radar->textC(192, b, FSR, COL_TEXT_DIM);
  int x0 = 62; for (int h = 0; h < 12; h++) { int v = wx.rainProb[h] * 12 / 100; radar->fillRect(x0 + h * 10, 214 - v, 7, v ? v : 1, wx.rainProb[h] >= 40 ? rgb(60, 140, 255) : rgb(40, 40, 48)); }
  radar->textC(226, "rain next 12 h", FSR, rgb(70, 70, 70));
}
void paintClockRightAgenda() {
  char b[40]; uint32_t now = time(NULL); int cn = calNow(), nx = calNext();
  if (!macLinked()) { radar->textC(124, "Mac helper offline", FSR, COL_TEXT_DIM); return; }
  if (cn >= 0) {
    const CalEvent &c = calEv[cn]; struct tm et; time_t ee = c.e; localtime_r(&ee, &et);
    radar->textC(44, "NOW", FSR, COL_DEP);
    char lines[3][40]; int nl = wrapText(calTitle(c), FM, 180, lines, 2); for (int l = 0; l < nl; l++) radar->textC(84 + l * 26, lines[l], FM, COL_TEXT);
    fmtHMA(b, sizeof(b), et.tm_hour, et.tm_min); char u[40]; snprintf(u, sizeof(u), "until %s", b); radar->textC(150, u, FS, COL_TEXT_DIM);
    long left = (long)c.e - (long)now; snprintf(b, sizeof(b), "%ld min left", (left + 59) / 60); radar->textC(176, b, FMB, COL_ARR);
    return;
  }
  if (nx < 0) { radar->textC(100, "no more", FM, COL_TEXT_DIM); radar->textC(126, "meetings today", FM, COL_TEXT_DIM); return; }
  const CalEvent &c = calEv[nx]; struct tm st; time_t ss = c.t; localtime_r(&ss, &st);
  radar->textC(44, "NEXT", FSR, COL_TEXT_DIM);
  fmtHMA(b, sizeof(b), st.tm_hour, st.tm_min); radar->textC(84, b, FB2, COL_TEXT);
  char lines[3][40]; int nl = wrapText(calTitle(c), FS, 180, lines, 2); for (int l = 0; l < nl; l++) radar->textC(114 + l * 20, lines[l], FS, COL_TEXT);
  long to = (long)c.t - (long)now;
  if (to >= 3600) snprintf(b, sizeof(b), "in %ld h %ld m", to / 3600, (to % 3600) / 60); else snprintf(b, sizeof(b), "in %ld min", (to + 59) / 60);
  radar->textC(176, b, FMB, to <= 600 ? COL_ARR : COL_TEXT_DIM);
}
void paintClockRightWorld() {
  char b[40]; zoneTick(); struct tm t; bool haveT = getTime(t);
  radar->textC(38, "WORLD", FSR, COL_TEXT_DIM);
  for (int i = 0; i < NUM_ZONES && i < 2; i++) {
    int y = 90 + i * 66; bool day = zoneT[i].hour >= 6 && zoneT[i].hour < 18;
    radar->textC(y - 26, ZONES[i].name, FS, COL_TEXT_DIM);
    fmtHMA(b, sizeof(b), zoneT[i].hour, zoneT[i].min); radar->textC(y + 2, b, FMB, day ? COL_TEXT : rgb(150, 150, 170));
    if (haveT && zoneT[i].wday != t.tm_wday) radar->textC(y + 20, ((zoneT[i].wday + 1) % 7 == t.tm_wday) ? "yesterday" : "tomorrow", FSR, rgb(90, 90, 90));
  }
}
void paintClockRightSun() {
  char b[40];
  radar->textC(38, "SUN AND MOON", FSR, COL_TEXT_DIM);
  if (wx.sunriseMin < 0) { radar->textC(124, "waiting for weather", FSR, COL_TEXT_DIM); return; }
  struct tm t; int nowM = getTime(t) ? t.tm_hour * 60 + t.tm_min : 0;
  for (int a = -90; a <= 90; a += 3) { int x, y; polarC(120, 118, a, 70, x, y); radar->fillCircle(x, y, 1, COL_LINE); }
  float f = (float)(nowM - wx.sunriseMin) / (float)(wx.sunsetMin - wx.sunriseMin);
  if (f >= 0 && f <= 1) { int x, y; polarC(120, 118, -90 + f * 180, 70, x, y); radar->fillCircle(x, y, 8, rgb(255, 190, 40)); }
  radar->drawLine(40, 118, 200, 118, COL_LINE);
  fmtHMA(b, sizeof(b), wx.sunriseMin / 60, wx.sunriseMin % 60); radar->text(34, 140, b, FS, COL_ARR);
  fmtHMA(b, sizeof(b), wx.sunsetMin / 60, wx.sunsetMin % 60); radar->textR(206, 140, b, FS, COL_DEP);
  float ph = moonPhase(); int illum = (int)(100 * (1 - fabsf(ph - 0.5f) * 2));
  int mx = 70, my = 184; radar->fillCircle(mx, my, 16, rgb(215, 215, 225)); float k = ph < 0.5f ? (0.5f - ph) * 2 : (ph - 0.5f) * 2; int off = (int)(32 * k); radar->fillCircle(mx + (ph < 0.5f ? -off : off), my, 16, COL_BG);
  snprintf(b, sizeof(b), "%s %d%%", ph < 0.5f ? "waxing" : "waning", illum); radar->text(96, 180, b, FS, COL_TEXT);
  int len = wx.sunsetMin - wx.sunriseMin; snprintf(b, sizeof(b), "%dh %02dm of day", len / 60, len % 60); radar->text(96, 198, b, FSR, COL_TEXT_DIM);
}
void paintClockRightTraffic() {
  char b[40]; int idx[MAX_PLANES]; int m = boardList(idx, 3, true, planes, planeCount, homeLat, homeLon);
  radar->textC(38, "NEXT ARRIVALS", FSR, COL_TEXT_DIM);
  if (!m) { snprintf(b, sizeof(b), "%d in range", planeCount); radar->textC(110, "no arrivals", FM, COL_TEXT_DIM); radar->textC(136, b, FS, COL_TEXT_DIM); return; }
  for (int r = 0; r < m; r++) {
    const Plane &p = planes[idx[r]]; int y = 84 + r * 46; char pcs[12]; prettyCallsign(p.callsign, pcs, sizeof(pcs));
    radar->text(40, y, pcs, r == 0 ? FMB : FM, r == 0 ? COL_TEXT : COL_TEXT_DIM);
    snprintf(b, sizeof(b), "~%d min", etaMin(p, p.distance_km)); radar->textR(200, y, b, FS, COL_ARR);
    const char *rt = routeFor(p.callsign); snprintf(b, sizeof(b), "%s  %.0f km", rt ? rt : typeName(p.type), p.distance_km); radar->text(40, y + 18, b, FSR, rgb(90, 90, 90));
  }
  snprintf(b, sizeof(b), "%d aircraft in range", planeCount); radar->textC(224, b, FSR, rgb(70, 70, 70));
}
void paintClockRight() {
  radar->fillScreen(COL_BG);
  int what = clkRight; if (clockFace == 3 && what == 2) what = 3;   // the World face already shows the zones
  switch (what) {
    case 1: paintClockRightAgenda(); break;
    case 2: paintClockRightWorld(); break;
    case 3: paintClockRightSun(); break;
    case 4: paintClockRightTraffic(); break;
    default: paintClockRightWeather(); break;
  }
  if (issOn && issPass.valid && issPass.visible && issPass.startT > (uint32_t)time(NULL) && what != 4) { time_t st = issPass.startT; struct tm pt; localtime_r(&st, &pt); char hm[12], b[40]; fmtHMA(hm, sizeof(hm), pt.tm_hour, pt.tm_min); snprintf(b, sizeof(b), "ISS %s  %.0f%s", hm, issPass.maxEl, DEG); radar->textC(226, b, FSR, COL_ARR); }
}
// WEATHER: next 12 hours: temperature line, rain chance bars, rain in the next 2 h
void paintWeatherRight() {
  radar->fillScreen(COL_BG); char b[40];
  radar->textC(36, "NEXT 12 HOURS", FS, COL_TEXT_DIM);
  if (!wx.valid) { radar->textC(124, "fetching weather", FSR, COL_TEXT_DIM); return; }
  struct tm t; int h0 = getTime(t) ? t.tm_hour : 0;
  float mn = wx.hourly[0], mx = wx.hourly[0];
  for (int i = 1; i < 12; i++) { if (wx.hourly[i] < mn) mn = wx.hourly[i]; if (wx.hourly[i] > mx) mx = wx.hourly[i]; }
  if (mx - mn < 2) mx = mn + 2;
  const int x0 = 36, x1 = 204, yT = 62, yB = 112;
  for (int i = 0; i < 11; i++) {
    int xa = x0 + i * (x1 - x0) / 11, xb = x0 + (i + 1) * (x1 - x0) / 11;
    int ya = yB - (int)((wx.hourly[i] - mn) / (mx - mn) * (yB - yT)), yb = yB - (int)((wx.hourly[i + 1] - mn) / (mx - mn) * (yB - yT));
    radar->drawLine(xa, ya, xb, yb, COL_ARR); radar->drawLine(xa, ya + 1, xb, yb + 1, COL_ARR);
  }
  snprintf(b, sizeof(b), "%.0f" DEG, mx); radar->text(x0 - 4, yT - 4, b, FSR, COL_TEXT_DIM);
  snprintf(b, sizeof(b), "%.0f" DEG, mn); radar->text(x0 - 4, yB + 12, b, FSR, COL_TEXT_DIM);
  // rain chance bars, same hours
  const int rB = 168, rH = 34;
  for (int i = 0; i < 12; i++) {
    int x = x0 + i * (x1 - x0) / 11 - 5, hgt = wx.rainProb[i] * rH / 100;
    radar->drawRect(x, rB - rH, 10, rH, rgb(25, 25, 35));
    if (hgt) radar->fillRect(x, rB - hgt, 10, hgt, wx.rainProb[i] >= 60 ? rgb(60, 140, 255) : rgb(40, 90, 170));
  }
  for (int i = 0; i < 12; i += 3) { int hh = (h0 + i) % 24; snprintf(b, sizeof(b), "%d%s", hh % 12 ? hh % 12 : 12, hh < 12 ? "a" : "p"); radar->text(x0 + i * (x1 - x0) / 11 - radar->textWidth(FSR, b) / 2, rB + 14, b, FSR, rgb(90, 90, 90)); }
  radar->text(x0 + 140, rB - rH - 4, "rain %", FSR, rgb(70, 70, 90));
  int rainIn = -1; float mm = 0; for (int i = 0; i < 8; i++) { if (wx.rain15[i] >= 0.2f && rainIn < 0) rainIn = i * 15; mm += wx.rain15[i] > 0 ? wx.rain15[i] : 0; }
  if (rainIn == 0) snprintf(b, sizeof(b), "raining: %.1f mm in 2 h", mm); else if (rainIn > 0) snprintf(b, sizeof(b), "rain in ~%d min", rainIn + 5); else strcpy(b, "no rain in the next 2 h");
  radar->textC(208, b, FS, rainIn >= 0 ? rgb(90, 160, 255) : COL_TEXT_DIM);
}
// POMODORO: this week (the left dial already has today, the goal ring, streak and the 7 days at home)
// 6.18.1: POMODORO right dial. Running: big countdown, progress around the rim, session dots, the task. Idle: today
// against the goal as a ring, streak, this week.
void paintPomoRight() {
  radar->fillScreen(COL_BG); char b[40], w[16];
  if (pomoState != POMO_SELECT && pomoTotalMs) {
    bool focus = pomoState == POMO_FOCUS || (pomoState == POMO_PAUSED && pomoWasFocus), paused = pomoState == POMO_PAUSED;
    unsigned long rem = pomoRemainMs(); float f = 1.0f - (float)rem / (float)pomoTotalMs; int lit = (int)(f * 24 + 0.5f);
    uint16_t col = paused ? rgb(90, 90, 90) : focus ? COL_ARR : rgb(40, 200, 90);
    for (int k = 0; k < 24; k++) radar->ringArc(RADAR_CX, RADAR_CY, 118, 108, k * 15 + 1.5f, k * 15 + 13.5f, k < lit ? col : rgb(28, 28, 32));
    radar->textC(58, paused ? "PAUSED" : focus ? "FOCUS" : (breathing ? "BREATHE" : "BREAK"), FS, col);
    snprintf(b, sizeof(b), "%02lu:%02lu", rem / 60000, (rem / 1000) % 60); radar->textC(RADAR_CY + 20, b, FB, paused ? COL_TEXT_DIM : COL_TEXT);
    for (int i = 0; i < POMO_SESSIONS_PER_CYCLE; i++) { int x = RADAR_CX - (POMO_SESSIONS_PER_CYCLE - 1) * 9 + i * 18; bool done = i < (pomoSession % POMO_SESSIONS_PER_CYCLE) || (pomoSession > 0 && pomoSession % POMO_SESSIONS_PER_CYCLE == 0 && !focus); bool cur = focus && i == pomoSession % POMO_SESSIONS_PER_CYCLE; if (done) radar->fillCircle(x, 164, 5, COL_ARR); else if (cur) { radar->drawCircle(x, 164, 5, COL_ARR); radar->fillCircle(x, 164, 2, COL_ARR); } else radar->drawCircle(x, 164, 5, rgb(70, 70, 70)); }
    if (focus && pomoTask[0]) radar->textC(190, pomoTask, FS, COL_TEXT);
    else if (focus) { snprintf(b, sizeof(b), "then %d min break", (pomoSession + 1) % POMO_SESSIONS_PER_CYCLE == 0 ? POMO_LONG_BREAK_MIN : pomoBreakMin()); radar->textC(190, b, FSR, COL_TEXT_DIM); }
    else if (breathing) { float ph = fmodf((millis() - breathStartMs) / 1000.0f, 16.0f); const char *st = ph < 4 ? "breathe in" : ph < 8 ? "hold" : ph < 12 ? "breathe out" : "hold"; radar->textC(190, st, FS, rgb(40, 200, 90)); }
    else { snprintf(b, sizeof(b), "next: %d min focus", pomoFocusMin()); radar->textC(190, b, FSR, COL_TEXT_DIM); }
    fmtHMs(w, sizeof(w), todayFocus()); snprintf(b, sizeof(b), "today %s   streak %d", w, streakNow()); radar->textC(212, b, FSR, rgb(80, 80, 80));
    return;
  }
  // idle: today against the goal
  unsigned today = todayFocus(), goal = pcfg.goalMin ? pcfg.goalMin : 1; int lit = (int)((float)today / goal * 24 + 0.5f); if (lit > 24) lit = 24;
  for (int k = 0; k < 24; k++) radar->ringArc(RADAR_CX, RADAR_CY, 118, 108, k * 15 + 1.5f, k * 15 + 13.5f, k < lit ? (today >= goal ? rgb(0, 190, 80) : COL_ARR) : rgb(28, 28, 32));
  radar->textC(58, "TODAY", FS, COL_TEXT_DIM);
  fmtHMs(w, sizeof(w), today); radar->textC(RADAR_CY + 12, w, FB2, COL_TEXT);
  fmtHMs(w, sizeof(w), goal); snprintf(b, sizeof(b), "goal %s   %d sessions", w, todaySessions()); radar->textC(RADAR_CY + 36, b, FSR, COL_TEXT_DIM);
  unsigned week = 0; for (int i = 0; i < 7; i++) week += last7(i); if (fstats.valid && fstats.week > week) week = fstats.week;
  fmtHMs(w, sizeof(w), week); snprintf(b, sizeof(b), "this week %s", w); radar->textC(178, b, FS, COL_TEXT);
  snprintf(b, sizeof(b), "streak %d days", streakNow()); radar->textC(198, b, FSR, COL_TEXT_DIM);
  if (fstats.valid && fstats.prev) { int pc = (int)(((long)week - (long)fstats.prev) * 100 / (long)fstats.prev); snprintf(b, sizeof(b), "%+d%% vs last week", pc); radar->textC(216, b, FSR, pc >= 0 ? C_GREEN : C_AMBER); }
}
// MARKETS: every other symbol as a coloured tile (the left dial has the selected one)
void paintMarketRight() {
  radar->fillScreen(COL_BG); char b[24];
  radar->textC(34, "MARKETS", FS, COL_TEXT_DIM);
  int k = 0;
  for (int i = 0; i < NUM_SYMBOLS && k < 6; i++) {
    if (i == marketSel) continue;
    const Quote &q = quotes[i]; int x = i == -1 ? 0 : 32 + (k % 2) * 90, y = 46 + (k / 2) * 52; k++;
    float c = q.valid && q.prevClose > 0 ? (q.price - q.prevClose) / q.prevClose * 100 : 0; float f = fabsf(c) > 2.5f ? 1.0f : fabsf(c) / 2.5f;
    uint16_t fill = !q.valid ? rgb(22, 22, 22) : c >= 0 ? rgb(0, (uint8_t)(40 + 110 * f), (uint8_t)(20 + 40 * f)) : rgb((uint8_t)(50 + 150 * f), (uint8_t)(10 + 20 * f), (uint8_t)(10 + 20 * f));
    radar->fillRoundRect(x, y, 86, 48, 6, fill);
    radar->text(x + 6, y + 14, MARKET_SHORT[i], FSR, COL_TEXT);
    if (q.valid) { fmtPrice(q.price, b, sizeof(b), q.price > 9999 ? 0 : (q.price > 999 ? 1 : 2)); radar->text(x + 6, y + 29, b, FS, COL_TEXT); snprintf(b, sizeof(b), "%+.2f%%", c); radar->text(x + 6, y + 43, b, FSR, COL_TEXT); }
    else radar->text(x + 6, y + 30, "...", FS, COL_TEXT_DIM);
  }
}
// NEWS: the next three headlines (the left dial has the current one, the OLED its position and age)
void paintNewsRight() {
  radar->fillScreen(COL_BG);
  radar->textC(34, "NEXT", FS, COL_TEXT_DIM);
  if (newsCount < 2) { radar->textC(124, "no more headlines", FSR, COL_TEXT_DIM); return; }
  int y = 58;
  for (int k = 1; k <= 3 && k < newsCount; k++) {
    const Headline &h = news[(newsScroll + k) % newsCount];
    char lines[3][40]; int nl = wrapText(h.text, FSR, 178, lines, 3);
    for (int l = 0; l < nl; l++) { radar->text(32, y, lines[l], FSR, k == 1 ? COL_TEXT : rgb(150, 150, 150)); y += 14; }
    y += 10; if (y > 210) break;
  }
}
// F1: last podium as podium blocks
void paintF1Right() {
  radar->fillScreen(COL_BG);
  radar->textC(40, "LAST PODIUM", FS, COL_TEXT_DIM);
  if (!f1.valid || !f1.podium[0][0]) { radar->textC(124, "no result yet", FSR, COL_TEXT_DIM); return; }
  const int base = 196; const int hgt[3] = {70, 50, 34}; const int xs[3] = {120, 66, 174};
  const uint16_t col[3] = {rgb(212, 175, 55), rgb(170, 170, 180), rgb(176, 110, 60)};
  for (int i = 0; i < 3; i++) {
    if (!f1.podium[i][0]) continue;
    radar->fillRect(xs[i] - 26, base - hgt[i], 52, hgt[i], dim565(col[i], 0.45f));
    char n[4]; snprintf(n, sizeof(n), "%d", i + 1); radar->textC(0, "", FSR, 0);
    radar->text(xs[i] - radar->textWidth(FMB, n) / 2, base - hgt[i] + 24, n, FMB, col[i]);
    char nm[14]; strlcpy(nm, f1.podium[i], sizeof(nm)); const char *sp = strrchr(nm, ' '); const char *last = sp ? sp + 1 : nm;
    radar->text(xs[i] - radar->textWidth(FSR, last) / 2, base - hgt[i] - 6, last, FSR, COL_TEXT);
  }
  radar->drawLine(34, base, 206, base, COL_LINE);
}
// STATS / SYSTEM: the next page (the right knob turns both dials)
void paintStatsRight()  { int sp = subPage; subPage = (subPage + 1) % 3; paintStats();  subPage = sp; }
void paintSystemRight() { int sp = subPage; subPage = (subPage + 1) % 3; paintSystem(); subPage = sp; }
// NETWORK: WiFi signal over the last 48 min (sampled every 30 s)
int8_t rssiHist[96]; int rssiN = 0, rssiIdx = 0;
void rssiTick() { static unsigned long last = 0; if (last && millis() - last < 30000) return; last = millis(); rssiHist[rssiIdx] = (int8_t)WiFi.RSSI(); rssiIdx = (rssiIdx + 1) % 96; if (rssiN < 96) rssiN++; }
void paintNetworkRight() {
  radar->fillScreen(COL_BG); char b[32];
  radar->textC(36, "WIFI SIGNAL, 48 MIN", FS, COL_TEXT_DIM);
  const int x0 = 34, x1 = 206, yT = 70, yB = 170;
  for (int db = -90; db <= -30; db += 20) { int y = yB - (db + 90) * (yB - yT) / 60; radar->drawLine(x0, y, x1, y, rgb(28, 28, 28)); snprintf(b, sizeof(b), "%d", db); radar->text(x0 - 2, y - 2, b, FSR, rgb(70, 70, 70)); }
  if (rssiN < 2) { radar->textC(124, "collecting", FSR, COL_TEXT_DIM); return; }
  int px = -1, py = -1;
  for (int k = 0; k < rssiN; k++) {
    int v = rssiHist[(rssiIdx - rssiN + k + 96) % 96]; if (v < -90) v = -90; if (v > -30) v = -30;
    int x = x0 + k * (x1 - x0) / 95, y = yB - (v + 90) * (yB - yT) / 60;
    if (px >= 0) radar->drawLine(px, py, x, y, v > -60 ? COL_OK : v > -75 ? COL_DEP : COL_SELECTED);
    px = x; py = y;
  }
  int v = rssiHist[(rssiIdx + 95) % 96]; snprintf(b, sizeof(b), "now %d dBm  %s", v, v > -60 ? "strong" : v > -75 ? "fair" : "weak");
  radar->textC(200, b, FS, v > -60 ? COL_OK : v > -75 ? COL_DEP : COL_SELECTED);
}
// DISPLAY: live preview of the theme and brightness
void paintBrightRight() {
  radar->fillScreen(COL_BG);
  radar->textC(36, "PREVIEW", FS, COL_TEXT_DIM);
  for (int r = 30; r <= 90; r += 30) radar->drawCircle(120, 130, r, COL_RING);
  radar->drawLine(120, 130, 186, 92, COL_SWEEP); radar->drawLine(120, 131, 186, 93, COL_SWEEP);
  radar->fillTriangle(80, 104, 88, 104, 84, 112, COL_ARR); radar->fillTriangle(160, 160, 168, 160, 164, 152, COL_DEP); radar->fillCircle(150, 104, 4, COL_ENROUTE_LOW);
  radar->textC(214, THEME_NAMES[theme], FS, COL_TEXT);
}
// LOCATION: every airport in the table on a world map, the highlighted one in red, radar home in blue
void paintLocationRight() {
  radar->fillScreen(COL_BG);
  radar->textC(40, locTarget ? "WEATHER LOCATION" : "RADAR HOME", FSR, COL_TEXT_DIM);
  auto mapXY = [](double lat, double lon, int &x, int &y) { x = 24 + (int)((lon + 130) / 290.0 * 192); y = 160 - (int)((lat + 45) / 115.0 * 100); };
  for (int lon = -120; lon <= 150; lon += 30) { int x, y0, y1, dummy; mapXY(-45, lon, x, y0); mapXY(70, lon, dummy, y1); radar->drawLine(x, y1, x, y0, rgb(20, 24, 30)); }
  int sel = locAirport(listSel < locCount() ? listSel : 0);
  for (int i = 0; i < NUM_AIRPORTS; i++) { int x, y; mapXY(AIRPORTS[i].lat, AIRPORTS[i].lon, x, y); radar->fillCircle(x, y, 1, COL_TEXT_DIM); }
  { int x, y; mapXY(AIRPORTS[homeIndex].lat, AIRPORTS[homeIndex].lon, x, y); radar->fillCircle(x, y, 4, COL_HOME); }
  if (sel >= 0) { int x, y; mapXY(AIRPORTS[sel].lat, AIRPORTS[sel].lon, x, y); radar->drawCircle(x, y, 7, COL_SELECTED); radar->fillCircle(x, y, 3, COL_SELECTED); radar->textC(196, AIRPORTS[sel].name, FS, COL_TEXT); }
  else radar->textC(196, WX_HOME_NAME, FS, COL_TEXT);
}
// MENU: the highlighted app, big, with its description
void paintMenuRight() {
  radar->fillScreen(COL_BG);
  int a = menuItems[menuSel];
  radar->textC(104, APP_NAMES[a], FMB, COL_TEXT);
  char lines[3][40]; int nl = wrapText(APP_DESC[a], FS, 176, lines, 3);
  for (int l = 0; l < nl; l++) radar->textC(132 + l * 18, lines[l], FS, COL_TEXT_DIM);
  for (int i = 0; i < menuCount; i++) { int x, y; polar(i * 360.0f / menuCount, 108, x, y); bool cur = i == menuSel; radar->fillCircle(x, y, cur ? 5 : 2, cur ? COL_SWEEP : COL_RING_DIM); }
}
void paintBrightLevel() {   // 6.17.0: the brightness gesture in progress: rings (hold left, turn right) or round displays (hold right, turn left)
  radar->fillScreen(COL_BG); char b[12]; bool ring = brightWhat == 1; int v = ring ? ringBright : tftBright;
  radar->textC(62, ring ? "RING BRIGHTNESS" : "DISPLAY BRIGHTNESS", FS, COL_TEXT_DIM);
  int lit = (v * 24 + 50) / 100;   // 24 segments, like the ring LEDs
  for (int i = 0; i < 24; i++) radar->ringArc(RADAR_CX, RADAR_CY, 102, 90, i * 15 + 1.5f, i * 15 + 13.5f, i < lit ? (ring ? COL_SWEEP : COL_TEXT) : rgb(34, 34, 40));
  snprintf(b, sizeof(b), "%d%%", v); radar->textC(RADAR_CY + 14, b, FB2, COL_TEXT);
  radar->textC(192, ring ? "hold left, turn right" : "hold right, turn left", FSR, COL_TEXT_DIM);
}
PaintFn rightPaintFor() {
  if (brightMode()) return paintBrightLevel;
  if (menuOpen) return paintMenuRight;
  switch (currentApp) {
    case APP_RADAR: return paintRadarRight;
    case APP_ARRIVALS: case APP_APPROACH: return paintBoardRight;
    case APP_CLOCK: return paintClockRight;
    case APP_WEATHER: return paintWeatherRight;
    case APP_POMODORO: return paintPomoRight;
    case APP_MARKET: return paintMarketRight;
    case APP_NEWS: return paintNewsRight;
    case APP_F1: return paintF1Right;
    case APP_STATS: return paintStatsRight;
    case APP_SYSTEM: return paintSystemRight;
    case APP_NETWORK: return paintNetworkRight;
    case APP_BRIGHT: return paintBrightRight;
    case APP_LOCATION: return paintLocationRight;
    case APP_AGENDA: return paintAgendaRight;
    case APP_SETTINGS: return paintSettingsRight;
    case APP_MEDIA: return paintMediaRight;
    default: return paintCompanion;
  }
}
// ===== 6.3.0 left OLED pages (return false = use the v5 page) =====
bool drawInfoV6() {
  char b[40], c[40]; struct tm t; bool haveT = getTime(t);
  switch (currentApp) {
    case APP_RADAR: drawInboundTimeline(); return true;
    case APP_CLOCK:
      if (clockFace == 3) { nextHoliday(b, sizeof(b)); o6Line(0, b[0] ? b : "no holiday soon", NULL); o6Band("HOLIDAY", NULL); return true; }
      if (wx.sunriseMin < 0) { o6Line(1, "waiting for weather", NULL); o6Band("SUN", NULL); return true; }
      fmtHMA(b, sizeof(b), wx.sunriseMin / 60, wx.sunriseMin % 60); o6Line(0, "sunrise", b);
      fmtHMA(b, sizeof(b), wx.sunsetMin / 60, wx.sunsetMin % 60); o6Line(1, "sunset", b);
      { int len = wx.sunsetMin - wx.sunriseMin; snprintf(b, sizeof(b), "%dh %02dm", len / 60, len % 60); o6Line(2, "daylight", b); }
      o6Band("SUN", NULL); return true;
    case APP_WEATHER:
      if (!wx.valid) { o6Line(1, "fetching", NULL); o6Band("AIR", NULL); return true; }
      if (!isnan(envT)) {   // indoor sensor on the desk
        if (envChip == 0x60 && !isnan(envH)) snprintf(b, sizeof(b), "%.1f C %.0f%%", envT, envH); else snprintf(b, sizeof(b), "%.1f C", envT); o6Line(0, "indoor", b);
        snprintf(b, sizeof(b), "%.0f hPa", envP); o6Line(1, "pressure", b);
      } else { snprintf(b, sizeof(b), "%d", wx.pm25); o6Line(0, "PM2.5", b); snprintf(b, sizeof(b), "%.0f %s", wx.uv, wx.uv >= 8 ? "very high" : wx.uv >= 6 ? "high" : wx.uv >= 3 ? "moderate" : "low"); o6Line(1, "UV", b); }
      snprintf(b, sizeof(b), "%d / %d", wx.pm25, wx.pm10); o6Line(2, "PM2.5 / 10", b);
      snprintf(b, sizeof(b), "AQI %s", aqiText(wx.aqi)); snprintf(c, sizeof(c), "UV %.0f", wx.uv); o6Band(b, c); return true;
    case APP_MARKET: {
      bool open = haveT && marketOpenNow(t);
      if (haveT) {
        int nowM = t.tm_hour * 60 + t.tm_min;
        if (open) { int left = 15 * 60 + 30 - nowM; snprintf(b, sizeof(b), "%dh %02dm", left / 60, left % 60); o6Line(0, "closes in", b); }
        else { bool wk = t.tm_wday >= 1 && t.tm_wday <= 5; o6Line(0, (wk && nowM < 9 * 60 + 15) ? "opens 9:15 today" : (t.tm_wday == 5 || t.tm_wday == 6) ? "opens Mon 9:15" : "opens 9:15 tmrw", NULL); }
      }
      if (marketSel < NUM_SYMBOLS && quotes[marketSel].valid) {
        const Quote &q = quotes[marketSel]; int d = q.price > 9999 ? 0 : 2;
        fmtPrice(q.low, b, sizeof(b), d); fmtPrice(q.high, c, sizeof(c), d); char lo[24], hi[24]; snprintf(lo, sizeof(lo), "lo %s", b); snprintf(hi, sizeof(hi), "hi %s", c); o6Line(1, lo, hi);
        fmtPrice(q.prevClose, b, sizeof(b), d); o6Line(2, "prev close", b);
      }
      if (fng >= 0 && marketSel != NUM_SYMBOLS - 1) { snprintf(b, sizeof(b), "F&G %d", fng); o6Band(open ? "OPEN" : "CLOSED", b); }
      else o6Band(open ? "NSE OPEN" : "NSE CLOSED", NULL);
      return true; }
    case APP_ARRIVALS: case APP_APPROACH: {
      bool other = currentApp == APP_ARRIVALS && boardAirport != homeIndex;
      if (other || !metar.valid) { o6Line(1, other ? "runway info: home airport only" : "no METAR yet", NULL); o6Band(currentApp == APP_ARRIVALS ? "RUNWAY" : "WEATHER", NULL); return true; }
      int rwy = activeRwy > 0 ? activeRwy : rwyHdg() / 10;
      if (currentApp == APP_ARRIVALS) {   // runway drawn in its direction with the wind arrow, components on the right
        int cx = 26, cy = 30; float a = toRad(rwy * 10.0f);
        int dx = (int)(22 * sinf(a)), dy = (int)(-22 * cosf(a)); infoDisplay.drawLine(cx - dx, cy - dy, cx + dx, cy + dy, SSD1306_WHITE); infoDisplay.drawLine(cx - dx + 1, cy - dy, cx + dx + 1, cy + dy, SSD1306_WHITE);
        if (metar.wdir >= 0) { float w = toRad((float)metar.wdir); int wx0 = cx + (int)(20 * sinf(w)), wy0 = cy - (int)(20 * cosf(w)), wx1 = cx + (int)(6 * sinf(w)), wy1 = cy - (int)(6 * cosf(w));
          infoDisplay.drawLine(wx0, wy0, wx1, wy1, SSD1306_WHITE); infoDisplay.fillCircle(wx1, wy1, 2, SSD1306_WHITE); }
        float rel = toRad((float)(metar.wdir < 0 ? 0 : metar.wdir) - rwy * 10.0f);
        int head = (int)lroundf(metar.wspd * cosf(rel)), cross = (int)lroundf(metar.wspd * sinf(rel));
        infoDisplay.setFont(&F_SM13);
        snprintf(b, sizeof(b), "RWY %02d", rwy); infoDisplay.setCursor(56, 20); infoDisplay.print(b);
        snprintf(b, sizeof(b), "%s %d kt", head >= 0 ? "head" : "tail", abs(head)); infoDisplay.setCursor(56, 33); infoDisplay.print(b);
        snprintf(b, sizeof(b), "cross %d %s", abs(cross), cross >= 0 ? "R" : "L"); infoDisplay.setCursor(56, 46); infoDisplay.print(b);
        infoDisplay.setFont();
        snprintf(b, sizeof(b), "QNH %d", metar.qnh); snprintf(c, sizeof(c), "%d/%d C", metar.temp, metar.dewp); o6Band(b, c);
      } else {   // APPROACH: METAR / TAF decoded
        if (metar.wdir >= 0) snprintf(b, sizeof(b), "%03d / %d kt%s", metar.wdir, metar.wspd, metar.wgst > 0 ? " G" : ""); else snprintf(b, sizeof(b), "VRB / %d kt", metar.wspd);
        o6Line(0, "wind", b);
        snprintf(b, sizeof(b), "%s  %s", metar.visib, metar.fltCat); o6Line(1, "vis", b);
        if (taf.valid && taf.nextRwy > 0) { time_t ct = taf.changeT; struct tm ctm; localtime_r(&ct, &ctm); fmtHMA(c, sizeof(c), ctm.tm_hour, ctm.tm_min); snprintf(b, sizeof(b), "rwy %02d at %s", taf.nextRwy, c); o6Line(2, "TAF", b); }
        else { snprintf(b, sizeof(b), "%d", metar.qnh); o6Line(2, "QNH", b); }
        c[0] = 0; { const char *z = strstr(metar.raw, "Z "); if (z && z - metar.raw >= 6) { int hh = (z[-4] - '0') * 10 + (z[-3] - '0'), mm = (z[-2] - '0') * 10 + (z[-1] - '0'); int lm = (hh * 60 + mm + TZ_OFFSET_SEC / 60) % 1440; char hm[12]; fmtHMA(hm, sizeof(hm), lm / 60, lm % 60); snprintf(c, sizeof(c), "%s", hm); } }
        o6Band("METAR", c);
      }
      return true; }
    case APP_NEWS: {
      if (!newsCount) { o6Line(1, "fetching", NULL); o6Band("NEWS", NULL); return true; }
      snprintf(b, sizeof(b), "%d of %d", newsScroll + 1, newsCount); o6Line(0, "headline", b);
      snprintf(b, sizeof(b), "%lu min ago", newsMs ? (millis() - newsMs) / 60000 : 0); o6Line(1, "updated", b);
      o6Line(2, "BBC, The Hindu", NULL); o6Band("NEWS", NULL); return true; }
    case APP_F1: {
      if (!f1.valid) { o6Line(1, "waiting for data", NULL); o6Band("F1", NULL); return true; }
      o6Line(0, f1.circuit, NULL);
      if (f1.t) { time_t rt = (time_t)f1.t; struct tm r; localtime_r(&rt, &r); static const char *D[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"}; static const char *M[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
        snprintf(b, sizeof(b), "%s %d %s", D[r.tm_wday], r.tm_mday, M[r.tm_mon]); o6Line(1, "race", b); fmtHMA(b, sizeof(b), r.tm_hour, r.tm_min); o6Line(2, "lights out", b); }
      o6Band("FORMULA 1", NULL); return true; }
    case APP_STATS: {
      int na, nd, ne; countClasses(na, nd, ne);
      snprintf(b, sizeof(b), "%d", planeCount); o6Line(0, "in range now", b);
      snprintf(b, sizeof(b), "%d / %d / %d", na, nd, ne); o6Line(1, "a / d / o", b);
      o6Band("LIVE", NULL); return true; }
    case APP_SYSTEM: {
      if (sdOk) { snprintf(b, sizeof(b), "%.1f GB", (SD.totalBytes() - SD.usedBytes()) / 1073741824.0f); o6Line(0, "SD free", b); } else o6Line(0, "SD", "no card");
      snprintf(b, sizeof(b), "%.1f MB", ESP.getFreePsram() / 1048576.0f); o6Line(1, "PSRAM", b);
      snprintf(b, sizeof(b), "%.0f C", temperatureRead()); o6Line(2, "chip", b);
      o6Band("HARDWARE", NULL); return true; }
    case APP_AGENDA: drawAgendaInfo(); return true;
    case APP_SETTINGS: drawSettingsInfo(); return true;
    case APP_MEDIA: drawMediaInfo(); return true;
    case APP_NETWORK: {
      o6Line(0, "gw", WiFi.gatewayIP().toString().c_str());
      o6Line(1, "dns", WiFi.dnsIP().toString().c_str());
      o6Line(2, "host", OTA_HOSTNAME);
      o6Band("LAN", NULL); return true; }
    default: return false;
  }
}
// ===== MEDIA app (6.9.0): the Mac helper reports now playing, volume and mute; the knobs send commands back =====
void splitNp(char *title, size_t tn, char *artist, size_t an) {
  strlcpy(title, mac.np, tn); artist[0] = 0;
  char *sep = strstr(title, " - "); if (sep) { *sep = 0; strlcpy(artist, sep + 3, an); }
}
void paintMedia() {
  radar->fillScreen(COL_BG); char b[48], title[64], artist[64];
  if (!macLinked()) { radar->textC(112, "Mac not linked", FM, COL_TEXT_DIM); radar->textC(134, "helper not running", FSR, rgb(80, 80, 80)); return; }
  if (!mac.app[0] || !mac.np[0]) { radar->textC(104, "nothing playing", FM, COL_TEXT_DIM); radar->textC(128, "start Music or Spotify", FSR, rgb(80, 80, 80)); return; }
  radar->textC(36, mac.app, FSR, COL_TEXT_DIM);
  // play / pause glyph
  if (mac.playing) { radar->fillTriangle(110, 48, 110, 66, 126, 57, COL_OK); } else { radar->fillRect(109, 48, 6, 18, COL_TEXT_DIM); radar->fillRect(119, 48, 6, 18, COL_TEXT_DIM); }
  splitNp(title, sizeof(title), artist, sizeof(artist));
  char lines[3][40]; int nl = wrapText(title, FM, 184, lines, 3);
  int y = 100; for (int l = 0; l < nl && l < 3; l++) { radar->textC(y, lines[l], FM, COL_TEXT); y += 24; }
  if (artist[0]) { char al[2][40]; int na = wrapText(artist, FS, 184, al, 2); for (int l = 0; l < na; l++) { radar->textC(y + 2, al[l], FS, COL_TEXT_DIM); y += 18; } }
}
void drawMediaFrame() { if (!staticDirty && millis() - lastHudMs < 1000) return; lastHudMs = millis(); staticDirty = false; present(paintMedia); }
// Right dial: volume as a big number inside an arc, mic mute state below
void paintMediaRight() {
  radar->fillScreen(COL_BG); char b[24];
  radar->textC(78, "MAC VOLUME", FSR, COL_TEXT_DIM);   // inside the arc (6.9.1)
  if (!macLinked() || mac.vol < 0) { radar->textC(124, "--", FB2, COL_TEXT_DIM); return; }
  radar->ringArc(RADAR_CX, RADAR_CY, 96, 106, -135, 135, COL_RING_DIM);
  radar->ringArc(RADAR_CX, RADAR_CY, 96, 106, -135, -135 + 270.0f * mac.vol / 100.0f, mac.vol > 80 ? COL_ARR : COL_OK);
  snprintf(b, sizeof(b), "%d", mac.vol); radar->textC(140, b, FB, COL_TEXT);
  radar->textC(176, mac.vol == 0 ? "silent" : mac.vol < 30 ? "quiet" : mac.vol < 70 ? "normal" : "loud", FSR, COL_TEXT_DIM);
  radar->textC(210, mac.muted ? "mic muted" : "mic live", FSR, mac.muted ? COL_DEP : rgb(80, 80, 80));
}
void drawMediaInfo() {
  char b[32];
  o6Head("MEDIA");
  if (!macLinked()) { o6Line(1, "Mac helper offline", NULL); o6Band("MEDIA", NULL); return; }
  o6Line(0, "source", mac.app[0] ? mac.app : "none");
  o6Line(1, "state", mac.playing ? "playing" : "paused");
  snprintf(b, sizeof(b), "%d%%", mac.vol < 0 ? 0 : mac.vol); o6Line(2, "volume", b);
  o6Band("turn: vol  click: play", NULL);
}
// ===== SETTINGS app (6.7.0) =====
// Right knob: turn = move, click = open category / edit (toggle for on-off, run for actions), hold = back.
// Values change live; saved to NVS 3 s after the last change (settingsDirty).
enum { ST_BOOL, ST_INT, ST_ENUM, ST_HOUR, ST_TIME, ST_MASK, ST_APP, ST_AIRPORT, ST_ACTION, ST_INFO, ST_U8, ST_U8ENUM, ST_U16, ST_WXLOC };   // ST_WXLOC 6.16.0: -2 house, -1 home airport, else airport
struct SetItem { uint8_t cat, type; const char *name; void *ptr; int mn, mx, step; const char *const *names; int nNames; const char *unit; const char *help; };
const char *SET_CATS[] = {"SOUND", "MEETINGS", "DISPLAY", "RADAR", "CLOCK", "POMODORO", "APPS", "SYSTEM"};
#define SET_NCAT 8
const char *const NIGHT_NAMES[] = {"auto", "always", "off"};
const char *const FILTER_NAMES[] = {"all", "arrivals", "departures"};
const char *const SRC_NAMES[] = {"OpenSky", "airplanes.live", "adsb.lol", "adsb.fi", "local"};
const char *const RFOC_NAMES[] = {"arrivals", "departures"};
const char *const RANGE_NAMES[] = {"10 km", "25 km", "50 km", "100 km", "200 km", "auto"};
const char *const LEAD_NAMES[] = {"2 min", "5 min", "10 min", "15 min"};
const int LEAD_VALS[] = {2, 5, 10, 15};
const char *const PQUIET_NAMES[] = {"off", "planes", "planes + market", "all alerts"};
const char *const PSOUND_NAMES[] = {"chime", "soft", "silent"};
const char *const ALDAYS_NAMES[] = {"weekdays", "daily", "weekends", "custom"};
const int ALDAYS_VALS[] = {0x3E, 0x7F, 0x41};
const char *const OSZ_NAMES[] = {"1x", "1.5x", "2x", "3x"};
const char *const NRING_NAMES[] = {"off", "nightlight", "normal"};
const char *const CLKR_NAMES[] = {"weather", "agenda", "world", "sun & moon", "traffic"};   // 6.18.0
const uint32_t MASK_PLANES = (1u << M_TRAFFIC) | (1u << M_HEAVY) | (1u << M_WATCH) | (1u << M_OVERHEAD) | (1u << M_GOAROUND);
const uint32_t MASK_WX = (1u << M_RAIN) | (1u << M_SUN) | (1u << M_QUAKE);
const uint32_t MASK_MKT = (1u << M_MARKET);
const uint32_t MASK_NUDGE = (1u << M_NUDGE);
int meetLeadIdx = 1, alDaysIdx = 0;   // enum mirrors for meetLead / alarmDays
enum { ACT_RESTART = 1, ACT_RESET, ACT_SAVE_SD, ACT_WIFI, ACT_CLEAR_CRASH };
const SetItem SET_ITEMS[] = {
  {0, ST_BOOL, "mute everything", &muteAll, 0, 0, 0, NULL, 0, "", "Every sound off. The alarm clock still rings."},
  {0, ST_BOOL, "knob clicks", &clickOn, 0, 0, 0, NULL, 0, "", "A short click on every knob action."},
  {0, ST_BOOL, "louder piezo", &piezoLoud, 0, 0, 0, NULL, 0, "", "Plays low notes an octave higher, where the piezo is loudest. Off: original pitches."},
  {0, ST_BOOL, "hour chime", &chimeOn, 0, 0, 0, NULL, 0, "", "Two notes on the hour."},
  {0, ST_HOUR, "quiet from", &quietStart, 0, 23, 1, NULL, 0, "", "No sounds between these hours. Alarm and emergencies still sound. Same hour both = off."},
  {0, ST_HOUR, "quiet until", &quietEnd, 0, 23, 1, NULL, 0, "", "End of the quiet hours."},
  {0, ST_MASK, "plane alerts", &alertMask, (int)MASK_PLANES, 0, 0, NULL, 0, "", "Inner ring traffic, heavies, watch list, overhead, go-arounds."},
  {0, ST_MASK, "weather alerts", &alertMask, (int)MASK_WX, 0, 0, NULL, 0, "", "Rain soon, sunset, earthquakes."},
  {0, ST_MASK, "market alerts", &alertMask, (int)MASK_MKT, 0, 0, NULL, 0, "", "Moves beyond the threshold on the web page."},
  {0, ST_MASK, "nudges", &alertMask, (int)MASK_NUDGE, 0, 0, NULL, 0, "", "Stand up, water and eye-break reminders."},
  {0, ST_U8ENUM, "pomodoro sound", &pcfg.sound, 0, 2, 1, PSOUND_NAMES, 3, "", "Session start and end sounds."},
  {1, ST_BOOL, "reminders", &meetOn, 0, 0, 0, NULL, 0, "", "Banner and chime before each calendar event."},
  {1, ST_ENUM, "first reminder", &meetLeadIdx, 0, 3, 1, LEAD_NAMES, 4, "", "How long before the event the first chime comes. A 1 min and a start cue follow."},
  {1, ST_BOOL, "show titles", &meetTitles, 0, 0, 0, NULL, 0, "", "Off: the device shows only 'Meeting' and the times."},
  {1, ST_BOOL, "on-air light", &onAirOn, 0, 0, 0, NULL, 0, "", "Both rings glow red while the Mac mic or camera is in use."},
  {1, ST_BOOL, "mic on left click", &micClick, 0, 0, 0, NULL, 0, "", "On a call, the left knob click mutes or unmutes the Mac mic."},
  {2, ST_INT, "round brightness", &tftBright, 10, 100, 5, NULL, 0, "%", "Both round displays. Quick: hold the right knob and turn the left."},
  {2, ST_INT, "OLED brightness", &oledBright, 0, 100, 5, NULL, 0, "%", "Both small displays."},
  {2, ST_INT, "ring brightness", &ringBright, 0, 100, 5, NULL, 0, "%", "Quick: hold the left knob and turn the right. 100 draws about 1.4 A."},
  {2, ST_ENUM, "theme", &theme, 0, 3, 1, THEME_NAMES, 4, "", "Colour theme of the dials."},
  {2, ST_ENUM, "night dimming", (void *)&nightMode, 0, 2, 1, NIGHT_NAMES, 3, "", "Auto dims between the night hours."},
  {2, ST_HOUR, "night from", &nightStart, 0, 23, 1, NULL, 0, "", "Start of night dimming (auto mode). Same hour both = off."},
  {2, ST_HOUR, "night until", &nightEnd, 0, 23, 1, NULL, 0, "", "End of night dimming."},
  {2, ST_HOUR, "screens off from", &sleepStart, 0, 23, 1, NULL, 0, "", "All screens and rings off between these hours. Any knob wakes them for 2 min. Same hour both = off."},
  {2, ST_HOUR, "screens on at", &sleepEnd, 0, 23, 1, NULL, 0, "", "End of the screens-off hours."},
  {2, ST_ENUM, "rings at night", &nightRings, 0, 2, 1, NRING_NAMES, 3, "", "During night dimming: off, a faint warm glow, or the normal content."},
  {2, ST_ENUM, "OLED text size", &oledScale, 0, 3, 1, OSZ_NAMES, 4, "", "Body text on the small displays."},
  {2, ST_BOOL, "radar sweep", &sweepOn, 0, 0, 0, NULL, 0, "", "The rotating sweep on the radar and the left ring."},
  {2, ST_INT, "slideshow after", &slideMin, 0, 240, 5, NULL, 0, " min", "Idle minutes before the apps rotate on their own. 0 = off."},
  {3, ST_ENUM, "range", (void *)&rangeIndex, 0, 5, 1, RANGE_NAMES, 6, "", "Radar radius. The right knob on the radar changes it too."},
  {3, ST_ENUM, "show", (void *)&filterMode, 0, 2, 1, FILTER_NAMES, 3, "", "Which traffic the radar draws."},
  {3, ST_ENUM, "mode", &radarFocus, 0, 1, 1, RFOC_NAMES, 2, "", "Right dial follows arrivals or departures. Right knob hold on the radar flips it."},
  {3, ST_ENUM, "data source", (void *)&dataSource, 0, 4, 1, SRC_NAMES, 5, "", "ADS-B feed. adsb.lol is the default; the others are fallbacks."},
  {3, ST_AIRPORT, "home airport", (void *)&homeIndex, 0, 0, 1, NULL, 0, "", "Centre of the radar and the arrivals board."},
  {3, ST_WXLOC, "weather location", (void *)&weatherIndex, 0, 0, 1, NULL, 0, "", "Weather for the house, the home airport or any airport in the list."},
  {3, ST_BOOL, "alert banners", &popupsOn, 0, 0, 0, NULL, 0, "", "Alerts take over the dials briefly. Off: chime and log only."},
  {3, ST_BOOL, "look-up alert", &lookupOn, 0, 0, 0, NULL, 0, "", "Chime and a ring pointer when an aircraft passes close to the house: where to look and how high."},
  {3, ST_INT, "look-up radius", &lookupKm, 1, 5, 1, NULL, 0, " km", "How close to the house an aircraft must come."},
  {4, ST_BOOL, "12-hour clock", &clock12, 0, 0, 0, NULL, 0, "", "AM / PM everywhere."},
  {4, ST_ENUM, "right dial", &clkRight, 0, 4, 1, CLKR_NAMES, 5, "", "What the right dial shows while the clock is up."},
  {4, ST_BOOL, "seconds ring", &secRing, 0, 0, 0, NULL, 0, "", "Seconds sweep on the right ring in every app; 12, 3, 6 and 9 marked. A running Pomodoro still takes the ring."},
  {4, ST_BOOL, "ISS passes", &issOn, 0, 0, 0, NULL, 0, "", "Next visible pass of the space station on the Clock dial; banner and ring pointer during the pass."},
  {4, ST_BOOL, "alarm", &alarmOn, 0, 0, 0, NULL, 0, "", "Alarm clock on or off."},
  {4, ST_TIME, "alarm time", &alarmMin, 0, 1435, 5, NULL, 0, "", "Rings on the chosen days. Right knob click snoozes 5 min, hold stops."},
  {4, ST_ENUM, "alarm days", &alDaysIdx, 0, 2, 1, ALDAYS_NAMES, 4, "", "Weekdays, daily or weekends. Other patterns: web page."},
  {5, ST_U16, "daily goal", &pcfg.goalMin, 30, 480, 15, NULL, 0, " min", "Focused minutes per day for the goal ring and streaks."},
  {5, ST_U8, "long break", &pcfg.longBreak, 5, 60, 5, NULL, 0, " min", "After each cycle of sessions."},
  {5, ST_U8, "sessions per cycle", &pcfg.cycle, 2, 8, 1, NULL, 0, "", "Focus sessions before the long break."},
  {5, ST_U8, "auto start", &pcfg.autoStart, 0, 1, 1, NULL, 0, "", "Next session starts by itself after the break."},
  {5, ST_U8ENUM, "quiet mode", &pcfg.quiet, 0, 3, 1, PQUIET_NAMES, 4, "", "Alerts held until the break while you focus."},
  {5, ST_BOOL, "Mac Focus sync", &focusSync, 0, 0, 0, NULL, 0, "", "Focus session turns the Mac's Focus mode on and off (Shortcuts 'DeskRadar Focus On / Off')."},
  {5, ST_INT, "stand-up nudge", &nudgeMin, 0, 120, 15, NULL, 0, " min", "Reminder to stand up every so many minutes. 0 = off."},
  {6, ST_ENUM, "boot app", &bootApp, 0, APP_COUNT - 1, 1, APP_NAMES, APP_COUNT, "", "The app shown at power on."},
  {6, ST_APP, "RADAR", &appMask, APP_RADAR, 0, 0, NULL, 0, "", "Show or hide this app in the left-knob list."},
  {6, ST_APP, "ARRIVALS", &appMask, APP_ARRIVALS, 0, 0, NULL, 0, "", "Airport board."},
  {6, ST_APP, "APPROACH", &appMask, APP_APPROACH, 0, 0, NULL, 0, "", "Glideslope side view."},
  {6, ST_APP, "CLOCK", &appMask, APP_CLOCK, 0, 0, NULL, 0, "", "Clock faces."},
  {6, ST_APP, "AGENDA", &appMask, APP_AGENDA, 0, 0, NULL, 0, "", "Calendar from the Mac."},
  {6, ST_APP, "WEATHER", &appMask, APP_WEATHER, 0, 0, NULL, 0, "", "Now, forecast, air quality."},
  {6, ST_APP, "POMODORO", &appMask, APP_POMODORO, 0, 0, NULL, 0, "", "Focus timer."},
  {6, ST_APP, "MARKETS", &appMask, APP_MARKET, 0, 0, NULL, 0, "", "Indices, gold, bitcoin."},
  {6, ST_APP, "NEWS", &appMask, APP_NEWS, 0, 0, NULL, 0, "", "Headlines."},
  {6, ST_APP, "MEDIA", &appMask, APP_MEDIA, 0, 0, NULL, 0, "", "Now playing and volume for the Mac."},
  {7, ST_INFO, "firmware", NULL, 0, 0, 0, NULL, 0, "", "Version running now."},
  {7, ST_INFO, "address", NULL, 1, 0, 0, NULL, 0, "", "Web page: this address or deskradar6.local"},
  {7, ST_INFO, "uptime", NULL, 2, 0, 0, NULL, 0, "", "Since the last restart."},
  {7, ST_INFO, "memory", NULL, 3, 0, 0, NULL, 0, "", "Free internal RAM / largest block."},
  {7, ST_ACTION, "save settings to SD", NULL, ACT_SAVE_SD, 0, 0, NULL, 0, "", "Writes /deskradar/config.json on the card."},
  {7, ST_ACTION, "reconnect WiFi", NULL, ACT_WIFI, 0, 0, NULL, 0, "", "Drops and rejoins the network."},
  {7, ST_ACTION, "clear crash record", NULL, ACT_CLEAR_CRASH, 0, 0, NULL, 0, "", "Erases the stored core dump."},
  {7, ST_ACTION, "restart", NULL, ACT_RESTART, 0, 0, NULL, 0, "", "Reboots the device. Settings are kept."},
  {7, ST_ACTION, "reset all settings", NULL, ACT_RESET, 0, 0, NULL, 0, "", "Back to defaults, then restart. Click twice."},
};
#define SET_N (int)(sizeof(SET_ITEMS) / sizeof(SET_ITEMS[0]))
int setCount(int cat) { int n = 0; for (int i = 0; i < SET_N; i++) if (SET_ITEMS[i].cat == cat) n++; return n; }
const SetItem *setItem(int cat, int row) { for (int i = 0; i < SET_N; i++) if (SET_ITEMS[i].cat == cat && row-- == 0) return &SET_ITEMS[i]; return NULL; }
void setSyncMirrors() { for (int i = 0; i < 4; i++) if (LEAD_VALS[i] == meetLead) meetLeadIdx = i; alDaysIdx = 3; for (int i = 0; i < 3; i++) if (ALDAYS_VALS[i] == alarmDays) alDaysIdx = i; }
int setGet(const SetItem &it) {
  switch (it.type) {
    case ST_BOOL: return *(bool *)it.ptr ? 1 : 0;
    case ST_U8: case ST_U8ENUM: return *(uint8_t *)it.ptr;
    case ST_U16: return *(uint16_t *)it.ptr;
    case ST_MASK: return (*(uint32_t *)it.ptr & (uint32_t)it.mn) ? 1 : 0;
    case ST_APP: return (*(uint32_t *)it.ptr & (1u << it.mn)) ? 1 : 0;
    case ST_ACTION: case ST_INFO: return 0;
    default: return *(int *)it.ptr;
  }
}
void setPut(const SetItem &it, int v) {
  switch (it.type) {
    case ST_BOOL: *(bool *)it.ptr = v != 0; break;
    case ST_U8: case ST_U8ENUM: *(uint8_t *)it.ptr = (uint8_t)v; break;
    case ST_U16: *(uint16_t *)it.ptr = (uint16_t)v; break;
    case ST_MASK: if (v) *(uint32_t *)it.ptr |= (uint32_t)it.mn; else *(uint32_t *)it.ptr &= ~(uint32_t)it.mn; break;
    case ST_APP: if (v) *(uint32_t *)it.ptr |= (1u << it.mn); else *(uint32_t *)it.ptr &= ~(1u << it.mn); buildMenu(); break;
    default: *(int *)it.ptr = v; break;
  }
  if (it.ptr == &meetLeadIdx) meetLead = LEAD_VALS[meetLeadIdx];
  if (it.ptr == &alDaysIdx && alDaysIdx < 3) alarmDays = ALDAYS_VALS[alDaysIdx];
  if (it.ptr == &theme) applyPalette();
  if (it.ptr == &tftBright || it.ptr == &oledBright || it.ptr == (void *)&nightMode || it.ptr == &nightStart || it.ptr == &nightEnd) { isNight = computeNight(); applyBrightness(); }
  if (it.ptr == (void *)&homeIndex) { applyHome(); followIcao[0] = 0; alertArmed = false; }
  if (it.ptr == &pcfg.goalMin) statsWanted = true;
  settingsDirty = true; staticDirty = true; clkS = -1;
}
void setFmt(const SetItem &it, char *b, size_t n) {
  int v = setGet(it);
  switch (it.type) {
    case ST_BOOL: case ST_MASK: strlcpy(b, v ? "on" : "off", n); break;
    case ST_APP: strlcpy(b, v ? "shown" : "hidden", n); break;
    case ST_U8: if (it.mx == 1) strlcpy(b, v ? "on" : "off", n); else snprintf(b, n, "%d%s", v, it.unit); break;
    case ST_INT: case ST_U16: if (v == 0 && it.mn == 0 && it.unit[0] == ' ') strlcpy(b, "off", n); else snprintf(b, n, "%d%s", v, it.unit); break;
    case ST_ENUM: case ST_U8ENUM: strlcpy(b, (v >= 0 && v < it.nNames) ? it.names[v] : "?", n); break;
    case ST_HOUR: fmtHMA(b, n, v, 0); break;
    case ST_TIME: fmtHMA(b, n, v / 60, v % 60); break;
    case ST_AIRPORT: snprintf(b, n, "%s", (v >= 0 && v < NUM_AIRPORTS) ? AIRPORTS[v].iata : "?"); break;
    case ST_WXLOC: if (v == -2) snprintf(b, n, "%.12s", WX_HOME_NAME); else if (v == -1) strlcpy(b, "home airport", n); else snprintf(b, n, "%s", (v >= 0 && v < NUM_AIRPORTS) ? AIRPORTS[v].iata : "?"); break;
    case ST_ACTION: strlcpy(b, "click", n); break;
    case ST_INFO: {
      unsigned long up = (millis() - bootMs) / 1000;
      if (it.mn == 0) snprintf(b, n, "v%s", FW_VERSION);
      else if (it.mn == 1) strlcpy(b, WiFi.localIP().toString().c_str(), n);
      else if (it.mn == 2) snprintf(b, n, "%lud %02lu:%02lu", up / 86400, (up / 3600) % 24, (up / 60) % 60);
      else snprintf(b, n, "%uK / %uK", ESP.getFreeHeap() / 1024, ESP.getMaxAllocHeap() / 1024);
      break; }
  }
}
unsigned long setArmMs = 0;
void setAction(int a) {
  switch (a) {
    case ACT_RESTART: raiseAlert("RESTARTING", "", COL_ARR, -1, 1500, NULL); delay(600); saveSettings(); ESP.restart(); break;
    case ACT_RESET:
      if (millis() - setArmMs > 6000) { setArmMs = millis(); raiseAlert("RESET ALL?", "click again to confirm", COL_SELECTED, -1, 6000, NULL); break; }
      prefs.begin("radar", false); prefs.clear(); prefs.end(); raiseAlert("SETTINGS RESET", "restarting", COL_SELECTED, -1, 1500, NULL); delay(600); ESP.restart(); break;
    case ACT_SAVE_SD: {
      bool ok = false;
      if (sdOk) { if (!SD.exists("/deskradar")) SD.mkdir("/deskradar"); File f = SD.open("/deskradar/config.json", FILE_WRITE); if (f) { String j = configJsonString(); ok = f.print(j) == (int)j.length(); f.close(); } }
      raiseAlert(ok ? "SAVED TO SD" : "SD WRITE FAILED", ok ? "/deskradar/config.json" : (sdOk ? "" : "no card"), ok ? COL_OK : COL_BAD, -1, 2500, NULL); break; }
    case ACT_WIFI: raiseAlert("WIFI", "reconnecting", COL_ARR, -1, 2000, NULL); wifiUp = false; WiFi.disconnect(); delay(100); if (haveCreds) WiFi.begin(WIFI_SSID, WIFI_PASSWORD); else WiFi.begin(); break;
    case ACT_CLEAR_CRASH: esp_core_dump_image_erase(); raiseAlert("CRASH RECORD", "cleared", COL_OK, -1, 1500, NULL); break;
  }
}
void settingsTurn(int step, int fast) {
  if (setCat < 0) { setSel = (setSel + step + SET_NCAT) % SET_NCAT; return; }
  int n = setCount(setCat); const SetItem *it = setItem(setCat, setSel);
  if (!setEdit || !it) { setSel = (setSel + step + n) % n; return; }
  int v = setGet(*it), mx = it->mx, mn = it->mn;
  switch (it->type) {
    case ST_BOOL: case ST_MASK: case ST_APP: setPut(*it, !v); break;
    case ST_AIRPORT: setPut(*it, ((v + step) % NUM_AIRPORTS + NUM_AIRPORTS) % NUM_AIRPORTS); break;
    case ST_WXLOC: { int nv = v + step; if (nv < -2) nv = NUM_AIRPORTS - 1; if (nv >= NUM_AIRPORTS) nv = -2; setPut(*it, nv); break; }
    case ST_HOUR: setPut(*it, (v + step + 24) % 24); break;
    case ST_TIME: setPut(*it, ((v + step * it->step * (fast > 1 ? 6 : 1)) % 1440 + 1440) % 1440); break;
    case ST_ENUM: case ST_U8ENUM: { int span = mx - mn + 1, nv = mn + ((v - mn + step) % span + span) % span;
      if (it->ptr == &bootApp) { int g = 0; while (appRetired(nv) && g++ < APP_COUNT) nv = mn + ((nv - mn + step) % span + span) % span; }   // 6.16.0: skip retired apps
      setPut(*it, nv); break; }
    case ST_U8: if (mx == 1) { setPut(*it, !v); break; }   // fall through for numeric u8
    default: { v += step * it->step * (fast > 1 ? 2 : 1); if (v < mn) v = mn; if (v > mx) v = mx; setPut(*it, v); break; }
  }
}
void settingsClick() {
  if (setCat < 0) { setCat = setSel; setSel = 0; setEdit = false; return; }
  const SetItem *it = setItem(setCat, setSel); if (!it) return;
  switch (it->type) {
    case ST_BOOL: case ST_MASK: case ST_APP: setPut(*it, !setGet(*it)); break;
    case ST_U8: if (it->mx == 1) { setPut(*it, !setGet(*it)); break; } setEdit = !setEdit; break;
    case ST_ACTION: setAction(it->mn); break;
    case ST_INFO: break;
    default: setEdit = !setEdit; break;
  }
}
void settingsHold() {
  if (setEdit) { setEdit = false; return; }
  if (setCat >= 0) { setSel = setCat; setCat = -1; return; }
}
void paintSettings() {
  radar->fillScreen(COL_BG); char b[40];
  if (setCat < 0) {
    radar->textC(34, "SETTINGS", FSR, COL_TEXT_DIM);
    for (int i = 0; i < SET_NCAT; i++) {
      int y = 60 + i * 21; bool sel = i == setSel;
      if (sel) radar->fillRoundRect(40, y - 15, 160, 21, 5, COL_MENU_BG);
      radar->textC(y, SET_CATS[i], sel ? FMB : FM, sel ? COL_TEXT : COL_TEXT_DIM);
    }
    return;
  }
  radar->textC(30, SET_CATS[setCat], FSR, COL_TEXT_DIM);
  int n = setCount(setCat); if (setSel >= n) setSel = n ? n - 1 : 0;
  int first = setSel > 2 ? setSel - 2 : 0; if (first > n - 6) first = n > 6 ? n - 6 : 0;
  for (int r = 0; r < 6 && first + r < n; r++) {
    const SetItem *it = setItem(setCat, first + r); int y = 56 + r * 26; bool sel = first + r == setSel;
    if (sel) radar->fillRoundRect(22, y - 16, 196, 24, 5, setEdit ? rgb(50, 36, 0) : COL_MENU_BG);
    radar->text(30, y, it->name, FS, sel ? COL_TEXT : rgb(150, 150, 150));
    setFmt(*it, b, sizeof(b));
    uint16_t vc = sel ? (setEdit ? C_AMBER : COL_TEXT) : COL_TEXT_DIM;
    if (it->type == ST_INFO) vc = COL_TEXT_DIM;
    if (sel && setEdit) { radar->textR(210, y, b, FS, vc); radar->text(212 - radar->textWidth(FS, b) - 12, y, "<", FSR, vc); radar->text(213, y, ">", FSR, vc); }
    else radar->textR(210, y, b, FS, vc);
  }
  if (n > 6) { snprintf(b, sizeof(b), "%d / %d", setSel + 1, n); radar->textC(226, b, FSR, rgb(70, 70, 70)); }
  else radar->textC(226, setEdit ? "turn: change  click: done" : "click: change  hold: back", FSR, rgb(70, 70, 70));
}
void drawSettingsFrame() { if (!staticDirty && millis() - lastHudMs < 1000) return; lastHudMs = millis(); staticDirty = false; present(paintSettings); }
void paintSettingsRight() {
  radar->fillScreen(COL_BG); char b[40];
  if (setCat < 0) {
    radar->textC(104, SET_CATS[setSel], FMB, COL_TEXT);
    static const char *CATH[SET_NCAT] = {"Mute, clicks, chime, quiet hours, which alerts sound", "Calendar reminders, titles, on-air light, mic mute", "Brightness of dials, OLEDs and rings, theme, night, sleep", "Range, filter, mode, data source, home airport", "12 h, alarm clock", "Goal, breaks, quiet mode, Mac Focus", "Which apps show, boot app", "Info, save to SD, WiFi, restart, reset"};
    char lines[3][40]; int nl = wrapText(CATH[setSel], FS, 180, lines, 3);
    for (int l = 0; l < nl; l++) radar->textC(132 + l * 18, lines[l], FS, COL_TEXT_DIM);
    return;
  }
  const SetItem *it = setItem(setCat, setSel); if (!it) return;
  if (setCat == 2 && (it->ptr == &theme || it->ptr == &tftBright)) { paintBrightRight(); return; }   // live preview
  radar->textC(44, it->name, FS, COL_TEXT_DIM);
  setFmt(*it, b, sizeof(b));
  const GFXfont *f = radar->textWidth(FMB, b) > 190 ? FS : FMB;
  radar->textC(88, b, f, setEdit ? C_AMBER : COL_TEXT);
  char lines[5][40]; int nl = wrapText(it->help, FSR, 184, lines, 5);
  for (int l = 0; l < nl; l++) radar->textC(124 + l * 16, lines[l], FSR, COL_TEXT_DIM);
  if (it->ptr == &ringBright) { radar->textC(214, "rings show it live", FSR, rgb(70, 70, 70)); }
}
void drawSettingsInfo() {
  char b[24];
  if (setCat < 0) { o6Head("SETTINGS"); o6Line(0, "turn: category", NULL); o6Line(1, "click: open", NULL); snprintf(b, sizeof(b), "%d of %d", setSel + 1, SET_NCAT); o6Band("SETTINGS", b); return; }
  { char h[40]; snprintf(h, sizeof(h), "SETTINGS > %s", SET_CATS[setCat]); o6Head(h); }
  if (setEdit) { o6Line(0, "turn: change value", NULL); o6Line(1, "click: done", NULL); o6Line(2, "hold: done", NULL); }
  else { o6Line(0, "turn: move", NULL); o6Line(1, "click: change / run", NULL); o6Line(2, "hold: back", NULL); }
  snprintf(b, sizeof(b), "%d of %d", setSel + 1, setCount(setCat)); o6Band(SET_CATS[setCat], b);
}
void drawCompanionFrame() {
  static unsigned long last = 0; static int lastApp = -1, lastSec = -1; static bool lastRwy = false;
  if (sleeping) return;
  unsigned long now = millis();
  struct tm t; int sec = getTime(t) ? t.tm_sec : -1;
  bool radarApp = currentApp == APP_RADAR && !menuOpen;
  bool rwyView = radarApp;                     // radar detail moves with dead reckoning
  unsigned long every = rwyView ? 200 : 1000;   // 6.5.3: radar detail at 5 fps (the DMA pipeline made it cheap)
  static bool lastMenu = false; static int lastSel = -1;
  if (menuOpen != lastMenu || (menuOpen && menuSel != lastSel) || brightMode()) { lastMenu = menuOpen; lastSel = menuSel; last = 0; }
  static bool navWas = false; bool navNow = navActive(), navChg = navNow != navWas;   // 6.14.0: legend on / off
  bool due = (int)currentApp != lastApp || rwyView != lastRwy || sec != lastSec || now - last >= every || (toastActive() && now - last > 250) || fadeK < 1.0f || navChg;
  if (!due || (now - last < 100 && fadeK >= 1.0f && !navChg)) return;
  if (!navChg && !radarApp && (int)currentApp == lastApp && sec == lastSec && last && !(toastActive() && now - last > 250) && now - last < 1000) return;
  bool oledDue = sec != lastSec || (int)currentApp != lastApp || toastActive() || navNow || navChg;   // the right OLED changes at most once a second (5 / s while the legend is up)
  last = now; lastApp = currentApp; lastSec = sec; lastRwy = rwyView; navWas = navNow;
  { bool h; const Plane *rp = menuOpen ? NULL : rightPlane(h); if (rp) { const AirlineRow *al = airlineFor(rp->callsign); if (al && al->iata[0]) logoLoad(al->iata);
      } }   // card read outside paint
  rssiTick();
  presentR(rightPaintFor());
  if (oledDue) drawInfoR();
}
// Right OLED: time in the blue area (big Poppins, small seconds), day and date in the yellow band at the
// bottom. The top rows stay empty in case the bezel edge hides them.
void drawInfoR() {
  if (sleeping) return;
  infoR.clearDisplay(); infoR.setTextColor(SSD1306_WHITE); infoR.setTextWrap(false); infoR.setTextSize(1);
  if (navActive()) { drawNavOled(1); oledPush(1); return; }   // 6.14.0: knob legend
  struct tm t;
  if (!getTime(t)) { infoR.setFont(); infoR.setCursor(16, 28); infoR.print("waiting for time"); oledPush(1); return; }
  char hm[8], ss[4]; int16_t x1, y1; uint16_t w, h, w2, h2;
  if (currentApp == APP_CLOCK && !menuOpen) {   // 6.18.0: the left dial is the clock: no time here. Week / day, next event, alarm in the band
    char d[40]; int wk = (t.tm_yday + 7 - ((t.tm_wday + 6) % 7)) / 7; snprintf(d, sizeof(d), "WEEK %d   DAY %d", wk, t.tm_yday + 1);
    infoR.setFont(&F_SM13); infoR.getTextBounds(d, 0, 14, &x1, &y1, &w, &h); infoR.setCursor((128 - (int)w) / 2 - x1, 14); infoR.print(d);
    int nx = calNext(), cn = calNow(); char ev[40] = "no meetings today";
    if (cn >= 0) snprintf(ev, sizeof(ev), "now: %.22s", calTitle(calEv[cn]));
    else if (nx >= 0) { time_t st = calEv[nx].t; struct tm et; localtime_r(&st, &et); char hm[12]; fmtHMA(hm, sizeof(hm), et.tm_hour, et.tm_min); snprintf(ev, sizeof(ev), "%s %.16s", hm, calTitle(calEv[nx])); }
    else if (!macLinked()) strlcpy(ev, "calendar: Mac offline", sizeof(ev));
    infoR.setFont(&F_SM13R); infoR.setCursor(0, 42); infoR.print(ev);
    char a[24]; if (alarmOn) { char hm[12]; fmtHMA(hm, sizeof(hm), alarmMin / 60, alarmMin % 60); snprintf(a, sizeof(a), "ALARM %s", hm); } else strlcpy(a, "ALARM OFF", sizeof(a));
    infoR.setFont(&F_SM13); infoR.setCursor(0, 61); infoR.print(a);
    infoR.setFont(); oledPush(1); return;
  }
  int hr = clock12 ? (t.tm_hour % 12 ? t.tm_hour % 12 : 12) : t.tm_hour;
  snprintf(hm, sizeof(hm), clock12 ? "%d:%02d" : "%02d:%02d", hr, t.tm_min); snprintf(ss, sizeof(ss), "%02d", t.tm_sec);
  const char *ampm = t.tm_hour < 12 ? "AM" : "PM";
  // 6.20.0: the time fills the blue area (46 px Poppins digits); AM / PM and the seconds sit in the yellow band with the date
  infoR.setFont(&F_BIG46); infoR.getTextBounds(hm, 0, 44, &x1, &y1, &w, &h);
  infoR.setCursor((128 - (int)w) / 2 - x1, 44); infoR.print(hm);
  (void)w2; (void)h2;
  static const char *DAY[7] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};
  static const char *MON[12] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
  // yellow band: on air, then a meeting in progress or within 30 min, else the date
  { char band[64] = ""; int cn = calNow(), nx = calNext(); uint32_t now = time(NULL);
    if (skyActive()) strlcpy(band, sky.label, sizeof(band));   // 6.20.0: no ON AIR text here (the ring pip says it)
    else if (cn >= 0) snprintf(band, sizeof(band), "%um left  %s", (unsigned)((calEv[cn].e - now + 59) / 60), calTitle(calEv[cn]));
    else if (nx >= 0 && meetOn && calEv[nx].t - now <= 1800) snprintf(band, sizeof(band), "in %um  %s", (unsigned)((calEv[nx].t - now + 59) / 60), calTitle(calEv[nx]));
    if (band[0]) {
      infoR.setFont(&F_SM13); int16_t bx, by; uint16_t bw, bh; int n = strlen(band);
      while (n > 3) { infoR.getTextBounds(band, 0, 61, &bx, &by, &bw, &bh); if (bw <= 104) break; band[--n] = 0; }   // 6.20.1: room for the seconds on the right
      infoR.setCursor(0, 61); infoR.print(band);
      infoR.getTextBounds(ss, 0, 61, &bx, &by, &bw, &bh); infoR.setCursor(127 - (int)bw - bx, 61); infoR.print(ss);
      infoR.setFont(); oledPush(1); return;
    } }
  // band: AM / PM left, date centred, seconds right (6.20.0)
  char d[32]; snprintf(d, sizeof(d), "%.3s %d %s", DAY[t.tm_wday], t.tm_mday, MON[t.tm_mon]);
  infoR.setFont(&F_SM13);
  if (clock12) { infoR.setCursor(0, 61); infoR.print(ampm); }
  infoR.getTextBounds(ss, 0, 61, &x1, &y1, &w, &h); infoR.setCursor(127 - (int)w - x1, 61); infoR.print(ss);
  infoR.getTextBounds(d, 0, 61, &x1, &y1, &w, &h); infoR.setCursor((128 - (int)w) / 2 - x1, 61); infoR.print(d);
  infoR.setFont();
  oledPush(1);
}

// =====================================================================
// RING ENGINE (6.8.0). One grammar for both 24-LED rings (logical LED 0 = 12 o'clock, clockwise):
//   fill from 12 = progress, single dot = a position, dot counting down from 12 = time left, slow breathing
//   = a state, quick flash = an event, red spin on both = alarm / emergency, amber pip at 12 = held alerts.
// Layers, drawn in this order into rl[][][]: ambient base -> app layer -> content overlays (Pomodoro
// everywhere, meeting countdown) -> night handling -> wake-up light -> holiday sparkle -> on air -> alarm /
// emergency -> feedback effects (tick, pulse, hold fill, app wipe, boot) -> output with brightness + colour
// correction. 25 Hz.
// =====================================================================
int ringPhys(int side, int logical) { int p = (RING_OFFSET[side] + RING_DIR[side] * logical) % 24; return p < 0 ? p + 24 : p; }
uint32_t ringCol(Adafruit_NeoPixel &r, uint8_t R, uint8_t G, uint8_t B, float k) {
  return r.Color((uint8_t)(R * 0.7f * k), (uint8_t)(G * k), (uint8_t)(B * 0.5f * k));
}
void rgb565to8(uint16_t c, uint8_t &r, uint8_t &g, uint8_t &b) { r = ((c >> 11) & 0x1F) << 3; g = ((c >> 5) & 0x3F) << 2; b = (c & 0x1F) << 3; }
float rl[2][24][3];
static inline int rj(int j) { return ((j % 24) + 24) % 24; }
void rset(int s, int j, float r, float g, float b) { j = rj(j); rl[s][j][0] = r; rl[s][j][1] = g; rl[s][j][2] = b; }
void radd(int s, int j, float r, float g, float b) { j = rj(j); rl[s][j][0] = fminf(255, rl[s][j][0] + r); rl[s][j][1] = fminf(255, rl[s][j][1] + g); rl[s][j][2] = fminf(255, rl[s][j][2] + b); }
void rfill(int s, int n, float r, float g, float b) { for (int j = 0; j < 24 && j < n; j++) rset(s, j, r, g, b); }
void rall(int s, float r, float g, float b) { for (int j = 0; j < 24; j++) rset(s, j, r, g, b); }
void rclear(int s) { for (int j = 0; j < 24; j++) rset(s, j, 0, 0, 0); }
void rscale(int s, float k) { for (int j = 0; j < 24; j++) { rl[s][j][0] *= k; rl[s][j][1] *= k; rl[s][j][2] *= k; } }
bool rquiet(int s) { for (int j = 0; j < 24; j++) if (rl[s][j][0] + rl[s][j][1] + rl[s][j][2] > 1) return false; return true; }
void rset565(int s, int j, uint16_t c, float k) { uint8_t r, g, b; rgb565to8(c, r, g, b); rset(s, j, r * k, g * k, b * k); }
// ---- feedback effects ----
enum { FX_NONE, FX_TICK, FX_PULSE, FX_WIPE, FX_FLASH, FX_BOOT };
struct Fx { uint8_t kind, side; unsigned long t0; int pos; float r, g, b; };
#define FX_N 8
Fx fxq[FX_N]; int fxPos[2] = {0, 0}; float fxHold = 0;
void fxPush(uint8_t kind, uint8_t side, float r, float g, float b, int pos) {
  int slot = -1; unsigned long oldest = 0xFFFFFFFF;
  for (int i = 0; i < FX_N; i++) { if (fxq[i].kind == FX_NONE) { slot = i; break; } if (fxq[i].t0 < oldest) { oldest = fxq[i].t0; slot = i; } }
  fxq[slot] = { kind, side, millis(), pos, r, g, b };
}
void fxTick(int side, int dir) { fxPos[side] = rj(fxPos[side] + dir); fxPush(FX_TICK, side, 255, 255, 255, fxPos[side]); }
void fxPulse(int side) { fxPush(FX_PULSE, side, 255, 255, 255, 0); }
void fxWipe(uint16_t col) { uint8_t r, g, b; rgb565to8(col, r, g, b); fxPush(FX_WIPE, 0, r, g, b, 0); }
void fxFlash(int side, float r, float g, float b) { fxPush(FX_FLASH, side, r, g, b, 0); }
void fxBoot() { fxPush(FX_BOOT, 0, 80, 255, 120, 0); }
void fxRender(unsigned long now) {
  for (int i = 0; i < FX_N; i++) {
    Fx &f = fxq[i]; if (f.kind == FX_NONE) continue;
    unsigned long age = now - f.t0; int s = f.side;
    switch (f.kind) {
      case FX_TICK: { const unsigned long dur = 300; if (age > dur) { f.kind = FX_NONE; break; } float a = 1.0f - (float)age / dur;
        radd(s, f.pos, 255 * a, 255 * a, 255 * a); radd(s, f.pos - 1, 60 * a, 60 * a, 60 * a); radd(s, f.pos + 1, 60 * a, 60 * a, 60 * a); break; }
      case FX_PULSE: { const unsigned long dur = 160; if (age > dur) { f.kind = FX_NONE; break; } float a = 0.5f * (1.0f - (float)age / dur);
        for (int j = 0; j < 24; j++) radd(s, j, 255 * a, 255 * a, 255 * a); break; }
      case FX_WIPE: { const unsigned long dur = 360; if (age > dur) { f.kind = FX_NONE; break; } float t = (float)age / dur; int n = (int)(t * 24 + 1);
        for (int j = 0; j < n && j < 24; j++) { float a = 0.8f * (1.0f - t) * (0.4f + 0.6f * (float)j / n); radd(0, j, f.r * a, f.g * a, f.b * a); } break; }
      case FX_FLASH: { const unsigned long dur = 400; if (age > dur) { f.kind = FX_NONE; break; } float a = 0.6f * (1.0f - (float)age / dur);
        for (int j = 0; j < 24; j++) radd(s, j, f.r * a, f.g * a, f.b * a); break; }
      case FX_BOOT: { const unsigned long dur = 1600; if (age > dur) { f.kind = FX_NONE; break; } float head = (float)age / dur * 72.0f; float fade = age > 1200 ? 1.0f - (age - 1200) / 400.0f : 1.0f;
        for (int t = 0; t < 10; t++) { float a = fade * (1.0f - t / 10.0f); for (int sd = 0; sd < 2; sd++) radd(sd, (int)head - t, f.r * a, f.g * a, f.b * a); } break; }
    }
  }
  if (fxHold > 0) { int n = (int)(fxHold * 24 + 0.5f); for (int j = 0; j < n; j++) radd(0, j, 140, 140, 140); }
}
// ---- app layer ----
void ringRadarLayer(unsigned long now) {
  // left: sweep head + fading tail locked to sweepAngle, aircraft dots at their bearing coloured by class
  // (amber arrivals, yellow departures, cyan overflights), flaring as the sweep passes; emergency squawk red
  for (int j = 0; j < 24 && sweepOn; j++) {   // 6.19.0: the sweep is the main thing: full green head, longer tail
    float d = fmodf(sweepAngle - j * 15.0f + 720.0f, 360.0f);
    float I = d > 345 ? 1 - (360 - d) / 15 : d < 100 ? (1 - d / 100) * (1 - d / 100) : 0;
    rset(0, j, 0, 255 * I, 70 * I);
  }
  float nearHouse = 1e9f; int nearJ = -1;   // 6.19.0: the aircraft closest to the house (overhead pre-cue below)
  float rangeKm = displayRangeKm(); double clat, clon; drCenter(clat, clon); int subj = radarSubject();
  for (int i = 0; i < planeCount; i++) {
    const Plane &p = planes[i]; if (!passesFilter(p)) continue;
    int x, y; float dist, brg; planeScreenPos(p, rangeKm, clat, clon, x, y, dist, brg);
    if (dist > rangeKm) continue;
    int j = rj((int)lroundf(brg / 15.0f));
    float since = fmodf(sweepAngle - brg + 720.0f, 360.0f);
    float I = sweepOn ? 0.12f + 0.6f * fmaxf(0, 1 - since / 60) : 0.35f; if (i == subj) I = fmaxf(I, 0.5f);   // 6.19.0: dots stay quiet, flare only as the sweep passes
    if (lookupOn && p.altitude_m * 3.28084f < 12000) { float dh = distanceKm(wxHomeLat, wxHomeLon, p.lat, p.lon); if (dh < nearHouse) { nearHouse = dh; nearJ = j; } }
    float r = 255, g = 140, b = 0;                                         // arrival amber
    if (p.cls == CLS_DEPARTURE) { r = 255; g = 230; b = 40; } else if (p.cls == CLS_ENROUTE) { r = 60; g = 200; b = 255; }
    rset(0, j, fmaxf(rl[0][j][0] * 0.25f, r * I), fmaxf(rl[0][j][1] * 0.25f, g * I), fmaxf(rl[0][j][2] * 0.25f, b * I));
  }
  // 6.19.0: overhead pre-cue: the aircraft nearest the house pulses white at its bearing once inside 3 x the look-up radius,
  // faster and brighter as it closes (the LOOK UP alert and the sky pointer take over inside the radius)
  if (nearJ >= 0 && nearHouse < lookupKm * 3 && !skyActive()) {
    float close = 1 - nearHouse / (lookupKm * 3); float w = (0.5f + 0.5f * sinf(now / (120.0f + 280.0f * (1 - close)))) * (0.3f + 0.7f * close);
    radd(0, nearJ, 255 * w, 255 * w, 255 * w);
  }
  // right: the subject. On final within 15 km: alignment meter (centre LED = runway centreline, dot drifts
  // left / right with the cross-track, green when within 300 m). Otherwise (6.19.0): a departure's fill grows
  // clockwise, one LED per km out; an arrival's fill counts down anticlockwise, one LED per minute to touchdown.
  if (subj >= 0) {
    const Plane &p = planes[subj]; int rw; float al, cr;
    if (p.cls == CLS_ARRIVAL && finalPlane(rw, al, cr) == subj && al < 15) {
      rset(1, 0, 0, 90, 30); rset(1, 12, 30, 30, 30);
      int off = (int)lroundf(cr * 4); if (off > 6) off = 6; if (off < -6) off = -6;
      bool ok = fabsf(cr) < 0.3f; rset(1, off, ok ? 0 : 255, ok ? 255 : 140, ok ? 60 : 0);
      int n = (int)al; if (n > 11) n = 11; for (int j = 0; j < n; j++) radd(1, 23 - j, 40, 20, 0);   // distance ticks on the left half
    } else if (p.cls == CLS_ARRIVAL) { int n = etaMin(p, p.distance_km); if (n > 24) n = 24; if (n < 1) n = 1; for (int j = 0; j < n; j++) rset(1, -j, 255, 140, 0); }
    else { int n = (int)p.distance_km; if (n > 24) n = 24; if (n < 1) n = 1; rfill(1, n, p.cls == CLS_DEPARTURE ? 255 : 60, p.cls == CLS_DEPARTURE ? 230 : 200, p.cls == CLS_DEPARTURE ? 60 : 255); }
  }
}
void ringAppLayer(unsigned long now, struct tm &t, bool haveT) {
  rclear(0); rclear(1);
  int cur = 0; for (int i = 0; i < menuCount; i++) if (menuItems[i] == (menuOpen ? menuItems[menuSel] : currentApp)) cur = i;
  if (menuOpen) { if (menuCount) rset(0, (cur * 24 + menuCount / 2) / menuCount, 255, 255, 255); return; }
  switch (currentApp) {
    case APP_RADAR: ringRadarLayer(now); break;
    case APP_ARRIVALS: case APP_APPROACH: {
      bool home; const Plane *p = rightPlane(home); if (!p) break;
      if (currentApp == APP_ARRIVALS) { int n = p->cls == CLS_ARRIVAL ? etaMin(*p, p->distance_km) : 0; if (n > 24) n = 24; rfill(0, n, 255, 140, 0); }
      else { float ft = p->altitude_m * 3.28084f, dev = ft - (homeElevM() * 3.28084f + p->distance_km * 172); int off = (int)lroundf(dev / 100); if (off > 6) off = 6; if (off < -6) off = -6;
             rset(0, 0, 0, 90, 30); bool ok = fabsf(dev) < 150; rset(0, off, ok ? 0 : 255, ok ? 255 : 140, ok ? 60 : 0); }
      int n = (int)p->distance_km; if (n > 24) n = 24; if (n < 1) n = 1; rfill(1, n, 255, 140, 0); break; }
    case APP_CLOCK: {
      if (!haveT) break;
      // 6.18.0: the rings follow the face
      if (clockFace == 1 || clockFace == 11 || clockFace == 13 || clockFace == 16) {   // hands on the rings: hour (white) and minute (amber) on the left, sweeping seconds on the right
        for (int q = 0; q < 24; q += 2) radd(0, q, 18, 18, 22);
        float hf = ((t.tm_hour % 12) + t.tm_min / 60.0f) * 2.0f, mf = t.tm_min * 24.0f / 60.0f;
        int hj = (int)hf; rset(0, hj, 255, 255, 255); radd(0, hj + 1, 255 * (hf - hj), 255 * (hf - hj), 255 * (hf - hj));
        int mj = (int)mf; radd(0, mj, 255, 150, 0); radd(0, mj + 1, 255 * (mf - mj), 150 * (mf - mj), 0);
        float sf = (t.tm_sec + ringSecFrac(now, t.tm_sec)) * 24.0f / 60.0f; int sj = (int)sf; float fr = sf - sj;
        rset(1, sj, 255 * (1 - fr) + 60 * fr, 60 * (1 - fr), 60 * (1 - fr)); radd(1, sj + 1, 255 * fr, 60 * fr, 60 * fr); for (int tl = 1; tl <= 4; tl++) radd(1, sj - tl, 60 / tl, 12 / tl, 12 / tl);
        break;
      }
      if (clockFace == 7 || clockFace == 5) {   // seconds fill on the right, minutes fill on the left
        rfill(0, t.tm_min * 24 / 60 + 1, 0, 90, 160); rfill(1, t.tm_sec * 24 / 60 + 1, 80, 200, 255); break;
      }
      if (clockFace == 12 || clockFace == 6) {   // warm glow, seconds as a dot
        rall(0, 40, 14, 2); rall(1, 40, 14, 2); rset(1, t.tm_sec * 24 / 60, 255, 90, 30); break;
      }
      if (clockFace == 8) {   // night sky: faint blue, the moon's illumination as a glow on the left
        float ph = moonPhase(); float il = 1 - fabsf(ph - 0.5f) * 2; rall(0, 20 * il + 2, 20 * il + 4, 60 * il + 12); rall(1, 2, 4, 12); rset(1, t.tm_sec * 24 / 60, 120, 140, 255); break;
      }
      // other faces: left 24 h daylight arc, noon at 12 o'clock; sun dot by day, moon dot by night. right: seconds.
      if (wx.sunriseMin >= 0) for (int h = 0; h < 24; h++) { int m = h * 60 + 30; bool day = m >= wx.sunriseMin && m <= wx.sunsetMin; int j = rj(h - 12); if (day) rset(0, j, 70, 55, 25); else rset(0, j, 2, 4, 12); }
      { int nowM = t.tm_hour * 60 + t.tm_min; bool day = wx.sunriseMin >= 0 && nowM >= wx.sunriseMin && nowM <= wx.sunsetMin; rset(0, rj(t.tm_hour - 12), day ? 255 : 90, day ? 200 : 120, day ? 60 : 255); }
      for (int q = 0; q < 24; q += 6) radd(1, q, 40, 40, 40); rset(1, t.tm_sec * 24 / 60, 255, 255, 255); break; }
    case APP_AGENDA: {
      uint32_t nowE = time(NULL); int cn = calNow();
      if (cn >= 0) { const CalEvent &c = calEv[cn]; float f = (float)((long)c.e - (long)nowE) / (float)(c.e - c.t); rfill(0, (int)(f * 24 + 0.5f), 255, 110, 0); }   // draining: time left in the meeting
      else { float a = clockAngle(nowE);
        for (int j = 0; j < 24; j++) for (int i = 0; i < calN; i++) { const CalEvent &c = calEv[i]; if (c.allday) continue; float d = fmodf(j * 15.0f - a + 360.0f, 360.0f); uint32_t slot = nowE + (uint32_t)(d * 120.0f);
          if (c.t < slot + 1800 && c.e > slot) { rset(0, j, 255, 150, 0); break; } }
        rset(0, (int)(a / 15.0f + 0.5f), 255, 255, 255); }
      int idx[MAX_CAL]; int n = agendaList(idx, MAX_CAL);
      if (n) { const CalEvent &c = calEv[idx[listSel < n ? listSel : 0]]; long to = (long)c.t - (long)nowE; if (to > 0 && to <= 3600) rfill(1, (int)((to * 24 + 3599) / 3600), 255, 150, 0); else if (to <= 0) rall(1, 150, 25, 25); }
      break; }
    case APP_WEATHER: {
      if (!wx.valid) break;
      bool rain = false; for (int i = 0; i < 8; i++) if (wx.rain15[i] >= 0.2f) rain = true;
      if (rain) for (int i = 0; i < 8; i++) { float mm = wx.rain15[i] > 0 ? wx.rain15[i] : 0; float I = fminf(1.0f, mm / 2.0f); for (int k = 0; k < 3; k++) rset(0, i * 3 + k, 20 * I, 90 * I, 255 * I); }   // rain radar: next 2 h, 15 min per 3 LEDs
      else { uint8_t r, g, b; rgb565to8(aqiColour(wx.aqi), r, g, b); rall(0, r * 0.5f, g * 0.5f, b * 0.5f); }
      for (int j = 0; j < 24; j++) { int pr = wx.rainProb[j]; if (pr >= 10) rset(1, j, 30 * pr / 100.0f, 90 * pr / 100.0f, 255 * pr / 100.0f); }
      break; }
    case APP_POMODORO: {
      if (pomoState == POMO_BREAK && breathing) {   // box breathing guide 4-4-4-4: fill grows, holds, shrinks, holds
        unsigned long ph = (now - breathStartMs) % 16000; float f = ph < 4000 ? ph / 4000.0f : ph < 8000 ? 1.0f : ph < 12000 ? 1.0f - (ph - 8000) / 4000.0f : 0.0f;
        rfill(0, (int)(f * 24 + 0.5f), 0, 180, 160); }
      else if (pomoState != POMO_SELECT && pomoTotalMs) { float f = 1.0f - (float)pomoRemainMs() / (float)pomoTotalMs; bool focus = pomoState == POMO_FOCUS || (pomoState == POMO_PAUSED && pomoWasFocus);
        rfill(0, (int)(f * 24 + 0.5f), focus ? 255 : 40, focus ? 140 : 220, focus ? 0 : 80); if (pomoState == POMO_PAUSED) rscale(0, 0.4f); }
      unsigned g = pcfg.goalMin ? todayFocus() * 24 / pcfg.goalMin : 0; rfill(1, g > 24 ? 24 : (int)g, 40, 220, 80); rscale(1, 0.7f);
      break; }
    case APP_MARKET: {
      if (marketSel < NUM_SYMBOLS && quotes[marketSel].valid && quotes[marketSel].prevClose > 0) { float c = (quotes[marketSel].price - quotes[marketSel].prevClose) / quotes[marketSel].prevClose * 100; int n = (int)(fabsf(c) / 3.0f * 24 + 1); rfill(0, n, c >= 0 ? 0 : 255, c >= 0 ? 220 : 30, c >= 0 ? 80 : 30);
        static unsigned long lastUpd = 0; static int lastSym = -1; if (marketSel != lastSym) { lastSym = marketSel; lastUpd = quotes[marketSel].updatedMs; }
        else if (quotes[marketSel].updatedMs != lastUpd) { lastUpd = quotes[marketSel].updatedMs; fxFlash(0, c >= 0 ? 0 : 255, c >= 0 ? 255 : 40, c >= 0 ? 80 : 40); } }
      if (haveT && marketOpenNow(t)) { int el = t.tm_hour * 60 + t.tm_min - (9 * 60 + 15); rfill(1, el * 24 / 375, 90, 90, 90); }
      break; }
    case APP_NEWS: { static unsigned long lastNews = 0; if (newsMs != lastNews) { if (lastNews) fxFlash(1, 255, 255, 255); lastNews = newsMs; } break; }
    case APP_F1: { if (f1.valid && f1.t) { long dt = (long)f1.t - (long)time(NULL); int days = dt > 0 ? (int)(dt / 86400) + 1 : 0; if (days > 24) days = 24; rfill(0, days, 220, 30, 30); if (dt > 0 && dt < 86400) rall(0, 220, 30, 30); } break; }
    case APP_STATS: { int mx = 1; for (int h = 0; h < 24; h++) { int v = hourArr[h] + hourDep[h] + hourEnr[h]; if (v > mx) mx = v; }
      for (int h = 0; h < 24; h++) { float f = (float)(hourArr[h] + hourDep[h] + hourEnr[h]) / mx; rset(0, rj(h - 12), 60 * f, 200 * f, 255 * f); } if (haveT) radd(0, rj(t.tm_hour - 12), 60, 60, 60); break; }
    case APP_MEDIA: {
      if (macLinked() && mac.vol >= 0) rfill(0, mac.vol * 24 / 100, 255, 255, 255);
      if (macLinked() && mac.playing) { float w = 0.4f + 0.6f * (0.5f + 0.5f * sinf(now / 500.0f)); rall(1, 40 * w, 200 * w, 120 * w); }
      break; }
    case APP_SETTINGS: {
      if (setCat == 2) { const SetItem *it = setItem(setCat, setSel); if (it && (it->ptr == &tftBright || it->ptr == &oledBright || it->ptr == &ringBright)) { int v = setGet(*it), n = v * 24 / 100; rfill(0, n, 255, 255, 255); rfill(1, n, 255, 255, 255); } }
      break; }
    default: if (menuCount) rset(0, (cur * 24 + menuCount / 2) / menuCount, 255, 255, 255); break;
  }
}
// ---- the tick ----
float ringSecFrac(unsigned long now, int sec) {   // 6.20.0: fraction of the current second, locked to the RTC second change
  static int lastSec = -1; static unsigned long secStart = 0;
  if (sec != lastSec) { lastSec = sec; secStart = now; }
  float f = (now - secStart) / 1000.0f; return f > 0.999f ? 0.999f : f;
}
void ringsTick() {
  static unsigned long last = 0; unsigned long now = millis();
  if (now - last < 40) return; last = now;
  struct tm t; bool haveT = getTime(t);
  float k = ringBright / 100.0f * tftBright / 100.0f;
  ringAppLayer(now, t, haveT);
  // 6.20.0: seconds ring (SETTINGS > CLOCK): the right ring is a seconds sweep in every app, quarters marked in another colour
  if (secRing && haveT && !menuOpen && !sleeping) {
    rclear(1); for (int q = 0; q < 24; q += 6) rset(1, q, 60, 60, 90);
    rset(1, t.tm_sec * 24 / 60, 255, 70, 0);   // 6.20.2: one LED (24 LEDs for 60 s: it steps every 2.5 s), no blend, no trail
  }
  // Pomodoro everywhere: a running session shows on the right ring in every other app (amber focus, green break)
  if (currentApp != APP_POMODORO && pomoState != POMO_SELECT && pomoTotalMs) {
    float f = 1.0f - (float)pomoRemainMs() / (float)pomoTotalMs; bool focus = pomoState == POMO_FOCUS || (pomoState == POMO_PAUSED && pomoWasFocus);
    rclear(1); rfill(1, (int)(f * 24 + 0.5f), focus ? 255 : 40, focus ? 140 : 220, focus ? 0 : 80); if (pomoState == POMO_PAUSED) rscale(1, 0.4f);
  }
  // meeting within 10 min: right ring counts it down, blinking in the last minute
  { int nx = calNext(); if (meetOn && nx >= 0) { long to = (long)calEv[nx].t - (long)time(NULL); if (to > 0 && to <= 600) { rclear(1); if (!(to < 60 && (now / 400) % 2)) rfill(1, (int)((to * 24 + 599) / 600), 255, 150, 0); } } }
  // sky pointer (look up / ISS): left ring dot at the bearing, flashing; right ring = elevation (0 to 90 deg)
  if (skyActive()) {
    rclear(0); rclear(1); int j = rj((int)lroundf(sky.az / 15.0f)); float w = 0.5f + 0.5f * sinf(now / 150.0f);
    rset(0, j, 255 * w, sky.iss ? 255 * w : 200 * w, sky.iss ? 255 * w : 40 * w); radd(0, j - 1, 60, 50, 10); radd(0, j + 1, 60, 50, 10);
    int n = (int)(fmaxf(0, sky.el) / 90.0f * 24 + 0.5f); rfill(1, n, 60, 200, 255);
  }
  // held alerts during focus: an amber pip at 12 on the left ring
  if (pomoFocusActive() && (heldPlanes + heldMkt + heldOther) > 0 && currentApp != APP_RADAR) radd(0, 0, 120, 70, 0);
  // ambient base when a ring is quiet: golden hour warm glow, rain ripple
  if (haveT && wx.valid) {
    int nowM = t.tm_hour * 60 + t.tm_min;
    bool golden = wx.sunsetMin > 0 && nowM >= wx.sunsetMin - 45 && nowM <= wx.sunsetMin + 20;
    bool raining = wx.rain15[0] >= 0.2f;
    for (int s2 = 0; s2 < 2; s2++) if (rquiet(s2)) {
      if (raining) for (int j = 0; j < 24; j++) { float w = 0.5f + 0.5f * sinf(now / 700.0f + j * 0.8f); rset(s2, j, 5 * w, 25 * w, 80 * w); }
      else if (golden) rall(s2, 60, 28, 6);
    }
  }
  // night dimming hours: off, nightlight or normal (SETTINGS > DISPLAY)
  if (isNight) { if (nightRings == 0) { rclear(0); rclear(1); } else if (nightRings == 1) { rall(0, 18, 8, 2); rall(1, 18, 8, 2); } else { rscale(0, 0.6f); rscale(1, 0.6f); } }
  if (sleeping) { rclear(0); rclear(1); }
  // wake-up light: 15 min before the alarm on an alarm day, red ramping to warm white
  if (alarmOn && haveT && (alarmDays & (1 << t.tm_wday))) { int to = alarmMin - (t.tm_hour * 60 + t.tm_min); if (to > 0 && to <= 15) { float p = 1.0f - to / 15.0f; for (int s2 = 0; s2 < 2; s2++) rall(s2, 255 * (0.3f + 0.7f * p), 20 + 160 * p, 120 * p); } }
  // holiday: sparkle
  if (haveT) { int m, d; if (holidayForYday(t.tm_year + 1900, t.tm_yday, m, d)) for (int s2 = 0; s2 < 2; s2++) for (int j = 0; j < 24; j++) if (random(100) < 3) radd(s2, j, 220, 170, 40); }
  // on air (6.11.1, owner: not two full red rings): a slow-breathing red pip at 12 on the right ring, amber when muted
  if (!sleeping && onAir()) { float w = 0.45f + 0.55f * (0.5f + 0.5f * sinf(now / 900.0f)); rset(1, 0, 255 * w, mac.muted ? 120 * w : 0, 0); rset(1, 1, 60 * w, mac.muted ? 30 * w : 0, 0); rset(1, 23, 60 * w, mac.muted ? 30 * w : 0, 0); }
  // network trouble (6.16.0): amber pip blinking at 6 o'clock on the left ring
  if (!sleeping && (!wifiUp || wanDown) && (now / 700) % 2) { rset(0, 12, 255, 70, 0); rset(0, 11, 60, 16, 0); rset(0, 13, 60, 16, 0); }
  // alarm ringing or an emergency squawk in range: red spin on both
  { bool emerg = false; for (int i = 0; i < planeCount && !emerg; i++) if (!strcmp(planes[i].squawk, "7700") || !strcmp(planes[i].squawk, "7600") || !strcmp(planes[i].squawk, "7500")) emerg = true;
    if (alarmRinging || emerg) { int head = (now / 40) % 24; for (int s2 = 0; s2 < 2; s2++) { rclear(s2); for (int tl = 0; tl < 8; tl++) rset(s2, head - tl, 255 * (1.0f - tl / 8.0f), 0, 0); } } }
  if (brightMode()) { for (int s2 = 0; s2 < 2; s2++) { rclear(s2); if (brightWhat == 1) rall(s2, 200, 200, 200); else rfill(s2, (tftBright * 24 + 50) / 100, 200, 200, 200); } }   // 6.17.0: rings show the level (ring gesture: full white at the new level)
  fxRender(now);
  for (int j = 0; j < 24; j++) {
    ringL.setPixelColor(ringPhys(0, j), ringCol(ringL, (uint8_t)fminf(255, rl[0][j][0]), (uint8_t)fminf(255, rl[0][j][1]), (uint8_t)fminf(255, rl[0][j][2]), k));
    ringR.setPixelColor(ringPhys(1, j), ringCol(ringR, (uint8_t)fminf(255, rl[1][j][0]), (uint8_t)fminf(255, rl[1][j][1]), (uint8_t)fminf(255, rl[1][j][2]), k));
  }
  ringL.show(); ringR.show();
}

// =====================================================================
// NAVIGATION HINTS (6.14.0). One table for what each knob does right now; the left-dial footer (uiHint) and
// the OLED legend (drawNavOled) both read it. Left knob: 0 turn, 1 click, 2 double click, 3 hold 2 s.
// Right knob: 0 turn, 1 click, 2 hold. Values stay short (11 characters) so they fit the OLED column.
// =====================================================================
bool navActive() { return menuOpen || (navMs && millis() - navMs < 3000); }
const char *navHint(int side, int what) {
  if (side == 0) {
    if (menuOpen) return what == 0 ? "move" : what == 1 ? "open" : "";
    if (alarmRinging) return what == 0 || what == 1 ? "snooze" : "";
    if (onAir() && micClick && what == 1) return mac.muted ? "mic live" : "mic mute";
    switch (what) { case 0: return "next app"; case 1: return "home view"; case 2: return "pomodoro"; default: return "menu"; }
  }
  if (menuOpen) return what == 0 ? "move" : what == 1 ? "open" : "";
  if (alarmRinging) return what == 0 || what == 1 ? "snooze" : "alarm off";
  switch (currentApp) {
    case APP_RADAR:
      if (currentMode == MODE_SELECT && planeCount) return what == 0 ? "next plane" : what == 1 ? "zoom mode" : (followIcao[0] ? "unfollow" : "follow");
      return what == 0 ? "zoom" : what == 1 ? "select mode" : (radarFocus ? "arrivals" : "departures");
    case APP_ARRIVALS: return what == 0 ? "scroll" : what == 1 ? (subPage ? "arrivals" : "departures") : "airport";
    case APP_APPROACH: return what == 0 ? "scroll" : what == 1 ? "to radar" : (subPage ? "list" : "profile");
    case APP_REPLAY:   return what == 0 ? "scrub" : what == 1 ? (replayPlay ? "pause" : "play") : "restart";
    case APP_CLOCK:    return what == 0 ? "face" : what == 1 ? "right dial" : (alarmOn ? "alarm off" : "alarm on");
    case APP_WEATHER:  return what == 0 ? "page" : what == 1 ? "next page" : "location";
    case APP_POMODORO:
      if (pomoSettings) return psetEdit ? (what == 0 ? "adjust" : what == 1 ? "save" : "cancel") : (what == 0 ? "move" : what == 1 ? "edit" : "back");
      if (pomoAskLog) return what == 0 ? "choose" : what == 1 ? "confirm" : "cancel";
      if (pomoState == POMO_SELECT) return pomoEditing ? (what == 0 ? "adjust" : what == 1 ? "start" : "") : (what == 0 ? "preset" : what == 1 ? "start" : "settings");
      if (pomoState == POMO_PAUSED) return what == 0 ? "" : what == 1 ? "resume" : "end";
      if (pomoState == POMO_FOCUS) return what == 0 ? "" : what == 1 ? "pause" : "end";
      return what == 0 ? "" : what == 1 ? (breathing ? "stop breath" : "breathing") : "stop";
    case APP_MARKET:   return what == 0 ? "symbol" : what == 1 ? "refresh" : "";
    case APP_STATS: case APP_SYSTEM: case APP_CRICKET: return what == 0 ? "page" : what == 1 ? "next page" : "";
    case APP_NEWS:     return what == 0 ? "headline" : what == 1 ? "skip 3" : "";
    case APP_BRIGHT:   return what == 0 ? "adjust" : what == 1 ? "next item" : "menu";
    case APP_LOCATION: return what == 0 ? "scroll" : what == 1 ? (locTarget ? "set weather" : "set home") : (locTarget ? "home list" : "weather list");
    case APP_AGENDA:   return what == 0 ? "events" : "";
    case APP_MEDIA:    return what == 0 ? "volume" : what == 1 ? (mac.playing ? "pause" : "play") : "next track";
    case APP_SETTINGS: {
      if (setCat < 0) return what == 0 ? "category" : what == 1 ? "open" : "";
      if (setEdit) return what == 0 ? "adjust" : what == 1 ? "done" : "cancel";
      const SetItem *it = setItem(setCat, setSel);
      const char *c = "edit";
      if (it) { if (it->type == ST_BOOL || it->type == ST_MASK || it->type == ST_APP || (it->type == ST_U8 && it->mx == 1)) c = "toggle"; else if (it->type == ST_ACTION) c = "run"; else if (it->type == ST_INFO) c = ""; }
      return what == 0 ? "item" : what == 1 ? c : "back";
    }
    default: return "";
  }
}
// 128 x 64 legend: title, rule, then label + action per row (left: 4 rows, right: 3 rows)
void drawNavOled(int side) {
  Adafruit_SSD1306 &d = side ? infoR : infoDisplay;
  static const char *LL[4] = {"turn", "click", "2x click", "hold 2s"};
  static const char *RL[3] = {"turn", "click", "hold"};
  // 6.15.4: the panel's yellow band is the bottom 16 rows (48..63): title there, the rows in the blue area above it
  int n = side ? 3 : 4, y = side ? 13 : 11, dy = side ? 15 : 12, col = side ? 46 : 64;
  for (int i = 0; i < n; i++) {
    const char *v = navHint(side, i); if (!v[0]) continue;
    d.setFont(&F_SM13); d.setCursor(0, y); d.print(side ? RL[i] : LL[i]);
    d.setFont(&F_SM13R); d.setCursor(col, y); d.print(v);
    y += dy;
  }
  d.setFont(&F_SM13); d.setCursor(0, 61); d.print(side ? "RIGHT KNOB" : "LEFT KNOB");
  d.setFont();
}
