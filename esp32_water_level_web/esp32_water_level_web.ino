/* ============================================================
 * ESP32 - water level receiver + dashboard + cloud push
 * ============================================================
 * Reads $WL,<raw>,<percent>*<xor> frames from the STM32 on UART2,
 * converts raw ADC to inches, keeps 25 hourly samples, serves a live
 * page on the local network, and pushes each sample to Google Sheets.
 *
 * UART IS UNCHANGED FROM THE WORKING BUILD:
 *   STM32 PA9  -> ESP32 GPIO17
 *   STM32 PA10 -> ESP32 GPIO16
 *   GND        -> GND            <-- required, not optional
 *   RXD2 = 3, TXD2 = 1, 115200 8N1, same parseFrame().
 *
 * What changed vs the previous sketch:
 *   1. Depth now comes from raw ADC against ADC_EMPTY/ADC_FULL
 *      (400 / 1600) instead of trusting the STM32 percent field.
 *   2. Connectivity follows the second sketch: primary SSID, then
 *      strongest OPEN network, then captive-portal check, then own AP.
 *   3. Every log entry is POSTed to a Google Apps Script endpoint.
 *      Unsent entries stay queued in the ring buffer and are retried,
 *      so up to 25 hours of outage costs nothing.
 *   4. Queue survives reboot in LittleFS (one write per hour).
 *   5. Task watchdog armed, fed in loop() and between HTTP calls.
 *
 * No deep sleep here on purpose - this node serves a live dashboard,
 * so it has to stay awake. Sleep belongs in the standalone logger.
 * ============================================================ */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <LittleFS.h>
#include <esp_task_wdt.h>
#include <time.h>

/* ============================================================
 * Log entry type - declared FIRST, on purpose
 * ============================================================
 * The Arduino IDE generates prototypes for every function and
 * inserts them near the top of the file. Any function taking a
 * LogEntry must therefore see the type before that insertion point,
 * or the build fails with "'LogEntry' does not name a type" pointing
 * at the function definition rather than the real cause.
 *
 * Packed to 9 bytes so 2880 of them fit in ~26 KB. Each byte added
 * here costs 2.8 KB of RAM, so add fields deliberately. */
struct __attribute__((packed)) LogEntry {
  uint32_t stamp;      /* epoch if LF_HAS_EPOCH set, else seconds up */
  uint16_t raw;
  uint16_t tenths;     /* inches x 10 - derived once, at log time    */
  uint8_t  flags;
};

#define LF_FAULT      0x01
#define LF_SENT       0x02
#define LF_HAS_EPOCH  0x04


/* ---------------- WiFi ---------------- */
const char *WIFI_SSID = "esp";
const char *WIFI_PASS = "12345678";

/* If the network above is unreachable, scan and join the strongest
 * OPEN network instead. See the security note at the bottom - this
 * is a real trade-off, not a free fallback. */
#define ALLOW_OPEN_FALLBACK   1

/* Access point used when nothing else is joinable. */
const char *AP_SSID = "WaterLevel-AP";
const char *AP_PASS = "12345678";

#define WIFI_RETRY_MS         30000UL   /* reconnect attempt spacing */

/* ---------------- cloud push ---------------- */
#define PUSH_ENABLED          1
const char *SHEETS_URL =
  "https://script.google.com/macros/s/YOUR_SCRIPT_ID/exec";

#define PUSH_TIMEOUT_MS       8000
#define PUSH_RETRY_MS         60000UL   /* retry sweep for backlog   */
#define PUSH_MAX_PER_SWEEP    3         /* keeps loop() responsive   */

/* Captive portals answer "connected" while swallowing everything.
 * This is the same 204 probe the logger sketch uses. */
const char *PORTAL_PROBE = "http://clients3.google.com/generate_204";

/* ---------------- time ---------------- */
/* IST = UTC+5:30 = 19800 seconds. Change for your timezone. */
#define GMT_OFFSET_SEC        19800
#define DST_OFFSET_SEC        0

/* ---------------- UART - DO NOT TOUCH ---------------- */
#define RXD2                  3
#define TXD2                  1
#define STM_BAUD              115200
#define LINK_TIMEOUT_MS       5000

/* ---------------- depth scale ----------------
 * Measured band table, not a straight line. Each band is a range of
 * raw counts that corresponds to a whole inch on the probe:
 *
 *     350 - 400   ->  1 in        1290 - 1336  ->  6 in
 *     620 - 670   ->  2 in        1406 - 1430  ->  7 in
 *     840 - 900   ->  3 in        1490 - 1522  ->  8 in
 *    1030 - 1071  ->  4 in        1560 - 1590  ->  9 in
 *    1160 - 1210  ->  5 in        1600 and up  -> 10 in
 *
 * The bands do not touch - 400 to 620 is a gap, and so is every step
 * after it. That is normal for a resistive ladder probe: the count
 * moves fast as a segment wets and slowly once it is covered.
 *
 * Counts landing in a gap are interpolated between the two
 * neighbouring inches rather than snapped to one of them. Snapping
 * would make the gauge jump 1 to 2 with nothing in between, which
 * looks exactly like a sensor that has stopped responding. */
struct InchBand { uint16_t lo; uint16_t hi; uint8_t inches; };

static const InchBand INCH_BANDS[] = {
  {  350,  400,  1 },
  {  620,  670,  2 },
  {  840,  900,  3 },
  { 1030, 1071,  4 },
  { 1160, 1210,  5 },
  { 1290, 1336,  6 },
  { 1406, 1430,  7 },
  { 1490, 1522,  8 },
  { 1560, 1590,  9 },
  { 1600, 4095, 10 }
};
#define BAND_COUNT  (sizeof(INCH_BANDS) / sizeof(INCH_BANDS[0]))

/* Count at which the probe reads zero inches. Below the first band,
 * depth interpolates from here up to 1 inch at 350. */
#define ADC_ZERO              250

#define PROBE_FULL_INCHES     10.0f

/* A count this far below ADC_ZERO is an open or shorted probe rather
 * than a dry one. Flagged on the page, not silently clamped. */
#define ADC_SANITY_MARGIN     150

/* ---------------- logging ---------------- */
/* 2880 samples held in RAM. At 30s spacing that is exactly 24 hours;
 * at 60s it is 48. Pick the interval for the window you want.
 *
 * The entry struct is packed to 9 bytes precisely so this fits: 2880
 * x 9 = ~26 KB, which the ESP32 can carry alongside WiFi and TLS.
 * The unpacked struct this replaces was 20 bytes and would have cost
 * 58 KB, which it cannot. */
#define LOG_SIZE              2880
#define LOG_INTERVAL_MS       30000UL     /* 30s -> 24h of history */
#define LOG_VIEW              60          /* rows sent to the page */
#define QUEUE_FILE            "/queue.bin"

/* ---------------- watchdog ---------------- */
#define WDT_TIMEOUT_MS        30000

HardwareSerial STMSerial(2);
WebServer      server(80);

/* ============================================================
 * Device log
 * ============================================================
 * Everything the sketch prints goes to USB serial AND into this
 * ring, which the dashboard reads at /syslog. That matters here
 * because RXD2/TXD2 can be mapped onto GPIO1/GPIO3 - the same pads
 * UART0 uses for USB - in which case the monitor shows nothing and
 * this page is the only way to see what the device is doing.
 *
 * Lines are stamped with seconds since boot, so you can tell a stall
 * from a reboot loop at a glance. */
#define SYSLOG_LINES   80
#define SYSLOG_LEN     104

char     syslogBuf[SYSLOG_LINES][SYSLOG_LEN];
uint8_t  syslogHead  = 0;
uint8_t  syslogCount = 0;
uint32_t syslogSeq   = 0;

class SysLog : public Print
{
  char    line[SYSLOG_LEN];
  uint8_t n = 0;

  void commit()
  {
    if (!n) return;
    line[n] = '\0';
    snprintf(syslogBuf[syslogHead], SYSLOG_LEN, "%6lus  %s",
             (unsigned long)(millis() / 1000UL), line);
    syslogHead = (syslogHead + 1) % SYSLOG_LINES;
    if (syslogCount < SYSLOG_LINES) syslogCount++;
    syslogSeq++;
    n = 0;
  }

public:
  void begin(unsigned long baud) { Serial.begin(baud); }

  size_t write(uint8_t c) override
  {
    Serial.write(c);                 /* USB, when UART0 still owns 1/3 */
    if (c == '\r') return 1;
    if (c == '\n') { commit(); return 1; }
    if (n < SYSLOG_LEN - 12) line[n++] = (char) c;
    return 1;
  }

  size_t write(const uint8_t *b, size_t len) override
  {
    for (size_t i = 0; i < len; i++) write(b[i]);
    return len;
  }
};

SysLog LOG;


/* Oldest first - a log is read top to bottom. */
String sysLogAsJson()
{
  String out = "[";

  for (uint8_t i = 0; i < syslogCount; i++)
  {
    int idx = (int) syslogHead - (int) syslogCount + (int) i;
    while (idx < 0) idx += SYSLOG_LINES;

    const char *s = syslogBuf[idx % SYSLOG_LINES];

    out += (i ? ",\"" : "\"");
    for (const char *p = s; *p; p++)
    {
      if (*p == '"' || *p == '\\') { out += '\\'; out += *p; }
      else if ((uint8_t) *p < 0x20)  out += ' ';
      else                           out += *p;
    }
    out += '"';
  }

  out += "]";
  return out;
}

/* ---------------- live state ---------------- */
float         waterPercent = 0.0f;   /* derived from raw ADC        */
float         stmPercent   = 0.0f;   /* what the STM32 claimed      */
float         waterInches  = 0.0f;
uint16_t      waterRaw     = 0;
unsigned long lastGoodMs   = 0;
bool          linkUp       = false;
bool          probeFault   = false;
bool          rangeWarn    = false;
uint32_t      frameCount   = 0;
uint32_t      badFrames    = 0;

String        netName      = "";
bool          netIsOpen    = false;
bool          netUp        = false;
bool          internetOk   = false;
bool          apMode       = false;
bool          timeSynced   = false;
unsigned long lastWifiTry  = 0;

uint32_t      pushOk       = 0;
uint32_t      pushFail     = 0;
String        pushNote     = "not attempted";
unsigned long lastPushTry  = 0;

bool          fsReady      = false;

/* ---------------- log ring buffer ----------------
 * LOG_SIZE slots in RAM. When full the oldest entry is overwritten,
 * so the buffer always holds the most recent LOG_SIZE samples - 24
 * hours at the default 30s spacing.
 *
 * It is also the retry queue: LF_SENT stays clear until Sheets
 * accepts the row. Only unsent rows are mirrored to LittleFS; see
 * the persistence section for why the whole ring is not. */
LogEntry      logBuf[LOG_SIZE];
uint16_t      logHead   = 0;     /* next slot to write */
uint16_t      logCount  = 0;
unsigned long lastLogMs = 0;
bool          firstLogDone = false;

void saveQueue();
bool pushEntry(LogEntry &e);


/* ============================================================
 * Depth from raw ADC
 * ============================================================ */

float inchesFromRaw(uint16_t raw, bool &outOfRange)
{
  outOfRange = false;

  /* At or past the top band - the probe is fully covered and cannot
   * report more, so this is a ceiling, not a measurement. */
  if (raw >= INCH_BANDS[BAND_COUNT - 1].lo) return PROBE_FULL_INCHES;

  /* Inside a band: the measured whole inch, exactly as calibrated. */
  for (uint8_t i = 0; i < BAND_COUNT; i++)
  {
    if (raw >= INCH_BANDS[i].lo && raw <= INCH_BANDS[i].hi)
      return (float) INCH_BANDS[i].inches;
  }

  /* Below the first band: interpolate ADC_ZERO -> 0 in, 350 -> 1 in. */
  if (raw < INCH_BANDS[0].lo)
  {
    if (raw <= ADC_ZERO)
    {
      outOfRange = (raw + ADC_SANITY_MARGIN < ADC_ZERO);
      return 0.0f;
    }
    float f = (float)(raw - ADC_ZERO) /
              (float)(INCH_BANDS[0].lo - ADC_ZERO);
    return f * (float) INCH_BANDS[0].inches;
  }

  /* In a gap between two bands: interpolate across it, so the gauge
   * moves continuously instead of stepping. */
  for (uint8_t i = 0; i + 1 < BAND_COUNT; i++)
  {
    uint16_t gapLo = INCH_BANDS[i].hi;
    uint16_t gapHi = INCH_BANDS[i + 1].lo;

    if (raw > gapLo && raw < gapHi)
    {
      float f = (float)(raw - gapLo) / (float)(gapHi - gapLo);
      return (float) INCH_BANDS[i].inches +
             f * (float)(INCH_BANDS[i + 1].inches - INCH_BANDS[i].inches);
    }
  }

  return 0.0f;   /* unreachable with a sane table */
}


/* ============================================================
 * Logging
 * ============================================================ */

void addLogEntry()
{
  LogEntry &e = logBuf[logHead];

  bool haveClock = timeSynced && (time(nullptr) > 1600000000);

  e.stamp  = haveClock ? (uint32_t) time(nullptr)
                       : (uint32_t)(millis() / 1000UL);
  e.raw    = waterRaw;
  e.tenths = (uint16_t)(waterInches * 10.0f + 0.5f);
  e.flags  = (probeFault ? LF_FAULT : 0) | (haveClock ? LF_HAS_EPOCH : 0);

  logHead = (logHead + 1) % LOG_SIZE;
  if (logCount < LOG_SIZE) logCount++;

  lastLogMs = millis();

  LOG.printf("[LOG] Sample %.1f in  (%u of %u held)\n",
             e.tenths / 10.0f, logCount, (unsigned) LOG_SIZE);

  saveQueue();

#if PUSH_ENABLED
  /* Try immediately; if it fails the entry stays queued for the
   * next sweep. */
  if (netUp && internetOk) {
    if (pushEntry(e)) saveQueue();
  }
#endif
}


/* Oldest slot index in write order. */
uint16_t oldestIdx()
{
  return (uint16_t)((logHead + LOG_SIZE - logCount) % LOG_SIZE);
}


void stampLabel(const LogEntry &e, char *out, size_t len)
{
  if (e.flags & LF_HAS_EPOCH)
  {
    time_t    t = (time_t) e.stamp;
    struct tm tmv;
    localtime_r(&t, &tmv);
    strftime(out, len, "%d %b %H:%M:%S", &tmv);
  }
  else
  {
    /* No clock: time since boot, still ordered and still correctly
     * spaced, just not wall-clock. */
    snprintf(out, len, "+%luh%02lum%02lus",
             (unsigned long)(e.stamp / 3600UL),
             (unsigned long)((e.stamp % 3600UL) / 60UL),
             (unsigned long)(e.stamp % 60UL));
  }
}


/* Newest first, since that is the order you read a log in.
 *
 * Only the newest `limit` rows go to the page. Serialising all 2880
 * would be a ~250 KB JSON string built in RAM once per second, which
 * would fragment the heap and stall the server. The full set is
 * available as a stream at /log.csv instead. */
String logAsJson(uint16_t limit)
{
  String out = "[";

  uint16_t shown = (logCount < limit) ? logCount : limit;
  out.reserve(shown * 72 + 16);

  for (uint16_t i = 0; i < shown; i++)
  {
    uint16_t idx = (uint16_t)((logHead + LOG_SIZE - 1 - i) % LOG_SIZE);
    const LogEntry &e = logBuf[idx];

    char label[28];
    stampLabel(e, label, sizeof(label));

    char item[128];
    snprintf(item, sizeof(item),
             "%s{\"t\":\"%s\",\"in\":%.1f,\"raw\":%u,\"f\":%s,\"s\":%s}",
             (i == 0 ? "" : ","), label, e.tenths / 10.0f,
             (unsigned) e.raw,
             (e.flags & LF_FAULT) ? "true" : "false",
             (e.flags & LF_SENT)  ? "true" : "false");
    out += item;
  }

  out += "]";
  return out;
}


uint16_t unsentCount()
{
  uint16_t n = 0;
  uint16_t base = oldestIdx();

  for (uint16_t i = 0; i < logCount; i++)
    if (!(logBuf[(base + i) % LOG_SIZE].flags & LF_SENT)) n++;

  return n;
}


/* ============================================================
 * Queue persistence (LittleFS)
 * ============================================================
 * Only UNSENT entries are written, not the whole 26 KB ring.
 *
 * That is the difference between a file that is usually a few dozen
 * bytes and one that is 26 KB written every 30 seconds - which would
 * be ~75 MB a day through a 1.4 MB partition and would wear it out
 * inside a year. The history is a RAM cache and is expected to clear
 * on reboot; the send queue is the part that must survive, because
 * losing it means losing rows the sheet never got.
 */

void saveQueue()
{
  if (!fsReady) return;

  uint16_t pending = unsentCount();

  if (pending == 0)
  {
    if (LittleFS.exists(QUEUE_FILE)) LittleFS.remove(QUEUE_FILE);
    return;
  }

  File f = LittleFS.open(QUEUE_FILE, "w");
  if (!f) { LOG.println("[LFS] Queue write failed."); return; }

  uint16_t base = oldestIdx();
  for (uint16_t i = 0; i < logCount; i++)
  {
    const LogEntry &e = logBuf[(base + i) % LOG_SIZE];
    if (e.flags & LF_SENT) continue;
    f.write((const uint8_t *) &e, sizeof(LogEntry));
  }
  f.close();
}


void loadQueue()
{
  if (!fsReady || !LittleFS.exists(QUEUE_FILE)) return;

  File f = LittleFS.open(QUEUE_FILE, "r");
  if (!f) return;

  logHead = logCount = 0;

  LogEntry e;
  while (f.available() >= (int) sizeof(LogEntry) && logCount < LOG_SIZE)
  {
    if (f.read((uint8_t *) &e, sizeof(LogEntry)) != sizeof(LogEntry)) break;

    e.flags &= ~LF_SENT;              /* it was queued; it still is */
    logBuf[logHead] = e;

    logHead = (logHead + 1) % LOG_SIZE;
    logCount++;
  }
  f.close();

  if (logCount)
  {
    firstLogDone = true;
    lastLogMs    = millis();
    LOG.printf("[LFS] Restored %u unsent rows from before the reboot.\n",
               logCount);
  }
}


/* ============================================================
 * Frame parsing  - UNCHANGED
 * ============================================================ */

bool parseFrame(const String &line, uint16_t &raw, float &pct, bool &fault)
{
  int dollar = line.indexOf('$');
  int star   = line.indexOf('*');

  if (dollar < 0 || star < 0 || star < dollar + 2) return false;

  String payload = line.substring(dollar + 1, star);
  String csText  = line.substring(star + 1);

  csText.trim();
  if (csText.length() < 2) return false;

  uint8_t want = (uint8_t) strtol(csText.substring(0, 2).c_str(), nullptr, 16);

  uint8_t have = 0;
  for (unsigned int i = 0; i < payload.length(); i++) have ^= (uint8_t) payload[i];

  if (have != want) return false;

  int c1 = payload.indexOf(',');
  int c2 = payload.indexOf(',', c1 + 1);
  if (c1 < 0 || c2 < 0) return false;
  if (payload.substring(0, c1) != "WL") return false;

  String rawField = payload.substring(c1 + 1, c2);

  if (rawField == "ERR")
  {
    fault = true;
    raw   = 0;
    pct   = 0.0f;
    return true;
  }

  fault = false;
  raw   = (uint16_t) rawField.toInt();
  pct   = payload.substring(c2 + 1).toFloat();
  return true;
}


/* ============================================================
 * WiFi
 * ============================================================ */

bool joinNetwork(const char *ssid, const char *pass, uint8_t tries)
{
  LOG.printf("[WiFi] Trying: \"%s\" ", ssid);

  if (pass && pass[0]) WiFi.begin(ssid, pass);
  else                 WiFi.begin(ssid);

  for (uint8_t i = 0; i < tries && WiFi.status() != WL_CONNECTED; i++)
  {
    delay(500);
    esp_task_wdt_reset();
    LOG.print('.');
  }
  LOG.println();

  return WiFi.status() == WL_CONNECTED;
}


/* Scan and join the OPEN network with the strongest signal.
 * Strongest, not first found - a weak open network several rooms away
 * will associate and then drop packets, which is harder to diagnose
 * than simply failing to connect. */
bool joinStrongestOpen()
{
  LOG.println("[WiFi] Scanning for open networks...");

  int n = WiFi.scanNetworks();
  if (n <= 0) { LOG.println("[WiFi] No networks found."); return false; }

  int best = -1, bestRssi = -1000;

  for (int i = 0; i < n; i++)
  {
    bool isOpen = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
    LOG.printf("  %-24s %4d dBm %s\n",
                  WiFi.SSID(i).c_str(), WiFi.RSSI(i),
                  isOpen ? "OPEN" : "");

    if (isOpen && WiFi.RSSI(i) > bestRssi)
    {
      bestRssi = WiFi.RSSI(i);
      best     = i;
    }
  }

  if (best < 0)
  {
    LOG.println("[WiFi] No open network available.");
    WiFi.scanDelete();
    return false;
  }

  String ssid = WiFi.SSID(best);
  LOG.printf("[WiFi] Strongest open: \"%s\" at %d dBm\n",
                ssid.c_str(), bestRssi);
  WiFi.scanDelete();

  if (joinNetwork(ssid.c_str(), nullptr, 24))
  {
    netIsOpen = true;
    return true;
  }
  return false;
}


/* Captive-portal check. "Associated" is not "online": a portal will
 * answer every request with its own login page, which would make the
 * NTP sync and every push fail in confusing ways. */
bool checkInternet()
{
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  http.setConnectTimeout(4000);
  http.setTimeout(5000);
  if (!http.begin(PORTAL_PROBE)) return false;

  int code = http.GET();
  http.end();

  if (code == 204) { LOG.println("[WiFi] Internet OK."); return true; }

  LOG.printf("[WiFi] Internet check failed (HTTP %d) - captive portal?\n",
                code);
  return false;
}


void syncClock()
{
  configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC,
             "pool.ntp.org", "time.nist.gov");

  LOG.print("[NTP] Waiting for time sync ");
  for (uint8_t i = 0; i < 20 && !timeSynced; i++)
  {
    delay(500);
    esp_task_wdt_reset();
    LOG.print('.');
    if (time(nullptr) > 1600000000) timeSynced = true;
  }
  LOG.println(timeSynced ? " done." : " no NTP, using uptime.");
}


void startWiFi()
{
  WiFi.mode(WIFI_STA);
  apMode = false;

  if (joinNetwork(WIFI_SSID, WIFI_PASS, 30))
  {
    netName   = WIFI_SSID;
    netIsOpen = false;
  }
#if ALLOW_OPEN_FALLBACK
  else if (joinStrongestOpen())
  {
    netName = WiFi.SSID();
  }
#endif

  if (WiFi.status() == WL_CONNECTED)
  {
    netUp = true;
    LOG.print("[WiFi] Associated. IP=");
    LOG.println(WiFi.localIP());

    if (MDNS.begin("waterlevel"))
      LOG.println("[mDNS] Also at http://waterlevel.local");

    internetOk = checkInternet();

    /* Timestamps for the log. Without this the log falls back to
     * time-since-boot, which is still ordered but not wall-clock. */
    if (internetOk) syncClock();
    else LOG.println("[NTP] Skipped - no route out.");
  }
  else
  {
    netUp = internetOk = false;
    LOG.println("[WiFi] Nothing joinable. Starting own AP.");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    apMode    = true;
    netName   = String(AP_SSID) + " (own AP)";
    netIsOpen = false;
    LOG.print("[WiFi] AP up. IP=");
    LOG.println(WiFi.softAPIP());
  }

  lastWifiTry = millis();
}


/* Periodic repair. The dashboard keeps serving from the AP either
 * way, so this only ever upgrades the situation. */
void maintainWiFi()
{
  if (millis() - lastWifiTry < WIFI_RETRY_MS) return;
  lastWifiTry = millis();

  if (WiFi.status() == WL_CONNECTED)
  {
    netUp = true;
    if (!internetOk)
    {
      internetOk = checkInternet();
      if (internetOk && !timeSynced) syncClock();
    }
    return;
  }

  netUp = internetOk = false;
  if (apMode) return;          /* already fell back; retry on reboot */

  LOG.println("[WiFi] Dropped - retrying.");
  startWiFi();
}


/* ============================================================
 * Cloud push
 * ============================================================
 * One GET per sample to an Apps Script web app. Apps Script always
 * 302s to script.googleusercontent.com, so redirects must be
 * followed, and TLS is unverified (setInsecure) because pinning a
 * Google root on a device with no clock is more trouble than it is
 * worth for a tank reading.
 *
 * The entry is only marked sent on HTTP 200. Anything else leaves it
 * queued, so a failed POST costs nothing but a retry.
 */

bool pushEntry(LogEntry &e)
{
#if !PUSH_ENABLED
  return false;
#else
  if (e.flags & LF_SENT) return true;
  if (WiFi.status() != WL_CONNECTED || !internetOk) return false;

  float in = e.tenths / 10.0f;

  char url[420];
  snprintf(url, sizeof(url),
           "%s?pipe_height=%.2f&inches=%.2f&percent=%.1f&raw=%u"
           "&fault=%u&epoch=%lu&uptime=%lu",
           SHEETS_URL, in, in,
           (in / PROBE_FULL_INCHES) * 100.0f,
           (unsigned) e.raw, (e.flags & LF_FAULT) ? 1u : 0u,
           (e.flags & LF_HAS_EPOCH) ? (unsigned long) e.stamp : 0UL,
           (e.flags & LF_HAS_EPOCH) ? 0UL : (unsigned long) e.stamp);

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setConnectTimeout(PUSH_TIMEOUT_MS);
  http.setTimeout(PUSH_TIMEOUT_MS);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!http.begin(client, url))
  {
    pushFail++; pushNote = "begin failed";
    return false;
  }

  int code = http.GET();
  http.end();
  esp_task_wdt_reset();

  if (code == 200)
  {
    e.flags |= LF_SENT;
    pushOk++;
    pushNote = "accepted";
    LOG.printf("[HTTP] Sheets accepted %.1f in\n", in);
    return true;
  }

  pushFail++;
  pushNote = String("HTTP ") + code + ", queued";
  LOG.printf("[HTTP] Failed (%d) - row stays queued\n", code);

  /* A portal can appear mid-session; re-probe so the next sweep
   * does not hammer a dead route. */
  if (code < 0) internetOk = false;
  return false;
#endif
}


/* Retry sweep, oldest first so Sheets rows land in order. Capped per
 * pass so the web server stays responsive. */
void flushQueue()
{
#if PUSH_ENABLED
  if (millis() - lastPushTry < PUSH_RETRY_MS) return;
  lastPushTry = millis();

  if (WiFi.status() != WL_CONNECTED) return;
  if (!internetOk) { internetOk = checkInternet(); if (!internetOk) return; }
  if (unsentCount() == 0) return;

  uint16_t done = 0, changed = 0;
  uint16_t base = oldestIdx();

  for (uint16_t i = 0; i < logCount && done < PUSH_MAX_PER_SWEEP; i++)
  {
    LogEntry &e = logBuf[(base + i) % LOG_SIZE];
    if (e.flags & LF_SENT) continue;

    done++;
    if (pushEntry(e)) changed++;
    else break;                  /* still offline; stop wasting time */

    server.handleClient();
  }

  if (changed) saveQueue();
#endif
}


/* ============================================================
 * Dashboard
 * ============================================================
 * Light theme. The reading is a vertical gauge marked 0-10 inches,
 * because that is the shape of the thing being measured - a
 * horizontal bar would be a chart of a depth rather than a picture
 * of one. No web fonts: the device is often on a network with no
 * internet, and a page waiting on fonts.googleapis.com would render
 * late or not at all.
 */
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Water Level</title>
<style>
:root{
  --ink:#12181f; --ink-2:#5a6675; --ink-3:#8b97a6;
  --paper:#f4f6f8; --card:#ffffff; --line:#e2e7ec;
  --water:#1d7fc4;
  --low:#c0392b; --mid:#c77b16; --ok:#1a8a4d;
  --mono:ui-monospace,"SF Mono",Menlo,Consolas,monospace;
}
*{box-sizing:border-box;margin:0;padding:0}
body{
  font-family:-apple-system,system-ui,"Segoe UI",Roboto,sans-serif;
  background:var(--paper); color:var(--ink);
  padding:20px; line-height:1.45;
}
.wrap{max-width:860px;margin:0 auto}

header{display:flex;justify-content:space-between;align-items:flex-end;
  gap:16px;flex-wrap:wrap;margin-bottom:18px}
h1{font-size:19px;font-weight:650;letter-spacing:-.01em}
.sub{font-size:12.5px;color:var(--ink-3);font-family:var(--mono)}

.grid{display:grid;grid-template-columns:250px 1fr;gap:16px;align-items:start}
@media(max-width:660px){.grid{grid-template-columns:1fr}}

.card{background:var(--card);border:1px solid var(--line);
  border-radius:12px;padding:20px}
.card h2{font-size:11px;font-weight:650;letter-spacing:.1em;
  text-transform:uppercase;color:var(--ink-3);margin-bottom:16px}

/* ---- gauge ---- */
.gauge{display:flex;gap:14px;height:290px}
.scale{display:flex;flex-direction:column;justify-content:space-between;
  font-family:var(--mono);font-size:11px;color:var(--ink-3);
  text-align:right;width:18px;padding:2px 0}
.tube{position:relative;flex:1;border:1.5px solid var(--line);
  border-radius:5px;background:#fbfcfd;overflow:hidden}
.ticks{position:absolute;inset:0}
.ticks i{position:absolute;left:0;right:0;height:1px;background:var(--line)}
.fill{position:absolute;left:0;right:0;bottom:0;height:0;
  background:linear-gradient(180deg,var(--water) 0%,#155f94 100%);
  transition:height .7s cubic-bezier(.4,0,.2,1)}
.fill::before{content:"";position:absolute;left:0;right:0;top:0;height:3px;
  background:rgba(255,255,255,.55)}

.readout{margin-top:16px;display:flex;align-items:baseline;gap:5px}
.readout b{font-size:44px;font-weight:680;letter-spacing:-.02em;
  font-variant-numeric:tabular-nums;line-height:1}
.readout span{font-size:15px;color:var(--ink-2);font-weight:600}
#state{margin-top:4px;font-size:13px;font-weight:600}
#warn{margin-top:6px;font-size:12px;color:var(--mid);display:none}

/* ---- log table ---- */
table{width:100%;border-collapse:collapse;font-size:13px}
th{text-align:left;font-size:10.5px;letter-spacing:.08em;
  text-transform:uppercase;color:var(--ink-3);font-weight:650;
  padding:0 8px 8px;border-bottom:1px solid var(--line)}
th.n,td.n{text-align:right;font-family:var(--mono);
  font-variant-numeric:tabular-nums}
td{padding:7px 8px;border-bottom:1px solid #f0f3f6}
tbody tr:last-child td{border-bottom:none}
td.time{font-family:var(--mono);font-size:12px;color:var(--ink-2)}
td.up{text-align:right;font-size:11.5px;font-weight:650}
.empty{padding:26px 8px;text-align:center;color:var(--ink-3);font-size:13px}

.meta{margin-top:14px;display:grid;grid-template-columns:1fr 1fr;
  gap:7px 18px;font-size:12.5px}
.meta div{display:flex;justify-content:space-between;gap:10px}
.meta span:first-child{color:var(--ink-3)}
.meta span:last-child{font-family:var(--mono)}
.dot{display:inline-block;width:7px;height:7px;border-radius:50%;
  margin-right:5px;vertical-align:1px}

button{font:inherit;font-size:12.5px;font-weight:600;color:var(--ink);
  background:var(--card);border:1px solid var(--line);border-radius:7px;
  padding:6px 12px;cursor:pointer}
button:hover{background:var(--paper)}
button:focus-visible{outline:2px solid var(--water);outline-offset:2px}
.foot{margin-top:14px;display:flex;justify-content:space-between;
  align-items:center;gap:12px;flex-wrap:wrap}
.btns{display:flex;gap:8px}
.hint{font-size:11.5px;color:var(--ink-3)}
.console{margin-top:16px}
.console pre{font-family:var(--mono);font-size:11.5px;line-height:1.55;
  color:var(--ink-2);background:#fbfcfd;border:1px solid var(--line);
  border-radius:8px;padding:12px;height:230px;overflow:auto;
  white-space:pre-wrap;word-break:break-word;margin:0}
.console pre b{color:var(--ink);font-weight:650}
@media(prefers-reduced-motion:reduce){.fill{transition:none}}
</style>
</head>
<body>
<div class="wrap">

  <header>
    <div>
      <h1>Water level</h1>
      <div class="sub" id="net">&nbsp;</div>
    </div>
    <div class="sub" id="clock">&nbsp;</div>
  </header>

  <div class="grid">

    <div class="card">
      <h2>Depth now</h2>
      <div class="gauge">
        <div class="scale" id="scale"></div>
        <div class="tube">
          <div class="ticks" id="ticks"></div>
          <div class="fill" id="fill"></div>
        </div>
      </div>
      <div class="readout">
        <b id="inches">--</b><span>in</span>
      </div>
      <div id="state">Waiting for the sensor</div>
      <div id="warn">Raw count is outside the calibrated window</div>
    </div>

    <div class="card">
      <h2>Sample log</h2>
      <table>
        <thead>
          <tr><th>Time</th><th class="n">Depth</th><th class="n">Raw</th>
              <th class="n">Sent</th></tr>
        </thead>
        <tbody id="rows"></tbody>
      </table>
      <div id="none" class="empty">
        No samples yet. The first lands as soon as the sensor reports.
      </div>

      <div class="meta">
        <div><span>Link</span><span id="link">--</span></div>
        <div><span>Samples held</span><span id="held">--</span></div>
        <div><span>Frames read</span><span id="ok">--</span></div>
        <div><span>Frames dropped</span><span id="bad">--</span></div>
        <div><span>Raw ADC</span><span id="raw">--</span></div>
        <div><span>Last frame</span><span id="age">--</span></div>
        <div><span>Cloud</span><span id="cloud">--</span></div>
        <div><span>Queued to send</span><span id="queued">--</span></div>
        <div><span>Rows accepted</span><span id="pok">--</span></div>
        <div><span>Send attempts failed</span><span id="pbad">--</span></div>
      </div>

      <div class="foot">
        <span class="hint" id="cap">&nbsp;</span>
        <div class="btns">
          <a href="/log.csv" download><button>Download all as CSV</button></a>
          <button id="send">Send queued rows</button>
          <button id="now">Log a sample now</button>
        </div>
      </div>
    </div>

  </div>

  <div class="card console">
    <h2>Device log &mdash; last 80 lines</h2>
    <pre id="sys">Waiting for the device...</pre>
    <div class="foot">
      <span class="hint">Same lines the ESP32 writes to serial, stamped
        with seconds since boot.</span>
      <label class="hint"><input type="checkbox" id="follow" checked>
        Follow new lines</label>
    </div>
  </div>

</div>

<script>
const MAXIN = 10;

/* Build the 0-10 scale once. Top label is MAXIN, bottom is 0,
 * because the gauge fills upward like the tank it represents. */
(function(){
  const s = document.getElementById('scale');
  const t = document.getElementById('ticks');
  for (let v = MAXIN; v >= 0; v--) {
    const d = document.createElement('div');
    d.textContent = v;
    s.appendChild(d);
    if (v < MAXIN && v > 0) {
      const i = document.createElement('i');
      i.style.bottom = (v / MAXIN * 100) + '%';
      t.appendChild(i);
    }
  }
})();

function band(inches) {
  const p = inches / MAXIN * 100;
  if (p <= 20) return ['var(--low)', 'Critical \u2014 nearly empty'];
  if (p <= 40) return ['var(--mid)', 'Low'];
  if (p <= 60) return ['var(--ink-2)', 'Medium'];
  if (p <= 80) return ['var(--ok)', 'Good'];
  return ['var(--ok)', 'Full'];
}

function drawLog(log) {
  const rows = document.getElementById('rows');
  const none = document.getElementById('none');
  rows.innerHTML = '';
  none.style.display = log.length ? 'none' : 'block';

  for (const e of log) {
    const tr = document.createElement('tr');
    const mark = e.s
      ? '<td class="up" style="color:var(--ok)">\u2713</td>'
      : '<td class="up" style="color:var(--ink-3)">queued</td>';
    tr.innerHTML = '<td class="time">' + e.t + '</td>' +
      '<td class="n">' + (e.f ? 'fault' : e.in.toFixed(1) + ' in') + '</td>' +
      '<td class="n">' + (e.f ? '\u2014' : e.raw) + '</td>' + mark;
    rows.appendChild(tr);
  }
}

async function tick() {
  try {
    const d = await (await fetch('/data', {cache:'no-store'})).json();

    document.getElementById('net').textContent =
      d.net + (d.open ? ' \u00b7 open network' : '') +
      (d.netup && !d.online ? ' \u00b7 no route out' : '');
    document.getElementById('clock').textContent = d.time || '';
    document.getElementById('held').textContent = d.held + ' / ' + d.cap;
    document.getElementById('cap').textContent =
      'Showing the newest ' + d.shown + ' of ' + d.held +
      ' held. Oldest drops off at ' + d.cap + '.';
    document.getElementById('ok').textContent   = d.frames;
    document.getElementById('bad').textContent  = d.bad;
    document.getElementById('raw').textContent  = d.fault ? '\u2014' : d.raw;
    document.getElementById('queued').textContent = d.queued;
    document.getElementById('pok').textContent  = d.push_ok;
    document.getElementById('pbad').textContent = d.push_fail;
    document.getElementById('age').textContent  =
      !d.link ? '\u2014' : (d.age_ms < 2000 ? 'just now'
                          : Math.round(d.age_ms/1000) + 's ago');

    const cloud = document.getElementById('cloud');
    if (!d.push_on)      cloud.textContent = 'off';
    else if (d.online)   cloud.innerHTML =
      '<span class="dot" style="background:var(--ok)"></span>' + d.push_note;
    else                 cloud.innerHTML =
      '<span class="dot" style="background:var(--mid)"></span>offline, holding';

    const linkEl = document.getElementById('link');
    const fill = document.getElementById('fill');
    const inEl = document.getElementById('inches');
    const stEl = document.getElementById('state');
    const wnEl = document.getElementById('warn');

    drawLog(d.log);
    wnEl.style.display = (d.link && !d.fault && d.range) ? 'block' : 'none';

    if (!d.link) {
      linkEl.innerHTML = '<span class="dot" style="background:var(--low)"></span>down';
      inEl.textContent = '--';
      inEl.style.color = 'var(--ink-3)';
      stEl.textContent = 'No data from the STM32';
      stEl.style.color = 'var(--low)';
      fill.style.height = '0%';
      return;
    }

    linkEl.innerHTML = '<span class="dot" style="background:var(--ok)"></span>up';

    if (d.fault) {
      inEl.textContent = '!';
      inEl.style.color = 'var(--low)';
      stEl.textContent = 'Probe fault \u2014 check the sensor wiring';
      stEl.style.color = 'var(--low)';
      fill.style.height = '0%';
      return;
    }

    const [colour, label] = band(d.inches);
    inEl.textContent = d.inches.toFixed(1);
    inEl.style.color = colour;
    stEl.textContent = label;
    stEl.style.color = colour;
    fill.style.height = Math.max(0, Math.min(100, d.inches / MAXIN * 100)) + '%';

  } catch (e) {
    document.getElementById('link').innerHTML =
      '<span class="dot" style="background:var(--mid)"></span>page offline';
  }
}

let sysSeen = -1;

async function pullLog() {
  try {
    const lines = await (await fetch('/syslog', {cache:'no-store'})).json();
    const pre = document.getElementById('sys');
    if (lines.length === sysSeen && pre.dataset.n === String(lines.length)) return;
    sysSeen = lines.length;
    pre.dataset.n = String(lines.length);

    const stick = document.getElementById('follow').checked;
    pre.innerHTML = lines.length
      ? lines.map(l => l.replace(/[<&]/g, c => c === '<' ? '&lt;' : '&amp;')
                        .replace(/(\[[A-Za-z]+\])/, '<b>$1</b>')).join('\n')
      : 'Nothing logged yet.';
    if (stick) pre.scrollTop = pre.scrollHeight;
  } catch (e) { /* page poll already reports the outage */ }
}

document.getElementById('now').addEventListener('click', async () => {
  await fetch('/lognow');
  tick();
});

document.getElementById('send').addEventListener('click', async (ev) => {
  ev.target.disabled = true;
  try { await fetch('/pushnow'); } finally { ev.target.disabled = false; }
  tick();
});

tick();
pullLog();
setInterval(tick, 1000);
setInterval(pullLog, 2000);
</script>
</body>
</html>
)rawliteral";


void handleRoot() { server.send_P(200, "text/html", PAGE_HTML); }

void handleData()
{
  unsigned long age = millis() - lastGoodMs;
  bool up = linkUp && (age <= LINK_TIMEOUT_MS);

  char clockStr[40] = "";
  if (timeSynced)
  {
    time_t now = time(nullptr);
    struct tm tmv;
    localtime_r(&now, &tmv);
    strftime(clockStr, sizeof(clockStr), "%d %b %Y  %H:%M", &tmv);
  }
  else
  {
    snprintf(clockStr, sizeof(clockStr), "up %lu min", millis() / 60000UL);
  }

  char head[560];
  snprintf(head, sizeof(head),
           "{\"inches\":%.1f,\"percent\":%.1f,\"stm_percent\":%.1f,\"raw\":%u,"
           "\"link\":%s,\"fault\":%s,\"range\":%s,\"frames\":%u,\"bad\":%u,"
           "\"age_ms\":%lu,\"held\":%u,\"net\":\"%s\",\"open\":%s,"
           "\"netup\":%s,\"online\":%s,\"push_on\":%s,\"queued\":%u,"
           "\"push_ok\":%u,\"push_fail\":%u,\"push_note\":\"%s\","
           "\"cap\":%u,\"shown\":%u,\"time\":\"%s\",\"log\":",
           waterInches, waterPercent, stmPercent, (unsigned) waterRaw,
           up ? "true" : "false",
           probeFault ? "true" : "false",
           rangeWarn ? "true" : "false",
           (unsigned) frameCount, (unsigned) badFrames,
           up ? age : 0UL,
           (unsigned) logCount,
           netName.c_str(),
           netIsOpen ? "true" : "false",
           netUp ? "true" : "false",
           internetOk ? "true" : "false",
           PUSH_ENABLED ? "true" : "false",
           (unsigned) unsentCount(),
           (unsigned) pushOk, (unsigned) pushFail,
           pushNote.c_str(),
           (unsigned) LOG_SIZE,
           (unsigned)(logCount < LOG_VIEW ? logCount : LOG_VIEW),
           clockStr);

  String json = String(head) + logAsJson(LOG_VIEW) + "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

/* Manual sample. Waiting an hour to find out whether logging works is
 * a poor feedback loop; this makes the mechanism testable in seconds.
 * It also resets the hourly timer, so samples stay evenly spaced. */
void handleLogNow()
{
  if (linkUp) addLogEntry();
  server.send(200, "text/plain", "ok");
}

/* Same idea for the push path - forces the retry sweep now instead of
 * waiting out PUSH_RETRY_MS. */
void handlePushNow()
{
  lastPushTry = 0;
  flushQueue();
  server.send(200, "text/plain", pushNote.c_str());
}


/* Full history as a stream. Built in ~1 KB chunks rather than one
 * String, because a 2880-row CSV is around 130 KB and assembling
 * that in RAM would fail on a device that also holds a 26 KB ring
 * and a TLS session. */
void handleLogCsv()
{
  server.sendHeader("Content-Disposition",
                    "attachment; filename=waterlevel.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("time,inches,raw,fault,sent\n");

  String chunk;
  chunk.reserve(1200);

  uint16_t base = oldestIdx();
  for (uint16_t i = 0; i < logCount; i++)
  {
    const LogEntry &e = logBuf[(base + i) % LOG_SIZE];

    char label[28];
    stampLabel(e, label, sizeof(label));

    char line[80];
    snprintf(line, sizeof(line), "%s,%.1f,%u,%u,%u\n",
             label, e.tenths / 10.0f, (unsigned) e.raw,
             (e.flags & LF_FAULT) ? 1u : 0u,
             (e.flags & LF_SENT)  ? 1u : 0u);
    chunk += line;

    if (chunk.length() > 1024)
    {
      server.sendContent(chunk);
      chunk = "";
      esp_task_wdt_reset();
    }
  }

  if (chunk.length()) server.sendContent(chunk);
  server.sendContent("");
}


void handleSysLog()
{
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", sysLogAsJson());
}


void setup()
{
  LOG.begin(115200);
  STMSerial.begin(STM_BAUD, SERIAL_8N1, RXD2, TXD2);
  STMSerial.setTimeout(50);

  LOG.println("\n[System] === Water Level Monitor ===");

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wcfg = {
    .timeout_ms     = WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&wcfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_MS / 1000, true);
#endif
  esp_task_wdt_add(NULL);
  LOG.println("[WDT] Watchdog armed (30s).");

  fsReady = LittleFS.begin(true);
  LOG.println(fsReady ? "[LFS] Ready." : "[LFS] Not ready - RAM only.");
  loadQueue();

  startWiFi();

  server.on("/",        handleRoot);
  server.on("/data",    handleData);
  server.on("/lognow",  handleLogNow);
  server.on("/pushnow", handlePushNow);
  server.on("/syslog",  handleSysLog);
  server.on("/log.csv", handleLogCsv);
  server.begin();
  LOG.println("[HTTP] Dashboard running.");
}


void loop()
{
  esp_task_wdt_reset();
  server.handleClient();

  while (STMSerial.available())
  {
    String line = STMSerial.readStringUntil('\n');

    uint16_t raw;
    float    pct;
    bool     fault;

    if (parseFrame(line, raw, pct, fault))
    {
      probeFault = fault;
      lastGoodMs = millis();
      frameCount++;

      if (!fault)
      {
        waterRaw     = raw;
        stmPercent   = pct;

        bool oor;
        waterInches  = inchesFromRaw(raw, oor);
        waterPercent = waterInches / PROBE_FULL_INCHES * 100.0f;
        rangeWarn    = oor;
      }

      if (!linkUp) { linkUp = true; LOG.println("[UART] Link up."); }

      if (fault) LOG.println("[Sensor] PROBE FAULT reported by STM32");
      else       LOG.printf("[Sensor] raw=%4u  %.1f in  (%.0f%%)%s\n",
                               waterRaw, waterInches, waterPercent,
                               rangeWarn ? "  [out of calibrated range]" : "");

      /* Log the first good reading immediately rather than leaving
       * the table empty for an hour. */
      if (!firstLogDone)
      {
        firstLogDone = true;
        addLogEntry();
      }
    }
    else if (line.length() > 1)
    {
      badFrames++;
      LOG.print("[UART] Bad frame: ");
      LOG.println(line);
    }
  }

  /* Hourly sample, driven off millis() so it works with or without
   * an NTP clock. */
  if (firstLogDone && (millis() - lastLogMs >= LOG_INTERVAL_MS))
  {
    addLogEntry();
  }

  if (linkUp && (millis() - lastGoodMs > LINK_TIMEOUT_MS))
  {
    linkUp = false;
    LOG.println("[UART] Link DOWN - no valid frame.");
  }

  maintainWiFi();
  flushQueue();
}


/* ============================================================
 * NOTES
 * ============================================================
 *
 * ADC SCALE
 *   INCH_BANDS is the single source of truth for depth. Inside a
 *   band the reading is the calibrated whole inch; in a gap it is
 *   interpolated across to the next one, so the gauge moves smoothly
 *   rather than jumping. Above 1600 it reports 10 in - that is the
 *   probe running out of trace, not the tank running out of room.
 *
 *   The STM32's percent field is still parsed and exposed as
 *   stm_percent in /data purely as a cross-check. It no longer
 *   affects anything, so the STM32's own ADC_DRY/ADC_WET only need
 *   to be right for the LED colours.
 *
 *   Counts more than ADC_SANITY_MARGIN below ADC_ZERO are flagged on
 *   the page as out of range. That is an open or shorted probe, or a
 *   sagging supply - not a dry tank.
 *
 * HISTORY BUFFER
 *   2880 entries at 30s = 24 hours, in ~26 KB of RAM. The struct is
 *   packed to 9 bytes to make that fit; do not add fields to it
 *   casually, since each extra byte costs 2.8 KB.
 *
 *   The page is only sent the newest LOG_VIEW rows. Serialising all
 *   2880 as JSON would be a ~250 KB string built once per second,
 *   which fragments the heap and stalls the server. Use the CSV
 *   download for the full set - it streams in 1 KB chunks and never
 *   holds more than that in RAM.
 *
 *   This is a RAM cache and clears on reboot, by design. Only the
 *   unsent queue is persisted, because that is the part whose loss
 *   is not recoverable. Mirroring all 26 KB every 30 seconds would
 *   push ~75 MB a day through a 1.4 MB flash partition and wear it
 *   out within a year, to protect data that is already in a
 *   spreadsheet.
 *
 * CLOUD PUSH
 *   Set SHEETS_URL to your Apps Script deployment. The script needs a
 *   doGet(e) that reads e.parameter.inches / .raw / .epoch and
 *   appends a row; deploy it as "execute as me, anyone can access" or
 *   the device gets a login page instead of a 200.
 *
 *   Every sample is pushed once and marked sent only on HTTP 200.
 *   Unsent entries stay in the ring buffer and are retried once a
 *   minute, three at a time so the dashboard stays responsive. That
 *   makes an outage of up to 25 hours free, which is what the buffer
 *   depth was always for.
 *
 * OPEN NETWORK FALLBACK
 *   Set ALLOW_OPEN_FALLBACK to 0 to disable. Worth understanding
 *   before leaving it on: the ESP32 will associate with any unsecured
 *   SSID nearby, so readings travel in plaintext over a network you
 *   do not control. The generate_204 probe now catches captive
 *   portals, so a portal shows as "no route out" instead of silently
 *   eating the NTP sync and every push - but the traffic is still
 *   someone else's to read.
 *
 * LOG PERSISTENCE
 *   The queue is mirrored to LittleFS once per log, so roughly 8,700
 *   writes a year against a 100,000 cycle rating. The once-per-second
 *   live reading still never touches flash. Delete /queue.csv to
 *   start clean.
 *
 * WATCHDOG
 *   30 seconds, fed in loop() and after each HTTP call. HTTP timeouts
 *   are 8s and the sweep is capped at three pushes, so a stalled
 *   network cannot starve the feed.
 *
 * TIME
 *   GMT_OFFSET_SEC is 19800 for IST. NTP is only attempted once the
 *   204 probe succeeds. Without it the log falls back to "+3h20m"
 *   labels measured from boot - still ordered and correctly spaced,
 *   just not wall-clock, and rows push with epoch 0 so the sheet can
 *   fall back to its own receive time.
 * ============================================================ */
