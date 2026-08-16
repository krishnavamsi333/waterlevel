/* ============================================================
 * ASTRA - Water Level Monitor (ESP32)
 * ============================================================
 * Reads $WL,<raw>,<percent>*<xor> frames from the STM32 on UART2,
 * serves a live dashboard on the LAN, and publishes to MQTT.
 *
 * WiFi policy:
 *   1. Try the configured SSID + password first.
 *   2. If that fails, scan and join the strongest OPEN network.
 *   3. If that fails too, start our own access point.
 *   No reset needed at any step; the ladder re-runs on its own.
 *
 * MQTT topics (subscribe to these):
 *   astra/monitor01/water    <- readings, JSON, retained
 *   astra/monitor01/status   <- "online" / "offline", retained
 *   astra/monitor01/cmd      -> send "now" to force a publish
 *
 * Wiring:
 *   STM32 PA9  (TX) -> ESP32 GPIO16 (RX2)
 *   STM32 PA10 (RX) -> ESP32 GPIO17 (TX2)
 *   GND             -> GND          <-- required, not optional
 *
 * Libraries: all bundled with the ESP32 Arduino core EXCEPT
 * PubSubClient (Nick O'Leary) - install via Library Manager.
 * ============================================================ */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>
#include <PubSubClient.h>

/* ---------------- WiFi: preferred network ---------------- */
/* Tried first, every time. Case sensitive and must be exact. The
 * ESP32 radio is 2.4 GHz only - it cannot see a 5 GHz network at all,
 * which is the usual "it will not connect" cause when the phone next
 * to it works fine. Leave WIFI_PASS as "" if this network is open. */
const char *WIFI_SSID = "esp";
const char *WIFI_PASS = "12345678";

#define PRIMARY_TIMEOUT_MS  15000   /* wait on the preferred network  */
#define OPEN_TIMEOUT_MS     10000   /* wait per open candidate        */
#define MAX_CANDIDATES      12      /* open APs remembered per scan   */
#define RETRY_INTERVAL_MS   45000   /* gap between reconnect attempts */

/* Require a working internet path before accepting an open network.
 * Rejects captive portals, which would otherwise associate fine and
 * then silently break MQTT. Set false if your open network is
 * deliberately LAN-only. */
#define CHECK_PORTAL        true
#define PORTAL_URL          "http://clients3.google.com/generate_204"

#define AP_SSID   "Astra-Monitor-AP"
#define AP_PASS   "12345678"

/* ---------------- MQTT ---------------- */
/* Port 1883 is plain MQTT over TCP - what the ESP32 speaks. A browser
 * client on the same broker uses 8083/8084 (WebSocket). Both see the
 * same topics; the broker joins them. */
#define MQTT_HOST        "broker.emqx.io"
#define MQTT_PORT        1883
#define MQTT_CLIENT_ID   "astra_monitor_03"
#define MQTT_USER        "astra"
#define MQTT_PASSWORD    "astra"

#define TOPIC_DATA       "astra/monitor01/water"
#define TOPIC_STATUS     "astra/monitor01/status"
#define TOPIC_CMD        "astra/monitor01/cmd"

#define MQTT_PUBLISH_MS  2000
#define MQTT_RETRY_MS    5000
#define MQTT_KEEPALIVE_S 60

/* ---------------- UART ---------------- */
#define RXD2 16
#define TXD2 17
#define STM_BAUD 115200
/* Treat the link as down after this long with no valid frame, so a
 * dead STM32 shows as a fault instead of a frozen reading. */
#define LINK_TIMEOUT_MS 5000

HardwareSerial STMSerial(2);        /* UART2; UART0 is the USB console */
WebServer      server(80);
WiFiClient     mqttNet;
PubSubClient   mqtt(mqttNet);

/* ---------------- state ---------------- */
float         waterPercent = 0.0f;
uint16_t      waterRaw     = 0;
unsigned long lastGoodMs   = 0;
bool          linkUp       = false;
bool          probeFault   = false;
uint32_t      frameCount   = 0;
uint32_t      badFrames    = 0;

String        joinedSsid   = "";
bool          joinedIsOpen = false;
bool          apMode       = false;
unsigned long lastWifiTry  = 0;

/* True only once server.begin() has actually run. Calling
 * handleClient() before that touches an lwIP socket that does not
 * exist yet, which panics rather than failing politely. */
bool          serverUp     = false;

unsigned long lastPubMs    = 0;
unsigned long lastMqttTry  = 0;
uint32_t      pubCount     = 0;
bool          forcePublish = false;

struct OpenNet { String ssid; int32_t rssi; };
OpenNet candidates[MAX_CANDIDATES];
int     candidateCount = 0;

void pumpUart();   /* forward declaration - used inside blocking waits */


/* ============================================================
 * Logging
 * ============================================================
 * Everything goes through one place so the console stays readable and
 * the tag tells you which subsystem spoke.
 */
void logf(const char *tag, const char *fmt, ...)
{
  char body[224];
  va_list args;
  va_start(args, fmt);
  vsnprintf(body, sizeof(body), fmt, args);
  va_end(args);

  Serial.printf("[%s] %s\n", tag, body);
}


/* ============================================================
 * Frame parsing
 * ============================================================ */

/* Verify the XOR checksum and extract the fields. Returns false for
 * anything malformed, so line noise can never be mistaken for a
 * reading. Sets fault=true for the $WL,ERR frame the STM32 sends when
 * the probe looks disconnected. */
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
 * WiFi - preferred network, then best open network, then AP
 * ============================================================ */

/* Captive-portal test. The URL returns HTTP 204 with an empty body on
 * a clean connection. A portal answers 200 with a login page or a 302
 * redirect, so anything but 204 means traffic is being intercepted. */
bool portalFree()
{
  HTTPClient http;
  http.setConnectTimeout(4000);
  http.setTimeout(4000);
  http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);

  if (!http.begin(PORTAL_URL)) { logf("WiFi", "  Portal check failed to start."); return false; }
  int code = http.GET();
  http.end();

  logf("WiFi", "  Portal check: HTTP %d - %s", code,
       (code == 204) ? "clear" : "intercepted");
  return (code == 204);
}


/* Blocking wait that keeps the UART drained and the web server
 * answering, so a slow association costs neither STM32 frames nor a
 * frozen dashboard. */
bool waitForAssoc(unsigned long timeoutMs)
{
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs)
  {
    pumpUart();
    if (serverUp) server.handleClient();
    delay(100);
  }
  return WiFi.status() == WL_CONNECTED;
}


bool tryPrimaryNetwork()
{
  if (strlen(WIFI_SSID) == 0) return false;

  logf("WiFi", "Trying: \"%s\" (%s)", WIFI_SSID,
       strlen(WIFI_PASS) ? "secured" : "open");

  WiFi.disconnect(true);
  delay(100);

  /* One-argument begin() is the open-network form. Passing an empty
   * key makes some core versions attempt a WPA2 handshake that an open
   * AP will never complete. */
  if (strlen(WIFI_PASS) == 0) WiFi.begin(WIFI_SSID);
  else                        WiFi.begin(WIFI_SSID, WIFI_PASS);

  if (!waitForAssoc(PRIMARY_TIMEOUT_MS))
  {
    /* 6 = SSID never seen (typo, out of range, or 5 GHz only)
     * 4 = seen but the handshake failed, usually a wrong password */
    logf("WiFi", "Timed out. Status=%d", WiFi.status());
    return false;
  }

  joinedSsid   = WIFI_SSID;
  joinedIsOpen = (strlen(WIFI_PASS) == 0);
  apMode       = false;

  logf("WiFi", "Associated. IP=%s RSSI=%d", WiFi.localIP().toString().c_str(),
       (int) WiFi.RSSI());
  logf("WiFi", "Success via preferred network: %s", WIFI_SSID);
  return true;
}


int scanOpenNetworks()
{
  candidateCount = 0;

  logf("WiFi", "Scanning for open networks...");
  int n = WiFi.scanNetworks(false, false);

  if (n <= 0) { logf("WiFi", "No networks found."); return 0; }

  logf("WiFi", "Found %d network(s):", n);
  for (int i = 0; i < n; i++)
  {
    bool open = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
    logf("WiFi", "  [%d] \"%s\" RSSI=%d %s", i, WiFi.SSID(i).c_str(),
         (int) WiFi.RSSI(i), open ? "OPEN" : "secured");

    if (!open || WiFi.SSID(i).length() == 0) continue;
    if (candidateCount >= MAX_CANDIDATES) continue;

    bool dupe = false;                       /* same AP on several channels */
    for (int j = 0; j < candidateCount; j++)
      if (candidates[j].ssid == WiFi.SSID(i)) { dupe = true; break; }
    if (dupe) continue;

    candidates[candidateCount].ssid = WiFi.SSID(i);
    candidates[candidateCount].rssi = WiFi.RSSI(i);
    candidateCount++;
  }

  WiFi.scanDelete();

  /* strongest first - insertion sort, the list is tiny */
  for (int i = 1; i < candidateCount; i++)
  {
    OpenNet key = candidates[i];
    int j = i - 1;
    while (j >= 0 && candidates[j].rssi < key.rssi) { candidates[j + 1] = candidates[j]; j--; }
    candidates[j + 1] = key;
  }

  return candidateCount;
}


bool tryOpenNetworks()
{
  if (scanOpenNetworks() == 0) { logf("WiFi", "No open networks to try."); return false; }

  for (int i = 0; i < candidateCount; i++)
  {
    logf("WiFi", "Trying: \"%s\" (open, %d dBm)",
         candidates[i].ssid.c_str(), (int) candidates[i].rssi);

    WiFi.disconnect(true);
    delay(100);
    WiFi.begin(candidates[i].ssid.c_str());

    if (!waitForAssoc(OPEN_TIMEOUT_MS))
    {
      logf("WiFi", "  No association. Status=%d", WiFi.status());
      continue;
    }

    logf("WiFi", "Associated. IP=%s - checking internet...",
         WiFi.localIP().toString().c_str());

#if CHECK_PORTAL
    if (!portalFree())
    {
      /* The LAN page would still work here, but MQTT will not: portals
       * block outbound 1883. Keep looking for a clean network. */
      logf("WiFi", "  Captive portal - trying next.");
      continue;
    }
    logf("WiFi", "Internet OK via: %s", candidates[i].ssid.c_str());
#endif

    joinedSsid   = candidates[i].ssid;
    joinedIsOpen = true;
    apMode       = false;
    logf("WiFi", "Success. Using open network: %s", joinedSsid.c_str());
    return true;
  }

  logf("WiFi", "All open candidates exhausted.");
  return false;
}


void startFallbackAP()
{
  /* No router in range, or nothing usable. Serve the dashboard from our
   * own AP so the device is never unreachable on site. */
  logf("WiFi", "Nothing usable. Starting access point.");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.softAP(AP_SSID, AP_PASS);

  joinedSsid   = AP_SSID;
  joinedIsOpen = false;
  apMode       = true;

  logf("WiFi", "AP up. Join \"%s\" then open http://%s",
       AP_SSID, WiFi.softAPIP().toString().c_str());
}


/* The full ladder. Runs at boot and again whenever the link drops. */
void connectWiFi()
{
  lastWifiTry = millis();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          /* stops the radio dozing mid-poll */
  WiFi.setAutoReconnect(true);

  if (tryPrimaryNetwork() || tryOpenNetworks())
  {
    if (MDNS.begin("astra")) logf("WEB", "Also at http://astra.local");
    logf("WEB", "Dashboard: http://%s", WiFi.localIP().toString().c_str());
    return;
  }

  startFallbackAP();
}


/* Watchdog for loop(). Only re-runs the ladder after the retry
 * interval, so a dead network does not become a scan loop. */
void maintainWiFi()
{
  if (apMode) return;                          /* AP has nothing to lose */
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWifiTry < RETRY_INTERVAL_MS) return;

  logf("WiFi", "Link lost. Re-running connection ladder.");
  connectWiFi();
}


/* ============================================================
 * MQTT
 * ============================================================ */

void mqttCallback(char *topic, byte *payload, unsigned int len)
{
  String msg;
  msg.reserve(len);
  for (unsigned int i = 0; i < len; i++) msg += (char) payload[i];

  logf("MQTT", "RX %s: %s", topic, msg.c_str());

  /* "now" forces an immediate publish instead of waiting out the
   * interval - handy when a dashboard first loads. */
  if (msg == "now") { forcePublish = true; logf("MQTT", "Immediate publish requested."); }
}


bool mqttConnect()
{
  if (WiFi.status() != WL_CONNECTED) return false;   /* AP mode has no uplink */

  logf("MQTT", "Connecting to %s:%d as \"%s\" (user=%s)...",
       MQTT_HOST, MQTT_PORT, MQTT_CLIENT_ID, MQTT_USER);

  /* Last will: if we drop off, the broker publishes "offline" for us so
   * subscribers grey out instead of trusting a stale reading. */
  bool ok = mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD,
                         TOPIC_STATUS, 1, true, "offline");

  if (!ok)
  {
    /* -4 timeout, -2 TCP refused (DNS/host/port/portal), -1 clean
     *  disconnect, 4 bad credentials, 5 not authorised */
    logf("MQTT", "Connect failed. state=%d", mqtt.state());
    return false;
  }

  logf("MQTT", "Connected.");
  mqtt.publish(TOPIC_STATUS, "online", true);
  mqtt.subscribe(TOPIC_CMD);
  logf("MQTT", "Publishing to  %s", TOPIC_DATA);
  logf("MQTT", "Subscribed to  %s", TOPIC_CMD);
  return true;
}


bool mqttPublishReading()
{
  if (!mqtt.connected()) return false;

  bool up = linkUp && (millis() - lastGoodMs <= LINK_TIMEOUT_MS);

  char payload[288];
  snprintf(payload, sizeof(payload),
           "{\"percent\":%.1f,\"raw\":%u,\"fault\":%s,\"link\":%s,"
           "\"frames\":%u,\"bad\":%u,\"ssid\":\"%s\",\"rssi\":%d,\"uptime\":%lu}",
           waterPercent, (unsigned) waterRaw,
           probeFault ? "true" : "false",
           up ? "true" : "false",
           (unsigned) frameCount, (unsigned) badFrames,
           joinedSsid.c_str(), (int) WiFi.RSSI(),
           (unsigned long)(millis() / 1000));

  /* Retained, so a subscriber connecting at any moment gets the last
   * reading straight away rather than nothing until the next tick. */
  bool ok = mqtt.publish(TOPIC_DATA, payload, true);

  if (ok) { pubCount++; logf("MQTT", "TX #%u %s", (unsigned) pubCount, payload); }
  else    logf("MQTT", "Publish failed. state=%d", mqtt.state());

  return ok;
}


void mqttTick()
{
  if (WiFi.status() != WL_CONNECTED) return;

  if (!mqtt.connected())
  {
    if (millis() - lastMqttTry < MQTT_RETRY_MS) return;
    lastMqttTry = millis();
    if (!mqttConnect()) return;
  }

  mqtt.loop();      /* services keepalive and inbound - must run often */

  if (forcePublish || millis() - lastPubMs >= MQTT_PUBLISH_MS)
  {
    forcePublish = false;
    lastPubMs    = millis();
    mqttPublishReading();
  }
}


/* ============================================================
 * Web page
 * ============================================================
 * A static shell that polls /data once a second. Keeping the HTML
 * static and the data separate means the page does not reload or lose
 * scroll position on every update.
 *
 * Colours match the WS2812B zones on the STM32 so the LED and the page
 * always agree.
 */
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Astra Water Level</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    font-family: -apple-system, system-ui, "Segoe UI", Roboto, sans-serif;
    background: #12161c; color: #e8edf4;
    min-height: 100vh; display: flex; align-items: center;
    justify-content: center; padding: 20px;
  }
  .card {
    background: #1a1f27; border: 1px solid #2a3341; border-radius: 14px;
    padding: 28px; width: 100%; max-width: 380px;
  }
  h1 { font-size: 13px; font-weight: 600; letter-spacing: .12em;
       text-transform: uppercase; color: #7d8899; margin-bottom: 22px; }
  .reading { display: flex; align-items: baseline; gap: 6px; margin-bottom: 4px; }
  #pct { font-size: 62px; font-weight: 700; line-height: 1;
         font-variant-numeric: tabular-nums; transition: color .3s; }
  .unit { font-size: 22px; color: #7d8899; font-weight: 600; }
  #zone { font-size: 14px; color: #7d8899; margin-bottom: 20px; }
  .track { height: 14px; background: #0d1116; border-radius: 7px;
           overflow: hidden; margin-bottom: 22px; }
  #bar { height: 100%; width: 0%; border-radius: 7px;
         transition: width .5s ease, background .3s; }
  .rows { border-top: 1px solid #2a3341; padding-top: 16px;
          display: grid; gap: 9px; }
  .row { display: flex; justify-content: space-between; font-size: 13px; }
  .row span:first-child { color: #7d8899; }
  .row span:last-child { font-variant-numeric: tabular-nums; }
  .dot { display: inline-block; width: 8px; height: 8px;
         border-radius: 50%; margin-right: 6px; vertical-align: 1px; }
  .net { margin-top: 14px; padding-top: 14px; border-top: 1px solid #2a3341; }
</style>
</head>
<body>
  <div class="card">
    <h1>Astra Water Monitor</h1>
    <div class="reading">
      <span id="pct">--</span><span class="unit">%</span>
    </div>
    <div id="zone">waiting for data</div>
    <div class="track"><div id="bar"></div></div>
    <div class="rows">
      <div class="row"><span>Raw ADC</span><span id="raw">--</span></div>
      <div class="row"><span>Link</span><span id="link">--</span></div>
      <div class="row"><span>Frames OK</span><span id="ok">--</span></div>
      <div class="row"><span>Frames bad</span><span id="bad">--</span></div>
      <div class="row"><span>Last update</span><span id="age">--</span></div>
    </div>
    <div class="rows net">
      <div class="row"><span>Network</span><span id="ssid">--</span></div>
      <div class="row"><span>Signal</span><span id="rssi">--</span></div>
      <div class="row"><span>MQTT</span><span id="mqtt">--</span></div>
    </div>
  </div>

<script>
/* Same five bands as the WS2812B zones on the STM32. */
function band(p) {
  if (p <= 20) return ["#ff2d2d", "Critical - almost empty"];
  if (p <= 40) return ["#ff69b4", "Low"];
  if (p <= 60) return ["#00bfff", "Medium"];
  if (p <= 80) return ["#3b6cff", "Good"];
  return ["#22d65e", "Full"];
}
const $ = id => document.getElementById(id);

async function tick() {
  try {
    const r = await fetch('/data', { cache: 'no-store' });
    const d = await r.json();

    $('raw').textContent = d.fault ? 'n/a' : d.raw;
    $('ok').textContent  = d.frames;
    $('bad').textContent = d.bad;
    $('age').textContent = d.age_ms < 2000
      ? 'just now' : (d.age_ms / 1000).toFixed(0) + 's ago';

    $('ssid').textContent = d.ssid + (d.ap ? ' (AP)' : d.open ? ' (open)' : '');
    $('rssi').textContent = d.ap ? 'n/a' : d.rssi + ' dBm';
    $('mqtt').innerHTML = d.mqtt
      ? '<span class="dot" style="background:#22d65e"></span>connected'
      : '<span class="dot" style="background:#7d8899"></span>offline';

    if (!d.link) {
      $('pct').textContent = '--';
      $('pct').style.color = '#7d8899';
      $('zone').textContent = 'No data from STM32';
      $('bar').style.width = '0%';
      $('link').innerHTML = '<span class="dot" style="background:#ff2d2d"></span>down';
      return;
    }

    $('link').innerHTML = '<span class="dot" style="background:#22d65e"></span>up';

    if (d.fault) {
      $('pct').textContent = '!';
      $('pct').style.color = '#e8edf4';
      $('zone').textContent = 'Probe fault - check sensor wiring';
      $('bar').style.width = '100%';
      $('bar').style.background = '#8892a0';
      return;
    }

    const [colour, label] = band(d.percent);
    $('pct').textContent  = d.percent.toFixed(1);
    $('pct').style.color  = colour;
    $('zone').textContent = label;
    $('bar').style.width  = Math.max(0, Math.min(100, d.percent)) + '%';
    $('bar').style.background = colour;

  } catch (e) {
    $('link').innerHTML = '<span class="dot" style="background:#ffa724"></span>page offline';
  }
}

tick();
setInterval(tick, 1000);
</script>
</body>
</html>
)rawliteral";


void handleRoot()
{
  server.send_P(200, "text/html", PAGE_HTML);
}

void handleData()
{
  unsigned long age = millis() - lastGoodMs;
  bool up = linkUp && (age <= LINK_TIMEOUT_MS);

  char json[360];
  snprintf(json, sizeof(json),
           "{\"percent\":%.1f,\"raw\":%u,\"link\":%s,\"fault\":%s,"
           "\"frames\":%u,\"bad\":%u,\"age_ms\":%lu,"
           "\"ssid\":\"%s\",\"open\":%s,\"ap\":%s,\"rssi\":%d,\"mqtt\":%s}",
           waterPercent,
           (unsigned) waterRaw,
           up ? "true" : "false",
           probeFault ? "true" : "false",
           (unsigned) frameCount,
           (unsigned) badFrames,
           up ? age : 0UL,
           joinedSsid.c_str(),
           joinedIsOpen ? "true" : "false",
           apMode ? "true" : "false",
           apMode ? 0 : (int) WiFi.RSSI(),
           mqtt.connected() ? "true" : "false");

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}


/* ============================================================
 * UART pump - factored out so it can also run during WiFi waits
 * ============================================================ */
void pumpUart()
{
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
        waterPercent = pct;
      }

      if (!linkUp) { linkUp = true; logf("UART", "Link up. Frames arriving from STM32."); }

      if (fault) logf("Sensor", "FAILURE - probe fault reported by STM32.");
      else       logf("Sensor", "raw=%u level=%.1f%%", (unsigned) waterRaw, waterPercent);
    }
    else if (line.length() > 1)
    {
      badFrames++;
      /* Keep this during bring-up. Solid mojibake means a baud mismatch
       * or swapped TX/RX; readable text with a bad checksum means
       * electrical noise on the line. */
      logf("UART", "Bad frame (#%u): %s", (unsigned) badFrames, line.c_str());
    }
  }

  if (linkUp && (millis() - lastGoodMs > LINK_TIMEOUT_MS))
  {
    linkUp = false;
    logf("UART", "Link DOWN - no valid frame for %d ms.", LINK_TIMEOUT_MS);
  }
}


/* ============================================================
 * setup / loop
 * ============================================================ */
void setup()
{
  Serial.begin(115200);
  delay(200);
  STMSerial.begin(STM_BAUD, SERIAL_8N1, RXD2, TXD2);
  STMSerial.setTimeout(50);

  Serial.println();
  logf("System", "=== ASTRA Water Monitor ===");
  logf("System", "Build: %s %s", __DATE__, __TIME__);
  logf("System", "Free heap: %u bytes", (unsigned) ESP.getFreeHeap());
  logf("UART",   "UART2 up. RX=%d TX=%d @ %d baud", RXD2, TXD2, STM_BAUD);

  /* ORDER MATTERS HERE.
   * WiFi.mode() is what brings up the lwIP TCP/IP stack. WebServer's
   * begin() opens a socket, and doing that before the stack exists
   * fails an assert inside FreeRTOS rather than returning an error:
   *     assert failed: xQueueSemaphoreTake queue.c:1709 ((pxQueue))
   * followed by a reboot loop. Radio first, server second. */
  WiFi.mode(WIFI_STA);
  delay(100);
  logf("WiFi", "Radio initialised in station mode.");

  server.on("/",     handleRoot);
  server.on("/data", handleData);
  server.begin();
  serverUp = true;
  logf("WEB", "HTTP server started on port 80.");

  connectWiFi();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
  mqtt.setBufferSize(384);      /* default 256 is tight once fields grow */

  logf("MQTT", "Broker %s:%d  topic %s", MQTT_HOST, MQTT_PORT, TOPIC_DATA);
  logf("System", "Setup complete.");
}


void loop()
{
  if (serverUp) server.handleClient();
  pumpUart();
  maintainWiFi();
  mqttTick();
}


/* ============================================================
 * NOTES
 * ============================================================
 *
 * Boot order
 *   WiFi.mode() must run before server.begin(). Reversing them gives
 *   an xQueueSemaphoreTake assert and a reboot loop, because the
 *   socket layer does not exist until the radio is initialised. The
 *   serverUp flag guards handleClient() for the same reason - it is
 *   called from inside waitForAssoc(), which can run early.
 *
 * Topics
 *   astra/monitor01/water   readings, retained JSON
 *   astra/monitor01/status  "online" / "offline", retained, LWT
 *   astra/monitor01/cmd     publish "now" to force an immediate send
 *
 *   Subscribe to astra/monitor01/# to see all three at once.
 *
 * Credentials on a public broker
 *   broker.emqx.io accepts anonymous connections, so astra/astra is
 *   sent and accepted but is not actually protecting anything. Anyone
 *   who knows the topic can read your data and publish to your cmd
 *   topic. For real use, run Mosquitto or take a free EMQX/HiveMQ
 *   Cloud tier where those credentials mean something.
 *
 * Fixed client ID
 *   astra_monitor_01 matches your desktop client config. Two clients
 *   sharing an ID get kicked in a loop by the broker, so give the
 *   desktop tool a different one - if you see rapid connect and
 *   disconnect cycles, that is the cause.
 *
 * Connection order
 *   Preferred SSID -> strongest open network -> own access point. The
 *   ladder re-runs every RETRY_INTERVAL_MS while the link is down, no
 *   reset required. Once the fallback AP is up the device stays there
 *   deliberately: hopping back mid-session would drop whoever is
 *   looking at the dashboard. Power-cycle to start the ladder again.
 *
 * Auto-joining open networks
 *   The device will associate with any open SSID, including one named
 *   to look inviting by someone else. Traffic is unencrypted over the
 *   air. Water readings are harmless, but do not put credentials in
 *   the MQTT payload while this is on. For a deployment, filter the
 *   candidates against an allowlist instead of taking the strongest.
 *
 * GPIO16/17 conflict
 *   On ESP32-WROVER modules those pins are wired to external PSRAM and
 *   unusable. The DOIT DEVKIT V1 is a WROOM board, so 16/17 are fine.
 *   If you move to a WROVER:
 *       #define RXD2 25
 *       #define TXD2 26
 *   Avoid GPIO 6-11 (SPI flash), 34-39 (input only, so no TX), and
 *   0/2/12/15 (strapping pins that affect boot).
 *
 * Mojibake in the console
 *   A run of garbage characters means something is transmitting at a
 *   different baud than STM_BAUD, or TX and RX are swapped. Readable
 *   text with a failing checksum means electrical noise instead.
 *
 * If MQTT never connects
 *   state=-2 with WiFi up is almost always a captive portal or a
 *   firewall blocking outbound 1883. CHECK_PORTAL rejects those
 *   networks during the scan so the failure surfaces early.
 * ============================================================ */
