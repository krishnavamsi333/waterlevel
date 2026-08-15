/* ============================================================
 * ESP32 - water level receiver + dashboard
 * ============================================================
 * Reads $WL,<raw>,<percent>*<xor> frames from the STM32 on UART2,
 * converts to inches, keeps 25 hourly samples, and serves a live
 * page on the local network.
 *
 * Wiring (as verified on this build - do not "correct" it):
 *   STM32 PA9  -> ESP32 GPIO17
 *   STM32 PA10 -> ESP32 GPIO16
 *   GND        -> GND            <-- required, not optional
 *
 * Swap RXD2/TXD2 below if your jumpers are the other way round.
 * ============================================================ */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <time.h>

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

/* ---------------- time ---------------- */
/* IST = UTC+5:30 = 19800 seconds. Change for your timezone. */
#define GMT_OFFSET_SEC        19800
#define DST_OFFSET_SEC        0

/* ---------------- UART ---------------- */
#define RXD2                  16
#define TXD2                  17
#define STM_BAUD              115200
#define LINK_TIMEOUT_MS       5000

/* ---------------- depth scale ----------------
 * The STM32 sends percent of its calibrated span. This converts to
 * inches. PROBE_FULL_INCHES must equal the depth at which ADC_WET
 * was measured on the STM32 - if you calibrated ADC_WET at 6 inches
 * of submersion, put 6.0 here, not 10.0, or every reading is wrong
 * by that ratio. */
#define PROBE_FULL_INCHES     10.0f

/* ---------------- logging ---------------- */
#define LOG_SIZE              25          /* samples retained      */
#define LOG_INTERVAL_MS       3600000UL   /* one hour              */

HardwareSerial STMSerial(2);
WebServer      server(80);

/* ---------------- live state ---------------- */
float         waterPercent = 0.0f;
float         waterInches  = 0.0f;
uint16_t      waterRaw     = 0;
unsigned long lastGoodMs   = 0;
bool          linkUp       = false;
bool          probeFault   = false;
uint32_t      frameCount   = 0;
uint32_t      badFrames    = 0;

String        netName      = "";
bool          netIsOpen    = false;
bool          timeSynced   = false;

/* ---------------- log ring buffer ----------------
 * Fixed 25 slots in RAM. When full, the oldest entry is overwritten,
 * so the buffer always holds the most recent 25 hours.
 *
 * RAM, deliberately - not flash. At one write per hour NVS would
 * survive fine, but there is no reason to spend erase cycles on data
 * that is about to go to a cloud endpoint anyway. The cost is that
 * the log clears on reboot. */
struct LogEntry {
  time_t   epoch;      /* 0 if clock was never synced */
  uint32_t uptimeSec;
  float    inches;
  uint16_t raw;
  bool     fault;
};

LogEntry      logBuf[LOG_SIZE];
uint8_t       logHead   = 0;     /* next slot to write */
uint8_t       logCount  = 0;
unsigned long lastLogMs = 0;
bool          firstLogDone = false;


/* ============================================================
 * Logging
 * ============================================================ */

void addLogEntry()
{
  LogEntry &e = logBuf[logHead];

  e.epoch     = timeSynced ? time(nullptr) : 0;
  e.uptimeSec = millis() / 1000UL;
  e.inches    = waterInches;
  e.raw       = waterRaw;
  e.fault     = probeFault;

  logHead = (logHead + 1) % LOG_SIZE;
  if (logCount < LOG_SIZE) logCount++;

  lastLogMs = millis();

  Serial.printf("logged: %.1f in  (%u entries held)\n", e.inches, logCount);
}


/* Newest first, since that is the order you read a log in. */
String logAsJson()
{
  String out = "[";

  for (uint8_t i = 0; i < logCount; i++)
  {
    /* Walk backwards from the most recently written slot. */
    int idx = (int) logHead - 1 - (int) i;
    while (idx < 0) idx += LOG_SIZE;

    const LogEntry &e = logBuf[idx];

    char label[24];
    if (e.epoch > 1600000000)
    {
      struct tm tmv;
      localtime_r(&e.epoch, &tmv);
      strftime(label, sizeof(label), "%d %b %H:%M", &tmv);
    }
    else
    {
      /* No clock: fall back to time since boot, which is still
       * ordered and still shows the spacing. */
      snprintf(label, sizeof(label), "+%luh%02lum",
               (unsigned long)(e.uptimeSec / 3600UL),
               (unsigned long)((e.uptimeSec % 3600UL) / 60UL));
    }

    char item[96];
    snprintf(item, sizeof(item),
             "%s{\"t\":\"%s\",\"in\":%.1f,\"raw\":%u,\"f\":%s}",
             (i == 0 ? "" : ","), label, e.inches,
             (unsigned) e.raw, e.fault ? "true" : "false");
    out += item;
  }

  out += "]";
  return out;
}


/* ============================================================
 * Frame parsing
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
  Serial.printf("Joining \"%s\" ", ssid);

  if (pass && pass[0]) WiFi.begin(ssid, pass);
  else                 WiFi.begin(ssid);

  for (uint8_t i = 0; i < tries && WiFi.status() != WL_CONNECTED; i++)
  {
    delay(500);
    Serial.print('.');
  }
  Serial.println();

  return WiFi.status() == WL_CONNECTED;
}


/* Scan and join the OPEN network with the strongest signal.
 * Strongest, not first found - a weak open network several rooms away
 * will associate and then drop packets, which is harder to diagnose
 * than simply failing to connect. */
bool joinStrongestOpen()
{
  Serial.println("Scanning for open networks...");

  int n = WiFi.scanNetworks();
  if (n <= 0) { Serial.println("No networks found."); return false; }

  int best = -1, bestRssi = -1000;

  for (int i = 0; i < n; i++)
  {
    bool isOpen = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
    Serial.printf("  %-24s %4d dBm %s\n",
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
    Serial.println("No open network available.");
    WiFi.scanDelete();
    return false;
  }

  String ssid = WiFi.SSID(best);
  Serial.printf("Strongest open network: \"%s\" at %d dBm\n",
                ssid.c_str(), bestRssi);
  WiFi.scanDelete();

  if (joinNetwork(ssid.c_str(), nullptr, 24))
  {
    netIsOpen = true;
    return true;
  }
  return false;
}


void startWiFi()
{
  WiFi.mode(WIFI_STA);

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
    Serial.print("Connected. Open  http://");
    Serial.println(WiFi.localIP());

    if (MDNS.begin("waterlevel"))
      Serial.println("Also at   http://waterlevel.local");

    /* Timestamps for the log. Without this the log falls back to
     * time-since-boot, which is still ordered but not wall-clock. */
    configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC,
               "pool.ntp.org", "time.nist.gov");

    Serial.print("Syncing clock ");
    for (uint8_t i = 0; i < 20 && !timeSynced; i++)
    {
      delay(500);
      Serial.print('.');
      if (time(nullptr) > 1600000000) timeSynced = true;
    }
    Serial.println(timeSynced ? " done." : " no NTP, using uptime.");
  }
  else
  {
    Serial.println("No network joinable. Starting access point.");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    netName   = String(AP_SSID) + " (own AP)";
    netIsOpen = false;
    Serial.print("Join it, then open  http://");
    Serial.println(WiFi.softAPIP());
  }
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
.hint{font-size:11.5px;color:var(--ink-3)}
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
    </div>

    <div class="card">
      <h2>Hourly log &mdash; last 25</h2>
      <table>
        <thead>
          <tr><th>Time</th><th class="n">Depth</th><th class="n">Raw</th></tr>
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
      </div>

      <div class="foot">
        <span class="hint">Samples every hour. Oldest drops off at 25.</span>
        <button id="now">Log a sample now</button>
      </div>
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
    tr.innerHTML = '<td class="time">' + e.t + '</td>' +
      '<td class="n">' + (e.f ? 'fault' : e.in.toFixed(1) + ' in') + '</td>' +
      '<td class="n">' + (e.f ? '\u2014' : e.raw) + '</td>';
    rows.appendChild(tr);
  }
}

async function tick() {
  try {
    const d = await (await fetch('/data', {cache:'no-store'})).json();

    document.getElementById('net').textContent =
      d.net + (d.open ? ' \u00b7 open network' : '');
    document.getElementById('clock').textContent = d.time || '';
    document.getElementById('held').textContent = d.held + ' / 25';
    document.getElementById('ok').textContent   = d.frames;
    document.getElementById('bad').textContent  = d.bad;
    document.getElementById('raw').textContent  = d.fault ? '\u2014' : d.raw;
    document.getElementById('age').textContent  =
      !d.link ? '\u2014' : (d.age_ms < 2000 ? 'just now'
                          : Math.round(d.age_ms/1000) + 's ago');

    const linkEl = document.getElementById('link');
    const fill = document.getElementById('fill');
    const inEl = document.getElementById('inches');
    const stEl = document.getElementById('state');

    drawLog(d.log);

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

document.getElementById('now').addEventListener('click', async () => {
  await fetch('/lognow');
  tick();
});

tick();
setInterval(tick, 1000);
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

  char head[360];
  snprintf(head, sizeof(head),
           "{\"inches\":%.1f,\"percent\":%.1f,\"raw\":%u,"
           "\"link\":%s,\"fault\":%s,\"frames\":%u,\"bad\":%u,"
           "\"age_ms\":%lu,\"held\":%u,\"net\":\"%s\",\"open\":%s,"
           "\"time\":\"%s\",\"log\":",
           waterInches, waterPercent, (unsigned) waterRaw,
           up ? "true" : "false",
           probeFault ? "true" : "false",
           (unsigned) frameCount, (unsigned) badFrames,
           up ? age : 0UL,
           (unsigned) logCount,
           netName.c_str(),
           netIsOpen ? "true" : "false",
           clockStr);

  String json = String(head) + logAsJson() + "}";

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


void setup()
{
  Serial.begin(115200);
  STMSerial.begin(STM_BAUD, SERIAL_8N1, RXD2, TXD2);
  STMSerial.setTimeout(50);

  Serial.println("\nWater level monitor starting.");

  startWiFi();

  server.on("/",       handleRoot);
  server.on("/data",   handleData);
  server.on("/lognow", handleLogNow);
  server.begin();
  Serial.println("Dashboard running.");
}


void loop()
{
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
        waterPercent = pct;
        waterInches  = pct * PROBE_FULL_INCHES / 100.0f;
      }

      if (!linkUp) { linkUp = true; Serial.println("Link up."); }

      if (fault) Serial.println("PROBE FAULT reported by STM32");
      else       Serial.printf("raw=%4u  %.1f in\n", waterRaw, waterInches);

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
      Serial.print("bad frame: ");
      Serial.println(line);
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
    Serial.println("Link DOWN - no valid frame from STM32.");
  }
}


/* ============================================================
 * NOTES
 * ============================================================
 *
 * INCHES
 *   PROBE_FULL_INCHES must match the depth at which ADC_WET was
 *   measured on the STM32. The STM32 reports percent of its
 *   calibrated span, so if ADC_WET was taken at 6 inches while this
 *   says 10.0, every reading is inflated by 1.67x. Calibrating at
 *   exactly 10 inches is the simplest way to keep the two in step.
 *
 *   Be aware that these resistive probes have only a few inches of
 *   sensing trace. A 0-10 inch reading assumes you have mounted the
 *   probe so its span covers the range you care about, and that you
 *   accept the scale as indicative. It is not a 0.1 inch accurate
 *   instrument, whatever the display resolution suggests.
 *
 * OPEN NETWORK FALLBACK
 *   Set ALLOW_OPEN_FALLBACK to 0 to disable. Worth understanding
 *   before leaving it on: the ESP32 will associate with any
 *   unsecured SSID nearby, so readings travel in plaintext over a
 *   network you do not control, and a captive portal will silently
 *   swallow the NTP sync and any future cloud push while still
 *   reporting "connected". It picks the strongest open network rather
 *   than the first found, because a marginal association that drops
 *   packets is harder to diagnose than a clean failure.
 *
 * WHAT THE CLOUD PUSH WILL NEED
 *   addLogEntry() is the hourly hook. When you have an endpoint, POST
 *   from there and keep this ring buffer as the retry queue: add a
 *   "sent" flag per entry and leave it unset on a failed POST so the
 *   next attempt re-sends. A network outage then costs you nothing up
 *   to 25 hours long, which is exactly what the buffer depth buys.
 *
 * LOG PERSISTENCE
 *   The buffer is RAM, so it clears on reboot. If it must survive
 *   power cuts, NVS at one write per hour is fine for wear - roughly
 *   8,700 writes a year against a 100,000 cycle rating. Do not be
 *   tempted to persist the once-per-second live reading.
 *
 * TIME
 *   GMT_OFFSET_SEC is 19800 for IST. Without internet the log falls
 *   back to "+3h20m" labels measured from boot - still ordered and
 *   correctly spaced, just not wall-clock.
 * ============================================================ */
