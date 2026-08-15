/* ============================================================
 * STM32F103C8T6 (Blue Pill) - Arduino IDE / STM32duino
 * Water depth sensor -> WS2812B indicator + UART frames to ESP32
 * ============================================================
 * Sends once per second:   $WL,<raw>,<percent>*<xor>\r\n
 * On probe fault:          $WL,ERR,0.0*<xor>\r\n
 *
 * Pairs with esp32_water_level_web.ino
 * ============================================================
 * BOARD SETTINGS (Tools menu)
 *   Board                Generic STM32F1 series
 *   Board part number    BluePill F103C8
 *   Upload method        STM32CubeProgrammer (SWD)
 *   USB support          None
 *   U(S)ART support      Enabled (generic Serial)
 *   Optimize             Smallest (-Os)
 *
 * LIBRARY
 *   Adafruit NeoPixel  (Library Manager)
 *
 * Leave USB support at None. USB CDC on these boards usually fails
 * anyway because the clones fit a 10k pull-up on D+ where the spec
 * needs 1.5k, and you do not need it: the ESP32 already prints every
 * frame to its own USB console, so that is your debug window.
 * ============================================================ */

#include <Adafruit_NeoPixel.h>

/* ---------------- pins ---------------- */
#define SENSOR_PIN        PA0     /* probe signal, analog in       */
#define SENSOR_PWR        PA1     /* probe VCC, driven as GPIO     */
#define LED_PIN           PA6     /* WS2812B DIN                   */
#define LED_COUNT         1

/* ---------------- timing ---------------- */
#define ADC_SAMPLES       16      /* conversions averaged per read */
#define SETTLE_MS         50      /* probe + 100nF cap charge time */
#define PERIOD_MS         1000    /* frame interval                */

/* ---------------- calibration ----------------
 * Measure these on YOUR probe, at 12-bit resolution.
 * ADC_DRY = averaged raw with the probe completely dry.
 * ADC_WET = averaged raw at your maximum useful depth.
 * See the calibration procedure at the bottom. */
#define ADC_DRY           400
#define ADC_WET           1500

/* A reading this far below the dry baseline means the signal wire is
 * open or the probe has come loose. Tune to your measured ADC_DRY. */
#define ADC_OPEN_MARGIN   370

/* Hysteresis on the colour thresholds, in tenths of a percent.
 * 30 = 3.0%. Without it the LED strobes between two colours whenever
 * the level sits near a boundary, because a few LSB of ADC noise is
 * enough to cross it on every sample. */
#define ZONE_HYST         30

/* LED brightness 0-255. A WS2812B at full white draws ~60 mA, which
 * is a lot to pull through the Blue Pill's regulator and is painfully
 * bright as an indicator. */
#define LED_BRIGHTNESS    60

/* Core 3.0.0 removed the HardwareSerial(rx, tx) constructor.
 * On the Generic F103C8 variant, Serial is already USART1 on
 * PA9/PA10 — exactly the pins we want — so bind a reference to it
 * and the rest of the sketch is unchanged. */
auto &espSerial = Serial;
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

/* ---------------- state ---------------- */
uint8_t ledZone = 0xFF;           /* 0xFF = nothing shown yet
                                   * 0xFE = fault indication        */


/* ============================================================
 * Sensor
 * ============================================================ */

/* Power the probe, let it settle, average ADC_SAMPLES conversions,
 * then cut power.
 *
 * The power gating is the point. A resistive probe held at a constant
 * DC potential in water electrolyses: the copper traces corrode,
 * readings drift upward over days, then it stops working. Energising
 * it only while sampling keeps the duty cycle near 5% here and takes
 * the probe from weeks to months.
 *
 * The first conversion after power-on is discarded because it tends
 * to sit low while the input settles. */
uint16_t readSensor()
{
  uint32_t sum = 0;

  digitalWrite(SENSOR_PWR, HIGH);
  delay(SETTLE_MS);

  (void) analogRead(SENSOR_PIN);            /* throwaway */

  for (uint8_t i = 0; i < ADC_SAMPLES; i++)
  {
    sum += analogRead(SENSOR_PIN);
    delayMicroseconds(200);
  }

  digitalWrite(SENSOR_PWR, LOW);

  return (uint16_t)(sum / ADC_SAMPLES);
}


/* Map raw ADC to 0..1000, i.e. percent in tenths.
 * Integer maths throughout - see the note about %f at the bottom. */
uint16_t percentTenths(uint16_t raw)
{
  int32_t span = (int32_t) ADC_WET - (int32_t) ADC_DRY;
  if (span == 0) return 0;                  /* bad calibration guard */

  int32_t t = ((int32_t) raw - (int32_t) ADC_DRY) * 1000 / span;

  if (t < 0)    t = 0;
  if (t > 1000) t = 1000;

  return (uint16_t) t;
}


/* ============================================================
 * LED
 * ============================================================ */

/* Classify into a colour zone with hysteresis, so the LED does not
 * chatter at a threshold. Only adjacent transitions are damped; a
 * two-zone jump is a real change and shows immediately. */
uint8_t zoneOf(uint16_t tenths, uint8_t prev)
{
  static const int16_t th[4] = { 200, 400, 600, 800 };
  int16_t t = (int16_t) tenths;
  uint8_t z = 0;

  for (uint8_t i = 0; i < 4; i++)
  {
    if (t > th[i]) z = i + 1;
  }

  if (prev <= 4 && z != prev)
  {
    if (z == prev + 1)
    {
      if (t < th[prev] + ZONE_HYST) z = prev;
    }
    else if (prev == z + 1)
    {
      if (t > th[z] - ZONE_HYST) z = prev;
    }
  }

  return z;
}


void showColor(uint8_t r, uint8_t g, uint8_t b)
{
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}


/* Update only on a zone change. Adafruit_NeoPixel bit-bangs the data
 * line with interrupts disabled, so there is no reason to redrive the
 * same colour every second. */
void ledUpdate(uint16_t tenths)
{
  uint8_t z = zoneOf(tenths, ledZone);

  if (z == ledZone) return;
  ledZone = z;

  switch (z)
  {
    case 0:  showColor(255,   0,   0); break;   /*  0-20  red      */
    case 1:  showColor(255, 105, 180); break;   /* 20-40  pink     */
    case 2:  showColor(  0, 191, 255); break;   /* 40-60  sky blue */
    case 3:  showColor(  0,   0, 255); break;   /* 60-80  blue     */
    default: showColor(  0, 255,   0); break;   /* 80-100 green    */
  }
}


/* Distinct indication for "the probe is not telling me anything".
 * A disconnected probe reads the same as an empty tank, so without
 * this a broken wire looks exactly like a genuine low-level alarm.
 * White sits deliberately outside the normal colour set. */

void ledFault()
{
  ledZone = 0xFE;
  showColor(150, 150, 150);
  delay(150);
  showColor(0, 0, 0);
}

/* ============================================================
 * UART framing
 * ============================================================ */

/* Wrap a payload in $...*XX\r\n and transmit.
 * The checksum earns its keep twice over: during bring-up a valid
 * frame proves baud, wiring and framing all at once, and in service
 * it stops a noise burst from being read as a plausible level. */
void sendPayload(const char *payload)
{
  char    frame[64];
  uint8_t cs = 0;

  for (const char *c = payload; *c; c++) cs ^= (uint8_t) *c;

  snprintf(frame, sizeof(frame), "$%s*%02X\r\n", payload, cs);
  espSerial.print(frame);
}


void sendReading(uint16_t raw, uint16_t tenths)
{
  char payload[40];
  snprintf(payload, sizeof(payload), "WL,%u,%u.%u",
           (unsigned) raw,
           (unsigned)(tenths / 10),
           (unsigned)(tenths % 10));
  sendPayload(payload);
}


void sendFault()
{
  sendPayload("WL,ERR,0.0");
}


/* ============================================================
 * Arduino entry points
 * ============================================================ */

void setup()
{
  pinMode(SENSOR_PWR, OUTPUT);
  digitalWrite(SENSOR_PWR, LOW);

  pinMode(SENSOR_PIN, INPUT_ANALOG);

  /* CRITICAL. STM32duino defaults analogRead to 10-bit, so without
   * this you get 0-1023 and any calibration value above 1023 becomes
   * unreachable - the level pins at 100% and never moves. */
  analogReadResolution(12);

  espSerial.begin(115200);

  led.begin();
  led.setBrightness(LED_BRIGHTNESS);
  led.clear();
  led.show();                     /* clear any stale colour at boot */

  delay(200);
}


void loop()
{
  uint16_t raw = readSensor();

  if (raw + ADC_OPEN_MARGIN < ADC_DRY)
  {
    ledFault();
    sendFault();
  }
  else
  {
    uint16_t tenths = percentTenths(raw);
    ledUpdate(tenths);
    sendReading(raw, tenths);
  }

  delay(PERIOD_MS - SETTLE_MS);
}


/* ============================================================
 * WIRING
 * ============================================================
 *   Probe VCC     -> PA1
 *   Probe GND     -> GND
 *   Probe Signal  -> PA0, plus 100nF from PA0 to GND
 *
 *   PA6 -> 330 ohm -> WS2812B DIN
 *   WS2812B VDD  <- 5V through a 1N4148 diode
 *   WS2812B GND  -> GND
 *   1000uF electrolytic across WS2812B VDD/GND
 *
 *   PA9  (TX) -> ESP32 GPIO16 (RX2)
 *   PA10 (RX) -> ESP32 GPIO17 (TX2)
 *   GND       -> ESP32 GND        <-- required, not optional
 *
 * Power the probe from 3.3V logic levels only. Its output on a 5V
 * supply can swing above VDDA and damage the ADC input.
 *
 *
 * THE 100nF ON PA0
 * ----------------
 * Not optional here. The F103 ADC samples onto an internal capacitor
 * through the pin, and a resistive water probe is a high-impedance
 * source of tens of kilohms. Under CubeIDE you would fix this by
 * setting sampling time to 239.5 cycles, but STM32duino does not
 * expose sampling time through analogRead. The capacitor solves it
 * in hardware instead: it holds local charge so the ADC's sample cap
 * fills from the cap rather than through the probe.
 *
 * Without it, readings come back low and jittery and no amount of
 * averaging fixes it - this is the usual reason an analog sensor
 * "works on Arduino Uno but reads wrong on STM32". The AVR's default
 * sampling window is far longer, so the Uno hides the problem.
 *
 * The cap is also why SETTLE_MS is 50 rather than 10: it now has to
 * charge through the probe on every power-on. If you shorten the
 * settle time you will see readings that creep upward across the
 * averaging loop instead of sitting flat.
 *
 *
 * WS2812B LOGIC LEVEL
 * -------------------
 * The WS2812B wants data high at >= 0.7 * VDD. At VDD = 5.0V that is
 * 3.5V, above what a 3.3V STM32 pin can drive. It often works on the
 * bench and then fails once warm, which is a miserable fault to
 * chase. A 1N4148 in series with the LED's supply brings VDD to about
 * 4.3V, so the threshold falls to 3.0V and 3.3V has real margin. One
 * part, no code change. A 74AHCT125 buffer is the fully correct fix.
 *
 *
 * CALIBRATION
 * -----------
 * Flash as-is, open the ESP32's USB serial monitor, and watch the raw
 * field - the ESP32 prints every frame it receives.
 *
 *   1. Probe dry, in air                    -> raw  -> ADC_DRY
 *   2. Probe submerged to your max depth    -> raw  -> ADC_WET
 *
 * Recompile with those numbers. Expect roughly 50-200 dry and
 * 2800-3500 fully wet, but every probe differs and conductivity
 * matters a lot: tap water, distilled water and salt water all read
 * differently on the same probe. Calibrate in the water you actually
 * intend to measure.
 *
 * If raw goes DOWN as the probe goes deeper, your module wires the
 * divider the other way round. Set ADC_DRY to the high number and
 * ADC_WET to the low one; span goes negative and the maths still
 * works. Then also flip the sign in the ADC_OPEN_MARGIN test.
 *
 * Robu's (CA - DA)/100 formula is the same idea with the endpoint
 * hardcoded. Measuring both ends yourself is more accurate because it
 * accounts for your supply voltage and your water.
 *
 *
 * WHY NO %f
 * ---------
 * Percent is carried as an integer in tenths and split with /10 and
 * %10 at print time. Float support in snprintf is not guaranteed on
 * every STM32duino build configuration, and when it is missing a
 * %.1f prints as a literal "f" or vanishes - which looks like a UART
 * fault rather than a printf problem. Integers sidestep it entirely.
 * ============================================================ */
