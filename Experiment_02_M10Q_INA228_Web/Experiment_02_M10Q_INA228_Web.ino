#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPSPlus.h>
#include <Adafruit_INA228.h>
#include <Wire.h>
#include <math.h>
#include "secrets.h"

// ---------- Wi-Fi ----------
// WIFI_SSID and WIFI_PASSWORD are defined in the local secrets.h file.
// Copy secrets.example.h to secrets.h and enter your own credentials.

// If the apartment Wi-Fi connection fails, connect your phone/computer to:
const char *FALLBACK_AP_SSID = "ESP32-GPS-M10Q";

// ---------- M10Q-5883 UART wiring ----------
// M10Q TX -> ESP32 GPIO25 (ESP32 receives here)
// M10Q RX <- ESP32 GPIO26 (ESP32 transmits here)
constexpr int GPS_RX_PIN = 25;
constexpr int GPS_TX_PIN = 26;
constexpr uint32_t GPS_BAUD = 9600;

// ---------- M10Q-5883 compass wiring ----------
// M10Q DA -> ESP32 GPIO21
// M10Q CL -> ESP32 GPIO22
constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;
constexpr uint8_t QMC5883L_ADDRESS = 0x0D;

// ---------- INA228 current monitor ----------
// INA228 shares GPIO21/GPIO22 with the compass. Default I2C address: 0x40.
// Logic: ESP32 3V3 -> INA228 VIN/VS, GND -> GND,
//        GPIO21 -> SDA, GPIO22 -> SCL. Leave ALRT unconnected.
// High-side measurement path:
//        ESP32 5V -> INA228 IN+/VIN+
//        INA228 IN-/VIN- -> M10Q 5V
//        INA228 VBUS -> IN+/VIN+ (wire it if the board does not tie them)
// Do not leave the old direct ESP32-5V-to-M10Q-5V wire in parallel.
// IMPORTANT: 0.015 ohm is correct for an R015 / 15 milliohm shunt.
// Change this value if the large shunt resistor on your board is different.
constexpr uint8_t INA228_ADDRESS = 0x40;
constexpr float INA228_SHUNT_OHMS = 0.015f;
constexpr float INA228_MAX_CURRENT_A = 0.5f;

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
WebServer server(80);
Adafruit_INA228 ina228;

constexpr size_t RAW_LINE_LENGTH = 128;
constexpr size_t RAW_LINE_COUNT = 8;
char currentRawLine[RAW_LINE_LENGTH];
size_t currentRawLength = 0;
bool capturingNmeaLine = false;
String recentRawLines[RAW_LINE_COUNT];
size_t nextRawLine = 0;

uint32_t totalGpsBytes = 0;
uint32_t totalRawLines = 0;
uint32_t lastGpsByteMs = 0;
uint32_t lastValidNmeaMs = 0;
bool usingFallbackAP = false;

bool compassFound = false;
bool compassDataValid = false;
int16_t compassRawX = 0;
int16_t compassRawY = 0;
int16_t compassRawZ = 0;
float compassFilteredX = 0.0f;
float compassFilteredY = 0.0f;
float compassFilteredZ = 0.0f;
float compassHeadingDeg = 0.0f;
uint32_t lastCompassReadMs = 0;
uint32_t lastCompassDataMs = 0;
uint32_t lastCompassProbeMs = 0;

bool inaFound = false;
bool inaDataValid = false;
float inaCurrentmA = 0.0f;
float inaAverageCurrentmA = 0.0f;
float inaBusVoltageV = 0.0f;
float inaShuntVoltagemV = 0.0f;
float inaPowermW = 0.0f;
float inaTemperatureC = 0.0f;
double measuredEnergymWh = 0.0;
double measuredChargemAh = 0.0;
uint32_t lastInaReadMs = 0;
uint32_t lastInaDataMs = 0;

enum GnssStateEstimate : uint8_t {
  GNSS_WIDE_SEARCH,
  GNSS_NARROW_SEARCH,
  GNSS_STABLE_TRACKING
};

GnssStateEstimate gnssStateEstimate = GNSS_WIDE_SEARCH;
uint32_t continuousFixSinceMs = 0;
double currentSumByGnssState[3] = {0.0, 0.0, 0.0};
uint32_t currentSamplesByGnssState[3] = {0, 0, 0};

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="he" dir="rtl">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32 + M10Q GNSS</title>
  <style>
    :root { color-scheme: dark; font-family: Arial, sans-serif; }
    body { margin: 0; background: #0b1220; color: #e5eefc; }
    main { width: min(1000px, calc(100% - 28px)); margin: 22px auto 50px; }
    h1 { font-size: 1.55rem; margin-bottom: 8px; }
    .sub { color: #9fb0c8; margin-bottom: 18px; }
    .status { padding: 15px; border-radius: 12px; border: 1px solid #334155; margin-bottom: 16px; }
    .waiting { background: #172033; }
    .bad { background: #3a171b; border-color: #7f1d1d; }
    .warn { background: #382c11; border-color: #854d0e; }
    .good { background: #0c3024; border-color: #166534; }
    .fix { background: #0d2844; border-color: #1d4ed8; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit,minmax(190px,1fr)); gap: 10px; }
    .card { background: #111b2d; border: 1px solid #26344a; border-radius: 10px; padding: 12px; }
    .label { color: #93a4bd; font-size: .82rem; }
    .value { font-size: 1.18rem; margin-top: 5px; overflow-wrap: anywhere; }
    h2 { font-size: 1.08rem; margin: 22px 0 10px; }
    pre { direction: ltr; text-align: left; white-space: pre-wrap; word-break: break-all; background: #07101e; border: 1px solid #26344a; padding: 13px; border-radius: 10px; min-height: 105px; }
    a { color: #72a7ff; }
    .small { color: #93a4bd; font-size: .83rem; margin-top: 12px; }
  </style>
</head>
<body><main>
  <h1>ESP32 + Matek M10Q-5883</h1>
  <div class="sub">בדיקת UART, משפטי NMEA וקליטת לוויינים בזמן אמת</div>
  <div id="status" class="status waiting">מתחבר...</div>

  <div class="grid">
    <div class="card"><div class="label">קואורדינטות</div><div id="position" class="value">—</div></div>
    <div class="card"><div class="label">לוויינים</div><div id="satellites" class="value">—</div></div>
    <div class="card"><div class="label">HDOP</div><div id="hdop" class="value">—</div></div>
    <div class="card"><div class="label">גובה</div><div id="altitude" class="value">—</div></div>
    <div class="card"><div class="label">מהירות</div><div id="speed" class="value">—</div></div>
    <div class="card"><div class="label">כיוון תנועה</div><div id="course" class="value">—</div></div>
    <div class="card"><div class="label">זמן GNSS (UTC)</div><div id="utc" class="value">—</div></div>
    <div class="card"><div class="label">גיל המדידה</div><div id="age" class="value">—</div></div>
  </div>

  <h2>מדידת זרם ומצב המקלט</h2>
  <div id="inaStatus" class="status waiting">בודק את INA228...</div>
  <div id="gnssModeStatus" class="status waiting">מחשב את מצב המקלט...</div>
  <div class="grid">
    <div class="card"><div class="label">מצב GNSS משוער</div><div id="gnssMode" class="value">—</div></div>
    <div class="card"><div class="label">זרם רגעי</div><div id="current" class="value">—</div></div>
    <div class="card"><div class="label">זרם ממוצע מסונן</div><div id="averageCurrent" class="value">—</div></div>
    <div class="card"><div class="label">ממוצע בחיפוש רחב</div><div id="wideCurrent" class="value">—</div></div>
    <div class="card"><div class="label">ממוצע בחיפוש צר</div><div id="narrowCurrent" class="value">—</div></div>
    <div class="card"><div class="label">ממוצע במצב יציב</div><div id="stableCurrent" class="value">—</div></div>
    <div class="card"><div class="label">מתח הזנת ה-GPS</div><div id="busVoltage" class="value">—</div></div>
    <div class="card"><div class="label">מתח על השאנט</div><div id="shuntVoltage" class="value">—</div></div>
    <div class="card"><div class="label">הספק</div><div id="power" class="value">—</div></div>
    <div class="card"><div class="label">אנרגיה מאז האתחול</div><div id="energy" class="value">—</div></div>
    <div class="card"><div class="label">מטען מאז האתחול</div><div id="charge" class="value">—</div></div>
    <div class="card"><div class="label">טמפרטורת INA228</div><div id="inaTemperature" class="value">—</div></div>
  </div>
  <div class="small">המצב הוא הערכה מנתוני GNSS: מספר לוויינים, Fix רציף ומשך היציבות. הזרם נמדד בנפרד ואינו לבדו הוכחה למצב פנימי מסוים.</div>

  <h2>מצפן QMC5883L</h2>
  <div id="compassStatus" class="status waiting">בודק את חיבור המצפן...</div>
  <div class="grid">
    <div class="card"><div class="label">כיוון מגנטי</div><div id="heading" class="value">—</div></div>
    <div class="card"><div class="label">כיוון</div><div id="cardinal" class="value">—</div></div>
    <div class="card"><div class="label">ציר X גולמי</div><div id="magX" class="value">—</div></div>
    <div class="card"><div class="label">ציר Y גולמי</div><div id="magY" class="value">—</div></div>
    <div class="card"><div class="label">ציר Z גולמי</div><div id="magZ" class="value">—</div></div>
    <div class="card"><div class="label">כתובת I²C</div><div id="compassAddress" class="value">0x0D</div></div>
  </div>

  <h2>הוכחת תקשורת</h2>
  <div class="grid">
    <div class="card"><div class="label">בתים שהתקבלו</div><div id="bytes" class="value">0</div></div>
    <div class="card"><div class="label">משפטים עם checksum תקין</div><div id="passed" class="value">0</div></div>
    <div class="card"><div class="label">checksum שגוי</div><div id="failed" class="value">0</div></div>
    <div class="card"><div class="label">כתובת האתר</div><div id="network" class="value">—</div></div>
  </div>

  <h2>משפטי NMEA אחרונים</h2>
  <pre id="raw">ממתין למידע...</pre>
  <div id="map"></div>
  <div class="small">העמוד מתעדכן פעם בשנייה. זמן GNSS הוא UTC, לא שעון ישראל.</div>
</main>
<script>
const val = (x, suffix='') => (x === null || x === undefined) ? '—' : x + suffix;
async function refresh() {
  try {
    const r = await fetch('/api/data', {cache:'no-store'});
    const d = await r.json();
    const s = document.getElementById('status');
    s.className = 'status ' + d.status_class;
    s.textContent = d.status_text;
    document.getElementById('position').textContent = d.fix ? `${d.latitude.toFixed(7)}, ${d.longitude.toFixed(7)}` : 'אין Fix עדיין';
    document.getElementById('satellites').textContent = val(d.satellites);
    document.getElementById('hdop').textContent = val(d.hdop);
    document.getElementById('altitude').textContent = val(d.altitude_m, ' m');
    document.getElementById('speed').textContent = val(d.speed_kmh, ' km/h');
    document.getElementById('course').textContent = val(d.course_deg, '°');
    document.getElementById('utc').textContent = d.utc || '—';
    document.getElementById('age').textContent = val(d.location_age_ms, ' ms');
    const is = document.getElementById('inaStatus');
    is.className = 'status ' + d.ina_status_class;
    is.textContent = d.ina_status_text;
    const gs = document.getElementById('gnssModeStatus');
    gs.className = 'status ' + d.gnss_mode_class;
    gs.textContent = d.gnss_mode_text;
    document.getElementById('gnssMode').textContent = d.gnss_mode;
    document.getElementById('current').textContent = val(d.current_ma, ' mA');
    document.getElementById('averageCurrent').textContent = val(d.average_current_ma, ' mA');
    document.getElementById('wideCurrent').textContent = val(d.wide_search_average_ma, ' mA');
    document.getElementById('narrowCurrent').textContent = val(d.narrow_search_average_ma, ' mA');
    document.getElementById('stableCurrent').textContent = val(d.stable_average_ma, ' mA');
    document.getElementById('busVoltage').textContent = val(d.bus_voltage_v, ' V');
    document.getElementById('shuntVoltage').textContent = val(d.shunt_voltage_mv, ' mV');
    document.getElementById('power').textContent = val(d.power_mw, ' mW');
    document.getElementById('energy').textContent = val(d.energy_mwh, ' mWh');
    document.getElementById('charge').textContent = val(d.charge_mah, ' mAh');
    document.getElementById('inaTemperature').textContent = val(d.ina_temperature_c, '°C');
    const cs = document.getElementById('compassStatus');
    cs.className = 'status ' + d.compass_status_class;
    cs.textContent = d.compass_status_text;
    document.getElementById('heading').textContent = val(d.compass_heading_deg, '°');
    document.getElementById('cardinal').textContent = d.compass_cardinal || '—';
    document.getElementById('magX').textContent = val(d.compass_x);
    document.getElementById('magY').textContent = val(d.compass_y);
    document.getElementById('magZ').textContent = val(d.compass_z);
    document.getElementById('bytes').textContent = d.total_bytes;
    document.getElementById('passed').textContent = d.checksum_passed;
    document.getElementById('failed').textContent = d.checksum_failed;
    document.getElementById('network').textContent = d.network;
    document.getElementById('raw').textContent = d.raw_nmea || 'ממתין למידע...';
    document.getElementById('map').innerHTML = d.fix
      ? `<p><a target="_blank" rel="noopener" href="https://www.google.com/maps?q=${d.latitude},${d.longitude}">פתיחת המיקום במפה</a></p>` : '';
  } catch (e) {
    const s = document.getElementById('status');
    s.className = 'status bad';
    s.textContent = 'החיבור ל-ESP32 נותק';
  }
}
refresh();
setInterval(refresh, 1000);
</script></body></html>
)HTML";

String jsonEscape(const String &input) {
  String output;
  output.reserve(input.length() + 16);
  for (size_t i = 0; i < input.length(); ++i) {
    const char c = input[i];
    switch (c) {
      case '"': output += "\\\""; break;
      case '\\': output += "\\\\"; break;
      case '\n': output += "\\n"; break;
      case '\r': output += "\\r"; break;
      case '\t': output += "\\t"; break;
      default:
        if (static_cast<uint8_t>(c) >= 0x20) output += c;
        break;
    }
  }
  return output;
}

void storeCompletedRawLine() {
  if (currentRawLength == 0) return;
  currentRawLine[currentRawLength] = '\0';
  recentRawLines[nextRawLine] = String(currentRawLine);
  nextRawLine = (nextRawLine + 1) % RAW_LINE_COUNT;
  ++totalRawLines;
  currentRawLength = 0;
}

void consumeGpsByte(char c) {
  if (gps.encode(c)) lastValidNmeaMs = millis();
  ++totalGpsBytes;
  lastGpsByteMs = millis();

  if (c == '$') {
    // A fresh NMEA sentence always starts with '$'. This also prevents binary
    // UBX traffic from being shown as garbage in the browser.
    currentRawLength = 0;
    currentRawLine[currentRawLength++] = c;
    capturingNmeaLine = true;
  } else if (capturingNmeaLine && c == '\n') {
    storeCompletedRawLine();
    capturingNmeaLine = false;
  } else if (capturingNmeaLine && c != '\r') {
    const uint8_t u = static_cast<uint8_t>(c);
    if (u >= 0x20 && u <= 0x7E) {
      if (currentRawLength < RAW_LINE_LENGTH - 1) {
        currentRawLine[currentRawLength++] = c;
      }
    } else {
      // A binary byte inside the candidate means this was not NMEA.
      currentRawLength = 0;
      capturingNmeaLine = false;
    }
  }
}

void readGps() {
  while (gpsSerial.available() > 0) {
    consumeGpsByte(static_cast<char>(gpsSerial.read()));
  }
}

bool qmcWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMC5883L_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool qmcReadRegisters(uint8_t startRegister, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(QMC5883L_ADDRESS);
  Wire.write(startRegister);
  if (Wire.endTransmission(false) != 0) return false;

  const uint8_t received = Wire.requestFrom(QMC5883L_ADDRESS, length);
  if (received != length) return false;
  for (uint8_t i = 0; i < length; ++i) buffer[i] = Wire.read();
  return true;
}

bool initializeCompass() {
  lastCompassProbeMs = millis();

  // First check that an I2C device acknowledges at the QMC5883L address.
  Wire.beginTransmission(QMC5883L_ADDRESS);
  if (Wire.endTransmission() != 0) return false;

  // QMC5883L register setup:
  // 0x0A: soft reset, 0x0B: set/reset period,
  // 0x09 = 0x1D: continuous mode, 200 Hz, +/-8 gauss, OSR 512.
  if (!qmcWriteRegister(0x0A, 0x80)) return false;
  delay(10);
  if (!qmcWriteRegister(0x0B, 0x01)) return false;
  if (!qmcWriteRegister(0x09, 0x1D)) return false;

  compassDataValid = false;
  Serial.println("QMC5883L compass found at I2C address 0x0D");
  return true;
}

const char *compassCardinal(float degrees) {
  static const char *directions[] = {
    "צפון (N)", "צפון-מזרח (NE)", "מזרח (E)", "דרום-מזרח (SE)",
    "דרום (S)", "דרום-מערב (SW)", "מערב (W)", "צפון-מערב (NW)"
  };
  const int index = static_cast<int>((degrees + 22.5f) / 45.0f) % 8;
  return directions[index];
}

void serviceCompass() {
  const uint32_t now = millis();

  if (!compassFound) {
    if (lastCompassProbeMs == 0 || now - lastCompassProbeMs >= 3000) {
      compassFound = initializeCompass();
      if (!compassFound) Serial.println("QMC5883L not found at I2C address 0x0D");
    }
    return;
  }

  if (now - lastCompassReadMs < 50) return;
  lastCompassReadMs = now;

  uint8_t status = 0;
  if (!qmcReadRegisters(0x06, &status, 1)) {
    compassFound = false;
    compassDataValid = false;
    return;
  }

  // Bit 0 = new data ready. Bit 1 = magnetic overflow.
  if ((status & 0x01) == 0 || (status & 0x02) != 0) return;

  uint8_t data[6];
  if (!qmcReadRegisters(0x00, data, sizeof(data))) {
    compassFound = false;
    compassDataValid = false;
    return;
  }

  compassRawX = static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                                     (static_cast<uint16_t>(data[1]) << 8));
  compassRawY = static_cast<int16_t>(static_cast<uint16_t>(data[2]) |
                                     (static_cast<uint16_t>(data[3]) << 8));
  compassRawZ = static_cast<int16_t>(static_cast<uint16_t>(data[4]) |
                                     (static_cast<uint16_t>(data[5]) << 8));

  if (!compassDataValid) {
    compassFilteredX = compassRawX;
    compassFilteredY = compassRawY;
    compassFilteredZ = compassRawZ;
  } else {
    constexpr float alpha = 0.18f;
    compassFilteredX += alpha * (compassRawX - compassFilteredX);
    compassFilteredY += alpha * (compassRawY - compassFilteredY);
    compassFilteredZ += alpha * (compassRawZ - compassFilteredZ);
  }

  compassHeadingDeg = atan2f(compassFilteredY, compassFilteredX) * 180.0f / PI;
  if (compassHeadingDeg < 0.0f) compassHeadingDeg += 360.0f;

  compassDataValid = true;
  lastCompassDataMs = now;
}

bool initializeINA228() {
  if (!ina228.begin(INA228_ADDRESS, &Wire)) {
    Serial.println("INA228 not found at I2C address 0x40");
    return false;
  }

  // Calibrate for the shunt fitted on the breakout and for the small current
  // expected from the GNSS module. The 0.5 A range gives sub-uA current LSB.
  ina228.setShunt(INA228_SHUNT_OHMS, INA228_MAX_CURRENT_A);
  ina228.setAveragingCount(INA228_COUNT_64);
  ina228.setVoltageConversionTime(INA228_TIME_1052_us);
  ina228.setCurrentConversionTime(INA228_TIME_1052_us);
  ina228.resetAccumulators();

  inaDataValid = false;
  lastInaReadMs = 0;
  Serial.println("INA228 found at I2C address 0x40");
  return true;
}

void serviceINA228() {
  if (!inaFound) return;

  const uint32_t now = millis();
  if (now - lastInaReadMs < 200) return;
  const uint32_t elapsedMs = (lastInaReadMs == 0) ? 0 : now - lastInaReadMs;
  lastInaReadMs = now;

  const float currentmA = ina228.getCurrent_mA();
  const float busVoltageV = ina228.getBusVoltage_V();
  const float shuntVoltagemV = ina228.getShuntVoltage_mV();
  const float powermW = ina228.getPower_mW();
  const float temperatureC = ina228.readDieTemp();

  if (!isfinite(currentmA) || !isfinite(busVoltageV) ||
      !isfinite(shuntVoltagemV) || !isfinite(powermW) ||
      !isfinite(temperatureC)) {
    inaDataValid = false;
    return;
  }

  inaCurrentmA = currentmA;
  inaBusVoltageV = busVoltageV;
  inaShuntVoltagemV = shuntVoltagemV;
  inaPowermW = powermW;
  inaTemperatureC = temperatureC;

  if (!inaDataValid) {
    inaAverageCurrentmA = currentmA;
  } else {
    // At five samples per second, alpha=0.04 gives a useful ~5 s smoothing.
    constexpr float alpha = 0.04f;
    inaAverageCurrentmA += alpha * (currentmA - inaAverageCurrentmA);
  }

  if (elapsedMs > 0 && elapsedMs < 2000) {
    const double elapsedHours = static_cast<double>(elapsedMs) / 3600000.0;
    measuredEnergymWh += static_cast<double>(powermW) * elapsedHours;
    measuredChargemAh += static_cast<double>(currentmA) * elapsedHours;
  }

  const uint8_t stateIndex = static_cast<uint8_t>(gnssStateEstimate);
  currentSumByGnssState[stateIndex] += currentmA;
  ++currentSamplesByGnssState[stateIndex];

  inaDataValid = true;
  lastInaDataMs = now;
}

void updateGnssStateEstimate() {
  const uint32_t now = millis();
  const bool freshFix = gps.location.isValid() && gps.location.age() < 5000;
  const bool freshSatellites = gps.satellites.isValid() &&
                               gps.satellites.age() < 5000;
  const uint32_t satellites = freshSatellites ? gps.satellites.value() : 0;

  if (freshFix) {
    if (continuousFixSinceMs == 0) continuousFixSinceMs = now;
  } else {
    continuousFixSinceMs = 0;
  }

  const bool stableFix = freshFix && satellites >= 4 &&
                         continuousFixSinceMs > 0 &&
                         now - continuousFixSinceMs >= 15000;

  if (stableFix) {
    gnssStateEstimate = GNSS_STABLE_TRACKING;
  } else if (freshFix || satellites >= 3) {
    gnssStateEstimate = GNSS_NARROW_SEARCH;
  } else {
    gnssStateEstimate = GNSS_WIDE_SEARCH;
  }
}

const char *gnssStateName() {
  switch (gnssStateEstimate) {
    case GNSS_STABLE_TRACKING: return "מצב יציב";
    case GNSS_NARROW_SEARCH: return "חיפוש צר";
    default: return "חיפוש רחב";
  }
}

String recentNmeaText() {
  String text;
  text.reserve(RAW_LINE_COUNT * 90);
  for (size_t i = 0; i < RAW_LINE_COUNT; ++i) {
    const size_t index = (nextRawLine + i) % RAW_LINE_COUNT;
    if (recentRawLines[index].length() > 0) {
      if (text.length() > 0) text += '\n';
      text += recentRawLines[index];
    }
  }
  return text;
}

void appendJsonNumberOrNull(String &json, bool valid, double value, unsigned int decimals) {
  if (valid) json += String(value, decimals);
  else json += "null";
}

String gnssUtcText() {
  if (!gps.date.isValid() || !gps.time.isValid()) return "";
  char buffer[28];
  snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
           gps.date.year(), gps.date.month(), gps.date.day(),
           gps.time.hour(), gps.time.minute(), gps.time.second());
  return String(buffer);
}

void handleApiData() {
  const uint32_t now = millis();
  const bool uartRecent = totalGpsBytes > 0 && (now - lastGpsByteMs) < 5000;
  const bool nmeaValid = lastValidNmeaMs > 0 && (now - lastValidNmeaMs) < 5000;
  const bool currentFix = gps.location.isValid() && gps.location.age() < 5000;
  const bool compassRecent = compassFound && compassDataValid &&
                             (now - lastCompassDataMs) < 1000;
  const bool inaRecent = inaFound && inaDataValid &&
                         (now - lastInaDataMs) < 1500;
  const bool wideAverageValid = currentSamplesByGnssState[GNSS_WIDE_SEARCH] > 0;
  const bool narrowAverageValid = currentSamplesByGnssState[GNSS_NARROW_SEARCH] > 0;
  const bool stableAverageValid = currentSamplesByGnssState[GNSS_STABLE_TRACKING] > 0;
  const double wideAverageCurrent = wideAverageValid
      ? currentSumByGnssState[GNSS_WIDE_SEARCH] /
        currentSamplesByGnssState[GNSS_WIDE_SEARCH] : 0.0;
  const double narrowAverageCurrent = narrowAverageValid
      ? currentSumByGnssState[GNSS_NARROW_SEARCH] /
        currentSamplesByGnssState[GNSS_NARROW_SEARCH] : 0.0;
  const double stableAverageCurrent = stableAverageValid
      ? currentSumByGnssState[GNSS_STABLE_TRACKING] /
        currentSamplesByGnssState[GNSS_STABLE_TRACKING] : 0.0;

  const char *statusClass;
  const char *statusText;
  if (totalGpsBytes == 0 && now < 5000) {
    statusClass = "waiting";
    statusText = "ממתין לנתונים מה-GNSS...";
  } else if (!uartRecent) {
    statusClass = "bad";
    statusText = "אין כרגע נתוני UART: לבדוק מתח, GND, הצלבת TX/RX ומהירות 9600";
  } else if (!nmeaValid) {
    statusClass = "warn";
    statusText = "בתים מגיעים ב-UART, אבל עדיין לא זוהה משפט NMEA עם checksum תקין";
  } else if (!currentFix) {
    statusClass = "good";
    statusText = "המודול עובד: מתקבלים משפטי NMEA תקינים. ממתין ל-Fix מלוויינים בחוץ";
  } else {
    statusClass = "fix";
    statusText = "הצלחה מלאה: GNSS עובד, NMEA תקין ויש Fix עדכני";
  }

  const char *inaStatusClass;
  const char *inaStatusText;
  if (!inaFound) {
    inaStatusClass = "bad";
    inaStatusText = "INA228 לא נמצא בכתובת 0x40: לבדוק 3.3V, GND, SDA ו-SCL";
  } else if (!inaRecent) {
    inaStatusClass = "warn";
    inaStatusText = "INA228 נמצא, אבל עדיין אין מדידה תקינה";
  } else if (inaCurrentmA < -0.05f) {
    inaStatusClass = "warn";
    inaStatusText = "הזרם שלילי: כנראה IN+ ו-IN- מחוברים הפוך";
  } else if (inaBusVoltageV < 3.5f) {
    inaStatusClass = "warn";
    inaStatusText = "זרם נמדד, אבל מתח VBUS נמוך: לבדוק את חיבור VBUS לכניסת ה-5V";
  } else {
    inaStatusClass = "good";
    inaStatusText = "INA228 מחובר ומודד את צריכת ה-M10Q";
  }

  const char *gnssModeClass;
  const char *gnssModeText;
  switch (gnssStateEstimate) {
    case GNSS_STABLE_TRACKING:
      gnssModeClass = "fix";
      gnssModeText = "מצב יציב: Fix נשמר לפחות 15 שניות עם 4 לוויינים או יותר";
      break;
    case GNSS_NARROW_SEARCH:
      gnssModeClass = "good";
      gnssModeText = "חיפוש צר: אותרו לפחות 3 לוויינים או שהתקבל Fix שעדיין מתייצב";
      break;
    default:
      gnssModeClass = "warn";
      gnssModeText = "חיפוש רחב: אין Fix ועדיין אין מספיק לוויינים למיקום";
      break;
  }

  const char *compassStatusClass;
  const char *compassStatusText;
  if (!compassFound) {
    compassStatusClass = "bad";
    compassStatusText = "המצפן לא נמצא ב-I²C: לבדוק DA→GPIO21, CL→GPIO22 ו-GND משותף";
  } else if (!compassRecent) {
    compassStatusClass = "warn";
    compassStatusText = "QMC5883L נמצא בכתובת 0x0D, אבל עדיין אין מדידה חדשה";
  } else {
    compassStatusClass = "good";
    compassStatusText = "המצפן מחובר ומחזיר נתוני X/Y/Z. סובב את הלוח כשהוא מאוזן ובדוק שהכיוון משתנה";
  }

  String networkText;
  if (usingFallbackAP) {
    networkText = String(FALLBACK_AP_SSID) + " / " + WiFi.softAPIP().toString();
  } else {
    networkText = WiFi.SSID() + " / " + WiFi.localIP().toString();
  }

  String json;
  json.reserve(3200);
  json += "{\"status_class\":\"";
  json += statusClass;
  json += "\",\"status_text\":\"";
  json += statusText;
  json += "\",\"fix\":";
  json += currentFix ? "true" : "false";
  json += ",\"latitude\":";
  appendJsonNumberOrNull(json, currentFix, gps.location.lat(), 7);
  json += ",\"longitude\":";
  appendJsonNumberOrNull(json, currentFix, gps.location.lng(), 7);
  json += ",\"satellites\":";
  if (gps.satellites.isValid()) json += String(gps.satellites.value()); else json += "null";
  json += ",\"hdop\":";
  appendJsonNumberOrNull(json, gps.hdop.isValid(), gps.hdop.hdop(), 2);
  json += ",\"altitude_m\":";
  appendJsonNumberOrNull(json, gps.altitude.isValid(), gps.altitude.meters(), 1);
  json += ",\"speed_kmh\":";
  appendJsonNumberOrNull(json, gps.speed.isValid(), gps.speed.kmph(), 1);
  json += ",\"course_deg\":";
  appendJsonNumberOrNull(json, gps.course.isValid(), gps.course.deg(), 1);
  json += ",\"utc\":\"";
  json += jsonEscape(gnssUtcText());
  json += "\",\"location_age_ms\":";
  if (gps.location.isValid()) json += String(gps.location.age()); else json += "null";
  json += ",\"total_bytes\":";
  json += String(totalGpsBytes);
  json += ",\"raw_lines\":";
  json += String(totalRawLines);
  json += ",\"checksum_passed\":";
  json += String(gps.passedChecksum());
  json += ",\"checksum_failed\":";
  json += String(gps.failedChecksum());
  json += ",\"network\":\"";
  json += jsonEscape(networkText);
  json += "\",\"raw_nmea\":\"";
  json += jsonEscape(recentNmeaText());
  json += "\",\"ina_status_class\":\"";
  json += inaStatusClass;
  json += "\",\"ina_status_text\":\"";
  json += inaStatusText;
  json += "\",\"gnss_mode_class\":\"";
  json += gnssModeClass;
  json += "\",\"gnss_mode_text\":\"";
  json += gnssModeText;
  json += "\",\"gnss_mode\":\"";
  json += gnssStateName();
  json += "\",\"current_ma\":";
  appendJsonNumberOrNull(json, inaRecent, inaCurrentmA, 3);
  json += ",\"average_current_ma\":";
  appendJsonNumberOrNull(json, inaRecent, inaAverageCurrentmA, 3);
  json += ",\"wide_search_average_ma\":";
  appendJsonNumberOrNull(json, wideAverageValid, wideAverageCurrent, 3);
  json += ",\"narrow_search_average_ma\":";
  appendJsonNumberOrNull(json, narrowAverageValid, narrowAverageCurrent, 3);
  json += ",\"stable_average_ma\":";
  appendJsonNumberOrNull(json, stableAverageValid, stableAverageCurrent, 3);
  json += ",\"bus_voltage_v\":";
  appendJsonNumberOrNull(json, inaRecent, inaBusVoltageV, 4);
  json += ",\"shunt_voltage_mv\":";
  appendJsonNumberOrNull(json, inaRecent, inaShuntVoltagemV, 4);
  json += ",\"power_mw\":";
  appendJsonNumberOrNull(json, inaRecent, inaPowermW, 3);
  json += ",\"energy_mwh\":";
  appendJsonNumberOrNull(json, inaRecent, measuredEnergymWh, 4);
  json += ",\"charge_mah\":";
  appendJsonNumberOrNull(json, inaRecent, measuredChargemAh, 4);
  json += ",\"ina_temperature_c\":";
  appendJsonNumberOrNull(json, inaRecent, inaTemperatureC, 2);
  json += ",\"compass_status_class\":\"";
  json += compassStatusClass;
  json += "\",\"compass_status_text\":\"";
  json += compassStatusText;
  json += "\",\"compass_heading_deg\":";
  appendJsonNumberOrNull(json, compassRecent, compassHeadingDeg, 1);
  json += ",\"compass_cardinal\":\"";
  if (compassRecent) json += compassCardinal(compassHeadingDeg);
  json += "\",\"compass_x\":";
  if (compassRecent) json += String(compassRawX); else json += "null";
  json += ",\"compass_y\":";
  if (compassRecent) json += String(compassRawY); else json += "null";
  json += ",\"compass_z\":";
  if (compassRecent) json += String(compassRawZ); else json += "null";
  json += "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", json);
}

void startNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");
  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < 15000) {
    readGps();
    delay(20);
    if ((millis() - started) % 500 < 25) Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    usingFallbackAP = false;
    Serial.print("Connected. Open: http://");
    Serial.println(WiFi.localIP());
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(FALLBACK_AP_SSID, FALLBACK_AP_PASSWORD);
    usingFallbackAP = true;
    Serial.println("Apartment Wi-Fi connection failed.");
    Serial.print("Connect to AP: ");
    Serial.println(FALLBACK_AP_SSID);
    Serial.print("Fallback site: http://");
    Serial.println(WiFi.softAPIP());
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("ESP32 + M10Q-5883 GNSS test starting");
  Serial.printf("GPS UART: 9600 8N1, RX=GPIO%d, TX=GPIO%d\n", GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("Compass I2C: SDA=GPIO%d, SCL=GPIO%d, address=0x%02X\n",
                I2C_SDA_PIN, I2C_SCL_PIN, QMC5883L_ADDRESS);
  Serial.printf("INA228 I2C: SDA=GPIO%d, SCL=GPIO%d, address=0x%02X, shunt=%.3f ohm\n",
                I2C_SDA_PIN, I2C_SCL_PIN, INA228_ADDRESS,
                INA228_SHUNT_OHMS);

  gpsSerial.setRxBufferSize(2048);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  compassFound = initializeCompass();
  inaFound = initializeINA228();

  startNetwork();

  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
  });
  server.on("/api/data", HTTP_GET, handleApiData);
  server.onNotFound([]() {
    server.send(404, "text/plain; charset=utf-8", "Not found");
  });
  server.begin();
  Serial.println("Web server started.");
}

void loop() {
  readGps();
  serviceCompass();
  serviceINA228();
  updateGnssStateEstimate();
  server.handleClient();
  delay(1);
}
