Experiment 02 — ESP32 GNSS, Compass, and Power Monitor

Experiment 02 in a practical ESP32 sensor series. This experiment connects a Matek M10Q-5883 navigation module and an INA228 precision power monitor to an ESP32, validates GNSS and compass communication, measures the navigation module's power consumption, and presents all results on a live local web dashboard.

The previous experiment in the series used a BMI270 IMU and is maintained separately.

Project goals

Receive and parse live NMEA data from the M10Q-5883 over UART.

Display position, satellite count, HDOP, altitude, speed, course, and UTC time.

Read the onboard QMC5883L magnetometer over I²C.

Measure only the complete navigation module's current, voltage, and power using an INA228.

Compare average current during wide search, narrow search, and stable GNSS operation.

Prove communication using raw NMEA sentences and checksum counters.

Serve all measurements from the ESP32 as a responsive Hebrew web dashboard.

Hardware

ESP32 development board

Matek M10Q-5883 GNSS and compass module

u-blox M10 GNSS receiver

QMC5883L three-axis magnetometer

INA228 I²C power monitor

R015 shunt resistor: 15 mΩ

Jumper wires and USB or regulated 5 V supply

Repository structure

Experiment_02_M10Q_INA228_Web/
├── Experiment_02_M10Q_INA228_Web.ino
├── README.md
├── secrets.example.h
├── .gitignore
└── images/
    ├── README.md
    ├── 01_physical_setup.jpg
    ├── 02_gnss_satellite_status.png
    ├── 03_current_measurements.png
    ├── 04_voltage_power_temperature.png
    ├── 05_compass_measurements.png
    └── 06_nmea_communication_proof.png

Wiring

M10Q-5883 communication

M10Q-5883

ESP32

Function

TX

GPIO25

GNSS data to ESP32 RX

RX

GPIO26

ESP32 TX to GNSS

DA

GPIO21

QMC5883L I²C data

CL

GPIO22

QMC5883L I²C clock

GND

GND

Common ground

The M10Q 5 V input is powered through the INA228 measurement path described below.

INA228 logic connection

INA228

ESP32

VIN / VCC / VS

3.3 V

GND

GND

SDA

GPIO21

SCL

GPIO22

ALRT

Not connected

The INA228 and QMC5883L safely share the same I²C bus. Their default addresses are 0x40 and 0x0D, respectively.

High-side current measurement

5 V supply ──┬────────────────────────────> ESP32 5 V
             │
             └──> INA228 IN+ ── R015 ──> INA228 IN- ──> M10Q 5 V

Common GND ───────────────────────────────> ESP32 + INA228 + M10Q

Connect INA228 VBUS to the same 5 V node as IN+ if the board does not already tie them together.

Do not leave a direct 5 V wire from the supply to the M10Q; it would bypass the shunt.

The ESP32 is powered directly. Only the complete M10Q-5883 board is measured through the shunt.

The measured current therefore includes the GNSS receiver, compass, onboard regulator, and indicator LED.

Arduino dependencies

Install these libraries from the Arduino IDE Library Manager:

TinyGPSPlus by Mikal Hart

Adafruit INA228 by Adafruit

Adafruit BusIO if it is not installed automatically

Select the appropriate ESP32 board and use Serial Monitor at 115200 baud.

Wi-Fi configuration

The repository does not contain real Wi-Fi credentials.

Copy secrets.example.h to secrets.h.

Enter the local Wi-Fi name and password.

Choose a private fallback access-point password with at least eight characters.

const char *WIFI_SSID = "YOUR_WIFI_NAME";
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char *FALLBACK_AP_PASSWORD = "CHANGE_ME_123";

secrets.h is listed in .gitignore and must never be committed.

If the ESP32 cannot connect to the configured Wi-Fi within 15 seconds, it creates an access point named ESP32-GPS-M10Q. The dashboard is then available at http://192.168.4.1.

Web dashboard

The dashboard updates once per second and reports:

GNSS fix and coordinates

Satellite count and HDOP

Altitude, speed, course, and UTC time

Estimated GNSS operating state

Instantaneous and filtered current

Average current accumulated separately for each GNSS state

Bus voltage and shunt voltage

Power, energy, and charge since startup

INA228 die temperature

Compass heading and raw X/Y/Z readings

UART byte count, NMEA checksum statistics, and recent raw NMEA sentences

GNSS state estimator

The state shown on the dashboard is an application-level estimate derived from NMEA data. It is not a direct report of the receiver's internal u-blox engine state.

Displayed state

Condition used by the firmware

Wide search

No fresh fix and fewer than three satellites

Narrow search

At least three satellites, or a new fix that has not stabilized yet

Stable state

A continuous fix for at least 15 seconds with four or more satellites

The firmware stores an independent average current for each state, allowing the measured power behavior to be compared with the navigation progress.

Measurement configuration

The firmware is configured for the installed R015 shunt:

constexpr float INA228_SHUNT_OHMS = 0.015f;
constexpr float INA228_MAX_CURRENT_A = 0.5f;

At approximately 14 mA, the expected shunt voltage is:

Vshunt = I × R = 0.014 A × 0.015 Ω ≈ 0.210 mV

This agrees with the observed readings of approximately 14 mA and 0.214 mV.

Experimental results

The ESP32 successfully received valid NMEA data at the M10Q default UART rate of 9600 baud.

A valid GNSS fix was obtained and displayed with coordinates, five satellites, and an observed HDOP of 4.72.

The QMC5883L was detected at 0x0D and returned live X/Y/Z and heading data.

The INA228 was detected at 0x40 and measured navigation-module current near 14 mA.

Observed state averages were close to 14 mA; the differences between the three estimated states were small for the complete M10Q-5883 board.

Valid NMEA checksum counts and raw $GNGGA / $GNRMC sentences provided direct communication proof.

Images

1. Physical setup



2. GNSS and satellite status



3. Current measurements



4. Voltage, power, and temperature



5. Compass measurements



6. Communication proof



Running the experiment

Complete the wiring with power disconnected.

Create the local secrets.h file.

Install the Arduino dependencies.

Upload Experiment_02_M10Q_INA228_Web.ino to the ESP32.

Open Serial Monitor at 115200 baud.

Open the printed local IP address in a browser on the same network.

Place the GNSS antenna face-up with a clear view of the sky.

Observe the transition from wide search to narrow search and finally to stable operation.

Notes

GNSS time is displayed in UTC.

GPS course is meaningful while moving; compass heading is available while stationary.

The compass reading is magnetic and requires calibration for accurate absolute heading.

Keep the compass away from motors, high-current wiring, magnets, and ferromagnetic material.

Never publish secrets.h or hard-code private Wi-Fi credentials in a public repository.
