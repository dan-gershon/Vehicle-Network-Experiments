# Experiment 01: "The Black Box" - Real-Time Crash Detection & CAN Transmission

## Overview
This experiment simulates a vehicle's "Black Box" (Event Data Recorder) system. It detects collisions or emergency braking and instantly transmits the event to the vehicle's electronic control units (ECUs). 
The system samples acceleration in real-time, hosts a live Web Dashboard, and simultaneously translates physical impact events into differential analog signals on a CAN bus network.

## Hardware & Components
* **Microcontroller:** ESP32 (Acting as the logic controller and local Web Server).
* **Inertial Measurement Unit (IMU):** BMI270 (Connected via Qwiic/I2C at address `0x69`).
* **CAN Transceiver:** SN65HVD230 module (3.3V logic).
* **Measurement Tool:** UT61D+ Digital Multimeter (Used for capturing transient peak voltages on the network).

## Hardware Setup
![Hardware Setup](images/01_hardware_setup.jpg)

## Logical Flow & Architecture
1. **IoT Interface:** The ESP32 acts as an access point/client, hosting a dynamic HTML/JS dashboard that refreshes every 100ms.
2. **Sensor Polling:** The I2C bus continuously samples the X-axis acceleration from the BMI270 sensor.
3. **Event Trigger (Threshold):** If the acceleration exceeds `2.0g` (simulating a physical crash or heavy shake), two actions occur simultaneously for 500ms:
   * **Software Layer:** The Web Dashboard triggers a flashing red "CRASH" alert.
   * **Physical Layer:** The TX pin (GPIO 13) is pulled `LOW`, forcing the CAN transceiver to broadcast a **Dominant State** to the bus.

## Experimental Results & Physical Measurements

### Idle (Recessive State)
When the system is at rest (Green Dashboard), the CAN transceiver holds the bus lines at a recessive, floating voltage (~2.3V).
![Dashboard Idle](images/02_dashboard_idle.jpg)

### Emergency (Dominant State)
During a physical shake, the system detects the impact (Red Dashboard) and transmits a logical "0" to the CAN bus. 
Using the Multimeter's MAX/MIN function during the physical shake validated the transceiver's output:
* **CAN_H** peaked at: `3.10V`
* **CAN_L** dropped to: `0.83V`
![Dashboard Crash](images/03_dashboard_crash.jpg)

**Differential Voltage Calculation:**
Delta V = CAN_H - CAN_L = 3.10V - 0.83V = 2.27V

*Conclusion:* The measured differential of `2.27V` is well above the required 1.5V CAN standard threshold. This proves the system generates a strong, stable, and noise-immune signal, successfully simulating a valid emergency transmission on an automotive network.

## Software Dependencies
* `WiFi.h` & `WebServer.h` (Built-in ESP32 core libraries)
* `SparkFun BMI270 Arduino Library` (Available via Arduino Library Manager)
