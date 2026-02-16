# STM32-Flight-Telemetry
# Wireless Flight Telemetry System (RTOS & DMA)
I am a third-year Software Engineering student interested in Embedded Systems and Avionics. This project represents my journey in mastering **RTOS architecture, Sensor Fusion, and Low-Level Communication protocols**.

While I strive for professional standards, this code may contain bugs or non-optimal solutions. I welcome any feedback, issues, or pull requests to help me improve

---

##  Project Description
This repository hosts an telemetry system designed for flight controller applications. Building upon my previous Flight Controller RTOS project, this iteration focuses on **high-speed wireless data transmission**.

The system runs on an **STM32F4** microcontroller using **FreeRTOS** to manage concurrent tasks such as sensor reading, data fusion, and display updates. The core innovation in this version is the **Telemetry Pipeline**:
1.  **Data Acquisition & Fusion:** The STM32 collects raw data from the gyroscope, accelerometer, and barometer, fusing them to calculate the aircraft's attitude (Pitch, Roll, Altitude).
2.  **Packet Serialization:** Critical flight data is packed into a custom struct, secured with a **Checksum (XOR)** algorithm to ensure data integrity.
3.  **DMA-Based Transmission:** To minimize CPU load, the serialized data packet is transmitted via **UART using Direct Memory Access (DMA)** to an ESP32 module.
4.  **Wireless Bridge:** The ESP32 (acting as a transparent bridge) transmits this telemetry data over WiFi to a PC/Ground Station for real-time monitoring.

---

##  Hardware Architecture
* **Microcontroller:** STM32F4 Series (STM32F4xx)
* **Wireless Bridge:** ESP32 (UART to WiFi)
* **IMU Sensor:** MPU6050 (Accelerometer & Gyroscope)
* **Barometer:** BME280 (Altitude)
* **Display:** SSD1306 OLED (I2C)

---

##  Software Features & Tech Stack
This project is written in **C** and leverages the following technologies:

* **Real-Time Operating System (FreeRTOS):**
    * `SensorTask`: Reads raw data from MPU6050 & BME280 via I2C (4ms cycle).(**Priority: 4 / Highest**)
    * `FusionTask`: Implements a complementary filter to calculate Pitch, Roll, and filtered Altitude.(**Priority: 3 / High**)
    * `DisplayTask`: Updates the OLED screen with system status .(**Priority: 1 / Low**)
    * `TelemetryTask`: Packages flight data and triggers DMA transfers.(**Priority: 2 / Medium**)
* **Direct Memory Access (DMA):** utilized for UART transmission to ensure non-blocking communication and high CPU efficiency.
* **Data Integrity:** Implements a custom **XOR Checksum** to detect packet corruption during wireless transmission.
* **I2C Management:** Uses Mutexes (`i2cMutex`) to prevent bus contention between sensors and the display.

---

##  Telemetry Data Packet Structure
The system transmits a **20-byte packed structure** (`__attribute__((packed))`) to ensure efficient bandwidth usage. The data is serialized in the following order:

| Byte Offset | Field | Data Type | Description |
| :--- | :--- | :--- | :--- |
| **0 - 1** | `header` | `uint16_t` | Start of Frame (0xABCD) |
| **2 - 5** | `timeStamp` | `uint32_t` | System tick count (ms) |
| **6 - 9** | `pitch` | `float` | Pitch angle (degrees) |
| **10 - 13** | `roll` | `float` | Roll angle (degrees) |
| **14 - 17** | `altitude` | `float` | Filtered altitude (meters) |
| **18** | `status` | `uint8_t` | System status flag |
| **19** | `checksum` | `uint8_t` | XOR Checksum (Calculated over bytes 0-18) |

---
##  Hardware Connections (Pinout)

The following table outlines how the STM32F4 interacts with the sensors and communication modules.

| STM32 Pin | Peripheral | Function | Connected Device |
| :--- | :--- | :--- | :--- |
| **PB6** | `I2C1` | **SCL** | MPU6050, BME280, OLED Display |
| **PB7** | `I2C1` | **SDA** | MPU6050, BME280, OLED Display |
| **PA2** | `USART2` | **TX** | ESP32 (RX Pin) |
| **PA3** | `USART2` | **RX** | ESP32 (TX Pin) |
| **PA10** | `GPIO` | **Output** | Status LED / Debug |
| **3.3V** | Power | VCC | All Modules (Common VCC) |
| **GND** | Power | GND | All Modules (Common Ground) |

> **Note:** Ensure that the STM32 and ESP32 share a common ground (GND) for reliable UART communication.
---
##  System Diagnostics & Error Handling

To aid in debugging without a serial monitor, the system implements a visual error feedback mechanism using the **Status LED (PA10)**.

If a critical hardware failure occurs during initialization (e.g., a sensor is disconnected), the system enters a `System_Error_Handler` loop. The LED will blink a specific number of times, pause for 2 seconds, and then repeat the sequence.

### Error Code Table

| Blink Count | Failed Component | Possible Cause |
| :---: | :--- | :--- |
| **2x Blinks** | **MPU6050 (IMU)** | Sensor not found at address `0x68`. Check wiring or pull-up resistors. |
| **3x Blinks** | **BMP280 (Barometer)** | Sensor not responding. Check connection or I2C address configuration. |

> **Note:** The blink pattern consists of a **1-second ON** pulse followed by a **0.5-second OFF** interval. The sequence repeats after a **2-second** long pause.
---
##  Future Improvements
* Implementation of PID Control Loop for active stabilization.
* Two-way communication (sending commands from PC to STM32).
* Advanced Kalman Filter for better altitude estimation.

##  Unit Testing (Telemetry Module)
To ensure the reliability of the communication protocol before deploying to hardware, I implemented **Unit Tests** using the **Unity Test Framework**.

This allows me to verify the logic of packet generation and data integrity algorithms on a host machine (PC) independent of the STM32 hardware.

### Test Coverage
The test suite (`test_main.c`) covers the following critical scenarios:

* Packet Creation: Verifies that `telemetry_create_packet` correctly populates the header (`0xABCD`), timestamp, and sensor data (Pitch, Roll, Altitude).
* Checksum Validation: Confirms that the XOR Checksum is calculated correctly for valid packets.
* Error Detection: Uses `telemetry_inject_error` to simulate data corruption and asserts that the system correctly identifies invalid packets.

### How to Run Tests
The tests are designed to be compiled with a standard C compiler (GCC).

```bash
# Example compilation command
gcc test_main.c telemetry.c unity/src/unity.c -o test_app.exe

# Output
./test_app.exe
# [Pass] test_telemetry_packet_creation
# [Pass] test_telemetry_checksum_valid
# [Pass] test_telemetry_checksum_invalid
# -----------------------
# 3 Tests 0 Failures 0 Ignored
