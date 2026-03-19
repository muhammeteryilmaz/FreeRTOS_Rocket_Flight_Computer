# FreeRTOS_Rocket_Flight_Computer

A real-time rocket flight computer built on **STM32F446ZE (Nucleo-144)** using **FreeRTOS**, designed for avionics applications. The system handles sensor fusion, telemetry, and flight control using a multi-task architecture.

---

## Hardware

| Component | Model | Interface |
|---|---|---|
| Microcontroller | STM32F446ZE (Nucleo-144) | — |
| IMU | MPU6050 (Accelerometer + Gyroscope) | I2C1 |
| Barometer | BME280 (Pressure / Temperature / Humidity) | I2C2 |
| LoRa Transmitter | E32 900T20D | UART4 |
| LoRa Receiver | E32 900T20D | UART5 |
| Servo Motor | Standard PWM Servo | TIM1 CH1 (PE9) |

---

## System Architecture

```
┌─────────────┐     xIMUQueue     ┌──────────────┐
│  IMUTask    │ ────────────────► │              │
│ (MPU6050)   │                   │ ControlTask  │ ──► Servo Motor
└─────────────┘                   │              │
                                  └──────┬───────┘
┌─────────────┐     xBMEQueue            │ xLoRaQueue
│  BMETask    │ ────────────────►        ▼
│ (BME280)    │             ┌────────────────────┐
└─────────────┘             │  TelemetryTTask    │ ──► LoRa TX (UART4 DMA)
                            └────────────────────┘

                            ┌────────────────────┐
                            │  TelemetryRTask    │ ◄── LoRa RX (UART5 DMA)
                            └────────────────────┘
```

---

## Task Overview

| Task | Priority | Stack | Description |
|---|---|---|---|
| IMUTask | 3 | 1024 | Reads MPU6050 gyroscope and accelerometer data |
| BMETask | 2 | 1024 | Reads BME280 pressure, temperature, humidity |
| ControlTask | 4 | 1024 | Processes sensor data, triggers servo on apogee detection |
| TelemetryTTask | 2 | 1024 | Sends telemetry data via LoRa (DMA) |
| TelemetryRTask | 3 | 1024 | Receives telemetry data via LoRa (DMA) |

---

## Features

- **Multi-task RTOS architecture** — FreeRTOS with queue-based inter-task communication
- **IMU driver** — Custom MPU6050 driver with configurable gyro (±2000°/s) and accelerometer (±16g) ranges
- **Barometer driver** — Custom BME280 driver with full calibration and compensation
- **LoRa telemetry** — 900MHz bidirectional communication via E32 modules with DMA-based UART
- **Payload deployment** — Pressure-based apogee detection triggers servo motor
- **DMA transfers** — CPU-offloaded UART TX/RX for telemetry

---

## Flight Logic

Apogee detection is based on barometric pressure change:

```c
if ((pressure - prevPressure) <= -5.00f) {  // 5 hPa drop
    ServoRun();                              // deploy payload
    vTaskDelay(pdMS_TO_TICKS(1000));
    ServoStop();
}
```

---

## LoRa Configuration

| Parameter | Value |
|---|---|
| Frequency | 900 MHz |
| Output Power | 20 dBm (100mW) |
| Baud Rate | 9600 |
| Channel | 7 (907 MHz) |
| Address | 0x0010 |
| Mode | Normal (M0=LOW, M1=LOW) |

Telemetry packet structure:

```c
typedef struct {
    float pressure;   // hPa
    float gX;         // °/s
    float gY;         // °/s
    float gZ;         // °/s
} LoRaBuffer_t;       // 16 bytes
```

---

## Servo Configuration

| Parameter | Value |
|---|---|
| Timer | TIM1 CH1 (PE9) |
| Frequency | 50 Hz |
| Prescaler | 167 |
| ARR | 19999 |
| Min position | CCR = 1000 |
| Center | CCR = 1500 |
| Max position | CCR = 2000 |

---

## IOC Configuration

- **Clock**: HSI 16MHz → PLL → 168MHz system clock
- **HAL Timebase**: TIM6 (SysTick reserved for FreeRTOS)
- **FreeRTOS Heap**: 30720 bytes (heap_4)
- **I2C1**: MPU6050 — SDA: PB9, SCL: PB8
- **I2C2**: BME280 — SDA: PB11, SCL: PB10
- **UART4**: LoRa TX — DMA TX enabled
- **UART5**: LoRa RX — DMA RX enabled

---

## Project Structure

```
FreeRTOS_Rocket_Flight_Computer/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   └── FreeRTOSConfig.h
│   └── Src/
│       └── main.c          # All tasks, drivers, and logic
├── Drivers/
│   └── STM32F4xx_HAL_Driver/
├── Middlewares/
│   └── FreeRTOS/
└── FreeRTOS_Rocket_Flight_Computer.ioc
```

---

## Getting Started

1. Clone the repository:
```bash
git clone https://github.com/muhammeteryilmaz/FreeRTOS_Rocket_Flight_Computer.git
```

2. Open with **STM32CubeIDE**

3. After any `.ioc` regeneration, manually restore:
```c
// Core/Inc/FreeRTOSConfig.h
#define configTOTAL_HEAP_SIZE ((size_t)30720)
```

4. Build and flash to Nucleo-F446ZE

---

## Roadmap

- [ ] GPS module integration
- [ ] SD card data logging
- [ ] Kalman filter for IMU sensor fusion
- [ ] PID control for stabilization
- [ ] Flight state machine (IDLE → LAUNCH → ASCENDING → APOGEE → DESCENDING → LANDED)
- [ ] OTA firmware update via LoRa

---

## Author

**Muhammet Eryılmaz**
- GitHub: [@muhammeteryilmaz](https://github.com/muhammeteryilmaz)
- LinkedIn: [muhammet-eryılmaz](https://www.linkedin.com/in/muhammet-eryılmaz-5535a522a)
