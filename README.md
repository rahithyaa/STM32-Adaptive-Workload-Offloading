# Adaptive Local-Cloud Workload Offloading for Embedded Robots using RTOS

**Platform:** STM32G491RE (ARM Cortex-M4, 170 MHz)  
**OS:** FreeRTOS  
**Language:** Embedded C, Python  
**Institution:** VIT Chennai — B.Tech ECE Final Year Project (April 2026)

---

## Overview

A real-time adaptive workload offloading system for vibration-based 
fault detection on STM32G491RE Cortex-M4 microcontroller using 
FreeRTOS. The system dynamically decides whether to execute FFT 
computation locally or offload to a Python edge gateway based on 
a real-time energy cost model.

---

## Key Results

| Metric | Value |
|--------|-------|
| Fault detection latency (Bare-metal) | ~57 ms |
| Fault detection latency (FreeRTOS) | 10–20 µs |
| Improvement | ~3000–4000x |
| Energy per window (Bare-metal) | ~5.01 mJ |
| Energy per window (Adaptive offload) | ~2.03 mJ |
| Energy saving vs baseline | ~59% |

---

## System Architecture

Three configurations implemented and compared:

- **Case A:** Bare-metal baseline (no RTOS)
- **Case B:** FreeRTOS local processing (task separation)
- **Case C:** FreeRTOS + adaptive offloading (energy-aware)

---

## FreeRTOS Task Design

| Task | Priority | Responsibility |
|------|----------|----------------|
| SafetyTask | High | Peak detection, fault flag, LED |
| ProcessingTask | Normal | FFT / offload decision |
| DecisionTask | Normal | Energy cost comparison |
| MetricsTask | Normal | CSV logging via UART |

---

## Hardware

- **MCU:** STM32G491RE Nucleo (ARM Cortex-M4, 170 MHz)
- **Communication:** UART at 460800 baud
- **DSP:** ARM CMSIS-DSP v5 (arm_rfft_fast_f32)
- **Timer:** TIM2 @ 10 kHz sampling ISR
- **Edge Gateway:** Python script on host PC via USB-UART

---

## Energy Cost Model
E_local   = T_local × I_CPU  × V  
E_offload = T_TX   × I_UART × V + T_cloud × I_idle × V

- I_CPU  = 26.5 mA (STM32G491 datasheet)
- I_idle =  6.45 mA  
- V      =  3.3 V

Decision: Offload when E_offload < E_local and no fault active.
Online calibration (calib_b) corrects timing predictions at runtime.
---

## Communication Protocol (UART Frame)

| Field | Size | Value |
|-------|------|-------|
| START | 1 byte | 0xAA |
| DATA | 1024 bytes | 256 × float32 samples |
| END | 1 byte | 0xBB |
| Response FREQ | 4 bytes | float32 Hz |
| Response T_COMPUTE | 4 bytes | float32 ms |
| CHECKSUM | 1 byte | XOR |

---

## Tech Stack

`Embedded C` `FreeRTOS` `STM32CubeIDE` `ARM CMSIS-DSP`  
`UART` `DWT Cycle Counter` `Python` `NumPy` `FFT`

---

## Project Structure
├── Core/
│   ├── main.c              # Initialization, ISR, clock config
│   ├── safety_task.c       # SafetyTask - fault detection
│   ├── processing_task.c   # ProcessingTask - FFT/offload
│   ├── cost_model.c        # Energy cost model + calibration
├── Gateway/
│   └── gateway.py          # Python edge gateway (NumPy FFT)
└── README.md


---

## Results Summary

- RTOS reduces worst-case fault latency by **3000–4000x**
- Adaptive offloading saves **58–59% energy** vs baseline
- Online calibration converges within **~200 windows**
- Offload success rate: **~96.7%** in steady state
