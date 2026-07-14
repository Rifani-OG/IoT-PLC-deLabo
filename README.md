# IoT-PLC-deLabo

> **deLabo Case Study Submission**  
> **Problem No. 1:** CIREn-IoT: MEMS Sensor

---

## Repository File Descriptions

*   **`dummy.ino` (Main ESP32 Firmware)**  
    This is the core firmware uploaded to the ESP32 microcontroller. It operates on a real-time operating system (FreeRTOS) to execute non-blocking multi-tasking, which includes: synthesizing three-axis vibration signals, calculating the True Time-Domain RMS, extracting the peak frequency (Peak Hz) using Fast Fourier Transform (FFT), and transmitting the encrypted payload via MQTT TLS protocol to HiveMQ Cloud.

*   **`display dummy.html` (HMI Dashboard)**  
    A static web-based user interface (HTML/CSS/JS) file. It serves as a real-time visual dashboard that receives high-speed data streams via WebSocket. The embedded script calculates the total spatial vector magnitude of the mechanical energy to provide an automated fault prediction indicator (Normal, Warning, Critical) purely on the client side.

*   **`for dummy.json` (Node-RED Gateway Flow)**  
    The JSON-based flow configuration for Node-RED, acting as the middleware gateway. Its primary functions are to subscribe to the HiveMQ Cloud broker, execute an outlier filtering algorithm to mitigate Electromagnetic Interference (EMI), and distribute the clean data to the HMI via a WebSocket network.

*   **`test1.ino` (Legacy Telemetry Base Code)**  
    The initial prototype code adapted from the telemetry system of the Rakata ITB energy-efficient vehicle team. This file represents the foundational architecture of the low-latency data transmission scheme, which was subsequently developed and refined into the final version in `dum.ino`.

*   **`fft.ino` (FFT Development Script)**  
    An experimental prototype script previously used in isolation to focus on testing the parameters of the `arduinoFFT.h` library. This file serves as a development branch record, ensuring the Hamming windowing and magnitude conversion algorithms were accurate before their full integration into the RTOS environment.
