# STM32 Biomedical DSP Monitor

A high-performance medical monitoring system developed on the STM32 platform. This project utilizes real-time Digital Signal Processing (DSP) to analyze heart rate frequency from a pulse sensor.

## Key Technical Features
* **Frequency Domain Analysis:** Implements a 1024-point Fast Fourier Transform (FFT) using the **ARM CMSIS-DSP** library to accurately calculate BPM.
* **Signal Conditioning:** Uses a **Hamming Window** and Mean-Removal algorithms to reduce spectral leakage and DC offset before processing.
* **Hardware Efficiency:** Utilizes **DMA (Direct Memory Access)** for ADC sampling to minimize CPU overhead, allowing for high-frequency processing.
* **Safety Systems:** Integrated state-machine for SpO2 alarm thresholds, utilizing PWM-driven buzzers and GPIO-mapped LED indicators.
* **Custom UI:** Optimized SSD1306 OLED driver with custom bitmap animations for real-time visual feedback.

## Hardware Stack
* **MCU:** STM32 (ARM Cortex-M series)
* **Display:** SSD1306 OLED (I2C)
* **Sensors:** Analog Pulse Sensor (ADC)
* **Indicators:** Active Piezo Buzzer (PWM) & Status LEDs