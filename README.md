# Adaptive Sampling IoT System

## Introduction

This project demonstrates an IoT system that collects information from a sensor, performs local data analysis, and communicates an aggregated value to a nearby server. The system adapts its sampling frequency to save energy and reduce communication overhead. The IoT device is based on an ESP32 prototype board and the firmware is developed using FreeRTOS.

## Features

- Generates a synthetic signal.
- Performs Fast Fourier Transform (FFT) on the sampled signal.
- Adapts the sampling frequency based on the detected maximum frequency in the signal.
- Computes the average of the signal over a specified time window.
- Communicates the aggregated value securely to a nearby server.
- Measures energy consumption, data volume, and end-to-end latency.
- Secure Communication: Transmits data to a nearby server using MQTT over SSL.

## Input Signal

The input signal is of the form: 

\[2 \cdot \sin(2 \pi \cdot 3 \cdot t) + 4 \cdot \sin(2 \pi \cdot 5 \cdot t)\]

## Hardware Requirements

- ESP32 prototype board
- Power supply (e.g., USB cable)

## Software Requirements

- ESP-IDF
- FreeRTOS
- MQTT Broker (e.g., Eclipse Mosquitto)
- Wi-Fi connection

## Running the Project

### 1.Clone the Repository

git clone https://github.com/Shlok2801/Adaptive-Sampling-Using-ESP32S3-and-FreeRTOS.git

### 2.Setup the ESP-IDF

Setup the ESP-IDF framework on your device 

### Configure the project

Navigate to the project directory and configure the project using 'menuconfig'.
For opening the menuconfig 'idf.py menuconfig' in your ESP-IDF terminal.
Once in menuconfig:
- Navigate to Example Connection Configuration.
- Put in the credentials to the WiFi (SSID, Password).
- You also have to set the MQTT broker url which is under Example Configuration.

### Build and Flash the firmware

idf.py build
idf.py flash

### Monitor the output

idf.py monitor

## Detailed Explaination

### Signal Generation and FFT

The input signal is generated using a sum of sine functions. The FFT is performed to determine the maximum frequency component using the Cooley-Tukey FFT-algorithm, which then informs the optimal sampling frequency.

### Adaptive Sampling Frequency

The system starts with the maximum sampling frequency. After performing FFT, it adjusts the sampling frequency to twice the maximum frequency component detected to set the optimal frequency, ensuring Nyquist rate compliance.

### Aggregate Function

The average of the sampled signal over a specified time window (I have defined 5 seconds) is computed. This aggregate value is then transmitted to the MQTT server.

### Secure Communication

MQTT over SSL is used for secure data transmission.

### Performance Measurement

The system measures:

- **Energy Consumption**: Compared between oversampled and adaptively sampled scenarios. I have defined the current and the voltage to measure the consumption with respect to the time taken by the sampling processes.

- **Data Volume**: Compared between oversampled and adaptively sampled scenarios. I measure the volume of samples by samples per second * sample size * size of each sample (float).

- **End-to-End Latency**: I record the start time in the task_signal_processing before generating the signal and then I record the end time in the MQTT PUBLISH case so that I can get the time that the whole system takes to run by subtracting the start time with the end time.

## Performance Evaluation

### Energy Consumption

The energy consumption is compared and reported in the output.

### Data volume

The data volume is calculated and reported in the output.

### End-to-End Latency

The end-to-end latency from the data generation to reception by the edge server us measured and reported.


## Conclusion

The adaptive sampling appreoach demonstrates significant energy savings and reduced data transmission volume compared to the oversampling method. The system is efficient and adaptable to varying signal characteristics.

This can be said as while comparing the difference in the power consumption we can see that the oversampling consumes more than the resampled signal which is sampled at the optimal frequency also the communication head is lower.


