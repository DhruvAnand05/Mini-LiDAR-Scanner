# 🌀 Time-of-Flight Spatial Mapping System – COMP ENG 2DX3 (McMaster University)

## 📖 Project Overview  
This project presents a **low-cost, embedded LIDAR system** designed for **3D spatial mapping** of indoor environments, developed for McMaster University’s **COMP ENG 2DX3** course.  
It integrates a **VL53L1X Time-of-Flight sensor**, **stepper motor**, and **MSP-EXP432E401Y microcontroller** to capture and visualize spatial data using **MATLAB**.

---

## 🛠️ Tools & Components  
- **MSP-EXP432E401Y Microcontroller** – ARM Cortex-M4F, 120MHz, 1MB Flash  
- **VL53L1X ToF Sensor** – 940nm laser, 400cm max range, 50Hz sampling  
- **28BYJ-48 Stepper Motor** – 512 steps/rev, 11.25° resolution  
- **ULN2003 Driver Board** – Motor control interface  
- **MATLAB** – Serial communication & 3D visualization  
- **I2C & UART Protocols** – Sensor and PC communication  

---

## 🔍 Design Details  
- **Measurement Principle:** Photon emission and reflection time used to calculate distance  
- **Rotation Logic:** 360° sweep in 11.25° increments → 32 measurements per layer  
- **Depth Simulation:** Each full rotation simulates 20cm movement in X-axis  
- **Coordinate Conversion:** Polar → Cartesian using trigonometric functions  
- **Visualization:** 3D scatterplot with layered and angular connections  

---

## 📊 Simulation & Testing  
### ✅ Functional Testing  
- Verified full 360° sweep with accurate distance capture  
- LEDs indicate sensor activation, measurement, and data transmission  

### 📉 Performance Metrics  
- **Bus Speed:** 20MHz (configured via PLL)  
- **UART Baud Rate:** 115200 BPS  
- **Minimum Motor Delay:** ~2ms between phases  
- **Sensor Resolution:** ±1mm quantization error  

### ⏱️ Visualization Output  
- MATLAB plots a **helical 3D structure** representing scanned space  
- Points connected by black (in-layer) and blue (inter-layer) lines  

---

## 🚀 Key Learnings  
- Embedded system integration of **sensor, motor, and microcontroller**  
- Importance of **timing and precision** in spatial data acquisition  
- Real-world limitations of **floating-point math** and **quantization errors**  
- Practical experience in **serial communication** and **data visualization**  

---

## 📚 References  
1. COMP ENG 2DX3 Lab Guide – *Embedded Systems & LIDAR*  
2. VL53L1X Datasheet – *STMicroelectronics*  
3. MSP-EXP432E401Y Documentation – *Texas Instruments*  
4. MATLAB Serial Communication Docs  

---

✍️ **Author:** Dhruv Anand  
📅 **Date:** April 2025  
📄 *2DX3 Final Report – Version 4*
