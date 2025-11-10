# 🧠 BMX160 Linux Device Driver (I²C)

A custom Linux kernel device driver for the **BMX160 9-axis sensor** (Accelerometer + Gyroscope + Magnetometer) integrated over the **I²C interface** on the **Raspberry Pi 4B**.  
This project demonstrates low-level driver development, device-tree binding, and kernel-level debugging using `dmesg`.

---

## ⚙️ Hardware Setup

| Component | Description |
|------------|-------------|
| **Board** | Raspberry Pi 4 Model B |
| **Sensor** | Bosch BMX160 (I²C) |
| **Interface** | I²C (400 kHz) |
| **OS** | Raspberry Pi OS (Bookworm / Kernel 6.12.x) |

**Wiring:**

| BMX160 Pin | Raspberry Pi Pin |
|-------------|------------------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 2 (I²C SDA) |
| SCL | GPIO 3 (I²C SCL) |

---

## 🧩 Driver Overview

This driver implements:
- I²C-based register read/write operations   
- Probe and Remove functions for device lifecycle  
- Reading the **Chip ID (0xD8)** for validation  
- Kernel-level logging using `dev_info()` and `dmesg`  
