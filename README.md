# Brushed to RC PWM Converter (STC8)

A lightweight signal translator that converts **brushed motor H-bridge PWM signals** into **RC/ESC servo PWM signals**.

Designed for **low-cost STC8 microcontrollers**, suitable for toys, robotics, and custom control systems.

---

## ✨ Features

- Convert **brushed motor PWM → RC servo PWM (1000–2000us)**
- Support:
  - **Dual-direction input (A/B)** → bidirectional ESC signal
  - **Single-direction input** → throttle output
- Up to:
  - **3 channels (single direction)**  
  - **2 channels (bidirectional)**

- Built-in support for:
  - Throttle scaling
  - Direction control
  - Signal mixing

- Customizable:
  - Throttle curve
  - Deadzone
  - Output range
  - Mixing logic

- Runs on **STC8 (8051)**  
- Tested up to **35MHz internal RC**

---

## 📂 Files

### `stc8g.c`
Core driver and timing implementation.

- Timer-based PWM generation
- Input sampling
- Core conversion logic

---

### `main.c`
**Stable version (recommended for toys / low-end applications)**

- 100Hz RC output
- Discrete throttle levels (0–60%)
- Reduced jitter
- Better stability on noisy signals

Use this if:
- You want **stable behavior**
- Input signal quality is poor
- Used in low-cost toy systems

---

### `main_v1.c`
**Linear version (higher performance, less filtering)**

- Linear mapping (better responsiveness)
- Full-range throttle
- Faster response

Trade-offs:
- May introduce **jitter**
- More sensitive to input noise

Use this if:
- You want **better control precision**
- Your input PWM is clean

---

## 🔧 Functionality

### Signal Conversion

| Input (Brushed) | Output (RC PWM) |
|------|----------------|
| A=0, B=0 | 1500us (stop) |
| A PWM | 1500 → 2000us |
| B PWM | 1500 → 1000us |

---

### Mixing Support

Supports custom mixing logic:

- **Tank steering (differential drive)**
  - Throttle + Steering → Left / Right output

- **Flying wing**
  - Elevator + Aileron → Elevon mix

- **Custom mixes**
  - Fully programmable

---

## ⚙️ Hardware

- MCU: **STC8G1K08A (or similar STC8 series)**
- Internal RC clock (up to 35MHz)
- Minimal external components

---

## 🧪 Typical Use Cases

- Toy car controller upgrade
- Converting brushed driver boards to brushless ESC control
- DIY robots
- Differential drive systems
- RC mixing experiments

---

## 🛠 Customization

You can modify:

- Throttle curve
- Output frequency (50Hz / 100Hz / 300Hz)
- Deadzone
- Response smoothing
- Mixing algorithm

---

## ⚠️ Notes

- Default RC output:
  - 1000–2000us pulse width
- Ensure proper **ground reference** between input and output systems
- Some ESCs require:
  - Neutral calibration
  - Specific startup sequence

---

## 🚀 Future Ideas

- ADC capture (higher capability just in case PWM decoding doesn't working)
- Configurable via UART to avoid re-download the full program each time
- EEPROM profile storage
- Auto calibration

---

## 📜 License

Free for personal user

---

## 👤 Author

Open-source project by Yang

---
