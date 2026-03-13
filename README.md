## 🖥️ ESP32-C3 E-Ink Environment Clock

**eink-environment-clock** is a minimal ESP32-C3 based E-Ink desk clock that displays time and indoor air data.
It combines a low-power E-Ink screen with CO₂, temperature, humidity, and pressure sensing, making it useful as both a clock and a compact environment monitor.

## 🧩 PCB

PCB design and assembly are illustrated with the following reference images:

```bash
images/
├── assembled_board_image.png   # photo of the assembled and soldered board
├── circuit_image.png           # circuit overview
├── pcb_back_image.png          # PCB back side
├── pcb_front_image.png         # PCB front side
└── schematic_image.png         # schematic diagram
```

### Circuit

![Circuit](images/circuit_image.png)

### Assembled PCB
![Assembled PCB](images/assembled_board_image.png)

## 💾 Sketch

The firmware for this project is located in:

```bash
sketch/
└── eink-environment-clock.ino
```

This sketch is responsible for:

* connecting and initializing all peripherals,
* synchronizing time via Wi-Fi and NTP,
* reading data from the environmental sensors,
* updating the E-Ink display,
* showing current time, CO₂ level, temperature, humidity, and pressure.

## 📚 Libraries

This project uses the following Arduino libraries:

* **GxEPD2** — by Jean-Marc Zingg
  Used to drive the E-Ink display.

* **Adafruit GFX Library** — by Adafruit
  Provides graphics primitives and text rendering for the display.

* **Adafruit BME280 Library** — by Adafruit
  Used to read temperature, humidity, and pressure from the BME280 sensor.

* **RTClib** — by Adafruit
  Used to communicate with the DS3231 real-time clock.

* **MHZ19** — by WifWaf
  Used to communicate with the MH-Z19 CO₂ sensor.

## ⚠️ Known Issues

* Holes for the **MH-Z19** are too small.
* There are several issues with the mounting holes in general. Some modules are placed too close to the mounting holes, which sometimes makes assembly and fastening inconvenient.
* The wrong header type was selected for external modules such as the display. The headers I used turned out to be smaller than the ones I had, so I ended up soldering the connection cable directly.

## 📜 License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details.
