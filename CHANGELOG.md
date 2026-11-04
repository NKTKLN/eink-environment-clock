## 0.2.0 (2026-11-04)

### Feat

- **notify**: separate permanent and temporary notifications

### Perf

- **wifi**: disable modem sleep to stop stalled page loads

## 0.1.0 (2026-11-04)

### Feat

- **diag**: log heap and task stack headroom
- **ui**: show the address for 30 seconds by default, log screen messages
- **ui**: configurable address notice, remove clears system notices
- **web**: add update controls and developer tools tab
- **logs**: keep recent device log in ram
- **update**: check github releases and install updates over https
- **ui**: keep the clock running when the saved network is lost
- **web**: add web interface with display editor and setup flow
- **web**: add http api, screen preview and ota updates
- **mqtt**: publish readings with home assistant discovery
- **wifi**: add setup access point with captive portal
- **ui**: add adaptive layouts, notifications and setup screen
- **settings**: store user settings in nvs as json
- **canvas**: add utf-8 text rendering and cyrillic glyphs
- **case**: update case design
- **firmware**: restore classic 5x7 font from the arduino version
- **firmware**: add clock ui and application logic
- **firmware**: add wifi station and sntp time sync
- **firmware**: add 1bpp canvas with bitmap fonts and icons
- **firmware**: add ssd1680 e-paper driver with anti-ghosting refresh policy
- **firmware**: scaffold esp-idf platformio project for esp32-c3 super mini
- **sketch**: Add a flag to skip the display refresh
- **sketch**: Add dark theme
- **case**: Add case model
- **sketch**: Add CO2 LED alert
- **sketch**: Add v1 sketch
- **pcb**: Add images and update R8
- **pcb**: Add pcb
- **cirkit-image**: Add an electrolytic capacitor to the MH-Z19B for stable working

### Fix

- **update**: treat a repository without releases as up to date, keep debug logs
- **web**: treat the default http port as equal in origin checks
- **display**: retry with a full refresh after a failed panel update
- **wifi**: give the event task enough stack and log the association
- **time**: set system time from the rtc and keep it across time zone changes
- **web**: reject cross-site and dns rebinding requests
- **settings**: zero metric padding and test settings across updates
- **web**: restore spacing between navigation and content
- **firmware**: raise socket and stack limits, delay restarts past refresh
- **mqtt**: reject malformed threshold and command payloads
- **web**: redirect only clients of the setup access point
- **ui**: final vertical alignment of clock and sensor blocks
- **display**: map source outputs from S8 to correct 8px panel shift
- **ui**: shift clock and sensor blocks ~20px down, tighten sensor rows
- **sketch**: trying to fix ghosting
- **sketch**: Update settings
- **sketch**: update logic and timing intervals
- **sensor-display**: skip e-paper refresh when data and time are unchanged
- **case**: New version (more durable)

### Refactor

- **web**: fix design and accessibility issues, add responsive layout
- **firmware**: move wifi credentials to untracked secrets header


- remove legacy arduino sketch superseded by esp-idf firmware
