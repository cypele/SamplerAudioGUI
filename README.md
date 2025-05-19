```markdown
## SamplerAudioGUI
*Graphical, touch‑driven audio sampler for the STM32F769I‑DISCO*

SamplerAudioGUI lets you trigger and play back preloaded audio samples in real time via the onboard touchscreen. Built on STM32CubeIDE + STM32 HAL + TouchGFX, it supports loading WAV files from internal flash or an SD card, and provides a clean, button‑grid interface on the 7‑inch display.

---

## Table of Contents
1. [Features](#features)
2. [Hardware Requirements](#hardware-requirements)
3. [Software Requirements](#software-requirements)
4. [Directory Structure](#directory-structure)
5. [How It Works](#how-it-works)
6. [Setup & Build](#setup--build)
7. [Usage](#usage)
8. [Configuration](#configuration)
9. [Troubleshooting](#troubleshooting)
10. [Future Improvements](#future-improvements)
11. [License](#license)

---

## Features
- **Touch‑driven GUI**  
  Custom TouchGFX interface displaying a grid of sample‑trigger buttons.
- **Flexible Storage**  
  Play WAV samples from onboard flash or SD card (FATFS).
- **High‑Quality Audio**  
  Uses SAI/I²S peripheral with CODEC for stereo PCM playback (44.1 kHz & 48 kHz).
- **Modular Drivers**  
  STM32 HAL + BSP layers; easily swap audio codec or storage backend.
- **Configurable Banks**  
  Support for multi‑bank sample layouts and dynamic loading.

---

## Hardware Requirements
- STM32F769I‑DISCO development board
- Micro‑USB cable (for ST‑Link debugging & power)
- (Optional) MicroSD card & adapter for external samples
- Powered speakers or headphones connected to on‑board audio jack

---

## Software Requirements
- STM32CubeIDE (v1.14 or later)
- STM32CubeMX (optional, for `.ioc` edits)
- ST‑Link/V2 driver (for flashing & debugging)
- TouchGFX Designer v4.x (GUI project files included)

---

## Directory Structure
```

SamplerAudioGUI/
├── Core/
│   ├── Inc/        # Application headers (main, audio ctrl, UI callbacks)
│   └── Src/        # Application code
├── Drivers/        # STM32 HAL + BSP (audio, SD, display)
├── Middlewares/    # FATFS, LIBJPEG, TouchGFX framework
├── TouchGFX/       # TouchGFX Designer project & assets
├── STM32CubeIDE/   # .project, .cproject & debug settings
├── STM32F769I\_DISCO.ioc # CubeMX hardware config
├── AudioSamples/   # Example .wav files (flash‑burned)
├── changelog.txt   # Project change history
└── LICENSE         # MIT License

````

---

## How It Works

### Sample Storage
1. **Flash**: Preload critical samples into internal flash via a flash loader.
2. **SD Card**: Place bulk samples on an SD card formatted as FAT32.

### GUI & Input
- TouchGFX draws a grid of buttons corresponding to sample slots.
- On touch, the UI layer triggers the audio‑playback callback.

### Audio Playback
- Decode samples if necessary (LIBJPEG or raw WAV PCM).
- Stream data to the audio codec over SAI/I²S via DMA for glitch‑free playback.

---

## Setup & Build
1. **Clone the repo**:
   ```bash
   git clone https://github.com/cypele/SamplerAudioGUI.git
   cd SamplerAudioGUI
````

2. **Import into STM32CubeIDE**:

   * File → Import → Existing Projects into Workspace
   * Select the `SamplerAudioGUI` root directory
3. **(Re)Generate with CubeMX**:

   * Open `STM32F769I_DISCO.ioc` in STM32CubeMX
   * Adjust pinouts or middleware as needed
   * Generate code → Overwrite project files
4. **Build & Flash**:

   * Set build target to `Debug` or `Release`
   * Click **Build**
   * Connect the DISCO board via ST‑Link
   * Click **Debug** to program and start

---

## Usage

* **Load samples**: Copy `.wav` files into the root of your FAT32 SD card.
* **Sample naming**: Name files `SLOT01.wav`, `SLOT02.wav`, … to map to buttons 1–n.
* **Playback**: Tap a button on the touchscreen to play the corresponding sample.
* **Bank navigation**: Swipe left/right or use bank‑select controls to switch sample sets.

---

## Configuration

| Parameter         | Default        | Notes                              |
| ----------------- | -------------- | ---------------------------------- |
| Audio format      | WAV PCM 16‑bit | Mono or stereo supported           |
| Sampling rate     | 44.1 kHz       | Change via `AUDIO_SAMPLER_RATE_HZ` |
| SD mount point    | `/sdcard/`     | Configurable in `ffconf.h`         |
| Touch calibration | Pre‑calibrated | Run TouchGFX calibrator if needed  |

---

## Troubleshooting

* **SD not mounting**:

  * Use FAT32‑formatted cards ≤32 GB.
  * Initilize SDMMC with 1 bit wide bus

---

## Future Improvements

* Multi‑bank and folder browsing in GUI
* Pitch shifting & tempo control
* MIDI over USB host input for external controllers

---

## License

Licensed under the MIT License. See [LICENSE](LICENSE) for details.

```
```
