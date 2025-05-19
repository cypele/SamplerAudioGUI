## SamplerAudioGUI
*Graphical, touch‑driven audio sampler for the STM32F769I‑DISCO*

SamplerAudioGUI lets you trigger, record, save, and play back audio samples in real time via the onboard touchscreen. Built on STM32CubeIDE + STM32 HAL + TouchGFX + FreeRTOS, it supports loading WAV files from an SD card, and leverages SDRAM for video buffers to provide a clean, button‑grid interface on the 7‑inch display.

---

## Table of Contents
1. [Features](#features)
2. [Hardware Requirements](#hardware-requirements)
3. [Software Requirements](#software-requirements)
4. [Directory Structure](#directory-structure)
5. [Architecture](#architecture)
6. [How It Works](#how-it-works)
7. [Setup & Build](#setup--build)
8. [Usage](#usage)
9. [Configuration](#configuration)
10. [Troubleshooting](#troubleshooting)
11. [Future Improvements](#future-improvements)
12. [Demo Video](#demo-video)
13. [License](#license)

---

## Features
- **Touch‑driven GUI**  
  Custom TouchGFX interface displaying a grid of sample‑trigger buttons.
- **Recording & Playback**  
  Record audio in real time and immediately replay.
- **Flexible Storage**  
  Store samples in SD card (FATFS).
- **High‑Quality Audio**  
  Uses SAI/I²S peripheral with CODEC for stereo PCM playback (44.1 kHz & 48 kHz).
- **FreeRTOS‑powered**  
  Five tasks manage audio, storage, GUI, and initialization.

---

## Hardware Requirements
- STM32F769I‑DISCO development board
- Micro‑USB cable (for ST‑Link debugging & power)
- MicroSD card
- Headphones connected to on‑board audio jack
- Microphine connected to on‑board audio jack

---

## Software Requirements
- STM32CubeIDE (v1.14 or later)
- STM32CubeMX (optional, for `.ioc` edits)
- ST‑Link/V2 driver (for flashing & debugging)
- TouchGFX Designer v4.x (GUI project files included)
- FreeRTOS (included via STM32CubeMX Middleware)
- FATFS (included via STM32CubeMX Middleware)

---

## Directory Structure
```plaintext
SamplerAudioGUI/
├── Core/
│   ├── Inc/        # Application headers (tasks, callbacks)
│   └── Src/        # Application and FreeRTOS code
├── Drivers/        # STM32 HAL + BSP (audio, SD, display)
├── Middlewares/    # FreeRTOS, FATFS, LIBJPEG, TouchGFX
├── TouchGFX/       # TouchGFX Designer project & assets
├── STM32CubeIDE/   # .project, .cproject & debug settings
├── STM32F769I_DISCO.ioc # CubeMX hardware config
├── AudioSamples/   # Example .wav files (flashed)
├── changelog.txt   # Project change history
└── LICENSE         # MIT License
````

---

## Architecture

This project runs under FreeRTOS with these primary tasks:

* **RecAndPlayTask**
  Handles recording from ADC/codec and playback operations.
* **SaveAndReadTask**
  Manages SD card reads/writes via FATFS.
* **TouchGFXTask**
  Drives GUI rendering and input processing.
* **VideoTask**
  Streams GUI frames; uses SDRAM for frame buffers.
* **AudioInitTask**
  Initializes codec and configures SAI and I²S peripherals.

> ▶ In the future, **AudioInitTask** should be merged into **RecAndPlayTask** using an I²C mutex to streamline initialization and avoid redundant code.

---

## How It Works

### Sample Storage

**SD Card**: Store bulk samples on FAT32-formatted SD cards.

### GUI & Input

* TouchGFX draws buttons and handles touch events via command queues.
* Button presses send messages to FreeRTOS queues.
* Tasks listen to commands.
* GUI:

![image](https://github.com/user-attachments/assets/6bfb3e0c-f98a-4f57-a66b-c6a995ba4be5)


### Audio Playback & Recording

* **Recording**: Audio sampled by the codec into a DMA buffer, then enqueued via FreeRTOS queues to the SaveAndReadTask for persistence.
* **Playback**: WAV data dequeued from the SaveAndReadTask into a playback buffer, then streamed via DMA to the SAI/I²S peripheral for output.

---

## Setup & Build

1. **Clone the repo**:

   ```bash
   git clone https://github.com/cypele/SamplerAudioGUI.git
   cd SamplerAudioGUI
   ```
2. **Import into STM32CubeIDE**:

   * File → Import → Existing Projects into Workspace
   * Select the `SamplerAudioGUI` root directory
3. **(Re)Generate with CubeMX**:

   * Open `STM32F769I_DISCO.ioc`
   * Enable FreeRTOS and middleware
   * Generate code → Overwrite project
4. **Build & Flash**:

   * Build target `Debug` or `Release`
   * Connect board via ST‑Link
   * Flash and start via Debug

---

## Usage

* **Record**: Press the record button; Led toggles to indicate that DMA is working; Press Stop recording button
* **Playback**: Press Play button
* **Sample management**: Need improvmed by now only last sample is played
---

## Configuration

| Parameter      | Default        | Notes                                    |
| -------------- | -------------- | ---------------------------------------- |
| Audio format   | WAV PCM 16‑bit | Mono or stereo supported                 |
| Sampling rate  | 44.1 kHz       | Change via `frequency` in AudioInit Task |

---

## Troubleshooting

* **DMA reuse artifacts**:
  Buffers aren’t cleared before reuse; residual samples may play.
  *Workaround*: Manually zero buffers before re-recording.

* **Random hangs**:
  Possible task-priority misconfiguration or RAM fragmentation.
  *Check*: Verify FreeRTOS task priorities and heap settings.

* **SD not mounting**:
  Use FAT32 ≤32 GB and initialize SDMMC in 1‑bit mode.

---

## Future Improvements

* Merge **AudioInitTask** into **RecAndPlayTask** with I²C mutex.
* Folder browsing and multi‑bank UI.
* On‑device real‑time sample processing (pitch shift, tempo).
* USB-MIDI host support.
* Improve sample management.

---

## Demo Video

Watch SamplerAudioGUI in use:

[![Watch the demo on YouTube](https://img.youtube.com/vi/SqMQi4vOvKg/0.jpg)](https://youtube.com/shorts/SqMQi4vOvKg)

---
## License

Licensed under the MIT License. See [LICENSE](LICENSE) for details.

```
```
