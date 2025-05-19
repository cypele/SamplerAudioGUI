SamplerAudioGUI
A graphical, touch‑driven audio sampler for the STM32F769I‑DISCO
SamplerAudioGUI lets you trigger and play back preloaded audio samples in real time via the onboard touchscreen. Built on STM32CubeIDE + STM32 HAL + TouchGFX, it supports loading WAV files from internal flash or an SD card, and provides a clean, button‑grid interface on the 7‑inch display.

Table of Contents
Features

Hardware Requirements

Software Requirements

Directory Structure

How It Works

Setup & Build

Usage

Configuration

Troubleshooting

Future Improvements

Contributing

License

Features
Touch‑driven GUI: Custom TouchGFX interface displaying a grid of sample‑trigger buttons.

Flexible storage: Play WAV samples from onboard flash or SD card (FATFS).

High‑quality audio: Uses SAI/I²S peripheral with CODEC for stereo PCM playback (44.1 kHz & 48 kHz).

Modular drivers: STM32 HAL + BSP layers; easily swap audio codec or storage backend.

Configurable banks: Support for multi‑bank sample layouts and dynamic loading.

Hardware Requirements
STM32F769I‑DISCO development board

Micro‑USB cable (for ST‑Link debugging & power)

(Optional) MicroSD card & adapter for external samples

Powered speakers or headphones connected to on‑board audio jack

Software Requirements
STM32CubeIDE (v1.14 or later)

STM32CubeMX (optional, for .ioc edits)

ST‑Link/V2 driver (for flashing & debugging)

TouchGFX Designer v4.x (GUI project files included)

Directory Structure
graphql
Kopiuj
Edytuj
SamplerAudioGUI/
├── Core/
│   ├── Inc/        # Application headers (main, audio ctrl, UI callbacks)
│   └── Src/        # Application code
├── Drivers/        # STM32 HAL + BSP (audio, SD, display)
├── Middlewares/    # FATFS, LIBJPEG, TouchGFX framework
├── TouchGFX/       # TouchGFX Designer project & assets
├── STM32CubeIDE/   # .project, .cproject & debug settings
├── STM32F769I_DISCO.ioc # CubeMX hardware config
├── AudioSamples/   # Example .wav files (flash‑burned)
├── changelog.txt   # Project change history
└── LICENSE         # MIT License
How It Works
Sample storage

Preload critical samples into internal flash (via flash loader).

Place bulk samples on SD card formatted as FAT32.

GUI & input

TouchGFX draws a grid of buttons corresponding to sample slots.

On touch, the UI layer invokes an audio‑playback callback.

Audio playback

Audio samples are decoded if necessary (LIBJPEG or raw WAV PCM).

Data streamed to the audio codec over SAI/I²S via DMA for glitch‑free playback.

Setup & Build
Clone the repo

bash
Kopiuj
Edytuj
git clone https://github.com/cypele/SamplerAudioGUI.git
cd SamplerAudioGUI
Import into STM32CubeIDE

File → Import → Existing Projects into Workspace

Select the SamplerAudioGUI root directory

(Re)Generate with CubeMX

Open STM32F769I_DISCO.ioc in STM32CubeMX

Tweak pinouts or middleware as needed

Generate code → Overwrite

Build & Flash

Set build target to Debug or Release

Click the “Build” button

Connect the DISCO board via ST‑Link

Click “Debug” to program and start

Usage
Load samples: Copy .wav files into the root of your FAT32 SD card.

Sample naming: Files named SLOT01.wav, SLOT02.wav, … map to buttons 1–n.

Interacting: Tap a button on the touchscreen to play the corresponding sample.

Banks: Swipe left/right or use bank‑select buttons (if enabled) to switch sample sets.

Configuration
Parameter	Default	Notes
Audio format	WAV PCM 16‑bit	Mono or stereo supported
Sampling rate	44.1 kHz	Can be changed via AUDIO_SAMPLER_RATE_HZ
SD mount point	"/sdcard/"	Configurable in ffconf.h
Touch calibration	Pre‑calibrated	Run TouchGFX calibrator if display drifts

Troubleshooting
No display:

Ensure LCD_BACKLIGHT pin enabled in .ioc.

Check TouchGFX BSP initialization logs.

Audio click/pops:

Verify DMA priority in stm32f7xx_hal_msp.c.

Confirm sample matches configured bit‑depth and rate.

SD not mounting:

Use FAT32‑formatted card ≤32 GB.

Verify SD_Detect GPIO mapping in stm32f7xx_hal_conf.h.

Future Improvements
Multi‑bank and folder browsing in GUI

On‑device real‑time sample recording

Pitch shifting & tempo control

MIDI over USB host input for external controllers

Contributing
Fork the repo.

Create a feature branch: git checkout -b feature/YourIdea.

Commit your changes: git commit -m "Add awesome feature".

Push: git push origin feature/YourIdea.

Open a Pull Request — describe what you’ve done and why.

Please follow the existing code style, comment liberally, and add or update unit tests where appropriate.

License
This project is licensed under the MIT License.
See LICENSE for details.
