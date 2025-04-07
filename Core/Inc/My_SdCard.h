/*
 * My_SdCard.c
 *
 *  Created on: Apr 5, 2025
 *      Author: adamc
 */

#ifndef INC_MY_SDCARD_C_
#define INC_MY_SDCARD_C_



#include "stdint.h"

#pragma pack(push, 1)  // Wymusza brak paddingu (wyrównania struktury)
typedef struct {
    char riff[4];        // "RIFF"
    uint32_t overall_size;     // Rozmiar całego pliku - 8 bajtów
    char wave[4];         // "WAVE"
    char fmt_chunk_marker[4];    // "fmt "
    uint32_t length_of_fmt; // 16 dla PCM
    uint16_t format_type;   // 1 = PCM
    uint16_t channels;   // Liczba kanałów (1 = mono, 2 = stereo)
    uint32_t sample_rate;    // 44100, 48000 itd.
    uint32_t byterate;      // sampleRate * numChannels * bitsPerSample/8
    uint16_t block_align;    // numChannels * bitsPerSample/8
    uint16_t bits_per_sample; // 8, 16, 24, 32
    char data_chunk_header[4];    // "data"
    uint32_t data_size; // Rozmiar danych audio (bez nagłówka)
} WAV_Header;
#pragma pack(pop)




#define CHUNK_SIZE_BYTES 4096

#define SAMPLE_RATE     44100   // Częstotliwość próbkowania 48 kHz
#define BITS_PER_SAMPLE 16      // Rozdzielczość 16-bitowa
#define NUM_CHANNELS    1       // 1 = mono, 2 = stereo
#define DURATION_SEC    1       // Czas nagrania w sekundach

#endif /* INC_MY_SDCARD_C_ */
