/*
 * My_SdCard.c
 *
 *  Created on: Apr 5, 2025
 *      Author: adamc
 */


#include "My_SdCard.h"
#include "math.h"
#include "stdio.h"
#include "main.h"
#include "fatfs.h"
#include "My_Audio.h"
#include "cmsis_os.h"
#include <stdbool.h>

extern osMessageQId PlayingQueueHandle;
extern osMessageQId CommandSDQueueHandle;
extern osMessageQId RecordingQueueHandle;

extern SemaphoreHandle_t StartSavingSemaphoreHandle;
extern SemaphoreHandle_t StartPlayingSemaphoreHandle;


char *filename = NULL;
int receivedData;

UINT byteswritten;
UINT bytesRead;
WAV_Header header;

/* Private function prototypes -----------------------------------------------*/
void StartSd_Task(void const * argument);
int GetAvailableFilename(FRESULT res, FILINFO fno );
void StartSDdCardTask(void const * argument);
void FillWavHeader(WAV_Header* header, uint32_t sampleRate, uint16_t bitsPerSample, uint16_t numChannels, uint32_t dataSize);


void StartPlayAndReadTask(void const * argument)
{
    char filename[32];
    FRESULT res;
    AudioChunk_t chunk;
    Audio_SdCard_Command_t command;

    for (;;)
    {
        // Czekamy na komendę z CommandQueueHandle
        if (xQueueReceive(CommandSDQueueHandle, &command, portMAX_DELAY) == pdTRUE)
        {
            if (command == CMD_START_RECORDING)
            {
                bool recordingInProgress = true;

                // Inicjalizacja SD, tworzenie pliku
                if (f_mount(&SDFatFS, (TCHAR const*)SDPath, 1) != FR_OK)
                    Error_Handler();

                GenerateNextFilename(filename);
                if (f_open(&SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
                    Error_Handler();

                // Dummy header
                WAV_Header header;
                FillWavHeader(&header, 44100, 16, 2, 0);
                UINT byteswritten = 0;
                res = f_write(&SDFile, &header, sizeof(WAV_Header), &byteswritten);
                if (res != FR_OK || byteswritten != sizeof(WAV_Header))
                    Error_Handler();

                while (recordingInProgress)
                {
                    if (xQueueReceive(RecordingQueueHandle, &chunk, portMAX_DELAY) == pdTRUE)
                    {
                        if (chunk.data == NULL)
                        {
                            // Zakończenie nagrywania
                            uint32_t fileSize = f_size(&SDFile);
                            WAV_Header finalHeader;
                            FillWavHeader(&finalHeader, 44100, 16, 2, fileSize - sizeof(WAV_Header));

                            UINT headerWritten;
                            f_lseek(&SDFile, 0);
                            f_write(&SDFile, &finalHeader, sizeof(WAV_Header), &headerWritten);
                            UpdateWavHeader(&SDFile);
                            f_close(&SDFile);
                            HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);

                            recordingInProgress = false;
                        }
                        else
                        {
                            res = f_write(&SDFile, chunk.data, chunk.length * sizeof(int16_t), &byteswritten);
                            if (res != FR_OK || byteswritten == 0)
                                Error_Handler();
                        }
                    }
                }
            }

            else if (command == CMD_START_PLAYING)
            {
                // Otwórz ostatni plik do odtwarzania
                if (f_mount(&SDFatFS, (TCHAR const*)SDPath, 1) != FR_OK)
                    Error_Handler();

                if (f_open(&SDFile, "AUDIO041.WAV", FA_READ) != FR_OK)
                    Error_Handler();

                // Pomiń nagłówek WAV
                f_lseek(&SDFile, sizeof(WAV_Header));

                // Zdefiniuj dwa oddzielne bufory do odczytu (double buffering)
                static int16_t playbackBuffer0[BUFFER_SIZE_SAMPLES];
                static int16_t playbackBuffer1[BUFFER_SIZE_SAMPLES];
                int currentBufferIdx = 0; // przełącznik między 0 a 1

                // Odczytaj i wysyłaj dane do PlayingQueueHandle
                for (;;)
                {
                    UINT bytesread = 0;
                    int16_t* currentBuffer = (currentBufferIdx == 0) ? playbackBuffer0 : playbackBuffer1;

                    // Odczytujemy pełny bufor
                    FRESULT res = f_read(&SDFile, currentBuffer, sizeof(playbackBuffer0), &bytesread);
                    if (res != FR_OK)
                        Error_Handler();

                    // Jeżeli nie ma już danych (EOF) - wyślij sygnał końca odtwarzania
                    if (bytesread == 0)
                        break;

                    AudioChunk_t chunk;
                    chunk.data = currentBuffer;
                    chunk.length = bytesread / sizeof(int16_t);

                    // Wysyłamy odczytany chunk do kolejki.
                    xQueueSend(PlayingQueueHandle, &chunk, portMAX_DELAY);

                    // Przełączamy bufor, aby nie nadpisać jeszcze nieprzetworzonych danych
                    currentBufferIdx ^= 1;
                }

                // Sygnał końca odtwarzania
                AudioChunk_t endChunk = { .data = NULL, .length = 0 };
                xQueueSend(PlayingQueueHandle, &endChunk, portMAX_DELAY);

                f_close(&SDFile);
            }
        }
    }
}


void GenerateNextFilename(char* filenameBuffer)
{
    static uint16_t fileIndex = 1;
    FRESULT res;
    FILINFO fno;

    do {
        sprintf(filenameBuffer, "AUDIO%03d.WAV", fileIndex++);
        res = f_stat(filenameBuffer, &fno);
    } while (res == FR_OK && fileIndex < 1000); // Sprawdź do AUDIO999.WAV
}

void GetLatestFilename(char* filenameBuffer)
{
    FILINFO fno;
    for (int i = 999; i >= 1; i--) {
        sprintf(filenameBuffer, "AUDIO%03d.WAV", i);
        if (f_stat(filenameBuffer, &fno) == FR_OK) return;
    }
    strcpy(filenameBuffer, "AUDIO001.WAV"); // jeśli nic nie ma
}


/**
 * @brief Fills a WAV header structure with proper values based on the provided audio parameters.
 *
 * This function initializes the provided `WAV_Header` structure with standard values for a WAV file header.
 * It takes sample rate, bit depth, number of channels, and data size as input and fills the structure
 * accordingly to form a valid WAV header.
 *
 * @param header Pointer to the `WAV_Header` structure that will be filled.
 * @param sampleRate Sample rate of the audio (in Hz).
 * @param bitsPerSample Number of bits per sample (e.g., 16 for 16-bit audio).
 * @param numChannels Number of channels (1 for mono, 2 for stereo).
 * @param dataSize Size of the audio data (in bytes), excluding the WAV header.
 */
void FillWavHeader(WAV_Header* header, uint32_t sampleRate, uint16_t bitsPerSample, uint16_t numChannels, uint32_t dataSize)
{
    memcpy(header->riff, "RIFF", 4);
    header->overall_size = dataSize + 36;
    memcpy(header->wave, "WAVE", 4);
    memcpy(header->fmt_chunk_marker, "fmt ", 4);
    header->length_of_fmt = 16;
    header->format_type = 1;
    header->channels = numChannels;
    header->sample_rate = sampleRate;
    header->byterate = sampleRate * numChannels * bitsPerSample / 8;
    header->block_align = numChannels * bitsPerSample / 8;
    header->bits_per_sample = bitsPerSample;
    memcpy(header->data_chunk_header, "data", 4);
    header->data_size = dataSize;
}




void UpdateWavHeader(FIL* SDFile) {
    FRESULT res;
    uint32_t fileSize = f_size(SDFile);  // Całkowity rozmiar pliku (po zapisaniu danych audio)
    uint32_t dataSize = fileSize - sizeof(WAV_Header);  // Rozmiar danych audio (po odjęciu nagłówka)

    // Przesunięcie na miejsce ChunkSize (offset 4)
    res = f_lseek(SDFile, 4);
    if (res != FR_OK) {
        Error_Handler();
    }

    // Zaktualizowanie ChunkSize (rozmiar całego pliku - 8)
    res = f_write(SDFile, &fileSize, sizeof(uint32_t), &byteswritten);
    if (res != FR_OK || byteswritten != sizeof(uint32_t)) {
        Error_Handler();
    }

    // Przesunięcie na miejsce Subchunk2Size (offset 40)
    res = f_lseek(SDFile, 40);
    if (res != FR_OK) {
        Error_Handler();
    }

    // Zaktualizowanie Subchunk2Size (rozmiar danych audio)
    res = f_write(SDFile, &dataSize, sizeof(uint32_t), &byteswritten);
    if (res != FR_OK || byteswritten != sizeof(uint32_t)) {
        Error_Handler();
    }
}


