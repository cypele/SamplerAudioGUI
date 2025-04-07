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
#include "cmsis_os.h"
#include <stdbool.h>

extern osMessageQId AudioQueueHandle;

char *filename = NULL;
int receivedData;

WAV_Header header;
UINT byteswritten;



/* Private function prototypes -----------------------------------------------*/
void StartSDCardTask(void const * argument);
int GetAvailableFilename(FRESULT res, FILINFO fno );
void StartSDdCardTask(void const * argument);
void FillWavHeader(WAV_Header* header, uint32_t sampleRate, uint16_t bitsPerSample, uint16_t numChannels, uint32_t dataSize);









void StartSDCardTask(void const * argument)
{
    char filename[32];
    FRESULT res;
    AudioChunk_t receivedChunk;
    bool recordingInProgress = true; // Flag to track if recording is in progress

    for (;;)
    {
        // Wait for data in the queue
        if (xQueueReceive(AudioQueueHandle, &receivedChunk, portMAX_DELAY) == pdTRUE)
        {
            // Check if we received the NULL pointer signaling end of recording
            if (receivedChunk.data == NULL && recordingInProgress)
            {
                // Prepare and write the correct WAV header
                uint32_t fileSize = f_size(&SDFile);
                WAV_Header finalHeader;
                FillWavHeader(&finalHeader, 44100, 16, 2, fileSize - sizeof(WAV_Header)); // sample rate, bits, channels

                // Write the correct WAV header (now with the correct size values)
                UINT headerWritten;
                f_lseek(&SDFile, 0);  // Go back to the beginning of the file to write the updated header
                f_write(&SDFile, &finalHeader, sizeof(WAV_Header), &headerWritten);

                // Final cleanup
                UpdateWavHeader(&SDFile); // Update chunk sizes before closing
                f_close(&SDFile);  // Close the file when the end of recording is signaled
                HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
                recordingInProgress = false;
                continue;
            }

            // If the file is not open, open it
            if (SDFile.obj.fs == NULL)
            {
                if (f_mount(&SDFatFS, (TCHAR const*)SDPath, 1) != FR_OK)
                    Error_Handler();

                GenerateNextFilename(filename);
                if (f_open(&SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
                    Error_Handler();

                // Write dummy WAV header (we'll fix it at the end)
                res = f_write(&SDFile, &header, sizeof(WAV_Header), &byteswritten);
                if ((byteswritten != sizeof(WAV_Header)) || (res != FR_OK))
                {
                    Error_Handler();
                }
            }

            // Write audio chunk
            res = f_write(&SDFile, receivedChunk.data, receivedChunk.length * sizeof(int16_t), &byteswritten);
            if (res != FR_OK || byteswritten == 0)
                Error_Handler();
        }
    }
}






/* USER CODE BEGIN Header_StartSDCardTask */
/**
  * @brief  Function implementing the SDCardTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSDCardTask */

void StartSDdCardTask(void const * argument)
{
    FRESULT res;
    UINT byteswritten, bytesread;
    uint8_t rtext[_MAX_SS];

    uint32_t numSamples = SAMPLE_RATE * DURATION_SEC;
    int16_t high = 100;
    int16_t low = -100;
    int samplesPerCycle = SAMPLE_RATE / 1000;
    int halfCycle = samplesPerCycle / 2;
    int16_t buffer[samplesPerCycle];

    // Wypełnienie bufora próbki (fale kwadratowe)
    for (int i = 0; i < samplesPerCycle; i++) {
        buffer[i] = (i < halfCycle) ? high : low;
    }
    uint32_t totalSamplesWritten = 0;



    /* Infinite loop */
    for(;;)
    {
        if(xQueueReceive(AudioQueueHandle, &receivedData, portMAX_DELAY))
        {
            if (f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)
            {
                if(f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, rtext, sizeof(rtext)) != FR_OK)
                {
                    Error_Handler();
                }
            }
            else
            {
            	GenerateNextFilename(filename);
                if(f_open(&SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
                {
                    Error_Handler();
                }
                else
                {
                    // Zapisz nagłówek WAV
                    res = f_write(&SDFile, &header, sizeof(WAV_Header), &byteswritten);
                    if ((byteswritten == 0) || (res != FR_OK))
                    {
                        Error_Handler();
                    }

                    // Zapis danych audio
                    while (totalSamplesWritten < numSamples)
                    {
                        uint32_t samplesToWrite = (numSamples - totalSamplesWritten) < samplesPerCycle * 1000 ?
                                                    (numSamples - totalSamplesWritten) :
                                                    samplesPerCycle * 1000;

                        for (uint32_t i = 0; i < samplesToWrite / samplesPerCycle; i++) {
                            res = f_write(&SDFile, buffer, sizeof(buffer), &byteswritten);
                            if (res != FR_OK || byteswritten == 0)
                            {
                                Error_Handler();
                            }
                        }

                        totalSamplesWritten += samplesToWrite;
                    }

                    // Po zapisaniu danych audio, zaktualizuj nagłówek WAV
                    uint32_t fileSize = f_size(&SDFile);  // Całkowity rozmiar pliku
                    uint32_t dataSize = fileSize - sizeof(WAV_Header);  // Rozmiar danych (po odjęciu nagłówka)

                    // Zaktualizowanie chunkSize w nagłówku WAV
                    res = f_lseek(&SDFile, 4);  // Przesuń na miejsce chunkSize (offset 4)
                    if (res != FR_OK) {
                        Error_Handler();
                    }

                    res = f_write(&SDFile, &fileSize, sizeof(uint32_t), &byteswritten);
                    if (res != FR_OK || byteswritten == 0) {
                        Error_Handler();
                    }

                    // Zaktualizowanie subchunk2Size w nagłówku WAV
                    res = f_lseek(&SDFile, 40);  // Przesuń na miejsce subchunk2Size (offset 40)
                    if (res != FR_OK) {
                        Error_Handler();
                    }

                    res = f_write(&SDFile, &dataSize, sizeof(uint32_t), &byteswritten);
                    if (res != FR_OK || byteswritten == 0) {
                        Error_Handler();
                    }

                    f_close(&SDFile);
                }
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


