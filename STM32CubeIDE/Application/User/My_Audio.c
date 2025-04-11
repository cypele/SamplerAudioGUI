/*
 * My_Audio.c
 *
 * Created on: Apr 1, 2025
 * Author: adamc
 */

#include "My_Audio.h"
#include "math.h"
#include "stdio.h"
#include "main.h"

#include "audio.h"
#include "Filter.h"
// **Zmienna globalna do konfiguracji audio**

#define DMA_MAX(x)           (((x) <= 0xFFFF)? (x):0XFFFF)

// **Bufory audio**
static int16_t record_buffer[BUFFER_SIZE_SAMPLES];
static int16_t play_buffer[BUFFER_SIZE_SAMPLES];

static int16_t compressed_buffer0[BUFFER_SIZE_SAMPLES/2];
static int16_t compressed_buffer1[BUFFER_SIZE_SAMPLES/2];

static int16_t filtered_buffer[BUFFER_SIZE_SAMPLES / 2];


uint32_t frequency = AUDIO_FREQUENCY_44K;
uint8_t volume = 100;




SAI_HandleTypeDef               haudio_out_sai;
SAI_HandleTypeDef               haudio_in_sai;

DMA_HandleTypeDef hdma_sai_tx;
DMA_HandleTypeDef hdma_sai_rx;
extern I2C_HandleTypeDef hi2c4;

extern osMessageQId CommandAudioQueueHandle;
extern osMessageQId RecordingQueueHandle;
extern osMessageQId PlayingQueueHandle;

extern TaskHandle_t xRecordingTaskHandle;
extern TaskHandle_t xPlayingTaskHandle;

extern SemaphoreHandle_t StartRecordingSemaphoreHandle;
extern SemaphoreHandle_t StopRecordingSemaphoreHandle;
extern SemaphoreHandle_t StartPlayingSemaphoreHandle;


volatile uint32_t audio_rec_buffer_state = BUFFER_OFFSET_NONE;
volatile uint32_t audio_tx_buffer_state = 0;



// **Deklaracja zewnętrznego handlera SAI**

void Audio_passThrough(void);
void sin_wave_data_to_buffer(int16_t *buf, uint32_t num_samples);
static void convert_to_dma_buffer(int16_t *sample_buffer, uint8_t *dma_buffer, uint32_t num_samples);
static void extract_from_dma_buffer(uint8_t *dma_buffer, int16_t *sample_buffer, uint32_t num_samples);
static void CopyBuffer(int16_t *dst, int16_t *src, uint32_t num_samples);
extern void MX_SAI1_Init(void);
static void SAIx_Out_Init(uint32_t AudioFreq);
static void SAIx_In_Init(uint32_t AudioFreq);
static void SAIx_In_DeInit(void);
static void SAIx_Out_DeInit(void);
void SAI_Clock_Configuration(uint32_t AudioFreq);
void Sai_Out_MspInit(SAI_HandleTypeDef *hsai);
void Sai_In_MspInit(SAI_HandleTypeDef *hsai);
void AudioInit(uint32_t AudioFreq);
void StartRecord_Task(void const * argument);
void init_polyphase();
void fir_notch_filter_polyphase(int16_t *input, int16_t *output, int size);



extern void StartSd_Task(void const * argument);


/* USER CODE BEGIN Header_StartRecordTask */
/**
* @brief Function implementing the Record_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRecordTask */

/*
void StartRecAndSaveTaskk(void const * argument)
{
    Audio_SdCard_Command_t command;
    FRESULT res;
    UINT byteswritten;
    char filename[32];
    xRecordingTaskHandle=xTaskGetCurrentTaskHandle();
    // Montujemy system plików na starcie (zakładamy, że jest to operacja globalna)
    if (f_mount(&SDFatFS, (TCHAR const*)SDPath, 1) != FR_OK)
        Error_Handler();

    // Inicjalizacja peryferiów audio – jeśli nie musimy tego robić przy każdej operacji,
    // można to przenieść poza pętlę
    AudioInit(AUDIO_FREQUENCY_44K);
    wm8994_Init();

    for(;;)
    {
        // Oczekujemy na polecenie startu nagrywania
        if(xQueueReceive(RecordingQueueHandle, &command, portMAX_DELAY) == pdTRUE)
        {
            if(command == CMD_START_RECORDING)
            {
                GenerateNextFilename(filename);
                if (f_open(&SDFile, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
                    Error_Handler();

                // Zapisujemy placeholder nagłówka WAV
                res = f_write(&SDFile, &header, sizeof(WAV_Header), &byteswritten);
                if ((byteswritten != sizeof(WAV_Header)) || (res != FR_OK))
                    Error_Handler();

                if (HAL_SAI_Transmit_DMA(&haudio_out_sai, (uint8_t*)play_buffer, BUFFER_SIZE_SAMPLES) != HAL_OK)
                {
                    Error_Handler();
                    return;
                }
                // Uruchom DMA dla odbioru audio
                if (HAL_SAI_Receive_DMA(&haudio_in_sai, (uint8_t*)record_buffer, BUFFER_SIZE_SAMPLES) != HAL_OK)
                    Error_Handler();

                // Pętla obsługująca nagrywanie – przetwarzanie danych z bufora
                for (;;)
                {
                    // Czekamy na powiadomienie, że bufor jest gotowy (np. z callbacka DMA)
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

                    if (audio_rec_buffer_state != BUFFER_OFFSET_NONE)
                    {
                        // Wybieramy odpowiednią połowę bufora
                        int16_t* source_ptr = (audio_rec_buffer_state == BUFFER_OFFSET_HALF) ?
                                               &record_buffer[0] :
                                               &record_buffer[BUFFER_SIZE_SAMPLES / 2];

                        // Przetwarzamy dane – przykładowo filtrujemy szum
                        simple_noise_filter(source_ptr, filtered_buffer, BUFFER_SIZE_SAMPLES / 2);

                        // Zapisujemy przetworzone próbki do pliku
                        res = f_write(&SDFile, filtered_buffer, (BUFFER_SIZE_SAMPLES / 2) * sizeof(int16_t), &byteswritten);
                        if (res != FR_OK || byteswritten == 0)
                            Error_Handler();

                        // Resetujemy stan bufora
                        audio_rec_buffer_state = BUFFER_OFFSET_NONE;
                    }

                    // Sprawdzamy, czy użytkownik zażądał przerwania nagrywania, np. przez przyciśnięcie przycisku STOP
                    if (xSemaphoreTake(StopRecordingSemaphoreHandle, 0) == pdTRUE)
                    {
                        // Zatrzymanie DMA – koniec nagrywania
                        HAL_SAI_DMAStop(&haudio_in_sai);

                        // Uzupełniamy nagłówek WAV z odpowiednią informacją o rozmiarze danych
                        uint32_t fileSize = f_size(&SDFile);
                        WAV_Header finalHeader;
                        FillWavHeader(&finalHeader, 44100, 16, 2, fileSize - sizeof(WAV_Header));
                        f_lseek(&SDFile, 0);
                        res = f_write(&SDFile, &finalHeader, sizeof(WAV_Header), &byteswritten);
                        if ((byteswritten != sizeof(WAV_Header)) || (res != FR_OK))
                            Error_Handler();

                        UpdateWavHeader(&SDFile);
                        f_close(&SDFile);
                        break;  // Zakończyliśmy bieżącą operację nagrywania – wracamy do stanu oczekiwania na komendę
                    }
                    taskYIELD();
                }
            }
        }
    }
}
*/

/*
void StartPlayAndReadTaskk(void const * argument)
{
    Audio_SdCard_Command_t command;
    FRESULT res;
    UINT bytesRead;
    char filename[32];

    // Montujemy system plików na starcie
    if (f_mount(&SDFatFS, (TCHAR const*)SDPath, 1) != FR_OK)
        Error_Handler();

    for(;;)
    {
        // Oczekujemy na polecenie startu odtwarzania
        if(xQueueReceive(PlayingQueueHandle, &command, portMAX_DELAY) == pdTRUE)
        {
            if(command == CMD_START_PLAYING)
            {
                // Nazwa pliku może być przekazywana lub wybierana z innego mechanizmu
                // Tutaj przykładowo ustawiamy statycznie
                strcpy(filename, "AUDIO015.WAV");

                if (f_open(&SDFile, filename, FA_READ) != FR_OK)
                    Error_Handler();

                // Pomijamy nagłówek WAV
                f_lseek(&SDFile, sizeof(WAV_Header));

                // Uruchamiamy transmisję DMA w trybie circular
                if (HAL_SAI_Transmit_DMA(&haudio_out_sai, (uint8_t*)play_buffer, BUFFER_SIZE_SAMPLES) != HAL_OK)
                    Error_Handler();

                // Pętla przetwarzania odtwarzania
                for (;;)
                {
                    // Oczekujemy na powiadomienie, że jedna z połówek bufora jest pusta (callback DMA)
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

                    // W zależności od stanu bufora określamy, którą połowę uzupełnić
                    int startIndex = (audio_tx_buffer_state == BUFFER_OFFSET_HALF) ? 0 : BUFFER_SIZE_SAMPLES / 2;
                    res = f_read(&SDFile, &play_buffer[startIndex], (BUFFER_SIZE_SAMPLES / 2) * sizeof(uint16_t), &bytesRead);
                    if (res != FR_OK)
                        Error_Handler();

                    if (bytesRead == 0)  // koniec pliku
                    {
                        HAL_SAI_DMAStop(&haudio_out_sai);
                        f_close(&SDFile);
                        break; // Zakończyliśmy bieżącą operację odtwarzania – wracamy do oczekiwania na komendę
                    }

                    // Jeśli odczytany fragment jest mniejszy niż oczekiwany, wyzeruj resztę bufora
                    if (bytesRead < (BUFFER_SIZE_SAMPLES / 2) * sizeof(uint16_t))
                    {
                        memset(&play_buffer[startIndex + bytesRead/sizeof(uint16_t)], 0,
                               (BUFFER_SIZE_SAMPLES/2) * sizeof(uint16_t) - bytesRead);
                    }

                    audio_tx_buffer_state = BUFFER_OFFSET_NONE;
                    taskYIELD();
                }
            }
        }
    }
}
*/


/* USER CODE BEGIN Header_StartRecordTask */
/**
* @brief Function implementing the Record_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRecordTask */
void StartRecAndSaveTask(void const * argument)
{
    int recordingComplete = 0;
    int playingComplete = 0;
    xRecordingTaskHandle = xTaskGetCurrentTaskHandle();
    xPlayingTaskHandle = xTaskGetCurrentTaskHandle();

    AudioChunk_t chunk;
    int current_compressed_buffer = 0;
    Audio_SdCard_Command_t command;

    for (;;)
    {
        if (xQueueReceive(CommandAudioQueueHandle, &command, portMAX_DELAY) == pdTRUE)
        {
            if (command == CMD_START_RECORDING)
            {
                recordingComplete = 0;
                audio_rec_buffer_state = BUFFER_OFFSET_NONE;

                if (HAL_SAI_Receive_DMA(&haudio_in_sai, (uint8_t*)record_buffer, BUFFER_SIZE_SAMPLES) != HAL_OK)
                    Error_Handler();

                if (HAL_SAI_Transmit_DMA(&haudio_out_sai, (uint8_t*)play_buffer, BUFFER_SIZE_SAMPLES) != HAL_OK)
                    Error_Handler();

                while (!recordingComplete)
                {
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

                    if (audio_rec_buffer_state != BUFFER_OFFSET_NONE)
                    {
                        int16_t* source_ptr = (audio_rec_buffer_state == BUFFER_OFFSET_HALF)
                                              ? &record_buffer[0]
                                              : &record_buffer[BUFFER_SIZE_SAMPLES / 2];

                        simple_noise_filter(source_ptr, filtered_buffer, BUFFER_SIZE_SAMPLES / 2);

                        int16_t* output_ptr = (current_compressed_buffer == 0) ? compressed_buffer0 : compressed_buffer1;

                        size_t out_idx = 0;
                        for (size_t i = 0; i < BUFFER_SIZE_SAMPLES / 2; i += 4)
                        {
                            output_ptr[out_idx++] = filtered_buffer[i];     // L
                            output_ptr[out_idx++] = filtered_buffer[i + 2]; // R
                        }

                        chunk.data = output_ptr;
                        chunk.length = out_idx;

                        xQueueSend(RecordingQueueHandle, &chunk, portMAX_DELAY);
                        current_compressed_buffer ^= 1;

                        audio_rec_buffer_state = BUFFER_OFFSET_NONE;

                        if (xQueueReceive(CommandAudioQueueHandle, &command, 0) == pdTRUE)
                        {
                            if (command == CMD_STOP_RECORDING)
                            {
                                HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
                                HAL_SAI_DMAStop(&haudio_in_sai);
                                HAL_SAI_DMAStop(&haudio_out_sai);
                                recordingComplete = 1;
                                break;
                            }
                        }
                    }
                }

                AudioChunk_t endSignal = { .data = NULL, .length = 0 };
                xQueueSend(RecordingQueueHandle, &endSignal, portMAX_DELAY);
            }

            else if (command == CMD_START_PLAYING)
            {
                playingComplete = 0;
                audio_tx_buffer_state = BUFFER_OFFSET_NONE;

                // Zmienne do przetwarzania bieżącego fragmentu odtwarzania
                AudioChunk_t currentChunk;
                int16_t *currentChunkData = NULL;
                size_t currentChunkRemaining = 0;
                size_t currentPosInChunk = 0;

                // Wstępne (pre-buffering) wypełnienie całego play_buffer danymi z kolejki.
                // Chcemy mieć pewność, że play_buffer (oba segmenty) zawiera dane zaczynające się od początku nagrania.
                for (int half = 0; half < 2; ++half)
                {
                    int16_t* dest = (half == 0) ? play_buffer : &play_buffer[BUFFER_SIZE_SAMPLES / 2];
                    size_t destSamples = BUFFER_SIZE_SAMPLES / 2; // liczba próbek w jednej połówce
                    size_t destIndex = 0;

                    while (destIndex < destSamples)
                    {
                        // Jeśli nie mamy już danych z bieżącego chunku, pobieramy kolejny element z kolejki.
                        if (currentChunkData == NULL || currentChunkRemaining == 0)
                        {
                            if (xQueueReceive(PlayingQueueHandle, &currentChunk, portMAX_DELAY) == pdTRUE)
                            {
                                // Jeśli otrzymano sygnał zakończenia transmisji (koniec pliku)
                                if (currentChunk.data == NULL || currentChunk.length == 0)
                                {
                                    playingComplete = 1;
                                    break;
                                }
                                currentChunkData    = currentChunk.data;
                                currentChunkRemaining = currentChunk.length;
                                currentPosInChunk   = 0;
                            }
                        }

                        // Kopiujemy jedną próbkę stereo (2 int16_t) do formatu [L, 0, R, 0]
                        if (currentChunkRemaining >= 2 && (destIndex + 4) <= destSamples)
                        {
                            dest[destIndex]     = currentChunkData[currentPosInChunk];       // lewy kanał
                            dest[destIndex + 1] = 0;                                           // cisza
                            dest[destIndex + 2] = currentChunkData[currentPosInChunk + 1];     // prawy kanał
                            dest[destIndex + 3] = 0;                                           // cisza

                            currentPosInChunk += 2;
                            currentChunkRemaining -= 2;
                            destIndex += 4;
                        }
                        else
                        {
                            break;
                        }
                    }

                    // Jeśli nie udało się całkowicie wypełnić danej połówki, uzupełniamy resztę zerami
                    for (; destIndex < destSamples; destIndex++)
                    {
                        dest[destIndex] = 0;
                    }

                    // Jeśli otrzymaliśmy sygnał końca transmisji, wychodzimy z pętli pre-bufferingu
                    if (playingComplete)
                        break;
                }

                // Jeżeli sygnał końca transmisji został odebrany przed wypełnieniem bufora – przerywamy odtwarzanie.
                if (playingComplete)
                {
                    HAL_SAI_DMAStop(&haudio_out_sai);
                    continue;
                }

                // Po pre-bufferingu uruchamiamy DMA transmisję.
                if (HAL_SAI_Transmit_DMA(&haudio_out_sai, (uint8_t*)play_buffer, BUFFER_SIZE_SAMPLES) != HAL_OK)
                    Error_Handler();

                // Pętla główna odtwarzania – DMA działa w trybie circular, a my aktualizujemy kolejne połówki bufora.
                while (!playingComplete)
                {
                    // Oczekujemy na powiadomienie, że jedna z połówek DMA jest wolna do uzupełnienia
                    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

                    // Określenie, którą połówkę bufora DMA mamy uzupełnić:
                    // BUFFER_OFFSET_HALF => pierwsza połówka, BUFFER_OFFSET_FULL => druga
                    int half = (audio_tx_buffer_state == BUFFER_OFFSET_HALF) ? 0 : 1;
                    int16_t* dest = (half == 0) ? play_buffer : &play_buffer[BUFFER_SIZE_SAMPLES / 2];
                    size_t destSamples = BUFFER_SIZE_SAMPLES / 2;
                    size_t destIndex = 0;

                    // Uzupełniamy aktywną połówkę bufora danymi z kolejki – przekształcamy ze standardu [L,R] do [L, 0, R, 0]
                    while (destIndex < destSamples)
                    {
                        if (currentChunkData == NULL || currentChunkRemaining == 0)
                        {
                            if (xQueueReceive(PlayingQueueHandle, &currentChunk, portMAX_DELAY) == pdTRUE)
                            {
                                if (currentChunk.data == NULL || currentChunk.length == 0)
                                {
                                    playingComplete = 1;
                                    break;
                                }
                                currentChunkData    = currentChunk.data;
                                currentChunkRemaining = currentChunk.length;
                                currentPosInChunk   = 0;
                            }
                        }

                        if (currentChunkRemaining >= 2 && (destIndex + 4) <= destSamples)
                        {
                            dest[destIndex]     = currentChunkData[currentPosInChunk];
                            dest[destIndex + 1] = 0;
                            dest[destIndex + 2] = currentChunkData[currentPosInChunk + 1];
                            dest[destIndex + 3] = 0;

                            currentPosInChunk += 2;
                            currentChunkRemaining -= 2;
                            destIndex += 4;
                        }
                        else
                        {
                            break;
                        }
                    }

                    // Uzupełnienie reszty aktywnej połówki bufora zerami, jeśli nie udało się jej całkowicie wypełnić
                    for (; destIndex < destSamples; destIndex++)
                    {
                        dest[destIndex] = 0;
                    }

                    // Resetujemy flagę, aby DMA mogło przekazać informację o kolejnej wolnej połowie
                    audio_tx_buffer_state = BUFFER_OFFSET_NONE;
                }

                // Zakończenie transmisji – zatrzymujemy DMA
                HAL_SAI_DMAStop(&haudio_out_sai);
            }
        }
    }
}



/**
 * @brief Prosty filtr
 * @param input - wskaźnik do bufora in
 * @param output - wskaźnik do bufora out
 * @param size - wielkosc bufora
 */
void simple_noise_filter(int16_t *input, int16_t *output, int size) {
    const int NOISE_THRESHOLD = 20; // Values below this are treated as noise
    const int SMOOTHING_WINDOW = 5; // Moving average window size
    int i, j;

    // 1️⃣ Apply noise gate
    for (i = 0; i < size; i++) {
        if (input[i] > -NOISE_THRESHOLD && input[i] < NOISE_THRESHOLD) {
            output[i] = 0; // Remove small noise
        } else {
            output[i] = input[i]; // Keep original signal
        }
    }

    // 2️⃣ Apply simple moving average filter
    for (i = 0; i < size; i++) {
        int32_t sum = 0;
        int count = 0;

        for (j = -SMOOTHING_WINDOW; j <= SMOOTHING_WINDOW; j++) {
            int idx = i + j;
            if (idx >= 0 && idx < size) { // Ensure index is in bounds
                sum += output[idx];
                count++;
            }
        }

        output[i] = (int16_t)(sum / count); // Average to smooth signal
    }
}


/**
 * @brief Generuje próbki fali sinusoidalnej
 * @param buf - wskaźnik do bufora
 * @param num_samples - liczba próbek
 */
void sin_wave_data_to_buffer(int16_t *buf, uint32_t num_samples) {
    for (uint32_t i = 0; i < num_samples; i++) {
        int16_t left_sample  = 1000;

        buf[i]     = left_sample;   // Lewy kanał
    }
}



static void convert_to_dma_buffer(int16_t *sample_buffer, uint8_t *dma_buffer, uint32_t num_frames) {
    for (uint32_t i = 0; i < num_frames; i++) {
        // Pobierz próbki: lewy i prawy
        int16_t left_sample  = sample_buffer[2 * i];
        int16_t right_sample = sample_buffer[2 * i + 1];


        int16_t *p = (int16_t *)&dma_buffer[i * 4];  // Teraz 4 bajty na ramkę (2 sloty)
        p[0] = left_sample;   // Slot 0 – lewy kanał
        p[1] = left_sample;  // Slot 1 – prawy kanał
        p[2] = right_sample;
        p[3] = right_sample;

    }
}

// Extract samples from DMA buffer (only extracting one channel for now)
static void extract_from_dma_buffer(uint8_t *dma_buffer, int16_t *sample_buffer, uint32_t num_samples) {
    for (uint32_t i = 0; i < num_samples; i++) {
        int16_t *sample_pointer = (int16_t *) &dma_buffer[i * 8]; // Extract from every 8-byte frame
        sample_buffer[i] = *sample_pointer;
    }
}

// Copy audio sample buffer from source to destination
static void CopyBuffer(int16_t *pbuffer1, int16_t *pbuffer2, uint32_t BufferSize)
{
    uint32_t i = 0;
    for (i = 0; i < BufferSize; i++)
    {
        pbuffer1[i] = pbuffer2[i];
    }
}

static void SAIx_Out_Init(uint32_t AudioFreq)
{
    /* Initialize the haudio_out_sai Instance parameter */
    haudio_out_sai.Instance = AUDIO_OUT_SAIx;

    /* Disable SAI peripheral to allow access to SAI internal registers */
    __HAL_SAI_DISABLE(&haudio_out_sai);

    /* Configure SAI_Block_x */
    haudio_out_sai.Init.MonoStereoMode = SAI_STEREOMODE;
    haudio_out_sai.Init.AudioFrequency = AudioFreq;
    haudio_out_sai.Init.AudioMode      = SAI_MODEMASTER_TX;
    haudio_out_sai.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
    haudio_out_sai.Init.Protocol       = SAI_FREE_PROTOCOL;
    haudio_out_sai.Init.DataSize       = SAI_DATASIZE_16;
    haudio_out_sai.Init.FirstBit       = SAI_FIRSTBIT_MSB;
    haudio_out_sai.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;
    haudio_out_sai.Init.Synchro        = SAI_ASYNCHRONOUS;
    haudio_out_sai.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
    haudio_out_sai.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
    haudio_out_sai.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
    haudio_out_sai.Init.CompandingMode = SAI_NOCOMPANDING;
    haudio_out_sai.Init.TriState       = SAI_OUTPUT_NOTRELEASED;
    haudio_out_sai.Init.Mckdiv         = 0;

    /* Configure SAI_Block_x Frame */
    haudio_out_sai.FrameInit.FrameLength       = 64;
    haudio_out_sai.FrameInit.ActiveFrameLength = 32;
    haudio_out_sai.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
    haudio_out_sai.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
    haudio_out_sai.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

    /* Configure SAI Block_x Slot */
    haudio_out_sai.SlotInit.FirstBitOffset = 0;
    haudio_out_sai.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
    haudio_out_sai.SlotInit.SlotNumber     = 4;
    haudio_out_sai.SlotInit.SlotActive     = CODEC_AUDIOFRAME_SLOT_0123;

    HAL_SAI_Init(&haudio_out_sai);

    /* Enable SAI peripheral to generate MCLK */
    __HAL_SAI_ENABLE(&haudio_out_sai);
}


static void SAIx_In_Init(uint32_t AudioFreq)
{
    /* Initialize the haudio_in_sai Instance parameter */
    haudio_in_sai.Instance = AUDIO_IN_SAIx;

    /* Disable SAI peripheral to allow access to SAI internal registers */
    __HAL_SAI_DISABLE(&haudio_in_sai);

    /* Configure SAI_Block_x */
    haudio_in_sai.Init.MonoStereoMode = SAI_STEREOMODE;
    haudio_in_sai.Init.AudioFrequency = AudioFreq;
    haudio_in_sai.Init.AudioMode      = SAI_MODESLAVE_RX;
    haudio_in_sai.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
    haudio_in_sai.Init.Protocol       = SAI_FREE_PROTOCOL;
    haudio_in_sai.Init.DataSize       = SAI_DATASIZE_16;
    haudio_in_sai.Init.FirstBit       = SAI_FIRSTBIT_MSB;
    haudio_in_sai.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;
    haudio_in_sai.Init.Synchro        = SAI_SYNCHRONOUS;
    haudio_in_sai.Init.OutputDrive    = SAI_OUTPUTDRIVE_DISABLE;
    haudio_in_sai.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
    haudio_in_sai.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
    haudio_in_sai.Init.CompandingMode = SAI_NOCOMPANDING;
    haudio_in_sai.Init.TriState       = SAI_OUTPUT_RELEASED;
    haudio_in_sai.Init.Mckdiv         = 0;

    /* Configure SAI_Block_x Frame */
    haudio_in_sai.FrameInit.FrameLength       = 64;
    haudio_in_sai.FrameInit.ActiveFrameLength = 32;
    haudio_in_sai.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
    haudio_in_sai.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
    haudio_in_sai.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

    /* Configure SAI Block_x Slot */
    haudio_in_sai.SlotInit.FirstBitOffset = 0;
    haudio_in_sai.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
    haudio_in_sai.SlotInit.SlotNumber     = 4;
    haudio_in_sai.SlotInit.SlotActive     = CODEC_AUDIOFRAME_SLOT_0123;

    HAL_SAI_Init(&haudio_in_sai);

    /* Enable SAI peripheral */
    __HAL_SAI_ENABLE(&haudio_in_sai);
}


static void SAIx_In_DeInit(void)
{
    /* Initialize the haudio_in_sai Instance parameter */
    haudio_in_sai.Instance = AUDIO_IN_SAIx;
    /* Disable SAI peripheral */
    __HAL_SAI_DISABLE(&haudio_in_sai);

    HAL_SAI_DeInit(&haudio_in_sai);
}


static void SAIx_Out_DeInit(void)
{
    /* Initialize the haudio_in_sai Instance parameter */
    haudio_out_sai.Instance = AUDIO_OUT_SAIx;
    /* Disable SAI peripheral */
    __HAL_SAI_DISABLE(&haudio_out_sai);

    HAL_SAI_DeInit(&haudio_out_sai);
}

void SAI_Clock_Configuration(uint32_t AudioFreq)
{
    RCC_PeriphCLKInitTypeDef rcc_ex_clk_init_struct = {0};

    HAL_RCCEx_GetPeriphCLKConfig(&rcc_ex_clk_init_struct);

    /* Set the PLL configuration according to the audio frequency */
    if ((AudioFreq == AUDIO_FREQUENCY_11K) ||
        (AudioFreq == AUDIO_FREQUENCY_22K) ||
        (AudioFreq == AUDIO_FREQUENCY_44K))
    {
        /* Configure PLLSAI prescalers for ~11.289 MHz */
        rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
        rcc_ex_clk_init_struct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
        rcc_ex_clk_init_struct.PLLI2S.PLLI2SN = 429;
        rcc_ex_clk_init_struct.PLLI2S.PLLI2SQ = 2;
        rcc_ex_clk_init_struct.PLLI2SDivQ = 19;

        HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);
    }
    else
    {
        /* Configure PLLSAI prescalers for ~49.142 MHz */
        rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
        rcc_ex_clk_init_struct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
        rcc_ex_clk_init_struct.PLLI2S.PLLI2SN = 344;
        rcc_ex_clk_init_struct.PLLI2S.PLLI2SQ = 7;
        rcc_ex_clk_init_struct.PLLI2SDivQ = 1;

        HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);
    }

    /* Route SAI2 as source for DFSDM1 audio clock */
    rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_DFSDM1_AUDIO;
    rcc_ex_clk_init_struct.Dfsdm1AudioClockSelection = RCC_DFSDM1AUDIOCLKSOURCE_SAI2;
    HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}



void Sai_Out_MspInit(SAI_HandleTypeDef *hsai)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* Enable SAI clock */
  AUDIO_OUT_SAIx_CLK_ENABLE();


  /* Enable GPIO clock */
  AUDIO_OUT_SAIx_MCLK_ENABLE();
  AUDIO_OUT_SAIx_SD_FS_CLK_ENABLE();

  /* CODEC_SAI pins configuration: FS, SCK, MCK and SD pins ------------------*/
  gpio_init_structure.Pin = AUDIO_OUT_SAIx_FS_PIN | AUDIO_OUT_SAIx_SCK_PIN | AUDIO_OUT_SAIx_SD_PIN;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = AUDIO_OUT_SAIx_AF;
  HAL_GPIO_Init(AUDIO_OUT_SAIx_SD_FS_SCK_GPIO_PORT, &gpio_init_structure);

  gpio_init_structure.Pin = AUDIO_OUT_SAIx_MCLK_PIN;
  HAL_GPIO_Init(AUDIO_OUT_SAIx_MCLK_GPIO_PORT, &gpio_init_structure);

  /* Enable the DMA clock */
  AUDIO_OUT_SAIx_DMAx_CLK_ENABLE();

  if(hsai->Instance == AUDIO_OUT_SAIx)
  {
    /* Configure the hdma_saiTx handle parameters */
    hdma_sai_tx.Init.Channel             = AUDIO_OUT_SAIx_DMAx_CHANNEL;
    hdma_sai_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_sai_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sai_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sai_tx.Init.PeriphDataAlignment = AUDIO_OUT_SAIx_DMAx_PERIPH_DATA_SIZE;
    hdma_sai_tx.Init.MemDataAlignment    = AUDIO_OUT_SAIx_DMAx_MEM_DATA_SIZE;
    hdma_sai_tx.Init.Mode                = DMA_CIRCULAR;
    hdma_sai_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_sai_tx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_sai_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_sai_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_sai_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    hdma_sai_tx.Instance = AUDIO_OUT_SAIx_DMAx_STREAM;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hsai, hdmatx, hdma_sai_tx);

    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_sai_tx);

    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_sai_tx);
  }

  /* SAI DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(AUDIO_OUT_SAIx_DMAx_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(AUDIO_OUT_SAIx_DMAx_IRQ);
}


void Sai_In_MspInit(SAI_HandleTypeDef *hsai)
{
    GPIO_InitTypeDef  gpio_init_structure;

    /* Enable SAI clock */
    AUDIO_IN_SAIx_CLK_ENABLE();

    /* Enable SD GPIO clock */
    AUDIO_IN_SAIx_SD_ENABLE();
    /* CODEC_SAI pin configuration: SD pin */
    gpio_init_structure.Pin = AUDIO_IN_SAIx_SD_PIN;
    gpio_init_structure.Mode = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FAST;
    gpio_init_structure.Alternate = AUDIO_IN_SAIx_AF;
    HAL_GPIO_Init(AUDIO_IN_SAIx_SD_GPIO_PORT, &gpio_init_structure);

    /* Enable Audio INT GPIO clock */
    AUDIO_IN_INT_GPIO_ENABLE();
    /* Audio INT pin configuration: input */
    gpio_init_structure.Pin = AUDIO_IN_INT_GPIO_PIN;
    gpio_init_structure.Mode = GPIO_MODE_INPUT;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(AUDIO_IN_INT_GPIO_PORT, &gpio_init_structure);

    /* Enable the DMA clock */
    AUDIO_IN_SAIx_DMAx_CLK_ENABLE();

    if (hsai->Instance == AUDIO_IN_SAIx)
    {
        /* Configure the hdma_sai_rx handle parameters */
        hdma_sai_rx.Init.Channel             = AUDIO_IN_SAIx_DMAx_CHANNEL;
        hdma_sai_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_sai_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_sai_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_sai_rx.Init.PeriphDataAlignment = AUDIO_IN_SAIx_DMAx_PERIPH_DATA_SIZE;
        hdma_sai_rx.Init.MemDataAlignment    = AUDIO_IN_SAIx_DMAx_MEM_DATA_SIZE;
        hdma_sai_rx.Init.Mode                = DMA_CIRCULAR;
        hdma_sai_rx.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_sai_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        hdma_sai_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
        hdma_sai_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
        hdma_sai_rx.Init.PeriphBurst         = DMA_MBURST_SINGLE;

        hdma_sai_rx.Instance = AUDIO_IN_SAIx_DMAx_STREAM;

        /* Associate the DMA handle */
        __HAL_LINKDMA(hsai, hdmarx, hdma_sai_rx);

        /* Deinitialize the Stream for new transfer */
        HAL_DMA_DeInit(&hdma_sai_rx);

        /* Configure the DMA Stream */
        HAL_DMA_Init(&hdma_sai_rx);
    }

    /* SAI DMA IRQ Channel configuration */
    HAL_NVIC_SetPriority(AUDIO_IN_SAIx_DMAx_IRQ, AUDIO_IN_IRQ_PREPRIO, 0);
    HAL_NVIC_EnableIRQ(AUDIO_IN_SAIx_DMAx_IRQ);

    /* Audio INT IRQ Channel configuration */
    HAL_NVIC_SetPriority(AUDIO_IN_INT_IRQ, AUDIO_IN_IRQ_PREPRIO, 0);
    HAL_NVIC_EnableIRQ(AUDIO_IN_INT_IRQ);
}




void AudioInit(uint32_t AudioFreq)
{
	SAIx_In_DeInit();
	SAIx_Out_DeInit();
	SAI_Clock_Configuration(AudioFreq);
    haudio_out_sai.Instance = AUDIO_OUT_SAIx;
    haudio_in_sai.Instance = AUDIO_IN_SAIx;
    if (HAL_SAI_GetState(&haudio_in_sai) == HAL_SAI_STATE_RESET)
    {
    	Sai_Out_MspInit(&haudio_out_sai);
    	Sai_In_MspInit(&haudio_in_sai);
    }
    SAIx_Out_Init(AudioFreq);
    SAIx_In_Init(AudioFreq);

}






/**
 * @brief Inicjalizacja modułu audio
 */
void Audio_passThrough(void) {


    /* Initialize the codec */
	AudioInit(frequency);

    if (HAL_SAI_Receive_DMA(&haudio_in_sai, (uint8_t*)record_buffer, BUFFER_SIZE_SAMPLES))
    {
    	Error_Handler();
        return 1;
    }


    if (HAL_SAI_Transmit_DMA(&haudio_out_sai, (uint8_t*) play_buffer, BUFFER_SIZE_SAMPLES) !=  HAL_OK)
    {
    	Error_Handler();
        return 1;

    }



    printf("Audio initialized successfully!\n");

    audio_rec_buffer_state = BUFFER_OFFSET_NONE;
     while (1)
     {
         /* 1st or 2nd half of the record buffer ready for being copied
         to the Playback buffer */
         if (audio_rec_buffer_state != BUFFER_OFFSET_NONE)
         {
             /* Copy half of the record buffer to the playback buffer */
       	  if (audio_rec_buffer_state == BUFFER_OFFSET_HALF)
       	  {

       	      CopyBuffer(&play_buffer[0],
       	    		  	 &record_buffer[0],
					     BUFFER_SIZE_SAMPLES / 2);


       	  } else {
       	      /* if(audio_rec_buffer_state == BUFFER_OFFSET_FULL) */
       	      CopyBuffer(&play_buffer[BUFFER_SIZE_SAMPLES / 2],
       	                 &record_buffer[BUFFER_SIZE_SAMPLES / 2],
						 BUFFER_SIZE_SAMPLES / 2);

       	  }
             /* Wait for next data */
             audio_rec_buffer_state = BUFFER_OFFSET_NONE;
         }
         if (audio_tx_buffer_state)
         {
             audio_tx_buffer_state = 0;
         }
     } // end while(1)
}


