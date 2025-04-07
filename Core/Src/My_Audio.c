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
// **Zmienna globalna do konfiguracji audio**


// **Bufory audio**
int16_t record_buffer[BUFFER_SIZE_SAMPLES];
int16_t play_buffer[BUFFER_SIZE_SAMPLES];
int16_t compressed_buffer[BUFFER_SIZE_SAMPLES / 2]; // bo usuwasz połowę danych

static uint32_t frequency = AUDIO_FREQUENCY_44K;
static uint8_t volume = 80;
SAI_HandleTypeDef               haudio_out_sai;
SAI_HandleTypeDef               haudio_in_sai;
AUDIO_DrvTypeDef *audio_drv = &wm8994_drv;
DMA_HandleTypeDef hdma_sai_tx;
DMA_HandleTypeDef hdma_sai_rx;

extern osMessageQId AudioQueueHandle;
extern TaskHandle_t xRecordingTaskHandle;

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
void CodekInit(AUDIO_DrvTypeDef  *audio_drv, uint32_t freq);
void AudioInit(uint32_t AudioFreq);



extern SemaphoreHandle_t recordTriggerSemaphore;
extern SemaphoreHandle_t xBufferReadySemaphore;
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



void CodekInit(AUDIO_DrvTypeDef  *audio_drv, uint32_t AudioFreq)
{
    audio_drv->Reset(AUDIO_I2C_ADDRESS);

    /* Inicjalizacja kodeka */
    audio_drv->Init(AUDIO_I2C_ADDRESS, INPUT_DEVICE_INPUT_LINE_1 | OUTPUT_DEVICE_HEADPHONE, 100, AudioFreq);

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
    CodekInit(audio_drv, AudioFreq);

}


/* USER CODE BEGIN Header_StartRecordTask */
/**
* @brief Function implementing the Record_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRecordTask */
void StartRecordTask(void const * argument)
{
    int recordingComplete = 0;
    HAL_StatusTypeDef res;
    xRecordingTaskHandle = xTaskGetCurrentTaskHandle();
    AudioChunk_t chunk;

    for (;;)
    {
        TickType_t startTime = xTaskGetTickCount();
        TickType_t stopTime = startTime + pdMS_TO_TICKS(60000);
        recordingComplete = 0;
        audio_rec_buffer_state = BUFFER_OFFSET_NONE;

        if (xSemaphoreTake(recordTriggerSemaphore, portMAX_DELAY) == pdTRUE) {

            // Initialize the DMA to receive and transmit audio data
            if (HAL_SAI_Receive_DMA(&haudio_in_sai, (uint8_t*)record_buffer, BUFFER_SIZE_SAMPLES) != HAL_OK)
            {
                Error_Handler();
            }
            if (HAL_SAI_Transmit_DMA(&haudio_out_sai, (uint8_t*)play_buffer, BUFFER_SIZE_SAMPLES) != HAL_OK)
            {
                Error_Handler();
                return;
            }

            while (!recordingComplete)
            {
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(portMAX_DELAY)); // Block until notified

                if (audio_rec_buffer_state != BUFFER_OFFSET_NONE)
                {
            		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
                    int16_t* source_ptr = NULL;

                    if (audio_rec_buffer_state == BUFFER_OFFSET_HALF)
                    {
                        source_ptr = &record_buffer[0];
                    }
                    else if (audio_rec_buffer_state == BUFFER_OFFSET_FULL)
                    {
                        source_ptr = &record_buffer[BUFFER_SIZE_SAMPLES / 2];
                    }

                    // Skopiuj tylko slot 0 i 2 (L i R)
                    size_t out_idx = 0;
                    for (size_t i = 0; i < BUFFER_SIZE_SAMPLES / 2; i += 4)
                    {
                        compressed_buffer[out_idx++] = source_ptr[i];     // slot 0 - Left
                        compressed_buffer[out_idx++] = source_ptr[i + 2]; // slot 2 - Right
                    }

                    chunk.data = compressed_buffer;
                    chunk.length = out_idx; // liczba próbek stereo

                    xQueueSend(AudioQueueHandle, &chunk, portMAX_DELAY);
                    audio_rec_buffer_state = BUFFER_OFFSET_NONE;

                    if (xTaskGetTickCount() >= stopTime)
                    {

                    	for(int i=0; i<10; i++)
                    	{
                    		HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
                    		osDelay(330);
                    		HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
                    	}

                        HAL_SAI_DMAStop(&haudio_in_sai);
                        HAL_SAI_DMAStop(&haudio_out_sai);
                		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

                        recordingComplete = 1;
                        break;
                    }
                }
            }

            // Send end signal to the queue
            AudioChunk_t endSignal = { .data = NULL, .length = 0 };
            xQueueSend(AudioQueueHandle, &endSignal, portMAX_DELAY);
        }
    }
}
