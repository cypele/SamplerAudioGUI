/*
 * My_Audio.h
 *
 * Created on: Feb2, 2025
 * Author: adamc
 */

#ifndef MY_AUDIO_H
#define MY_AUDIO_H

#include "stdint.h"
#include "main.h"

//
// Konfiguracja buforów i próbkowania audio
//
#define BUFFER_SIZE_SAMPLES     4096              // Liczba próbek audio
#define DMA_BYTES_PER_FRAME     4                 // Bajty na ramkę DMA (4 sloty × 2 bajty)
#define DMA_BYTES_PER_MSIZE     2                 // Bajty na element (16-bit)
#define DMA_BUFFER_SIZE_BYTES   (BUFFER_SIZE_SAMPLES * DMA_BYTES_PER_FRAME)
#define DMA_BUFFER_MSIZE        (DMA_BUFFER_SIZE_BYTES / DMA_BYTES_PER_MSIZE)

#define SAMPLE_RATE             44100
#define DURATION_SEC            3
#define CHUNK_SIZE_BYTES        4096
#define MAX_CHUNKS              ((SAMPLE_RATE * 2 * 2 * DURATION_SEC) / CHUNK_SIZE_BYTES) // ~129

//
// Statusy audio
//
#define AUDIO_OK                ((uint8_t)0)
#define AUDIO_ERROR             ((uint8_t)1)
#define AUDIO_TIMEOUT           ((uint8_t)2)

//
// Domyślna konfiguracja wejścia audio
//
#define DEFAULT_AUDIO_IN_FREQ               BSP_AUDIO_FREQUENCY_16K
#define DEFAULT_AUDIO_IN_BIT_RESOLUTION     ((uint8_t)16)
#define DEFAULT_AUDIO_IN_CHANNEL_NBR        ((uint8_t)2)
#define DEFAULT_AUDIO_IN_VOLUME             ((uint16_t)64)

//
// Konwersje głośności
//
#define VOLUME_CONVERT(Volume)        (((Volume) > 100)? 100:((uint8_t)(((Volume) * 63) / 100)))
#define VOLUME_IN_CONVERT(Volume)     (((Volume) >= 100)? 239:((uint8_t)(((Volume) * 240) / 100)))

#define AUDIO_MUTE_ON                 1
#define AUDIO_MUTE_OFF                0

//
// Konfiguracja slotów ramki audio (SAI, tryb TDM)
//
#define CODEC_AUDIOFRAME_SLOT_0123   (SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1 | SAI_SLOTACTIVE_2 | SAI_SLOTACTIVE_3) // Słuchawki + głośnik
#define CODEC_AUDIOFRAME_SLOT_02     (SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_2) // Tylko słuchawki
#define CODEC_AUDIOFRAME_SLOT_13     (SAI_SLOTACTIVE_1 | SAI_SLOTACTIVE_3) // Tylko głośnik

//
// Konfiguracja peryferiów SAI (wyjście)
//
#define AUDIO_OUT_SAIx                           SAI1_Block_A
#define AUDIO_OUT_SAIx_CLK_ENABLE()              __HAL_RCC_SAI1_CLK_ENABLE()
#define AUDIO_OUT_SAIx_CLK_DISABLE()             __HAL_RCC_SAI1_CLK_DISABLE()
#define AUDIO_OUT_SAIx_AF                        GPIO_AF6_SAI1

#define AUDIO_OUT_SAIx_MCLK_ENABLE()             __HAL_RCC_GPIOG_CLK_ENABLE()
#define AUDIO_OUT_SAIx_MCLK_GPIO_PORT            GPIOG
#define AUDIO_OUT_SAIx_MCLK_PIN                  GPIO_PIN_7

#define AUDIO_OUT_SAIx_SD_FS_CLK_ENABLE()        __HAL_RCC_GPIOE_CLK_ENABLE()
#define AUDIO_OUT_SAIx_SD_FS_SCK_GPIO_PORT       GPIOE
#define AUDIO_OUT_SAIx_FS_PIN                    GPIO_PIN_4
#define AUDIO_OUT_SAIx_SCK_PIN                   GPIO_PIN_5
#define AUDIO_OUT_SAIx_SD_PIN                    GPIO_PIN_6

//
// DMA dla wyjścia audio
//
#define AUDIO_OUT_SAIx_DMAx_CLK_ENABLE()         __HAL_RCC_DMA2_CLK_ENABLE()
#define AUDIO_OUT_SAIx_DMAx_STREAM               DMA2_Stream1
#define AUDIO_OUT_SAIx_DMAx_CHANNEL              DMA_CHANNEL_0
#define AUDIO_OUT_SAIx_DMAx_IRQ                  DMA2_Stream1_IRQn
#define AUDIO_OUT_SAIx_DMAx_PERIPH_DATA_SIZE     DMA_PDATAALIGN_HALFWORD
#define AUDIO_OUT_SAIx_DMAx_MEM_DATA_SIZE        DMA_MDATAALIGN_HALFWORD
#define DMA_MAX_SZE                              0xFFFF
#define AUDIO_OUT_SAIx_DMAx_IRQHandler           DMA2_Stream1_IRQHandler

//
// DMA dla wejścia audio
//
#define AUDIO_IN_SAIx_DMAx_CLK_ENABLE()          __HAL_RCC_DMA2_CLK_ENABLE()
#define AUDIO_IN_SAIx_DMAx_STREAM                DMA2_Stream4
#define AUDIO_IN_SAIx_DMAx_CHANNEL               DMA_CHANNEL_1
#define AUDIO_IN_SAIx_DMAx_IRQ                   DMA2_Stream4_IRQn
#define AUDIO_IN_SAIx_DMAx_PERIPH_DATA_SIZE      DMA_PDATAALIGN_HALFWORD
#define AUDIO_IN_SAIx_DMAx_MEM_DATA_SIZE         DMA_MDATAALIGN_HALFWORD

//
// Wejście INT audio
//
#define AUDIO_IN_INT_GPIO_ENABLE()               __HAL_RCC_GPIOJ_CLK_ENABLE()
#define AUDIO_IN_INT_GPIO_PORT                   GPIOJ
#define AUDIO_IN_INT_GPIO_PIN                    GPIO_PIN_12
#define AUDIO_IN_INT_IRQ                         EXTI15_10_IRQn

#define AUDIO_OUT_IRQ_PREPRIO                    ((uint32_t)0x0E)

//
// Standardy kodeka
//
#define CODEC_STANDARD                           0x04
#define I2S_STANDARD                             I2S_STANDARD_PHILIPS

//
// Konfiguracja wejścia audio (SAI)
//
#define AUDIO_IN_SAIx                            SAI1_Block_B
#define AUDIO_IN_SAIx_CLK_ENABLE()               __HAL_RCC_SAI1_CLK_ENABLE()
#define AUDIO_IN_SAIx_CLK_DISABLE()              __HAL_RCC_SAI1_CLK_DISABLE()
#define AUDIO_IN_SAIx_AF                         GPIO_AF6_SAI1

#define AUDIO_IN_SAIx_SD_ENABLE()                __HAL_RCC_GPIOE_CLK_ENABLE()
#define AUDIO_IN_SAIx_SD_GPIO_PORT               GPIOE
#define AUDIO_IN_SAIx_SD_PIN                     GPIO_PIN_3

#define AUDIO_IN_IRQ_PREPRIO                     ((uint32_t)0x0F)

//
// Częstotliwości próbkowania audio
//
#define AUDIO_FREQUENCY_192K          ((uint32_t)192000)
#define AUDIO_FREQUENCY_96K           ((uint32_t)96000)
#define AUDIO_FREQUENCY_48K           ((uint32_t)48000)
#define AUDIO_FREQUENCY_44K           ((uint32_t)44100)
#define AUDIO_FREQUENCY_32K           ((uint32_t)32000)
#define AUDIO_FREQUENCY_22K           ((uint32_t)22050)
#define AUDIO_FREQUENCY_16K           ((uint32_t)16000)
#define AUDIO_FREQUENCY_11K           ((uint32_t)11025)
#define AUDIO_FREQUENCY_8K            ((uint32_t)8000)

//
// Urządzenia wejścia/wyjścia audio
//
#define OUTPUT_DEVICE_SPEAKER                 ((uint16_t)0x0001)
#define OUTPUT_DEVICE_HEADPHONE               ((uint16_t)0x0002)
#define OUTPUT_DEVICE_BOTH                    ((uint16_t)0x0003)
#define OUTPUT_DEVICE_AUTO                    ((uint16_t)0x0004)

#define INPUT_DEVICE_DIGITAL_MICROPHONE_1     ((uint16_t)0x0100)
#define INPUT_DEVICE_DIGITAL_MICROPHONE_2     ((uint16_t)0x0200)
#define INPUT_DEVICE_INPUT_LINE_1             ((uint16_t)0x0300)
#define INPUT_DEVICE_INPUT_LINE_2             ((uint16_t)0x0400)
#define INPUT_DEVICE_DIGITAL_MIC1_MIC2        ((uint16_t)0x0800)

//
// Głośność – wartości domyślne
//
#define DEFAULT_VOLMIN                0x00
#define DEFAULT_VOLMAX                0xFF
#define DEFAULT_VOLSTEP               0x04

//
// Polecenia odtwarzania
//
#define AUDIO_PAUSE                   0
#define AUDIO_RESUME                  1

//
// Tryby uśpienia kodeka
//
#define CODEC_PDWN_HW                 1
#define CODEC_PDWN_SW                 2

//
// Komendy wyciszenia
//
#define AUDIO_MUTE_ON                 1
#define AUDIO_MUTE_OFF                0

//
// Adres I2C kodeka audio
//
#define AUDIO_I2C_ADDRESS             ((uint16_t)0x34)

//
// Stan bufora audio
//


//
// Deklaracje zewnętrznych zmiennych globalnych
//
extern volatile uint32_t audio_rec_buffer_state;
extern volatile uint32_t audio_tx_buffer_state;

#endif /* MY_AUDIO_H */
