/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : TouchGFXHAL.cpp
  ******************************************************************************
  * This file is generated by TouchGFX Generator 4.19.1.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include <TouchGFXHAL.hpp>

/* USER CODE BEGIN TouchGFXHAL.cpp */

#include "stm32f7xx_hal.h"
#include <touchgfx/hal/OSWrappers.hpp>

/* USER CODE BEGIN Includes */
#include <touchgfx/hal/GPIO.hpp>
#include "../otm8009a/otm8009a.h"
#include <CortexMMCUInstrumentation.hpp>
#include <KeySampler.hpp>
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* USER CODE BEGIN private defines */
/**
  * @brief  LCD Display OTM8009A ID
  */
#define LCD_OTM8009A_ID  ((uint32_t) 0)
/* USER CODE END private defines */

/* USER CODE BEGIN private variables */

static bool displayRefreshing = false;
static bool refreshRequested = true;
static int currentRegion = 0;
static uint16_t* currFbBase = 0;

/* USER CODE END private variables */

/* USER CODE BEGIN private functions */

/* USER CODE END private functions */

/* USER CODE BEGIN extern C prototypes */
extern "C" {
    extern DSI_HandleTypeDef hdsi;
    extern LTDC_HandleTypeDef hltdc;

    /* Request tear interrupt at specific scanline. */
    void LCD_ReqTear();

    /* Configures display to update indicated region of the screen (200pixel wide chunks) - 16bpp mode */
    void LCD_SetUpdateRegion(uint8_t region);

    /* LCD Display IO functions */
    void OTM8009A_IO_Delay(uint32_t Delay);
}
/* USER CODE END extern C prototypes */

using namespace touchgfx;

/* USER CODE BEGIN private class objects */
static CortexMMCUInstrumentation instrumentation;
static KeySampler btnctrl;
/* USER CODE END private class objects */

TouchGFXHAL::TouchGFXHAL(touchgfx::DMA_Interface& dma, touchgfx::LCD& display, touchgfx::TouchController& tc, uint16_t width, uint16_t height)
/* USER CODE BEGIN TouchGFXHAL Constructor */
    : TouchGFXGeneratedHAL(dma,
                           display,
                           tc,
                           width,
                           height)
      /* USER CODE END TouchGFXHAL Constructor */
{
    /* USER CODE BEGIN TouchGFXHAL Constructor Code */

    /* USER CODE END TouchGFXHAL Constructor Code */
}

void TouchGFXHAL::initialize()
{
    /* USER CODE BEGIN initialize step 1 */
    /* USER CODE END initialize step 1 */

    // Calling parent implementation of initialize().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.
    // Please note, HAL::initialize() must be called to initialize the framework.

    TouchGFXGeneratedHAL::initialize();

    /* USER CODE BEGIN initialize step 2 */
    lockDMAToFrontPorch(false);

    instrumentation.init();
    setMCUInstrumentation(&instrumentation);
    enableMCULoadCalculation(true);

    setButtonController(&btnctrl);
    /* USER CODE END initialize step 2 */
}

void TouchGFXHAL::taskEntry()
{
    /* USER CODE BEGIN taskEntry step 1 */

    /* USER CODE END taskEntry step 1 */

    enableLCDControllerInterrupt();
    enableInterrupts();

    /* USER CODE BEGIN taskEntry step 2 */

    /* USER CODE END taskEntry step 2 */

    OSWrappers::waitForVSync();
    backPorchExited();

    /* USER CODE BEGIN taskEntry step 3 */
    /* Enable the LCD, Send Display on DCS command to display */
    HAL_DSI_ShortWrite(&hdsi, LCD_OTM8009A_ID, DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_DISPON, 0x00);
    /* USER CODE END taskEntry step 3 */

    for (;;)
    {
        OSWrappers::waitForVSync();
        backPorchExited();
    }
}

/**
 * Gets the frame buffer address used by the TFT controller.
 *
 * @return The address of the frame buffer currently being displayed on the TFT.
 */
uint16_t* TouchGFXHAL::getTFTFrameBuffer() const
{
    // Calling parent implementation of getTFTFrameBuffer().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    /* USER CODE BEGIN getTFTFrameBuffer */
    return currFbBase;
    /* USER CODE END getTFTFrameBuffer */
}

/**
 * Sets the frame buffer address used by the TFT controller.
 *
 * @param [in] address New frame buffer address.
 */
void TouchGFXHAL::setTFTFrameBuffer(uint16_t* address)
{
    // Calling parent implementation of setTFTFrameBuffer(uint16_t* address).
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    /* USER CODE BEGIN setTFTFrameBuffer */
    currFbBase = address;
    TouchGFXGeneratedHAL::setTFTFrameBuffer(address);
    /* USER CODE END setTFTFrameBuffer */
}

/**
 * This function is called whenever the framework has performed a partial draw.
 *
 * @param rect The area of the screen that has been drawn, expressed in absolute coordinates.
 *
 * @see flushFrameBuffer().
 */
void TouchGFXHAL::flushFrameBuffer(const touchgfx::Rect& rect)
{
    // Calling parent implementation of flushFrameBuffer(const touchgfx::Rect& rect).
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.
    // Please note, HAL::flushFrameBuffer(const touchgfx::Rect& rect) must
    // be called to notify the touchgfx framework that flush has been performed.

    /* USER CODE BEGIN flushFrameBuffer step 1 */
    // If the framebuffer is placed in Write Through cached memory (e.g. SRAM) then we need
    // to flush the Dcache to make sure framebuffer is correct in RAM. That's done
    // using SCB_CleanInvalidateDCache().

    SCB_CleanInvalidateDCache();
    /* USER CODE END flushFrameBuffer step 1 */

    /* USER CODE BEGIN flushFrameBuffer step 2 */
    TouchGFXGeneratedHAL::flushFrameBuffer(rect);
    /* USER CODE END flushFrameBuffer step 2 */
}

/**
 * Configures the interrupts relevant for TouchGFX. This primarily entails setting
 * the interrupt priorities for the DMA and LCD interrupts.
 */
void TouchGFXHAL::configureInterrupts()
{
    // Calling parent implementation of configureInterrupts().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    /* USER CODE BEGIN configureInterrupts */
    TouchGFXGeneratedHAL::configureInterrupts();

    // These two priorities MUST be EQUAL, and MUST be functionally lower than
    // RTOS scheduler interrupts.
    HAL_NVIC_SetPriority(DMA2D_IRQn, 7, 0);
    HAL_NVIC_SetPriority(DSI_IRQn, 7, 0);
    /* USER CODE END configureInterrupts */
}

/**
 * Used for enabling interrupts set in configureInterrupts()
 */
void TouchGFXHAL::enableInterrupts()
{
    // Calling parent implementation of enableInterrupts().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    /* USER CODE BEGIN enableInterrupts */
    TouchGFXGeneratedHAL::enableInterrupts();
    NVIC_EnableIRQ(DSI_IRQn);
    /* USER CODE END enableInterrupts */
}

/**
 * Used for disabling interrupts set in configureInterrupts()
 */
void TouchGFXHAL::disableInterrupts()
{
    // Calling parent implementation of disableInterrupts().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    /* USER CODE BEGIN disableInterrupts */
    TouchGFXGeneratedHAL::disableInterrupts();
    NVIC_DisableIRQ(DSI_IRQn);
    /* USER CODE END disableInterrupts */
}

/**
 * Configure the LCD controller to fire interrupts at VSYNC. Called automatically
 * once TouchGFX initialization has completed.
 */
void TouchGFXHAL::enableLCDControllerInterrupt()
{
    // Calling parent implementation of enableLCDControllerInterrupt().
    //
    // To overwrite the generated implementation, omit call to parent function
    // and implemented needed functionality here.

    /* USER CODE BEGIN enableLCDControllerInterrupt */
    LCD_ReqTear();
    __HAL_DSI_CLEAR_FLAG(&hdsi, DSI_IT_ER);
    __HAL_DSI_CLEAR_FLAG(&hdsi, DSI_IT_TE);
    __HAL_DSI_ENABLE_IT(&hdsi, DSI_IT_TE);
    __HAL_DSI_ENABLE_IT(&hdsi, DSI_IT_ER);
    __HAL_LTDC_ENABLE_IT(&hltdc, (LTDC_IT_LI | LTDC_IT_FU)); /* Enable line and FIFO underrun interrupts */
    TouchGFXGeneratedHAL::enableLCDControllerInterrupt();
    /* USER CODE END enableLCDControllerInterrupt */
}

/* USER CODE BEGIN virtual overloaded methods */
void TouchGFXHAL::setFrameBufferStartAddresses(void* frameBuffer, void* doubleBuffer, void* animationStorage)
{
    currFbBase = (uint16_t*)frameBuffer;
    HAL::setFrameBufferStartAddresses(frameBuffer, doubleBuffer, animationStorage);
}

bool TouchGFXHAL::beginFrame()
{
    refreshRequested = false;
    return HAL::beginFrame();
}

void TouchGFXHAL::endFrame()
{
    TouchGFXGeneratedHAL::endFrame();
    if (frameBufferUpdatedThisFrame)
    {
        refreshRequested = true;
    }
}
/* USER CODE END virtual overloaded methods */

/* USER CODE BEGIN extern C functions */
extern "C" {

    /**************************** LINK OTM8009A (Display driver) ******************/
    /**
      * @brief  OTM8009A delay
      * @param  Delay: Delay in ms
      */
    __weak void OTM8009A_IO_Delay(uint32_t Delay)
    {
        HAL_Delay(Delay);
    }

    /**
     * @brief  DCS or Generic short/long write command
     * @param  NbParams: Number of parameters. It indicates the write command mode:
     *                 If inferior to 2, a long write command is performed else short.
     * @param  pParams: Pointer to parameter values table.
     * @retval HAL status
     */
    void DSI_IO_WriteCmd(uint32_t NbrParams, uint8_t* pParams)
    {
        if (NbrParams <= 1)
        {
            HAL_DSI_ShortWrite(&hdsi, LCD_OTM8009A_ID, DSI_DCS_SHORT_PKT_WRITE_P1, pParams[0], pParams[1]);
        }
        else
        {
            HAL_DSI_LongWrite(&hdsi, LCD_OTM8009A_ID, DSI_DCS_LONG_PKT_WRITE, NbrParams, pParams[NbrParams], pParams);
        }
    }
    /****************************END LINK OTM8009A (Display driver) ******************/

    /**
     * Request TE at certain scanline.
     */
    void LCD_ReqTear(void)
    {
        static uint8_t ScanLineParams[2];

        // The display is initialized in portrait mode, so the last "scanline" is 479 - regardless of the actual scanlines
        uint16_t scanline = 479;
        ScanLineParams[0] = scanline >> 8;
        ScanLineParams[1] = scanline & 0x00FF;

        HAL_DSI_LongWrite(&hdsi, LCD_OTM8009A_ID, DSI_DCS_LONG_PKT_WRITE, 2, 0x44, ScanLineParams);
        // set_tear_on
        HAL_DSI_ShortWrite(&hdsi, LCD_OTM8009A_ID, DSI_DCS_SHORT_PKT_WRITE_P1, 0x35, 0x00);
    }

    void LCD_SetUpdateRegion(uint8_t region)
    {
        // define four equal sized regions
        uint8_t pCols[4][4] =
        {
            {0x00, 0x00, 0x00, 0xC7}, /*   0 -> 199 */
            {0x00, 0xC8, 0x01, 0x8F}, /* 200 -> 399 */
            {0x01, 0x90, 0x02, 0x57}, /* 400 -> 599 */
            {0x02, 0x58, 0x03, 0x1F}, /* 600 -> 799 */
        };

        // set the display GRAM write pointer to the start of the current region
        HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_CASET, pCols[region]);

        // Disable DSI Host wrapper to access LTDC
        __HAL_DSI_WRAPPER_DISABLE(&hdsi);

        uint16_t WIDTH = 200;

        // Configure LTDC layer to match region of framebuffer
        LTDC_Layer1->CFBAR = (uint32_t)currFbBase + WIDTH * 2 * currentRegion;

        // configure the LTDC to transfer the data that belongs in the current region
        LTDC->AWCR = ((WIDTH + 2) << 16) | 0x1E2;
        LTDC->TWCR = ((WIDTH + 2 + 1) << 16) | 0x1E3;
        LTDC_Layer1->WHPCR = ((WIDTH + 2) << 16) | 3;
        LTDC_Layer1->CFBLR = (((800 * 2) << 16) | ((WIDTH * 2) + 3));
        LTDC->SRCR = (uint32_t)LTDC_SRCR_IMR;

        //Enable DSI wrapper
        __HAL_DSI_WRAPPER_ENABLE(&hdsi);

        //Start transfer
        HAL_DSI_Refresh(&hdsi);
    }

    void HAL_DSI_TearingEffectCallback(DSI_HandleTypeDef* hdsi)
    {
        HAL::getInstance()->vSync();
        OSWrappers::signalVSync();

        // In single buffering, only require that the system waits for display update to be finished if we
        // actually intend to update the display in this frame.
        if (HAL::USE_DOUBLE_BUFFERING == false)
        {
            HAL::getInstance()->lockDMAToFrontPorch(refreshRequested);
        }

        if (refreshRequested && !displayRefreshing)
        {
            GPIO::set(GPIO::VSYNC_FREQ);
            // We have an update pending.
            if (HAL::getInstance())
            {
                // Swap frame buffers immediately instead of waiting for the task to be scheduled in.
                // Note: task will also swap when it wakes up, but that operation is guarded and will not have
                // any effect if already swapped.
                HAL::getInstance()->swapFrameBuffers();
            }

            // Because the display is designed for portrait view, and it is used in landscape, it scans in the wrong direction.
            // To mitigate this, the display is split into 4 regions that are filled in sequentially
            // This way, the update can be done before the scan line catches up
            currentRegion = 0;
            LCD_SetUpdateRegion(currentRegion);

            // Remember that we are refreshing
            displayRefreshing = true;
        }
    }

    void HAL_DSI_EndOfRefreshCallback(DSI_HandleTypeDef* hdsi)
    {
        if (currentRegion < 3)
        {
            // Configure next region
            currentRegion++;
            LCD_SetUpdateRegion(currentRegion);
        }
        else
        {
            // Otherwise we are done refreshing
            displayRefreshing = false;

            if (HAL::getInstance() && HAL::USE_DOUBLE_BUFFERING == false)
            {
                // Signal to the framework that display update has finished.
                HAL::getInstance()->frontPorchEntered();
            }
            GPIO::clear(GPIO::VSYNC_FREQ);
        }
    }

    portBASE_TYPE IdleTaskHook(void* p)
    {
        if ((int)p) //idle task sched out
        {
            touchgfx::HAL::getInstance()->setMCUActive(true);
        }
        else //idle task sched in
        {
            touchgfx::HAL::getInstance()->setMCUActive(false);
        }
        return pdTRUE;
    }
}
/* USER CODE END extern C functions */

/* USER CODE END TouchGFXHAL.cpp */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
