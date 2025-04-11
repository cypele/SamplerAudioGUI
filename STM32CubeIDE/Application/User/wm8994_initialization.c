/*
 * wm8994_initialization.c
 *
 *  Created on: Apr 9, 2025
 *      Author: adamc
 */


#include "stdio.h"
#include "main.h"

#define VOLUME_CONVERT(Volume)        (((Volume) > 100)? 100:((uint8_t)(((Volume) * 63) / 100)))
#define VOLUME_IN_CONVERT(Volume)     (((Volume) >= 100)? 239:((uint8_t)(((Volume) * 240) / 100)))
#define AUDIO_MUTE_ON                 1
#define AUDIO_MUTE_OFF                0

extern I2C_HandleTypeDef hi2c4;
extern uint8_t volume;

uint32_t outputEnabled, inputEnabled;



uint8_t wm8994_Init(void);
HAL_StatusTypeDef WM8994_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint16_t reg, uint16_t value);
uint32_t wm8994_SetMute(uint16_t DeviceAddr, uint32_t Cmd);
uint32_t wm8994_SetVolume(uint16_t DeviceAddr, uint8_t Volume);
uint8_t WriteCodecRegister_WithVeriy(uint8_t deviceAddr, uint16_t reg, uint16_t value);
void I2C4_AudioInit();



uint8_t wm8994_Init(void)
{
	  uint8_t counter = 0;
	  uint16_t power_mgnt_reg_1 = 0;
	  uint8_t   CODEC_I2C_ADDR;

	  //I2C4_AudioInit();

	  inputEnabled=0;
	  outputEnabled=0;
	  CODEC_I2C_ADDR = 0x34;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x0000, 0x0000) != HAL_OK) counter++;

	  /* wm8994 Errata Work-Arounds */
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x102, 0x0003) != HAL_OK) counter++;

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x817, 0x0000) != HAL_OK) counter++;

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x102, 0x0000) != HAL_OK) counter++;

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x39, 0x006C) != HAL_OK) counter++;

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x01, 0x0013) != HAL_OK) counter++;

	  outputEnabled = 1;
	  HAL_Delay(50);

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x05, 0x0303) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x601, 0x0001) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x602, 0x0001) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x604, 0x0000) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x605, 0x0000) != HAL_OK) counter++;
	  inputEnabled = 1;

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x28, 0x0011) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x29, 0x0020) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x2A, 0x0020) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x04, 0x0303) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x440, 0x00DB) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x02, 0x6350) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x606, 0x0002) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x607, 0x0002) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x700, 0x000D) != HAL_OK) counter++;

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x210, 0x0073) != HAL_OK) counter++;

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x300, 0x4010) != HAL_OK) counter++;

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x302, 0x0000) != HAL_OK) counter++;

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x208, 0x000A) != HAL_OK) counter++;

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x200, 0x0001) != HAL_OK) counter++;

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x03, 0x0300) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x22, 0x0000) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x23, 0x0000) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x36, 0x0300) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x01, 0x3003) != HAL_OK) counter++;

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x51, 0x0005) != HAL_OK) counter++;

	  // Dodanie bitów do power_mgnt_reg_1
	  power_mgnt_reg_1 |= 0x0303 | 0x3003;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x01, power_mgnt_reg_1) != HAL_OK) counter++;

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x60, 0x0022) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x4C, 0x9F25) != HAL_OK) counter++;

	  HAL_Delay(15);

	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x2D, 0x0001) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x2E, 0x0001) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x03, 0x0030 | 0x0300) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x54, 0x0033) != HAL_OK) counter++;

	  HAL_Delay(257);


	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x60, 0x00EE) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x610, 0x00C0) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x611, 0x00C0) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x420, 0x0000) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x612, 0x00C0) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x613, 0x00C0) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x422, 0x0000) != HAL_OK) counter++;

	  wm8994_SetVolume(CODEC_I2C_ADDR, volume);


	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x18, 0x000B) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x1A, 0x000B) != HAL_OK) counter++;
	  if (WM8994_WriteRegister(&hi2c4, CODEC_I2C_ADDR, 0x410, 0x7800) != HAL_OK) counter++;


	  wm8994_SetVolume(CODEC_I2C_ADDR, volume);
	  if(counter != 0)
		  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
	  return counter;
}

uint8_t WriteCodecRegister_WithVeriy(uint8_t deviceAddr, uint16_t reg, uint16_t value)
{
    HAL_StatusTypeDef status;
    uint16_t swappedValue;
    uint16_t readBack = 0;
    uint8_t result = 0;

    // Zamiana bajtów (big endian) jak w AUDIO_IO_Write
    swappedValue = ((value >> 8) & 0x00FF) | ((value << 8) & 0xFF00);

    // Wysyłka
    status = HAL_I2C_Mem_Write(&hi2c4, deviceAddr, reg, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&swappedValue, 2, 1000);

    if (status != HAL_OK)
    {
        // Jeśli błąd — spróbuj zresetować I2C jak w I2Cx_Error
        HAL_I2C_DeInit(&hi2c4);
        I2C4_AudioInit();
        return result+1;
    }

#ifdef VERIFY_WRITTENDATA
    // Weryfikacja zapisu
    status = HAL_I2C_Mem_Read(&hi2c4, deviceAddr, reg, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&readBack, 2, 1000);
    if (status == HAL_OK)
    {
        // Tu też trzeba zamienić bajty, bo WM8994 to big endian
        readBack = ((readBack >> 8) & 0x00FF) | ((readBack << 8) & 0xFF00);
        if (readBack != value)
        {
            result = 1;
        }
    }
    else
    {
        result = 1;
    }
#endif

    return result;
}


HAL_StatusTypeDef WM8994_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint16_t reg, uint16_t value)
{
    HAL_StatusTypeDef status;
    volatile uint8_t data[2];
    data[0] = (value >> 8) & 0xFF;  // MSB
    data[1] = value & 0xFF;         // LSB


    // Zapis do rejestru przez I2C
    status = HAL_I2C_Mem_Write(hi2c, devAddr, reg, I2C_MEMADD_SIZE_16BIT, data, 2, 1000);

    // LED feedback
    if (status == HAL_OK)
    {

    }
    else
    {
    	I2C4_AudioInit();
    }

    return status;
}


uint32_t wm8994_SetMute(uint16_t DeviceAddr, uint32_t Cmd)
{
  uint32_t counter = 0;

  if (outputEnabled != 0)
  {
    /* Set the Mute mode */
    if(Cmd == AUDIO_MUTE_ON)
    {
      /* Soft Mute the AIF1 Timeslot 0 DAC1 path L&R */
      if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x420, 0x0200) != HAL_OK) counter++;


      /* Soft Mute the AIF1 Timeslot 1 DAC2 path L&R */
      if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x422, 0x0200) != HAL_OK) counter++;

    }
    else /* AUDIO_MUTE_OFF Disable the Mute */
    {
      /* Unmute the AIF1 Timeslot 0 DAC1 path L&R */
      if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x420, 0x0000) != HAL_OK) counter++;

      /* Unmute the AIF1 Timeslot 1 DAC2 path L&R */
      if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x422, 0x0000) != HAL_OK) counter++;

    }
  }
  return counter;
}


uint32_t wm8994_SetVolume(uint16_t DeviceAddr, uint8_t Volume)
{
  uint32_t counter = 0;
  uint8_t convertedvol = VOLUME_CONVERT(Volume);

  /* Output volume */
  if (outputEnabled != 0)
  {
    if(convertedvol > 0x3E)
    {
      /* Unmute audio codec */
      counter += wm8994_SetMute(DeviceAddr, AUDIO_MUTE_OFF);


      /* Left Headphone Volume */
      if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x1C, 0x3F | 0x140) != HAL_OK) counter++;


      /* Right Headphone Volume */
      if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x1D,  0x3F | 0x140) != HAL_OK) counter++;


      /* Left Speaker Volume */
      if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x26,  0x3F | 0x140) != HAL_OK) counter++;


      /* Right Speaker Volume */
      if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x27, 0x3F | 0x140) != HAL_OK) counter++;

    }
    else if (Volume == 0)
    {
      /* Mute audio codec */
      counter += wm8994_SetMute(DeviceAddr, AUDIO_MUTE_ON);

    }
    else
    {
      /* Unmute audio codec */
      counter += wm8994_SetMute(DeviceAddr, AUDIO_MUTE_OFF);

      /* Left Headphone Volume */
      if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x1C, convertedvol) != HAL_OK) counter++;


      /* Right Headphone Volume */
      if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x1D, convertedvol | 0x140) != HAL_OK) counter++;


      /* Left Speaker Volume */
      if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x26, convertedvol | 0x140) != HAL_OK) counter++;


      /* Right Speaker Volume */
      if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x422, convertedvol | 0x140) != HAL_OK) counter++;


    }
  }

  /* Input volume */
  if (inputEnabled != 0)
  {
    convertedvol = VOLUME_IN_CONVERT(Volume);

    /* Left AIF1 ADC1 volume */
    if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x400, convertedvol | 0x100) != HAL_OK) counter++;


    /* Right AIF1 ADC1 volume */
    if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x401, convertedvol | 0x100) != HAL_OK) counter++;

    /* Left AIF1 ADC2 volume */
    if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x404, convertedvol | 0x100) != HAL_OK) counter++;

    /* Right AIF1 ADC2 volume */
    if (WM8994_WriteRegister(&hi2c4, DeviceAddr, 0x405, convertedvol | 0x100) != HAL_OK) counter++;

  }
  return counter;
}




void I2C4_AudioInit()
{

	HAL_I2C_DeInit(&hi2c4);
	I2C_HandleTypeDef *i2c_handler = &hi2c4;
	i2c_handler->Instance 			   = I2C4;
    i2c_handler->Init.Timing           = 0x40912732;
    i2c_handler->Init.OwnAddress1      = 0;
    i2c_handler->Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    i2c_handler->Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    i2c_handler->Init.OwnAddress2      = 0;
    i2c_handler->Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    i2c_handler->Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;



	  /*** Configure the GPIOs ***/
	  /* Enable GPIO clock */
	GPIO_InitTypeDef  gpio_init_structure;

	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	  /* Configure I2C Tx as alternate function */
	gpio_init_structure.Pin = GPIO_PIN_12;
	gpio_init_structure.Mode = GPIO_MODE_AF_OD;
	gpio_init_structure.Pull = GPIO_NOPULL;
	gpio_init_structure.Speed = GPIO_SPEED_FAST;
	gpio_init_structure.Alternate = GPIO_AF4_I2C4;
	HAL_GPIO_Init(GPIOD, &gpio_init_structure);

	  /* Configure I2C Rx as alternate function */
	gpio_init_structure.Pin = GPIO_PIN_7;
	gpio_init_structure.Alternate = GPIO_AF11_I2C4;
	HAL_GPIO_Init(GPIOB, &gpio_init_structure);
	  /*** Configure the I2C peripheral ***/
	  /* Enable I2C clock */
	__HAL_RCC_I2C4_CLK_ENABLE();

	  /* Force the I2C peripheral clock reset */
	__HAL_RCC_I2C4_FORCE_RESET();

	  /* Release the I2C peripheral clock reset */
	__HAL_RCC_I2C4_RELEASE_RESET();

	  /* Enable and set I2C1 Interrupt to a lower priority */
	HAL_NVIC_SetPriority(I2C4_EV_IRQn, 0x0F, 0);
	HAL_NVIC_EnableIRQ(I2C4_EV_IRQn);

	  /* Enable and set I2C1 Interrupt to a lower priority */
	HAL_NVIC_SetPriority(I2C4_ER_IRQn, 0x0F, 0);
	HAL_NVIC_EnableIRQ(I2C4_ER_IRQn);


	HAL_I2C_Init(i2c_handler);

}
