/**
  ******************************************************************************
  * @file    usbd_storage_msd.c
  * @author  MCD application Team
  * @version V1.2.1
  * @date    17-March-2018
  * @brief   This file provides the disk operations functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      <http://www.st.com/SLA0044>
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------ */
#include "usbd_msc_mem.h"
#include "s25fl132k_spi_flash.h"
#include "FAT.h"
#include "string.h"
/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup STORAGE
  * @brief media storage application module
  * @{
  */

/** @defgroup STORAGE_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup STORAGE_Private_Defines
  * @{
  */

#define STORAGE_LUN_NBR                  1
/**
  * @}
  */


/** @defgroup STORAGE_Private_Macros
  * @{
  */
/**
  * @}
  */


/** @defgroup STORAGE_Private_Variables
  * @{
  */
/* USB Mass storage Standard Inquiry Data */
//const int8_t STORAGE_Inquirydata[] = {  // 36

//  /* LUN 0 */
//  0x00,
//  0x80,
//  0x02,
//  0x02,
//  (USBD_STD_INQUIRY_LENGTH - 5),
//  0x00,
//  0x00,
//  0x00,
//  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
//  'm', 'i', 'c', 'r', 'o', 'S', 'D', ' ', /* Product : 16 Bytes */
//  'F', 'l', 'a', 's', 'h', ' ', ' ', ' ',
//  //'1', '.', '0', '0',           /* Version : 4 Bytes */
//};
const int8_t  STORAGE_Inquirydata[] = { 
  
  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (USBD_STD_INQUIRY_LENGTH - 5),
  0x00,
  0x00,
  0x00,
  'H', 'D', 'S', 'C', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'm', 'i', 'c', 'r', 'o', 'S', 'D', ' ', /* Product : 16 Bytes */
  'F', 'l', 'a', 's', 'h', ' ', ' ', ' ',
  '1', '.', '0', '0',           /* Version : 4 Bytes */
}; 

/**
  * @}
  */
const int8_t  STORAGE_Fatdata[] = { 
  
  /* LUN 0 */
    0xeb,
    0xfe,
    0x90,
    0x4d,

    0x53,
    0x44,
    0x4f,
    0x53,

    0x35,
    0x2e,
    0x30,
    0x00,

    0x02,
    0x08,
    0x01,
    0x00,

    0x01, 
    0x00, 
    0x02, 
    0x00,

    0xc8, 
    0xf0, 
    0x1f, 
    0x00,

    0x3f, 
    0x00, 
    0xff, 
    0x00,

    0x00, 
    0x00, 
    0x00, 
    0x00,

    0x00, 
    0x00, 
    0x00, 
    0x00,

    0x80, 
    0x00, 
    0x29, 
    0x03,

    0x00, 
    0x9c, 
    0x47, 
    0x4e,

    0x4f, 
    0x20, 
    0x4e, 
    0x41,

    0x4d, 
    0x45, 
    0x20, 
    0x20,

    0x20, 
    0x20, 
    0x46, 
    0x41,

    0x54, 
    0x20, 
    0x20, 
    0x20, 

    0x20, 
    0x20, 
    0x00, 
    0x00,
}; 

/** @defgroup STORAGE_Private_FunctionPrototypes
  * @{
  */
int8_t STORAGE_Init(uint8_t lun);

int8_t STORAGE_GetCapacity(uint8_t lun,
                           uint32_t * block_num, uint32_t * block_size);

int8_t STORAGE_IsReady(uint8_t lun);

int8_t STORAGE_IsWriteProtected(uint8_t lun);

int8_t STORAGE_Read(uint8_t lun,
                    uint8_t * buf, uint32_t blk_addr, uint16_t blk_len);

int8_t STORAGE_Write(uint8_t lun,
                     uint8_t * buf, uint32_t blk_addr, uint16_t blk_len);

int8_t STORAGE_GetMaxLun(void);


USBD_STORAGE_cb_TypeDef USBD_MICRO_SDIO_fops = {
  STORAGE_Init,
  STORAGE_GetCapacity,
  STORAGE_IsReady,
  STORAGE_IsWriteProtected,
  STORAGE_Read,
  STORAGE_Write,
  STORAGE_GetMaxLun,
  (int8_t *) STORAGE_Inquirydata,
};

USBD_STORAGE_cb_TypeDef *USBD_STORAGE_fops = &USBD_MICRO_SDIO_fops;

__IO uint32_t count = 0;
/**
  * @}
  */


/** @defgroup STORAGE_Private_Functions
  * @{
  */


/**
  * @brief  Initialize the storage medium
  * @param  lun : logical unit number
  * @retval Status
  */

int8_t STORAGE_Init(uint8_t lun)
{
    S25FL132K_SPI_FLASH_Init();//SPI初始化
  return (0);

}

/**
  * @brief  return medium capacity and block size
  * @param  lun : logical unit number
  * @param  block_num :  number of physical block
  * @param  block_size : size of a physical block
  * @retval Status
  */
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t * block_num,
                           uint32_t * block_size)
{

  *block_size = 512;
  *block_num = 1024*1024*128/512;

  return (0);

}

/**
  * @brief  check whether the medium is ready
  * @param  lun : logical unit number
  * @retval Status
  */
int8_t STORAGE_IsReady(uint8_t lun)
{
  return (0);
}

/**
  * @brief  check whether the medium is write-protected
  * @param  lun : logical unit number
  * @retval Status
  */
int8_t STORAGE_IsWriteProtected(uint8_t lun)
{
  return 0;
}

/**
  * @brief  Read data from the medium
  * @param  lun : logical unit number
  * @param  buf : Pointer to the buffer to save data
  * @param  blk_addr :  address of 1st block to be read
  * @param  blk_len : nmber of blocks to be read
  * @retval Status
  */
uint16_t cnt=0;
int8_t STORAGE_Read(uint8_t lun,
                    uint8_t * buf, uint32_t blk_addr, uint16_t blk_len)
{
    uint16_t tmp;
    cnt++;
    if(blk_addr*512 == 0 || blk_len ==1)  //返回DBR：磁盘操作系统引导操作记录
    {
        for(tmp = 0; tmp<512; tmp++)
        {
            buf[tmp] = Dbr[tmp];
        }
    }
    else
        S25FL132K_SPI_FLASH_BufferRead(buf, blk_addr*512, blk_len*512);
 return 0;
}

/**
  * @brief  Write data to the medium
  * @param  lun : logical unit number
  * @param  buf : Pointer to the buffer to write from
  * @param  blk_addr :  address of 1st block to be written
  * @param  blk_len : nmber of blocks to be read
  * @retval Status
  */
int8_t STORAGE_Write(uint8_t lun,
                     uint8_t * buf, uint32_t blk_addr, uint16_t blk_len)
{
    S25FL132K_SPI_FLASH_BufferWrite(buf, blk_addr*512, blk_len*512);
  return (0);
}

/**
  * @brief  Return number of supported logical unit
  * @param  None
  * @retval number of logical unit
  */

int8_t STORAGE_GetMaxLun(void)
{
  return 0;
}

/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
