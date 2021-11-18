/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MX25__H_
#define _MX25__H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
//#include "../Components/MX25L25673GM2I/MX25L25673GM2I.h"

  
/* Exported constants --------------------------------------------------------*/
#define MX25_OK            ((uint8_t)0x00)
#define MX25_ERROR         ((uint8_t)0x01)
#define MX25_BUSY          ((uint8_t)0x02)
#define MX25_NOT_SUPPORTED ((uint8_t)0x04)
#define MX25_SUSPENDED     ((uint8_t)0x08)

#define MX25_FLASH_SIZE                      0x2000000 /* 256 MBits => 32MBytes */
#define MX25_SUBSECTOR_SIZE                  0x1000    /* 4096 subsectors of 4kBytes */
#define MX25_PAGE_SIZE                       0x100     /* 65536 pages of 256 bytes */

#define MX25_DUMMY_CYCLES_READ_QUAD          6
#define MX25_BULK_ERASE_MAX_TIME             250000
#define MX25_SUBSECTOR_ERASE_MAX_TIME        800

/*** Reset Operations ***/
#define MX25_RESET_ENABLE_CMD__RSTEN         0x66

/*** Identification Operations ***/
#define MX25_READ_ID_CMD__RDID               0x9F

/*** Read Operations ***/
#define MX25_READ_CMD__READ                  0x03
#define MX25_4READ_CMD__4READ                0xEB

/*** Write Operations ***/
#define MX25_WRITE_EN_CMD__WREN              0x06

/*** Config ***/
#define MX25_READ_CFG_CMD__RDCR              0x15
#define MX25_WRITE_CFG_SR_CMD__WRSR          0x01
#define MX25_READ_STATUS_CMD__RDSR           0x05

/*** Program Operations ***/
#define MX25_PAGE_PROG_CMD__PP               0x02
#define MX25_FAST_PROG_CMD__PP4B             0x38

/*** Erase Operations ***/
#define MX25_ERASE_64K_CMD__BE4B             0xDC
#define MX25_CHIP_ERASE_CMD__CE              0xC7

/*** Status Register ***/
#define MX25_SR_WIP                         ((uint8_t)0x01)
#define MX25_SR_WEL                         ((uint8_t)0x02)

/* Exported types ------------------------------------------------------------*/
typedef struct {
  uint32_t FlashSize;
  uint32_t EraseSectorSize;
  uint32_t EraseSectorsNumber;
  uint32_t ProgPageSize;
  uint32_t ProgPagesNumber;
} MX25_InfoTypeDef;

/* Exported functions --------------------------------------------------------*/
uint8_t MX25_Init       (QSPI_HandleTypeDef *hqspi);
uint8_t MX25_ReadId(uint8_t *id, uint32_t size);
uint8_t MX25_NormalRead(uint8_t* data, uint32_t address, uint32_t size);
uint8_t MX25_Read       (uint8_t* data, uint32_t address, uint32_t size);
uint8_t MX25_NormalWrite(uint8_t* data, uint32_t addr, uint32_t size);
uint8_t MX25_Write      (uint8_t* data, uint32_t WriteAddr, uint32_t Size);
uint8_t MX25_Erase64kBlock(uint32_t BlockAddress);
uint8_t MX25_EraseChip  (void);
uint8_t MX25_GetInfo    (MX25_InfoTypeDef* pInfo);

uint8_t MX25_ResetMemory          (QSPI_HandleTypeDef *hqspi);
uint8_t MX25_Config               (QSPI_HandleTypeDef *hqspi);
uint8_t MX25_WriteEnable          (QSPI_HandleTypeDef *hqspi);
uint8_t MX25_AutoPollingMemReady  (QSPI_HandleTypeDef *hqspi, uint32_t Timeout);



#ifdef __cplusplus
}
#endif

#endif
