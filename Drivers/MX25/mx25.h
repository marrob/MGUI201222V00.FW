/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MX25__H_
#define _MX25__H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx.h"


  
/* Exported constants --------------------------------------------------------*/
#define MX25_OK            ((uint8_t)0x00)
#define MX25_ERROR         ((uint8_t)0x01)
#define MX25_BUSY          ((uint8_t)0x02)
#define MX25_NOT_SUPPORTED ((uint8_t)0x04)
#define MX25_SUSPENDED     ((uint8_t)0x08)

#define MX25_FLASH_SIZE                      0x2000000 /* 256 MBits => 32MBytes */
#define MX25_SUBSECTOR_SIZE                  0x1000    /* 4096 subsectors of 4kBytes */
#define MX25_PAGE_SIZE                       0x100     /* 65536 pages of 256 bytes */

#define MX25_4L_DUMMY_CYCLES_READ            6
#define MX25_BULK_ERASE_MAX_TIME             250000
#define MX25_SUBSECTOR_ERASE_MAX_TIME        800

/*** Identification Operations ***/
#define MX25_1L_READ_ID_CMD                  0x9F

/*** Reset Operations ***/
#define MX25_1L_RESET_ENABLE_CMD             0x66
#define MX25_1L_RESET_CMD                    0x99

/*** Read Operations ***/
#define MX25_3B_1L_READ                      0x03
#define MX25_4B_1L_READ                      0x13
#define MX25_4B_4L_READ                      0xEC

/*** Write Operations ***/
#define MX25_1L_WRITE_EN_CMD                 0x06

/*** Config ***/
#define MX25_1L_READ_CFG_CMD                 0x15
#define MX25_1L_WRITE_CFG_SR_CMD             0x01
#define MX25_1L_READ_STATUS_CMD              0x05

/*** Program Operations ***/
#define MX25_3B_1L_PP_CMD                    0x02
#define MX25_4B_1L_PP_CMD                    0x12
#define MX25_4B_4L_PP_CMD                    0x3E

/*** Erase Operations ***/
#define MX25_3B_1L_SECTOR_ERASE_CMD          0x20
#define MX25_4B_1L_SECTOR_ERASE_CMD          0xDC
#define MX25_1L_BULK_ERASE_CMD               0xC7

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
uint8_t MX25_Init(QSPI_HandleTypeDef *hqspi);
uint8_t MX25_ReadId(QSPI_HandleTypeDef *hqspi, uint8_t *id, uint32_t size);
uint8_t MX25_NormalRead(QSPI_HandleTypeDef *hqspi, uint8_t* data, uint32_t address, uint32_t size);
uint8_t MX25_Read       (QSPI_HandleTypeDef *hqspi, uint8_t* data, uint32_t address, uint32_t size);
uint8_t MX25_NormalWrite(QSPI_HandleTypeDef *hqspi, uint8_t* data, uint32_t addr, uint32_t size);
uint8_t MX25_Write      (QSPI_HandleTypeDef *hqspi, uint8_t* data, uint32_t WriteAddr, uint32_t Size);
uint8_t MX25_EraseBlock (QSPI_HandleTypeDef *hqspi, uint32_t blockAddress);
uint8_t MX25_EraseChip  (QSPI_HandleTypeDef *hqspi);
uint8_t MX25_GetInfo    (MX25_InfoTypeDef* pInfo);
uint8_t MX25_EnableMemoryMappedMode(QSPI_HandleTypeDef *hqspi);

uint8_t MX25_ResetMemory          (QSPI_HandleTypeDef *hqspi);
uint8_t MX25_Config               (QSPI_HandleTypeDef *hqspi);
uint8_t MX25_WriteEnable          (QSPI_HandleTypeDef *hqspi);
uint8_t MX25_AutoPollingMemReady  (QSPI_HandleTypeDef *hqspi, uint32_t Timeout);



#ifdef __cplusplus
}
#endif

#endif
