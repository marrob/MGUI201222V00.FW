
/* Includes ------------------------------------------------------------------*/
#include "mx25.h"

/* Private variables ---------------------------------------------------------*/
QSPI_HandleTypeDef *QSPIHandle;

/* Private functions ---------------------------------------------------------*/
uint8_t MX25_Init(QSPI_HandleTypeDef *hqspi)
{ 
  QSPIHandle = hqspi;

  if (MX25_ResetMemory(QSPIHandle) != MX25_OK)
    return MX25_NOT_SUPPORTED;

  if(MX25_Config(QSPIHandle) != MX25_OK)
    return MX25_ERROR;
 
  MX25_InfoTypeDef pInfo;
  MX25_GetInfo(&pInfo);


  return MX25_OK;
}


uint8_t MX25_ReadId(uint8_t *id, uint32_t size)
{
  QSPI_CommandTypeDef s_command;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX25_READ_ID_CMD__RDID;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = size;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return MX25_ERROR;

  if (HAL_QSPI_Receive(QSPIHandle, id, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return MX25_ERROR;

  return MX25_OK;
}

uint8_t MX25_NormalRead(uint8_t* data, uint32_t address, uint32_t size)
{
  QSPI_CommandTypeDef s_command;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX25_READ_CMD__READ;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.Address           = address;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = size;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return MX25_ERROR;

  /* Set S# timing for Read command */
  MODIFY_REG(QSPIHandle->Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_3_CYCLE);

  if (HAL_QSPI_Receive(QSPIHandle, data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return MX25_ERROR;

  /* Restore S# timing for nonRead commands */
  MODIFY_REG(QSPIHandle->Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_6_CYCLE);

  return MX25_OK;
}

uint8_t MX25_Read(uint8_t* data, uint32_t address, uint32_t size)
{
  QSPI_CommandTypeDef s_command;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX25_4READ_CMD__4READ;
  s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.Address           = address;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = MX25_DUMMY_CYCLES_READ_QUAD;
  s_command.NbData            = size;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  
  if (HAL_QSPI_Command(QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return MX25_ERROR;
  
  /* Set S# timing for Read command */
  MODIFY_REG(QSPIHandle->Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_3_CYCLE);
  
  if (HAL_QSPI_Receive(QSPIHandle, data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return MX25_ERROR;
  
  /* Restore S# timing for nonRead commands */
  MODIFY_REG(QSPIHandle->Instance->DCR, QUADSPI_DCR_CSHT, QSPI_CS_HIGH_TIME_6_CYCLE);

  return MX25_OK;
}

uint8_t MX25_NormalWrite(uint8_t* data, uint32_t address, uint32_t size)
{
  QSPI_CommandTypeDef s_command;
  uint32_t end_addr, current_size, current_addr;

  /* Calculation of the size between the write address and the end of the page */
  current_size = MX25_PAGE_SIZE - (address % MX25_PAGE_SIZE);

  /* Check if the size of the data is less than the remaining place in the page */
  if (current_size > size)
  {
    current_size = size;
  }

  /* Initialize the adress variables */
  current_addr = address;
  end_addr = address + size;

  /* Initialize the program command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX25_PAGE_PROG_CMD__PP;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Perform the write page by page */
  do
  {
    s_command.Address = current_addr;
    s_command.NbData  = current_size;

    /* Enable write operations */
    if (MX25_WriteEnable(QSPIHandle) != MX25_OK)
    {
      return MX25_ERROR;
    }

    /* Configure the command */
    if (HAL_QSPI_Command(QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return MX25_ERROR;
    }

    /* Transmission of the data */
    if (HAL_QSPI_Transmit(QSPIHandle, data, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return MX25_ERROR;
    }

    /* Configure automatic polling mode to wait for end of program */
    if (MX25_AutoPollingMemReady(QSPIHandle, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != MX25_OK)
    {
      return MX25_ERROR;
    }

    /* Update the address and size variables for next page programming */
    current_addr += current_size;
    data += current_size;
    current_size = ((current_addr + MX25_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : MX25_PAGE_SIZE;
  } while (current_addr < end_addr);

  return MX25_OK;
}

uint8_t MX25_Write(uint8_t* pData, uint32_t addr, uint32_t size)
{
  QSPI_CommandTypeDef s_command;
  uint32_t end_addr, current_size, current_addr;

  /* Calculation of the size between the write address and the end of the page */
  current_size = MX25_PAGE_SIZE - (addr % MX25_PAGE_SIZE);

  /* Check if the size of the data is less than the remaining place in the page */
  if (current_size > size)
  {
    current_size = size;
  }

  /* Initialize the adress variables */
  current_addr = addr;
  end_addr = addr + size;

  /* Initialize the program command */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX25_FAST_PROG_CMD__PP4B;
  s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  
  /* Perform the write page by page */
  do
  {
    s_command.Address = current_addr;
    s_command.NbData  = current_size;

    /* Enable write operations */
    if (MX25_WriteEnable(QSPIHandle) != MX25_OK)
    {
      return MX25_ERROR;
    }
    
    /* Configure the command */
    if (HAL_QSPI_Command(QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return MX25_ERROR;
    }
    
    /* Transmission of the data */
    if (HAL_QSPI_Transmit(QSPIHandle, pData, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      return MX25_ERROR;
    }
    
    /* Configure automatic polling mode to wait for end of program */  
    if (MX25_AutoPollingMemReady(QSPIHandle, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != MX25_OK)
    {
      return MX25_ERROR;
    }
    
    /* Update the address and size variables for next page programming */
    current_addr += current_size;
    pData += current_size;
    current_size = ((current_addr + MX25_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : MX25_PAGE_SIZE;
  } while (current_addr < end_addr);
  
  return MX25_OK;
}

uint8_t MX25_Erase64kBlock(uint32_t BlockAddress)
{
  QSPI_CommandTypeDef s_command;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX25_ERASE_64K_CMD__BE4B;
  s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.Address           = BlockAddress;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (MX25_WriteEnable(QSPIHandle) != MX25_OK)
    return MX25_ERROR;

  if (HAL_QSPI_Command(QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return MX25_ERROR;
  
  if (MX25_AutoPollingMemReady(QSPIHandle, MX25_SUBSECTOR_ERASE_MAX_TIME) != MX25_OK)
    return MX25_ERROR;

  return MX25_OK;
}

uint8_t MX25_EraseChip(void)
{
  QSPI_CommandTypeDef s_command;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX25_CHIP_ERASE_CMD__CE;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (MX25_WriteEnable(QSPIHandle) != MX25_OK)
    return MX25_ERROR;

  if (HAL_QSPI_Command(QSPIHandle, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return MX25_ERROR;
  
  if (MX25_AutoPollingMemReady(QSPIHandle, MX25_BULK_ERASE_MAX_TIME) != MX25_OK)
    return MX25_ERROR;

  return MX25_OK;
}

uint8_t MX25_GetInfo(MX25_InfoTypeDef* pInfo)
{
  pInfo->FlashSize          = MX25_FLASH_SIZE;
  pInfo->EraseSectorSize    = MX25_SUBSECTOR_SIZE;
  pInfo->EraseSectorsNumber = (MX25_FLASH_SIZE/MX25_SUBSECTOR_SIZE);
  pInfo->ProgPageSize       = MX25_PAGE_SIZE;
  pInfo->ProgPagesNumber    = (MX25_FLASH_SIZE/MX25_PAGE_SIZE);
  
  return MX25_OK;
}

uint8_t MX25_ResetMemory(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef s_command;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX25_RESET_ENABLE_CMD__RSTEN;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return MX25_ERROR;
  }
  return MX25_OK;
}


uint8_t MX25_Config(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef s_command;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX25_READ_CFG_CMD__RDCR;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = 2;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return MX25_ERROR;

  uint8_t rx_satus_config[] = { 0x00, 0x00 };

  if (HAL_QSPI_Receive(hqspi, rx_satus_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return MX25_ERROR;

  if (MX25_WriteEnable(hqspi) != MX25_OK)
    return MX25_ERROR;

  /*   |                    STATS                        |                    CONFIG                       |
   *   |  7  |  6  | 5     |  4  |  3  |  2  |  1  |  0  |  7  |  6  | 5     |  4  |  3  |  2  |  1  |  0  |
   *   |     |     |       |     |     |     |     |     | DC1 | DC0 | 4BYTE | PBE |  TB | REV |ODS1 |ODS0 |
   *   |     |     |       |     |     |     |     |     |     |     |       |     |     |     |     |     |
   */
  s_command.Instruction = MX25_WRITE_CFG_SR_CMD__WRSR;
  uint8_t tx_satus_config[] = { 0x00, 0x20 };
      
  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return MX25_ERROR;

  if (HAL_QSPI_Transmit(hqspi, tx_satus_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return MX25_ERROR;
  
  return MX25_OK;
}

uint8_t MX25_WriteEnable(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef     s_command;
  QSPI_AutoPollingTypeDef s_config;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX25_WRITE_EN_CMD__WREN;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return MX25_ERROR;
  

  s_config.Match           = MX25_SR_WEL;
  s_config.Mask            = MX25_SR_WEL;
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  s_command.Instruction    = MX25_READ_STATUS_CMD__RDSR;
  s_command.DataMode       = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    return MX25_ERROR;

  return MX25_OK;
}

uint8_t MX25_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi, uint32_t Timeout)
{
  QSPI_CommandTypeDef     s_command;
  QSPI_AutoPollingTypeDef s_config;

  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = MX25_READ_STATUS_CMD__RDSR;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  s_config.Match           = 0;
  s_config.Mask            = MX25_SR_WIP;
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.StatusBytesSize = 1;
  s_config.Interval        = 0x10;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(hqspi, &s_command, &s_config, Timeout) != HAL_OK)
    return MX25_ERROR;

  return MX25_OK;
}

