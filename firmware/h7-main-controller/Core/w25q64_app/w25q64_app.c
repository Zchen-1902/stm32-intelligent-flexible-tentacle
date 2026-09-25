#include "w25q64_app.h"

#include "spi.h"
#include "stm32h7xx_hal.h"
#include <string.h>

#define W25Q64_CMD_JEDEC_ID         0x9FU
#define W25Q64_CMD_READ_STATUS1     0x05U
#define W25Q64_CMD_WRITE_ENABLE     0x06U
#define W25Q64_CMD_READ_DATA        0x03U
#define W25Q64_CMD_PAGE_PROGRAM     0x02U
#define W25Q64_CMD_SECTOR_ERASE_4K  0x20U

#define W25Q64_STATUS1_WIP          0x01U
#define W25Q64_STATUS1_WEL          0x02U
#define W25Q64_PAGE_SIZE            256U

#define W25Q64_APP_TIMEOUT_MS       1000U
#define W25Q64_APP_ERASE_TIMEOUT_MS 5000U
#define W25Q64_APP_TEST_ADDR        0x007FF000UL
#define W25Q64_APP_TEST_LEN         64U
#define W25Q64_APP_CONFIG_ADDR      0x007DF000UL /* 独立4K扇区保存映射配置，避免影响HOME/ACTUAL日志。 */
#define W25Q64_APP_CONFIG_MAGIC     0x4746434DUL /* "MCFG" */
#define W25Q64_APP_CONFIG_VERSION   2U

#define W25Q64_APP_LOG_BASE_ADDR    0x007E0000UL
#define W25Q64_APP_LOG_SECTOR_SIZE  4096UL
#define W25Q64_APP_LOG_SECTOR_COUNT 16UL
#define W25Q64_APP_LOG_SIZE         (W25Q64_APP_LOG_SECTOR_SIZE * W25Q64_APP_LOG_SECTOR_COUNT)
#define W25Q64_APP_LOG_SLOT_SIZE    64UL   /* 固定槽位，避免结构体大小变化或跨页导致日志扫描异常。 */
#define W25Q64_APP_LOG_RECORD_COUNT (W25Q64_APP_LOG_SIZE / W25Q64_APP_LOG_SLOT_SIZE)

#define W25Q64_APP_POS_MAGIC        0x54504F53UL  /* "TPOS" */
#define W25Q64_APP_POS_VERSION      3U
#define W25Q64_APP_TENTACLE_MAX     4U
#define W25Q64_APP_BOARD_MAX        4U

/* 每个触手独占64KB完整状态日志；旧0x7E0000日志只读，用于首次迁移。 */
#define W25Q64_APP_STATE_BASE_ADDR          0x00780000UL
#define W25Q64_APP_STATE_REGION_SIZE        0x00010000UL
#define W25Q64_APP_STATE_SLOT_SIZE          64UL
#define W25Q64_APP_STATE_RECORD_COUNT       (W25Q64_APP_STATE_REGION_SIZE / W25Q64_APP_STATE_SLOT_SIZE)
#define W25Q64_APP_STATE_SLOTS_PER_SECTOR   (W25Q64_APP_LOG_SECTOR_SIZE / W25Q64_APP_STATE_SLOT_SIZE)
#define W25Q64_APP_STATE_MAGIC              0x54535441UL /* "TSTA" */
#define W25Q64_APP_STATE_VERSION            1U
#define W25Q64_APP_STATE_VALID_HOME         0x01U
#define W25Q64_APP_STATE_VALID_ACTUAL       0x02U
#define W25Q64_APP_STATE_VALID_ALL          (W25Q64_APP_STATE_VALID_HOME | W25Q64_APP_STATE_VALID_ACTUAL)
#define W25Q64_APP_STATE_INDEX_NONE         0xFFFFFFFFUL

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  W25Q64_App_MotorConfig_t config;
  uint32_t tick_ms;
  uint32_t crc32;
} W25Q64_App_ConfigRecord_t;

/* 后台擦除任务。所有字段只在主循环中修改，不放入中断。 */
static W25Q64_App_EraseInfo_t s_w25_erase =
{
  W25Q64_ERASE_IDLE,
  W25Q64_APP_OK,
  0U,
  0U
};

typedef struct
{
  uint8_t initialized;  /* 已完成新日志扫描或旧数据装载。 */
  uint8_t valid;        /* RAM中存在完整HOME+ACTUAL。 */
  uint8_t persisted;    /* 最新状态已写入新日志。 */
  uint8_t reserved;
  uint32_t index;       /* 新日志中的最新槽位。 */
  W25Q64_App_TentacleState_t state;
} W25Q64_App_StateCache_t;

static W25Q64_App_StateCache_t s_w25_state_cache[W25Q64_APP_TENTACLE_MAX];

/* 编译期锁定Flash格式，防止结构体扩展后悄悄跨越64字节槽位。 */
typedef char W25Q64_App_StateSizeCheck[(sizeof(W25Q64_App_TentacleState_t) == 48U) ? 1 : -1];

/**
  * @brief  选中W25Q64片选。
  * @note   新板使用SPI4普通三线通信，PE4作为软件片选，低电平有效。
  */
static void W25Q64_App_Select(void)
{
  HAL_GPIO_WritePin(W25CS_GPIO_Port, W25CS_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  释放W25Q64片选。
  * @note   每条SPI命令结束后拉高CS，避免命令边界不清。
  */
static void W25Q64_App_Deselect(void)
{
  HAL_GPIO_WritePin(W25CS_GPIO_Port, W25CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  发送单字节无地址命令。
  * @param  instruction W25Q64命令码。
  * @return 0成功，非0失败。
  */
static uint8_t W25Q64_App_CommandOnly(uint8_t instruction)
{
  uint8_t status = W25Q64_APP_OK;

  W25Q64_App_Select();
  if (HAL_SPI_Transmit(&hspi4, &instruction, 1U, W25Q64_APP_TIMEOUT_MS) != HAL_OK)
  {
    status = W25Q64_APP_ERR_QSPI;
  }
  W25Q64_App_Deselect();

  return status;
}

/**
  * @brief  生成命令+24位地址的4字节SPI命令头。
  * @param  cmd 输出命令缓冲区，长度必须为4。
  * @param  instruction W25Q64命令码。
  * @param  address 24bit flash地址。
  */
static void W25Q64_App_BuildAddressCommand(uint8_t cmd[4], uint8_t instruction, uint32_t address)
{
  cmd[0] = instruction;
  cmd[1] = (uint8_t)((address >> 16) & 0xFFU);
  cmd[2] = (uint8_t)((address >> 8) & 0xFFU);
  cmd[3] = (uint8_t)(address & 0xFFU);
}

/**
  * @brief  读取状态寄存器1。
  * @param  status1 状态寄存器1输出。
  * @return 0成功，非0失败。
  */
static uint8_t W25Q64_App_ReadStatus1(uint8_t *status1)
{
  uint8_t cmd = W25Q64_CMD_READ_STATUS1;
  uint8_t status = W25Q64_APP_OK;

  if (status1 == 0)
  {
    return W25Q64_APP_ERR_PARAM;
  }

  W25Q64_App_Select();
  if (HAL_SPI_Transmit(&hspi4, &cmd, 1U, W25Q64_APP_TIMEOUT_MS) != HAL_OK)
  {
    status = W25Q64_APP_ERR_QSPI;
  }
  else if (HAL_SPI_Receive(&hspi4, status1, 1U, W25Q64_APP_TIMEOUT_MS) != HAL_OK)
  {
    status = W25Q64_APP_ERR_QSPI;
  }
  W25Q64_App_Deselect();

  return status;
}

/**
  * @brief  等待W25Q64内部擦写完成。
  * @param  timeout_ms 超时时间，单位ms。
  * @param  last_status1 返回最后一次读到的状态寄存器1，可为NULL。
  * @return 0表示空闲，非0表示超时或QSPI错误。
  */
static uint8_t W25Q64_App_WaitBusy(uint32_t timeout_ms, uint8_t *last_status1)
{
  uint32_t start_ms;
  uint8_t status;
  uint8_t status1;

  start_ms = HAL_GetTick();
  do
  {
    status = W25Q64_App_ReadStatus1(&status1);
    if (status != W25Q64_APP_OK)
    {
      return status;
    }

    if (last_status1 != 0)
    {
      *last_status1 = status1;
    }

    if ((status1 & W25Q64_STATUS1_WIP) == 0U)
    {
      return W25Q64_APP_OK;
    }
  } while ((HAL_GetTick() - start_ms) < timeout_ms);

  return W25Q64_APP_ERR_WAIT_ERASE;
}

/**
  * @brief  发送写使能命令，并确认WEL置位。
  * @param  status1 返回写使能后的状态寄存器1，可为NULL。
  * @return 0成功，非0失败。
  */
static uint8_t W25Q64_App_WriteEnable(uint8_t *status1)
{
  uint8_t sr1;

  if (W25Q64_App_CommandOnly(W25Q64_CMD_WRITE_ENABLE) != W25Q64_APP_OK)
  {
    return W25Q64_APP_ERR_QSPI;
  }

  if (W25Q64_App_ReadStatus1(&sr1) != W25Q64_APP_OK)
  {
    return W25Q64_APP_ERR_QSPI;
  }

  if (status1 != 0)
  {
    *status1 = sr1;
  }

  return ((sr1 & W25Q64_STATUS1_WEL) != 0U) ? W25Q64_APP_OK : W25Q64_APP_ERR_WEL;
}

/**
  * @brief  擦除一个4KB扇区。
  * @param  address 24bit flash地址。
  * @return 0成功，非0失败。
  */
static uint8_t W25Q64_App_EraseSector4K(uint32_t address)
{
  uint8_t cmd[4];
  uint8_t status = W25Q64_APP_OK;

  W25Q64_App_BuildAddressCommand(cmd, W25Q64_CMD_SECTOR_ERASE_4K, address);

  W25Q64_App_Select();
  if (HAL_SPI_Transmit(&hspi4, cmd, (uint16_t)sizeof(cmd), W25Q64_APP_TIMEOUT_MS) != HAL_OK)
  {
    status = W25Q64_APP_ERR_ERASE;
  }
  W25Q64_App_Deselect();

  return status;
}

/**
  * @brief  请求非阻塞擦除一个4KB扇区。
  * @param  sector_address 4KB对齐的扇区首地址。
  * @return 0表示已接受请求；BUSY表示上一次任务尚未取走结果。
  * @note   本函数只登记任务，不访问SPI，也不等待Flash完成。
  */
uint8_t W25Q64_App_RequestEraseSector(uint32_t sector_address)
{
  if ((sector_address % W25Q64_APP_LOG_SECTOR_SIZE) != 0U)
  {
    return W25Q64_APP_ERR_PARAM;
  }

  if (s_w25_erase.state != W25Q64_ERASE_IDLE)
  {
    return W25Q64_APP_ERR_BUSY;
  }

  s_w25_erase.address = sector_address;
  s_w25_erase.result = W25Q64_APP_OK;
  s_w25_erase.start_ms = 0U;
  s_w25_erase.state = W25Q64_ERASE_START;
  return W25Q64_APP_OK;
}

/**
  * @brief  推进一次W25后台擦除任务。
  * @note   START只发送一次擦除命令；WAIT每次只读取一次WIP忙标志，绝不循环等待。
  */
void W25Q64_App_Task(void)
{
  uint8_t status1;
  uint8_t status;

  switch (s_w25_erase.state)
  {
    case W25Q64_ERASE_IDLE:
    case W25Q64_ERASE_DONE:
    case W25Q64_ERASE_ERROR:
      return;

    case W25Q64_ERASE_START:
      status = W25Q64_App_WriteEnable(NULL);
      if (status == W25Q64_APP_OK)
      {
        status = W25Q64_App_EraseSector4K(s_w25_erase.address);
      }

      if (status != W25Q64_APP_OK)
      {
        s_w25_erase.result = status;
        s_w25_erase.state = W25Q64_ERASE_ERROR;
        return;
      }

      s_w25_erase.start_ms = HAL_GetTick();
      s_w25_erase.state = W25Q64_ERASE_WAIT;
      return;

    case W25Q64_ERASE_WAIT:
      status = W25Q64_App_ReadStatus1(&status1);
      if (status != W25Q64_APP_OK)
      {
        s_w25_erase.result = status;
        s_w25_erase.state = W25Q64_ERASE_ERROR;
      }
      else if ((status1 & W25Q64_STATUS1_WIP) == 0U)
      {
        s_w25_erase.result = W25Q64_APP_OK;
        s_w25_erase.state = W25Q64_ERASE_DONE;
      }
      else if ((HAL_GetTick() - s_w25_erase.start_ms) >= W25Q64_APP_ERASE_TIMEOUT_MS)
      {
        s_w25_erase.result = W25Q64_APP_ERR_WAIT_ERASE;
        s_w25_erase.state = W25Q64_ERASE_ERROR;
      }
      return;

    default:
      s_w25_erase.result = W25Q64_APP_ERR_PARAM;
      s_w25_erase.state = W25Q64_ERASE_ERROR;
      return;
  }
}

const W25Q64_App_EraseInfo_t *W25Q64_App_GetEraseInfo(void)
{
  return &s_w25_erase;
}

uint8_t W25Q64_App_TakeEraseResult(uint8_t *result)
{
  if ((s_w25_erase.state != W25Q64_ERASE_DONE) &&
      (s_w25_erase.state != W25Q64_ERASE_ERROR))
  {
    return 0U;
  }

  if (result != NULL)
  {
    *result = s_w25_erase.result;
  }

  s_w25_erase.state = W25Q64_ERASE_IDLE;
  return 1U;
}

/**
  * @brief  页编程，长度不能跨页。
  * @param  address 24bit flash地址。
  * @param  data 写入数据。
  * @param  len 写入长度。
  * @return 0成功，非0失败。
  */
static uint8_t W25Q64_App_PageProgram(uint32_t address, uint8_t *data, uint32_t len)
{
  uint8_t cmd[4];
  uint8_t status = W25Q64_APP_OK;

  if ((data == 0) || (len == 0U) ||
      (len > W25Q64_PAGE_SIZE) ||
      (((address & (W25Q64_PAGE_SIZE - 1U)) + len) > W25Q64_PAGE_SIZE))
  {
    return W25Q64_APP_ERR_PARAM;
  }

  W25Q64_App_BuildAddressCommand(cmd, W25Q64_CMD_PAGE_PROGRAM, address);

  W25Q64_App_Select();
  if (HAL_SPI_Transmit(&hspi4, cmd, (uint16_t)sizeof(cmd), W25Q64_APP_TIMEOUT_MS) != HAL_OK)
  {
    status = W25Q64_APP_ERR_PROGRAM;
  }
  else if (HAL_SPI_Transmit(&hspi4, data, (uint16_t)len, W25Q64_APP_TIMEOUT_MS) != HAL_OK)
  {
    status = W25Q64_APP_ERR_PROGRAM;
  }
  W25Q64_App_Deselect();

  return status;
}

/**
  * @brief  普通03h命令读取flash数据。
  * @param  address 24bit flash地址。
  * @param  data 读出缓冲区。
  * @param  len 读取长度。
  * @return 0成功，非0失败。
  */
static uint8_t W25Q64_App_ReadData(uint32_t address, uint8_t *data, uint32_t len)
{
  uint8_t cmd[4];
  uint8_t status = W25Q64_APP_OK;

  if ((data == 0) || (len == 0U) || (len > 0xFFFFU))
  {
    return W25Q64_APP_ERR_PARAM;
  }

  W25Q64_App_BuildAddressCommand(cmd, W25Q64_CMD_READ_DATA, address);

  W25Q64_App_Select();
  if (HAL_SPI_Transmit(&hspi4, cmd, (uint16_t)sizeof(cmd), W25Q64_APP_TIMEOUT_MS) != HAL_OK)
  {
    status = W25Q64_APP_ERR_READ;
  }
  else if (HAL_SPI_Receive(&hspi4, data, (uint16_t)len, W25Q64_APP_TIMEOUT_MS) != HAL_OK)
  {
    status = W25Q64_APP_ERR_READ;
  }
  W25Q64_App_Deselect();

  return status;
}

/**
  * @brief  计算标准CRC32，用于判断掉电记录是否完整。
  * @param  data 数据指针。
  * @param  len 数据长度。
  * @return CRC32结果。
  */
static uint32_t W25Q64_App_CalcCrc32(const uint8_t *data, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t i;
  uint8_t bit;

  for (i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      crc = ((crc & 1UL) != 0UL) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
  }

  return ~crc;
}

/**
  * @brief  判断记录类型是否允许写入。
  * @param  record_type 记录类型。
  * @return 1表示允许，0表示非法。
  */
static uint8_t W25Q64_App_IsValidRecordType(uint16_t record_type)
{
  return ((record_type == W25Q64_APP_RECORD_TYPE_HOME) ||
          (record_type == W25Q64_APP_RECORD_TYPE_ACTUAL)) ? 1U : 0U;
}

/**
  * @brief  判断位置记录所属触手/板号是否合法。
  * @param  tentacle_id 触手编号，从1开始。
  * @param  board_id G4板号。
  * @return 1表示合法，0表示非法。
  */
static uint8_t W25Q64_App_IsValidRecordOwner(uint8_t tentacle_id, uint8_t board_id)
{
  return ((tentacle_id >= 1U) &&
          (tentacle_id <= W25Q64_APP_TENTACLE_MAX) &&
          (board_id < W25Q64_APP_BOARD_MAX)) ? 1U : 0U;
}

/**
  * @brief  判断一条记录区域是否仍为擦除态。
  * @param  record 记录指针。
  * @return 1表示全FF，0表示非空。
  */
static uint8_t W25Q64_App_IsErasedRecord(const W25Q64_App_PositionRecord_t *record)
{
  const uint8_t *p = (const uint8_t *)record;
  uint32_t i;

  for (i = 0U; i < sizeof(*record); i++)
  {
    if (p[i] != 0xFFU)
    {
      return 0U;
    }
  }

  return 1U;
}

/**
  * @brief  校验一条位置记录是否有效。
  * @param  record 记录指针。
  * @return 0有效，非0无效。
  */
static uint8_t W25Q64_App_CheckPositionRecord(const W25Q64_App_PositionRecord_t *record)
{
  uint32_t crc;

  if ((record == 0) ||
      (record->magic != W25Q64_APP_POS_MAGIC) ||
      (record->version != W25Q64_APP_POS_VERSION) ||
      (record->size != sizeof(W25Q64_App_PositionRecord_t)) ||
      (W25Q64_App_IsValidRecordType(record->record_type) == 0U) ||
      (W25Q64_App_IsValidRecordOwner(record->tentacle_id, record->board_id) == 0U))
  {
    return W25Q64_APP_ERR_NO_RECORD;
  }

  crc = W25Q64_App_CalcCrc32((const uint8_t *)record,
                             sizeof(W25Q64_App_PositionRecord_t) - sizeof(record->crc32));

  return (crc == record->crc32) ? W25Q64_APP_OK : W25Q64_APP_ERR_CRC;
}

/**
  * @brief  获取指定触手完整状态日志的首地址。
  */
static uint32_t W25Q64_App_GetStateBase(uint8_t tentacle_id)
{
  return W25Q64_APP_STATE_BASE_ADDR +
         ((uint32_t)(tentacle_id - 1U) * W25Q64_APP_STATE_REGION_SIZE);
}

/**
  * @brief  判断状态记录是否仍为擦除态。
  */
static uint8_t W25Q64_App_IsErasedState(const W25Q64_App_TentacleState_t *state)
{
  const uint8_t *p = (const uint8_t *)state;
  uint32_t i;

  for (i = 0U; i < sizeof(*state); i++)
  {
    if (p[i] != 0xFFU)
    {
      return 0U;
    }
  }
  return 1U;
}

/**
  * @brief  校验一条完整触手状态记录。
  */
static uint8_t W25Q64_App_CheckTentacleState(const W25Q64_App_TentacleState_t *state)
{
  uint32_t crc;

  if ((state == 0) ||
      (state->magic != W25Q64_APP_STATE_MAGIC) ||
      (state->version != W25Q64_APP_STATE_VERSION) ||
      (state->size != sizeof(W25Q64_App_TentacleState_t)) ||
      (state->valid_flags != W25Q64_APP_STATE_VALID_ALL) ||
      (W25Q64_App_IsValidRecordOwner(state->tentacle_id, state->board_id) == 0U))
  {
    return W25Q64_APP_ERR_NO_RECORD;
  }

  crc = W25Q64_App_CalcCrc32((const uint8_t *)state,
                             sizeof(*state) - sizeof(state->crc32));
  return (crc == state->crc32) ? W25Q64_APP_OK : W25Q64_APP_ERR_CRC;
}

/**
  * @brief  判断循环序号a是否比b更新，兼容uint32自然回绕。
  */
static uint8_t W25Q64_App_IsSeqNewer(uint32_t a, uint32_t b)
{
  return ((int32_t)(a - b) > 0) ? 1U : 0U;
}

/**
  * @brief  校验一条映射配置记录是否有效。
  * @param  record 配置记录指针。
  * @return 0有效，非0无效。
  */
static uint8_t W25Q64_App_CheckConfigRecord(const W25Q64_App_ConfigRecord_t *record)
{
  uint32_t crc;

  if ((record == 0) ||
      (record->magic != W25Q64_APP_CONFIG_MAGIC) ||
      (record->version != W25Q64_APP_CONFIG_VERSION) ||
      (record->size != sizeof(W25Q64_App_ConfigRecord_t)))
  {
    return W25Q64_APP_ERR_NO_RECORD;
  }

  crc = W25Q64_App_CalcCrc32((const uint8_t *)record,
                             sizeof(W25Q64_App_ConfigRecord_t) - sizeof(record->crc32));

  return (crc == record->crc32) ? W25Q64_APP_OK : W25Q64_APP_ERR_CRC;
}

/**
  * @brief  扫描旧共享日志，查找指定触手和类型的最新记录。
  * @note   仅用于首次升级迁移，新格式运行期间不再调用。
  */
static uint8_t W25Q64_App_FindLatestPositionRecord(uint8_t tentacle_id,
                                                   uint16_t record_type,
                                                   W25Q64_App_PositionRecord_t *record)
{
  W25Q64_App_PositionRecord_t temp;
  W25Q64_App_PositionRecord_t latest;
  uint32_t index;
  uint8_t found = 0U;
  uint8_t status;

  memset(&latest, 0, sizeof(latest));
  for (index = 0U; index < W25Q64_APP_LOG_RECORD_COUNT; index++)
  {
    status = W25Q64_App_ReadData(W25Q64_APP_LOG_BASE_ADDR +
                                 (index * W25Q64_APP_LOG_SLOT_SIZE),
                                 (uint8_t *)&temp,
                                 sizeof(temp));
    if (status != W25Q64_APP_OK)
    {
      return status;
    }
    if ((W25Q64_App_IsErasedRecord(&temp) != 0U) ||
        (W25Q64_App_CheckPositionRecord(&temp) != W25Q64_APP_OK) ||
        (temp.tentacle_id != tentacle_id) ||
        (temp.record_type != record_type))
    {
      continue;
    }
    if ((found == 0U) || (W25Q64_App_IsSeqNewer(temp.seq, latest.seq) != 0U))
    {
      latest = temp;
      found = 1U;
    }
  }

  if (found == 0U)
  {
    return W25Q64_APP_ERR_NO_RECORD;
  }
  *record = latest;
  return W25Q64_APP_OK;
}

/**
  * @brief  首次访问某触手时扫描一次新日志，并缓存最新记录和写指针。
  */
static uint8_t W25Q64_App_InitStateCache(uint8_t tentacle_id)
{
  W25Q64_App_StateCache_t *cache;
  W25Q64_App_TentacleState_t temp;
  W25Q64_App_TentacleState_t latest;
  W25Q64_App_PositionRecord_t legacy_home;
  W25Q64_App_PositionRecord_t legacy_actual;
  uint32_t base;
  uint32_t index;
  uint32_t latest_index = 0U;
  uint8_t owner;
  uint8_t found = 0U;
  uint8_t home_status;
  uint8_t actual_status;
  uint8_t status;

  if ((tentacle_id < 1U) || (tentacle_id > W25Q64_APP_TENTACLE_MAX))
  {
    return W25Q64_APP_ERR_PARAM;
  }

  owner = (uint8_t)(tentacle_id - 1U);
  cache = &s_w25_state_cache[owner];
  if (cache->initialized != 0U)
  {
    return W25Q64_APP_OK;
  }

  memset(&latest, 0, sizeof(latest));
  base = W25Q64_App_GetStateBase(tentacle_id);
  for (index = 0U; index < W25Q64_APP_STATE_RECORD_COUNT; index++)
  {
    status = W25Q64_App_ReadData(base + (index * W25Q64_APP_STATE_SLOT_SIZE),
                                 (uint8_t *)&temp,
                                 sizeof(temp));
    if (status != W25Q64_APP_OK)
    {
      return status;
    }
    if ((W25Q64_App_IsErasedState(&temp) != 0U) ||
        (W25Q64_App_CheckTentacleState(&temp) != W25Q64_APP_OK) ||
        (temp.tentacle_id != tentacle_id))
    {
      continue;
    }
    if ((found == 0U) || (W25Q64_App_IsSeqNewer(temp.seq, latest.seq) != 0U))
    {
      latest = temp;
      latest_index = index;
      found = 1U;
    }
  }

  memset(cache, 0, sizeof(*cache));
  cache->index = W25Q64_APP_STATE_INDEX_NONE;
  if (found != 0U)
  {
    cache->state = latest;
    cache->index = latest_index;
    cache->valid = 1U;
    cache->persisted = 1U;
  }
  else
  {
    home_status = W25Q64_App_FindLatestPositionRecord(tentacle_id,
                                                       W25Q64_APP_RECORD_TYPE_HOME,
                                                       &legacy_home);
    actual_status = W25Q64_App_FindLatestPositionRecord(tentacle_id,
                                                         W25Q64_APP_RECORD_TYPE_ACTUAL,
                                                         &legacy_actual);
    if (((home_status != W25Q64_APP_OK) && (home_status != W25Q64_APP_ERR_NO_RECORD)) ||
        ((actual_status != W25Q64_APP_OK) && (actual_status != W25Q64_APP_ERR_NO_RECORD)))
    {
      return (home_status != W25Q64_APP_OK) ? home_status : actual_status;
    }
    if ((home_status == W25Q64_APP_OK) && (actual_status == W25Q64_APP_OK))
    {
      cache->state.magic = W25Q64_APP_STATE_MAGIC;
      cache->state.version = W25Q64_APP_STATE_VERSION;
      cache->state.size = sizeof(cache->state);
      cache->state.seq = (W25Q64_App_IsSeqNewer(legacy_actual.seq, legacy_home.seq) != 0U) ?
                         legacy_actual.seq : legacy_home.seq;
      cache->state.tentacle_id = tentacle_id;
      cache->state.board_id = legacy_actual.board_id;
      cache->state.valid_flags = W25Q64_APP_STATE_VALID_ALL;
      memcpy(cache->state.home_q, legacy_home.position_q, sizeof(cache->state.home_q));
      memcpy(cache->state.actual_q, legacy_actual.position_q, sizeof(cache->state.actual_q));
      cache->state.tick_ms = legacy_actual.tick_ms;
      cache->state.crc32 = W25Q64_App_CalcCrc32(
        (const uint8_t *)&cache->state,
        sizeof(cache->state) - sizeof(cache->state.crc32));
      cache->valid = 1U;
    }
  }

  cache->initialized = 1U;
  return W25Q64_APP_OK;
}

uint8_t W25Q64_App_ReadJedecId(uint8_t jedec_id[3])
{
  uint8_t cmd = W25Q64_CMD_JEDEC_ID;
  uint8_t status = W25Q64_APP_OK;

  if (jedec_id == 0)
  {
    return W25Q64_APP_ERR_PARAM;
  }

  W25Q64_App_Select();
  if (HAL_SPI_Transmit(&hspi4, &cmd, 1U, W25Q64_APP_TIMEOUT_MS) != HAL_OK)
  {
    status = W25Q64_APP_ERR_QSPI;
  }
  else if (HAL_SPI_Receive(&hspi4, jedec_id, 3U, W25Q64_APP_TIMEOUT_MS) != HAL_OK)
  {
    status = W25Q64_APP_ERR_QSPI;
  }
  W25Q64_App_Deselect();

  return status;
}

uint8_t W25Q64_App_RunBasicTest(W25Q64_App_TestResult_t *result)
{
  uint8_t tx_data[W25Q64_APP_TEST_LEN];
  uint8_t rx_data[W25Q64_APP_TEST_LEN];
  uint32_t i;
  uint8_t status;

  if (result == 0)
  {
    return W25Q64_APP_ERR_PARAM;
  }

  memset(result, 0, sizeof(*result));
  result->test_addr = W25Q64_APP_TEST_ADDR;
  result->test_len = W25Q64_APP_TEST_LEN;
  result->mismatch_index = 0xFFFFFFFFUL;

  status = W25Q64_App_ReadJedecId(result->jedec_id);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  if ((result->jedec_id[0] != 0xEFU) ||
      (result->jedec_id[1] != 0x40U) ||
      (result->jedec_id[2] != 0x17U))
  {
    return W25Q64_APP_ERR_JEDEC_ID;
  }

  for (i = 0U; i < W25Q64_APP_TEST_LEN; i++)
  {
    tx_data[i] = (uint8_t)(0xA5U ^ (uint8_t)(i * 13U));
    rx_data[i] = 0U;
  }

  status = W25Q64_App_WriteEnable(&result->status1);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  status = W25Q64_App_EraseSector4K(W25Q64_APP_TEST_ADDR);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  status = W25Q64_App_WaitBusy(W25Q64_APP_ERASE_TIMEOUT_MS, &result->status1);
  if (status != W25Q64_APP_OK)
  {
    return W25Q64_APP_ERR_WAIT_ERASE;
  }

  status = W25Q64_App_WriteEnable(&result->status1);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  status = W25Q64_App_PageProgram(W25Q64_APP_TEST_ADDR, tx_data, W25Q64_APP_TEST_LEN);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  status = W25Q64_App_WaitBusy(W25Q64_APP_TIMEOUT_MS, &result->status1);
  if (status != W25Q64_APP_OK)
  {
    return W25Q64_APP_ERR_WAIT_PROG;
  }

  status = W25Q64_App_ReadData(W25Q64_APP_TEST_ADDR, rx_data, W25Q64_APP_TEST_LEN);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  for (i = 0U; i < W25Q64_APP_TEST_LEN; i++)
  {
    if (rx_data[i] != tx_data[i])
    {
      result->mismatch_index = i;
      result->expected = tx_data[i];
      result->actual = rx_data[i];
      return W25Q64_APP_ERR_VERIFY;
    }
  }

  result->expected = tx_data[0];
  result->actual = rx_data[0];
  return W25Q64_APP_OK;
}

uint8_t W25Q64_App_LoadLatestPositionRecord(uint16_t record_type,
                                            W25Q64_App_PositionRecord_t *record)
{
  return W25Q64_App_LoadLatestPositionRecordForTentacle(1U, record_type, record);
}

uint8_t W25Q64_App_LoadLatestPositionRecordForTentacle(uint8_t tentacle_id,
                                                       uint16_t record_type,
                                                       W25Q64_App_PositionRecord_t *record)
{
  W25Q64_App_TentacleState_t state;
  const int32_t *position_q;
  uint8_t status;

  if ((record == 0) || (W25Q64_App_IsValidRecordType(record_type) == 0U))
  {
    return W25Q64_APP_ERR_PARAM;
  }

  status = W25Q64_App_LoadTentacleState(tentacle_id, &state);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  position_q = (record_type == W25Q64_APP_RECORD_TYPE_HOME) ? state.home_q : state.actual_q;
  memset(record, 0, sizeof(*record));
  record->magic = W25Q64_APP_POS_MAGIC;
  record->version = W25Q64_APP_POS_VERSION;
  record->size = sizeof(*record);
  record->seq = state.seq;
  record->record_type = record_type;
  record->tentacle_id = state.tentacle_id;
  record->board_id = state.board_id;
  memcpy(record->position_q, position_q, sizeof(record->position_q));
  record->tick_ms = state.tick_ms;
  record->crc32 = W25Q64_App_CalcCrc32((const uint8_t *)record,
                                       sizeof(*record) - sizeof(record->crc32));
  return W25Q64_APP_OK;
}

uint8_t W25Q64_App_LoadTentacleState(uint8_t tentacle_id,
                                     W25Q64_App_TentacleState_t *state)
{
  W25Q64_App_StateCache_t *cache;
  uint8_t status;

  if ((state == 0) ||
      (tentacle_id < 1U) ||
      (tentacle_id > W25Q64_APP_TENTACLE_MAX))
  {
    return W25Q64_APP_ERR_PARAM;
  }

  status = W25Q64_App_InitStateCache(tentacle_id);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  cache = &s_w25_state_cache[tentacle_id - 1U];
  if (cache->valid == 0U)
  {
    return W25Q64_APP_ERR_NO_RECORD;
  }

  *state = cache->state;
  return W25Q64_APP_OK;
}

uint8_t W25Q64_App_LoadMotorConfig(W25Q64_App_MotorConfig_t *config)
{
  W25Q64_App_ConfigRecord_t record;
  uint8_t status;

  if (config == 0)
  {
    return W25Q64_APP_ERR_PARAM;
  }

  status = W25Q64_App_ReadData(W25Q64_APP_CONFIG_ADDR, (uint8_t *)&record, sizeof(record));
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  status = W25Q64_App_CheckConfigRecord(&record);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  *config = record.config;
  return W25Q64_APP_OK;
}

uint8_t W25Q64_App_SaveMotorConfig(const W25Q64_App_MotorConfig_t *config)
{
  W25Q64_App_ConfigRecord_t record;
  W25Q64_App_ConfigRecord_t check;
  uint8_t status;

  if (config == 0)
  {
    return W25Q64_APP_ERR_PARAM;
  }

  memset(&record, 0, sizeof(record));
  record.magic = W25Q64_APP_CONFIG_MAGIC;
  record.version = W25Q64_APP_CONFIG_VERSION;
  record.size = sizeof(record);
  record.config = *config;
  record.tick_ms = HAL_GetTick();
  record.crc32 = W25Q64_App_CalcCrc32((const uint8_t *)&record,
                                      sizeof(record) - sizeof(record.crc32));

  status = W25Q64_App_WriteEnable(0);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  status = W25Q64_App_EraseSector4K(W25Q64_APP_CONFIG_ADDR);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  status = W25Q64_App_WaitBusy(W25Q64_APP_ERASE_TIMEOUT_MS, 0);
  if (status != W25Q64_APP_OK)
  {
    return W25Q64_APP_ERR_WAIT_ERASE;
  }

  status = W25Q64_App_WriteEnable(0);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  status = W25Q64_App_PageProgram(W25Q64_APP_CONFIG_ADDR, (uint8_t *)&record, sizeof(record));
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  status = W25Q64_App_WaitBusy(W25Q64_APP_TIMEOUT_MS, 0);
  if (status != W25Q64_APP_OK)
  {
    return W25Q64_APP_ERR_WAIT_PROG;
  }

  status = W25Q64_App_ReadData(W25Q64_APP_CONFIG_ADDR, (uint8_t *)&check, sizeof(check));
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  if ((W25Q64_App_CheckConfigRecord(&check) != W25Q64_APP_OK) ||
      (memcmp(&check, &record, sizeof(record)) != 0))
  {
    return W25Q64_APP_ERR_VERIFY;
  }

  return W25Q64_APP_OK;
}

uint8_t W25Q64_App_SaveTentacleState(uint8_t tentacle_id,
                                     uint8_t board_id,
                                     const int32_t home_q[3],
                                     const int32_t actual_q[3],
                                     W25Q64_App_TentacleState_t *saved_state)
{
  W25Q64_App_StateCache_t *cache;
  W25Q64_App_TentacleState_t next;
  W25Q64_App_TentacleState_t check;
  const W25Q64_App_EraseInfo_t *erase_info;
  uint32_t base;
  uint32_t next_index;
  uint32_t next_addr;
  uint32_t next_sector;
  uint32_t latest_sector;
  uint8_t erase_result;
  uint8_t target_occupied;
  uint8_t status;

  if ((home_q == 0) || (actual_q == 0) ||
      (W25Q64_App_IsValidRecordOwner(tentacle_id, board_id) == 0U))
  {
    return W25Q64_APP_ERR_PARAM;
  }

  /* 擦除由主循环后台推进；完成前不读取Flash，也不阻塞等待。 */
  erase_info = W25Q64_App_GetEraseInfo();
  if ((erase_info->state == W25Q64_ERASE_START) ||
      (erase_info->state == W25Q64_ERASE_WAIT))
  {
    return W25Q64_APP_ERR_BUSY;
  }
  if ((erase_info->state == W25Q64_ERASE_DONE) ||
      (erase_info->state == W25Q64_ERASE_ERROR))
  {
    if (W25Q64_App_TakeEraseResult(&erase_result) == 0U)
    {
      return W25Q64_APP_ERR_BUSY;
    }
    if (erase_result != W25Q64_APP_OK)
    {
      return erase_result;
    }
  }

  status = W25Q64_App_InitStateCache(tentacle_id);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  cache = &s_w25_state_cache[tentacle_id - 1U];
  base = W25Q64_App_GetStateBase(tentacle_id);
  next_index = (cache->persisted != 0U) ?
               ((cache->index + 1U) % W25Q64_APP_STATE_RECORD_COUNT) : 0U;
  next_addr = base + (next_index * W25Q64_APP_STATE_SLOT_SIZE);

  status = W25Q64_App_ReadData(next_addr, (uint8_t *)&check, sizeof(check));
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  target_occupied = (W25Q64_App_IsErasedState(&check) == 0U) ? 1U : 0U;
  if (target_occupied != 0U)
  {
    next_sector = next_index / W25Q64_APP_STATE_SLOTS_PER_SECTOR;
    if (cache->persisted != 0U)
    {
      latest_sector = cache->index / W25Q64_APP_STATE_SLOTS_PER_SECTOR;
      if (next_sector == latest_sector)
      {
        /* 异常半写槽位与最新记录同扇区时，跳到下一扇区，绝不擦最新记录。 */
        next_sector = (latest_sector + 1U) %
                      (W25Q64_APP_STATE_RECORD_COUNT / W25Q64_APP_STATE_SLOTS_PER_SECTOR);
        next_index = next_sector * W25Q64_APP_STATE_SLOTS_PER_SECTOR;
        next_addr = base + (next_index * W25Q64_APP_STATE_SLOT_SIZE);
        status = W25Q64_App_ReadData(next_addr, (uint8_t *)&check, sizeof(check));
        if (status != W25Q64_APP_OK)
        {
          return status;
        }
        target_occupied = (W25Q64_App_IsErasedState(&check) == 0U) ? 1U : 0U;
      }
    }

    if (target_occupied != 0U)
    {
      next_sector = next_addr - (next_addr % W25Q64_APP_LOG_SECTOR_SIZE);
      status = W25Q64_App_RequestEraseSector(next_sector);
      return (status == W25Q64_APP_OK) ? W25Q64_APP_ERR_BUSY : status;
    }
  }

  memset(&next, 0, sizeof(next));
  next.magic = W25Q64_APP_STATE_MAGIC;
  next.version = W25Q64_APP_STATE_VERSION;
  next.size = sizeof(next);
  next.seq = (cache->valid != 0U) ? (cache->state.seq + 1U) : 1U;
  next.tentacle_id = tentacle_id;
  next.board_id = board_id;
  next.valid_flags = W25Q64_APP_STATE_VALID_ALL;
  memcpy(next.home_q, home_q, sizeof(next.home_q));
  memcpy(next.actual_q, actual_q, sizeof(next.actual_q));
  next.tick_ms = HAL_GetTick();
  next.crc32 = W25Q64_App_CalcCrc32((const uint8_t *)&next,
                                    sizeof(next) - sizeof(next.crc32));

  status = W25Q64_App_WriteEnable(0);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }
  status = W25Q64_App_PageProgram(next_addr, (uint8_t *)&next, sizeof(next));
  if (status != W25Q64_APP_OK)
  {
    return status;
  }
  status = W25Q64_App_WaitBusy(W25Q64_APP_TIMEOUT_MS, 0);
  if (status != W25Q64_APP_OK)
  {
    return W25Q64_APP_ERR_WAIT_PROG;
  }

  status = W25Q64_App_ReadData(next_addr, (uint8_t *)&check, sizeof(check));
  if (status != W25Q64_APP_OK)
  {
    return status;
  }
  if ((W25Q64_App_CheckTentacleState(&check) != W25Q64_APP_OK) ||
      (memcmp(&check, &next, sizeof(next)) != 0))
  {
    return W25Q64_APP_ERR_VERIFY;
  }

  cache->state = check;
  cache->index = next_index;
  cache->valid = 1U;
  cache->persisted = 1U;
  if (saved_state != 0)
  {
    *saved_state = check;
  }
  return W25Q64_APP_OK;
}

uint8_t W25Q64_App_SavePositionRecord(uint16_t record_type,
                                      const int32_t position_q[3],
                                      W25Q64_App_PositionRecord_t *saved_record)
{
  return W25Q64_App_SavePositionRecordForTentacle(1U,
                                                  0U,
                                                  record_type,
                                                  position_q,
                                                  saved_record);
}

uint8_t W25Q64_App_SavePositionRecordForTentacle(uint8_t tentacle_id,
                                                 uint8_t board_id,
                                                 uint16_t record_type,
                                                 const int32_t position_q[3],
                                                 W25Q64_App_PositionRecord_t *saved_record)
{
  W25Q64_App_TentacleState_t state;
  W25Q64_App_TentacleState_t saved_state;
  uint8_t status;

  if ((position_q == 0) ||
      (W25Q64_App_IsValidRecordType(record_type) == 0U) ||
      (W25Q64_App_IsValidRecordOwner(tentacle_id, board_id) == 0U))
  {
    return W25Q64_APP_ERR_PARAM;
  }

  status = W25Q64_App_LoadTentacleState(tentacle_id, &state);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  if (record_type == W25Q64_APP_RECORD_TYPE_HOME)
  {
    memcpy(state.home_q, position_q, sizeof(state.home_q));
  }
  else
  {
    memcpy(state.actual_q, position_q, sizeof(state.actual_q));
  }

  status = W25Q64_App_SaveTentacleState(tentacle_id,
                                        board_id,
                                        state.home_q,
                                        state.actual_q,
                                        &saved_state);
  if (status != W25Q64_APP_OK)
  {
    return status;
  }

  if (saved_record != 0)
  {
    return W25Q64_App_LoadLatestPositionRecordForTentacle(tentacle_id,
                                                           record_type,
                                                           saved_record);
  }
  return W25Q64_APP_OK;
}
