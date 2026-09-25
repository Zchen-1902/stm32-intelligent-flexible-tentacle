#ifndef W25Q64_APP_H
#define W25Q64_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define W25Q64_APP_OK              0U
#define W25Q64_APP_ERR_PARAM       1U
#define W25Q64_APP_ERR_JEDEC_ID    2U
#define W25Q64_APP_ERR_WEL         3U
#define W25Q64_APP_ERR_ERASE       4U
#define W25Q64_APP_ERR_WAIT_ERASE  5U
#define W25Q64_APP_ERR_PROGRAM     6U
#define W25Q64_APP_ERR_WAIT_PROG   7U
#define W25Q64_APP_ERR_READ        8U
#define W25Q64_APP_ERR_VERIFY      9U
#define W25Q64_APP_ERR_QSPI        10U
#define W25Q64_APP_ERR_NO_RECORD   11U
#define W25Q64_APP_ERR_CRC         12U
#define W25Q64_APP_ERR_BUSY        13U

/**
 * @brief W25后台擦除状态。
 * @note  擦除命令发出后，由主循环周期查询WIP，不阻塞AI和CAN。
 */
typedef enum
{
  W25Q64_ERASE_IDLE = 0, /* 空闲：当前没有擦除任务。 */
  W25Q64_ERASE_START,    /* 准备：下一次Task发送擦除命令。 */
  W25Q64_ERASE_WAIT,     /* 等待：W25内部正在执行4KB扇区擦除。 */
  W25Q64_ERASE_DONE,     /* 完成：扇区已经擦除，可继续写快照。 */
  W25Q64_ERASE_ERROR     /* 错误：擦除失败或等待超时。 */
} W25Q64_App_EraseState_t;

typedef struct
{
  W25Q64_App_EraseState_t state; /* 当前擦除状态。 */
  uint8_t result;                /* 最近结果：0成功，非0为错误码。 */
  uint32_t address;              /* 当前擦除的4KB扇区首地址。 */
  uint32_t start_ms;             /* 擦除命令发送时间，用于超时判断。 */
} W25Q64_App_EraseInfo_t;

#define W25Q64_APP_RECORD_TYPE_ANY     0U
#define W25Q64_APP_RECORD_TYPE_HOME    1U
#define W25Q64_APP_RECORD_TYPE_ACTUAL  2U

typedef struct
{
  int32_t bend_gain_q;          /* 最大弯曲幅度，单位0.01rad。 */
  int32_t stiff_gain_q;         /* 三绳共同预紧量，单位0.01rad。 */
  int32_t input_deadband;       /* 手中心死区，范围0~100。 */
  int32_t auto_dir_full;        /* 手偏移达到该值后方向响应拉满，范围1~100。 */
  int32_t motor_speed_base_q;   /* 基础速度，单位rad/s*100；radius<=auto_dir_full时使用。 */
  int32_t motor_speed_max_q;    /* 最大速度，单位rad/s*100；radius=100时使用。 */
} W25Q64_App_MotorConfig_t;

typedef struct
{
  uint8_t jedec_id[3];
  uint32_t test_addr;
  uint32_t test_len;
  uint32_t mismatch_index;
  uint8_t expected;
  uint8_t actual;
  uint8_t status1;
} W25Q64_App_TestResult_t;

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t seq;
  uint16_t record_type;     /* 记录类型：HOME为初始零点，ACTUAL为运行多圈位置。 */
  uint8_t tentacle_id;      /* 触手编号，从1开始。 */
  uint8_t board_id;         /* 对应G4板号，用于区分多触手记录。 */
  uint16_t reserved;
  int32_t position_q[3];    /* 三路位置，单位0.01rad。 */
  uint32_t tick_ms;
  uint32_t crc32;
} W25Q64_App_PositionRecord_t;

/**
 * @brief 单个触手的完整掉电恢复状态。
 * @note  HOME和ACTUAL必须在同一条记录中提交，避免断电后出现新旧坐标混用。
 */
typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t seq;
  uint8_t tentacle_id;      /* 触手编号，从1开始。 */
  uint8_t board_id;         /* 对应G4板号。 */
  uint8_t valid_flags;      /* bit0=HOME有效，bit1=ACTUAL有效。 */
  uint8_t reserved;
  int32_t home_q[3];        /* HOME初始点，单位0.01rad。 */
  int32_t actual_q[3];      /* 最近可信实际位置，单位0.01rad。 */
  uint32_t tick_ms;
  uint32_t crc32;
} W25Q64_App_TentacleState_t;

uint8_t W25Q64_App_ReadJedecId(uint8_t jedec_id[3]);
uint8_t W25Q64_App_RunBasicTest(W25Q64_App_TestResult_t *result);
uint8_t W25Q64_App_SavePositionRecord(uint16_t record_type,
                                      const int32_t position_q[3],
                                      W25Q64_App_PositionRecord_t *saved_record);
uint8_t W25Q64_App_LoadLatestPositionRecord(uint16_t record_type,
                                            W25Q64_App_PositionRecord_t *record);
uint8_t W25Q64_App_SavePositionRecordForTentacle(uint8_t tentacle_id,
                                                 uint8_t board_id,
                                                 uint16_t record_type,
                                                 const int32_t position_q[3],
                                                 W25Q64_App_PositionRecord_t *saved_record);
uint8_t W25Q64_App_LoadLatestPositionRecordForTentacle(uint8_t tentacle_id,
                                                       uint16_t record_type,
                                                       W25Q64_App_PositionRecord_t *record);
uint8_t W25Q64_App_SaveTentacleState(uint8_t tentacle_id,
                                     uint8_t board_id,
                                     const int32_t home_q[3],
                                     const int32_t actual_q[3],
                                     W25Q64_App_TentacleState_t *saved_state);
uint8_t W25Q64_App_LoadTentacleState(uint8_t tentacle_id,
                                     W25Q64_App_TentacleState_t *state);
uint8_t W25Q64_App_SaveMotorConfig(const W25Q64_App_MotorConfig_t *config);
uint8_t W25Q64_App_LoadMotorConfig(W25Q64_App_MotorConfig_t *config);
uint8_t W25Q64_App_RequestEraseSector(uint32_t sector_address); /* 登记一个后台4KB扇区擦除请求。 */
void W25Q64_App_Task(void);                                      /* 主循环持续调用，每次只推进一步。 */
const W25Q64_App_EraseInfo_t *W25Q64_App_GetEraseInfo(void);     /* 获取当前后台擦除状态。 */
uint8_t W25Q64_App_TakeEraseResult(uint8_t *result);             /* 取得完成结果并恢复空闲。 */

#ifdef __cplusplus
}
#endif

#endif
