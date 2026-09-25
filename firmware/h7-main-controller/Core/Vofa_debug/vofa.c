#include "vofa.h"

#include "act_policy_app.h"
#include "act_stream.h"
#include "main.h"
#include "can_app.h"
#include "i2c.h"
#include "motor_control_app.h"
#include "usart.h"
#include "vl53_app.h"
#include "w25q64_app.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VOFA_RX_BUF_SIZE        128U
#define VOFA_LINE_BUF_SIZE      96U
#define VOFA_LED_ON             GPIO_PIN_RESET
#define VOFA_LED_OFF            GPIO_PIN_SET
#define VOFA_DEFAULT_REC_COUNT  30U
#define VOFA_DEFAULT_REC_MS     500U
#define VOFA_SAMPLE_TIMEOUT_MS  3000U
#define VOFA_SAMPLE_MAX_RETRY   3U
#define VOFA_CACHE_LINE_SIZE    32U
#define VOFA_BAL_MAX_BINS       16U
#define VOFA_G4_STATUS_MAX_AGE_MS 250U
#define VOFA_UART_PORT          7U     /* VOFA串口选择：1=USART1，7=UART7。 */
#define VOFA_TX_DMA_ENABLE      1U     /* 1=串口输出走DMA队列，避免日志阻塞主循环。 */
#define VOFA_TX_BUF_SIZE        256U
#define VOFA_TX_QUEUE_DEPTH     8U
#define VOFA_RX_RETRY_MS        100U   /* RX DMA停止后，自动恢复的最小间隔。 */

#if (VOFA_UART_PORT == 7U)
#define VOFA_UART_HANDLE        huart7
#define VOFA_UART_NAME          "UART7"
#else
#define VOFA_UART_HANDLE        huart1
#define VOFA_UART_NAME          "USART1"
#endif

/* ARMCC V5 对 UTF-8 中文字符串支持不稳定，中文输出统一使用UTF-8字节转义。 */
#define VOFA_TXT_KEY_LOCK       "\xE6\x8C\x89\xE9\x94\xAE\xE9\x94\x81\xE5\xAE\x9A" /* 按键锁定 */
#define VOFA_TXT_CONTROL_MODE   "\xE6\x8E\xA7\xE5\x88\xB6\xE6\xA8\xA1\xE5\xBC\x8F" /* 控制模式 */
#define VOFA_TXT_LOCKED         "\xE5\xB7\xB2\xE9\x94\x81\xE5\xAE\x9A"             /* 已锁定 */
#define VOFA_TXT_MODE_SELECTED  "\xE5\xB7\xB2\xE9\x80\x89\xE6\xA8\xA1\xE5\xBC\x8F" /* 已选模式 */
#define VOFA_TXT_INIT_DONE      "\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96\xE5\xAE\x8C\xE6\x88\x90" /* 初始化完成 */
#define VOFA_TXT_READY          "\xE5\x87\x86\xE5\xA4\x87\xE8\xBF\x9B\xE5\x85\xA5\xE8\xBF\x90\xE5\x8A\xA8" /* 准备进入运动 */
#define VOFA_TXT_IDLE           "\xE7\xA9\xBA\xE9\x97\xB2"                         /* 空闲 */
#define VOFA_TXT_COOP           "\xE5\x8F\x8C\xE8\xA7\xA6\xE6\x89\x8B\xE5\x8D\x8F\xE5\x90\x8C\xE6\x8A\x93\xE5\x8F\x96" /* 双触手协同抓取 */
#define VOFA_TXT_SOLO           "\xE5\x8D\x95\xE8\xA7\xA6\xE6\x89\x8B\xE7\x8B\xAC\xE7\xAB\x8B\xE6\x8E\xA7\xE5\x88\xB6" /* 单触手独立控制 */
#define VOFA_TXT_WRAP           "\xE5\x8F\xAF\xE7\xA9\xBF\xE6\x88\xB4\xE7\x8E\xAF\xE6\x8A\xB1\xE7\xBC\xA0\xE7\xBB\x95" /* 可穿戴环抱缠绕 */
#define VOFA_TXT_NONE           "\xE6\x9C\xAA\xE9\x80\x89\xE6\x8B\xA9"             /* 未选择 */
#define VOFA_TXT_STATE          "\xE7\x8A\xB6\xE6\x80\x81"                         /* 状态 */
#define VOFA_TXT_MODE           "\xE6\xA8\xA1\xE5\xBC\x8F"                         /* 模式 */
#define VOFA_TXT_KEY            "\xE6\x8C\x89\xE9\x94\xAE"                         /* 按键 */
#define VOFA_TXT_INIT           "\xE5\x88\x9D\xE5\xA7\x8B\xE5\x8C\x96"             /* 初始化 */
#define VOFA_TXT_TENTACLE       "\xE8\xA7\xA6\xE6\x89\x8B"                         /* 触手 */
#define VOFA_TXT_COOP_ANGLE     "\xE5\x8D\x8F\xE5\x90\x8C\xE8\xA7\x92\xE5\xBA\xA6" /* 协同角度 */
#define VOFA_TXT_COUNT          "\xE8\xAE\xA1\xE6\x95\xB0"                         /* 计数 */

#ifndef LED1_Pin
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOE
#endif

#ifndef LED2_Pin
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOE
#endif

#ifndef LED3_Pin
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOE
#endif

typedef enum
{
  VOFA_SAMPLE_MODE_NORMAL = 0U,
  VOFA_SAMPLE_MODE_BALANCED = 1U
} Vofa_SampleMode_t;

typedef struct
{
  uint8_t sample_waiting;        /* 采样请求已发出，正在等待结果。 */
  uint8_t record_active;         /* 连续采样是否开启。 */
  uint8_t error_flag;            /* 最近一次采样或命令是否出错。 */
  Vofa_SampleMode_t sample_mode; /* 当前连续采样模式：普通或距离均衡。 */
  uint8_t label_score;           /* 当前标签：0~100 为张握程度，255 表示无手。 */
  uint32_t record_count_cfg;      /* 连续采样默认次数。 */
  uint32_t record_interval_cfg;   /* 连续采样默认间隔。 */
  uint32_t record_remaining;     /* 连续采样剩余次数。 */
  uint32_t record_interval_ms;    /* 连续采样间隔。 */
  uint32_t record_next_ms;       /* 下次触发时间。 */
  uint8_t record_retry_count;     /* 当前连续采样点的失败重试次数。 */
  uint32_t sample_deadline_ms;    /* 当前这次采样的超时截止时间。 */
  uint32_t normal_saved_count;    /* 普通连续采样已成功保存数量。 */
  uint16_t bal_start_mm;          /* 均衡采样起始距离。 */
  uint16_t bal_end_mm;            /* 均衡采样结束距离。 */
  uint16_t bal_step_mm;           /* 均衡采样距离间隔。 */
  uint32_t bal_target_count;      /* 每个距离档目标数量。 */
  uint8_t bal_bin_num;            /* 当前有效距离档数量。 */
  uint8_t bal_pending_valid;      /* 当前等待写入的均衡采样是否已有分桶。 */
  uint8_t bal_pending_bin;        /* 当前等待写入样本对应的距离档。 */
  uint16_t bal_pending_z_mm;      /* 当前等待写入样本触发时的中心距离。 */
  uint32_t bal_counts[VOFA_BAL_MAX_BINS]; /* 每个距离档已保存数量。 */
  uint32_t bal_saved_total;       /* 均衡采样已保存总数。 */
} Vofa_State_t;

ALIGN_32BYTES(static uint8_t g_rx_buf[VOFA_RX_BUF_SIZE]);
static uint16_t g_rx_tail;
static char g_line_buf[VOFA_LINE_BUF_SIZE];
static uint16_t g_line_len;
static uint32_t g_rx_last_retry_ms;
static Vofa_State_t g_state;
static uint8_t g_stall_notice_mask[CAN_APP_G4_BOARD_COUNT]; /* 每块G4上一次已提示的堵转位，避免串口刷屏。 */

#if (VOFA_TX_DMA_ENABLE != 0U)
ALIGN_32BYTES(static uint8_t g_tx_queue[VOFA_TX_QUEUE_DEPTH][VOFA_TX_BUF_SIZE]);
static uint16_t g_tx_len[VOFA_TX_QUEUE_DEPTH];
static volatile uint8_t g_tx_head;
static volatile uint8_t g_tx_tail;
static volatile uint8_t g_tx_count;
static volatile uint8_t g_tx_busy;
static volatile uint32_t g_tx_drop_count;
#endif

UART_HandleTypeDef *Vofa_GetUartHandle(void)
{
  return &VOFA_UART_HANDLE;
}

#if (VOFA_TX_DMA_ENABLE != 0U)
/**
  * @brief  清理TX缓冲区DCache，保证DMA读到最新串口内容。
  * @param  data 待发送缓冲区。
  * @param  len  待发送字节数。
  */
static void Vofa_CleanTxDCache(const uint8_t *data, uint16_t len)
{
  uintptr_t start;
  uintptr_t end;

  if ((data == NULL) || (len == 0U))
  {
    return;
  }

  start = ((uintptr_t)data) & ~((uintptr_t)VOFA_CACHE_LINE_SIZE - 1U);
  end = ((uintptr_t)data + (uintptr_t)len + (uintptr_t)VOFA_CACHE_LINE_SIZE - 1U) &
        ~((uintptr_t)VOFA_CACHE_LINE_SIZE - 1U);
  SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}

/**
  * @brief  如果串口空闲，则启动队首一条DMA发送。
  * @param  无。
  *
  * 说明：发送失败时直接丢弃队首，避免调试输出反向卡死控制主循环。
  */
static void Vofa_TxTryStart(void)
{
  uint8_t tail;
  uint16_t len;

  if ((g_tx_busy != 0U) || (g_tx_count == 0U))
  {
    return;
  }

  tail = g_tx_tail;
  len = g_tx_len[tail];
  g_tx_busy = 1U;
  Vofa_CleanTxDCache(g_tx_queue[tail], len);

  if (HAL_UART_Transmit_DMA(&VOFA_UART_HANDLE, g_tx_queue[tail], len) != HAL_OK)
  {
    __disable_irq();
    g_tx_tail = (uint8_t)((g_tx_tail + 1U) % VOFA_TX_QUEUE_DEPTH);
    if (g_tx_count > 0U)
    {
      g_tx_count--;
    }
    g_tx_busy = 0U;
    g_tx_drop_count++;
    __enable_irq();
  }
}
#endif

uint8_t Vofa_Write(const char *text)
{
  size_t len;

  if (text == NULL)
  {
    return 1U;
  }

  len = strlen(text);
  if (len == 0U)
  {
    return 0U;
  }

#if (VOFA_TX_DMA_ENABLE != 0U)
  if (len >= VOFA_TX_BUF_SIZE)
  {
    len = VOFA_TX_BUF_SIZE - 1U;
  }

  __disable_irq();
  if (g_tx_count >= VOFA_TX_QUEUE_DEPTH)
  {
    g_tx_drop_count++;
    __enable_irq();
    return 2U;
  }

  memcpy(g_tx_queue[g_tx_head], text, len);
  g_tx_len[g_tx_head] = (uint16_t)len;
  g_tx_head = (uint8_t)((g_tx_head + 1U) % VOFA_TX_QUEUE_DEPTH);
  g_tx_count++;
  __enable_irq();

  Vofa_TxTryStart();
  return 0U;
#else
  if (len > UINT16_MAX)
  {
    len = UINT16_MAX;
  }
  return (HAL_UART_Transmit(&VOFA_UART_HANDLE,
                            (uint8_t *)text,
                            (uint16_t)len,
                            HAL_MAX_DELAY) == HAL_OK) ? 0U : 3U;
#endif
}

/**
  * @brief  获取足够新的G4状态帧。
  * @param  无。
  * @return 状态有效且未超时返回指针，否则返回NULL。
  */
static const Can_App_G4Status_t *Vofa_GetFreshG4StatusByBoard(uint8_t board_id)
{
  const Can_App_G4Status_t *status = Can_App_GetG4StatusById(board_id);

  if ((status == NULL) || (status->valid == 0U))
  {
    return NULL;
  }

  if ((HAL_GetTick() - status->last_rx_tick_ms) > VOFA_G4_STATUS_MAX_AGE_MS)
  {
    return NULL;
  }

  return status;
}

/**
  * @brief  获取当前选中触手对应G4板的新鲜状态。
  */
static const Can_App_G4Status_t *Vofa_GetFreshSelectedG4Status(void)
{
  return Vofa_GetFreshG4StatusByBoard(MotorControl_GetSelectedBoardId());
}

/**
  * @brief  通过当前VOFA串口输出格式化字符串。
  * @param  fmt printf 风格格式串。
  * @param  ... 参数列表。
  */
static void Vofa_Printf(const char *fmt, ...)
{
  char text[256];
  va_list ap;
  int len;

  if (fmt == NULL)
  {
    return;
  }

  va_start(ap, fmt);
  len = vsnprintf(text, sizeof(text), fmt, ap);
  va_end(ap);

  if (len <= 0)
  {
    return;
  }

  if (len >= (int)sizeof(text))
  {
    len = (int)sizeof(text) - 1;
  }
  text[len] = '\0';

  (void)Vofa_Write(text);
}

/**
  * @brief  刷新板载指示灯。
  * @param  无。
  *
  * 约定：LED1 表示采样模式，LED2 表示采样/写入进行中，LED3 表示错误。
  */
static void Vofa_RefreshLeds(void)
{
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
                    (VL53_App_GetMode() == VL53_APP_MODE_SAMPLE) ? VOFA_LED_ON : VOFA_LED_OFF);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,
                    (g_state.sample_waiting != 0U) ? VOFA_LED_ON : VOFA_LED_OFF);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,
                    (g_state.error_flag != 0U) ? VOFA_LED_ON : VOFA_LED_OFF);
}

/**
  * @brief  计算均衡采样总目标数量。
  * @param  无。
  * @return 总目标数量。
  */
static uint32_t Vofa_BalTotalTarget(void)
{
  return (uint32_t)g_state.bal_bin_num * g_state.bal_target_count;
}

/**
  * @brief  获取某个均衡距离档中心距离。
  * @param  bin 距离档序号。
  * @return 距离档中心，单位mm。
  */
static uint16_t Vofa_BalBinCenter(uint8_t bin)
{
  return (uint16_t)(g_state.bal_start_mm + ((uint16_t)bin * g_state.bal_step_mm));
}

/**
  * @brief  判断均衡采样是否已经采满。
  * @param  无。
  * @return 1已满，0未满。
  */
static uint8_t Vofa_BalIsDone(void)
{
  return ((g_state.bal_bin_num > 0U) &&
          (g_state.bal_saved_total >= Vofa_BalTotalTarget())) ? 1U : 0U;
}

/**
  * @brief  打印均衡采样计数表，三行横排显示。
  * @param  无。
  */
static void Vofa_PrintBalCounts(void)
{
  uint8_t bin;

  Vofa_Printf("BALCNT label=%u total=%lu/%lu target=%lu\r\n",
              (unsigned int)g_state.label_score,
              (unsigned long)g_state.bal_saved_total,
              (unsigned long)Vofa_BalTotalTarget(),
              (unsigned long)g_state.bal_target_count);

  Vofa_Printf("DIST ");
  for (bin = 0U; bin < g_state.bal_bin_num; bin++)
  {
    Vofa_Printf("%5u", (unsigned int)Vofa_BalBinCenter(bin));
  }
  Vofa_Printf("\r\n");

  Vofa_Printf("COUNT");
  for (bin = 0U; bin < g_state.bal_bin_num; bin++)
  {
    Vofa_Printf("%5lu", (unsigned long)g_state.bal_counts[bin]);
  }
  Vofa_Printf("\r\n");
}

/**
  * @brief  根据当前center_z查找最近的均衡距离档。
  * @param  center_z_mm 当前中心距离，单位mm。
  * @return >=0为距离档序号，-1表示超出所有档位容差。
  *
  * 说明：容差使用 step/2，例如step=20时允许±10mm。
  */
static int16_t Vofa_FindBalBin(uint16_t center_z_mm)
{
  uint16_t tolerance;
  uint32_t index;
  uint16_t center;
  uint16_t diff;

  if ((g_state.bal_bin_num == 0U) || (g_state.bal_step_mm == 0U))
  {
    return -1;
  }

  tolerance = (uint16_t)(g_state.bal_step_mm / 2U);
  if (center_z_mm < g_state.bal_start_mm)
  {
    diff = (uint16_t)(g_state.bal_start_mm - center_z_mm);
    return (diff <= tolerance) ? 0 : -1;
  }

  index = ((uint32_t)(center_z_mm - g_state.bal_start_mm) + tolerance) /
          g_state.bal_step_mm;
  if (index >= g_state.bal_bin_num)
  {
    index = g_state.bal_bin_num - 1U;
  }

  center = Vofa_BalBinCenter((uint8_t)index);
  diff = (center_z_mm >= center) ?
         (uint16_t)(center_z_mm - center) :
         (uint16_t)(center - center_z_mm);

  return (diff <= tolerance) ? (int16_t)index : -1;
}

/**
  * @brief  停止连续采样。
  * @param  清除错误标志后继续运行时可重新置位。
  */
static void Vofa_StopRecord(void)
{
  g_state.record_active = 0U;
  g_state.record_remaining = 0U;
  g_state.record_interval_ms = 0U;
  g_state.record_next_ms = 0U;
  g_state.record_retry_count = 0U;
  g_state.bal_pending_valid = 0U;
}

/**
  * @brief  强制结束采样流程并切回推理模式。
  * @param  无。
  *
  * 说明：用于 DONE、STOP、超时、失败等收尾路径，确保状态和CSV文件统一关闭。
  */
static void Vofa_ForceStopSamplingToInfer(void)
{
  Vofa_StopRecord();
  g_state.sample_waiting = 0U;
  g_state.sample_deadline_ms = 0U;
  g_state.bal_pending_valid = 0U;
  (void)VL53_App_SetMode(VL53_APP_MODE_INFER);
}

/**
  * @brief  打印当前支持的文本命令。
  * @param  无。
  */
static void Vofa_PrintHelp(void)
{
  Vofa_Printf("CMD: MODE, SAMPLE, REC, STOP, SETREC n ms, BALCFG start end step target ms, LABEL x, STATUS, ERRCLR, HELP\r\n");
  Vofa_Printf("CAN: MWORK?, MWORK SOLO n, MWORK DUAL 12/34 SAME/MIRROR, MWORK QUAD SAME/CENTER, TSEL n, TSTAT, CANSTAT, CANSTOP, CANREL/CANPOS dq0 dq1 dq2, CANABS home_dq0 home_dq1 home_dq2, CANZERO, CANHOME\r\n");
  Vofa_Printf("MCTRL: SNAP?, POS?, MSRC?/MSRC AI/PC/ACT, MCTRL?/MCTRL 0/1, ACTSTAT?, ACTOSC?, MKEY, MKEY?, MKEYCLR, MCOOP INIT x, MCOOP Bxx x, MCFG [bend stiff dead full speed_base speed_max], MVIRT x y bend stiff, MSETZERO q0 q1 q2, MSTAT\r\n");
  Vofa_Printf("W25Q64: W25ID, W25TEST, W25SAVE, W25LOAD\r\n");
  Vofa_Printf("VL53: VLSTAT, VLSCAN\r\n");
  Vofa_Printf("ACT: ACTMODEL\r\n");
}

static const char *Vofa_WorkModeName(uint8_t mode)
{
  return (mode == MOTOR_CONTROL_WORK_COOP) ? "COOP" : "SOLO";
}

/**
  * @brief  将控制来源转换为上位机可读字符串。
  */
static const char *Vofa_ControlSourceName(uint8_t source)
{
  switch (source)
  {
    case MOTOR_CONTROL_SOURCE_PC:
      return "PC";
    case MOTOR_CONTROL_SOURCE_ACT:
      return "ACT";
    default:
      return "AI";
  }
}

/**
  * @brief  将运动阻止原因转换为VOFA可读字符串。
  */
static const char *Vofa_BlockReasonName(uint8_t reason)
{
  switch (reason)
  {
    case MOTOR_CONTROL_BLOCK_NOT_READY:
      return "NOT_READY";
    case MOTOR_CONTROL_BLOCK_TX_BUSY:
      return "TX_BUSY";
    case MOTOR_CONTROL_BLOCK_FAULT:
      return "FAULT";
    default:
      return "NONE";
  }
}

/**
  * @brief  将安全停止原因转换为上位机协议字符串。
  */
static const char *Vofa_StopReasonName(uint8_t reason)
{
  switch (reason)
  {
    case MOTOR_CONTROL_STOP_CAN_TIMEOUT:
      return "CAN_TIMEOUT";
    case MOTOR_CONTROL_STOP_FAULT:
      return "FAULT";
    case MOTOR_CONTROL_STOP_RESTORE_LOST:
      return "RESTORE_LOST";
    case MOTOR_CONTROL_STOP_ESTOP:
      return "ESTOP";
    case MOTOR_CONTROL_STOP_GROUP_SYNC:
      return "GROUP_SYNC";
    case MOTOR_CONTROL_STOP_CAN_BUS_OFF:
      return "CAN_BUS_OFF";
    default:
      return "UNKNOWN";
  }
}

static const char *Vofa_KeyRunName(uint8_t value)
{
  return (value == MOTOR_CONTROL_RUN_KEY_MODE) ? VOFA_TXT_KEY_LOCK : VOFA_TXT_CONTROL_MODE;
}

static const char *Vofa_KeyStateName(uint8_t value)
{
  switch (value)
  {
    case MOTOR_CONTROL_KEY_STATE_LOCKED:
      return VOFA_TXT_LOCKED;
    case MOTOR_CONTROL_KEY_STATE_MODE:
      return VOFA_TXT_MODE_SELECTED;
    case MOTOR_CONTROL_KEY_STATE_INIT:
      return VOFA_TXT_INIT_DONE;
    case MOTOR_CONTROL_KEY_STATE_READY:
      return VOFA_TXT_READY;
    default:
      return VOFA_TXT_IDLE;
  }
}

static const char *Vofa_KeyModeName(uint8_t value)
{
  switch (value)
  {
    case MOTOR_CONTROL_MODE_COOP:
      return VOFA_TXT_COOP;
    case MOTOR_CONTROL_MODE_SOLO:
      return VOFA_TXT_SOLO;
    case MOTOR_CONTROL_MODE_WRAP:
      return VOFA_TXT_WRAP;
    default:
      return VOFA_TXT_NONE;
  }
}

/**
  * @brief  打印当前虚拟按键状态。
  * @param  info 按键状态指针。
  * @param  prefix 输出前缀，用于区分事件和查询。
  */
static void Vofa_PrintKeyInfo(const MotorControl_KeyInfo_t *info, const char *prefix)
{
  if (info == NULL)
  {
    return;
  }

  Vofa_Printf("%s%s\r\n", prefix, Vofa_KeyRunName(info->run_state));
  Vofa_Printf("  %s=%s  %s=%s\r\n",
              VOFA_TXT_STATE,
              Vofa_KeyStateName(info->key_state),
              VOFA_TXT_MODE,
              Vofa_KeyModeName(info->selected_mode));
  Vofa_Printf("  key=%u  init=%u  T=%u  angle=%ddeg  cnt=%lu\r\n",
              (unsigned int)info->last_key,
              (unsigned int)info->init_done,
              (unsigned int)(info->selected_tentacle + 1U),
              (int)info->coop_init_angle_deg,
              (unsigned long)info->notice_count);
  Vofa_Printf("  pulse=%u  raw=%u  stable=%u  pending=%u  wait=%ums\r\n",
              (unsigned int)info->pulse_state,
              (unsigned int)info->raw_bend,
              (unsigned int)info->pulse_stable,
              (unsigned int)info->pending_zone,
              (unsigned int)info->pending_wait_ms);
}

/**
  * @brief  打印一行虚拟按键调试状态。
  * @param  info 按键状态指针。
  */
static void Vofa_PrintKeyDebug(const MotorControl_KeyInfo_t *info)
{
  if (info == NULL)
  {
    return;
  }

  Vofa_Printf("MKEY DBG pulse=%u raw=%u stable=%u pending=%u wait=%ums\r\n",
              (unsigned int)info->pulse_state,
              (unsigned int)info->raw_bend,
              (unsigned int)info->pulse_stable,
              (unsigned int)info->pending_zone,
              (unsigned int)info->pending_wait_ms);
}

/**
  * @brief  低频任务中打印一次按键状态变化，避免在控制模块里直接依赖串口。
  * @param  无。
  */
static void Vofa_KeyNoticeTask(void)
{
  MotorControl_KeyInfo_t info;

  if (MotorControl_TakeKeyNotice(&info) != 0U)
  {
    Vofa_PrintKeyInfo(&info, "MKEY EVT ");
  }
}

/**
  * @brief  输出一次安全停止事件；DMA队列满时保留事件等待下一轮。
  */
static void Vofa_StopNoticeTask(void)
{
  MotorControl_StopNotice_t notice;

#if (VOFA_TX_DMA_ENABLE != 0U)
  if (g_tx_count >= VOFA_TX_QUEUE_DEPTH)
  {
    return;
  }
#endif

  if (MotorControl_TakeStopNotice(&notice) != 0U)
  {
    Vofa_Printf("MSTOP reason=%s mask=0x%02X\r\n",
                Vofa_StopReasonName(notice.reason),
                (unsigned int)notice.mask);
  }
}

/**
  * @brief  按需输出虚拟按键调试行。
  * @param  无。
  */
static void Vofa_KeyDebugTask(void)
{
  MotorControl_KeyInfo_t info;

  if (MotorControl_TakeKeyDebug(&info) != 0U)
  {
    Vofa_PrintKeyDebug(&info);
  }
}

/**
  * @brief  自动提示G4堵转保护状态。
  * @param  无。
  * @note   只在堵转位从0变为非0时打印一次；CANSTOP清故障后再次堵转会重新提示。
  */
static void Vofa_StallNoticeTask(void)
{
  uint8_t board_id;

  for (board_id = 0U; board_id < CAN_APP_G4_BOARD_COUNT; board_id++)
  {
    const Can_App_G4Status_t *status = Can_App_GetG4StatusById(board_id);
    uint8_t stall_mask;
    int32_t line_q[3] = {0, 0, 0};

    if ((status == NULL) || (status->valid == 0U) ||
        (Can_App_IsG4OnlineById(board_id, VOFA_G4_STATUS_MAX_AGE_MS) == 0U))
    {
      g_stall_notice_mask[board_id] = 0U;
      continue;
    }

    stall_mask = (uint8_t)(status->stall_fault_mask & 0x07U);
    if (stall_mask == 0U)
    {
      g_stall_notice_mask[board_id] = 0U;
      continue;
    }

    if ((stall_mask & (uint8_t)~g_stall_notice_mask[board_id]) == 0U)
    {
      continue;
    }

    g_stall_notice_mask[board_id] = stall_mask;
    Can_App_PositionToLineQ(board_id, status->actual_q, line_q);
    Vofa_Printf("ERR STALL board=%u mask=0x%X pwm=%u line=%ld,%ld,%ld\r\n",
                (unsigned int)board_id,
                (unsigned int)stall_mask,
                (unsigned int)status->pwm_started_mask,
                (long)line_q[0],
                (long)line_q[1],
                (long)line_q[2]);
  }
}

/**
  * @brief  扫描I2C2总线地址，用于确认VL53是否有ACK。
  * @param  无。
  *
  * 说明：打印的是7-bit地址；VL53默认应能看到0x29。
  */
static void Vofa_ScanI2c2(void)
{
  uint8_t addr;
  uint8_t found = 0U;

  Vofa_Printf("I2C2 scan:");
  for (addr = 1U; addr < 0x7FU; addr++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(addr << 1), 1U, 5U) == HAL_OK)
    {
      Vofa_Printf(" 0x%02X", (unsigned int)addr);
      found++;
    }
  }

  if (found == 0U)
  {
    Vofa_Printf(" none");
  }
  Vofa_Printf("\r\n");
}

/**
  * @brief  打印 VL53 推理链路状态，用于判断是否初始化和出帧。
  * @param  无。
  */
static void Vofa_PrintVl53Status(void)
{
  const VL53_App_Frame_t *frame = VL53_App_GetLatestFrame();
  const VL53_App_AiEstimate_t *ai = VL53_App_GetAiEstimate();
  const VL53_App_RuntimeStats_t *stats = VL53_App_GetRuntimeStats();
  Act_StreamStatus_t act_status;
  uint32_t age_ms = 0U;

  Act_Stream_GetStatus(&act_status);
  if ((frame != NULL) && (frame->valid != 0U))
  {
    age_ms = HAL_GetTick() - frame->timestamp_ms;
  }

  Vofa_Printf("VL ready=%u mode=%u new=%u valid=%u res=%u frame=%lu age=%lu ai_valid=%u ai_st=%u score=%u\r\n",
              (unsigned int)VL53_App_IsReady(),
              (unsigned int)VL53_App_GetMode(),
              (unsigned int)VL53_App_HasNewFrame(),
              (unsigned int)((frame != NULL) ? frame->valid : 0U),
              (unsigned int)((frame != NULL) ? frame->resolution : 0U),
              (unsigned long)((frame != NULL) ? frame->frame_count : 0UL),
              (unsigned long)age_ms,
              (unsigned int)((ai != NULL) ? ai->valid : 0U),
              (unsigned int)((ai != NULL) ? ai->status : 0U),
              (unsigned int)((ai != NULL) ? ai->openness_score : 0U));

  Vofa_Printf("VL PERF req=%uHz actual=%uHz int=%lums bytes=%lu cnh=%lu\r\n",
              (unsigned int)stats->requested_hz,
              (unsigned int)stats->actual_hz,
              (unsigned long)stats->actual_integration_ms,
              (unsigned long)stats->read_bytes,
              (unsigned long)stats->cnh_bytes);
  Vofa_Printf("VL TIME wait=%lums polls=%lu get=%lums post=%lums period=%lums\r\n",
              (unsigned long)stats->ready_wait_ms,
              (unsigned long)stats->ready_poll_count,
              (unsigned long)stats->get_data_ms,
              (unsigned long)stats->post_process_ms,
              (unsigned long)stats->frame_period_ms);
  Vofa_Printf("VL SEQ stream=%u delta=%u skip=%lu err=%lu act_sent=%lu act_drop=%lu\r\n",
              (unsigned int)stats->sensor_streamcount,
              (unsigned int)stats->stream_delta,
              (unsigned long)stats->skipped_frame_count,
              (unsigned long)stats->read_error_count,
              (unsigned long)act_status.sent_frame_count,
              (unsigned long)act_status.dropped_frame_count);
}

/**
  * @brief  触发一次采样请求。
  * @param  无。
  * @return 0 成功，非 0 表示当前繁忙或模式切换失败。
  */
static uint8_t Vofa_StartSample(void)
{
  if (g_state.sample_waiting != 0U)
  {
    return 1U;
  }

  if (VL53_App_SetMode(VL53_APP_MODE_SAMPLE) != 0U)
  {
    g_state.error_flag = 1U;
    return 2U;
  }

  VL53_App_RequestSample();
  g_state.sample_waiting = 1U;
  g_state.sample_deadline_ms = HAL_GetTick() + VOFA_SAMPLE_TIMEOUT_MS;
  Vofa_RefreshLeds();
  return 0U;
}

/**
  * @brief  触发一次均衡采样；不满足距离条件时只跳过本次，不写CSV。
  * @param  now_ms 当前HAL tick。
  * @return 0表示状态机可继续，非0表示底层采样启动失败。
  */
static uint8_t Vofa_StartBalancedSample(uint32_t now_ms)
{
  VL53_App_SampleStats_t stats;
  int16_t bin;

  if (Vofa_BalIsDone() != 0U)
  {
    Vofa_ForceStopSamplingToInfer();
    Vofa_Printf("BAL DONE label=%u total=%lu/%lu\r\n",
                (unsigned int)g_state.label_score,
                (unsigned long)g_state.bal_saved_total,
                (unsigned long)Vofa_BalTotalTarget());
    Vofa_PrintBalCounts();
    return 0U;
  }

  if (VL53_App_GetSampleStats(&stats) != 0U)
  {
    g_state.record_next_ms = now_ms + g_state.record_interval_ms;
    Vofa_Printf("BAL SKIP no_z label=%u\r\n", (unsigned int)g_state.label_score);
    Vofa_PrintBalCounts();
    return 0U;
  }

  bin = Vofa_FindBalBin(stats.center_z_mm);
  if (bin < 0)
  {
    g_state.record_next_ms = now_ms + g_state.record_interval_ms;
    Vofa_Printf("BAL SKIP out_of_range label=%u z=%u\r\n",
                (unsigned int)g_state.label_score,
                (unsigned int)stats.center_z_mm);
    Vofa_PrintBalCounts();
    return 0U;
  }

  if (g_state.bal_counts[bin] >= g_state.bal_target_count)
  {
    g_state.record_next_ms = now_ms + g_state.record_interval_ms;
    Vofa_Printf("BAL SKIP bin_full label=%u z=%u bin=%u count=%lu/%lu\r\n",
                (unsigned int)g_state.label_score,
                (unsigned int)stats.center_z_mm,
                (unsigned int)Vofa_BalBinCenter((uint8_t)bin),
                (unsigned long)g_state.bal_counts[bin],
                (unsigned long)g_state.bal_target_count);
    Vofa_PrintBalCounts();
    return 0U;
  }

  if (Vofa_StartSample() != 0U)
  {
    return 1U;
  }

  g_state.bal_pending_valid = 1U;
  g_state.bal_pending_bin = (uint8_t)bin;
  g_state.bal_pending_z_mm = stats.center_z_mm;
  return 0U;
}

/**
  * @brief  根据当前采样模式触发下一次连续采样。
  * @param  now_ms 当前HAL tick。
  * @return 0成功，非0表示底层采样启动失败。
  */
static uint8_t Vofa_StartRecordSample(uint32_t now_ms)
{
  if (g_state.sample_mode == VOFA_SAMPLE_MODE_BALANCED)
  {
    return Vofa_StartBalancedSample(now_ms);
  }

  return Vofa_StartSample();
}

/**
  * @brief  判断UART7命令是否会与ACT单触手控制发生冲突。
  * @param  cmd 已完成分词的UART7命令名。
  * @return 1表示ACT控制期间禁止，0表示查询或安全停止仍可使用。
  * @note   CANSTOP、SNAP?、POS?、CANSTAT等停止和查询命令不在禁止列表。
  */
static uint8_t Vofa_IsBlockedByActControl(const char *cmd)
{
  return ((strcmp(cmd, "MWORK") == 0) ||
          (strcmp(cmd, "TSEL") == 0) ||
          (strcmp(cmd, "TMODE") == 0) ||
          (strcmp(cmd, "MKEY") == 0) ||
          (strcmp(cmd, "MKEYCLR") == 0) ||
          (strcmp(cmd, "MCOOP") == 0) ||
          (strcmp(cmd, "MVIRT") == 0) ||
          (strcmp(cmd, "MCFG") == 0) ||
          (strcmp(cmd, "CANZERO") == 0) ||
          (strcmp(cmd, "CANHOME") == 0) ||
          (strcmp(cmd, "CANPOS") == 0) ||
          (strcmp(cmd, "CANREL") == 0) ||
          (strcmp(cmd, "CANABS") == 0) ||
          (strcmp(cmd, "MSETZERO") == 0)) ? 1U : 0U;
}

/**
  * @brief  将ACT模型输出四舍五入为串口显示使用的整数q。
  * @param  value 模型输出的浮点位置，单位为0.01rad。
  * @return 限制在int32_t范围内的整数位置。
  *
  * @note   避免使用printf浮点格式，减少运行库和Flash开销。
  */
static int32_t Vofa_ActQToInt(float value)
{
  if (value >= 2147483520.0f)
  {
    return INT32_MAX;
  }

  if (value <= -2147483648.0f)
  {
    return INT32_MIN;
  }

  return (value >= 0.0f) ?
         (int32_t)(value + 0.5f) :
         (int32_t)(value - 0.5f);
}

/**
  * @brief  处理一整行文本命令。
  * @param  line 以 '\0' 结尾的可写命令行。
  */
static void Vofa_HandleLine(char *line)
{
  char *cmd;
  char *arg1;
  char *arg2;
  uint32_t count;
  uint32_t interval_ms;
  uint32_t label;

  if (line == NULL)
  {
    return;
  }

  cmd = strtok(line, " \t");
  if (cmd == NULL)
  {
    return;
  }

  if ((Act_Stream_HasControl() != 0U) &&
      (Vofa_IsBlockedByActControl(cmd) != 0U))
  {
    Vofa_Printf("ERR ACT_CONTROL\r\n");
    return;
  }

  if (strcmp(cmd, "ACTMODEL") == 0)
  {
    ActPolicyApp_Status_t status;
    ActPolicyApp_Result_t result;
    int32_t q0;
    int32_t q1;
    int32_t q2;

    result = ActPolicyApp_Init();
    if (result == ACT_POLICY_APP_RESULT_OK)
    {
      result = ActPolicyApp_RunSelfTest();
    }

    ActPolicyApp_GetStatus(&status);
    q0 = Vofa_ActQToInt(status.first_action_q[0]);
    q1 = Vofa_ActQToInt(status.first_action_q[1]);
    q2 = Vofa_ActQToInt(status.first_action_q[2]);

    Vofa_Printf(
      "ACTMODEL init=%u result=%u runs=%lu time=%lums q=%ld,%ld,%ld\r\n",
      (unsigned int)status.initialized,
      (unsigned int)result,
      (unsigned long)status.run_count,
      (unsigned long)status.inference_time_ms,
      (long)q0,
      (long)q1,
      (long)q2);
    return;
  }

  if (strcmp(cmd, "ACTSTAT?") == 0)
  {
    ActPolicyApp_Status_t status;

    ActPolicyApp_GetStatus(&status);
    Vofa_Printf("ACTSTAT,%u,%u,%u,%lu,%lu\r\n",
                (unsigned int)status.run_state,
                (unsigned int)status.history_count,
                (unsigned int)status.result,
                (unsigned long)status.run_count,
                (unsigned long)status.inference_time_ms);
    return;
  }

  if (strcmp(cmd, "ACTOSC?") == 0)
  {
    ActPolicyApp_OscDebug_t debug;

    ActPolicyApp_GetOscDebug(&debug);
    Vofa_Printf(
      "ACTOSC,%u,0x%02X,%u,%u,%lu,%lu,%u,%u,%u,%lu\r\n",
      (unsigned int)debug.active_ensemble_count,
      (unsigned int)debug.oscillation_mask,
      (unsigned int)debug.sample_count,
      (unsigned int)debug.motor,
      (unsigned long)debug.actual_span_q,
      (unsigned long)debug.actual_backtrack_q,
      (unsigned int)debug.actual_reversals,
      (unsigned int)debug.sent_reversals,
      (unsigned int)debug.model_reversals,
      (unsigned long)debug.frame_delta);
    return;
  }

  if ((strcmp(cmd, "GESTURE") == 0) ||
      (strcmp(cmd, "GESTURE?") == 0))
  {
    arg1 = strtok(NULL, " \t");

    if (strcmp(cmd, "GESTURE?") == 0)
    {
      Vofa_Printf("GESTURE,%u\r\n",
                  (unsigned int)VL53_App_GetGestureInferEnable());
      return;
    }

    if ((arg1 == NULL) ||
        ((strcmp(arg1, "0") != 0) && (strcmp(arg1, "1") != 0)))
    {
      Vofa_Printf("ERR GESTURE\r\n");
      return;
    }

    VL53_App_SetGestureInferEnable(
      (strcmp(arg1, "1") == 0) ? 1U : 0U);
    Vofa_Printf("OK GESTURE %u\r\n",
                (unsigned int)VL53_App_GetGestureInferEnable());
    return;
  }

  if (strcmp(cmd, "MODE") == 0)
  {
    VL53_App_Mode_t current_mode;

    if (g_state.sample_waiting != 0U)
    {
      Vofa_Printf("ERR BUSY\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    current_mode = VL53_App_GetMode();
    Vofa_ForceStopSamplingToInfer();
    g_state.error_flag = 0U;

    if (current_mode == VL53_APP_MODE_SAMPLE)
    {
      Vofa_Printf("OK MODE INFER\r\n");
      Vofa_RefreshLeds();
      return;
    }

    (void)VL53_App_SetMode(VL53_APP_MODE_SAMPLE);
    Vofa_Printf("OK MODE SAMPLE\r\n");
    Vofa_RefreshLeds();
    return;
  }

  if (strcmp(cmd, "SAMPLE") == 0)
  {
    Vofa_ForceStopSamplingToInfer();
    g_state.error_flag = 0U;
    if (Vofa_StartSample() == 0U)
    {
      Vofa_Printf("OK SAMPLE\r\n");
    }
    else
    {
      Vofa_Printf("ERR BUSY\r\n");
    }
    return;
  }

  if (strcmp(cmd, "REC") == 0)
  {
    if (g_state.record_active != 0U)
    {
      Vofa_ForceStopSamplingToInfer();
      Vofa_Printf("OK REC STOP\r\n");
      Vofa_RefreshLeds();
      return;
    }

    if ((g_state.sample_waiting != 0U) || (g_state.record_active != 0U))
    {
      Vofa_Printf("ERR BUSY\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    g_state.record_active = 1U;
    g_state.record_remaining = (g_state.sample_mode == VOFA_SAMPLE_MODE_BALANCED) ?
                               (Vofa_BalTotalTarget() - g_state.bal_saved_total) :
                               g_state.record_count_cfg;
    g_state.record_interval_ms = g_state.record_interval_cfg;
    g_state.record_next_ms = HAL_GetTick();
    g_state.error_flag = 0U;
    g_state.bal_pending_valid = 0U;
    if (g_state.sample_mode == VOFA_SAMPLE_MODE_NORMAL)
    {
      g_state.normal_saved_count = 0U;
    }

    if (Vofa_StartRecordSample(g_state.record_next_ms) == 0U)
    {
      Vofa_Printf("OK REC START\r\n");
    }
    else
    {
      Vofa_ForceStopSamplingToInfer();
      Vofa_Printf("ERR BUSY\r\n");
    }
    return;
  }

  if (strcmp(cmd, "SETREC") == 0)
  {
    arg1 = strtok(NULL, " \t");
    arg2 = strtok(NULL, " \t");
    if ((arg1 == NULL) || (arg2 == NULL))
    {
      Vofa_Printf("ERR SETREC\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    count = (uint32_t)strtoul(arg1, NULL, 10);
    interval_ms = (uint32_t)strtoul(arg2, NULL, 10);
    if ((count == 0U) || (interval_ms == 0U))
    {
      Vofa_Printf("ERR SETREC_ARG\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    if ((g_state.sample_waiting != 0U) || (g_state.record_active != 0U))
    {
      Vofa_Printf("ERR BUSY\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    g_state.record_count_cfg = count;
    g_state.record_interval_cfg = interval_ms;
    g_state.sample_mode = VOFA_SAMPLE_MODE_NORMAL;
    g_state.normal_saved_count = 0U;
    g_state.error_flag = 0U;
    Vofa_Printf("OK SETREC %lu %lu mode=NORMAL\r\n",
                (unsigned long)g_state.record_count_cfg,
                (unsigned long)g_state.record_interval_cfg);
    Vofa_RefreshLeds();
    return;
  }

  if (strcmp(cmd, "BALCFG") == 0)
  {
    char *arg3;
    char *arg4;
    char *arg5;
    uint32_t start_mm;
    uint32_t end_mm;
    uint32_t step_mm;
    uint32_t target_count;
    uint32_t bin_num;
    uint8_t bin;

    arg1 = strtok(NULL, " \t");
    arg2 = strtok(NULL, " \t");
    arg3 = strtok(NULL, " \t");
    arg4 = strtok(NULL, " \t");
    arg5 = strtok(NULL, " \t");
    if ((arg1 == NULL) || (arg2 == NULL) || (arg3 == NULL) ||
        (arg4 == NULL) || (arg5 == NULL))
    {
      Vofa_Printf("ERR BALCFG\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    start_mm = (uint32_t)strtoul(arg1, NULL, 10);
    end_mm = (uint32_t)strtoul(arg2, NULL, 10);
    step_mm = (uint32_t)strtoul(arg3, NULL, 10);
    target_count = (uint32_t)strtoul(arg4, NULL, 10);
    interval_ms = (uint32_t)strtoul(arg5, NULL, 10);

    if ((step_mm == 0U) || (target_count == 0U) || (interval_ms == 0U) ||
        (start_mm > end_mm) || (((end_mm - start_mm) % step_mm) != 0U))
    {
      Vofa_Printf("ERR BALCFG_ARG\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    bin_num = ((end_mm - start_mm) / step_mm) + 1U;
    if ((bin_num == 0U) || (bin_num > VOFA_BAL_MAX_BINS) ||
        (target_count > (0xFFFFFFFFUL / bin_num)) ||
        (end_mm > 65535U) || (step_mm > 65535U))
    {
      Vofa_Printf("ERR BALCFG_RANGE\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    if ((g_state.sample_waiting != 0U) || (g_state.record_active != 0U))
    {
      Vofa_Printf("ERR BUSY\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    g_state.sample_mode = VOFA_SAMPLE_MODE_BALANCED;
    g_state.record_interval_cfg = interval_ms;
    g_state.record_count_cfg = bin_num * target_count;
    g_state.bal_start_mm = (uint16_t)start_mm;
    g_state.bal_end_mm = (uint16_t)end_mm;
    g_state.bal_step_mm = (uint16_t)step_mm;
    g_state.bal_target_count = target_count;
    g_state.bal_bin_num = (uint8_t)bin_num;
    g_state.bal_pending_valid = 0U;
    g_state.bal_saved_total = 0U;
    for (bin = 0U; bin < VOFA_BAL_MAX_BINS; bin++)
    {
      g_state.bal_counts[bin] = 0U;
    }

    g_state.error_flag = 0U;
    Vofa_Printf("OK BALCFG %u-%u step=%u target=%lu interval=%lu bins=%u\r\n",
                (unsigned int)g_state.bal_start_mm,
                (unsigned int)g_state.bal_end_mm,
                (unsigned int)g_state.bal_step_mm,
                (unsigned long)g_state.bal_target_count,
                (unsigned long)g_state.record_interval_cfg,
                (unsigned int)g_state.bal_bin_num);
    Vofa_PrintBalCounts();
    Vofa_RefreshLeds();
    return;
  }

  if (strcmp(cmd, "LABEL") == 0)
  {
    arg1 = strtok(NULL, " \t");
    if (arg1 == NULL)
    {
      Vofa_Printf("ERR LABEL\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    label = (uint32_t)strtoul(arg1, NULL, 10);
    if ((label > 100U) && (label != VL53_APP_LABEL_NONE))
    {
      Vofa_Printf("ERR LABEL_ARG\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    g_state.label_score = (uint8_t)label;
    VL53_App_SetLabelScore(g_state.label_score);
    g_state.error_flag = 0U;
    Vofa_Printf("OK LABEL %u\r\n", (unsigned int)g_state.label_score);
    Vofa_RefreshLeds();
    return;
  }

  if (strcmp(cmd, "STOP") == 0)
  {
    Vofa_ForceStopSamplingToInfer();
    Vofa_Printf("OK STOP\r\n");
    Vofa_RefreshLeds();
    return;
  }

  if (strcmp(cmd, "ERRCLR") == 0)
  {
    g_state.error_flag = 0U;
    Vofa_Printf("OK ERRCLR\r\n");
    Vofa_RefreshLeds();
    return;
  }

  if (strcmp(cmd, "HELP") == 0)
  {
    Vofa_PrintHelp();
    return;
  }

  if (strcmp(cmd, "W25ID") == 0)
  {
    uint8_t id[3];
    uint8_t status;

    status = W25Q64_App_ReadJedecId(id);
    if (status == W25Q64_APP_OK)
    {
      Vofa_Printf("OK W25ID %02X %02X %02X\r\n",
                  (unsigned int)id[0],
                  (unsigned int)id[1],
                  (unsigned int)id[2]);
    }
    else
    {
      Vofa_Printf("ERR W25ID status=%u\r\n", (unsigned int)status);
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    return;
  }

  if (strcmp(cmd, "W25TEST") == 0)
  {
    W25Q64_App_TestResult_t result;
    uint8_t status;

    status = W25Q64_App_RunBasicTest(&result);
    if (status == W25Q64_APP_OK)
    {
      Vofa_Printf("OK W25TEST id=%02X %02X %02X addr=0x%06lX len=%lu sr1=%02X\r\n",
                  (unsigned int)result.jedec_id[0],
                  (unsigned int)result.jedec_id[1],
                  (unsigned int)result.jedec_id[2],
                  (unsigned long)result.test_addr,
                  (unsigned long)result.test_len,
                  (unsigned int)result.status1);
    }
    else
    {
      Vofa_Printf("ERR W25TEST status=%u id=%02X %02X %02X addr=0x%06lX mismatch=%lu exp=%02X act=%02X sr1=%02X\r\n",
                  (unsigned int)status,
                  (unsigned int)result.jedec_id[0],
                  (unsigned int)result.jedec_id[1],
                  (unsigned int)result.jedec_id[2],
                  (unsigned long)result.test_addr,
                  (unsigned long)result.mismatch_index,
                  (unsigned int)result.expected,
                  (unsigned int)result.actual,
                  (unsigned int)result.status1);
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    return;
  }

  if (strcmp(cmd, "W25SAVE") == 0)
  {
    uint8_t tentacle = MotorControl_GetSelectedTentacle();
    uint8_t board_id = MotorControl_GetSelectedBoardId();
    const Can_App_G4Status_t *can_status = Vofa_GetFreshSelectedG4Status();
    W25Q64_App_PositionRecord_t record;
    int32_t actual_line_q[3] = {0, 0, 0};
    uint8_t status;

    if (MotorControl_IsRestorePending() != 0U)
    {
      Vofa_Printf("ERR W25SAVE RESTORE_PENDING\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    if (can_status == NULL)
    {
      Vofa_Printf("ERR W25SAVE CAN_STALE\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    if (can_status->position_restore_valid == 0U)
    {
      Vofa_Printf("ERR W25SAVE RESTORE_INVALID\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    status = W25Q64_App_SavePositionRecordForTentacle(tentacle,
                                                      board_id,
                                                      W25Q64_APP_RECORD_TYPE_ACTUAL,
                                                      can_status->actual_q,
                                                      &record);
    if (status == W25Q64_APP_OK)
    {
      MotorControl_UpdateRestoreActualQ(record.position_q[0],
                                        record.position_q[1],
                                        record.position_q[2]);
      Can_App_PositionToLineQ(board_id, record.position_q, actual_line_q);
      Vofa_Printf("OK W25SAVE tentacle=%u board=%u actual seq=%lu actual_line=%ld,%ld,%ld tick=%lu\r\n",
                  (unsigned int)tentacle,
                  (unsigned int)board_id,
                  (unsigned long)record.seq,
                  (long)actual_line_q[0],
                  (long)actual_line_q[1],
                  (long)actual_line_q[2],
                  (unsigned long)record.tick_ms);
    }
    else
    {
      Vofa_Printf("ERR W25SAVE status=%u\r\n", (unsigned int)status);
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    return;
  }

  if (strcmp(cmd, "W25LOAD") == 0)
  {
    uint8_t tentacle = MotorControl_GetSelectedTentacle();
    uint8_t board_id = MotorControl_GetSelectedBoardId();
    W25Q64_App_PositionRecord_t home_record;
    W25Q64_App_PositionRecord_t actual_record;
    int32_t actual_line_q[3] = {0, 0, 0};
    uint8_t status;
    uint8_t actual_status;

    status = W25Q64_App_LoadLatestPositionRecordForTentacle(tentacle,
                                                            W25Q64_APP_RECORD_TYPE_HOME,
                                                            &home_record);
    if (status == W25Q64_APP_OK)
    {
      MotorControl_SetNeutralQ(home_record.position_q[0],
                               home_record.position_q[1],
                               home_record.position_q[2]);

      actual_status = W25Q64_App_LoadLatestPositionRecordForTentacle(tentacle,
                                                                     W25Q64_APP_RECORD_TYPE_ACTUAL,
                                                                     &actual_record);
      if (actual_status == W25Q64_APP_OK)
      {
        MotorControl_UpdateRestoreActualQ(actual_record.position_q[0],
                                          actual_record.position_q[1],
                                          actual_record.position_q[2]);
        Can_App_PositionToLineQ(board_id, actual_record.position_q, actual_line_q);
        Vofa_Printf("OK W25LOAD tentacle=%u board=%u home_seq=%lu home=%ld,%ld,%ld actual_seq=%lu actual_line=%ld,%ld,%ld\r\n",
                    (unsigned int)tentacle,
                    (unsigned int)board_id,
                    (unsigned long)home_record.seq,
                    (long)home_record.position_q[0],
                    (long)home_record.position_q[1],
                    (long)home_record.position_q[2],
                    (unsigned long)actual_record.seq,
                    (long)actual_line_q[0],
                    (long)actual_line_q[1],
                    (long)actual_line_q[2]);
      }
      else
      {
        Vofa_Printf("OK W25LOAD tentacle=%u board=%u home_seq=%lu home=%ld,%ld,%ld actual_status=%u\r\n",
                    (unsigned int)tentacle,
                    (unsigned int)board_id,
                    (unsigned long)home_record.seq,
                    (long)home_record.position_q[0],
                    (long)home_record.position_q[1],
                    (long)home_record.position_q[2],
                    (unsigned int)actual_status);
      }
    }
    else
    {
      Vofa_Printf("ERR W25LOAD status=%u\r\n", (unsigned int)status);
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    return;
  }

  if (strcmp(cmd, "SNAP?") == 0)
  {
    const MotorControl_Status_t *mstat = MotorControl_GetStatus();
    MotorControl_Summary_t summary;

    MotorControl_GetSummary(&summary);
    Vofa_Printf("SNAP,1,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%u,%u,%u\r\n",
                (unsigned int)mstat->control_source,
                (unsigned int)mstat->enable,
                (unsigned int)mstat->active_mask,
                (unsigned int)summary.relation,
                (unsigned int)mstat->selected_tentacle,
                (unsigned int)summary.online_mask,
                (unsigned int)summary.ready_mask,
                (unsigned int)summary.fault_mask,
                (unsigned int)mstat->hand_present,
                (int)mstat->input_x,
                (int)mstat->input_y,
                (unsigned int)mstat->bend_0_100,
                (unsigned int)mstat->stiffness_0_100,
                (unsigned int)mstat->last_tx_status);
    return;
  }

  if (strcmp(cmd, "POS?") == 0)
  {
    MotorControl_Pose_t poses[MOTOR_CONTROL_POSE_COUNT];

    MotorControl_GetPoses(poses);
    Vofa_Printf(
      "POS,1,%u,%ld,%ld,%ld,%u,%ld,%ld,%ld,"
      "%u,%ld,%ld,%ld,%u,%ld,%ld,%ld\r\n",
      (unsigned int)poses[0].status,
      (long)poses[0].dq[0], (long)poses[0].dq[1], (long)poses[0].dq[2],
      (unsigned int)poses[1].status,
      (long)poses[1].dq[0], (long)poses[1].dq[1], (long)poses[1].dq[2],
      (unsigned int)poses[2].status,
      (long)poses[2].dq[0], (long)poses[2].dq[1], (long)poses[2].dq[2],
      (unsigned int)poses[3].status,
      (long)poses[3].dq[0], (long)poses[3].dq[1], (long)poses[3].dq[2]);
    return;
  }

  if ((strcmp(cmd, "MWORK") == 0) || (strcmp(cmd, "MWORK?") == 0))
  {
    const MotorControl_Status_t *mstat = MotorControl_GetStatus();
    uint8_t active_mask = 0U;
    uint8_t mirror_enable = 0U;
    uint8_t status;

    arg1 = strtok(NULL, " \t");
    if ((strcmp(cmd, "MWORK?") == 0) || (arg1 == NULL))
    {
      Vofa_Printf("MWORK mode=%s mask=0x%02X mirror=%u TSEL=%u\r\n",
                  Vofa_WorkModeName(mstat->work_mode),
                  (unsigned int)mstat->active_mask,
                  (unsigned int)mstat->mirror_enable,
                  (unsigned int)mstat->selected_tentacle);
      return;
    }

    arg2 = strtok(NULL, " \t");
    if ((strcmp(arg1, "SOLO") == 0) && (arg2 != NULL))
    {
      int32_t tentacle = strtol(arg2, NULL, 10);

      if ((tentacle < 1) || (tentacle > 4))
      {
        Vofa_Printf("ERR MWORK SOLO_ARG\r\n");
        return;
      }
      active_mask = (uint8_t)(1U << (tentacle - 1));
    }
    else if ((strcmp(arg1, "DUAL") == 0) && (arg2 != NULL))
    {
      char *relation = strtok(NULL, " \t");

      if (strcmp(arg2, "12") == 0)
      {
        active_mask = MOTOR_CONTROL_DUAL12_MASK;
      }
      else if (strcmp(arg2, "34") == 0)
      {
        active_mask = MOTOR_CONTROL_DUAL34_MASK;
      }
      else
      {
        Vofa_Printf("ERR MWORK DUAL_GROUP\r\n");
        return;
      }

      if ((relation != NULL) && (strcmp(relation, "SAME") == 0))
      {
        mirror_enable = 0U;
      }
      else if ((relation != NULL) && (strcmp(relation, "MIRROR") == 0))
      {
        mirror_enable = 1U;
      }
      else
      {
        Vofa_Printf("ERR MWORK DUAL_RELATION\r\n");
        return;
      }
    }
    else if ((strcmp(arg1, "QUAD") == 0) &&
             (arg2 != NULL) &&
             ((strcmp(arg2, "SAME") == 0) ||
              (strcmp(arg2, "CENTER") == 0)))
    {
      active_mask = MOTOR_CONTROL_QUAD_MASK;
      mirror_enable = (strcmp(arg2, "CENTER") == 0) ? 1U : 0U;
    }
    else
    {
      Vofa_Printf("ERR MWORK ARG\r\n");
      return;
    }

    status = MotorControl_SelectWork(active_mask, mirror_enable);
    if (status == 0U)
    {
      mstat = MotorControl_GetStatus();
      Vofa_Printf("OK MWORK mode=%s mask=0x%02X mirror=%u TSEL=%u\r\n",
                  Vofa_WorkModeName(mstat->work_mode),
                  (unsigned int)mstat->active_mask,
                  (unsigned int)mstat->mirror_enable,
                  (unsigned int)mstat->selected_tentacle);
    }
    else
    {
      Vofa_Printf("ERR MWORK status=%u\r\n", (unsigned int)status);
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    return;
  }

  if (strcmp(cmd, "TSEL") == 0)
  {
    int32_t tentacle;
    uint8_t status;

    arg1 = strtok(NULL, " \t");
    if (arg1 == NULL)
    {
      Vofa_Printf("ERR TSEL\r\n");
      return;
    }

    tentacle = strtol(arg1, NULL, 10);
    if ((tentacle < 1) || (tentacle > 255))
    {
      Vofa_Printf("ERR TSEL_ARG\r\n");
      return;
    }

    status = MotorControl_SetSelectedTentacle((uint8_t)tentacle);
    if (status == 0U)
    {
      Vofa_Printf("OK TSEL tentacle=%u board=%u mctrl=%u\r\n",
                  (unsigned int)MotorControl_GetSelectedTentacle(),
                  (unsigned int)MotorControl_GetSelectedBoardId(),
                  (unsigned int)MotorControl_GetStatus()->enable);
    }
    else
    {
      Vofa_Printf("ERR TSEL_RANGE\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    return;
  }

  if (strcmp(cmd, "TMODE") == 0)
  {
    uint8_t mode;

    arg1 = strtok(NULL, " \t");
    if (arg1 == NULL)
    {
      Vofa_Printf("TMODE %s\r\n", Vofa_WorkModeName(MotorControl_GetWorkMode()));
      return;
    }

    if (strcmp(arg1, "SOLO") == 0)
    {
      mode = MOTOR_CONTROL_WORK_SOLO;
    }
    else if (strcmp(arg1, "COOP") == 0)
    {
      mode = MOTOR_CONTROL_WORK_COOP;
    }
    else
    {
      Vofa_Printf("ERR TMODE_ARG\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    if (MotorControl_SetWorkMode(mode) == 0U)
    {
      Vofa_Printf("OK TMODE %s\r\n", Vofa_WorkModeName(MotorControl_GetWorkMode()));
    }
    else
    {
      Vofa_Printf("ERR TMODE\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    return;
  }

  if (strcmp(cmd, "TSTAT") == 0)
  {
    uint8_t board_id = MotorControl_GetSelectedBoardId();
    const Can_App_G4Status_t *can_status = Can_App_GetG4StatusById(board_id);
    uint32_t age_ms = 0U;
    int32_t line_q[3] = {0, 0, 0};

    if (can_status->valid != 0U)
    {
      age_ms = HAL_GetTick() - can_status->last_rx_tick_ms;
    }

    Can_App_PositionToLineQ(board_id, can_status->actual_q, line_q);
    Vofa_Printf("TSTAT tentacle=%u board=%u valid=%u online=%u age=%lu restore=%u pending=%u stall=0x%X line=%ld,%ld,%ld rx=%lu\r\n",
                (unsigned int)MotorControl_GetSelectedTentacle(),
                (unsigned int)board_id,
                (unsigned int)can_status->valid,
                (unsigned int)Can_App_IsG4OnlineById(board_id, 500U),
                (unsigned long)age_ms,
                (unsigned int)can_status->position_restore_valid,
                (unsigned int)can_status->position_pending,
                (unsigned int)can_status->stall_fault_mask,
                (long)line_q[0],
                (long)line_q[1],
                (long)line_q[2],
                (unsigned long)can_status->rx_frame_count);
    return;
  }

  if (strcmp(cmd, "CANSTAT") == 0)
  {
    uint8_t board_id = MotorControl_GetSelectedBoardId();
    const Can_App_G4Status_t *can_status = Can_App_GetG4StatusById(board_id);
    uint32_t age_ms = 0U;
    int32_t line_q[3] = {0, 0, 0};

    if (can_status->valid != 0U)
    {
      age_ms = HAL_GetTick() - can_status->last_rx_tick_ms;
    }

    Can_App_PositionToLineQ(board_id, can_status->actual_q, line_q);
    Vofa_Printf("CAN board=%u ready=%u tx=%lu g4_valid=%u g4_h7_recent=%u age=%lu rx=%lu g4rx=%u g4tx=%u err=%lu last_seq=%u cmd=%u flags=%u mask=%u pending=%u restore=%u stall=0x%X pwm=%u line=%ld,%ld,%ld\r\n",
                (unsigned int)board_id,
                (unsigned int)Can_App_IsReady(),
                (unsigned long)Can_App_GetTxCount(),
                (unsigned int)can_status->valid,
                (unsigned int)Can_App_IsG4OnlineById(board_id, 500U),
                (unsigned long)age_ms,
                (unsigned long)can_status->rx_frame_count,
                (unsigned int)can_status->g4_rx_count_low16,
                (unsigned int)can_status->g4_tx_count_low16,
                (unsigned long)can_status->error_count,
                (unsigned int)can_status->last_position_seq,
                (unsigned int)can_status->last_position_command,
                (unsigned int)can_status->last_position_flags,
                (unsigned int)can_status->last_position_motor_mask,
                (unsigned int)can_status->position_pending,
                (unsigned int)can_status->position_restore_valid,
                (unsigned int)can_status->stall_fault_mask,
                (unsigned int)can_status->pwm_started_mask,
                (long)line_q[0],
                (long)line_q[1],
                (long)line_q[2]);
    return;
  }

  if (strcmp(cmd, "CANSTOP") == 0)
  {
    uint8_t status;

    /* 先急停，再清ACT状态，避免同一按键连续发送两轮STOP。 */
    status = MotorControl_EmergencyStop();
    (void)ActPolicyApp_SetRun(0U, ACT_POLICY_APP_TENTACLE);

    if (status == 0U)
    {
      const MotorControl_Status_t *mstat = MotorControl_GetStatus();
      Vofa_Printf("OK CANSTOP mode=%s mask=0x%02X status=0 tx=%lu\r\n",
                  Vofa_WorkModeName(MotorControl_GetWorkMode()),
                  (unsigned int)mstat->active_mask,
                  (unsigned long)Can_App_GetTxCount());
    }
    else
    {
      Vofa_Printf("ERR CANSTOP status=%u\r\n", (unsigned int)status);
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    return;
  }

  if (strcmp(cmd, "CANZERO") == 0)
  {
    uint8_t tentacle = MotorControl_GetSelectedTentacle();
    uint8_t board_id = MotorControl_GetSelectedBoardId();
    const Can_App_G4Status_t *can_status = Vofa_GetFreshSelectedG4Status();
    W25Q64_App_TentacleState_t saved_state;
    int32_t zero_q[3] = {0, 0, 0};
    uint8_t status;
    uint8_t restore_status;

    if (can_status == NULL)
    {
      Vofa_Printf("ERR CANZERO CAN_STALE\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    if ((can_status->pwm_started_mask != 0U) || (can_status->position_pending != 0U))
    {
      Vofa_Printf("ERR CANZERO BUSY pwm=%u pending=%u\r\n",
                  (unsigned int)can_status->pwm_started_mask,
                  (unsigned int)can_status->position_pending);
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    /*
     * CANZERO语义：当前物理姿态定义为新的0点。
     * H7保存HOME=0、ACTUAL=0，并通过RESTORE 0让G4当前姿态对应0。
     */
    status = W25Q64_App_SaveTentacleState(tentacle,
                                          board_id,
                                          zero_q,
                                          zero_q,
                                          &saved_state);
    if (status == W25Q64_APP_OK)
    {
      /* HOME和ACTUAL已原子保存，成功后才修改RAM并要求G4重建坐标。 */
      MotorControl_SetNeutralQ(zero_q[0], zero_q[1], zero_q[2]);
      MotorControl_StartRestoreConfirmQ(zero_q[0], zero_q[1], zero_q[2]);
      restore_status = Can_App_SendRestoreActualQToBoard(board_id,
                                                         zero_q[0],
                                                         zero_q[1],
                                                         zero_q[2]);
      Vofa_Printf("OK CANZERO tentacle=%u board=%u home_seq=%lu actual_seq=%lu home=0,0,0 restore_tx=%u\r\n",
                  (unsigned int)tentacle,
                  (unsigned int)board_id,
                  (unsigned long)saved_state.seq,
                  (unsigned long)saved_state.seq,
                  (unsigned int)restore_status);
    }
    else
    {
      Vofa_Printf("ERR CANZERO status=%u\r\n", (unsigned int)status);
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    return;
  }

  if (strcmp(cmd, "CANHOME") == 0)
  {
    uint8_t status;

    if ((MotorControl_GetWorkMode() == MOTOR_CONTROL_WORK_SOLO) &&
        (Vofa_GetFreshSelectedG4Status() == NULL))
    {
      Vofa_Printf("ERR CANHOME CAN_STALE\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    status = MotorControl_HomeActive();

    if (status == 0U)
    {
      const MotorControl_Status_t *mstat = MotorControl_GetStatus();
      Vofa_Printf("OK CANHOME mode=%s mask=0x%02X status=0 tx=%lu\r\n",
                  Vofa_WorkModeName(MotorControl_GetWorkMode()),
                  (unsigned int)mstat->active_mask,
                  (unsigned long)Can_App_GetTxCount());
    }
    else if (status == 2U)
    {
      Vofa_Printf("ERR CANHOME RESTORE_PENDING\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    else if (status == 3U)
    {
      Vofa_Printf("ERR CANHOME CAN_STALE\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    else
    {
      Vofa_Printf("ERR CANHOME status=%u\r\n", (unsigned int)status);
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    return;
  }

  if ((strcmp(cmd, "CANPOS") == 0) || (strcmp(cmd, "CANREL") == 0) || (strcmp(cmd, "CANABS") == 0))
  {
    char *arg3;
    int32_t q0;
    int32_t q1;
    int32_t q2;
    uint8_t status;
    uint8_t is_abs;
    const MotorControl_Status_t *mstat;
    int32_t tx0;
    int32_t tx1;
    int32_t tx2;
    uint8_t board_id = MotorControl_GetSelectedBoardId();

    arg1 = strtok(NULL, " \t");
    arg2 = strtok(NULL, " \t");
    arg3 = strtok(NULL, " \t");
    if ((arg1 == NULL) || (arg2 == NULL) || (arg3 == NULL))
    {
      Vofa_Printf("ERR %s\r\n", cmd);
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    q0 = (int32_t)strtol(arg1, NULL, 10);
    q1 = (int32_t)strtol(arg2, NULL, 10);
    q2 = (int32_t)strtol(arg3, NULL, 10);

    is_abs = (strcmp(cmd, "CANABS") == 0) ? 1U : 0U;
    if (is_abs != 0U)
    {
      if (MotorControl_IsRestorePending() != 0U)
      {
        Vofa_Printf("ERR %s RESTORE_PENDING\r\n", cmd);
        g_state.error_flag = 1U;
        Vofa_RefreshLeds();
        return;
      }

      mstat = MotorControl_GetStatus();
      tx0 = mstat->neutral_q[0] + q0;
      tx1 = mstat->neutral_q[1] + q1;
      tx2 = mstat->neutral_q[2] + q2;
      status = Can_App_SendMotorTargetsAbsResetQToBoard(board_id, tx0, tx1, tx2);
    }
    else
    {
      tx0 = q0;
      tx1 = q1;
      tx2 = q2;
      status = MotorControl_SendRelativeActive(q0, q1, q2);
    }
    if (status == 0U)
    {
      if (is_abs != 0U)
      {
        Vofa_Printf("OK %s board=%u home_offset=%ld,%ld,%ld target=%ld,%ld,%ld tx=%lu\r\n",
                    cmd,
                    (unsigned int)board_id,
                    (long)q0, (long)q1, (long)q2,
                    (long)tx0, (long)tx1, (long)tx2,
                    (unsigned long)Can_App_GetTxCount());
      }
      else
      {
        mstat = MotorControl_GetStatus();
        Vofa_Printf("OK %s mask=0x%02X mirror=%u line_in=%ld,%ld,%ld tx=%lu\r\n",
                    cmd,
                    (unsigned int)mstat->active_mask,
                    (unsigned int)mstat->mirror_enable,
                    (long)q0, (long)q1, (long)q2,
                    (unsigned long)Can_App_GetTxCount());
      }
    }
    else
    {
      Vofa_Printf("ERR %s status=%u\r\n", cmd, (unsigned int)status);
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    return;
  }

  if ((strcmp(cmd, "MSRC") == 0) || (strcmp(cmd, "MSRC?") == 0))
  {
    const MotorControl_Status_t *mstat = MotorControl_GetStatus();
    Act_StreamStatus_t stream_status;
    ActPolicyApp_Result_t act_result;
    uint8_t source;
    uint8_t status;

    arg1 = strtok(NULL, " \t");
    if ((strcmp(cmd, "MSRC?") == 0) || (arg1 == NULL))
    {
      Vofa_Printf("MSRC,%s,%u\r\n",
                  Vofa_ControlSourceName(mstat->control_source),
                  (unsigned int)mstat->enable);
      return;
    }

    if (strcmp(arg1, "AI") == 0)
    {
      source = MOTOR_CONTROL_SOURCE_AI;
    }
    else if (strcmp(arg1, "PC") == 0)
    {
      source = MOTOR_CONTROL_SOURCE_PC;
    }
    else if (strcmp(arg1, "ACT") == 0)
    {
      source = MOTOR_CONTROL_SOURCE_ACT;
    }
    else
    {
      Vofa_Printf("ERR MSRC ARG\r\n");
      return;
    }

    /* USART1采样或示教期间禁止UART7切换运动来源。 */
    Act_Stream_GetStatus(&stream_status);
    if ((stream_status.control_enabled != 0U) ||
        (stream_status.capture_state == ACT_STREAM_CAPTURE_RUNNING))
    {
      Vofa_Printf("ERR MSRC ACT_STREAM_BUSY\r\n");
      return;
    }

    /* 离开ACT前先退出策略状态机；恢复后必须由用户重新启动。 */
    if ((source != MOTOR_CONTROL_SOURCE_ACT) &&
        (ActPolicyApp_GetRunState() != ACT_POLICY_APP_STATE_OFF))
    {
      act_result = ActPolicyApp_SetRun(0U, ACT_POLICY_APP_TENTACLE);
      if (act_result != ACT_POLICY_APP_RESULT_OK)
      {
        Vofa_Printf("ERR MSRC ACT_STOP result=%u\r\n",
                    (unsigned int)act_result);
        return;
      }
    }

    status = MotorControl_SetControlSource(source);
    mstat = MotorControl_GetStatus();
    if (status == 0U)
    {
      /* ACT不需要手势CNN；切回AI时自动恢复手势推理。 */
      if (source == MOTOR_CONTROL_SOURCE_ACT)
      {
        VL53_App_SetGestureInferEnable(0U);
      }
      else if (source == MOTOR_CONTROL_SOURCE_AI)
      {
        VL53_App_SetGestureInferEnable(1U);
      }

      Vofa_Printf("OK MSRC %s enable=%u\r\n",
                  Vofa_ControlSourceName(mstat->control_source),
                  (unsigned int)mstat->enable);
    }
    else
    {
      Vofa_Printf("ERR MSRC STOP status=%u\r\n", (unsigned int)status);
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    return;
  }

  if ((strcmp(cmd, "MCTRL") == 0) || (strcmp(cmd, "MCTRL?") == 0))
  {
    const MotorControl_Status_t *mstat = MotorControl_GetStatus();
    ActPolicyApp_RunState_t act_state;
    ActPolicyApp_Result_t act_result;
    Act_StreamStatus_t stream_status;
    uint8_t request_enable;

    act_state = ActPolicyApp_GetRunState();
    arg1 = strtok(NULL, " \t");
    if (strcmp(cmd, "MCTRL?") == 0)
    {
      request_enable =
        ((act_state == ACT_POLICY_APP_STATE_WARMUP) ||
         (act_state == ACT_POLICY_APP_STATE_RUNNING)) ? 1U : 0U;
      Vofa_Printf("MCTRL,%u\r\n",
                  (unsigned int)
                  ((mstat->control_source == MOTOR_CONTROL_SOURCE_ACT) ?
                   request_enable : mstat->enable));
      return;
    }

    if (arg1 == NULL)
    {
      if (mstat->control_source == MOTOR_CONTROL_SOURCE_ACT)
      {
        request_enable =
          (act_state == ACT_POLICY_APP_STATE_OFF) ? 1U : 0U;
      }
      else
      {
        /* 兼容原有Space翻转按钮。 */
        request_enable = (mstat->enable == 0U) ? 1U : 0U;
      }
    }
    else if (strcmp(arg1, "0") == 0)
    {
      request_enable = 0U;
    }
    else if (strcmp(arg1, "1") == 0)
    {
      request_enable = 1U;
    }
    else
    {
      Vofa_Printf("ERR MCTRL ARG\r\n");
      return;
    }

    /* 开启运动时禁止与USART1采样或示教控制并行；停止始终允许。 */
    Act_Stream_GetStatus(&stream_status);
    if ((request_enable != 0U) &&
        ((stream_status.control_enabled != 0U) ||
         (stream_status.capture_state == ACT_STREAM_CAPTURE_RUNNING)))
    {
      Vofa_Printf("ERR MCTRL ACT_STREAM_BUSY\r\n");
      return;
    }

    if (mstat->control_source == MOTOR_CONTROL_SOURCE_ACT)
    {
      act_result = ActPolicyApp_SetRun(request_enable,
                                      ACT_POLICY_APP_TENTACLE);
      mstat = MotorControl_GetStatus();
      if (act_result != ACT_POLICY_APP_RESULT_OK)
      {
        Vofa_Printf("ERR MCTRL ACT result=%u state=%u\r\n",
                    (unsigned int)act_result,
                    (unsigned int)ActPolicyApp_GetRunState());
        return;
      }

      Vofa_Printf("OK MCTRL %u mode=ACT mask=0x%02X\r\n",
                  (unsigned int)request_enable,
                  (unsigned int)mstat->active_mask);
      return;
    }

    MotorControl_SetEnable(request_enable);
    mstat = MotorControl_GetStatus();
    if ((request_enable != 0U) && (mstat->enable == 0U))
    {
      MotorControl_Summary_t summary;

      MotorControl_GetSummary(&summary);
      Vofa_Printf("ERR MCTRL %s online=0x%02X ready=0x%02X fault=0x%02X\r\n",
                  Vofa_BlockReasonName(mstat->block_reason),
                  (unsigned int)summary.online_mask,
                  (unsigned int)summary.ready_mask,
                  (unsigned int)summary.fault_mask);
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    Vofa_Printf("OK MCTRL %u mode=%s mask=0x%02X mirror=%u\r\n",
                (unsigned int)mstat->enable,
                Vofa_WorkModeName(MotorControl_GetWorkMode()),
                (unsigned int)mstat->active_mask,
                (unsigned int)mstat->mirror_enable);
    return;
  }

  if (strcmp(cmd, "MKEY") == 0)
  {
    (void)MotorControl_ToggleKeyMode();
    Vofa_PrintKeyInfo(MotorControl_GetKeyInfo(), "MKEY ");
    return;
  }

  if (strcmp(cmd, "MKEY?") == 0)
  {
    Vofa_PrintKeyInfo(MotorControl_GetKeyInfo(), "MKEY ");
    return;
  }

  if (strcmp(cmd, "MKEYCLR") == 0)
  {
    MotorControl_ClearKeySelection();
    Vofa_PrintKeyInfo(MotorControl_GetKeyInfo(), "MKEYCLR ");
    return;
  }

  if (strcmp(cmd, "MCOOP") == 0)
  {
    int32_t x;
    uint8_t status;

    arg1 = strtok(NULL, " \t");
    arg2 = strtok(NULL, " \t");
    if ((arg1 == NULL) || (arg2 == NULL))
    {
      Vofa_Printf("ERR MCOOP usage: MCOOP INIT x | MCOOP Bxx x\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    x = strtol(arg2, NULL, 10);
    if (strcmp(arg1, "INIT") == 0)
    {
      status = MotorControl_RunCoopInit((int16_t)x);
      if (status == 0U)
      {
        g_state.error_flag = 0U;
        Vofa_Printf("OK MCOOP INIT x=%ld\r\n", (long)x);
      }
      else
      {
        Vofa_Printf("ERR MCOOP INIT status=%u\r\n", (unsigned int)status);
        g_state.error_flag = 1U;
      }
      Vofa_RefreshLeds();
      return;
    }

    if ((arg1[0] == 'B') && (arg1[1] != '\0'))
    {
      int32_t bend = strtol(&arg1[1], NULL, 10);

      if ((bend < 0) || (bend > 100))
      {
        Vofa_Printf("ERR MCOOP_BEND\r\n");
        g_state.error_flag = 1U;
        Vofa_RefreshLeds();
        return;
      }

      status = MotorControl_RunCoopBend((uint8_t)bend, (int16_t)x);
      if (status == 0U)
      {
        g_state.error_flag = 0U;
        Vofa_Printf("OK MCOOP B%ld x=%ld\r\n", (long)bend, (long)x);
      }
      else
      {
        Vofa_Printf("ERR MCOOP B%ld status=%u\r\n", (long)bend, (unsigned int)status);
        g_state.error_flag = 1U;
      }
      Vofa_RefreshLeds();
      return;
    }

    Vofa_Printf("ERR MCOOP usage: MCOOP INIT x | MCOOP Bxx x\r\n");
    g_state.error_flag = 1U;
    Vofa_RefreshLeds();
    return;
  }

  if (strcmp(cmd, "MCFG") == 0)
  {
    const MotorControl_Config_t *cfg;
    MotorControl_Config_t new_cfg;
    char *arg3;
    char *arg4;
    char *arg5;
    char *arg6;
    uint8_t status;

    arg1 = strtok(NULL, " \t");
    if (arg1 == NULL)
    {
      cfg = MotorControl_GetConfig();
      Vofa_Printf("MCFG bend=%ld stiff=%ld dead=%ld full=%ld speed_base=%ld speed_max=%ld\r\n",
                  (long)cfg->bend_gain_q,
                  (long)cfg->stiff_gain_q,
                  (long)cfg->input_deadband,
                  (long)cfg->auto_dir_full,
                  (long)cfg->motor_speed_base_q,
                  (long)cfg->motor_speed_max_q);
      return;
    }

    arg2 = strtok(NULL, " \t");
    arg3 = strtok(NULL, " \t");
    arg4 = strtok(NULL, " \t");
    arg5 = strtok(NULL, " \t");
    arg6 = strtok(NULL, " \t");
    if ((arg2 == NULL) || (arg3 == NULL) || (arg4 == NULL) || (arg5 == NULL) || (arg6 == NULL))
    {
      Vofa_Printf("ERR MCFG\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    new_cfg.bend_gain_q = strtol(arg1, NULL, 10);
    new_cfg.stiff_gain_q = strtol(arg2, NULL, 10);
    new_cfg.input_deadband = strtol(arg3, NULL, 10);
    new_cfg.auto_dir_full = strtol(arg4, NULL, 10);
    new_cfg.motor_speed_base_q = strtol(arg5, NULL, 10);
    new_cfg.motor_speed_max_q = strtol(arg6, NULL, 10);

    status = MotorControl_SetConfig(&new_cfg, 1U);
    if (status == W25Q64_APP_OK)
    {
      g_state.error_flag = 0U;
      Vofa_Printf("OK MCFG bend=%ld stiff=%ld dead=%ld full=%ld speed_base=%ld speed_max=%ld\r\n",
                  (long)new_cfg.bend_gain_q,
                  (long)new_cfg.stiff_gain_q,
                  (long)new_cfg.input_deadband,
                  (long)new_cfg.auto_dir_full,
                  (long)new_cfg.motor_speed_base_q,
                  (long)new_cfg.motor_speed_max_q);
    }
    else
    {
      Vofa_Printf("ERR MCFG_SAVE status=%u\r\n", (unsigned int)status);
      g_state.error_flag = 1U;
    }
    Vofa_RefreshLeds();
    return;
  }

  if (strcmp(cmd, "MVIRT") == 0)
  {
    const MotorControl_Status_t *mstat = MotorControl_GetStatus();
    char *arg3;
    char *arg4;
    int32_t x;
    int32_t y;
    int32_t bend;
    int32_t stiff;

    arg1 = strtok(NULL, " \t");
    arg2 = strtok(NULL, " \t");
    arg3 = strtok(NULL, " \t");
    arg4 = strtok(NULL, " \t");
    if ((arg1 == NULL) || (arg2 == NULL) || (arg3 == NULL) || (arg4 == NULL))
    {
      Vofa_Printf("ERR MVIRT\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    x = strtol(arg1, NULL, 10);
    y = strtol(arg2, NULL, 10);
    bend = strtol(arg3, NULL, 10);
    stiff = strtol(arg4, NULL, 10);
    if ((bend < 0) || (bend > 100) || (stiff < 0) || (stiff > 100))
    {
      Vofa_Printf("ERR MVIRT_ARG\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    if (mstat->control_source != MOTOR_CONTROL_SOURCE_PC)
    {
      Vofa_Printf("ERR MVIRT SOURCE\r\n");
      return;
    }
    if (mstat->enable == 0U)
    {
      Vofa_Printf("ERR MVIRT DISABLED\r\n");
      return;
    }

    MotorControl_SetVirtualInput((int16_t)x, (int16_t)y, (uint8_t)bend, (uint8_t)stiff);
    mstat = MotorControl_GetStatus();
    if (mstat->block_reason != MOTOR_CONTROL_BLOCK_NONE)
    {
      Vofa_Printf("ERR MVIRT %s\r\n",
                  Vofa_BlockReasonName(mstat->block_reason));
      g_state.error_flag = 1U;
    }
    else if (mstat->last_tx_status == 0U)
    {
      g_state.error_flag = 0U;
      Vofa_Printf("OK MVIRT mode=%s %ld %ld %ld %ld\r\n",
                  Vofa_WorkModeName(MotorControl_GetWorkMode()),
                  (long)x,
                  (long)y,
                  (long)bend,
                  (long)stiff);
    }
    else
    {
      Vofa_Printf("ERR MVIRT status=%u\r\n", (unsigned int)mstat->last_tx_status);
      g_state.error_flag = 1U;
    }
    Vofa_RefreshLeds();
    return;
  }

  if (strcmp(cmd, "MSETZERO") == 0)
  {
    uint8_t tentacle = MotorControl_GetSelectedTentacle();
    uint8_t board_id = MotorControl_GetSelectedBoardId();
    char *arg3;
    int32_t q0;
    int32_t q1;
    int32_t q2;
    W25Q64_App_PositionRecord_t record;
    int32_t home_q[3];
    uint8_t status;

    arg1 = strtok(NULL, " \t");
    arg2 = strtok(NULL, " \t");
    arg3 = strtok(NULL, " \t");
    if ((arg1 == NULL) || (arg2 == NULL) || (arg3 == NULL))
    {
      Vofa_Printf("ERR MSETZERO\r\n");
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
      return;
    }

    q0 = strtol(arg1, NULL, 10);
    q1 = strtol(arg2, NULL, 10);
    q2 = strtol(arg3, NULL, 10);
    home_q[0] = q0;
    home_q[1] = q1;
    home_q[2] = q2;
    status = W25Q64_App_SavePositionRecordForTentacle(tentacle,
                                                      board_id,
                                                      W25Q64_APP_RECORD_TYPE_HOME,
                                                      home_q,
                                                      &record);
    if (status == W25Q64_APP_OK)
    {
      /* Flash确认成功后再更新RAM，避免保存失败时内存和掉电状态不一致。 */
      MotorControl_SetNeutralQ(q0, q1, q2);
      g_state.error_flag = 0U;
      Vofa_Printf("OK MSETZERO tentacle=%u board=%u seq=%lu home=%ld,%ld,%ld\r\n",
                  (unsigned int)tentacle,
                  (unsigned int)board_id,
                  (unsigned long)record.seq,
                  (long)record.position_q[0],
                  (long)record.position_q[1],
                  (long)record.position_q[2]);
      Vofa_RefreshLeds();
    }
    else
    {
      Vofa_Printf("ERR MSETZERO_SAVE status=%u\r\n", (unsigned int)status);
      g_state.error_flag = 1U;
      Vofa_RefreshLeds();
    }
    return;
  }

  if (strcmp(cmd, "MSTAT") == 0)
  {
    const MotorControl_Status_t *mstat = MotorControl_GetStatus();

    Vofa_Printf("M mode=%s tentacle=%u board=%u en=%u in=%d,%d bend=%u stiff=%u coop_stage=%u open=%u spd=%ld nq=%ld,%ld,%ld pull=%ld,%ld,%ld tq=%ld,%ld,%ld oq=%ld,%ld,%ld tx=%lu st=%u hand=%u dz=%u armed=%u ev=%u ec=%lu\r\n",
                 Vofa_WorkModeName(mstat->work_mode),
                 (unsigned int)mstat->selected_tentacle,
                 (unsigned int)mstat->selected_board_id,
                 (unsigned int)mstat->enable,
                 (int)mstat->input_x,
                 (int)mstat->input_y,
                 (unsigned int)mstat->bend_0_100,
                 (unsigned int)mstat->stiffness_0_100,
                 (unsigned int)mstat->coop_stage,
                 (unsigned int)mstat->coop_open_0_100,
                 (long)mstat->motor_speed_limit_q,
                 (long)mstat->neutral_q[0],
                 (long)mstat->neutral_q[1],
                (long)mstat->neutral_q[2],
                (long)mstat->pull_q[0],
                (long)mstat->pull_q[1],
                (long)mstat->pull_q[2],
                (long)mstat->target_q[0],
                (long)mstat->target_q[1],
                (long)mstat->target_q[2],
                (long)mstat->output_q[0],
                (long)mstat->output_q[1],
                (long)mstat->output_q[2],
                (unsigned long)mstat->tx_count,
                (unsigned int)mstat->last_tx_status,
                (unsigned int)mstat->hand_present,
                (unsigned int)mstat->distance_zone,
                (unsigned int)mstat->select_armed,
                (unsigned int)mstat->select_event,
                (unsigned long)mstat->select_count);
    return;
  }

  if (strcmp(cmd, "STATUS") == 0)
  {
    Vofa_Printf("MODE=%u SMODE=%s WAIT=%u REC=%u REM=%lu CFG=%lu/%lu LABEL=%u ERR=%u\r\n",
                (unsigned int)VL53_App_GetMode(),
                (g_state.sample_mode == VOFA_SAMPLE_MODE_BALANCED) ? "BAL" : "NORMAL",
                (unsigned int)g_state.sample_waiting,
                (unsigned int)g_state.record_active,
                (unsigned long)g_state.record_remaining,
                (unsigned long)g_state.record_count_cfg,
                (unsigned long)g_state.record_interval_cfg,
                (unsigned int)g_state.label_score,
                (unsigned int)g_state.error_flag);
    if (g_state.sample_mode == VOFA_SAMPLE_MODE_BALANCED)
    {
      Vofa_PrintBalCounts();
    }
    return;
  }

  if (strcmp(cmd, "VLSTAT") == 0)
  {
    Vofa_PrintVl53Status();
    return;
  }

  if (strcmp(cmd, "VLSCAN") == 0)
  {
    Vofa_ScanI2c2();
    return;
  }

  Vofa_Printf("ERR CMD\r\n");
  g_state.error_flag = 1U;
  Vofa_RefreshLeds();
}

/**
  * @brief  使VOFA串口DMA接收缓冲区的DCache失效。
  * @param  offset 起始偏移。
  * @param  len 需要刷新的字节数。
  *
  * 说明：VOFA RX DMA 会直接写 RAM，CPU 读之前必须 invalidate DCache，
  * 否则可能读到旧缓存数据。g_rx_buf 已 32 字节对齐且长度为 32 的倍数。
  */
static void Vofa_InvalidateRxDCache(uint16_t offset, uint16_t len)
{
  uint32_t start_addr;
  uint32_t end_addr;

  if (len == 0U)
  {
    return;
  }

  start_addr = ((uint32_t)&g_rx_buf[offset]) & ~(VOFA_CACHE_LINE_SIZE - 1U);
  end_addr = ((uint32_t)&g_rx_buf[offset + len] + VOFA_CACHE_LINE_SIZE - 1U) &
             ~(VOFA_CACHE_LINE_SIZE - 1U);

  SCB_InvalidateDCache_by_Addr((uint32_t *)start_addr, (int32_t)(end_addr - start_addr));
}

/**
  * @brief  启动UART循环DMA接收。
  * @return 0成功；1表示DMA句柄无效；2表示HAL启动失败。
  * @note   重新启动时会丢弃尚未收到换行符的半条命令。
  */
static uint8_t Vofa_StartRxDma(void)
{
  if (VOFA_UART_HANDLE.hdmarx == NULL)
  {
    return 1U;
  }

  Vofa_InvalidateRxDCache(0U, VOFA_RX_BUF_SIZE);
  g_rx_tail = 0U;
  g_line_len = 0U;

  if (HAL_UART_Receive_DMA(&VOFA_UART_HANDLE,
                           g_rx_buf,
                           VOFA_RX_BUF_SIZE) != HAL_OK)
  {
    return 2U;
  }

  /* 当前使用主循环读取NDTR，不需要半满和传输完成中断。 */
  __HAL_DMA_DISABLE_IT(VOFA_UART_HANDLE.hdmarx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(VOFA_UART_HANDLE.hdmarx, DMA_IT_TC);
  return 0U;
}

/**
  * @brief  根据 DMA 当前写入位置刷新新增接收区间的 DCache。
  * @param  rx_pos DMA 当前写入位置。
  */
static void Vofa_InvalidateNewRxData(uint16_t rx_pos)
{
  if (rx_pos == g_rx_tail)
  {
    return;
  }

  if (rx_pos > g_rx_tail)
  {
    Vofa_InvalidateRxDCache(g_rx_tail, (uint16_t)(rx_pos - g_rx_tail));
  }
  else
  {
    Vofa_InvalidateRxDCache(g_rx_tail, (uint16_t)(VOFA_RX_BUF_SIZE - g_rx_tail));
    Vofa_InvalidateRxDCache(0U, rx_pos);
  }
}

/**
  * @brief  处理VOFA串口DMA环形缓冲区中的新增字符。
  * @param  无。
  */
static void Vofa_UartTask(void)
{
  uint16_t rx_pos;
  uint32_t now_ms;

  if ((VOFA_UART_HANDLE.hdmarx == NULL) || (HAL_UART_GetState(&VOFA_UART_HANDLE) == HAL_UART_STATE_RESET))
  {
    return;
  }

  /* ORE或DMA错误可能终止RX，但不会影响TX；正常接收时始终为BUSY_RX。 */
  if (VOFA_UART_HANDLE.RxState != HAL_UART_STATE_BUSY_RX)
  {
    now_ms = HAL_GetTick();
    if ((now_ms - g_rx_last_retry_ms) >= VOFA_RX_RETRY_MS)
    {
      g_rx_last_retry_ms = now_ms;
      (void)HAL_UART_AbortReceive(&VOFA_UART_HANDLE);
      (void)Vofa_StartRxDma();
    }
    return;
  }

  rx_pos = (uint16_t)(VOFA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(VOFA_UART_HANDLE.hdmarx));

  /* DMA循环模式在重装瞬间可能返回缓冲区长度，统一转换为合法下标0。 */
  if (rx_pos >= VOFA_RX_BUF_SIZE)
  {
    rx_pos = 0U;
  }

  Vofa_InvalidateNewRxData(rx_pos);

  while (g_rx_tail != rx_pos)
  {
    char ch = (char)g_rx_buf[g_rx_tail++];
    if (g_rx_tail >= VOFA_RX_BUF_SIZE)
    {
      g_rx_tail = 0U;
    }

    if ((ch == '\r') || (ch == '\n'))
    {
      if (g_line_len > 0U)
      {
        g_line_buf[g_line_len] = '\0';
        Vofa_HandleLine(g_line_buf);
        g_line_len = 0U;
      }
      continue;
    }

    if (g_line_len < (VOFA_LINE_BUF_SIZE - 1U))
    {
      g_line_buf[g_line_len++] = ch;
    }
    else
    {
      g_line_len = 0U;
      g_state.error_flag = 1U;
      Vofa_Printf("ERR LINE_TOO_LONG\r\n");
      Vofa_RefreshLeds();
    }
  }
}

/**
  * @brief  处理采样完成后的结果并推进连续采样状态机。
  * @param  无。
  */
static void Vofa_SampleTask(void)
{
  uint8_t sample_status;
  uint32_t now_ms;

  if (g_state.sample_waiting != 0U)
  {
    now_ms = HAL_GetTick();

    if (VL53_App_ConsumeSampleResult(&sample_status) != 0U)
    {
      g_state.sample_waiting = 0U;
      g_state.sample_deadline_ms = 0U;

      if (sample_status == 0U)
      {
        VL53_App_SampleStats_t stats;

        g_state.error_flag = 0U;
        g_state.record_retry_count = 0U;
        if ((g_state.record_active != 0U) && (g_state.record_remaining > 0U))
        {
          if (g_state.sample_mode == VOFA_SAMPLE_MODE_BALANCED)
          {
            if (g_state.bal_pending_valid != 0U)
            {
              uint8_t bin = g_state.bal_pending_bin;

              if ((bin < g_state.bal_bin_num) &&
                  (g_state.bal_counts[bin] < g_state.bal_target_count))
              {
                g_state.bal_counts[bin]++;
                g_state.bal_saved_total++;
              }
              g_state.bal_pending_valid = 0U;
              g_state.record_remaining = Vofa_BalTotalTarget() - g_state.bal_saved_total;

              Vofa_Printf("BAL OK label=%u z=%u bin=%u count=%lu/%lu total=%lu/%lu\r\n",
                          (unsigned int)g_state.label_score,
                          (unsigned int)g_state.bal_pending_z_mm,
                          (unsigned int)Vofa_BalBinCenter(bin),
                          (unsigned long)g_state.bal_counts[bin],
                          (unsigned long)g_state.bal_target_count,
                          (unsigned long)g_state.bal_saved_total,
                          (unsigned long)Vofa_BalTotalTarget());
              Vofa_PrintBalCounts();
            }

            if (Vofa_BalIsDone() != 0U)
            {
              Vofa_ForceStopSamplingToInfer();
              Vofa_Printf("BAL DONE label=%u total=%lu/%lu\r\n",
                          (unsigned int)g_state.label_score,
                          (unsigned long)g_state.bal_saved_total,
                          (unsigned long)Vofa_BalTotalTarget());
              Vofa_PrintBalCounts();
            }
            else
            {
              g_state.record_next_ms = now_ms + g_state.record_interval_ms;
            }
          }
          else
          {
            uint16_t center_z = 0U;

            g_state.normal_saved_count++;
            g_state.record_remaining--;
            if (VL53_App_GetSampleStats(&stats) == 0U)
            {
              center_z = stats.center_z_mm;
            }

            Vofa_Printf("NORMAL OK label=%u saved=%lu/%lu z=%u\r\n",
                        (unsigned int)g_state.label_score,
                        (unsigned long)g_state.normal_saved_count,
                        (unsigned long)g_state.record_count_cfg,
                        (unsigned int)center_z);

            if (g_state.record_remaining == 0U)
            {
              Vofa_ForceStopSamplingToInfer();
              Vofa_Printf("NORMAL DONE label=%u saved=%lu\r\n",
                          (unsigned int)g_state.label_score,
                          (unsigned long)g_state.normal_saved_count);
            }
            else
            {
              g_state.record_next_ms = now_ms + g_state.record_interval_ms;
            }
          }
        }
        else
        {
          /* 单次 SAMPLE 成功后关闭 CSV，避免文件长时间保持打开。 */
          Vofa_ForceStopSamplingToInfer();
        }
      }
      else
      {
        g_state.error_flag = 1U;
        g_state.bal_pending_valid = 0U;
        if ((g_state.record_active != 0U) &&
            (g_state.record_remaining > 0U) &&
            (g_state.record_retry_count < VOFA_SAMPLE_MAX_RETRY))
        {
          g_state.record_retry_count++;
          g_state.record_next_ms = now_ms + g_state.record_interval_ms;
          Vofa_Printf("WARN SAMPLE_RETRY %u/%u\r\n",
                      (unsigned int)g_state.record_retry_count,
                      (unsigned int)VOFA_SAMPLE_MAX_RETRY);
        }
        else
        {
          Vofa_ForceStopSamplingToInfer();
          Vofa_Printf("ERR SAMPLE_FAIL\r\n");
        }
      }

      Vofa_RefreshLeds();
    }
    else if ((int32_t)(now_ms - g_state.sample_deadline_ms) >= 0)
    {
      g_state.sample_waiting = 0U;
      g_state.sample_deadline_ms = 0U;
      g_state.error_flag = 1U;
      g_state.bal_pending_valid = 0U;

      /* 超时后退出采样模式，清掉 VL53 内部挂起采样，避免 LED2 长亮。 */
      (void)VL53_App_SetMode(VL53_APP_MODE_INFER);

      if ((g_state.record_active != 0U) &&
          (g_state.record_remaining > 0U) &&
          (g_state.record_retry_count < VOFA_SAMPLE_MAX_RETRY))
      {
        g_state.record_retry_count++;
        g_state.record_next_ms = now_ms + g_state.record_interval_ms;
        Vofa_Printf("WARN SAMPLE_TIMEOUT_RETRY %u/%u\r\n",
                    (unsigned int)g_state.record_retry_count,
                    (unsigned int)VOFA_SAMPLE_MAX_RETRY);
      }
      else
      {
        Vofa_ForceStopSamplingToInfer();
        Vofa_Printf("ERR SAMPLE_TIMEOUT\r\n");
      }
      Vofa_RefreshLeds();
    }
  }

  if ((g_state.sample_waiting == 0U) &&
      (g_state.record_active != 0U) &&
      (g_state.record_remaining > 0U))
  {
    now_ms = HAL_GetTick();
    if ((int32_t)(now_ms - g_state.record_next_ms) >= 0)
    {
      if (Vofa_StartRecordSample(now_ms) != 0U)
      {
        Vofa_ForceStopSamplingToInfer();
        Vofa_Printf("ERR BUSY\r\n");
        g_state.error_flag = 1U;
        Vofa_RefreshLeds();
      }
    }
  }
}

void Vofa_Init(void)
{
  memset(&g_state, 0, sizeof(g_state));
  memset(g_stall_notice_mask, 0, sizeof(g_stall_notice_mask));
  g_state.label_score = VL53_APP_LABEL_NONE;
  VL53_App_SetLabelScore(g_state.label_score);
  g_state.sample_mode = VOFA_SAMPLE_MODE_NORMAL;
  g_state.record_count_cfg = VOFA_DEFAULT_REC_COUNT;
  g_state.record_interval_cfg = VOFA_DEFAULT_REC_MS;
  g_rx_tail = 0U;
  g_line_len = 0U;
  g_rx_last_retry_ms = 0U;

  if (Vofa_StartRxDma() != 0U)
  {
    Vofa_Printf("ERR UART_RX_DMA\r\n");
  }

  Vofa_RefreshLeds();
  Vofa_Printf("READY %s\r\n", VOFA_UART_NAME);
  Vofa_PrintHelp();
}

void Vofa_Task(void)
{
  Vofa_UartTask();
  Vofa_SampleTask();
  Vofa_StopNoticeTask();
  Vofa_KeyNoticeTask();
  Vofa_KeyDebugTask();
  Vofa_StallNoticeTask();
}

#if (VOFA_TX_DMA_ENABLE != 0U)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    Act_Stream_OnUartTxComplete(huart);
    return;
  }

  if (huart != &VOFA_UART_HANDLE)
  {
    return;
  }

  if (g_tx_count > 0U)
  {
    g_tx_tail = (uint8_t)((g_tx_tail + 1U) % VOFA_TX_QUEUE_DEPTH);
    g_tx_count--;
  }
  g_tx_busy = 0U;
  Vofa_TxTryStart();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    Act_Stream_OnUartError(huart);
    return;
  }

  if (huart != &VOFA_UART_HANDLE)
  {
    return;
  }

  if (g_tx_count > 0U)
  {
    g_tx_tail = (uint8_t)((g_tx_tail + 1U) % VOFA_TX_QUEUE_DEPTH);
    g_tx_count--;
  }
  g_tx_busy = 0U;
  g_tx_drop_count++;
  Vofa_TxTryStart();
}
#endif
