#ifndef VL53_PLATFORM_H
#define VL53_PLATFORM_H

#include <stdint.h>
#include <string.h>
#include "main.h"

#define VL53LMZ_NB_TARGET_PER_ZONE 1U
/* ACT不使用SPAD数量，关闭该输出可减少每帧I2C传输量。 */
#define VL53LMZ_DISABLE_NB_SPADS_ENABLED
/* ACT和手势模型均未使用运动检测块，关闭它以减少每帧I2C读取量。 */
#define VL53LMZ_DISABLE_MOTION_INDICATOR
/* 关闭 raw format，让 ULD API 输出真实单位；distance_mm 才是毫米值。 */
/* #define VL53LMZ_USE_RAW_FORMAT */

typedef struct
{
  uint16_t address;
} VL53LMZ_Platform;

uint8_t RdByte(VL53LMZ_Platform *p_platform, uint16_t reg, uint8_t *p_value);
uint8_t WrByte(VL53LMZ_Platform *p_platform, uint16_t reg, uint8_t value);
uint8_t RdMulti(VL53LMZ_Platform *p_platform, uint16_t reg, uint8_t *p_values, uint32_t size);
uint8_t WrMulti(VL53LMZ_Platform *p_platform, uint16_t reg, uint8_t *p_values, uint32_t size);
uint8_t SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t WaitMs(VL53LMZ_Platform *p_platform, uint32_t time_ms);

#endif
