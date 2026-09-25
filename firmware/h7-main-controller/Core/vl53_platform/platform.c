#include "platform.h"
#include "i2c.h"

#define VL53_I2C_BYTE_TIMEOUT_MS   100U
#define VL53_I2C_MULTI_TIMEOUT_MS  500U
#define VL53_I2C_FW_TIMEOUT_MS     65535U
#define VL53_I2C_HANDLE            hi2c2

/**
  * @brief Convert HAL I2C status to the VL53 ULD platform status convention.
  * @param status HAL I2C return value.
  * @retval 0 on success, non-zero on failure.
  */
static uint8_t VL53_Platform_Status(HAL_StatusTypeDef status)
{
  return (status == HAL_OK) ? 0U : 255U;
}

uint8_t RdByte(VL53LMZ_Platform *p_platform, uint16_t reg, uint8_t *p_value)
{
  return VL53_Platform_Status(HAL_I2C_Mem_Read(&VL53_I2C_HANDLE,
                                               p_platform->address,
                                               reg,
                                               I2C_MEMADD_SIZE_16BIT,
                                               p_value,
                                               1U,
                                               VL53_I2C_BYTE_TIMEOUT_MS));
}

uint8_t WrByte(VL53LMZ_Platform *p_platform, uint16_t reg, uint8_t value)
{
  return VL53_Platform_Status(HAL_I2C_Mem_Write(&VL53_I2C_HANDLE,
                                                p_platform->address,
                                                reg,
                                                I2C_MEMADD_SIZE_16BIT,
                                                &value,
                                                1U,
                                                VL53_I2C_BYTE_TIMEOUT_MS));
}

uint8_t RdMulti(VL53LMZ_Platform *p_platform, uint16_t reg, uint8_t *p_values, uint32_t size)
{
  return VL53_Platform_Status(HAL_I2C_Mem_Read(&VL53_I2C_HANDLE,
                                               p_platform->address,
                                               reg,
                                               I2C_MEMADD_SIZE_16BIT,
                                               p_values,
                                               (uint16_t)size,
                                               VL53_I2C_MULTI_TIMEOUT_MS));
}

uint8_t WrMulti(VL53LMZ_Platform *p_platform, uint16_t reg, uint8_t *p_values, uint32_t size)
{
  return VL53_Platform_Status(HAL_I2C_Mem_Write(&VL53_I2C_HANDLE,
                                                p_platform->address,
                                                reg,
                                                I2C_MEMADD_SIZE_16BIT,
                                                p_values,
                                                (uint16_t)size,
                                                ((size > 1024U) ? VL53_I2C_FW_TIMEOUT_MS : VL53_I2C_MULTI_TIMEOUT_MS)));
}

uint8_t SwapBuffer(uint8_t *buffer, uint16_t size)
{
  uint32_t i;
  uint32_t tmp;

  for (i = 0U; i < size; i += 4U)
  {
    tmp = ((uint32_t)buffer[i] << 24) |
          ((uint32_t)buffer[i + 1U] << 16) |
          ((uint32_t)buffer[i + 2U] << 8) |
          ((uint32_t)buffer[i + 3U]);

    memcpy(&buffer[i], &tmp, 4U);
  }

  return 0U;
}

uint8_t WaitMs(VL53LMZ_Platform *p_platform, uint32_t time_ms)
{
  (void)p_platform;
  HAL_Delay(time_ms);
  return 0U;
}
