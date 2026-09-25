#ifndef ADC_APP_H
#define ADC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "main.h"
#include "adc.h"

//******************************************ADC应用层基础参数*******************************************/
#define ADC_APP_MOTOR_ADC_NUM       3U      // 三个电机电流采样ADC：ADC1/ADC3/ADC4
#define ADC_APP_PHASE_NUM           3U      // 每个电机三相电流采样：SOA/SOB/SOC
#define ADC_APP_TEMP_ADC_NUM        2U      // ADC2当前用于温度/辅助模拟量采样

//******************************************ADC应用层索引枚举*******************************************/
typedef enum
{
    ADC_APP_MOTOR_ADC1 = 0,                  // 电机电流采样组1：ADC1，当前由TIM20_TRGO2触发
    ADC_APP_MOTOR_ADC3,                      // 电机电流采样组2：ADC3，当前由TIM8_TRGO2触发
    ADC_APP_MOTOR_ADC4                       // 电机电流采样组3：ADC4，当前由TIM1_TRGO2触发
} adc_app_motor_adc_t;

typedef enum
{
    ADC_APP_PHASE_A = 0,                     // 三相采样Rank1，对应当前ADC组的SOA/相A
    ADC_APP_PHASE_B,                         // 三相采样Rank2，对应当前ADC组的SOB/相B
    ADC_APP_PHASE_C                          // 三相采样Rank3，对应当前ADC组的SOC/相C
} adc_app_phase_t;

typedef enum
{
    ADC_APP_TEMP_CH_0 = 0,                   // ADC2 Rank1，当前PA0/ADC2_IN1
    ADC_APP_TEMP_CH_1                        // ADC2 Rank2，当前PB11/ADC2_IN14
} adc_app_temp_ch_t;

//******************************************ADC应用层接口*******************************************/
void ADC_App_Init(void);                                                     // 初始化ADC应用层状态和DMA完成标志
HAL_StatusTypeDef ADC_App_CalibrateMotorAdc(void);                           // 校准ADC1/ADC3/ADC4电流采样ADC，必须在启动DMA前调用
HAL_StatusTypeDef ADC_App_StartMotorCurrentDma(void);                        // 启动ADC1/ADC3/ADC4三组电机电流DMA采样，采样时刻仍由对应定时器触发
HAL_StatusTypeDef ADC_App_StartTempDma(void);                                // 启动ADC2温度/辅助模拟量DMA采样，当前由TIM6_TRGO触发

uint16_t ADC_App_GetMotorRaw(adc_app_motor_adc_t adc_id,
                             adc_app_phase_t phase);                         // 读取指定电机ADC组三相原始ADC值，返回值范围通常为0~4095
uint8_t ADC_App_GetMotorRaw3FastUnchecked(uint8_t adc_id,
                                          uint16_t *raw_a,
                                          uint16_t *raw_b,
                                          uint16_t *raw_c,
                                          uint32_t *update_count);             // 快环按已绑定ADC id读取三相raw，不做id合法性检查
uint8_t ADC_App_GetMotorRaw3FastAdc1(uint16_t *raw_a,
                                     uint16_t *raw_b,
                                     uint16_t *raw_c,
                                     uint32_t *update_count);                 // 快环读取ADC1一帧三相raw和帧计数，当前用于TIM20电流环减负
uint32_t ADC_App_GetMotorUpdateCount(adc_app_motor_adc_t adc_id);             // 读取指定电机ADC组稳定帧更新计数，用于判断是否有新采样帧
uint8_t ADC_App_GetMotorAdcCalibErrorMask(void);                              // 获取ADC1/ADC3/ADC4自校准错误掩码，bit0/1/2对应ADC1/3/4
uint8_t ADC_App_GetMotorAdcStartErrorMask(void);                              // 获取ADC1/ADC3/ADC4 DMA启动错误掩码，bit0/1/2对应ADC1/3/4
uint16_t ADC_App_GetTempRaw(adc_app_temp_ch_t ch);                            // 读取ADC2温度/辅助模拟量原始ADC值，返回值范围通常为0~4095

uint8_t ADC_App_IsMotorDmaDone(adc_app_motor_adc_t adc_id);                   // 查询指定电机ADC组DMA是否完成一次序列搬运
void ADC_App_ClearMotorDmaDone(adc_app_motor_adc_t adc_id);                   // 清除指定电机ADC组DMA完成标志，通常在读取raw后调用

uint8_t ADC_App_IsTempDmaDone(void);                                          // 查询ADC2 DMA是否完成一次序列搬运
void ADC_App_ClearTempDmaDone(void);                                          // 清除ADC2 DMA完成标志，通常在读取温度/辅助raw后调用

void ADC_App_ConvCpltCallback(ADC_HandleTypeDef *hadc);                       // 在HAL_ADC_ConvCpltCallback中转发调用，只置位完成标志
void ADC_App_ErrorCallback(ADC_HandleTypeDef *hadc);                          // 在HAL_ADC_ErrorCallback中转发调用，只记录错误状态

#ifdef __cplusplus
}
#endif

#endif /* ADC_APP_H */
