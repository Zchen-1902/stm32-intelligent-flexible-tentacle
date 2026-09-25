/*********************************************************
 * platform.c
 * Copyright (c) 2012 - 2024 RTrobot Inc.
 *
 * Unless otherwise stated, the use of this software is subject to the following conditions:
 * 1. Any form of redistribution must include the original copyright notice and the following disclaimer.
 * 2. Not for commercial use without explicit written permission, including but not limited to sales, licensing, or commercial support.
 * 3. Any modifications to this software must be clearly marked with attribution and documented in the modified files.
 * 4. If modifications are made, they must be clearly indicated in the modified files.
 * 5. Without explicit written permission, the names of the authors or original contributors may not be used to endorse or promote derived products.
 *
 * This software is provided "as is," without any warranties of any kind, express or implied, including but not limited to the warranties of merchantability or fitness for a particular purpose.
 * The authors are not liable for any direct, indirect, incidental, special, exemplary, or consequential damages arising in any way out of the use of this software.
 *********************************************************/


#include "platform.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t RdByte(VL53LMZ_Platform* p_platform, uint16_t RegisterAdress, uint8_t* p_value)
{
	uint8_t status = 0;
	uint8_t data_write[2];
	uint8_t data_read[1];

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	status = HAL_I2C_Master_Transmit(&hi2c1, p_platform->address, data_write, 2, 100);
	status = HAL_I2C_Master_Receive(&hi2c1, p_platform->address, data_read, 1, 100);
	*p_value = data_read[0];
	return status;
}

uint8_t WrByte(VL53LMZ_Platform* p_platform, uint16_t RegisterAdress, uint8_t value)
{
	uint8_t data_write[3];
	uint8_t status = 0;
	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	data_write[2] = value & 0xFF;

	status = HAL_I2C_Master_Transmit(&hi2c1, p_platform->address, data_write, 3, 100);
	return status;
}

uint8_t WrMulti(VL53LMZ_Platform* p_platform, uint16_t RegisterAdress, uint8_t* p_values, uint32_t size)
{
	uint8_t status;
	status = HAL_I2C_Mem_Write(&hi2c1, p_platform
		->address, RegisterAdress, I2C_MEMADD_SIZE_16BIT, p_values, size, 65535);
	return status;
}

uint8_t RdMulti(VL53LMZ_Platform* p_platform, uint16_t RegisterAdress, uint8_t* p_values, uint32_t size)
{
	uint8_t status;
	uint8_t data_write[2];
	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;

	status = HAL_I2C_Master_Transmit(&hi2c1, p_platform->address, data_write, 2, 10);
	status += HAL_I2C_Master_Receive(&hi2c1, p_platform->address, p_values, size, 400);

	return status;
}

void SwapBuffer(uint8_t* buffer, uint16_t size)
{
	uint32_t i, tmp;
	for (i = 0; i < size; i = i + 4)
	{
		tmp = (buffer[i] << 24) | (buffer[i + 1] << 16) | (buffer[i + 2] << 8) | (buffer[i + 3]);

		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t WaitMs(VL53LMZ_Platform* p_platform, uint32_t TimeMs)
{
	HAL_Delay(TimeMs);
	return 0;
}
