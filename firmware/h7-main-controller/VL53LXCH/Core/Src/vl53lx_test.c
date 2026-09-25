/*********************************************************
 * vl53lx_test.c
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
#include <stdio.h>
#include <stdlib.h>
#include "vl53lx_test.h"
#include "examples.h"

/* global variables		   ---------------------------------------------------*/
VL53LMZ_Configuration g_Config;

int8_t vl53lx_init(void)
{
	uint8_t isAlive = 0;
	int status = 0;
	g_Config.platform.address = 0x52;
	/* (Optional) Check if there is a VL53LMZ sensor connected */
	status = vl53lmz_is_alive(&g_Config, &isAlive);
	if (!isAlive || status)
	{
		printf("VL53LMZ not detected at requested address\r\n");
		return -1;
	}
	else
	{
		printf("VL53LMZ found,wait for update firmware\r\n");
	}

	// Initialise the sensor (SW reset sequence + sensor boot)
	status = vl53lmz_init(&g_Config);
	if (status != VL53LMZ_STATUS_OK)
	{
		printf("vl53lmz_init failed : %d\r\n", status);
		return -1;
	}

	printf("Module type (0 for VL53L5, 1 for VL53L7, 2 for VL53L8) : %u\r\n", g_Config.module_type);
	if ((g_Config.module_type != VL53LMZ_MODULE_TYPE_L7) && (g_Config.module_type != VL53LMZ_MODULE_TYPE_L8))
	{
		printf("Module type %u not supported !\r\n", g_Config.module_type);
		return -1;
	}
	return 0;
}



uint8_t vl53lx_test(uint8_t test_id)
{
	uint8_t status;
	switch (test_id)
	{
		case 1:
			status=example1(&g_Config);
			break;
		case 2:
			status=example2(&g_Config);
			break;
		case 3:
			status=example3(&g_Config);
			break;
		case 4:
			status=example4(&g_Config);
			break;
		case 5:
			status=example5(&g_Config);
			break;
		case 6:
			status=example6(&g_Config);
			break;
		case 7:
			status=example7(&g_Config);
			break;
		case 8:
			status=example8(&g_Config);
			break;
		case 9:
			status=example9(&g_Config);
			break;
		case 10:
			status=example10(&g_Config);
			break;
		case 11:
			status=example11(&g_Config);
			break;
		case 12:
			status=example12(&g_Config);
			break;
		case 13:
			status= example13(&g_Config);
			break;
		default:
			break;
	}
	return status;
}
