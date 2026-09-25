/*********************************************************
 * Example_13_rtrobot_software.c
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
#include "vl53lmz_api.h"

extern VL53LMZ_Configuration g_Config;

int example13(VL53LMZ_Configuration *p_dev)
{
	/*********************************/
	/*   VL53LMZ ranging variables  */
	/*********************************/

	uint8_t 				status, isReady, i;
	uint32_t 				integration_time_ms;
	VL53LMZ_ResultsData 	Results;		/* Results data from VL53LMZ */

	/*********************************/
	/*        Set some params        */
	/*********************************/

	/* Set resolution in 8x8. WARNING : As others settings depend to this one, it must be the first to use.*/
	status = vl53lmz_set_resolution(p_dev, VL53LMZ_RESOLUTION_8X8);
	if(status)
	{
		printf("vl53lmz_set_resolution failed, status %u\r\n", status);
		return status;
	}
	uint8_t buf=0;
	vl53lmz_get_resolution(p_dev,&buf);
	printf("resolution:%d\r\n",buf);

	/* Set ranging frequency to 10Hz.
	 * Using 4x4, min frequency is 1Hz and max is 60Hz
	 * Using 8x8, min frequency is 1Hz and max is 15Hz
	 */
	status = vl53lmz_set_ranging_frequency_hz(p_dev, 15);
	if(status)
	{
		printf("vl53lmz_set_ranging_frequency_hz failed, status %u\r\n", status);
		return status;
	}

	/* Set target order to closest */
	status = vl53lmz_set_target_order(p_dev, VL53LMZ_TARGET_ORDER_CLOSEST);
	if(status)
	{
		printf("vl53lmz_set_target_order failed, status %u\r\n", status);
		return status;
	}

	/* Get current integration time */
	status = vl53lmz_get_integration_time_ms(p_dev, &integration_time_ms);
	if(status)
	{
		printf("vl53lmz_get_integration_time_ms failed, status %u\r\n", status);
		return status;
	}
	printf("Current integration time is : %d ms\r\n", integration_time_ms);


	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53lmz_start_ranging(p_dev);

	while(status == VL53LMZ_STATUS_OK)
	{
		status = vl53lmz_check_data_ready(p_dev, &isReady);

		if(isReady)
		{
			vl53lmz_get_ranging_data(p_dev, &Results);

			/* As the sensor is set in 8x8 mode, we have a total
			 * of 64 zones to print. For this example, only the data of
			 * first zone are print */
			for(i = 0; i < 64; i++)
			{
				printf("%d,",Results.distance_mm[VL53LMZ_NB_TARGET_PER_ZONE*i]);
			}
			printf("\r\n");
		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		WaitMs(&p_dev->platform, 5);
	}

	status = vl53lmz_stop_ranging(p_dev);
	return status;
}
