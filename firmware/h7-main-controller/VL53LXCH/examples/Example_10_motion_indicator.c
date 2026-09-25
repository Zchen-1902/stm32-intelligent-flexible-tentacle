/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/***********************************/
/*  VL53LMZ ULD motion indicator  */
/***********************************/
/*
* This example shows the VL53LMZ motion indicator capabilities.
* To use this example, user needs to be sure that macro
* VL53LMZ_DISABLE_MOTION_INDICATOR is NOT enabled (see file platform.h).
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53lmz_api.h"
#include "vl53lmz_plugin_motion_indicator.h"

int example10(VL53LMZ_Configuration *p_dev)
{

	/*********************************/
	/*   VL53LMZ ranging variables  */
	/*********************************/

	uint8_t 		status, loop, isAlive, isReady, i;
	VL53LMZ_Motion_Configuration 	motion_config;	/* Motion configuration*/
	VL53LMZ_ResultsData 	Results;		/* Results data from VL53LMZ */


	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53LMZ sensor connected */
	status = vl53lmz_is_alive(p_dev, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53LMZ not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53LMZ sensor */
	status = vl53lmz_init(p_dev);
	if(status)
	{
		printf("VL53LMZ ULD Loading failed\n");
		return status;
	}

	printf("VL53LMZ ULD ready ! (Version : %s)\n",
			VL53LMZ_API_REVISION);


	/*********************************/
	/*   Program motion indicator    */
	/*********************************/

	/* Create motion indicator with resolution 4x4 */
	status = vl53lmz_motion_indicator_init(p_dev, &motion_config, VL53LMZ_RESOLUTION_4X4);
	if(status)
	{
		printf("Motion indicator init failed with status : %u\n", status);
		return status;
	}

	/* (Optional) Change the min and max distance used to detect motions. The
	 * distance between min and max must never be >1500mm, otherwise the function below returns error 127 */
	status = vl53lmz_motion_indicator_set_distance_motion(p_dev, &motion_config, 1000, 2000);
	if(status)
	{
		printf("Motion indicator set distance motion failed with status : %u\n", status);
		return status;
	}

	/* If user want to change the resolution, he also needs to update the motion indicator resolution */
	//status = vl53lmz_set_resolution(p_dev, VL53LMZ_RESOLUTION_4X4);
	//status = vl53lmz_motion_indicator_set_resolution(p_dev, &motion_config, VL53LMZ_RESOLUTION_4X4);

	/* Increase ranging frequency for the example */
	status = vl53lmz_set_ranging_frequency_hz(p_dev, 2);


	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53lmz_start_ranging(p_dev);

	loop = 0;
	while(loop < 10)
	{
		/* Use polling function to know when a new measurement is ready.
		 * Another way can be to wait for HW interrupt raised on PIN A3
		 * (GPIO 1) when a new measurement is ready */

		status = vl53lmz_check_data_ready(p_dev, &isReady);

		if(isReady)
		{
			vl53lmz_get_ranging_data(p_dev, &Results);

			/* As the sensor is set in 4x4 mode by default, we have a total
			 * of 16 zones to print. For this example, only the data of first zone are
			 * print */
			printf("Print data no : %3u\n", p_dev->streamcount);
			for(i = 0; i < 16; i++)
			{
				printf("Zone : %3d, Motion power : %6d\n",
					i,
					(int)Results.motion_indicator.motion[motion_config.map_id[i]]);
			}
			printf("\n");
			loop++;
		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		WaitMs(&p_dev->platform, 5);
	}

	status = vl53lmz_stop_ranging(p_dev);
	printf("End of ULD demo\n");
	return status;
}
