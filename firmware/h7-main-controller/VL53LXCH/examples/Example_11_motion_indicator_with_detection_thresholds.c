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
#include "vl53lmz_plugin_detection_thresholds.h"

/* This function needs to be filled by the customer. It allows knowing when
 * the VL53LMZ interrupt is raised on GPIO1. This is the only way to use detection thresholds.
 */

static int WaitForL5Interrupt(VL53LMZ_Configuration* pDev)
{

	//Add your implementation here ...
	UNUSED(pDev);

	return 0;
}

int example11(VL53LMZ_Configuration* p_dev)
{

	/*********************************/
	/*   VL53LMZ ranging variables  */
	/*********************************/

	uint8_t status, loop, isAlive, isReady, i;
	VL53LMZ_Motion_Configuration motion_config;    /* Motion configuration*/
	VL53LMZ_ResultsData Results;        /* Results data from VL53LMZ */


	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53LMZ sensor connected */
	status = vl53lmz_is_alive(p_dev, &isAlive);
	if (!isAlive || status)
	{
		printf("VL53LMZ not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53LMZ sensor */
	status = vl53lmz_init(p_dev);
	if (status)
	{
		printf("VL53LMZ ULD Loading failed\n");
		return status;
	}

	printf("VL53LMZ ULD ready ! (Version : %s)\n", VL53LMZ_API_REVISION);


	/*********************************/
	/*   Program motion indicator    */
	/*********************************/

	/* Create motion indicator with resolution 8x8 */
	status = vl53lmz_motion_indicator_init(p_dev, &motion_config, VL53LMZ_RESOLUTION_8X8);
	if (status)
	{
		printf("Motion indicator init failed with status : %u\n", status);
		return status;
	}

	/* (Optional) Change the min and max distance used to detect motions. The
	 * difference between min and max must never be >1500mm, and minimum never be <400mm,
	 * otherwise the function below returns error 127 */
	status = vl53lmz_motion_indicator_set_distance_motion(p_dev, &motion_config, 1000, 2000);
	if (status)
	{
		printf("Motion indicator set distance motion failed with status : %u\n", status);
		return status;
	}

	/* If user want to change the resolution, he also needs to update the motion indicator resolution */
	//status = vl53lmz_set_resolution(p_dev, VL53LMZ_RESOLUTION_4X4);
	//status = vl53lmz_motion_indicator_set_resolution(p_dev, &motion_config, VL53LMZ_RESOLUTION_4X4);


	/* Set the device in AUTONOMOUS and set a small integration time to reduce power consumption */
	status = vl53lmz_set_resolution(p_dev, VL53LMZ_RESOLUTION_8X8);
	status = vl53lmz_set_ranging_mode(p_dev, VL53LMZ_RANGING_MODE_AUTONOMOUS);
	status = vl53lmz_set_ranging_frequency_hz(p_dev, 2);
	status = vl53lmz_set_integration_time_ms(p_dev, 10);


	/*********************************/
	/*  Program detection thresholds */
	/*********************************/

	/* In this example, we want 1 thresholds per zone for a 8x8 resolution */
	/* Create array of thresholds (size cannot be changed) */
	VL53LMZ_DetectionThresholds thresholds[VL53LMZ_NB_THRESHOLDS];

	/* Set all values to 0 */
	memset(&thresholds, 0, sizeof(thresholds));

	/* Add thresholds for all zones (64 zones in resolution 4x4, or 64 in 8x8) */
	for (i = 0; i < 64; i++)
	{
		thresholds[i].zone_num = i;
		thresholds[i].measurement = VL53LMZ_MOTION_INDICATOR;
		thresholds[i].type = VL53LMZ_GREATER_THAN_MAX_CHECKER;
		thresholds[i].mathematic_operation = VL53LMZ_OPERATION_NONE;

		/* The value 44 is given as example. All motion above 44 will be considered as a movement */
		thresholds[i].param_low_thresh = 44;
		thresholds[i].param_high_thresh = 44;
	}

	/* The last thresholds must be clearly indicated. As we have 64
	 * checkers, the last one is the 63 */
	thresholds[63].zone_num = VL53LMZ_LAST_THRESHOLD | thresholds[63].zone_num;

	/* Send array of thresholds to the sensor */
	vl53lmz_set_detection_thresholds(p_dev, thresholds);

	/* Enable detection thresholds */
	vl53lmz_set_detection_thresholds_enable(p_dev, 1);


	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53lmz_start_ranging(p_dev);

	loop = 0;
	while (loop < 10)
	{
		/* Function WaitForL5Interrupt() does not exists, and must be
		 * implemented by user. It allows catching the interrupt raised on
		 * pin A3 (INT), when the checkers detect the programmed
		 * conditions.
		 */

		isReady = WaitForL5Interrupt(p_dev);

		if (isReady)
		{
			vl53lmz_get_ranging_data(p_dev, &Results);

			/* As the sensor is set in 4x4 mode by default, we have a total
			 * of 16 zones to print. For this example, only the data of first zone are
			 * print */
			printf("Print data no : %3u\n", p_dev->streamcount);
			for (i = 0; i < 16; i++)
			{
				printf("Zone : %3d, Motion power : %6d\n", i, (int)Results.motion_indicator.motion[motion_config
					.map_id[i]]);
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
