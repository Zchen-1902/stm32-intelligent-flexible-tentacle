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
/*    VL53LMZ ULD ranging mode    */
/***********************************/
/*
* This example shows the differences between ranging modes of VL53LMZ
* (mode continuous and autonomous). For both modes, it initializes the VL53LMZ
* ULD, set the mode, and starts a ranging to capture 10 frames.
*
* In this example, we also suppose that the number of target per zone is
* set to 1 , and all output are enabled (see file platform.h).
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53lmz_api.h"

int example3(VL53LMZ_Configuration *p_dev)
{

	/*********************************/
	/*   VL53LMZ ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, isReady, i;
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
	/*  Set ranging mode autonomous  */
	/*********************************/

	status = vl53lmz_set_ranging_mode(p_dev, VL53LMZ_RANGING_MODE_AUTONOMOUS);
	if(status)
	{
		printf("vl53lmz_set_ranging_mode failed, status %u\n", status);
		return status;
	}

	/* Using autonomous mode, the integration time can be updated (not possible
	 * using continuous) */
	status = vl53lmz_set_integration_time_ms(p_dev, 20);

	/* Start a ranging session */
   	status = vl53lmz_start_ranging(p_dev);
   	printf("Start ranging autonomous\n");

   	loop = 0;
   	while(status == VL53LMZ_STATUS_OK)
   	{
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
				printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
					i,
					Results.target_status[VL53LMZ_NB_TARGET_PER_ZONE*i],
					Results.distance_mm[VL53LMZ_NB_TARGET_PER_ZONE*i]);
   			}
   			printf("\n");
   			loop++;
   		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
   		WaitMs(&p_dev->platform, 5);
   	}

   	status = vl53lmz_stop_ranging(p_dev);
   	printf("Stop ranging autonomous\n");

	
	/*********************************/
	/* Set ranging mode continuous   */
	/*********************************/
//
//	/* In continuous mode, the integration time cannot be programmed
//	 * (automatically set to maximum value) */
//
//	status = vl53lmz_set_ranging_mode(p_dev, VL53LMZ_RANGING_MODE_CONTINUOUS);
//	if(status)
//	{
//		printf("vl53lmz_set_ranging_mode failed, status %u\n", status);
//		return status;
//	}
//
//	/* Trying to update value below will have no impact on integration time */
//	//status = vl53lmz_set_integration_time_ms(p_dev, 20);
//
//	/* Start a ranging session */
//   	status = vl53lmz_start_ranging(p_dev);
//   	printf("Start ranging continuous\n");
//
//   	loop = 0;
//   	while(loop < 10)
//   	{
//   		status = vl53lmz_check_data_ready(p_dev, &isReady);
//   		if(isReady)
//   		{
//   			vl53lmz_get_ranging_data(p_dev, &Results);
//
//   			/* As the sensor is set in 4x4 mode by default, we have a total
//			 * of 16 zones to print */
//   			printf("Print data no : %3u\n", p_dev->streamcount);
//   			for(i = 0; i < 16; i++)
//   			{
//   				printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
//					i,
//					Results.target_status[VL53LMZ_NB_TARGET_PER_ZONE*i],
//					Results.distance_mm[VL53LMZ_NB_TARGET_PER_ZONE*i]);
//   			}
//   			printf("\n");
//   			loop++;
//   		}
//
//		/* Wait a few ms to avoid too high polling (function in platform
//		 * file, not in API) */
//   		WaitMs(&p_dev->platform, 5);
//   	}
//
//   	status = vl53lmz_stop_ranging(p_dev);
//   	printf("Stop ranging continuous\n");

   	printf("End of ULD demo\n");
   	return status;
}
