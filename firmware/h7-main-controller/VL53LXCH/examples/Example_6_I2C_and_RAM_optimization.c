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

/**************************************/
/*  VL53LMZ ULD I2C/RAM optimization */
/**************************************/
/*
* This example shows the possibility of VL53LMZ to reduce I2C transactions
* and RAM footprint. It initializes the VL53LMZ ULD, and starts
* a ranging to capture 10 frames.
*
* In this example, we also suppose that the number of target per zone is
* set to 1 , and all output are enabled (see file platform.h).
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53lmz_api.h"

int example6(VL53LMZ_Configuration* p_dev)
{

	/*********************************/
	/*   VL53LMZ ranging variables  */
	/*********************************/

	uint8_t status, loop, isAlive, isReady, i;
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
	/*   Reduce RAM & I2C access	 */
	/*********************************/

	/* Results can be tuned in order to reduce I2C access and RAM footprints.
	 * The 'platform.h' file contains macros used to disable output. If user declare 
	 * one of these macros, the results will not be sent through I2C, and the array will not 
	 * be created into the VL53LMZ_ResultsData structure.
	 * For the minimum size, ST recommends 1 targets per zone, and only keep distance_mm,
	 * target_status, and nb_target_detected. The following macros can be defined into file 'platform.h':
	 *
	 * #define VL53LMZ_DISABLE_AMBIENT_PER_SPAD
	 * #define VL53LMZ_DISABLE_NB_SPADS_ENABLED
	 * #define VL53LMZ_DISABLE_SIGNAL_PER_SPAD
	 * #define VL53LMZ_DISABLE_RANGE_SIGMA_MM
	 * #define VL53LMZ_DISABLE_REFLECTANCE_PERCENT
	 * #define VL53LMZ_DISABLE_MOTION_INDICATOR
	 */

	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53lmz_start_ranging(p_dev);

	loop = 0;
	while (loop < 10)
	{
		/* Use polling function to know when a new measurement is ready.
		 * Another way can be to wait for HW interrupt raised on PIN A3
		 * (GPIO 1) when a new measurement is ready */

		status = vl53lmz_check_data_ready(p_dev, &isReady);

		if (isReady)
		{
			vl53lmz_get_ranging_data(p_dev, &Results);

			/* As the sensor is set in 4x4 mode by default, we have a total 
			 * of 16 zones to print. For this example, only the data of first zone are 
			 * print */
			printf("Print data no : %3u\n", p_dev->streamcount);
			for (i = 0; i < 16; i++)
			{
				printf("Zone : %3d, Status : %3u, Distance : %4d mm\n", i, Results.target_status[
					VL53LMZ_NB_TARGET_PER_ZONE * i], Results.distance_mm[VL53LMZ_NB_TARGET_PER_ZONE * i]);
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
