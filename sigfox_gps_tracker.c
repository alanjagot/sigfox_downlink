/* @file
 * @brief Simple accelerometer event monitor application for the TDxxxx RF modules.
 * @author Telecom Design S.A.
 * @version 1.1.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014-2015 Telecom Design S.A., http://www.telecomdesign.fr</b>
 ******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Telecom Design SA has no
 * obligation to support this Software. Telecom Design SA is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Telecom Design SA will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
  ******************************************************************************/

#include "config.h"
#include "at_user.h"

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include <efm32.h>

#include <td_core.h>
#include <td_rtc.h>
#include <td_uart.h>
#include <td_printf.h>
#include <td_stream.h>
#include <td_flash.h>
#include <td_scheduler.h>
#include <td_gpio.h>
#include <td_utils.h>
#include <td_measure.h>

#include <at_parse.h>
#include <at_radio.h>
#include <at_sigfox.h>

#include <td_accelero.h>
#include <td_geoloc.h>
#include <td_sensor.h>
#include <sensor_data_geoloc.h>

#include <td_sigfox.h>

/*******************************************************************************
 ******************************  DEFINES ***************************************
 ******************************************************************************/

/** Manufacturer */
#define MANUFACTURER		"Telecom Design"

/** Product */
#define PRODUCT				"TDxxxx"

/** Hardware revision */
#define HARDWARE_VERSION	"0F"

/** Software version */
#define SOFTWARE_VERSION	"SOFTxxxx"

/** Release data */
#define RELEASE_DATE		"M10+2012"

/** Telecom Design 12-digit serial number */
#define SERIAL_NUMBER		"123456789012"

#include <td_config.h>

/** Flash variable version ID */
#define VARIABLES_VERSION 0

/** Acceptable minimum horizontal accuracy, 800 to be very accurate */
#define FIX_HDOP 800

/** Boot monitoring, 1 to enable */
#define BOOT_MONITORING 0

/** Keepalive monitoring interval in hours, 0 to disable
 * If you wish to send a keepalive frame remember to add a scheduler as well in the setup function */
#define KEEPALIVE_INTERVAL 0

/**************************************************************************************
 *************************  GLOBAL VARIABLES   ****************************************
 **************************************************************************************/

TD_STREAM_t *out_stream; // Pointer to UART stream structure
uint8_t gps_mode = TD_GEOLOC_HW_BCKP; // Default GPS sleep mode
int ds_id; // Delayed start scheduler id
int fix_timeout = 300; // Maximum waiting time of the device for finding GPS position
int fix_interval_movement = 3600; // Default interval used to send data if there is movement (t1 period)
int acc_print = 1; // Variable for choosing if the user wants to print data on serial, 1 is ok
uint8_t acc_freq = TD_ACCELERO_1HZ;
uint8_t acc_scale = TD_ACCELERO_2G;

/*******************************************************************************
 ***********************   ENUMERATIONS   **************************************
 ******************************************************************************/

/*******************************************************************************
 *************************  CONSTANTS   ****************************************
 ******************************************************************************/


/*******************************************************************************
 ******************************  FUNCTIONS  ************************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   Callback function called when the accelerometer data changes.
 *
 * @param[in] x
 *   X-axis acceleration value.
 *
 * @param[in] y
 *   Y-axis acceleration value.
 *
 * @param[in] z
 *   Z-axis acceleration value.
 ******************************************************************************/
void DataCallback(TD_ACCELERO_Data_t data[32], uint8_t count, bool overrun) {
	int i;
	for (i = 0; i < count; i++) {
		if(acc_print) {
			tfp_printf("%d\t%d\t%d\r\n", data[i].x, data[i].y, data[i].z);
		}
	}
	if (overrun) {
		tfp_printf("overrun\r\n");
	}
}

/***************************************************************************//**
 * @brief
 *  GPS fix callback
 *
 * @param[in] fix
 *   The GPS fix data structure.
 *
 * @param[in] timeout
 *   Flag that indicates whether a timeout occurred if set to true.
 ******************************************************************************/
static void GPSFix(TD_GEOLOC_Fix_t * fix, bool timeout) {
	int i;
	uint8_t bytes[12], temp;

	unsigned long latitude, longitude;
	int size;

	// Message init - Set to zero not to get random unwanted values
	for(i=0;i<12;i++) {
		bytes[i] = 0x00;
	}

	size = sizeof(bytes)/sizeof(bytes[0]);

	// Disable monitor accelerometer event (movement)
	TD_ACCELERO_MonitorData(false,	// Monitoring disabled
		false,						// Low-power mode disabled
		acc_freq, 			        // Sampling rate
		TD_ACCELERO_ALL_AXIS,		// Axis mask: all axis
		acc_scale,				    // Scale
		0,							// High-pass filter disabled
		TD_ACCELERO_STREAM,			// FIFO stream mode
		1,							// Update watermark enabled (32 max, 0 is real-time)
		DataCallback);



	// If GPS fix has been found
	if (fix->type >= TD_GEOLOC_2D_FIX && fix->hard.rtc_calibrated) {

		// Voltage measure
		uint32_t mv = TD_MEASURE_VoltageTemperatureExtended(false);

		// Temperature measure
		int32_t mv2 = TD_MEASURE_VoltageTemperatureExtended(true) / 10;

		// Stop GPS
		TD_GEOLOC_StopFix(gps_mode);

		// Now we add the received data in the bytes array:

		// Latitude
		// Hint: divide the return value by 10 will make it understandable by Sigfox backend
		if (fix->position.latitude < 0) {
			  latitude = (int32_t)(-1)* ((int32_t)fix->position.latitude / 10) ;
			  // latitude_direction = 'S';
			  latitude = latitude| 0x80000000;
		} else {
			  latitude = fix->position.latitude / 10;
			  // latitude_direction = 'N';
		}
		bytes[0] = (latitude >> 24) & 0xFF;
		bytes[1] = (latitude >> 16) & 0xFF;
		bytes[2] = (latitude >> 8) & 0xFF;
		bytes[3] = latitude & 0xFF;

		// Longitude
		if (fix->position.longitude < 0) {
			  longitude = (int32_t)(-1)* ((int32_t)fix->position.longitude / 10) ;
			  // longitude_direction = 'W';
			  longitude = longitude| 0x80000000;
		} else {
			  longitude = fix->position.longitude / 10;
			  // longitude_direction = 'E';
		}
		bytes[4] = (longitude >> 24) & 0xFF;
		bytes[5] = (longitude >> 16) & 0xFF;
		bytes[6] = (longitude >> 8) & 0xFF;
		bytes[7] = longitude & 0xFF;

		// Battery
		for(i=0; i<255; i++) {
			if(mv>=i*15 && mv<=(i+1)*15) {
				bytes[8] = bytes[8] |  i;
			}
			else if(mv>3825) {
				bytes[8] = bytes[8] | 0xFF;
			}
		}

		// Temperature
		if (mv2 < 0) {
			temp = (int32_t)(-1) * (int32_t)mv2; // If the temperature is below 0;
			temp = temp | 0x80;
		}
		else {
			temp = mv2;
		}
		bytes[9] = temp & 0xFF;

		// Sending the message
		//TD_SIGFOX_Send(bytes, size, 2);
		tfp_printf("TX BEGIN\r\n");
		TD_SIGFOX_SendV1(MODE_FRAME, false, bytes, 12, 2, true, false);
		tfp_dump("TX=", bytes, 12);
		tfp_printf("TX END\r\n");
	}
	else if (timeout) {
		// If no GPS fix has been found, we still send the voltage and the temperature

		// Voltage measure
		uint32_t mv = TD_MEASURE_VoltageTemperatureExtended(false);

		// Temperature measure
		int32_t mv2 = TD_MEASURE_VoltageTemperatureExtended(true) / 10;

		// Set the device in its sleep mode
		TD_GEOLOC_StopFix(gps_mode);

		// Battery
		for(i=0; i<255; i++) {
			if(mv>=i*15 && mv<=(i+1)*15) {
				bytes[8] = bytes[8] |  i;
			}
			else if(mv>3825) {
				bytes[8] = bytes[8] | 0xFF;
			}
		}

		// Temperature
		if (mv2 < 0) {
			temp = (int32_t)(-1) * (int32_t)mv2; // Temperature below 0;
			temp = temp | 0x80;
		}
		else {
			temp = mv2;
		}
		bytes[9] = temp & 0xFF;

		// Sending the message
		//TD_SIGFOX_Send(bytes, 12, 2);
		tfp_printf("TX BEGIN\r\n");
		TD_SIGFOX_SendV1(MODE_FRAME, false, bytes, size, 2, true, false);
		tfp_dump("TX=", bytes, size);
		tfp_printf("TX END\r\n");
	}



	// Enable monitor accelerometer event (movement)
	TD_ACCELERO_MonitorData(true,	// Monitoring enabled
		false,						// Low-power mode disabled
		acc_freq, 			        // Sampling rate
		TD_ACCELERO_ALL_AXIS,		// Axis mask: all axis
		acc_scale,				    // Scale
		0,							// High-pass filter disabled
		TD_ACCELERO_STREAM,			// FIFO stream mode
		1,							// Update watermark enabled (32 max, 0 is real-time)
		DataCallback);

}
/***************************************************************************//**
 * @brief
 *   Call-back function for SIGFOX downlink.
 *
 * @param[in] rx_frame
 *   Pointer to the received frame or null if timeout occurred.
 *
 * @param[in] length
 *   Length in bytes of the received frame.
 *
 * @return
 *   Returns the buffer length if OK, -1 if an error occurred.
 ******************************************************************************/
static int DownlinkCallback(uint8_t *rx_frame, uint8_t length)
{
	int i;

	//tfp_printf("Downlink...\r\n");
	if (rx_frame == 0) {
		// Finished receiving
		//TD_SIGFOX_DOWNLINK_SetUserCallback(0);
		tfp_printf("RX END\r\n");
		// Done
		return 1;
	} else {
		if (length == 0) {
			// Start receiving
			tfp_printf("RX BEGIN\r\n");
			// Done
			return 1;
		}
		// Received one good frame
		tfp_dump("RX=", rx_frame, length);
		tfp_printf("rx_frame: %d - %x - %s\r\n", rx_frame, rx_frame, rx_frame);
		for(i=0;i<8;i++){
			tfp_printf("rx_frame[%d]: %x - %d\r\n", i, rx_frame[i], rx_frame[i]);
			switch(rx_frame[0]) {

			case 0x04 :
				acc_scale = TD_ACCELERO_16G;
				break;

			case 0x03 :
				acc_scale = TD_ACCELERO_8G;
				break;

			case 0x02 :
				acc_scale = TD_ACCELERO_4G;
				break;

			case 0x01 :
				acc_scale = TD_ACCELERO_2G;
				break;

			case 0x00 :
				break;

			default :
				acc_scale = TD_ACCELERO_2G;
				break;
			} switch(rx_frame[1]) {

			case 0x08 :
				acc_freq = TD_ACCELERO_1_25KHZ;
				break;

			case 0x07 :
				acc_freq = TD_ACCELERO_400HZ;
				break;

			case 0x06 :
				acc_freq = TD_ACCELERO_200HZ;
				break;

			case 0x05 :
				acc_freq = TD_ACCELERO_100HZ;
				break;

			case 0x04 :
				acc_freq = TD_ACCELERO_50HZ;
				break;

			case 0x03 :
				acc_freq = TD_ACCELERO_25HZ;
				break;

			case 0x02 :
				acc_freq = TD_ACCELERO_10HZ;
				break;

			case 0x01 :
				acc_freq = TD_ACCELERO_1HZ;
				break;

			case 0x00 :
				break;

			default :
				acc_freq = TD_ACCELERO_1HZ;
				break;

			} switch (rx_frame[2])	{
			case 0x0F :
				TD_SCHEDULER_SetInterval(ds_id,86400,0,0); //24h
				break;

			case 0x0E :
				TD_SCHEDULER_SetInterval(ds_id,75600,0,0); //21h
				break;

			case 0x0D :
				TD_SCHEDULER_SetInterval(ds_id,64800,0,0); // 18h
				break;

			case 0x0C :
				TD_SCHEDULER_SetInterval(ds_id,54000,0,0); //15h
				break;

			case 0x0B :
				TD_SCHEDULER_SetInterval(ds_id,43200,0,0); //12h
				break;

			case 0x0A :
				TD_SCHEDULER_SetInterval(ds_id,36000,0,0); //10h
				break;

			case 0x09 :
				TD_SCHEDULER_SetInterval(ds_id,28800,0,0); //8h
				break;

			case 0x08 :
				TD_SCHEDULER_SetInterval(ds_id,21600,0,0); //6h
				break;

			case 0x07 :
				TD_SCHEDULER_SetInterval(ds_id,18000,0,0); //5h
				break;

			case 0x06 :
				TD_SCHEDULER_SetInterval(ds_id,14400,0,0); //4h
				break;

			case 0x05 :
				TD_SCHEDULER_SetInterval(ds_id,10800,0,0); //3h
				break;

			case 0x04 :
				TD_SCHEDULER_SetInterval(ds_id,7200,0,0); //2h
				break;

			case 0x03 :
				TD_SCHEDULER_SetInterval(ds_id,3600,0,0); //1h
				break;

			case 0x02 :
				TD_SCHEDULER_SetInterval(ds_id,3600,0,0); //30 min
				break;

			case 0x01 :
				TD_SCHEDULER_SetInterval(ds_id,1200,0,0); //20 min
				break;

			case 0x00 :
				TD_SCHEDULER_SetInterval(ds_id,600,0,0); //10 min
				break;

			default :
				TD_SCHEDULER_SetInterval(ds_id,3600,0,0); //1 h
			break;

			}
		}
	}
		//Here is the GPS timeout value meaning the maximum time you will try to get a GPS fix
		fix_timeout = rx_frame[0];
		tfp_printf("timeout: %d (in seconds)\r\n", fix_timeout);

		// Done
		return 1;

	}

/***************************************************************************//**
 * @brief
 *  Start fixing periodically.
 *
 * @param[in] arg
 *  Generic argument set by the user that is passed along to the callback
 *  function.
 *
 * @param[in] repeat_count
 *  Updated repeat count, decremented at each timer trigger, unless it is an
 *  infinite timer.
 ******************************************************************************/

static void StartFixing(uint32_t arg, uint8_t repeat_count)  {

	// Trying to send GPS coordinates, and temperature, voltage
	TD_GEOLOC_TryToFix(TD_GEOLOC_NAVIGATION, fix_timeout, GPSFix);
}

/***************************************************************************//**
 * @brief
 *  delayed_start callback invokes StartFixing
 *
 * @param[in] arg
 *  Generic argument set by the user that is passed along to the callback
 *  function.
 *
 * @param[in] repeat_count
 *  Updated repeat count, decremented at each timer trigger, unless it is an
 *  infinite timer.
 ******************************************************************************/
static void delayed_start(uint32_t arg, uint8_t repeat_count) {

	// Enable monitor accelerometer event (movement)
	TD_ACCELERO_MonitorData(true,	// Monitoring enabled
		false,						// Low-power mode disabled
		acc_freq, 			        // Sampling rate
		TD_ACCELERO_ALL_AXIS,		// Axis mask: all axis
		acc_scale,				    // Scale
		0,							// High-pass filter disabled
		TD_ACCELERO_STREAM,			// FIFO stream mode
		1,							// Update watermark enabled (32 max, 0 is real-time)
		DataCallback);


	// Start the GPS immediately
	StartFixing(0, 0);

	// StartFixing function will executed every fix_interval_movement period
	TD_SCHEDULER_Append(fix_interval_movement, 0, 0, TD_SCHEDULER_INFINITE, StartFixing, 0);

}


/***************************************************************************//**
 * @brief
 *  User Setup function.
 ******************************************************************************/
void TD_USER_Setup(void) {

	TD_UART_Options_t options = {LEUART_DEVICE, LEUART_LOCATION, 9600, 8, 'N',
	1, false};

	// Open an I/O stream using LEUART0
	out_stream = TD_UART_Open(&options, TD_STREAM_RDWR);

	// Set flash variables version
	TD_FLASH_DeleteVariables();

	// Add the RF information AT extension
	//AT_AddExtension(&user_extension);

	// Initialize the AT command parser
	//AT_Init();

	// Set the device as a Sensor transmitter
	TD_SENSOR_Init(SENSOR_TRANSMITTER, 0, 0);

	// Geoloc and accelerometer initialization
	TD_GEOLOC_Init();
	TD_ACCELERO_Init();

	// Delayed start, waiting for commands during 30 seconds
	ds_id = TD_SCHEDULER_Append(0, 0, 30, TD_SCHEDULER_ONE_SHOT, delayed_start, 0);

	// Send a keep-alive frame immediately
	TD_SENSOR_MonitorKeepAlive(true, 0);

	TD_SIGFOX_DOWNLINK_SetUserCallback(DownlinkCallback);

}

/***************************************************************************//**
 * @brief
 *   User loop function.
 ******************************************************************************/
void TD_USER_Loop(void) {


	// Process Sensor events
	TD_SENSOR_Process();

	// Process Geoloc events
	TD_GEOLOC_Process();

	// Process downlink events
	TD_SIGFOX_DOWNLINK_Process();

	// Process Accelerometer events
	TD_ACCELERO_Process();

}
