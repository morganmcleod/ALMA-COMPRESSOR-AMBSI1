/* C167 register definitions */
#include <reg167.h>							// C167 register definitions

/* Includes */
#include "..\error\error.h"					// Error library
#include "onboard_adc.h"					// Serial communication include

/* Static for error library */
static unsigned char			facility_descr[3]="ADC";	// Facility description for error library
static unsigned char			facility_no;				// Facility number assigned at registation with the library
static FACILITY_ERROR_ARRAY		facility_error_array[4]={{0, "NoEr"},	// 0x00 -> No Error
														 {1, "ChOR"},	// 0x01 -> Required Channel Out of Range
														 {2, "MdOR"},	// 0x02 -> Required Mode Out of Range
														 {3, "CROR"}	// 0x03 -> Required Conversion Time Out of Range
														};
				
/* SFRs */
sfr P5DIDIS = 0xFFA4; // Register to disable digital input on port 5

/* Prototypes */
int adc_init(unsigned char mode, bit wait_read, bit ch_inj_ena, unsigned char conv_time);
float get_adc_single(unsigned char channel);
unsigned int get_adc_single_uint(unsigned char channel);

/* Initialize ADC */
int adc_init(unsigned char mode, bit wait_read, bit ch_inj_ena, unsigned char conv_time){

	unsigned char ADSTC=0, ADCTC=0;

	facility_no=reg_facility(facility_descr,facility_error_array); // Register the device into the error library

	/* Stops all running conversion and wait for the last one to be done */
	ADST = 0;
	while(ADBSY);

	P5DIDIS = 0xFFFF; // Disable digital input on P5 pins

	ADCON = 0; // Clears all setups, stops any conversion in progress

	if(mode>0x03){
		report_error(facility_no,2); // ERROR - 0x02 -> Required Mode Out of Range
		return -1;
	}

	if(conv_time>0x0F){
		report_error(facility_no,3); // ERROR - 0x03 -> Required Conversion Time Out of Range
		return -1;
	}

	// ADCON |= mode<<4; // Set the operation mode for the ADC
	// Not implemented yet!

	// ADWR = wait_read; // Set the wait-for-read flag
	ADWR = 0; // Not implemented yet!

	// ADCIN = ch_inj_ena; // Enable/disable the channel injection
	ADCIN = 0; // Not implemented yet!
	ADCRQ = 0; // Disable the injection request

	switch(conv_time){
		case 0x0:
			ADSTC = 0;
			ADCTC = 1;
			break;
		case 0x1:
			ADSTC = 1;
			ADCTC = 1;
			break;
		case 0x2:
			ADSTC = 2;
			ADCTC = 1;
			break;
		case 0x3:
			ADSTC = 0;
			ADCTC = 0;
			break;
		case 0x4:
			ADSTC = 3;
			ADCTC = 1;
			break;
		case 0x5:
			ADSTC = 1;
			ADCTC = 0;
			break;
		case 0x6:
			ADSTC = 2;
			ADCTC = 0;
			break;
		case 0x7:
			ADSTC = 0;
			ADCTC = 3;
			break;
		case 0x8:
			ADSTC = 3;
			ADCTC = 0;
			break;
		case 0x9:
			ADSTC = 1;
			ADCTC = 3;
			break;
		case 0xA:
			ADSTC = 2;
			ADCTC = 3;
			break;
		case 0xB:
			ADSTC = 0;
			ADCTC = 2;
			break;
		case 0xC:
			ADSTC = 3;
			ADCTC = 3;
			break;
		case 0xD:
			ADSTC = 1;
			ADCTC = 2;
			break;
		case 0xE:
			ADSTC = 2;
			ADCTC = 2;
			break;
		case 0xF:
			ADSTC = 3;
			ADCTC = 2;
			break;
		default:
			break;
	}

	ADCON |= ((ADCTC<<14) | (ADSTC<<12)); // Set the conversion/sampling timing

	return 0;

}

float get_adc_single(unsigned char channel){

	float temp_data;

	if(channel>0xF){
		report_error(facility_no,1); // ERROR - 0x01 -> Required Channel Out of Range
		return -1;
	}

	while(ADBSY); // Wait until current conversion done

	/* Load new channel */
	ADCON &= 0xFFF0;
	ADCON |= channel;

	ADST = 1; // Start Conversion

	while(ADBSY); // Wait for conversion to be over

	temp_data = 0.0048828125*(ADDAT & 0x03FF);

	return temp_data;

}

unsigned int get_adc_single_uint(unsigned char channel){

	unsigned int temp_data;

	if(channel>0xF){
		report_error(facility_no,1); // ERROR - 0x01 -> Required Channel Out of Range
		return -1;
	}

	while(ADBSY); // Wait until current conversion done

	/* Load new channel */
	ADCON &= 0xFFF0;
	ADCON |= channel;

	ADST = 1; // Start Conversion

	while(ADBSY); // Wait for conversion to be over

	temp_data = (ADDAT & 0x03FF);

	return temp_data;

}


