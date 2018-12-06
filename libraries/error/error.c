/****************************************************************************
 # $Id: error.c,v 1.8 2003/08/01 22:19:35 tucelec Exp $
 #
 # Copyright (C) 1999
 # Associated Universities, Inc. Washington DC, USA.
 #
 # Correspondence concerning ALMA should be addressed as follows:
 #        Internet email: mmaswgrp@nrao.edu
 ****************************************************************************
 *
 *  MAIN.C 
 *
 *  Slave node implementation for AMBSI board.  This implementation
 *  of an M&C slave node provides access to the analog and digital
 *  I/O on the C167, and the ambient temperature in the DS1820.  Note that it 
 *  is specific to the hardware setup of the board.
 *
 ****************************************************************************
 */

/* Include general libraries */
#include <stdlib.h>
#include <string.h>

/* include library interface */
#include "error.h"

/* Typedefs */
typedef struct{
	unsigned char			*facility_descr;
	FACILITY_ERROR_ARRAY	*facility_error_array;
} MAIN_ERROR_ARRAY;

/* Globals */
static MAIN_ERROR_ARRAY		main_array[8];
unsigned char				error_status[8];
static unsigned char		already_initialized=0;
static unsigned char		devices=0;

/* Error array for Error library */
static FACILITY_ERROR_ARRAY	facility_error_array[6]={{0, "NoEr"},	// 0x00 -> No Error
													 {1, "NMem"},	// 0x01 -> Not Enough Memory
													 {2, "AIni"},	// 0x02 -> Library Already Initialized
													 {3, "NIni"},	// 0x03 -> Library not Initialized
													 {4, "DMax"},	// 0x04 -> Exceeded Max Number Devices
													 {5, "DNRg"}};	// 0x05 -> Device not Accounted for in the Initialization (Not Registered)
static unsigned char		facility_descr[3]="ERR";

/* Prototypes */
int				init_error_handler(unsigned char devices_no);
unsigned char	reg_facility(unsigned char *facility_descr, FACILITY_ERROR_ARRAY *facility_error_array);
unsigned char	report_error(unsigned char facility_no, unsigned char error_no);
void			write_error_CAN(CAN_MSG_TYPE *can_msg);
void			clear_error(void);

/* Initialize main error array */
int init_error_handler(unsigned char devices_no){

	clear_error();

	if(already_initialized){
		report_error(0,2); // ERROR - Library Already Initialized
		return -1;
	}

	if(devices_no>8){
		report_error(0,4); // ERROR - Exceeded Max Number Devices
		return -1;
	}

	/* Registring error array for error facility (facility# = 0) */
	main_array[0].facility_descr=facility_descr;
	main_array[0].facility_error_array=facility_error_array;
	devices=devices_no;

	already_initialized=1;

	return 0;
}

/* Register facilities into the main array */
unsigned char reg_facility(unsigned char *facility_descr, FACILITY_ERROR_ARRAY *facility_error_array){

	unsigned char static facility_index=1;
	unsigned char i;

	if(already_initialized){
		if(facility_index>devices){
			/* Check if the device has already been registered */
			for(i=1;i<devices;i++){
				if(!strcmp(main_array[i].facility_descr,facility_descr)){
					return i; // Device already initialize: return the device number
				}
			}
			report_error(0,5); // ERROR - Device not Accounted for in the Initialization
			return -1;
		}
		/* Check if the device has already been registered */
		for(i=1;i<facility_index;i++){
			if(!strcmp(main_array[i].facility_descr,facility_descr)){
				return i; // Device already initialize: return the device number
			}
		}
		main_array[facility_index].facility_descr=facility_descr;
		main_array[facility_index].facility_error_array=facility_error_array;

		return facility_index++;
	}

	report_error(0,3); // ERROR - Library not Initialized
	return -1;
}

/* Fill in error status */
unsigned char report_error(unsigned char facility_no, unsigned char error_no){
	
	if(facility_no==0){
	 	memcpy(&error_status[0],facility_descr,3);
		memcpy(&error_status[3],&facility_error_array[error_no].description,4);
		error_status[7]=(facility_no<<5)|(error_no&0x1F);
	} else {
		if(already_initialized){
			if(facility_no>devices){
				return report_error(0,5); // ERROR - Device not Accounted for in the Initialization
			}
 			memcpy(&error_status[0],main_array[facility_no].facility_descr,3);
			memcpy(&error_status[3],main_array[facility_no].facility_error_array[error_no].description,4);
			error_status[7]=(facility_no<<5)|(error_no&0x1F);

			return error_status[7];
		}
		
		return report_error(0,3); // ERROR - Library not Initialized
	}

	return error_status[7];
}

/* Write error to CAN message */
void write_error_CAN(CAN_MSG_TYPE *can_msg){

	can_msg->len = 8;	

	memcpy(can_msg->data,&error_status,7);
	can_msg->data[7]=error_status[7];

}

/* Clear error status */
void clear_error(void){
	memcpy(error_status,"\0\0\0\0\0\0\0\0",8);
}
