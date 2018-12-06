/****************************************************************************
 # $Id: main.c,v 1.3 2009/06/15 22:02:02 avaccari Exp $
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
 *  Slave node implementation for ALMA compressor monitor and control.
 *
 *	Revision history
 *
 *  2008-03-05
 *  Rev. 1.0.1
 *  - Fixed mapping of analog channels 
 *  - Changed scaling for pressure
 *  - Inverted logic for drive readout
 * 
 *  2007-09-19
 *  Rev. 1.0.0
 *  First version ready for debugging.
 *
 ****************************************************************************
 */

/* Uses serial port */
#include <reg167.h>

/* include library interface */
#include "..\..\libraries\amb\amb.h"
#include "..\..\libraries\ds1820\ds1820.h"
#include "..\..\libraries\onboard_adc\onboard_adc.h"

/* Defines */
#define USE_48MS	0	// Defines if the 48ms pulse is used to trigger the correponding interrupt
						// If yes then P8.0 will not be available for use as a normal I/O pin since
 					    // it will receive the 48ms pulse.

/* General defines */
#define HIGH	1
#define LOW		0

/* Revision Level Defines */
#define	MAJOR	1
#define MINOR	0
#define PATCH	1

/** RCAs **/
/* Monitor */
#define FIRST_MONITOR_RCA		0x00001
#define GET_TEMP_1				0x00001
#define GET_TEMP_2				0x00002
#define GET_TEMP_3				0x00003
#define GET_TEMP_4				0x00004
#define GET_AUX_1				0x00005
#define GET_AUX_2				0x00006
#define GET_PRESSURE			0x00007
#define GET_PRESSURE_ALARM		0x00008
#define GET_TEMP_ALARM			0x00009
#define GET_DRIVE_INDICATION	0x0000A
#define GET_REVISION_LEVEL		0x0000B
#define LAST_MONITOR_RCA		0x0000B
/* Control */
#define FIRST_CONTROL_RCA		0x01001
#define SET_REMOTE_DRIVE		0x01001
#define SET_REMOTE_RESET		0x01002
#define LAST_CONTROL_RCA		0x01002
/* General */
#define BYTE_LEN				1
#define REVISION_LEN			3
#define FLOAT_LEN				4

/* Analog monitor channels defines */
#define CH_T1		8	// 0->5V => -30->60C
#define CH_T2		9	// 0->5V => -30->60C
#define CH_T3		10	// 0->5V => -30->60C
#define	CH_T4		11	// 0->5V => -30->60C
#define CH_AUX1		12	// 0->5V
#define CH_AUX2		13	// 0->5V
#define CH_PRESS	14	// 0->5V => 0->300psi

/* Analog conversion factors defines */
#define TEMP(V)		((18.0*(V))-30.0)
#define AUX(V)		V
#define PRESSURE(V)	(V-1.0)

/* Special register defines */
#define PRES_ALARM			P2^0	// Pressure alarm (active low)
#define PRES_ALARM_DP		DP2		// Port for pressure alarm
#define	PRES_ALARM_MASK 	0x0001	// Direction mask for pressure alarm

#define TEMP_ALARM			P2^2	// Temperature alarm (active low)
#define TEMP_ALARM_DP		DP2		// Port for temperature alarm
#define	TEMP_ALARM_MASK 	0x0004	// Direction mask for temperature alarm

#define DRIVE_IND			P2^4	// Drive indicator (active high)
#define DRIVE_IND_DP		DP2		// Port for drive indicator
#define	DRIVE_IND_MASK 		0x0010	// Direction mask for drive indicator
 
#define REMOTE_DRV			P7^0	// Remote drive
#define REMOTE_DRV_DP		DP7		// Port for remote drive
#define	REMOTE_DRV_MASK 	0x01	// Direction mask for remote drive

#define REMOTE_RST			P8^0	// Remote reset
#define REMOTE_RST_DP		DP8		// Port for remote reset
#define	REMOTE_RST_MASK 	0x01	// Direction mask for remote reset

/* Special function registers */
sbit pAlarm = PRES_ALARM;	// Pressure alarm bit (input)
sbit tAlarm = TEMP_ALARM;	// Temperature alarm bit (input)
sbit drvInd = DRIVE_IND;	// Drive indicator bit (input)
sbit rmtDrv	= REMOTE_DRV;	// Remote drive bit (ouput)
sbit rmtRst	= REMOTE_RST;	// Remote reset bit (ouput)

/* Set aside memory for the callbacks in the AMB library */
static CALLBACK_STRUCT cb_memory[3];

/* CAN message callbacks */
int ambient_msg(CAN_MSG_TYPE *message);  /* Called to get DS1820 temperature */
int monitor_msg(CAN_MSG_TYPE *message);  /* Called to get monitor messages */
int control_msg(CAN_MSG_TYPE *message);  /* Called to set control messages */

/* A global for the last read temperature */
ubyte ambient_temp_data[4];

/* A union for conversion float <-> 4 bytes */
union {
	float flt_val;
	ubyte chr_val[4];
} conv;

/* A couple of globals to keep track of last sent messages */
unsigned char lastRemoteDrive = 0;
unsigned char lastRemoteReset = 0;

/* External bus control signal buffer chip enable is on P4.7 */
sbit  DISABLE_EX_BUF	= P4^7;




/* Main */
void main(void) {
	if(USE_48MS){
		// Setup the CAPCOM2 unit to receive the 48ms pulse from the Xilinx
		P8&=0xFE; // Set value of P8.0 to 0
		DP8&=0xFE; // Set INPUT direction for P8.0
		CCM4&=0xFFF0; // Clear setup for CCMOD16
		CCM4|=0x0001; // Set CCMOD16 to trigger on rising edge
		CC16IC=0x0078; // Interrupt: ILVL=14, GLVL=0;
	}

	/* Make sure that external bus control signal buffer is disabled */
	DP4 |= 0x01;
	DISABLE_EX_BUF = 1;

	/* Initialise the slave library */
	if (amb_init_slave((void *) cb_memory) != 0) 
		return;

	/* Register callbacks for CAN events */
	if (amb_register_function(0x30003, 0x30003, ambient_msg) != 0)
		return;
	
	/* Register monitor callbacks */
	if (amb_register_function(FIRST_MONITOR_RCA, LAST_MONITOR_RCA, monitor_msg) !=0)
		return;

	/* Register control callbacks */
	if (amb_register_function(FIRST_CONTROL_RCA, LAST_CONTROL_RCA, control_msg) !=0)
		return;

	/* Initialize control lines */
	rmtDrv = 0; // Compressor off
	rmtRst = 0;	// Reset line default to low. It needs an high pulse to reset the compressor

	/* Port direction initialization */
	/* Write (set corresponding port bits to 1) */
	REMOTE_RST_DP |= REMOTE_RST_MASK;
	REMOTE_DRV_DP |= REMOTE_DRV_MASK;
	/* Read (set corresponding port bits to 0) */
	PRES_ALARM_DP &= ~PRES_ALARM_MASK;
	TEMP_ALARM_DP &= ~TEMP_ALARM_MASK;
	DRIVE_IND_DP &= ~DRIVE_IND_MASK;

	/* Initialize modules */
	adc_init(0,0,0,0); // ADC initialization

	/* globally enable interrupts */
  	amb_start();

	/* Never return */
	while (1){
			/* Read the DS1820 when nothing else occurs */
			ds1820_get_temp(&ambient_temp_data[1], &ambient_temp_data[0], &ambient_temp_data[2], &ambient_temp_data[3]);
	}
}




/* Temperature request messages */
int ambient_msg(CAN_MSG_TYPE *message) {

	if (message->dirn == CAN_MONITOR) {  /* Should only be a monitor requests */
		message->len = 4;
		message->data[0] = ambient_temp_data[0];
		message->data[1] = ambient_temp_data[1];
		message->data[2] = ambient_temp_data[2];
		message->data[3] = ambient_temp_data[3];
	} 

	return 0;
}






/* Monitor requests */
int monitor_msg(CAN_MSG_TYPE *message) {

	/* If it is a control message on monitor RCA, do nothing. */
	if(message->dirn==CAN_CONTROL){
		return 0;
	}

	/* Perform the monitor operation */
	switch(message->relative_address){
		case GET_TEMP_1:
			conv.flt_val = TEMP(get_adc_single(CH_T1));
			message->data[0] = conv.chr_val[3];
			message->data[1] = conv.chr_val[2];
			message->data[2] = conv.chr_val[1];
			message->data[3] = conv.chr_val[0];
			message->len = FLOAT_LEN;
			break;
		case GET_TEMP_2:
			conv.flt_val = TEMP(get_adc_single(CH_T2));
			message->data[0] = conv.chr_val[3];
			message->data[1] = conv.chr_val[2];
			message->data[2] = conv.chr_val[1];
			message->data[3] = conv.chr_val[0];
			message->len = FLOAT_LEN;
			break;
		case GET_TEMP_3:
			conv.flt_val = TEMP(get_adc_single(CH_T3));
			message->data[0] = conv.chr_val[3];
			message->data[1] = conv.chr_val[2];
			message->data[2] = conv.chr_val[1];
			message->data[3] = conv.chr_val[0];
			message->len = FLOAT_LEN;
			break;
		case GET_TEMP_4:
			conv.flt_val = TEMP(get_adc_single(CH_T4));
			message->data[0] = conv.chr_val[3];
			message->data[1] = conv.chr_val[2];
			message->data[2] = conv.chr_val[1];
			message->data[3] = conv.chr_val[0];
			message->len = FLOAT_LEN;
			break;
		case GET_AUX_1:
			conv.flt_val = AUX(get_adc_single(CH_AUX1));
			message->data[0] = conv.chr_val[3];
			message->data[1] = conv.chr_val[2];
			message->data[2] = conv.chr_val[1];
			message->data[3] = conv.chr_val[0];
			message->len = FLOAT_LEN;
			break;
		case GET_AUX_2:
			conv.flt_val = AUX(get_adc_single(CH_AUX2));
			message->data[0] = conv.chr_val[3];
			message->data[1] = conv.chr_val[2];
			message->data[2] = conv.chr_val[1];
			message->data[3] = conv.chr_val[0];
			message->len = FLOAT_LEN;
			break;
		case GET_PRESSURE:
			conv.flt_val = PRESSURE(get_adc_single(CH_PRESS));
			message->data[0] = conv.chr_val[3];
			message->data[1] = conv.chr_val[2];
			message->data[2] = conv.chr_val[1];
			message->data[3] = conv.chr_val[0];
			message->len = FLOAT_LEN;
			break;
		case GET_PRESSURE_ALARM:
			message->data[0] = (pAlarm == LOW) ? HIGH : LOW;
			message->len = BYTE_LEN;
			break; 
		case GET_TEMP_ALARM:
			message->data[0] = (tAlarm == LOW) ? HIGH : LOW;
			message->len = BYTE_LEN;
			break; 
		case GET_DRIVE_INDICATION:
			message->data[0] = (drvInd == HIGH) ? LOW : HIGH;
			message->len = BYTE_LEN;
			break;
		case GET_REVISION_LEVEL:
			message->data[0] = MAJOR;
			message->data[1] = MINOR;
			message->data[2] = PATCH;
			message->len = REVISION_LEN;
			break;
		default:
			break;
	}

	return 0;
}






/* Control requests */
int control_msg(CAN_MSG_TYPE *message) {

	if(message->dirn==CAN_CONTROL){ // If control on control RCA
		switch(message->relative_address){
			case SET_REMOTE_DRIVE:
				rmtDrv = (message->data[0] == LOW) ? LOW : HIGH;
				lastRemoteDrive = message->data[0];
				break;
			case SET_REMOTE_RESET:
				/* Generate an high pulse */
				rmtRst = HIGH;
				rmtRst = LOW;
				lastRemoteReset = message->data[0];
				break;
			default:
				break;
		}
	} else { // If monitor on control RCA
		switch(message->relative_address){
			case SET_REMOTE_DRIVE:
				message->data[0] = lastRemoteDrive;
				message->len = BYTE_LEN;
				break;
			case SET_REMOTE_RESET:
				message->data[0] = lastRemoteReset;
				message->len = BYTE_LEN;
				break;
			default: 
				break;
		}
	}

	return 0;
}








/* Triggers every 48ms pulse */
void received_48ms(void) interrupt 0x30{
// Put whatever you want to be execute at the 48ms clock.
// Remember that right now this interrupt has higher priority than the CAN.
// Also to be able to use the 48ms, the Xilinx has to be programmed to connect the
// incoming pulse on (pin31) to the cpu (pin28).
}


