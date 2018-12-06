/****************************************************************************
 # $Id: main.c,v 1.7 2011/03/02 19:35:09 avaccari Exp $
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
 *	2011-02-28
 *	Rev. 2.0.1
 *	- Lowered interrupt priority for the second counter and the internal 
 *	  monitoring. They are now placed lower than CAN not just in GLVL but
 *	  also in ILVL. This will assure CAN can interrupt them both in case
 *    of arbitration.
 *
 *	2011-02-21c
 *  Rev. 2.0.0
 *  - Updated to comply with CMC module rev.B mounting rev.C PCB
 *  - Implemented Sumitomo timing requirements for hardware operation
 *	- Modified for continuous hardware monitoring
 *	- Modified to allow data on FE cryogenics to be pushed
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
#include <stdio.h>
#include <string.h>

/* include library interface */
#include "..\..\libraries\amb\amb.h"
#include "..\..\libraries\ds1820\ds1820.h"
#include "..\..\libraries\onboard_adc\onboard_adc.h"
#include "serial.h"

/*** Defines ****/
#define USE_48MS	0	// Defines if the 48ms pulse is used to trigger the correponding interrupt
						// If yes then P8.0 will not be available for use as a normal I/O pin since
 					    // it will receive the 48ms pulse.

/* General defines */
#define HIGH	1
#define LOW		0

/* Serial message defines */
#define SIZE_OF_SERIAL_MESSAGE	80

/* Revision Level Defines */
#define	MAJOR	2
#define MINOR	0
#define PATCH	1

/** RCAs **/
/* Monitor */
#define FIRST_MONITOR_RCA				0x00001
#define GET_TEMP_1						0x00001
#define GET_TEMP_2						0x00002
#define GET_TEMP_3						0x00003
#define GET_TEMP_4						0x00004
#define GET_RET_PRESSURE				0x00005
#define GET_AUX_2						0x00006
#define GET_PRESSURE					0x00007
#define GET_PRESSURE_ALARM				0x00008
#define GET_TEMP_ALARM					0x00009
#define GET_DRIVE_INDICATION			0x0000A
#define GET_ICCU_STATUS					0x0000B
#define GET_ICCU_CABLE_DETECT			0x0000C
#define GET_FETIM_STATUS				0x0000D
#define GET_FETIM_CABLE					0x0000E
#define GET_INTERLOCK_OVERRIDE			0x0000F
#define GET_ECU_TYPE					0x00010
#define GET_FAULT_STATUS				0x00011
#define GET_SW_REVISION_LEVEL			0x00012
#define GET_TIME_SINCE_LAST_POWER_ON	0x00013
#define GET_TIME_SINCE_LAST_POWER_OFF	0x00014
#define LAST_MONITOR_RCA				0x00014
/* Control */
#define FIRST_CONTROL_RCA					0x01001
#define SET_REMOTE_DRIVE					0x01001
#define SET_REMOTE_RESET					0x01002
#define SET_FAULT_LATCH_RESET				0x01003
#define SET_PUSH_4K_CRYOCOOLER_TEMP			0x01004
#define SET_PUSH_15K_CRYOCOOLER_TEMP		0x01005
#define SET_PUSH_110K_CRYOCOOLER_TEMP		0x01006
#define SET_PUSH_PORT_PRESSURE				0x01007
#define SET_PUSH_DEWAR_PRESSURE				0x01008
#define SET_PUSH_GATE_VALVE_STATE			0x01009
#define SET_PUSH_SOLENOID_VALVE_STATE		0x0100A
#define SET_PUSH_BACKING_PUMP_ENABLE		0x0100B
#define SET_PUSH_TURBO_PUMP_ENABLE			0x0100C
#define SET_PUSH_TURBO_PUMP_STATE			0x0100D
#define SET_PUSH_TURBO_PUMP_SPEED			0x0100E
#define SET_PUSH_CRYO_SUPPLY_CURRENT_230V	0x0100F
#define SET_BYPASS_TIMERS					0x02000
#define LAST_CONTROL_RCA					0x02000

/* General */
#define BYTE_LEN				1
#define REVISION_LEN			3
#define FLOAT_LEN				4
#define ULONG_LEN				4

/* Analog monitor channels defines */
#define CH_T1		8	// 0->5V => -30->60C
#define CH_T2		9	// 0->5V => -30->60C
#define CH_T3		10	// 0->5V => -30->60C
#define	CH_T4		11	// 0->5V => -30->60C
#define CH_RET_PRES	12	// 0->5V => 0->300psi
#define CH_AUX2		13	// 0->5V
#define CH_PRESS	14	// 0->5V => 0->300psi




/* Analog conversion factors defines */
#define TEMP(V)		((18.0*(V))-30.0)
#define AUX(V)		(V)
#define PRESSURE(V)	((V<1.0)?(-1.0):(V-1.0))

/* Timer values */
#define REMOTE_DRIVE_ON_TIME	180L	// Seconds the compressor has to stay ON
#define REMOTE_DRIVE_OFF_TIME	420L	// Seconds the compressor has to stay OFF


/* Macros */

#define REMOTE_ON_SEC(time)			(time-lastOnSec)
#define REMOTE_OFF_SEC(time)		(time-lastOffSec)
#define REMOTE_ON_MIN(time)			((ubyte)((time-lastOnSec)/60))
#define REMOTE_OFF_MIN(time)		((ubyte)((time-lastOffSec)/60))
#define REMOTE_ON_OK(time)			(((time-lastOffSec)>REMOTE_DRIVE_OFF_TIME)?HIGH:LOW)
#define REMOTE_OFF_OK(time)			(((time-lastOnSec)>REMOTE_DRIVE_ON_TIME)?HIGH:LOW)

#define INVERT(IO)					((IO == LOW) ? HIGH : LOW)	// Invert I/O signals

#define VALVE_STATE(state)			((state==0)?"Closed":((state==1)?"Opened":((state==2)?"Unknown":"Error")))

/* Special register defines */
/* Port 2 */
/* P2.0 */
#define PRES_ALARM			P2^0	// Pressure alarm (0:error) => Invert in fimrware
#define PRES_ALARM_DP		DP2		// Port for pressure alarm
#define	PRES_ALARM_MASK 	0x0001	// Bit mask for pressure alarm
/* P2.2 */
#define TEMP_ALARM			P2^2	// Temperature alarm (0:error) => Invert in firmware
#define TEMP_ALARM_DP		DP2		// Port for temperature alarm
#define	TEMP_ALARM_MASK 	0x0004	// Bit mask for temperature alarm
/* P2.4 */
#define DRIVE_IND			P2^4	// Drive indicator (1:on)
#define DRIVE_IND_DP		DP2		// Port for drive indicator
#define	DRIVE_IND_MASK 		0x0010	// Bit mask for drive indicator
/* P2.5 */ 
#define ICCU_STATUS			P2^5	// ICCU status (0:error) => Invert in firmware
#define ICCU_STATUS_DP		DP2		// Port for ICCU status
#define ICCU_STATUS_MASK	0x0020	// Bit mask for ICCU status
/* P2.6 */
#define ICCU_CABLE			P2^6	// ICCU cable detect (1:error)
#define ICCU_CABLE_DP		DP2		// Port for ICCU cable detect
#define ICCU_CABLE_MASK		0x0040	// Bit mask for ICCU cable detect
/* P2.7 */
#define FETIM_STATUS		P2^7	// FETIM status (0:error) => Invert in firmware
#define FETIM_STATUS_DP		DP2		// Port for FETIM status
#define FETIM_STATUS_MASK	0x0080	// Bit mask for FETIM status
/* P2.8 */
#define FETIM_CABLE			P2^8	// FETIM cable detect (0:error) => Invert in fimrware
#define FETIM_CABLE_DP		DP2		// Port for FETIM cable detect
#define FETIM_CABLE_MASK	0x0100	// Bit mask for FETIM cable detect
/* P2.9 */
#define INTRLK_OVRD			P2^9	// Interlock override (1:enabled)
#define INTRLK_OVRD_DP		DP2		// Port for interlock override
#define INTRLK_OVRD_MASK	0x0200	// Bit mask for interlock override
/* P2.10 */
#define ECU_TYPE			P2^10	// ECU type (0:EU, 1:Jap)
#define ECU_TYPE_DP			DP2		// Port for ECU type
#define ECU_TYPE_MASK		0x0400	// Bit mask for ECU type
/* P2.11 */
#define FAULT_STAT			P2^11	// Fault status (1:error)
#define FAULT_STAT_DP		DP2		// Port for fault status
#define FAULT_STAT_MASK		0x0800	// Bit mask for fault status

/* Port 7 */
/* P7.0 */
#define REMOTE_DRV			P7^0	// Remote drive (1:on)
#define REMOTE_DRV_DP		DP7		// Port for remote drive
#define	REMOTE_DRV_MASK 	0x01	// Bit mask for remote drive
/* P7.1 */
#define FAULT_RST			P7^1	// Fault latch reset (Pulse high)
#define FAULT_RST_DP		DP7		// Port for fault latch reset
#define FAULT_RST_MASK		0x02	// Bit mask for fault latch reset

/* Port 8 */
/* P8.0 */
#define REMOTE_RST			P8^0	// Remote reset (Pulse high)
#define REMOTE_RST_DP		DP8		// Port for remote reset
#define	REMOTE_RST_MASK 	0x01	// Bit mask for remote reset






/* Special function registers */
/* Read */
sbit pAlarm = PRES_ALARM;	// Pressure alarm
sbit tAlarm = TEMP_ALARM;	// Temperature alarm
sbit drvInd = DRIVE_IND;	// Drive indicator
sbit iccuSt = ICCU_STATUS;	// ICCU status
sbit iccuCb = ICCU_CABLE;	// ICCU cable
sbit fetimS = FETIM_STATUS;	// FETIM status
sbit fetimC = FETIM_CABLE;	// FETIM cable
sbit intOvr = INTRLK_OVRD;	// Interlock override
sbit ecuTyp = ECU_TYPE;		// ECU type
sbit faultS = FAULT_STAT;	// Fault status
/* Write */
sbit rmtDrv	= REMOTE_DRV;	// Remote drive
sbit fltRst = FAULT_RST;	// Fault latch reset
sbit rmtRst	= REMOTE_RST;	// Remote reset
/* External bus control signal buffer chip enable is on P4.7 */
sbit DISABLE_EX_BUF	= P4^7;



/* Set aside memory for the callbacks in the AMB library */
static CALLBACK_STRUCT cb_memory[3];




/* Typedefs */
/* A union for conversion float <-> 4 bytes */
typedef union {
	float flt_val;
	ubyte chr_val[4];
	ulong ulng_val;
} CONVERSION;

/* A structure to hold single elements of data */
typedef struct {
	CONVERSION	data;
	ulong		time;
} DATA;

/* enum to address compressor data */
typedef enum {
	comp_temp1,
	comp_temp2,
	comp_temp3,
	comp_temp4,
	comp_ret_pres,
	comp_aux2,
	comp_sup_pres,
	comp_pres_alarm,
	comp_temp_alarm,
	comp_drive_ind,
	comp_iccu_stat,
	comp_iccu_cable,
	comp_fetim_stat,
	comp_fetim_cable,
	comp_intrlk_ovrd,
	comp_ecu_type,
	comp_fault_stat,
	comp_sw_rev,
	comp_time_on,
	comp_time_off,

	comp_min_item=comp_temp1,
	comp_max_item=comp_time_off
} COMP_ITEMS;

/* enum to address cryostat data */
typedef enum {
	cryo_temp_4k,
	cryo_temp_15k,
	cryo_temp_110k,
	cryo_pres_port,
	cryo_pres_dewar,
	cryo_gate_state,
	cryo_sole_state,
	cryo_back_ena,
	cryo_turb_ena,
	cryo_turb_sta,
	cryo_turb_spe,
	cryo_sup_curr,
	
	cryo_min_item=cryo_temp_4k,
	cryo_max_item=cryo_sup_curr
} CRYO_ITEMS;

/* A structure to hold the available data */
typedef struct {
	DATA	comp_data[comp_max_item+1];
	DATA	cryo_data[cryo_max_item+1];
} STATUS;	

/* An enum to define the possible data types */
typedef enum {
	chr_val,
	flt_val,
	rev_val,
	tim_val,
	exp_val
} DATA_TYPE;



/* Prototypes */
void GPT1_vInit(void);
void GPT1_viTmr3(void);
void GPT1_viTmr4(void);
ubyte *buildMessage(ubyte *text, DATA *data, ubyte *units, DATA_TYPE type);


/* CAN message callbacks */
int ambient_msg(CAN_MSG_TYPE *message);  /* Called to get DS1820 temperature */
int monitor_msg(CAN_MSG_TYPE *message);  /* Called to get monitor messages */
int control_msg(CAN_MSG_TYPE *message);  /* Called to set control messages */










/*** Globals ***/
/* A global containing the current status info */
volatile STATUS idata status;

/* A global for conversion of float */
CONVERSION idata conv;

/* A global for the last read temperature */
ubyte ambient_temp_data[4];

/* A couple of globals to keep track of last sent messages */
ubyte lastRemoteDrive = LOW;
ubyte lastRemoteReset = LOW;
ubyte lastFaultLatchReset = LOW;
ubyte lastBypassTimers = LOW;

/* Second counters (look at defined macros before changing the names) */
volatile ulong idata lastOnSec = 0x00000000;
volatile ulong idata lastOffSec = 0x00000000;
volatile ulong idata timerSec = 0x00000000;

ubyte bypassTimers = 0; // Used for troubleshooting the unit when not connected to compressor






/* Main */
void main(void) {

	/* A counter */
	ubyte cnt;

	/* A variable to keep track of time betwen RS232 messages */
	ulong lastMessageTime = timerSec;

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
	DISABLE_EX_BUF = HIGH;




	/* Initialize the slave library */
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
	rmtDrv = LOW; 	// Compressor off
	rmtRst = LOW;	// Reset line default to low. It needs an high pulse to reset the compressor.

	/* Fault latch reset is idle low. It needs an high pulse to reset the compressor. */
	fltRst = HIGH;	
	fltRst = LOW;




	/* Port direction initialization */
	/* Write (set corresponding port direction bits to 1) */
	REMOTE_RST_DP |= REMOTE_RST_MASK;
	REMOTE_DRV_DP |= REMOTE_DRV_MASK;
	FAULT_RST_DP |= FAULT_RST_MASK;
	
	/* Read (set corresponding port direction bits to 0) */
	PRES_ALARM_DP &= ~PRES_ALARM_MASK;
	TEMP_ALARM_DP &= ~TEMP_ALARM_MASK;
	DRIVE_IND_DP &= ~DRIVE_IND_MASK;
	ICCU_STATUS_DP &= ~ICCU_STATUS_MASK;
	ICCU_CABLE_DP &= ~ICCU_CABLE_MASK;
	FETIM_STATUS_DP &= ~FETIM_STATUS_MASK;
	FETIM_CABLE_DP &= ~FETIM_CABLE_MASK;
	INTRLK_OVRD_DP &= ~INTRLK_OVRD_MASK;
	ECU_TYPE_DP	&= ~ECU_TYPE_MASK;
	FAULT_STAT_DP &= ~FAULT_STAT_MASK;




	/* Initialize modules */
	adc_init(0,0,0,0); // ADC initialization
	GPT1_vInit(); // Timer initialization (GPT1, Core T3 and aux T2)
	serialInit('\0'); // Serial interface (termination is only required when receiving)
	



	/* globally enable interrupts */
  	amb_start();



	/* Start timers */
	T3R = 1; // timer 3 run bit is set
	T4R = 1; // timer 4 run bit is set


	/* Never return */
	while (1){
		/* Read the DS1820 when nothing else occurs */
		ds1820_get_temp(&ambient_temp_data[1], &ambient_temp_data[0], &ambient_temp_data[2], &ambient_temp_data[3]);

		/* Write compressor data to RS232 */
		for(cnt=comp_min_item;cnt<comp_max_item+1;cnt++){

			/* Wait 3 second before sending next message */
			while((timerSec-lastMessageTime)<3){};

			/* Wait for serial port to be done transmitting */
			while((*serialGetStatus())&SER_TX_BUSY){};

			switch(cnt){
				case comp_temp1:
					serialWrite(buildMessage("Temperature 1",&status.comp_data[cnt],"C",flt_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_temp2:
					serialWrite(buildMessage("Temperature 2",&status.comp_data[cnt],"C",flt_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_temp3:
					serialWrite(buildMessage("Temperature 3",&status.comp_data[cnt],"C",flt_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_temp4:
					serialWrite(buildMessage("Temperature 4",&status.comp_data[cnt],"C",flt_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_ret_pres:
					serialWrite(buildMessage("Return Pressure",&status.comp_data[cnt],"MPa",flt_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_aux2:
					serialWrite(buildMessage("Aux Input 2",&status.comp_data[cnt],"V",flt_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_sup_pres:
					serialWrite(buildMessage("Supply Pressure",&status.comp_data[cnt],"MPa",flt_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_pres_alarm:
					serialWrite(buildMessage("Pressure Alarm",&status.comp_data[cnt],status.comp_data[cnt].data.chr_val[0]?"Alarm":"Ok",chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_temp_alarm:
					serialWrite(buildMessage("Temperature Alarm",&status.comp_data[cnt],status.comp_data[cnt].data.chr_val[0]?"Alarm":"Ok",chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_drive_ind:
					serialWrite(buildMessage("Drive Indicator",&status.comp_data[cnt],status.comp_data[cnt].data.chr_val[0]?"On":"Off",chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_iccu_stat:
					serialWrite(buildMessage("ICCU Status",&status.comp_data[cnt],status.comp_data[cnt].data.chr_val[0]?"Error":"Ok",chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_iccu_cable:
					serialWrite(buildMessage("ICCU Cable",&status.comp_data[cnt],status.comp_data[cnt].data.chr_val[0]?"Error":"Ok",chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_fetim_stat:
					serialWrite(buildMessage("FETIM Status",&status.comp_data[cnt],status.comp_data[cnt].data.chr_val[0]?"Error":"Ok",chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_fetim_cable:
					serialWrite(buildMessage("FETIM Cable",&status.comp_data[cnt],status.comp_data[cnt].data.chr_val[0]?"Error":"Ok",chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_intrlk_ovrd:
					serialWrite(buildMessage("Interlock Override",&status.comp_data[cnt],status.comp_data[cnt].data.chr_val[0]?"Engaged":"Off",chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_ecu_type:
					serialWrite(buildMessage("ECU Type",&status.comp_data[cnt],status.comp_data[cnt].data.chr_val[0]?"Japanese":"European",chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_fault_stat:
					serialWrite(buildMessage("Compressor Fault",&status.comp_data[cnt],status.comp_data[cnt].data.chr_val[0]?"Error":"Ok",chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_sw_rev:
					serialWrite(buildMessage("Software Revision",&status.comp_data[cnt],"",rev_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_time_on:
					serialWrite(buildMessage("Time since last ON",&status.comp_data[cnt],"min",tim_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case comp_time_off:
					serialWrite(buildMessage("Time since last OFF",&status.comp_data[cnt],"min",tim_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				default:
					break;
			}

		lastMessageTime=timerSec;
	
		}

					
		/* Write cryostat (from FEMC) data to RS232 */
		for(cnt=cryo_min_item;cnt<cryo_max_item+1;cnt++){

			/* Wait 3 second before sending next message */
			while((timerSec-lastMessageTime)<3){};

			/* Wait for serial port to be done transmitting */
			while((*serialGetStatus())&SER_TX_BUSY){};

			switch(cnt){
				case cryo_temp_4k:
					serialWrite(buildMessage("4K stage",&status.cryo_data[cnt],"K",flt_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case cryo_temp_15k:
					serialWrite(buildMessage("15K stage",&status.cryo_data[cnt],"K",flt_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case cryo_temp_110k:
					serialWrite(buildMessage("110K stage",&status.cryo_data[cnt],"K",flt_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case cryo_pres_port:
					serialWrite(buildMessage("Port Pressure",&status.cryo_data[cnt],"mbar",flt_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case cryo_pres_dewar:
					serialWrite(buildMessage("Dewar Pressure",&status.cryo_data[cnt],"mbar",exp_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case cryo_gate_state:
					serialWrite(buildMessage("Gate Valve",&status.cryo_data[cnt],VALVE_STATE(status.cryo_data[cnt].data.chr_val[0]),chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case cryo_sole_state:
					serialWrite(buildMessage("Solenoid Valve",&status.cryo_data[cnt],VALVE_STATE(status.cryo_data[cnt].data.chr_val[0]),chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case cryo_back_ena:
					serialWrite(buildMessage("Backing Pump",&status.cryo_data[cnt],status.cryo_data[cnt].data.chr_val[0]?"On":"Off",chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case cryo_turb_ena:
					serialWrite(buildMessage("Turbo Pump",&status.cryo_data[cnt],status.cryo_data[cnt].data.chr_val[0]?"On":"Off",chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case cryo_turb_sta:
					serialWrite(buildMessage("Turbo Pump Status",&status.cryo_data[cnt],status.cryo_data[cnt].data.chr_val[0]?"Error":"Ok",chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case cryo_turb_spe:
					serialWrite(buildMessage("Turbo Pump Speed",&status.cryo_data[cnt],status.cryo_data[cnt].data.chr_val[0]?"Up to speed":"Low",chr_val),SIZE_OF_SERIAL_MESSAGE);
					break;
				case cryo_sup_curr:
					serialWrite(buildMessage("FE 230V Current",&status.cryo_data[cnt],"A",flt_val),SIZE_OF_SERIAL_MESSAGE);
					break;

				default:
					break;
			}

		lastMessageTime=timerSec;
	
		}


	}
}



/* Buil the serial message and returns a pointer to it */
ubyte *buildMessage(ubyte *text, DATA *data, ubyte *units, DATA_TYPE type){

	/* A static buffer */
	static ubyte idata message[SIZE_OF_SERIAL_MESSAGE];

	/* Some locals */
	ubyte	ageOfData;

	/* Clear message */
	memset(message,'\0',SIZE_OF_SERIAL_MESSAGE);
	
	ageOfData = (timerSec - data->time)/60;
	
	if(ageOfData>=255){
		ageOfData=255;
	}

	switch(type){
		case chr_val:
			sprintf(message,"%s\r\nValue: %u %s\r\nAge: %u min\r\n\r\n",text,data->data.chr_val[0],units,ageOfData);
			break;
		case flt_val:
			sprintf(message,"%s\r\nValue: %5.2f %s\r\nAge: %u min\r\n\r\n",text,data->data.flt_val,units,ageOfData);
			break;
		case exp_val:
			sprintf(message,"%s\r\nValue: %5.2E %s\r\nAge: %u min\r\n\r\n",text,data->data.flt_val,units,ageOfData);
			break;
		case rev_val:
			sprintf(message,"%s\r\nValue: %u.%u.%u %s\r\nAge: %u min\r\n\r\n",text,data->data.chr_val[0],data->data.chr_val[1],data->data.chr_val[2],units,ageOfData);
			break;
		case tim_val:
			sprintf(message,"%s\r\nValue: %lu %s\r\nAge: %u min\r\n\r\n",text,(data->data.ulng_val/60),units,ageOfData);
			break;
		default:
			break;
	}	

	return message;

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
			message->data[0] = status.comp_data[comp_temp1].data.chr_val[3];
			message->data[1] = status.comp_data[comp_temp1].data.chr_val[2];
			message->data[2] = status.comp_data[comp_temp1].data.chr_val[1];
			message->data[3] = status.comp_data[comp_temp1].data.chr_val[0];
			message->len = FLOAT_LEN;
			break;

		case GET_TEMP_2:
			message->data[0] = status.comp_data[comp_temp2].data.chr_val[3];
			message->data[1] = status.comp_data[comp_temp2].data.chr_val[2];
			message->data[2] = status.comp_data[comp_temp2].data.chr_val[1];
			message->data[3] = status.comp_data[comp_temp2].data.chr_val[0];
			message->len = FLOAT_LEN;
			break;

		case GET_TEMP_3:
			message->data[0] = status.comp_data[comp_temp3].data.chr_val[3];
			message->data[1] = status.comp_data[comp_temp3].data.chr_val[2];
			message->data[2] = status.comp_data[comp_temp3].data.chr_val[1];
			message->data[3] = status.comp_data[comp_temp3].data.chr_val[0];
			message->len = FLOAT_LEN;
			break;

		case GET_TEMP_4:
			message->data[0] = status.comp_data[comp_temp4].data.chr_val[3];
			message->data[1] = status.comp_data[comp_temp4].data.chr_val[2];
			message->data[2] = status.comp_data[comp_temp4].data.chr_val[1];
			message->data[3] = status.comp_data[comp_temp4].data.chr_val[0];
			message->len = FLOAT_LEN;
			break;

		case GET_RET_PRESSURE:
			message->data[0] = status.comp_data[comp_ret_pres].data.chr_val[3];
			message->data[1] = status.comp_data[comp_ret_pres].data.chr_val[2];
			message->data[2] = status.comp_data[comp_ret_pres].data.chr_val[1];
			message->data[3] = status.comp_data[comp_ret_pres].data.chr_val[0];
			message->len = FLOAT_LEN;
			break;

		case GET_AUX_2:
			message->data[0] = status.comp_data[comp_aux2].data.chr_val[3];
			message->data[1] = status.comp_data[comp_aux2].data.chr_val[2];
			message->data[2] = status.comp_data[comp_aux2].data.chr_val[1];
			message->data[3] = status.comp_data[comp_aux2].data.chr_val[0];
			message->len = FLOAT_LEN;
			break;

		case GET_PRESSURE:
			message->data[0] = status.comp_data[comp_sup_pres].data.chr_val[3];
			message->data[1] = status.comp_data[comp_sup_pres].data.chr_val[2];
			message->data[2] = status.comp_data[comp_sup_pres].data.chr_val[1];
			message->data[3] = status.comp_data[comp_sup_pres].data.chr_val[0];
			message->len = FLOAT_LEN;
			break;

		case GET_PRESSURE_ALARM:
			message->data[0] = status.comp_data[comp_pres_alarm].data.chr_val[0];
			message->len = BYTE_LEN;
			break; 

		case GET_TEMP_ALARM:
			message->data[0] = status.comp_data[comp_temp_alarm].data.chr_val[0];
			message->len = BYTE_LEN;
			break; 

		case GET_DRIVE_INDICATION:
			message->data[0] = status.comp_data[comp_drive_ind].data.chr_val[0];
			message->len = BYTE_LEN;
			break;

		case GET_ICCU_STATUS:
			message->data[0] = status.comp_data[comp_iccu_stat].data.chr_val[0];
			message->len = BYTE_LEN;
			break;

		case GET_ICCU_CABLE_DETECT:
			message->data[0] = status.comp_data[comp_iccu_cable].data.chr_val[0];
			message->len = BYTE_LEN;
			break;

		case GET_FETIM_STATUS:
			message->data[0] = status.comp_data[comp_fetim_stat].data.chr_val[0];
			message->len = BYTE_LEN;
			break;

		case GET_FETIM_CABLE:
			message->data[0] = status.comp_data[comp_fetim_cable].data.chr_val[0];
			message->len = BYTE_LEN;
			break;

		case GET_INTERLOCK_OVERRIDE:
			message->data[0] = status.comp_data[comp_intrlk_ovrd].data.chr_val[0];
			message->len = BYTE_LEN;
			break;

		case GET_ECU_TYPE:
			message->data[0] = status.comp_data[comp_ecu_type].data.chr_val[0];
			message->len = BYTE_LEN;
			break;

		case GET_FAULT_STATUS:
			message->data[0] = status.comp_data[comp_fault_stat].data.chr_val[0];
			message->len = BYTE_LEN;
			break;

		case GET_SW_REVISION_LEVEL:
			message->data[0] = status.comp_data[comp_sw_rev].data.chr_val[0];
			message->data[1] = status.comp_data[comp_sw_rev].data.chr_val[1];
			message->data[2] = status.comp_data[comp_sw_rev].data.chr_val[2];
			message->len = REVISION_LEN;
			break;

		case GET_TIME_SINCE_LAST_POWER_ON:
			if(REMOTE_OFF_OK(timerSec)){
				message->data[0] = 0xFF;
				message->data[1] = 0xFF;
				message->data[2] = 0xFF;
				message->data[3] = 0xFF;
			} else {
				message->data[0] = status.comp_data[comp_time_on].data.chr_val[3];
				message->data[1] = status.comp_data[comp_time_on].data.chr_val[2];
				message->data[2] = status.comp_data[comp_time_on].data.chr_val[1];
				message->data[3] = status.comp_data[comp_time_on].data.chr_val[0];
			}
			message->len = ULONG_LEN;
			break;

		case GET_TIME_SINCE_LAST_POWER_OFF:
			if(REMOTE_ON_OK(timerSec)){
				message->data[0] = 0xFF;
				message->data[1] = 0xFF;
				message->data[2] = 0xFF;
				message->data[3] = 0xFF;
			} else {
				message->data[0] = status.comp_data[comp_time_off].data.chr_val[3];
				message->data[1] = status.comp_data[comp_time_off].data.chr_val[2];
				message->data[2] = status.comp_data[comp_time_off].data.chr_val[1];
				message->data[3] = status.comp_data[comp_time_off].data.chr_val[0];
			}
			message->len = ULONG_LEN;
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
				switch(message->data[0]){
					case LOW:
						if(bypassTimers||((rmtDrv==HIGH)&&REMOTE_OFF_OK(timerSec))){
							rmtDrv = LOW;
							lastOffSec=timerSec;
						}
						break;
					case HIGH:
						if(bypassTimers||((rmtDrv==LOW)&&REMOTE_ON_OK(timerSec))){
							rmtDrv = HIGH;
							lastOnSec=timerSec;
						}
						break;
					default:
						break;
				}
				lastRemoteDrive = message->data[0];
				break;

			case SET_REMOTE_RESET:
				/* Generate a high pulse */
				rmtRst = HIGH;
				rmtRst = LOW;
				lastRemoteReset = message->data[0];
				break;

			case SET_FAULT_LATCH_RESET:
				/* Generate a high pulse */
				fltRst = HIGH;
				fltRst = LOW;
				lastFaultLatchReset = message->data[0];
				break;

			case SET_PUSH_4K_CRYOCOOLER_TEMP:
				status.cryo_data[cryo_temp_4k].data.chr_val[0]=message->data[3];
				status.cryo_data[cryo_temp_4k].data.chr_val[1]=message->data[2];
				status.cryo_data[cryo_temp_4k].data.chr_val[2]=message->data[1];
				status.cryo_data[cryo_temp_4k].data.chr_val[3]=message->data[0];
				status.cryo_data[cryo_temp_4k].time=timerSec;
				break;

			case SET_PUSH_15K_CRYOCOOLER_TEMP:
				status.cryo_data[cryo_temp_15k].data.chr_val[0]=message->data[3];
				status.cryo_data[cryo_temp_15k].data.chr_val[1]=message->data[2];
				status.cryo_data[cryo_temp_15k].data.chr_val[2]=message->data[1];
				status.cryo_data[cryo_temp_15k].data.chr_val[3]=message->data[0];
				status.cryo_data[cryo_temp_15k].time=timerSec;
				break;

			case SET_PUSH_110K_CRYOCOOLER_TEMP:
				status.cryo_data[cryo_temp_110k].data.chr_val[0]=message->data[3];
				status.cryo_data[cryo_temp_110k].data.chr_val[1]=message->data[2];
				status.cryo_data[cryo_temp_110k].data.chr_val[2]=message->data[1];
				status.cryo_data[cryo_temp_110k].data.chr_val[3]=message->data[0];
				status.cryo_data[cryo_temp_110k].time=timerSec;
				break;

			case SET_PUSH_PORT_PRESSURE:
				status.cryo_data[cryo_pres_port].data.chr_val[0]=message->data[3];
				status.cryo_data[cryo_pres_port].data.chr_val[1]=message->data[2];
				status.cryo_data[cryo_pres_port].data.chr_val[2]=message->data[1];
				status.cryo_data[cryo_pres_port].data.chr_val[3]=message->data[0];
				status.cryo_data[cryo_pres_port].time=timerSec;
				break;

			case SET_PUSH_DEWAR_PRESSURE:
				status.cryo_data[cryo_pres_dewar].data.chr_val[0]=message->data[3];
				status.cryo_data[cryo_pres_dewar].data.chr_val[1]=message->data[2];
				status.cryo_data[cryo_pres_dewar].data.chr_val[2]=message->data[1];
				status.cryo_data[cryo_pres_dewar].data.chr_val[3]=message->data[0];
				status.cryo_data[cryo_pres_dewar].time=timerSec;
				break;

			case SET_PUSH_GATE_VALVE_STATE:
				status.cryo_data[cryo_gate_state].data.chr_val[0]=message->data[0];
				status.cryo_data[cryo_gate_state].time=timerSec;
				break;

			case SET_PUSH_SOLENOID_VALVE_STATE:
				status.cryo_data[cryo_sole_state].data.chr_val[0]=message->data[0];
				status.cryo_data[cryo_sole_state].time=timerSec;
				break;

			case SET_PUSH_BACKING_PUMP_ENABLE:
				status.cryo_data[cryo_back_ena].data.chr_val[0]=message->data[0];
				status.cryo_data[cryo_back_ena].time=timerSec;
				break;

			case SET_PUSH_TURBO_PUMP_ENABLE:
				status.cryo_data[cryo_turb_ena].data.chr_val[0]=message->data[0];
				status.cryo_data[cryo_turb_ena].time=timerSec;
				break;

			case SET_PUSH_TURBO_PUMP_STATE:
				status.cryo_data[cryo_turb_sta].data.chr_val[0]=message->data[0];
				status.cryo_data[cryo_turb_sta].time=timerSec;
				break;

			case SET_PUSH_TURBO_PUMP_SPEED:
				status.cryo_data[cryo_turb_spe].data.chr_val[0]=message->data[0];
				status.cryo_data[cryo_turb_spe].time=timerSec;
				break;

			case SET_PUSH_CRYO_SUPPLY_CURRENT_230V:
				status.cryo_data[cryo_sup_curr].data.chr_val[0]=message->data[3];
				status.cryo_data[cryo_sup_curr].data.chr_val[1]=message->data[2];
				status.cryo_data[cryo_sup_curr].data.chr_val[2]=message->data[1];
				status.cryo_data[cryo_sup_curr].data.chr_val[3]=message->data[0];
				status.cryo_data[cryo_sup_curr].time=timerSec;
				break;

			/* For troubleshooting use only RCA 0x2000 */
			case SET_BYPASS_TIMERS:
				bypassTimers = message->data[0];
				lastBypassTimers = message->data[0];
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

			case SET_FAULT_LATCH_RESET:
				message->data[0] = lastFaultLatchReset;
				message->len = BYTE_LEN;
				break;

			case SET_PUSH_4K_CRYOCOOLER_TEMP:
				message->data[3]=status.cryo_data[cryo_temp_4k].data.chr_val[0];
				message->data[2]=status.cryo_data[cryo_temp_4k].data.chr_val[1];
				message->data[1]=status.cryo_data[cryo_temp_4k].data.chr_val[2];
				message->data[0]=status.cryo_data[cryo_temp_4k].data.chr_val[3];
				message->len = FLOAT_LEN;
				break;

			case SET_PUSH_15K_CRYOCOOLER_TEMP:
				message->data[3]=status.cryo_data[cryo_temp_15k].data.chr_val[0];
				message->data[2]=status.cryo_data[cryo_temp_15k].data.chr_val[1];
				message->data[1]=status.cryo_data[cryo_temp_15k].data.chr_val[2];
				message->data[0]=status.cryo_data[cryo_temp_15k].data.chr_val[3];
				message->len = FLOAT_LEN;
				break;

			case SET_PUSH_110K_CRYOCOOLER_TEMP:
				message->data[3]=status.cryo_data[cryo_temp_110k].data.chr_val[0];
				message->data[2]=status.cryo_data[cryo_temp_110k].data.chr_val[1];
				message->data[1]=status.cryo_data[cryo_temp_110k].data.chr_val[2];
				message->data[0]=status.cryo_data[cryo_temp_110k].data.chr_val[3];
				message->len = FLOAT_LEN;
				break;

			case SET_PUSH_PORT_PRESSURE:
				message->data[3]=status.cryo_data[cryo_pres_port].data.chr_val[0];
				message->data[2]=status.cryo_data[cryo_pres_port].data.chr_val[1];
				message->data[1]=status.cryo_data[cryo_pres_port].data.chr_val[2];
				message->data[0]=status.cryo_data[cryo_pres_port].data.chr_val[3];
				message->len = FLOAT_LEN;
				break;

			case SET_PUSH_DEWAR_PRESSURE:
				message->data[3]=status.cryo_data[cryo_pres_dewar].data.chr_val[0];
				message->data[2]=status.cryo_data[cryo_pres_dewar].data.chr_val[1];
				message->data[1]=status.cryo_data[cryo_pres_dewar].data.chr_val[2];
				message->data[0]=status.cryo_data[cryo_pres_dewar].data.chr_val[3];
				message->len = FLOAT_LEN;
				break;

			case SET_PUSH_GATE_VALVE_STATE:
				message->data[0]=status.cryo_data[cryo_gate_state].data.chr_val[0];
				message->len = BYTE_LEN;
				break;

			case SET_PUSH_SOLENOID_VALVE_STATE:
				message->data[0]=status.cryo_data[cryo_sole_state].data.chr_val[0];
				message->len = BYTE_LEN;
				break;

			case SET_PUSH_BACKING_PUMP_ENABLE:
				message->data[0]=status.cryo_data[cryo_back_ena].data.chr_val[0];
				message->len = BYTE_LEN;
				break;

			case SET_PUSH_TURBO_PUMP_ENABLE:
				message->data[0]=status.cryo_data[cryo_turb_ena].data.chr_val[0];
				message->len = BYTE_LEN;
				break;

			case SET_PUSH_TURBO_PUMP_STATE:
				message->data[0]=status.cryo_data[cryo_turb_sta].data.chr_val[0];
				message->len = BYTE_LEN;
				break;

			case SET_PUSH_TURBO_PUMP_SPEED:
				message->data[0]=status.cryo_data[cryo_turb_spe].data.chr_val[0];
				message->len = BYTE_LEN;
				break;

			case SET_PUSH_CRYO_SUPPLY_CURRENT_230V:
				message->data[3]=status.cryo_data[cryo_sup_curr].data.chr_val[0];
				message->data[2]=status.cryo_data[cryo_sup_curr].data.chr_val[1];
				message->data[1]=status.cryo_data[cryo_sup_curr].data.chr_val[2];
				message->data[0]=status.cryo_data[cryo_sup_curr].data.chr_val[3];
				message->len = FLOAT_LEN;
				break;

			/* For troubleshooting use only RCA 0x2000 */
			case SET_BYPASS_TIMERS:
				message->data[0] = lastBypassTimers;
				break;

			default: 
				break;
		}
	}

	return 0;
}









/* Triggers every 48ms pulse */
void received_48ms(void) interrupt 0x30 {
	// Put whatever you want to be execute at the 48ms clock.
	// Remember that right now this interrupt has higher priority than the CAN.
	// Also to be able to use the 48ms, the Xilinx has to be programmed to connect the
	// incoming pulse on (pin31) to the cpu (pin28).
}







/* Configures Timer 3 to overflow every 1 sec */
void GPT1_vInit(void)
{
  ///  -----------------------------------------------------------------------
  ///  Configuration of the GPT1 Core Timer 3:
  ///  -----------------------------------------------------------------------
  ///  - timer 3 works in timer mode
  ///  - maximum input frequency for timer 3 is 2.5 MHz
  ///  - prescaler factor is 512
  ///  - up/down control bit is reset
  ///  - external up/down control is disabled
  ///  - alternate output function T3OUT (P3.3) is disabled
  ///  - timer 3 output toggle latch (T3OTL) is set to 0
  ///  - timer 3 run bit is reset

  T3CON          =  0x0006;      // load timer 3 control register
  T3             =  0x676A;      // load timer 3 register

  ///  -----------------------------------------------------------------------
  ///  Configuration of the GPT1 Core Timer 4:
  ///  -----------------------------------------------------------------------
  ///  - timer 4 works in timer mode
  ///  - maximum input frequency for timer 4 is 2.5 MHz
  ///  - prescaler factor is 32
  ///  - up/down control bit is reset
  ///  - external up/down control is disabled
  ///  - timer 4 run bit is reset

  T4CON          =  0x0002;      // load timer 4 control register
  T4             =  0x0000;      // load timer 4 register

  ///  -----------------------------------------------------------------------
  ///  Configuration of the used GPT1 Port Pins:
  ///  -----------------------------------------------------------------------


  ///  -----------------------------------------------------------------------
  ///  Configuration of the used GPT1 Interrupts:
  ///  -----------------------------------------------------------------------
  ///  - timer 3 service request node configuration:
  ///  - timer 3 interrupt priority level (ILVL) = 12
  ///  - timer 3 interrupt group level (GLVL) = 3

  T3IC           =  0x0073;     

  ///  - timer 4 service request node configuration:
  ///  - timer 4 interrupt priority level (ILVL) = 11
  ///  - timer 4 interrupt group level (GLVL) = 3

  T4IC           =  0x006F;     


} //  End of function GPT1_vInit








/* Triggered every time Timer 3 overflows (~1 sec) */
void GPT1_viTmr3(void) interrupt 0x23 {
	
	T3 =  0x676A;      // reload timer 3 register
	
	/* Increase seconds timer */
	timerSec++;

} 



/* Triggered every time Timer 4 overflows (~105 msec) */
void GPT1_viTmr4(void) interrupt 0x24 {

	/* A local counter for loops */
	ubyte cnt;

	/* Loop over the compressor monitor points and store the values */
	for(cnt=comp_min_item;cnt<comp_max_item+1;cnt++){
		switch(cnt){
			case comp_temp1:
				status.comp_data[cnt].data.flt_val = TEMP(get_adc_single(CH_T1));
				break;
			case comp_temp2:
				status.comp_data[cnt].data.flt_val = TEMP(get_adc_single(CH_T2));
				break;
			case comp_temp3:
				status.comp_data[cnt].data.flt_val = TEMP(get_adc_single(CH_T3));
				break;
			case comp_temp4:
				status.comp_data[cnt].data.flt_val = TEMP(get_adc_single(CH_T4));
				break;
			case comp_ret_pres:
				status.comp_data[cnt].data.flt_val = PRESSURE(get_adc_single(CH_RET_PRES));
				break;
			case comp_aux2:
				status.comp_data[cnt].data.flt_val = AUX(get_adc_single(CH_AUX2));
				break;
			case comp_sup_pres:
				status.comp_data[cnt].data.flt_val = PRESSURE(get_adc_single(CH_PRESS));
				break;
			case comp_pres_alarm:
				status.comp_data[cnt].data.chr_val[0] = INVERT(pAlarm);
				break;
			case comp_temp_alarm:
				status.comp_data[cnt].data.chr_val[0] = INVERT(tAlarm);
				break;
			case comp_drive_ind:
				status.comp_data[cnt].data.chr_val[0] = drvInd;
				break;
			case comp_iccu_stat:
				status.comp_data[cnt].data.chr_val[0] = INVERT(iccuSt);
				break;
			case comp_iccu_cable:
				status.comp_data[cnt].data.chr_val[0] = iccuCb;
				break;
			case comp_fetim_stat:
				status.comp_data[cnt].data.chr_val[0] = INVERT(fetimS);
				break;
			case comp_fetim_cable:
				status.comp_data[cnt].data.chr_val[0] = INVERT(fetimC);
				break;
			case comp_intrlk_ovrd:
				status.comp_data[cnt].data.chr_val[0] = intOvr;
				break;
			case comp_ecu_type:
				status.comp_data[cnt].data.chr_val[0] = ecuTyp;
				break;
			case comp_fault_stat:
				status.comp_data[cnt].data.chr_val[0] = faultS;
				break;
			case comp_sw_rev:
				status.comp_data[cnt].data.chr_val[0] = MAJOR;
				status.comp_data[cnt].data.chr_val[1] = MINOR;
				status.comp_data[cnt].data.chr_val[2] = PATCH;
				break;
			case comp_time_on:
				status.comp_data[cnt].data.ulng_val = REMOTE_ON_SEC(timerSec);
				break;
			case comp_time_off:
				status.comp_data[cnt].data.ulng_val = REMOTE_OFF_SEC(timerSec);
				break;
			default:
				break;
		}

		status.comp_data[cnt].time = timerSec; // store time info
	}
}

