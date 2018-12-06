#include <reg167.h>
#include <intrins.h>
#include <string.h>

#include "serial.h"

/* Static */
static char idata serialTxBuffer[SER_TX_BUF_SIZE];

static char msgTerm;

static char volatile bdata serialStatus;
sbit serialTxBusy = serialStatus^0;
sbit serialTxBufOvr	= serialStatus^2;



/* Initialize serial port */
void serialInit(char termination){

	/* Set port 3 to handle serial communication */
	P3  |= 0x0400;        /* SET PORT 3.10 OUTPUT LATCH (TXD)              */
	DP3 |= 0x0400;        /* SET PORT 3.10 DIRECTION CONTROL (TXD OUTPUT)  */
	DP3 &= 0xF7FF;        /* RESET PORT 3.11 DIRECTION CONTROL (RXD INPUT) */

	/* Set serial mode */	
	S0BG  = 0x20;         /* SET BAUDRATE TO 19200 BAUD                    */
	S0CON = 0x8011;       /* SET SERIAL MODE:
								- asynchronous
								- 8-N-1
								- rx enabled
								- baud generator enabled */
	
	/* Set interrupts */
	S0TIC = 0x0078;       /* SET TRANSMIT INTERRUPT:
								- Tx Irq flag cleared
								- Tx Irq enable
								- PEC service channel 0:
									- ILVL = 14
									- GLVL = 0 */

	/* Setup termination character */
	msgTerm = termination;

	/* Set up PEC channels */
	/* Tx: PEC channel 0 */
	DSTP0 = (unsigned int)&S0TBUF;		/* DSTP0 points to the hardware tx buffer */

}





/* Writes the message to the serial port */
void serialWrite(void *message, unsigned char size){

	/* Clear the Tx buffer overrun if any */
	serialTxBufOvr=0;

	/* If the size of the message is bigger than the buffer, return */
	if(size>SER_TX_BUF_SIZE){
		serialTxBufOvr=1;
		return;
	}

	/* Copy message to the serial Tx buffer */
	memcpy(serialTxBuffer, message, (unsigned int)size);

	/* Setup PEC channel 0 */
	PECC0 = 0x0500 | size; 		/* Transfer 1 byte, increment SRCP0 and send a total of *size bytes */	
	SRCP0 = _sof_ (serialTxBuffer);	/* SRCP0 points to the transmit buffer */
	
	/* Notify the status that the serial Tx is busy */
	serialTxBusy=1;

	/* Trigger the PEC transfer by raising the Tx Irq Flag */
	S0TIR=1;
}



/* Serial Tx interrupt service routine */
void serialTxIrq(void) interrupt S0TINT = 42 {

	/* Notify the status that the serial Tx is done */
	serialTxBusy=0;
	
}


/* Get serial interface status */
char volatile *serialGetStatus(void){
	return &serialStatus;
}
