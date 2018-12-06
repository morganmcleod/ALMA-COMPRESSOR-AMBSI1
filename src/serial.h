#ifndef _SERIAL_H
	
	#define _SERIAL_H

	/* Defines */
	#define SER_TX_BUF_SIZE		80

	/* Defines to help with driver status */
	#define SER_TX_BUSY		0x01
	#define SER_TX_BUF_OVR	0x04


	/* Defines to help with bytes */
	#define LF		0x0A	// Line Feed
	#define CR		0x0D	// Carriage Return
	
	/* Prototypes */
	/* Externs */
	extern void serialInit(char termination);	
	extern void serialWrite(void *message, unsigned char size);
	extern char volatile *serialGetStatus(void);

#endif /* _SERIAL_H */
