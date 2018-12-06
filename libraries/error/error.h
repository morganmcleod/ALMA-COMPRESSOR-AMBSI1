#ifndef ERROR_H

	#define ERROR_H

	#ifndef AMB_H

		/* An enum for CAN message direction */
		typedef enum {	CAN_MONITOR,
						CAN_CONTROL
		} CAN_DIRN_TYPE;

		/* Configuration and current data structure for CAN messages */
		typedef struct {
			unsigned long			relative_address;	/* CAN ID - Slave base address */
			unsigned char			data[8];			/* Current value */
			unsigned char			len;				/* Amount of data */
			CAN_DIRN_TYPE	dirn;						/* Direction of message */
		} CAN_MSG_TYPE;

	#endif
	/* Externs */
	extern unsigned char	error_status[8];
	
	/* Typedefs */
	typedef struct{
		unsigned char		error_no;
		unsigned char		description[4];
	} FACILITY_ERROR_ARRAY;

	/* Prototypes */
	extern int				init_error_handler(unsigned char devices_no);
	extern unsigned char	reg_facility(unsigned char *facility_descr, FACILITY_ERROR_ARRAY *facility_error_array);
	extern unsigned char 	report_error(unsigned char facility_no, unsigned char error_no);
	extern void				write_error_CAN(CAN_MSG_TYPE *can_msg);
	extern void				clear_error(void);

#endif
