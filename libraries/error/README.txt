This library should be included in the build and linked before any other library if the link
orded has some effect and the header file should always come after the amb.h if present to
avoid redefinition errors.
It is initialized with a call from main to the function

int init_error_handler(ubyte devices_no)

declaring the number of different devices (max 8) which will return errors (max 32).
If eny error occurs, the global error_status[8] will contain a brief 7 characters
description of the error and an error number given by the following algorithm

error_status[7]=(facility_no<<5)|(error_no&0x1F)

where the facility_no is 0 for the error library itself and is assigned to the other
facility when they register. The error number is the index into the array of the error
list of the facility itself.
The registration of every single devices (facility) should be done during the initialization
of the device itself via a call to the function

ubyte reg_facility(ubyte *facility_descr, FACILITY_ERROR_ARRAY *facility_error_array)

if the parameters of this function are correct and the error library registered correctly
this function will return the facility number to be used in subsequent call to the library.
If any error occurs during this process a number describing the error will be returned instead.
In case of error a call should be done to the following function

ubyte report_error(ubyte facility_no, ubyte error_no)

this function will return the error code and will fill up the global error_status[8].
It is possible to write the error_status array into a can message using the function

void write_error_CAN(CAN_MSG_TYPE *can_msg)

The mnemonics will be written in the first 7 byte of the payload, the error code will be
written in the last byte.


The global error_status[8] is declare as an extern in the header file.

Every device should contain the following static globals

FACILITY_ERROR_ARRAY static	facility_error_array[]=...	containing the array of errors
ubyte static			facility_descr[3]=...		containing the three characters
								which describes the facility
ubyte static			facility_no			containing the facility number
								assigned by the registration
								function

This library will not work in tiny memory models because of the function malloc.
It will not work properly when using the monitor model.

Error description:

The facility descriptor for this library is "Err"

The errors are:

0x0 -> NoEr -> No Error
0x1 -> NMem -> Not Enough Memory to initialize the error library
0x2 -> AIni -> Attempt to initialize the error library more than once
0x3 -> NIni -> Attempt to utilize the error library before initialization
0x4 -> DMax -> Attempt to register more than 8 devices (maximum number allowed)
0x5 -> DNRg -> Attempt to register more devices than the ones specified in the initialization of the library



