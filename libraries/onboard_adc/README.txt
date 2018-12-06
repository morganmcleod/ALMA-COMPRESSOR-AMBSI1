For now this is just a partial driver that support only the
"fixed channel single conversion" mode.

Functions:
int adc_init(unsigned char mode, bit wait_read, bit ch_inj_ena, unsigned char conv_time);
Initialize the data acquisition system on the c167.

mode
Is a value from 0x0 to 0x3 and determines the A/D modes:
0 -> Fixed Channel Single Conversion
1 -> Fixed Channel Continuos Conversion (not yet implemented)
2 -> Auto Scan Single Conversion (not yet implemented)
3 -> Auto Scan Continuos Conversion (not yet implemented)

wait_read
If set to 1 will force the ADC to wait for the converted value to be read before starting a
new conversion. (not yet implemented)

ch_inj_ena
Enables the channel injection mode. (not yet implemented)

conv_time
Is a value from 0x00 (98*tcpu) to 0x0F (1666*tcpu). It is not a linear scale.


float get_adc_single(unsigned char channel)
Returns the float (0-5 V) representing the digital conversion for the selected channel

channel
Is the channel to sample (Allowed: 0x0 -> 0xF)



unsigned int get_adc_single_uint(unsigned char channel)
Returns the unsigned int representing the digital conversion for the selected channel

channel
Is the channel to sample (Allowed: 0x0 -> 0xF)



Error description:

The facility descriptor for this library is "ADC"

The errors are:

0x00 -> NoEr -> No Error
0x01 -> ChOR -> Required Channel Out of Range
0x02 -> MdOR -> Required Mode Out of Range
0x03 -> CTOR -> Required Conversion Time Out of Range
