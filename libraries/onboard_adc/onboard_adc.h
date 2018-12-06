#ifndef	ONBOARD_ADC_H

	#define ONBOARD_ADC_H

	/* Prototypes */
	extern int adc_init(unsigned char mode, bit wait_read, bit ch_inj_ena, unsigned char conv_time);
	extern float get_adc_single(unsigned char channel);
    extern unsigned int get_adc_single_uint(unsigned char channel);

#endif
