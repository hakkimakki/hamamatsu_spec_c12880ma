/** @file
 * @brief Simple ADC control
 *
 * A simple interface to read adc samples. 
 */

/*
 * Copyright (c) 2021 Raffael Anklin
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SZL_ADC_H_
#define SZL_ADC_H_

#include <zephyr.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief  Read one ADC Channel
 *
 * @param  channel: channel id.
 *
 * @retval adc value or -123 on Bad Read.
 */
int16_t szl_adc_readOneChannel(int channel);

/**
 * @brief  Convert Raw Value to mV
 *
 * @param  raw_value: Raw value as got from szl_adc_readOneChannel.
 *
 * @retval adc value in mV
 */
int16_t szl_adc_raw_to_millivolts(int16_t raw_value);


#ifdef __cplusplus
}
#endif

#endif /* SZL_UART_H_ */
