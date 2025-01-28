/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"



#define PH4_ADC 1638252
#define PH7_ADC 2054624
#define PH10_ADC 2439090



float calculate_ph(int adc_value);
int smooth_adc_read(cyhal_adc_channel_t *channel, int samples);

int main(void)
{
    cy_rslt_t result;
    cyhal_adc_t adc_obj;
    cyhal_adc_channel_t adc_chan_0_obj;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    printf("pH Sensor Application Initialized.\r\n");

    /* ADC conversion result. */
    int adc_out;
    float ph_value;

    /* Initialize ADC */
    result = cyhal_adc_init(&adc_obj, P10_0, NULL);

    // ADC configuration structure
    const cyhal_adc_config_t ADCconfig = {
        .continuous_scanning = false,
        .resolution = 12,
        .average_count = 1,
        .average_mode_flags = 0,
        .ext_vref_mv = 0,
        .vneg = CYHAL_ADC_VNEG_VREF,
        .vref = CYHAL_ADC_REF_VDDA,
        .ext_vref = NC,
        .is_bypassed = false,
        .bypass_pin = NC
    };

    // Configure ADC
    result = cyhal_adc_configure(&adc_obj, &ADCconfig);

    /* Initialize ADC channel */
    const cyhal_adc_channel_config_t channel_config = {
        .enable_averaging = false,
        .min_acquisition_ns = 220,
        .enabled = true
    };
    result = cyhal_adc_channel_init_diff(&adc_chan_0_obj, &adc_obj, P10_0, CYHAL_ADC_VNEG, &channel_config);

    for (;;)
    {
        /* Read smoothed ADC conversion result */
        adc_out = smooth_adc_read(&adc_chan_0_obj, 10); // Average over 10 readings

        // Calculate the pH value
        ph_value = calculate_ph(adc_out);

        // Print ADC and pH values
        printf("ADC = %d, pH = %.2f\r\n", adc_out, ph_value);

        cyhal_system_delay_ms(500);
    }
}

/**
 * @brief Calculate pH value based on ADC reading.
 *
 * @param adc_value The raw ADC value.
 * @return The calculated pH value.
 */
float calculate_ph(int adc_value)
{
    float ph_value;

    if (adc_value <= PH4_ADC)
    {
        // Extrapolate below pH 4
        ph_value = 4.0 - ((PH4_ADC - adc_value) * 3.0) / (PH7_ADC - PH4_ADC);
    }
    else if (adc_value <= PH7_ADC)
    {
        // Interpolate between pH 4 and pH 7
        ph_value = 4.0 + ((adc_value - PH4_ADC) * 3.0) / (PH7_ADC - PH4_ADC);
    }
    else if (adc_value <= PH10_ADC)
    {
        // Interpolate between pH 7 and pH 10
        ph_value = 7.0 + ((adc_value - PH7_ADC) * 3.0) / (PH10_ADC - PH7_ADC);
    }
    else
    {
        // Extrapolate above pH 10
        ph_value = 10.0 + ((adc_value - PH10_ADC) * 3.0) / (PH10_ADC - PH7_ADC);
    }

    return ph_value;
}

/**
 * @brief Smooth ADC readings by averaging multiple samples.
 *
 * @param channel Pointer to the ADC channel object.
 * @param samples Number of samples to average.
 * @return Averaged ADC value.
 */
int smooth_adc_read(cyhal_adc_channel_t *channel, int samples)
{
    int sum = 0;

    for (int i = 0; i < samples; i++)
    {
        sum += cyhal_adc_read_uv(channel); // Read ADC in microvolts
        cyhal_system_delay_ms(10);        // Small delay between readings
    }

    return sum / samples;
}

