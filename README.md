# BIIL_MTB-BDH_PH_Sensor_Template
Lab Objective:
This lab demonstrates the process of reading a ph sensor value through an Analog-to-Digital Converter (ADC) using a Hardware Abstraction Layer (HAL) on a PSoC 6 microcontroller.        
## 🔥 Requirements
| Resources                                  | Links                                                                                                  |
|--------------------------------------------|--------------------------------------------------------------------------------------------------------|
| Computer                                   | 💻                                                                                                    |
| ModusToolbox™ software v3.0 or later       | [Link](https://www.infineon.com/modustoolbox)                                                         |
| CY8CKIT-062S2-43012 Infineon Board         | [Link](https://github.com/Advance-Innovation-Centre-AIC/BIIL_MTB-100_Hello_World_and_LED_Blinking_Programming_Template/assets/88732241/0215501d-b774-4045-8e64-ef49e28d8404) |
| Technical Report | [dropbox](https://www.dropbox.com/scl/fi/amaxc94pte0ut2i1r5ewx/Technical-Report-Lab00.paper?rlkey=b3xm3vrerz9xgv1glb30cvy9z&dl=0)


## 🚩 Let start
### Create Application 
![image](https://github.com/Advance-Innovation-Centre-AIC/BIIL_MTB-107_Read_potentiometer_sensor_value_via_an_ADC_HAL_Template/assets/88732241/0122ad71-1b62-4d0f-b607-91d15bc26c60)

### Coding
Coding: Open the main.c file and add the following code to the main(void) function.
```
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


int main(void)
{
    cy_rslt_t result;
	cyhal_adc_t adc_obj;
	cyhal_adc_channel_t adc_chan_0_obj;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
	result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
	CY_ASSERT(result == CY_RSLT_SUCCESS);

	/* ADC conversion result. */
	int adc_out;

	/* Initialize ADC. The ADC block which can connect to pin 10[6] is selected */
	result = cyhal_adc_init(&adc_obj, P10_6, NULL);

	// ADC configuration structure
	const cyhal_adc_config_t ADCconfig ={
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

	// Configure to use VDD as Vref
	result = cyhal_adc_configure(&adc_obj, &ADCconfig);

	/* Initialize ADC channel, allocate channel number 0 to pin 10[6] as this is the first channel initialized */
	const cyhal_adc_channel_config_t channel_config = { .enable_averaging = false, .min_acquisition_ns = 220, .enabled = true };
	result = cyhal_adc_channel_init_diff(&adc_chan_0_obj, &adc_obj, P10_6, CYHAL_ADC_VNEG, &channel_config);

    for (;;)
    {
    	/* Read the ADC conversion result for corresponding ADC channel. Repeat as necessary. */
		adc_out = cyhal_adc_read_uv(&adc_chan_0_obj);
		printf("ADC = %d\r\n", adc_out);
		cyhal_system_delay_ms(100);
    }
}
```
### Build the Application      
![image](https://github.com/Advance-Innovation-Centre-AIC/BIIL_MTB-107_Read_potentiometer_sensor_value_via_an_ADC_HAL_Template/assets/88732241/2d61a9f5-128d-42fc-9351-51d2786fd75f)


### Launching the Application      
![image](https://github.com/Advance-Innovation-Centre-AIC/BIIL_MTB-107_Read_potentiometer_sensor_value_via_an_ADC_HAL_Template/assets/88732241/1830d0c4-1c68-49ce-9d71-93bf59c14a8e)

  Note: Before launching the program to the board, make sure that you have already connected the board to the computer through a USB cable.    
  ![image](https://github.com/Advance-Innovation-Centre-AIC/BIIL_MTB-107_Read_potentiometer_sensor_value_via_an_ADC_HAL_Template/assets/88732241/c9966b5b-702f-478e-bbe8-ba9e277800d2)


### Result     

![image](https://github.com/Advance-Innovation-Centre-AIC/BIIL_MTB-107_Read_potentiometer_sensor_value_via_an_ADC_HAL_Template/assets/88732241/1d3f3785-f43c-40a1-bc28-41da08e3ed08)

### 🎉  Congratulations! You can now complete Lab107

## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; embedded compiler v10.3.1 (`GCC_ARM`) - Default value of `TOOLCHAIN`
- Arm&reg; compiler v6.16 (`ARM`)
- IAR C/C++ compiler v9.30.1 (`IAR`)

## Supported kits (make variable 'TARGET')

- [PSoC&trade; 62S2 Wi-Fi Bluetooth&reg; pioneer kit](https://www.infineon.com/CY8CKIT-062S2-43012) (`CY8CKIT-062S2-43012`)
- [PSoC&trade; 62S1 Wi-Fi Bluetooth&reg; pioneer kit](https://www.infineon.com/CYW9P62S1-43438EVB-01) (`CYW9P62S1-43438EVB-01`)
- [PSoC&trade; 62S1 Wi-Fi Bluetooth&reg; pioneer kit](https://www.infineon.com/CYW9P62S1-43012EVB-01) (`CYW9P62S1-43012EVB-01`)
- [PSoC&trade; 62S3 Wi-Fi Bluetooth&reg; prototyping kit](https://www.infineon.com/CY8CPROTO-062S3-4343W) (`CY8CPROTO-062S3-4343W`)


## Related resources
Resources  | Links
-----------|----------------------------------
ModusToolbox™ Software Training | [link](https://www.dropbox.com/sh/waj898o4o8eccx0/AAB3hBBaIQo2OvJ5-fubGJIha/training-modustoolbox-level1-getting-started-master/Manual/Ch2-Tools.pdf?dl=0)

## Other resources

Infineon provides a wealth of data at www.infineon.com to help you select the right device, and quickly and effectively integrate it into your design.


## Document history

Document title: BILL_MTB-107 – Read potentiometer sensor value via an ADC HAL

 Version | Description of change
 ------- | ---------------------
 1.0.0   | Lab 107: Learn basic GPIO control with PSoC 6, using ADC read potentiometer sensor value via HAL


## Authors:
- *Assoc. Prof. Wiroon Sriborrirux*
- *Mr. Sriengchhun Chheang*
- *Mr. Sabol Socare*
<br>

<br>

---------------------------------------------------------

© BDH Corporation, 2022-2023
