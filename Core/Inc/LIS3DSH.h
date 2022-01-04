

#ifndef LIS3DSH_H

#define LIS3DSH_H 
/**
 * Includes
 */
#include <inttypes.h>

#define LIS3DSH_CS_LOW				TM_GPIO_SetPinLow(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN)
#define LIS3DSH_CS_HIGH			TM_GPIO_SetPinHigh(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN)

/* Who I am values */

#define LIS3DSH_ID							0x3F

/* ----------------------------------------- */
/* LIS3DSH registers */
/* ----------------------------------------- */
#define LIS3DSH_WHO_AM_I_ADDR				0x0F
#define LIS3DSH_CTRL_REG4_ADDR				0x20
#define LIS3DSH_CTRL_REG1_ADDR				0x21
#define LIS3DSH_CTRL_REG2_ADDR				0x22
#define LIS3DSH_CTRL_REG3_ADDR				0x23
#define LIS3DSH_CTRL_REG5_ADDR				0x24
#define LIS3DSH_CTRL_REG6_ADDR				0x25
#define LIS3DSH_OUT_X_L_ADDR				0x28
#define LIS3DSH_OUT_X_H_ADDR				0x29
#define LIS3DSH_OUT_Y_L_ADDR				0x2A
#define LIS3DSH_OUT_Y_H_ADDR				0x2B
#define LIS3DSH_OUT_Z_L_ADDR				0x2C
#define LIS3DSH_OUT_Z_H_ADDR				0x2D

#define LIS3DSH_SENSITIVITY_0_06G            0.06  /* 0.06 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_12G            0.12  /* 0.12 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_18G            0.18  /* 0.18 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_24G            0.24  /* 0.24 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_73G            0.73  /* 0.73 mg/digit*/


#define LIS3DSH_DATARATE_100				((uint8_t)0x60)

#define LIS3DSH_FULLSCALE_2					((uint8_t)0x00)  /* 2 g  */
#define LIS3DSH_FULLSCALE_4					((uint8_t)0x08)  /* 4 g  */
#define LIS3DSH_FULLSCALE_6					((uint8_t)0x10)  /* 6 g  */
#define LIS3DSH_FULLSCALE_8					((uint8_t)0x18)  /* 8 g  */
#define LIS3DSH_FULLSCALE_16				((uint8_t)0x20)  /* 16 g */
#define LIS3DSH__FULLSCALE_SELECTION		((uint8_t)0x38)

#define LIS3DSH_FILTER_BW_800				((uint8_t)0x00)  /* 800 Hz */
#define LIS3DSH_FILTER_BW_400				((uint8_t)0x40)//((uint8_t)0x08) /* 400 Hz  */
#define LIS3DSH_FILTER_BW_200				((uint8_t)0x80)//((uint8_t)0x10)  /* 200 Hz */
#define LIS3DSH_FILTER_BW_50				((uint8_t)(0x80 | 0x40))//((uint8_t)0x18)  /* 50 Hz  */
#define LIS3DSH_SELFTEST_NORMAL				((uint8_t)0x00)
#define LIS3DSH_XYZ_ENABLE					((uint8_t)0x07)
#define LIS3DSH_SERIALINTERFACE_4WIRE		((uint8_t)0x00)
#define LIS3DSH_SM_ENABLE					((uint8_t)0x01)
#define LIS3DSH_SM_DISABLE					((uint8_t)0x00)

/**
 * Typedef enumeration for returning from functions
 * 
 * Parameters:
 * 	- TM_LIS302DL_LIS3DSH_Device_Error:
 * 		Returned from function when device is not recognized
 * 	- TM_LIS302DL_LIS3DSH_Device_LIS302DL:
 * 		Returned from functions when device is LIS302DL on old boards
 * 	- TM_LIS302DL_LIS3DSH_Device_LIS3DSH:
 * 		Returned from functions when device is LIS3DSH on new boards
 */
typedef enum {
	LIS3DSH_Device_Error,
	LIS3DSH_Device_LIS3DSH,
} LIS3DSH_Device_t;

/**
 * Possible sensitivities for both devices
 * 
 * Parameters:
 * 	- TM_LIS3DSH_Sensitivity_2G:
 * 		2G sensitivity on LIS3DSH device
 * 	- TM_LIS3DSH_Sensitivity_4G:
 * 		4G sensitivity on LIS3DSH device
 * 	- TM_LIS3DSH_Sensitivity_6G:
 * 		6G sensitivity on LIS3DSH device
 * 	- TM_LIS3DSH_Sensitivity_8G:
 * 		8G sensitivity on LIS3DSH device
 * 	- TM_LIS3DSH_Sensitivity_16G:
 * 		16G sensitivity on LIS3DSH device
 * 
 * 	- TM_LIS302DL_Sensitivity_2_3G:
 * 		2.3G  sensitivity on LIS3DSH device
 * 	- TM_LIS302DL_Sensitivity_9_2G:
 * 		9.2G sensitivity on LIS3DSH device
 */
typedef enum {
	/* LIS3DSH */
	LIS3DSH_Sensitivity_2G,
	LIS3DSH_Sensitivity_4G,
	LIS3DSH_Sensitivity_6G,
	LIS3DSH_Sensitivity_8G,
	LIS3DSH_Sensitivity_16G,
} LIS3DSH_Sensitivity_t;


/**
 * Filter values for both accelerometers
 * 
 * Parameters:
 * 	- TM_LIS3DSH_Filter_800Hz:
 * 		800 Hz high pass filter on LIS3DSH
 * 	- TM_LIS3DSH_Filter_400Hz:
 * 		400 Hz high pass filter on LIS3DSH
 * 	- TM_LIS3DSH_Filter_200Hz:
 * 		200 Hz high pass filter on LIS3DSH
 * 	- TM_LIS3DSH_Filter_50Hz:
 * 		50 Hz high pass filter on LIS3DSH
 * 	
 * 	- TM_LIS302DL_Filter_2Hz:
 * 		2 Hz filter on LIS302DL
 * 	- TM_LIS302DL_Filter_1Hz:
 * 		1 Hz filter on LIS302DL
 * 	- TM_LIS302DL_Filter_500mHz:
 * 		500 mHz filter on LIS302DL
 * 	- TM_LIS302DL_Filter_250mHz
 * 		250 mHz filter on LIS302DL
 */
 
 typedef enum {
	/* LIS3DSH */
	LIS3DSH_Filter_800Hz,
	LIS3DSH_Filter_400Hz,
	LIS3DSH_Filter_200Hz,
	LIS3DSH_Filter_50Hz,
} LIS3DSH_Filter_t;


typedef struct {
	int16_t X;
	int16_t Y;
	int16_t Z;
} LIS3DSH_t;

/**
 * Detect LIS302DL or LIS3DSH device connected on F4-Discovery board
 * 
 * Member of TM_LIS302DL_LIS3DSH_Device_t is returned
 */

extern LIS3DSH_Device_t TM_LIS302DL_LIS3DSH_Detect(void);

/**
 * Initialize proper device
 * 
 * Parameters:
 * 	- TM_LIS302DL_LIS3DSH_Sensitivity_t Sensitivity:
 * 		Select sensitivity for device. There are separate possible sensitivities for proper device
 * 	- TM_LIS302DL_LIS3DSH_Filter_t Filter:
 * 		Select filter for device. There are separate possible sensitivities for proper device
 * 
 * Member of LIS3DSH_Device_t is returned
 */

extern LIS3DSH_Device_t LIS3DSH_Init(LIS3DSH_Sensitivity_t Sensitivity, LIS3DSH_Filter_t Filter);

/**
 * Read axes from device
 * 
 * Parameters:
 * 	- TM_LIS302DL_LIS3DSH_t* Axes_Data
 * 		Pointer to TM_LIS302DL_LIS3DSH_t struct
 * 
 * Member of LIS3DSH_Device_t is returned
 */
 
 extern LIS3DSH_Device_t LIS3DSH_ReadAxes(LIS3DSH_t* Axes_Data);
 
 #endif