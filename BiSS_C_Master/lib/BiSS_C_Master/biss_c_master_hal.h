/*!
 * @file biss_c_master_hal.h
 * @author Kirill Rostovskiy (kmrost@lenzencoders.com)
 * @brief BiSS C Master Hardware abstraction layer driver
 * @version 0.1
 * @copyright Lenz Encoders (c) 2024
 */

#ifndef __BISS_C_MASTER_HAL_H
#define __BISS_C_MASTER_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @struct AngleData_t
 * @brief Angle Data with time of life counter type
 * 
 */
typedef struct{
    uint32_t angle_data:24; /**< Value of Angle */
    uint32_t time_of_life_counter:8; /**< Value of time of life countre to check
	that angle was updated*/
} AngleData_t;

typedef enum{
	BISS_SPI_CH_1,
	BISS_SPI_CH_2,
}BiSS_SPI_Ch_t;

extern volatile BiSS_SPI_Ch_t BiSS_SPI_Ch;
/**
 * @brief BiSS C Master hardware abstruction layer initialization function
 * 
 */
void BiSS_C_Master_HAL_Init(void);
		
/**
 * @brief Get the Angle object
 * 
 * @return AngleData_t 
 */
static inline AngleData_t getAngle1(void){
	extern volatile AngleData_t AngleData1;
	return(AngleData1);
}

static inline AngleData_t getAngle2(void){
	extern volatile AngleData_t AngleData2;
	return(AngleData2);
}

void SetBiSS_SPI_Ch(BiSS_SPI_Ch_t ch_to_set);
uint32_t GetSSIFlag(void);

void NextRS485Address(void);

#ifdef __cplusplus
}
#endif

#endif /* __BISS_C_MASTER_HAL_H */
