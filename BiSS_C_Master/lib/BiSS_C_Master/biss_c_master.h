/*!
 * @file biss_c_master.h
 * @author Kirill Rostovskiy (kmrost@lenzencoders.com)
 * @brief BiSS C Master library
 * @version 0.1
 * @copyright Lenz Encoders (c) 2024
 *
 * @note BiSS C Master state machine CDM setting and CDS receiving timing diagram 
 * 
 *          ↓ CDM(n) Master request                 CDM(n) setting up ↓                    ↓ CDM(n+1) Master request     
 * MA  ‾‾‾‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾‾|__|‾xxxxxxx_|‾‾|xxxx nCDM xxxxx|‾‾‾‾|__|‾‾|_xxx_|‾‾|__|‾‾|_xxxxx
 *
 * SLO ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|__________ACK__________|‾ST‾‾|xCDSx|xx SCD xx|_xx_TimeOut_xx_|‾‾‾‾‾‾‾‾‾‾‾‾‾xxxx‾‾|xCDSx|x SCD xx 
 *                              CDS(n-1) setting up ↑            CDM(n)Slave read ↑   ↑ CDS(n-1)Master read  ↑ CDS(n) setting up 
 */

#ifndef __BISS_C_MASTER_H
#define __BISS_C_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/**
 * @enum BiSSExternalState_t
 * @brief BiSS C Master States of state machine enumeration 
 * 
 */
typedef enum{
	BISS_REQ_OK = 0, /**< Ready to new request */
	BISS_BUSY = 1u,	/**< Request is beging processed */
	BISS_WRITE_FINISHED, /**< Writing Request has been completed */
	BISS_READ_FINISHED, /**< Reading Request has been completed */
	BISS_FAULT,        /**< BiSS Fault State */
}BiSSExternalState_t;

/**
 * @enum BiSSFaultState_t
 * @brief BiSS C Master Fault States enumeration 
 * 
 */
typedef enum{
	BISS_NO_FAULTS = 0, /**< OK State */
	BISS_FAULT_IDL = 1u,	/**< ID Lock fault returned */
	BISS_FAULT_WRITE, /**< Writing fault */
	BISS_FAULT_READ_CRC /**< CRC of read data fault */
}BiSSFaultState_t;

/**
 * @enum CDM_t
 * @brief BiSS C Control Data Master bit enumeration
 * 
 */
typedef enum{
	nCDM = 0, /**< */
	CDM = 1U, /**< */
}CDM_t;

/**
 * @enum CDS_t
 * @brief BiSS C Control Data Slave bit enumeration
 * 
 */
typedef enum{
	nCDS = 0, /**< */
	CDS = 1U /**< */
}CDS_t;

/**
 * @brief BISS C Master state machine function 
 * Should be requested every BiSS С angle(position) reading
 * @param CDS_in (n-1) BiSS C Control Data Slave bit
 * @return CDM_t (n+1) BiSS C Control Data Master bit
 */
CDM_t BiSS_C_Master_StateMachine(CDS_t CDS_in);

/**
 * @brief BiSS C Register reading request function
 * 
 * @param addr First address of BiSS C register (from 0 to 127) 
 * @param cnt Number of bytes to read (from 1 to 65)
 * @param ptr Pointer to uint8_t buffer for reading
 * @return BiSSExternalState_t Request condition. Need to check 
 * IsBiSSReqBusy() == BISS_READ_FINISHED before reading data in ptr*
 */
BiSSExternalState_t BiSSRequestRead(uint16_t addr, uint8_t cnt, uint8_t *ptr);

/**
 * @brief BiSS C Register writing request function
 * 
 * @param addr First address of BiSS C register (from 0 to 127) 
 * @param cnt Number of bytes to write (from 1 to 65)
 * @param ptr Pointer to uint8_t buffer for writing
 * @return BiSSExternalState_t Request condition. Need to check 
 * IsBiSSReqBusy() == BISS_WRITE_FINISHED before new request
 */
BiSSExternalState_t BiSSRequestWrite(uint16_t addr, uint8_t cnt, uint8_t *ptr);

/**
 * @brief State machine status request fuction
 * Should be used before sending new request or checking for read/write request completion
 * @return BiSSExternalState_t 
 */
BiSSExternalState_t IsBiSSReqBusy(void);

/**
 * @brief Reset state machine status fuction
 * Should be called after reading/writing completion
 * @return BiSSExternalState_t BISS_REQ_OK after successful reset
 */
BiSSExternalState_t BiSSResetExternalState(void);

/**
 * @brief BiSS fault state request
 * @return BiSSFaultState_t 
 */
BiSSFaultState_t BiSSGetFaultState(void);

#ifdef __cplusplus
}
#endif

#endif /* __BISS_C_MASTER_H */
