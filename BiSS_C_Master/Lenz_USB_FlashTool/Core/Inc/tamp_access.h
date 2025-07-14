/*!
 * @file tamp_access.h
 * @author Kirill Rostovskiy (kmrost@lenzencoders.com)
 * @brief TAMP backup registers access library
 * @version 0.1
 * @copyright Lenz Encoders (c) 2024
 */

#ifndef __TAMP_ACCESS_H
#define __TAMP_ACCESS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx.h"
#include "stm32g4xx_ll_rtc.h"
#include "stm32g4xx_ll_pwr.h"


#define TAMP_FLAGS_STAY_BL_Pos					0U
#define TAMP_FLAGS_STAY_BL							(1U << TAMP_FLAGS_STAY_BL_Pos)
#define TAMP_FLAGS_ERROROCRC32_BL_Pos		1U
#define TAMP_FLAGS_ERROROCRC32_BL				(1U << TAMP_FLAGS_ERROROCRC32_BL_Pos)
#define TAMP_FLAGS_RDP_AA_Pos						2U
#define TAMP_FLAGS_RDP_AA								(1U << TAMP_FLAGS_RDP_AA_Pos)
#define TAMP_FLAGS_STAY_MAIN_FW_POS			3U
#define TAMP_FLAGS_STAY_MAIN_FW					(1U << TAMP_FLAGS_STAY_MAIN_FW_POS)

void SetTampFlag(uint32_t flag);

uint32_t GetTampFlag(uint32_t flag);

void IncrementStartupCounter(void);

uint32_t GetStartupCounter(void);

void ResetStartupCounter(void);

void ClearTampFlag(uint32_t flag);


#ifdef __cplusplus
}
#endif

#endif /* __TAMP_ACCESS_H */
