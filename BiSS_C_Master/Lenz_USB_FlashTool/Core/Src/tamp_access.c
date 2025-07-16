/*!
 * @file tamp_access.c 
 * @author Kirill Rostovskiy (kmrost@lenzencoders.com)
 * @brief TAMP backup registers access library
 * @version 0.1
 * @copyright Lenz Encoders (c) 2024
 */

#include "tamp_access.h"

#define TAMP_FLAGS_REGISTER          LL_RTC_BKP_DR0
#define TAMP_COUNTER_REGISTER        LL_RTC_BKP_DR1


void SetTampFlag(uint32_t flag)
{
    LL_PWR_EnableBkUpAccess();
	  uint32_t regValue = LL_RTC_BKP_GetRegister(RTC, TAMP_FLAGS_REGISTER);
    regValue |= flag;
    LL_RTC_BKP_SetRegister(RTC, TAMP_FLAGS_REGISTER, regValue);
    LL_PWR_DisableBkUpAccess();
}

uint32_t GetTampFlag(uint32_t flag)
{
    uint32_t tampFlags = LL_RTC_BKP_GetRegister(RTC, TAMP_FLAGS_REGISTER);
    return tampFlags & flag;
}


void ClearTampFlag(uint32_t flag)
{
    LL_PWR_EnableBkUpAccess();
    uint32_t regValue = LL_RTC_BKP_GetRegister(RTC, TAMP_FLAGS_REGISTER);
    regValue &= ~flag;
    LL_RTC_BKP_SetRegister(RTC, TAMP_FLAGS_REGISTER, regValue);
    LL_PWR_DisableBkUpAccess();
}

void IncrementStartupCounter(void)
{
    LL_PWR_EnableBkUpAccess();
    uint32_t counter = LL_RTC_BKP_GetRegister(RTC, TAMP_COUNTER_REGISTER);
    counter++;
    LL_RTC_BKP_SetRegister(RTC, TAMP_COUNTER_REGISTER, counter);
    LL_PWR_DisableBkUpAccess();
}

uint32_t GetStartupCounter(void)
{
    return (LL_RTC_BKP_GetRegister(RTC, TAMP_COUNTER_REGISTER));
}

void ResetStartupCounter(void)
{
    LL_PWR_EnableBkUpAccess();
    LL_RTC_BKP_SetRegister(RTC, TAMP_COUNTER_REGISTER, 0);
    LL_PWR_DisableBkUpAccess();
}
