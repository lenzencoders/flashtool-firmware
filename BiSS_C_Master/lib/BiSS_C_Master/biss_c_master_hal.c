/*!
 * @file biss_c_master_hal.c
 * @author Kirill Rostovskiy (kmrost@lenzencoders.com)
 * @brief BiSS C Master STM32G4x Hardware abstraction layer driver
 * @version 0.1
 * @copyright Lenz Encoders (c) 2024
 * @section disclaimer  
 * Disclaimer! This is tamplate driver for debug/review BiSS C master library on STM32Gx (and similar) microcontrollers.
 * It supports only LENZ IRS/SAB/SIB encoders only with 3 additional Ack bits and SCD length - 32 bits(24 bits position data, 
 * 2 bits nError/nWaring and 6 bits CRC). You should implement and test this part on your own.
 */

#include "hw_cfg.h"
#include "stm32g4xx.h"
#include "biss_c_master.h"
#include "biss_c_master_hal.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_adc.h"
#include "uart.h"
#include "string.h"

#define SPI_CR1_BISS_CDM    SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_SSM | ((0x5U << SPI_CR1_BR_Pos) & SPI_CR1_BR_Msk)
#define SPI_CR1_BISS_nCDM   SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BISS_CDM
#define SPI_CR2_BISS_CFG		SPI_CR2_RXDMAEN |SPI_CR2_TXDMAEN | SPI_CR2_FRXTH | (0x7U << SPI_CR2_DS_Pos)

#define RX_BUF_SIZE 252U
#define TX_BUF_SIZE 252U
#define ADC_BUF_SIZE 1U
#define VREF	3300U
#define ADC_MAX	4095U
#define CURRENT_GAIN 101753U


// #define PWR1_EN_PIN		GPIOA, LL_GPIO_PIN_8

static volatile enum {RS485_ADR1, RS485_ADR2} RS485_ADR = RS485_ADR2;
static volatile uint8_t adr_reset_cou = 0;
typedef enum{
	RS485_CDM_ADR1_REQ = 0x81U,
	RS485_CDM_ADR2_REQ = 0x22U,
	RS485_CDM_ADR3_REQ = 0x43U,
	RS485_CDM_ADR4_REQ = 0x04U,
	RS485_CDM_ADR5_REQ = 0x65U,
	RS485_CDM_ADR6_REQ = 0xC6U,
	RS485_CDM_ADR7_REQ = 0xA7U,
	RS485_nCDM_ADR1_REQ = 0xB1U,
	RS485_nCDM_ADR2_REQ = 0x12U,
	RS485_nCDM_ADR3_REQ = 0x73U,
	RS485_nCDM_ADR4_REQ = 0x34U,
	RS485_nCDM_ADR5_REQ = 0x55U,
	RS485_nCDM_ADR6_REQ = 0xF6U,
	RS485_nCDM_ADR7_REQ = 0x97U,
}RS485_Req_t;

typedef union{
	volatile uint8_t buf[8];
	struct{
		uint32_t reserv1:24;
		volatile uint32_t CDS:1;
		uint32_t reserv2:7;
		volatile uint32_t revSCD;
	};
}SPI_rx_t;

typedef union{
	volatile uint32_t u32;
	struct{
		uint32_t adr:3;
		uint32_t frame_format:1;
		uint32_t CDS:1;
		uint32_t nW:1;
		uint32_t nE:1;
		uint32_t Pos:19;
		uint32_t _CRC6:6;
	};	
	struct{
		uint32_t Data4CRC:26;
		uint32_t CRC6:6;
	};
}USART_rx_t;

typedef enum{
	CRC6_OK,CRC6_FAULT
}CRC_State_t;

volatile uint32_t SSI_HALF_FREQ_FLAG = 0;
uint8_t usart2_tx_buffer[TX_BUF_SIZE] = {0};
volatile uint8_t usart2_rx_buffer[RX_BUF_SIZE] = {0};
volatile uint16_t adc1_buffer[ADC_BUF_SIZE] = {0};
SPI_rx_t BiSS1_SPI_rx;
SPI_rx_t BiSS2_SPI_rx;
USART_rx_t USART_rx;

volatile BISS_Mode_t Current_Mode = BISS_MODE_DEFAULT_SPI; // BISS_MODE_SPI_SPI or BISS_MODE_AB_UART or BISS_MODE_SPI_UART_IRS or BISS_MODE_AB_SPI or BISS_MODE_DEFAULT_SPI
volatile BiSS_SPI_Ch_t BiSS_SPI_Ch = BISS_SPI_CH_2; // BISS_SPI_CH_1 or BISS_SPI_CH_2
volatile CH1_SSI_t CH1_SSI = CH1_SSI_FULL_FREQ;
volatile Current_Sensor_Mode_t Current_Sensor_Mode = CURRENT_SENSOR_MODE_ENABLE; // CURRENT_SENSOR_MODE_ENABLE CURRENT_SENSOR_MODE_DISABLE
volatile PinDef PWR1_EN_PIN = {GPIOA, LL_GPIO_PIN_8};

volatile CDS_t USART_CDS_last = CDS;
volatile uint32_t BISS1_SCD;
volatile uint32_t BISS2_SCD;
volatile AngleData_t AngleData1;
volatile AngleData_t AngleData2;
volatile AngleDataRenishaw_t AngleDataRenishaw;
volatile CRC_State_t CRC6_State1 = CRC6_FAULT;
volatile CRC_State_t CRC6_State2 = CRC6_FAULT;
volatile CDM_t last_CDM = CDM;

static const uint8_t CRC6_LUT[256U] = {
	0x00U, 0x0CU, 0x18U, 0x14U, 0x30U, 0x3CU, 0x28U, 0x24U, 0x60U, 0x6CU, 0x78U, 0x74U, 0x50U, 0x5CU, 0x48U, 0x44U, 
	0xC0U, 0xCCU, 0xD8U, 0xD4U, 0xF0U, 0xFCU, 0xE8U, 0xE4U, 0xA0U, 0xACU, 0xB8U, 0xB4U, 0x90U, 0x9CU, 0x88U, 0x84U, 
	0x8CU, 0x80U, 0x94U, 0x98U, 0xBCU, 0xB0U, 0xA4U, 0xA8U, 0xECU, 0xE0U, 0xF4U, 0xF8U, 0xDCU, 0xD0U, 0xC4U, 0xC8U, 
	0x4CU, 0x40U, 0x54U, 0x58U, 0x7CU, 0x70U, 0x64U, 0x68U, 0x2CU, 0x20U, 0x34U, 0x38U, 0x1CU, 0x10U, 0x04U, 0x08U, 
	0x14U, 0x18U, 0x0CU, 0x00U, 0x24U, 0x28U, 0x3CU, 0x30U, 0x74U, 0x78U, 0x6CU, 0x60U, 0x44U, 0x48U, 0x5CU, 0x50U, 
	0xD4U, 0xD8U, 0xCCU, 0xC0U, 0xE4U, 0xE8U, 0xFCU, 0xF0U, 0xB4U, 0xB8U, 0xACU, 0xA0U, 0x84U, 0x88U, 0x9CU, 0x90U, 
	0x98U, 0x94U, 0x80U, 0x8CU, 0xA8U, 0xA4U, 0xB0U, 0xBCU, 0xF8U, 0xF4U, 0xE0U, 0xECU, 0xC8U, 0xC4U, 0xD0U, 0xDCU, 
	0x58U, 0x54U, 0x40U, 0x4CU, 0x68U, 0x64U, 0x70U, 0x7CU, 0x38U, 0x34U, 0x20U, 0x2CU, 0x08U, 0x04U, 0x10U, 0x1CU, 
	0x28U, 0x24U, 0x30U, 0x3CU, 0x18U, 0x14U, 0x00U, 0x0CU, 0x48U, 0x44U, 0x50U, 0x5CU, 0x78U, 0x74U, 0x60U, 0x6CU, 
	0xE8U, 0xE4U, 0xF0U, 0xFCU, 0xD8U, 0xD4U, 0xC0U, 0xCCU, 0x88U, 0x84U, 0x90U, 0x9CU, 0xB8U, 0xB4U, 0xA0U, 0xACU, 
	0xA4U, 0xA8U, 0xBCU, 0xB0U, 0x94U, 0x98U, 0x8CU, 0x80U, 0xC4U, 0xC8U, 0xDCU, 0xD0U, 0xF4U, 0xF8U, 0xECU, 0xE0U, 
	0x64U, 0x68U, 0x7CU, 0x70U, 0x54U, 0x58U, 0x4CU, 0x40U, 0x04U, 0x08U, 0x1CU, 0x10U, 0x34U, 0x38U, 0x2CU, 0x20U, 
	0x3CU, 0x30U, 0x24U, 0x28U, 0x0CU, 0x00U, 0x14U, 0x18U, 0x5CU, 0x50U, 0x44U, 0x48U, 0x6CU, 0x60U, 0x74U, 0x78U, 
	0xFCU, 0xF0U, 0xE4U, 0xE8U, 0xCCU, 0xC0U, 0xD4U, 0xD8U, 0x9CU, 0x90U, 0x84U, 0x88U, 0xACU, 0xA0U, 0xB4U, 0xB8U, 
	0xB0U, 0xBCU, 0xA8U, 0xA4U, 0x80U, 0x8CU, 0x98U, 0x94U, 0xD0U, 0xDCU, 0xC8U, 0xC4U, 0xE0U, 0xECU, 0xF8U, 0xF4U, 
	0x70U, 0x7CU, 0x68U, 0x64U, 0x40U, 0x4CU, 0x58U, 0x54U, 0x10U, 0x1CU, 0x08U, 0x04U, 0x20U, 0x2CU, 0x38U, 0x34U
};

/*!
 * @brief CRC6 calculation function
 * @param data including position data, error and warning bits
 * @return uint8_t CRC6, poly 0x43, inverted
 */
__STATIC_INLINE uint8_t BISS_CRC6_Calc(uint32_t data){	
	uint8_t crc = CRC6_LUT[(data >> 24U) & 0x3U];
	crc = CRC6_LUT[((data >> 16U) & 0xFFU) ^ crc];
	crc = CRC6_LUT[((data >> 8U) & 0xFFU) ^ crc];
	crc = CRC6_LUT[(data & 0xFFU) ^ crc];
	crc = ((crc ^ 0xFFU) >> 2U) & 0x3FU;
	return(crc);
}

static void USART2_Init_IRS(void);
static void USART2_Config_IRS(void);
static void USART2_IRS_Transmit(uint8_t* data, uint8_t length);
static void USART2_IRS_Receive(uint8_t* data, uint8_t length);
static void Reenable_USART2_RX(void);
static void BISS1_SPI_Init(void);
static void BISS1_SPI_DeInit(void);
static void BISS2_SPI_Init(void);
static void BISS2_SPI_DeInit(void);
static void BISS_UART_Init(void);
static void BISS_UART_DeInit(void);
static void USART2_UART_Init(void);
static void MX_TIM3_ENC_Init(void);
static void Quadrature_Renishaw_Init(void);
static void Quadrature_Renishaw_DeInit(void);
static void MX_ADC1_Init(void);
static void Config_ADC1(void);
static void ADC1_DeInit(void);

static void LED1TurnRed(void){
	LL_GPIO_SetOutputPin(LED1_GREEN);
	LL_GPIO_ResetOutputPin(LED1_RED);
}
static void LED1TurnGreen(void){
	LL_GPIO_SetOutputPin(LED1_RED);
	LL_GPIO_ResetOutputPin(LED1_GREEN);
}
static void LED2TurnRed(void){
	LL_GPIO_SetOutputPin(LED2_GREEN);
	LL_GPIO_ResetOutputPin(LED2_RED);
}
static void LED2TurnGreen(void){
	LL_GPIO_SetOutputPin(LED2_RED);
	LL_GPIO_ResetOutputPin(LED2_GREEN);
}

static void BiSS1_SPI_nCDM_Req(void){
	LL_DMA_DisableChannel(DMA_BISS1_RX);
	LL_DMA_DisableChannel(DMA_BISS1_TX);
	LL_SPI_DeInit(BISS1_SPI);
	BISS1_SPI->CR1 = SPI_CR1_BISS_nCDM;
	BISS1_SPI->CR2 = SPI_CR2_BISS_CFG;
	// #ifdef CH1_SSI
	if(CH1_SSI == CH1_SSI_HALF_FREQ){
		LL_SPI_SetBaudRatePrescaler(BISS1_SPI, LL_SPI_BAUDRATEPRESCALER_DIV128);
		LL_DMA_SetDataLength(DMA_BISS1_TX, 3U); 
		LL_DMA_SetDataLength(DMA_BISS1_RX, 3U); 
	} 
	else if(CH1_SSI == CH1_SSI_FULL_FREQ){
		// #else
		LL_DMA_SetDataLength(DMA_BISS1_TX, 5U); // TODO try 1U via define
		LL_DMA_SetDataLength(DMA_BISS1_RX, 5U); // TODO try 1U via define
		// #endif
	}
	LL_DMA_EnableChannel(DMA_BISS1_TX);	 
	LL_DMA_EnableChannel(DMA_BISS1_RX);		
}

static void BiSS2_SPI_nCDM_Req(void){
	LL_DMA_DisableChannel(DMA_BISS2_RX);
	LL_DMA_DisableChannel(DMA_BISS2_TX);
	LL_SPI_DeInit(BISS2_SPI);
	BISS2_SPI->CR1 = SPI_CR1_BISS_nCDM;
	BISS2_SPI->CR2 = SPI_CR2_BISS_CFG;
	LL_DMA_SetDataLength(DMA_BISS2_TX, 5U); // TODO try 1U via define
	LL_DMA_SetDataLength(DMA_BISS2_RX, 5U); // TODO try 1U via define
	LL_DMA_EnableChannel(DMA_BISS2_TX);
	LL_DMA_EnableChannel(DMA_BISS2_RX);		
}

static void BiSS1_SPI_CDM_Req(void){			
	LL_GPIO_SetPinMode(MA1_PIN, LL_GPIO_MODE_OUTPUT);
	LL_DMA_DisableChannel(DMA_BISS1_RX);
	LL_DMA_DisableChannel(DMA_BISS1_TX);
	LL_SPI_DeInit(BISS1_SPI);
	BISS1_SPI->CR1 = SPI_CR1_BISS_CDM;
	BISS1_SPI->CR2 = SPI_CR2_BISS_CFG;
	LL_DMA_SetDataLength(DMA_BISS1_TX, 5U); // TODO try 1U via define
	LL_DMA_SetDataLength(DMA_BISS1_RX, 5U); // TODO try 1U via define
	LL_DMA_EnableChannel(DMA_BISS1_TX);	    
	LL_GPIO_SetPinMode(MA1_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_DMA_EnableChannel(DMA_BISS1_RX);	
}

static void BiSS2_SPI_CDM_Req(void){			
	LL_GPIO_SetPinMode(MA2_PIN, LL_GPIO_MODE_OUTPUT);
	LL_DMA_DisableChannel(DMA_BISS2_RX);
	LL_DMA_DisableChannel(DMA_BISS2_TX);
	LL_SPI_DeInit(BISS2_SPI);
	BISS2_SPI->CR1 = SPI_CR1_BISS_CDM;
	BISS2_SPI->CR2 = SPI_CR2_BISS_CFG;
	LL_DMA_SetDataLength(DMA_BISS2_TX, 5U); // TODO try 1U via define
	LL_DMA_SetDataLength(DMA_BISS2_RX, 5U); // TODO try 1U via define
	LL_DMA_EnableChannel(DMA_BISS2_TX);
	LL_GPIO_SetPinMode(MA2_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_DMA_EnableChannel(DMA_BISS2_RX);	
}

void BissRequest_CDM(void){
	switch(Current_Mode){
		case BISS_MODE_SPI_SPI:
			BiSS1_SPI_CDM_Req();
			BiSS2_SPI_CDM_Req();
			break;
		case BISS_MODE_AB_UART:
			LL_DMA_DisableChannel(DMA_BISS2_UART_RX);
			LL_DMA_SetDataLength(DMA_BISS2_UART_RX, 4U);
			LL_DMA_EnableChannel(DMA_BISS2_UART_RX);	 
			switch(RS485_ADR){
				case RS485_ADR1:
					LL_USART_TransmitData8(BISS2_UART, RS485_CDM_ADR1_REQ);
					break;
				case RS485_ADR2:
					LL_USART_TransmitData8(BISS2_UART, RS485_CDM_ADR2_REQ);
					break;
			}
			break;
		case BISS_MODE_SPI_UART_IRS:
			break;
		case BISS_MODE_AB_SPI:
			BiSS2_SPI_CDM_Req();
			break;
		case BISS_MODE_DEFAULT_SPI:
			BiSS2_SPI_CDM_Req();
			break;
	}
}

void BissRequest_nCDM(void){
	switch(Current_Mode){
		case BISS_MODE_SPI_SPI:		
			BiSS1_SPI_nCDM_Req();
			BiSS2_SPI_nCDM_Req();
			break;
		case BISS_MODE_AB_UART:
			LL_DMA_DisableChannel(DMA_BISS2_UART_RX);
			LL_DMA_SetDataLength(DMA_BISS2_UART_RX, 4U);
			LL_DMA_EnableChannel(DMA_BISS2_UART_RX);
			switch(RS485_ADR){
				case RS485_ADR1:
					LL_USART_TransmitData8(BISS2_UART, RS485_nCDM_ADR1_REQ);
					break;
				case RS485_ADR2:
					LL_USART_TransmitData8(BISS2_UART, RS485_nCDM_ADR2_REQ);
					break;
			}
			break;
		case BISS_MODE_SPI_UART_IRS:
			break;
		case BISS_MODE_AB_SPI:
			BiSS2_SPI_nCDM_Req();
			break;
		case BISS_MODE_DEFAULT_SPI:
			BiSS2_SPI_nCDM_Req();
			break;
	}		
}

void BISS_Task_IRQHandler(void) {
	LL_TIM_ClearFlag_UPDATE(BISS_Task_TIM);
	switch(Current_Mode){
		case BISS_MODE_SPI_SPI:
			if(CH1_SSI == CH1_SSI_HALF_FREQ){
				// #ifdef CH1_SSI
				(void) BiSS1_SPI_rx;
				uint32_t rev_temp = __REV(*(uint32_t *)&BiSS1_SPI_rx.buf[0]);		
				if((rev_temp & 0xC00000) == 0x400000){
					CRC6_State1 = CRC6_OK;
					LED1TurnGreen();
					AngleData1.angle_data = (rev_temp << 2) & 0xFFFF80;
					AngleData1.time_of_life_counter++;
				}
				else{
					LED1TurnRed();
					CRC6_State1 = CRC6_FAULT;
				}
				if(SSI_HALF_FREQ_FLAG > 0){
					SSI_HALF_FREQ_FLAG = 0;
					BiSS1_SPI_nCDM_Req();
				}
				else{
					SSI_HALF_FREQ_FLAG = 1;
				}
			}
			else if(CH1_SSI == CH1_SSI_FULL_FREQ){
				// #else
				BISS1_SCD = __REV(BiSS1_SPI_rx.revSCD);
				if(BISS_CRC6_Calc(BISS1_SCD >> 6) == (BISS1_SCD & 0x3FU)){
					CRC6_State1 = CRC6_OK;
					AngleData1.angle_data = BISS1_SCD >> 8;
					AngleData1.time_of_life_counter++;
					if(((BISS1_SCD >> 6) & 0x3) == 0x3){
						LED1TurnGreen();
					}
					else{
						LED1TurnRed();
					}
				}
				else{
					LED1TurnRed();
					CRC6_State1 = CRC6_FAULT;
				}
				// #endif
			}
			
			BISS2_SCD = __REV(BiSS2_SPI_rx.revSCD);
			if(BISS_CRC6_Calc(BISS2_SCD >> 6) == (BISS2_SCD & 0x3FU)){
				CRC6_State2 = CRC6_OK;
				AngleData2.angle_data = BISS2_SCD >> 8;
				AngleData2.time_of_life_counter++;
				if(((BISS2_SCD >> 6) & 0x3) == 0x3){
 					LED2TurnGreen();
 				}
 				else{
 					LED2TurnRed();
 				}
			}
			else{
				LED2TurnRed();
				CRC6_State2 = CRC6_FAULT;
			}					
			switch(BiSS_SPI_Ch){
				case BISS_SPI_CH_1:
					if(CH1_SSI == CH1_SSI_FULL_FREQ){
						// #ifndef CH1_SSI
						if (BiSS_C_Master_StateMachine(BiSS1_SPI_rx.CDS) == CDM) {					
							BiSS1_SPI_CDM_Req();
						}
						else{
							BiSS1_SPI_nCDM_Req();
						}
						BiSS2_SPI_nCDM_Req();
						// #endif
					}
					break;
				case BISS_SPI_CH_2:
					if (BiSS_C_Master_StateMachine(BiSS2_SPI_rx.CDS) == CDM) {					
						BiSS2_SPI_CDM_Req();
					}
					else{
						BiSS2_SPI_nCDM_Req();
					}
					if(CH1_SSI == CH1_SSI_FULL_FREQ){
						// #ifndef CH1_SSI
						BiSS1_SPI_nCDM_Req();
						// #endif
					}
					break;
				default:
					if(CH1_SSI == CH1_SSI_FULL_FREQ){
						// #ifndef CH1_SSI
						BiSS1_SPI_nCDM_Req();
						// #endif
					}
					BiSS2_SPI_nCDM_Req();
					break;
			}
			break;
			
		case BISS_MODE_AB_UART:
			if(LL_DMA_GetDataLength(DMA_BISS2_UART_RX) == 0) {
				if(BISS_CRC6_Calc(USART_rx.Data4CRC) == USART_rx.CRC6) {
					CRC6_State2 = CRC6_OK;
					AngleData2.angle_data = USART_rx.Pos;
					AngleData2.time_of_life_counter++;
					LED2TurnGreen();
				}
				else {
					LED2TurnRed();
					CRC6_State2 = CRC6_FAULT;
				}		
				if (BiSS_C_Master_StateMachine(USART_CDS_last) == CDM) {
					BissRequest_CDM();
					last_CDM = CDM;
				}
				else {
					BissRequest_nCDM();
					last_CDM = nCDM;
				}
				USART_CDS_last = (CDS_t)USART_rx.CDS;
				adr_reset_cou = 0;
			}
			else{
				LED2TurnRed();
				adr_reset_cou++;
				if(adr_reset_cou == 255) {
					RS485_ADR = RS485_ADR == RS485_ADR1? RS485_ADR2 : RS485_ADR1;
				}
				if(last_CDM == CDM) {
					BissRequest_CDM();
				}
				else {
					BissRequest_nCDM();
				}
			}
			if (LL_TIM_IsActiveFlag_UPDATE(TIM_RENISHAW)) {
				AngleDataRenishaw.angle_data = LL_TIM_GetCounter(TIM_RENISHAW);
				LED1TurnGreen();
			} else {
				LED1TurnRed();
			}
			break;
			
		case BISS_MODE_SPI_UART_IRS:
			break;
			
		case BISS_MODE_AB_SPI:
			BISS2_SCD = __REV(BiSS2_SPI_rx.revSCD);
			if(BISS_CRC6_Calc(BISS2_SCD >> 6) == (BISS2_SCD & 0x3FU)) {
				CRC6_State2 = CRC6_OK;
				AngleData2.angle_data = BISS2_SCD >> 8;
				AngleData2.time_of_life_counter++;
				if(((BISS2_SCD >> 6) & 0x3) == 0x3) {
 					LED2TurnGreen();
 				}
 				else {
 					LED2TurnRed();
 				}
			}
			else {
				LED2TurnRed();
				CRC6_State2 = CRC6_FAULT;
			}
			if(BiSS_SPI_Ch == BISS_SPI_CH_2) {
				if (BiSS_C_Master_StateMachine(BiSS2_SPI_rx.CDS) == CDM) {					
					BiSS2_SPI_CDM_Req();
				}
				else {
					BiSS2_SPI_nCDM_Req();
				}
				// BiSS1_SPI_nCDM_Req();
			}
			if (LL_TIM_IsActiveFlag_UPDATE(TIM_RENISHAW)) {
				AngleDataRenishaw.angle_data = LL_TIM_GetCounter(TIM_RENISHAW);
				LED1TurnGreen();
			}
			else {
				LED1TurnRed();
			}
			break;
		case BISS_MODE_DEFAULT_SPI:
			BISS2_SCD = __REV(BiSS2_SPI_rx.revSCD);
			if(BISS_CRC6_Calc(BISS2_SCD >> 6) == (BISS2_SCD & 0x3FU)) {
				CRC6_State2 = CRC6_OK;
				AngleData2.angle_data = BISS2_SCD >> 8;
				AngleData2.time_of_life_counter++;
				if(((BISS2_SCD >> 6) & 0x3) == 0x3) {
 					LED2TurnGreen();
 				}
 				else {
 					LED2TurnRed();
 				}
			}
			else {
				LED2TurnRed();
				CRC6_State2 = CRC6_FAULT;
			}
			if(BiSS_SPI_Ch == BISS_SPI_CH_2) {
				if (BiSS_C_Master_StateMachine(BiSS2_SPI_rx.CDS) == CDM) {					
					BiSS2_SPI_CDM_Req();
				}
				else {
					BiSS2_SPI_nCDM_Req();
				}
				// BiSS1_SPI_nCDM_Req();
			}
			break;
	}
	UART_StateMachine();
}

void Current_Sensor_Init(void){
	switch(Current_Sensor_Mode){
		case CURRENT_SENSOR_MODE_ENABLE:
			MX_ADC1_Init();
			Config_ADC1();
			break;
		case CURRENT_SENSOR_MODE_DISABLE:
			ADC1_DeInit();
			break;
	}
}

void BiSS_C_Master_HAL_Init(void){
	LED1TurnRed();
	LED2TurnRed();
	switch(Current_Mode){
		case BISS_MODE_SPI_SPI:
			BISS1_SPI_Init();
			BISS2_SPI_Init();
			break;
		case BISS_MODE_AB_UART:
			MX_TIM3_ENC_Init();
			Quadrature_Renishaw_Init();
			USART2_UART_Init();
			BISS_UART_Init();
			break;
		case BISS_MODE_SPI_UART_IRS:
			BISS1_SPI_Init();
			USART2_Init_IRS();
			USART2_Config_IRS();
			break;
		case BISS_MODE_AB_SPI:
			MX_TIM3_ENC_Init();
			Quadrature_Renishaw_Init();
			BISS2_SPI_Init();
			break;
		case BISS_MODE_DEFAULT_SPI:
			BISS2_SPI_Init();
			break;
	}
}

static void USART2_IRS_Transmit(uint8_t* data, uint8_t length){
	LL_DMA_DisableChannel(DMA_BISS2_UART_TX);
	LL_DMA_SetDataLength(DMA_BISS2_UART_TX, length);
	memcpy(usart2_tx_buffer, data, length);
	LL_USART_EnableDMAReq_TX(USART2);
	LL_DMA_EnableChannel(DMA_BISS2_UART_TX);
	
	uint32_t timeout = 20000;
	while(!LL_DMA_IsActiveFlag_HT6(DMA1)){
		if(--timeout == 0)
			break;
	}
	LL_DMA_ClearFlag_HT6(DMA1);

	while(!LL_DMA_IsActiveFlag_TC6(DMA1)){
		if(--timeout == 0)
			break;
	}
	LL_DMA_ClearFlag_TC6(DMA1);
}

static void USART2_IRS_Receive(uint8_t* data, uint8_t length){
	uint32_t timeout = 1000000;
	while (!LL_DMA_IsActiveFlag_TC5(DMA1)) {
		if(--timeout == 0)
			break;
	}
	LL_DMA_ClearFlag_TC5(DMA1);
	LL_DMA_ClearFlag_HT5(DMA1);
	memcpy(data,(uint8_t*)usart2_rx_buffer,RX_BUF_SIZE);
	memset((void*)usart2_rx_buffer,0,RX_BUF_SIZE);
}

static void Reenable_USART2_RX(void)
{
	LL_DMA_DisableChannel(DMA_BISS2_UART_RX);
	
	LL_DMA_SetMemoryAddress(DMA_BISS2_UART_RX, (uint32_t)usart2_rx_buffer);
	LL_DMA_SetDataLength(DMA_BISS2_UART_RX, RX_BUF_SIZE);
	LL_USART_EnableDMAReq_RX(USART2);
	
	LL_DMA_EnableChannel(DMA_BISS2_UART_RX);
}

void USART2_Write_Read_IRS(uint8_t* txData, uint8_t* rxData, uint8_t data_len)
{
	LED2TurnGreen();
	Reenable_USART2_RX();
	USART2_IRS_Transmit(txData, data_len);
	USART2_IRS_Receive(rxData, data_len);
	Reenable_USART2_RX();
	LED2TurnRed();
}

static void BISS1_SPI_Init(void)
{	
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	
  /* Peripheral clock enable */
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);	
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	BISS1_SPI_GRP_EN();
		
	/* Disable UartPin*/
	
	LL_GPIO_SetPinMode(BISS_MA_UART_PIN, LL_GPIO_MODE_ANALOG);
	
  /**SPI1 GPIO Configuration
  MA1_PIN		------> SPI_SCK		------>	PA5
  SLO1_PIN  ------> SPI_MISO	------>	PA6
	DE1_PIN		------>						------>	PA10
  */	
	
	LL_GPIO_SetOutputPin(PWR1_EN_PIN.port, PWR1_EN_PIN.pin);
	LL_GPIO_SetPinMode(PWR1_EN_PIN.port, PWR1_EN_PIN.pin, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(DE1_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	
	GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/* Enable DE2 PIN */
	LL_GPIO_SetOutputPin(DE1_PIN);
	
	
	GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* Enable MA2 PIN */
	LL_GPIO_SetOutputPin(MA1_PIN);
	
	
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*
	LL_GPIO_SetOutputPin(DE1_PIN);
	LL_GPIO_SetPinMode(DE1_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(DE1_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	
	LL_GPIO_SetPinMode(MA1_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetOutputPin(MA1_PIN);
	LL_GPIO_SetPinOutputType(MA1_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	
	LL_GPIO_SetPinMode(SLO1_PIN, LL_GPIO_MODE_ALTERNATE);
	
	BISS1_GPIO_SET_AF();
	*/

  /* SPI1 DMA Init */

  /* SPI1_RX Init */
	LL_DMA_DisableChannel(DMA_BISS1_RX);
  LL_DMA_SetPeriphRequest(DMA_BISS1_RX, DMA_BISS1_RX_Req);
  LL_DMA_SetDataTransferDirection(DMA_BISS1_RX, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);	
  LL_DMA_SetChannelPriorityLevel(DMA_BISS1_RX, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA_BISS1_RX, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA_BISS1_RX, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA_BISS1_RX, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA_BISS1_RX, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA_BISS1_RX, LL_DMA_MDATAALIGN_BYTE);

  /* SPI1_TX Init */
	LL_DMA_DisableChannel(DMA_BISS1_TX);
  LL_DMA_SetPeriphRequest(DMA_BISS1_TX, DMA_BISS1_TX_Req);
  LL_DMA_SetDataTransferDirection(DMA_BISS1_TX, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA_BISS1_TX, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA_BISS1_TX, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA_BISS1_TX, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA_BISS1_TX, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA_BISS1_TX, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA_BISS1_TX, LL_DMA_MDATAALIGN_BYTE);
	
	/* Init setup DMA/SPI */
	LL_SPI_Disable(BISS1_SPI);
	LL_DMA_SetPeriphAddress(DMA_BISS1_RX, (uint32_t) &BISS1_SPI->DR);
	if(CH1_SSI == CH1_SSI_HALF_FREQ){
		// #ifdef CH1_SSI
		LL_DMA_SetMemoryAddress(DMA_BISS1_RX, (uint32_t) &BiSS1_SPI_rx.buf[1]);
	} 
	else if(CH1_SSI == CH1_SSI_FULL_FREQ){
		// #else
		LL_DMA_SetMemoryAddress(DMA_BISS1_RX, (uint32_t) &BiSS1_SPI_rx.buf[3]);
		// #endif
	}
	LL_DMA_SetDataLength(DMA_BISS1_RX, 5);	
	LL_DMA_SetPeriphAddress(DMA_BISS1_TX, (uint32_t) &BISS1_SPI->DR);
	LL_DMA_SetDataLength(DMA_BISS1_TX, 5);	
	LL_SPI_EnableDMAReq_TX(BISS1_SPI);
	LL_SPI_EnableDMAReq_RX(BISS1_SPI);
	LL_SPI_Enable(BISS1_SPI);
	LL_DMA_EnableChannel(DMA_BISS1_RX);
}

static void BISS2_SPI_Init(void)
{	
	
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	
  /* Peripheral clock enable */
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);	
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	BISS2_SPI_GRP_EN();
		
	/* Disable UartPin*/
	
	LL_GPIO_SetPinMode(BISS_MA_UART_PIN, LL_GPIO_MODE_ANALOG);
	
  /**SPI2 GPIO Configuration
  MA2_PIN   ------> SPI_SCK 	------>	PB3
  SLO2_PIN  ------> SPI_MISO 	------>	PB4
	DE2_PIN		------>						------>	PA1
  */
	
	/* Enable PWR2 PIN */ 
	LL_GPIO_SetOutputPin(PWR2_EN_PIN);
	LL_GPIO_SetPinMode(PWR2_EN_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(PWR2_EN_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	
	/*
	LL_GPIO_SetOutputPin(DE2_PIN);
	LL_GPIO_SetPinMode(DE2_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(DE2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	*/
	
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* Enable DE2 PIN */
	LL_GPIO_SetOutputPin(DE2_PIN);
	
	
	GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	/* Enable MA2 PIN */
	LL_GPIO_SetOutputPin(MA2_PIN);
	
	/*
	LL_GPIO_SetPinMode(MA2_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetOutputPin(MA2_PIN);
	LL_GPIO_SetPinOutputType(MA2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	*/

	GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	// LL_GPIO_SetPinMode(SLO2_PIN, LL_GPIO_MODE_ALTERNATE);
	
	// BISS2_GPIO_SET_AF();

  /* SPI2 DMA Init */

  /* SPI2_RX Init */
	LL_DMA_DisableChannel(DMA_BISS2_RX);
  LL_DMA_SetPeriphRequest(DMA_BISS2_RX, DMA_BISS2_RX_Req);
  LL_DMA_SetDataTransferDirection(DMA_BISS2_RX, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);	
  LL_DMA_SetChannelPriorityLevel(DMA_BISS2_RX, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA_BISS2_RX, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA_BISS2_RX, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA_BISS2_RX, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA_BISS2_RX, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA_BISS2_RX, LL_DMA_MDATAALIGN_BYTE);

  /* SPI2_TX Init */
	LL_DMA_DisableChannel(DMA_BISS2_TX);
  LL_DMA_SetPeriphRequest(DMA_BISS2_TX, DMA_BISS2_TX_Req);
  LL_DMA_SetDataTransferDirection(DMA_BISS2_TX, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA_BISS2_TX, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA_BISS2_TX, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA_BISS2_TX, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA_BISS2_TX, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA_BISS2_TX, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA_BISS2_TX, LL_DMA_MDATAALIGN_BYTE);
	
	/* Init setup DMA/SPI */
	LL_SPI_Disable(BISS2_SPI);
	LL_DMA_SetPeriphAddress(DMA_BISS2_RX, (uint32_t) &BISS2_SPI->DR);
	LL_DMA_SetMemoryAddress(DMA_BISS2_RX, (uint32_t) &BiSS2_SPI_rx.buf[3]);
	LL_DMA_SetDataLength(DMA_BISS2_RX, 5);	
	LL_DMA_SetPeriphAddress(DMA_BISS2_TX, (uint32_t) &BISS2_SPI->DR);
	LL_DMA_SetDataLength(DMA_BISS2_TX, 5);	
	LL_SPI_EnableDMAReq_TX(BISS2_SPI);
	LL_SPI_EnableDMAReq_RX(BISS2_SPI);
	LL_SPI_Enable(BISS2_SPI);
	LL_DMA_EnableChannel(DMA_BISS2_RX);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**USART2 GPIO Configuration
  PA1   ------> USART2_DE
  PA15  ------> USART2_RX
  PB3   ------> USART2_TX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE; // LL_GPIO_MODE_OUTPUT
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART2 DMA Init */

  /* USART2_RX Init */
	LL_DMA_DisableChannel(DMA_BISS2_UART_RX);
  LL_DMA_SetPeriphRequest(DMA_BISS2_UART_RX, LL_DMAMUX_REQ_USART2_RX);
  LL_DMA_SetDataTransferDirection(DMA_BISS2_UART_RX, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA_BISS2_UART_RX, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA_BISS2_UART_RX, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA_BISS2_UART_RX, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA_BISS2_UART_RX, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA_BISS2_UART_RX, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA_BISS2_UART_RX, LL_DMA_MDATAALIGN_BYTE);
	
	/* USART2_TX Init */
	LL_DMA_DisableChannel(DMA_BISS2_UART_TX);
	LL_DMA_SetPeriphRequest(DMA_BISS2_UART_TX, LL_DMAMUX_REQ_USART2_TX);
  LL_DMA_SetDataTransferDirection(DMA_BISS2_UART_TX, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA_BISS2_UART_TX, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA_BISS2_UART_TX, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA_BISS2_UART_TX, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA_BISS2_UART_TX, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA_BISS2_UART_TX, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA_BISS2_UART_TX, LL_DMA_MDATAALIGN_BYTE);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 3000000;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_EnableDEMode(USART2);
  LL_USART_SetDESignalPolarity(USART2, LL_USART_DE_POLARITY_HIGH);
  LL_USART_SetDEAssertionTime(USART2, 0);
  LL_USART_SetDEDeassertionTime(USART2, 0);
  LL_USART_DisableFIFO(USART2);
  LL_USART_ConfigAsyncMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);
	LL_DMA_EnableChannel(DMA_BISS2_UART_RX);
	LL_DMA_EnableChannel(DMA_BISS2_UART_TX);
	
  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

static void BISS_UART_Init(void)
{
	/* Enable Power2 Enc PIN */
	LL_GPIO_SetOutputPin(PWR2_EN_PIN);
	//LL_GPIO_SetPinMode(PWR2_EN_PIN, LL_GPIO_MODE_OUTPUT);
	//LL_GPIO_SetPinOutputType(PWR2_EN_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	
	//LL_GPIO_ResetOutputPin(DE2_PIN); 
	//LL_GPIO_SetPinMode(DE2_PIN, LL_GPIO_MODE_OUTPUT);
	//LL_GPIO_SetPinOutputType(DE2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	
	LL_USART_Disable(BISS2_UART);		
	LL_GPIO_SetPinMode(MA2_PIN, LL_GPIO_MODE_ALTERNATE); // LL_GPIO_MODE_ANALOG
	LL_GPIO_SetPinMode(BISS_MA_UART_PIN, LL_GPIO_MODE_ALTERNATE);
	
	/* Init setup DMA/UART RX */	
	LL_DMA_SetPeriphAddress(DMA_BISS2_UART_RX, (uint32_t) &BISS2_UART->RDR);
	LL_DMA_SetMemoryAddress(DMA_BISS2_UART_RX, (uint32_t) &USART_rx.u32);
	LL_DMA_SetDataLength(DMA_BISS2_UART_RX, 4);	
	LL_USART_EnableDMAReq_RX(BISS2_UART);
	
	/* Init setup DMA/UART TX */	
	//LL_DMA_SetPeriphAddress(DMA_BISS2_UART_TX, (uint32_t) &BISS2_UART->TDR);  // USART TX Register
	//LL_DMA_SetMemoryAddress(DMA_BISS2_UART_TX, (uint32_t) &USART_tx.u32);     // Pointer to TX buffer
	//LL_DMA_SetDataLength(DMA_BISS2_UART_TX, 4);                  							// Number of bytes to send
	//LL_USART_EnableDMAReq_TX(BISS2_UART);																			// Enable DMA for USART TX
	
	LL_USART_SetDEAssertionTime(BISS2_UART, 16U);
	LL_USART_SetDEDeassertionTime(BISS2_UART, 16U);
	LL_USART_Enable(BISS2_UART);		
	LL_DMA_EnableChannel(DMA_BISS2_UART_RX);
	//LL_DMA_EnableChannel(DMA_BISS2_UART_TX);                                  // Enable DMA TX channel
	//LL_USART_EnableDMAReq_TX(BISS2_UART);                                     // Enable DMA for USART TX
}

static void USART2_Init_IRS(void)
{
	/* Enable Power2 Enc PIN */
	LL_GPIO_SetOutputPin(PWR2_EN_PIN);
	LL_GPIO_SetPinMode(PWR2_EN_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(PWR2_EN_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	
	// GPIO struct Init
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	// USART struct Init
	LL_USART_InitTypeDef USART_InitStruct = {0};
	// Enable the clocks for the required GPIO ports
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
	// Enable USART2 peripheral clock
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
	// Enable DMAMUX1 peripheral clock
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
	// Enable DMA1 peripheral clock
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	/**USART2 GPIO Configuration
  PA1   ------> USART2_DE
	PB4   ------> USART2_RX
  PB3   ------> USART2_TX
	*/
	// PA1 -> USART2_DE
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* Enable DE2 PIN */
	LL_GPIO_SetOutputPin(DE2_PIN);
	
	// PB4 -> USART2_RX (input)
	GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// PB3 -> USART2_TX (output)
	GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/* USART2 RX Init DMA (DMA_BISS2_UART_RX -> DMA1, LL_DMA_CHANNEL_5)*/
	LL_DMA_SetPeriphRequest(DMA_BISS2_UART_RX, LL_DMAMUX_REQ_USART2_RX);
  LL_DMA_SetDataTransferDirection(DMA_BISS2_UART_RX, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA_BISS2_UART_RX, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA_BISS2_UART_RX, LL_DMA_MODE_NORMAL); // LL_DMA_MODE_NORMAL LL_DMA_MODE_CIRCULAR
  LL_DMA_SetPeriphIncMode(DMA_BISS2_UART_RX, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA_BISS2_UART_RX, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA_BISS2_UART_RX, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA_BISS2_UART_RX, LL_DMA_MDATAALIGN_BYTE);
	
	/* USART2 TX Init DMA (DMA_BISS2_UART_TX -> DMA1, LL_DMA_CHANNEL_6)*/
	LL_DMA_SetPeriphRequest(DMA_BISS2_UART_TX, LL_DMAMUX_REQ_USART2_TX);
  LL_DMA_SetDataTransferDirection(DMA_BISS2_UART_TX, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetChannelPriorityLevel(DMA_BISS2_UART_TX, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA_BISS2_UART_TX, LL_DMA_MODE_NORMAL); // LL_DMA_MODE_NORMAL LL_DMA_MODE_CIRCULAR
  LL_DMA_SetPeriphIncMode(DMA_BISS2_UART_TX, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA_BISS2_UART_TX, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA_BISS2_UART_TX, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA_BISS2_UART_TX, LL_DMA_MDATAALIGN_BYTE);
	
	/* USART2 Configuration */
	USART_InitStruct.BaudRate = 460800;      // Baud rate
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	
	// Init USART2
	LL_USART_Init(USART2, &USART_InitStruct); 
	
	// Enable USART2
	LL_USART_Enable(USART2);
	while (!LL_USART_IsActiveFlag_TEACK(USART2) || !LL_USART_IsActiveFlag_REACK(USART2)){
	}
}

static void USART2_Config_IRS(void)
{
	LL_TIM_EnableIT_UPDATE(BISS_Task_TIM);
	LL_TIM_EnableCounter(BISS_Task_TIM);
	LL_DMA_SetMemoryAddress(DMA_BISS2_UART_RX, (uint32_t)usart2_rx_buffer);
	LL_DMA_SetPeriphAddress(DMA_BISS2_UART_RX, (uint32_t)&USART2->RDR);
	LL_DMA_SetDataLength(DMA_BISS2_UART_RX, RX_BUF_SIZE);
	LL_DMA_SetMemoryAddress(DMA_BISS2_UART_TX, (uint32_t)usart2_tx_buffer);
	LL_DMA_SetPeriphAddress(DMA_BISS2_UART_TX, (uint32_t)&USART2->TDR);
	// LL_DMA_SetDataLength(DMA_BISS2_UART_TX, TX_BUF_SIZE);
	LL_USART_EnableDMAReq_TX(USART2);
	LL_USART_EnableDMAReq_RX(USART2);
	LL_DMA_EnableChannel(DMA_BISS2_UART_RX);
	// LL_DMA_EnableChannel(DMA_BISS2_UART_TX);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_ENC_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
	
	LL_TIM_SetRemap(TIM3, LL_TIM_TIM3_TI1_RMP_COMP1);
	
  /* USER CODE END TIM3_Init 0 */
	
  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	
  /**TIM3 GPIO Configuration
  PA4   ------> TIM3_CH2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  /* USER CODE BEGIN TIM3_Init 1 */
	/**TIM3 GPIO Configuration
  PA6   ------> TIM3_CH1
  */	
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END TIM3_Init 1 */
	
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetEncoderMode(TIM3, LL_TIM_ENCODERMODE_X2_TI1);
  LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  //LL_TIM_SetRemap(TIM3, LL_TIM_TIM3_TI1_RMP_COMP1);
	
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0   ------> ADC1_IN1
	PB0   ------> ADC1_IN15
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE BEGIN ADC1_Init 1 */
	
	/* ADC1_DMA1 Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_6, LL_DMAMUX_REQ_ADC1);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MDATAALIGN_HALFWORD);
	
  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
	
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS; // LL_ADC_REG_CONV_SINGLE 
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN; // LL_ADC_REG_OVR_DATA_PRESERVED
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
	
  LL_ADC_SetGainCompensation(ADC1, 0);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
  LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_256, LL_ADC_OVS_SHIFT_RIGHT_4);
	LL_ADC_SetOverSamplingDiscont(ADC1, LL_ADC_OVS_SHIFT_RIGHT_4);

  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  volatile uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
	// LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_CONTINUOUS);
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_15);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_15, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_15, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC1_Init 2 */
	
	/* GPIO Init PA8 PIN
	PA8   ------> En1 
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* Set Enc1 power to PA8 PIN */
	PWR1_EN_PIN.port = GPIOA;
	PWR1_EN_PIN.pin = LL_GPIO_PIN_8;
	/* Enable PA8 PIN */
	LL_GPIO_SetOutputPin(PWR1_EN_PIN.port, PWR1_EN_PIN.pin);

  /* USER CODE END ADC1_Init 2 */

}

static void Config_ADC1(void)
{
	/* Set DMA buffer length and address */ 
	LL_DMA_SetMemoryAddress(DMA_ADC1, (uint32_t)&adc1_buffer);
	LL_DMA_SetPeriphAddress(DMA_ADC1, (uint32_t)&ADC1->DR);
	LL_DMA_SetDataLength(DMA_ADC1, ADC_BUF_SIZE);
	
	/* Enable DMA channel */
	LL_DMA_EnableChannel(DMA_ADC1);
	
	/* Enable DMA for ADC1 unlimited transfer */
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
	
	/* Start calibration for ADC1 */
	LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
  while(LL_ADC_IsCalibrationOnGoing(ADC1));
	/* Wait 4 ADC clock cycles */ 
	volatile uint32_t wait_loop_index = 4;
	while(wait_loop_index != 0) {
    wait_loop_index--;
  }
	
	/* Enable ADC1 and start conversion */
  LL_ADC_Enable(ADC1);
	while(!LL_ADC_IsActiveFlag_ADRDY(ADC1));
	LL_ADC_REG_StartConversion(ADC1);
}

static void ADC1_DeInit(void)
{
	/* Disable ADC1 */
	LL_ADC_REG_StopConversion(ADC1);
	
	if(LL_ADC_IsEnabled(ADC1))
	{
			LL_ADC_Disable(ADC1);
			while(LL_ADC_IsEnabled(ADC1) != 0);
	}
	
	LL_ADC_DisableInternalRegulator(ADC1);

	LL_ADC_DisableDeepPowerDown(ADC1);

	LL_ADC_DeInit(ADC1);

	LL_AHB2_GRP1_DisableClock(LL_AHB2_GRP1_PERIPH_ADC12);
	
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* GPIO Init PB0 PIN
	PB0   ------> En1 
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	/* Set Enc1 power to PB0 PIN */
	PWR1_EN_PIN.port = GPIOB;
	PWR1_EN_PIN.pin = LL_GPIO_PIN_0;
	/* Enable PB0 PIN */
	LL_GPIO_SetOutputPin(PWR1_EN_PIN.port, PWR1_EN_PIN.pin);
	
	/* GPIO Init PA8 PIN */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	/* Disable PA8 PIN */
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
	/* Disable DMA for ADC1 */
	LL_DMA_DisableChannel(DMA_ADC1);
	/* Clear ADC buffer */
	memset((void*)adc1_buffer, 0, ADC_BUF_SIZE*sizeof(adc1_buffer[0]));
}

int32_t Read_Current_Enc2(void){
	uint16_t adc_value = adc1_buffer[0] >> 4;
	if (adc_value > ADC_MAX){
				adc_value = ADC_MAX; // Prevent overflow
	}
	return (((adc_value * VREF) / ADC_MAX) * CURRENT_GAIN) / 1000;
}

static void Quadrature_Renishaw_Init(void){
	LL_GPIO_ResetOutputPin(DE1_PIN);
	LL_GPIO_SetOutputPin(PWR1_EN_PIN.port, PWR1_EN_PIN.pin);
	LL_TIM_SetCounter(TIM_RENISHAW, 0);
	LL_TIM_EnableCounter(TIM_RENISHAW);
}

static void BISS1_SPI_DeInit(void){
	LL_GPIO_SetPinMode(MA1_PIN, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(MA1_PIN, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(SLO1_PIN, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(SLO1_PIN, LL_GPIO_MODE_ANALOG);
	LL_GPIO_ResetOutputPin(PWR1_EN_PIN.port, PWR1_EN_PIN.pin);
	LL_GPIO_ResetOutputPin(DE1_PIN);
	LL_SPI_Disable(BISS1_SPI);
	LL_SPI_Disable(BISS1_SPI);
	LL_DMA_DisableChannel(DMA_BISS1_RX);
	LL_DMA_DisableChannel(DMA_BISS1_TX);
	memset((void*)BiSS1_SPI_rx.buf, 0, sizeof(BiSS1_SPI_rx.buf));
}

static void BISS2_SPI_DeInit(void){
	LL_GPIO_SetPinMode(MA2_PIN, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(MA2_PIN, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(SLO2_PIN, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(SLO2_PIN, LL_GPIO_MODE_ANALOG);
	LL_GPIO_ResetOutputPin(PWR2_EN_PIN);
	LL_GPIO_ResetOutputPin(DE2_PIN);
	LL_SPI_Disable(BISS2_SPI);
	LL_SPI_Disable(BISS2_SPI);
	LL_DMA_DisableChannel(DMA_BISS2_RX);
	LL_DMA_DisableChannel(DMA_BISS2_TX);
	memset((void*)BiSS2_SPI_rx.buf, 0, sizeof(BiSS2_SPI_rx.buf));
}

static void BISS_UART_DeInit(void){
	LL_USART_Disable(BISS2_UART);
	LL_USART_Disable(BISS2_UART);
	LL_DMA_DisableChannel(DMA_BISS2_UART_RX);
	LL_DMA_DisableChannel(DMA_BISS2_UART_TX);
	memset((void*)USART_rx.u32, 0, sizeof(USART_rx.u32));
}	

static void Quadrature_Renishaw_DeInit(void){
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ANALOG);
	LL_GPIO_ResetOutputPin(PWR1_EN_PIN.port, PWR1_EN_PIN.pin);
	LL_GPIO_ResetOutputPin(DE1_PIN);
	LL_TIM_DisableCounter(TIM_RENISHAW);
	LL_TIM_DisableCounter(TIM_RENISHAW);
	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM3);
	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM3);
}

static void BISS_USART2_DeInit(void){
	LL_USART_Disable(BISS2_UART);
	LL_USART_Disable(BISS2_UART);
	LL_DMA_DisableChannel(DMA_BISS2_UART_RX);
	LL_DMA_DisableChannel(DMA_BISS2_UART_TX);
	LL_GPIO_ResetOutputPin(PWR2_EN_PIN);
	LL_GPIO_ResetOutputPin(DE2_PIN);
	memset((void*)usart2_tx_buffer, 0, TX_BUF_SIZE);
	memset((void*)usart2_rx_buffer, 0, RX_BUF_SIZE);
}

void SetBiSS_SPI_Ch(BiSS_SPI_Ch_t ch_to_set){
	// #ifndef CH1_SSI
	if(CH1_SSI == CH1_SSI_FULL_FREQ){
		if(IsBiSSReqBusy() == BISS_REQ_OK){
			BiSS_SPI_Ch = ch_to_set;
		}	
	}	
	// #endif
}

void SetCh1SSIFreq(CH1_SSI_t freq_to_set){
	if(IsBiSSReqBusy() == BISS_REQ_OK){
		CH1_SSI = freq_to_set;
	}
}

void Stop_Current_Mode(void){
	LED1TurnRed();
	LED2TurnRed();
	if(Current_Mode == BISS_MODE_SPI_SPI){
		BISS1_SPI_DeInit();
		BISS2_SPI_DeInit();
	}else if(Current_Mode == BISS_MODE_AB_UART){
		Quadrature_Renishaw_DeInit();
		BISS_UART_DeInit();
	} else if(Current_Mode == BISS_MODE_SPI_UART_IRS){
		BISS1_SPI_DeInit();
		BISS_USART2_DeInit();
	} else if(Current_Mode == BISS_MODE_AB_SPI){
		Quadrature_Renishaw_DeInit();
		BISS2_SPI_DeInit();
	} else if(Current_Mode == BISS_MODE_DEFAULT_SPI){
		BISS2_SPI_DeInit();
	}
}

void Change_Current_Mode(BISS_Mode_t New_Mode){
	Stop_Current_Mode();
	Current_Mode = New_Mode;
	if(Current_Mode == BISS_MODE_SPI_SPI){
		BiSS_C_Master_HAL_Init();
	} else if(Current_Mode == BISS_MODE_AB_UART){
		BiSS_C_Master_HAL_Init();
	} else if(Current_Mode == BISS_MODE_SPI_UART_IRS){
		BiSS_C_Master_HAL_Init();
	} else if(Current_Mode == BISS_MODE_AB_SPI){
		BiSS_C_Master_HAL_Init();
	} else if(Current_Mode == BISS_MODE_DEFAULT_SPI){
		BiSS_C_Master_HAL_Init();
	}
}

void Change_Current_Sensor_Mode(Current_Sensor_Mode_t New_Mode){
	Current_Sensor_Mode = New_Mode;
	if(Current_Sensor_Mode == CURRENT_SENSOR_MODE_ENABLE){
		Current_Sensor_Init();
	} else if(Current_Sensor_Mode == CURRENT_SENSOR_MODE_DISABLE){
		Current_Sensor_Init();
	}
}