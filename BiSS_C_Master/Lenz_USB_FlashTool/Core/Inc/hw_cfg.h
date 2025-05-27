/*!
 * @file hw_cfg.h
 * @author Kirill Rostovskiy (kmrost@lenzencoders.com)
 * @brief Hardware configurator file
 * @version 0.1
 * @copyright Lenz Encoders (c) 2024
 */

#ifndef _HW_CFG_H_
#define _HW_CFG_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
	BISS_MODE_SPI_SPI = 0,
	BISS_MODE_AB_UART = 1u,
	BISS_MODE_SPI_UART_IRS = 2u,
	BISS_MODE_AB_SPI = 3u,
	BISS_MODE_DEFAULT_SPI = 4u,
} BISS_Mode_t;

extern volatile BISS_Mode_t Current_Mode;

typedef enum{
	CH1_SSI_HALF_FREQ = 0,
	CH1_SSI_FULL_FREQ = 1,
} CH1_SSI_t;

extern volatile CH1_SSI_t CH1_SSI;

typedef enum{
	CURRENT_SENSOR_MODE_DISABLE = 0,
	CURRENT_SENSOR_MODE_ENABLE = 1,
} Current_Sensor_Mode_t;

extern volatile Current_Sensor_Mode_t Current_Sensor_Mode;

// #define CH1_SSI
	
#define BISS_Task_TIM 					TIM7
#define BISS_Task_IRQHandler 		TIM7_IRQHandler
#define DMA_LPUART_RX 					DMA1, LL_DMA_CHANNEL_1
#define DMA_LPUART_TX 					DMA1, LL_DMA_CHANNEL_2
#define DMA_ADC1							  DMA1, LL_DMA_CHANNEL_6

/* BiSS1 SPI Config*/
#define BISS1_SPI								SPI1
#define MA1_PIN									GPIOA, LL_GPIO_PIN_5
#define SLO1_PIN								GPIOA, LL_GPIO_PIN_6
#define DMA_BISS1_RX 						DMA1, LL_DMA_CHANNEL_3
#define DMA_BISS1_RX_Req				LL_DMAMUX_REQ_SPI1_RX
#define DMA_BISS1_TX 						DMA1, LL_DMA_CHANNEL_4
#define DMA_BISS1_TX_Req				LL_DMAMUX_REQ_SPI1_TX

#define BISS1_SPI_GRP_EN()			LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
#define BISS1_GPIO_SET_AF()     LL_GPIO_SetAFPin_0_7(MA1_PIN, LL_GPIO_AF_5); LL_GPIO_SetAFPin_0_7(SLO1_PIN, LL_GPIO_AF_5)
/* End BiSS1 SPI Config*/

/* BiSS2 SPI Config*/
#define BISS2_SPI								SPI3
#define MA2_PIN									GPIOB, LL_GPIO_PIN_3
#define SLO2_PIN								GPIOB, LL_GPIO_PIN_4
#define DMA_BISS2_RX 						DMA2, LL_DMA_CHANNEL_1
#define DMA_BISS2_RX_Req				LL_DMAMUX_REQ_SPI3_RX
#define DMA_BISS2_TX 						DMA2, LL_DMA_CHANNEL_2
#define DMA_BISS2_TX_Req				LL_DMAMUX_REQ_SPI3_TX

#define BISS2_SPI_GRP_EN()			LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
#define BISS2_GPIO_SET_AF()     LL_GPIO_SetAFPin_0_7(MA2_PIN, LL_GPIO_AF_6); LL_GPIO_SetAFPin_0_7(SLO2_PIN, LL_GPIO_AF_6)
/* End BiSS1 SPI Config*/

#define BISS2_UART							USART2
#define DMA_BISS2_UART_RX				DMA1, LL_DMA_CHANNEL_5
#define DMA_BISS2_UART_TX				DMA1, LL_DMA_CHANNEL_6
				
//#define BISS_SLO_DE_PIN					GPIOA, LL_GPIO_PIN_10 /* RS485 Slave Out (uart in_out) Driver enable pin */
#define BISS_MA_UART_PIN				GPIOA, LL_GPIO_PIN_15
#define BISS_SLO_UART_PIN				GPIOB, LL_GPIO_PIN_4

/* END BISS C Config*/
// #define PWR1_EN_PIN							GPIOA, LL_GPIO_PIN_8  // GPIOB, LL_GPIO_PIN_0
#define PWR2_EN_PIN							GPIOB, LL_GPIO_PIN_7
#define DE1_PIN									GPIOA, LL_GPIO_PIN_10
#define DE2_PIN									GPIOA, LL_GPIO_PIN_1
#define TIM_RENISHAW						TIM3

#define LED1_RED								GPIOA, LL_GPIO_PIN_11
#define LED1_GREEN							GPIOA, LL_GPIO_PIN_12
#define LED2_RED								GPIOB, LL_GPIO_PIN_5
#define LED2_GREEN							GPIOB, LL_GPIO_PIN_6

//BISS_MODE BissGetState(void);


#ifdef __cplusplus
}
#endif

#endif /* _HW_CFG_H_ */