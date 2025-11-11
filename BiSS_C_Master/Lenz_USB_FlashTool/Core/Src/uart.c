/*!
 * @file uart.c
 * @author Kirill Rostovskiy (kmrost@lenzencoders.com)
 * @brief UART library
 * @version 0.1
 * @copyright Lenz Encoders (c) 2024
 */
#include "uart.h"
#include "string.h"
#include "stm32g4xx_ll_lpuart.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_gpio.h"
#include "main.h"
#include "hw_cfg.h"
#include "biss_c_master.h"
#include "biss_c_master_hal.h"
#include "tamp_access.h"


//#define UART_LINE_SIZE		133U
#define HEXLEN_ADR_CMD_CRC_LEN								5U // Length of data (1) + Address (2) + Cmd (1) + CRC (1) bytes
#define HEX_DATA_LEN													128U
#define UART_LINE_SIZE												HEXLEN_ADR_CMD_CRC_LEN + HEX_DATA_LEN
#define QUEUE_SIZE 														36U //FIFO
#define MAX_RETRY															3U
#define UART_ANGLE_LEN 												60U // 60U --> Encoder
#define UART_ANGLE_BUF_SIZE 									(UART_ANGLE_LEN * 4U) // *4U --> Encoder
#define UART_ANGLE_TWO_ENC_LEN 								30U // 30U --> Encoder1 + Encoder2
#define UART_ANGLE_TWO_ENC_BUF_SIZE 					(UART_ANGLE_TWO_ENC_LEN * 8U) // *(4U + 4U) --> Encoder1 + Encoder2
#define UART_ANGLE_TWO_ENC_AB_UART_LEN 				40U // 40U --> Encoder1 + Renishaw
#define UART_ANGLE_TWO_ENC_AB_UART_BUF_SIZE 	(UART_ANGLE_TWO_ENC_AB_UART_LEN * 6U) // *(4U + 2U) --> Encoder1 + Renishaw
#define ANGLE_DATA_SIZE												4U
#define RENISHAW_ANGLE_DATA_SIZE							2U
#define WRITE_BANK_DATA_LEN 									64U
#define PAGE_ADDR       											0x18U
#define BSEL_ADDR		    											0x40U
#define FIRST_USER_BANK 											0x05U
#define BISS_ABORT_CNT_CYCLES 								14U		/* Cycles to abort control data frame */

typedef enum{
    ERROR_TYPE_BISS = 0xDEU,
    ERROR_TYPE_UART = 0xEFU,
		ERROR_TYPE_NONE = 0xFFU,
}UART_Error_Type_t;

typedef enum{
	UART_STATE_IDLE,
	UART_STATE_RECEIVE,
	UART_STATE_SEND,
	UART_STATE_CHECKCRC,
	UART_STATE_RUNCMD,
	UART_STATE_ANGLE_READING_TWO_ENC_AB_SPI,
	UART_STATE_ANGLE_READING_TWO_ENC_AB_UART,
	UART_STATE_ANGLE_READING_TWO_ENC_SPI,
	UART_STATE_ANGLE_READING_ENC_SPI,
	UART_STATE_ABORT,
}UART_State_t;

typedef enum{
	UART_ERROR_NONE = 0x00,
	UART_ERROR_CRC = 0x01U,
	UART_ERROR_QUEUE_FULL = 0x02U,
	UART_ERROR_BISS = 0x03U,
	UART_ERROR_BISS_WRITE_FAULT = 0x04U,
	UART_ERROR_BISS_READ_FAULT = 0x05U,
	UART_ERROR_LEN_DATA_IS_ZERO = 0x06U,
	UART_ERROR_LEN_IS_NOT_CORRECT = 0x07U,
	UART_ERROR_INVALID_CMD = 0x08U,
}UART_Error_t;

volatile enum{
	CRC_OK,
	CRC_FAULT
}CRC_State = CRC_FAULT;

typedef enum{
	QUEUE_OK,
	QUEUE_FULL
}QUEUE_Status_t;

typedef struct{
	uint8_t len;
	uint16_t addr;
	UART_Command_t cmd;
	uint8_t data[HEX_DATA_LEN];
}CommandQueue_t;

volatile struct{
	AngleData_t AngleFIFO[256];
	uint16_t len;
	uint8_t ToL_cnt;
	uint8_t FIFO_start_ptr;
	uint8_t FIFO_current_ptr;
}ReadingStrEnc1;

volatile struct{
	AngleData_t AngleFIFO[256];
	uint16_t len;
	uint8_t ToL_cnt;
	uint8_t FIFO_start_ptr;
	uint8_t FIFO_current_ptr;
}ReadingStrEnc2;

volatile struct{
	AngleDataRenishaw_t AngleFIFO[256];
	uint16_t len;
	uint8_t FIFO_start_ptr;
	uint8_t FIFO_current_ptr;
}ReadingStrRenishaw;

UartTxStr_t UART_TX;
UART_Error_t UART_Error = UART_ERROR_NONE;
UART_State_t UART_State = UART_STATE_IDLE;
UART_Error_Type_t UART_Error_Type = ERROR_TYPE_NONE;
CommandQueue_t CommandQueue[QUEUE_SIZE];

volatile uint8_t usb_rx_buffer[RX_BUFFER_SIZE] = {0};
uint8_t usb_tx_buffer[TX_BUFFER_SIZE] = {0};
uint8_t hex_line_buffer[UART_LINE_SIZE] = {0};

uint32_t dma_rx_cnt = 0; 
volatile uint32_t uart_expected_length = 0; 
volatile uint8_t uart_length = 0;
volatile uint32_t new_cnt = 0;
uint8_t queue_read_cnt = 0;
uint8_t queue_write_cnt = 0;
uint8_t queue_cnt = 0;
uint8_t retry_cnt = 0;

void JumpToBootloader(void);
void UART_Config(void);
void UART_Transmit(UartTxStr_t *TxStr);
static QUEUE_Status_t EnqueueCommand(UART_Command_t cmd, uint16_t addr, uint8_t len,	uint8_t *data);
static QUEUE_Status_t EnqueueCommandToBegining(UART_Command_t cmd, uint16_t addr, uint8_t len,	uint8_t *data);
static uint8_t CalculateCRC(uint8_t *data, uint32_t length);
static uint8_t CalculateCRCCircularBuffer(uint8_t *buffer, uint16_t buffer_size, uint8_t start_index, uint8_t length);
static void complete_command_processing(void);
static void handle_write_bank_command(uint8_t cmd_data_len, uint16_t cmd_addr, uint8_t* cmd_data);
static void handle_change_mode_command(uint8_t mode);
static void handle_page_command(uint8_t data_to_page);
static void handle_select_spi_ch_command(uint8_t channel);
static void handle_change_ch1_mode_command(uint8_t ch1_mode);
static void handle_write_reg_command(uint8_t cmd_data_len, uint16_t cmd_addr, uint8_t* cmd_data);
static void handle_read_reg_command(uint8_t cmd_data_len, uint16_t cmd_addr, UART_Command_t command);
static void handle_write_read_enc_usart2_command(uint8_t cmd_data_len, uint8_t* cmd_data, UART_Command_t command);
static void handle_read_angle_enc_spi_instant_command(uint8_t cmd_data_len, UART_Command_t command);
static void handle_change_current_sensor_mode_command(uint8_t current_mode);
static void handle_read_enc2_current_command(uint8_t cmd_data_len, UART_Command_t command);
static void prepare_encoder_ab_spi_reading(uint16_t cmd_addr, UART_Command_t command);
static void prepare_encoder_ab_uart_reading(uint16_t cmd_addr, UART_Command_t command);
static void prepare_encoder_spi_spi_reading(uint16_t cmd_addr, UART_Command_t command);
static void prepare_encoder_spi_reading(uint16_t cmd_addr, UART_Command_t command);
static void handle_idle_state(void);
static void handle_receive_state(uint8_t crc, uint8_t calculated_crc);
static void handle_run_command_state(void);
static void handle_reading_encoder_ab_spi_state(void);
static void handle_reading_encoder_spi_spi_state(void);
static void handle_reading_encoder_ab_uart_state(void);
static void handle_reading_encoder_spi_state(void);
static void handle_abort_state(void);


void TAMP_Init(void) {
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_RTCAPB);

	LL_RCC_EnableRTC();
	LL_PWR_EnableBkUpAccess();
	
	LL_RTC_TAMPER_Disable(RTC, LL_RTC_TAMPER_1);
	LL_RTC_TAMPER_SetFilterCount(RTC, LL_RTC_TAMPER_FILTER_4SAMPLE);
  SET_BIT(TAMP->IER, TAMP_IER_TAMP1IE);
	
	LL_RTC_TAMPER_Enable(RTC, LL_RTC_TAMPER_1);
	
	NVIC_SetPriority(RTC_TAMP_LSECSS_IRQn, 1);
	NVIC_EnableIRQ(RTC_TAMP_LSECSS_IRQn);
}

void TAMP_DeInit(void)
{
	CLEAR_BIT(TAMP->IER, TAMP_IER_TAMP1IE);
	NVIC_DisableIRQ(RTC_TAMP_LSECSS_IRQn);
	
	LL_RTC_TAMPER_Disable(RTC, LL_RTC_TAMPER_1);
	
	WRITE_REG(TAMP->SCR, TAMP_SCR_CTAMP1F);
 
	LL_RTC_TAMPER_SetFilterCount(RTC, LL_RTC_TAMPER_FILTER_DISABLE);
	
	LL_PWR_DisableBkUpAccess();
	
	LL_RCC_DisableRTC();
	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_RTCAPB);
}

void Stay_in_FW_Config(void) {
	ClearTampFlag(TAMP_FLAGS_STAY_BL);
	SetTampFlag(TAMP_FLAGS_STAY_MAIN_FW);
}

void JumpToBootloader(void) {
		ClearTampFlag(TAMP_FLAGS_STAY_MAIN_FW);
		SetTampFlag(TAMP_FLAGS_STAY_BL);
//		ClearTampFlag(TAMP_FLAGS_STAY_MAIN_FW);
		TAMP_DeInit();
    NVIC_SystemReset();
}

QUEUE_Status_t EnqueueCommand(UART_Command_t cmd, uint16_t addr, uint8_t len,	uint8_t *data) {
	if (queue_cnt < QUEUE_SIZE){
		CommandQueue[queue_write_cnt].len = len;
		CommandQueue[queue_write_cnt].addr = addr;
		CommandQueue[queue_write_cnt].cmd = cmd;
		memcpy(CommandQueue[queue_write_cnt].data, data, len);
		queue_write_cnt = (queue_write_cnt + 1U) % QUEUE_SIZE;
		queue_cnt++;
		return QUEUE_OK;
	}
	return QUEUE_FULL;
}

QUEUE_Status_t EnqueueCommandToBegining(UART_Command_t cmd, uint16_t addr, uint8_t len,	uint8_t *data) {
	if (queue_cnt < QUEUE_SIZE) {
		queue_read_cnt = (queue_read_cnt + QUEUE_SIZE - 1U) % QUEUE_SIZE;
		CommandQueue[queue_read_cnt].len = len;
		CommandQueue[queue_read_cnt].addr = addr;
		CommandQueue[queue_read_cnt].cmd = cmd;
		memcpy(CommandQueue[queue_read_cnt].data, data, len);
		queue_cnt++;
		return QUEUE_OK;
	}
	return QUEUE_FULL;
}

void UART_Config(void) {
	LL_TIM_EnableIT_UPDATE(BISS_Task_TIM);
	LL_TIM_EnableCounter(BISS_Task_TIM);
	LL_DMA_SetMemoryAddress(DMA_LPUART_RX, (uint32_t)usb_rx_buffer);
	LL_DMA_SetPeriphAddress(DMA_LPUART_RX, (uint32_t)&LPUART1->RDR);
	LL_DMA_SetMemoryAddress(DMA_LPUART_TX, (uint32_t)usb_tx_buffer);
	LL_DMA_SetPeriphAddress(DMA_LPUART_TX, (uint32_t)&LPUART1->TDR);
	LL_DMA_SetDataLength(DMA_LPUART_RX, RX_BUFFER_SIZE);
	LL_LPUART_EnableDMAReq_RX(LPUART1);
	LL_LPUART_EnableDMAReq_TX(LPUART1);
	LL_DMA_EnableChannel(DMA_LPUART_RX);
}

uint8_t CalculateCRC(uint8_t *data, uint32_t length) {
    uint32_t sum = 0;
    for (uint32_t i = 0; i < length; i++) {
        sum += data[i];
    }
    uint8_t lsb = sum & 0xFF;
    return (uint8_t)(~lsb + 1);
}

uint8_t CalculateCRCCircularBuffer(uint8_t *buffer, uint16_t buffer_size, uint8_t start_index, uint8_t length) {
     uint8_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        uint8_t index = (start_index + i) % buffer_size;
        sum += buffer[index];
    }
    uint8_t lsb = sum & 0xFF;
    return (uint8_t)(~lsb + 1);
}

void UART_Transmit(UartTxStr_t *TxStr) { //*ptr to struct
	uint8_t size = TxStr->len;
	if (size > TX_BUFFER_SIZE) {
		size = TX_BUFFER_SIZE; // handle error
	}
	LL_DMA_DisableChannel(DMA_LPUART_TX);
	LL_DMA_SetDataLength(DMA_LPUART_TX, size + 5U); //1U for CRC additional byte
	//len, addr, cmd
	memcpy(usb_tx_buffer, TxStr, size + 4U);
	usb_tx_buffer[3] += 0x10U;
	uint8_t crc = CalculateCRC(usb_tx_buffer, size + 4U);
	usb_tx_buffer[size + 4U] = crc;
	LL_DMA_EnableChannel(DMA_LPUART_TX);
}

static void complete_command_processing(void) {
	UART_State = UART_STATE_IDLE;
	queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
	queue_cnt--;
}

static void handle_write_bank_command(uint8_t cmd_data_len, uint16_t cmd_addr, uint8_t* cmd_data) {
	if (IsBiSSReqBusy() != BISS_BUSY) {
		if (cmd_data_len == WRITE_BANK_DATA_LEN) {
			cmd_data_len += 1;
			cmd_data[cmd_data_len] = ((cmd_addr %(0x00A0U & 0xFFU)) % 0x20U) + 6;
		}
		if (BiSSRequestWrite(cmd_addr, cmd_data_len, cmd_data) == BISS_REQ_OK) {
			complete_command_processing();
			retry_cnt = 0;
		} else {
			retry_cnt++;
			if (retry_cnt >= MAX_RETRY) {
				if (BiSSGetFaultState() == BISS_NO_FAULTS) {
					UART_State = UART_STATE_IDLE;
					retry_cnt = 0;
				} else {
					UART_Error = UART_ERROR_BISS;
					UART_State = UART_STATE_ABORT;
					//queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
					//queue_cnt--;
					retry_cnt = 0;
				}
			}
		}
	}
}

static void handle_change_mode_command(uint8_t mode) {
	switch (mode) {
		case 0:
			if (Current_Mode != BISS_MODE_SPI_SPI){
					Change_Current_Mode(BISS_MODE_SPI_SPI);
			}
			break;
		case 1:
			if (Current_Mode != BISS_MODE_AB_UART){
					Change_Current_Mode(BISS_MODE_AB_UART);
			}
			break;
		case 2:
			if (Current_Mode != BISS_MODE_SPI_UART_IRS){
					Change_Current_Mode(BISS_MODE_SPI_UART_IRS);
			}
			break;
		case 3:
			if (Current_Mode != BISS_MODE_AB_SPI){
					Change_Current_Mode(BISS_MODE_AB_SPI);
			}
			break;
		case 4:
			if (Current_Mode != BISS_MODE_DEFAULT_SPI){
					Change_Current_Mode(BISS_MODE_DEFAULT_SPI);
			}
			break;
		default:
			if (Current_Mode != BISS_MODE_DEFAULT_SPI){
					Change_Current_Mode(BISS_MODE_DEFAULT_SPI);
			}
			break;
	}
}

static void handle_page_command(uint8_t data_to_page) {
	if (IsBiSSReqBusy() != BISS_BUSY) {
		uint8_t cmd_data_page = data_to_page;
		if (BiSSRequestWrite(PAGE_ADDR, 1U, &cmd_data_page) == BISS_REQ_OK) {
			complete_command_processing();
			retry_cnt = 0;
//											uint8_t add_data = 0x05;
			if (EnqueueCommandToBegining(UART_COMMAND_WRITE_REG, BSEL_ADDR, 1U, (uint8_t *)FIRST_USER_BANK) != QUEUE_OK){
				UART_Error = UART_ERROR_QUEUE_FULL;
				UART_State = UART_STATE_ABORT; 
			}
		} else {
			retry_cnt++;
			if (retry_cnt >= MAX_RETRY) {
				if (BiSSGetFaultState() == BISS_NO_FAULTS) {
					UART_State = UART_STATE_IDLE;
					retry_cnt = 0;
				} else {
					UART_Error = UART_ERROR_BISS;
					UART_State = UART_STATE_ABORT;
					//queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
					//queue_cnt--;
					retry_cnt = 0;
				}
			}
		}
	}
}

static void handle_select_spi_ch_command(uint8_t channel) {
	switch (channel) {
		case 0:
			if (BiSS_SPI_Ch != BISS_SPI_CH_1){
				SetBiSS_SPI_Ch(BISS_SPI_CH_1);
			}
			break;
		case 1:
			if (BiSS_SPI_Ch != BISS_SPI_CH_2){
				SetBiSS_SPI_Ch(BISS_SPI_CH_2);
			}
			break;
		default:
			if (BiSS_SPI_Ch != BISS_SPI_CH_1){
				SetBiSS_SPI_Ch(BISS_SPI_CH_1);
			}
			break;
	}
}

static void handle_change_ch1_mode_command(uint8_t ch1_mode) {
	switch (ch1_mode) {
		case 0:
			if (CH1_SPI_MODE != CH1_LENZ_BISS) {
				Set_Ch1_Mode(CH1_LENZ_BISS);
			}
			break;
		case 1:
			if (CH1_SPI_MODE != CH1_LIR_SSI){
				Set_Ch1_Mode(CH1_LIR_SSI);
			}
			break;
		case 2:
			if (CH1_SPI_MODE != CH1_LIR_BISS_21B){
				Set_Ch1_Mode(CH1_LIR_BISS_21B);
			}
			break;
		default:
			if (CH1_SPI_MODE != CH1_LENZ_BISS) {
				Set_Ch1_Mode(CH1_LENZ_BISS);
			}
			break;
	}
}

static void handle_write_reg_command(uint8_t cmd_data_len, uint16_t cmd_addr, uint8_t* cmd_data) {
	if (IsBiSSReqBusy() != BISS_BUSY) {
		if (BiSSRequestWrite(cmd_addr, cmd_data_len, cmd_data) == BISS_REQ_OK) {
			complete_command_processing();
			retry_cnt = 0;
			} else {
				retry_cnt++;
				if (retry_cnt >= MAX_RETRY) {
					UART_Error = UART_ERROR_BISS_WRITE_FAULT;
					UART_State = UART_STATE_ABORT;
					retry_cnt = 0;
//					BiSSResetExternalState();
//					queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
//					queue_cnt--;
				}
			}
		}
				
//											if (retry_cnt >= MAX_RETRY) {
//												if(BiSSGetFaultState() == BISS_NO_FAULTS) {
//													UART_State = UART_STATE_IDLE;
//													retry_cnt = 0;
//												} else {
//													UART_Error = UART_ERROR_BISS;
//													UART_State = UART_STATE_ABORT;
//													queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
//													queue_cnt--;
//													retry_cnt = 0;
//												}
//											}
					
//									} else {
//										if(BiSSGetFaultState() != BISS_NO_FAULTS) {
//											UART_Error = UART_ERROR_BISS;
//											UART_State = UART_STATE_ABORT;
//											queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
//											queue_cnt--;
//											retry_cnt = 0;
//										}
//									}
}

static void handle_read_reg_command(uint8_t cmd_data_len, uint16_t cmd_addr, UART_Command_t command) {
	if (IsBiSSReqBusy() != BISS_BUSY) { 
		UART_TX.cmd = command;
		UART_TX.len = cmd_data_len;
		UART_TX.adr_h = (cmd_addr >> 8U) & 0xFFU;
		UART_TX.adr_l = cmd_addr & 0xFFU;
		
		if (BiSSRequestRead(cmd_addr, cmd_data_len, UART_TX.Buf) == BISS_REQ_OK) {
			complete_command_processing();
			retry_cnt = 0;
		} else {
			retry_cnt++;
			if (retry_cnt >= MAX_RETRY) {
				UART_Error = UART_ERROR_BISS_READ_FAULT;
				UART_State = UART_STATE_ABORT;
				retry_cnt = 0;
//				BiSSResetExternalState();
//				queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
//				queue_cnt--;
			}
		}
	}
					
//											} else {
//												retry_cnt++;
//												if (retry_cnt >= MAX_RETRY) {
//													if(BiSSGetFaultState() == BISS_NO_FAULTS) {
//														UART_State = UART_STATE_IDLE;
//														retry_cnt = 0;
//													} else {
//														UART_Error = UART_ERROR_BISS;
//														UART_State = UART_STATE_ABORT;
//														queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
//														queue_cnt--;
//														retry_cnt = 0;
//													}
//												}
//												// UART_State = UART_STATE_ABORT;
//											}
//										} else {
//											if(BiSSGetFaultState() != BISS_NO_FAULTS) {
//												UART_Error = UART_ERROR_BISS;
//												UART_State = UART_STATE_ABORT;
//												queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
//												queue_cnt--;
//												retry_cnt = 0;
//											}
//										}
}

static void handle_write_read_enc_usart2_command(uint8_t cmd_data_len, uint8_t* cmd_data, UART_Command_t command) {
	UART_TX.cmd = command;
	UART_TX.len = cmd_data_len;
	UART_TX.adr_h = 0;
	UART_TX.adr_l = 0;
	USART2_Write_Read_IRS(cmd_data, UART_TX.Buf, cmd_data_len);
	UART_Transmit(&UART_TX);
}

static void handle_read_angle_enc_spi_instant_command(uint8_t cmd_data_len, UART_Command_t command) {
	UART_TX.cmd = command;
	UART_TX.len = cmd_data_len;
	UART_TX.adr_h = 0;
	UART_TX.adr_l = 0;
	if(BiSS_SPI_Ch == BISS_SPI_CH_2){
		AngleData_t angle_data2 = getAngle2();
		*((AngleData_t*)&UART_TX.Buf[0]) = angle_data2;
		UART_Transmit(&UART_TX);
	} else if(BiSS_SPI_Ch == BISS_SPI_CH_1){
		AngleData_t angle_data1 = getAngle1();
		*((AngleData_t*)&UART_TX.Buf[0]) = angle_data1;
		UART_Transmit(&UART_TX);
	}
}

static void handle_change_current_sensor_mode_command(uint8_t current_mode) {
	switch (current_mode) {
		case 0:
			if (Current_Sensor_Mode != CURRENT_SENSOR_MODE_DISABLE){
				Change_Current_Sensor_Mode(CURRENT_SENSOR_MODE_DISABLE);
			}
			break;
		case 1:
			if (Current_Sensor_Mode != CURRENT_SENSOR_MODE_ENABLE){
				Change_Current_Sensor_Mode(CURRENT_SENSOR_MODE_ENABLE);
			}
			break;
		default:
			if (Current_Sensor_Mode != CURRENT_SENSOR_MODE_ENABLE){
				Change_Current_Sensor_Mode(CURRENT_SENSOR_MODE_ENABLE);
			}
			break;
	}
}

static void handle_read_enc2_current_command(uint8_t cmd_data_len, UART_Command_t command) {
	UART_TX.cmd = command;
	UART_TX.len = cmd_data_len;
	UART_TX.adr_h = 0;
	UART_TX.adr_l = 0;
	if (Current_Sensor_Mode == CURRENT_SENSOR_MODE_ENABLE){
		int32_t current = Read_Current_Enc2();
		memcpy(&UART_TX.Buf[0], &current, sizeof(current));
		UART_Transmit(&UART_TX);
	}
}

static void prepare_encoder_ab_spi_reading(uint16_t cmd_addr, UART_Command_t command) {
	UART_TX.cmd = command;
	UART_TX.len = UART_ANGLE_TWO_ENC_AB_UART_BUF_SIZE;
	ReadingStrRenishaw.len = cmd_addr + 1;// Address = buf_size * 63
	ReadingStrRenishaw.FIFO_current_ptr = 0;
	ReadingStrRenishaw.FIFO_start_ptr = 0;
	ReadingStrEnc2.len = cmd_addr + 1; // Address = buf_size * 63
	ReadingStrEnc2.FIFO_current_ptr = 0;
	ReadingStrEnc2.FIFO_start_ptr = 0;
	ReadingStrEnc2.ToL_cnt = 0;
	queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
	queue_cnt--;
}

static void prepare_encoder_ab_uart_reading(uint16_t cmd_addr, UART_Command_t command) {
	UART_TX.cmd = command;
	UART_TX.len = UART_ANGLE_TWO_ENC_AB_UART_BUF_SIZE;
	ReadingStrRenishaw.len = cmd_addr + 1;// Address = buf_size * 63
	ReadingStrRenishaw.FIFO_current_ptr = 0;
	ReadingStrRenishaw.FIFO_start_ptr = 0;
	ReadingStrEnc2.len = cmd_addr + 1;// Address = buf_size * 63
	ReadingStrEnc2.FIFO_current_ptr = 0;
	ReadingStrEnc2.FIFO_start_ptr = 0;
	ReadingStrEnc2.ToL_cnt = 0;
	queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
	queue_cnt--;
}

static void prepare_encoder_spi_spi_reading(uint16_t cmd_addr, UART_Command_t command) {
	UART_TX.cmd = command;
	UART_TX.len = UART_ANGLE_TWO_ENC_BUF_SIZE;
	ReadingStrEnc1.len = cmd_addr + 1;// Address = buf_size * 63
	ReadingStrEnc1.FIFO_current_ptr = 0;
	ReadingStrEnc1.FIFO_start_ptr = 0;
	ReadingStrEnc1.ToL_cnt = 0;
	ReadingStrEnc2.len = cmd_addr + 1;// Address = buf_size * 63
	ReadingStrEnc2.FIFO_current_ptr = 0;
	ReadingStrEnc2.FIFO_start_ptr = 0;
	ReadingStrEnc2.ToL_cnt = 0;
	queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
	queue_cnt--;
}

static void prepare_encoder_spi_reading(uint16_t cmd_addr, UART_Command_t command) {
	UART_TX.cmd = command;
	UART_TX.len = UART_ANGLE_BUF_SIZE;
	if(BiSS_SPI_Ch == BISS_SPI_CH_1) {
		ReadingStrEnc1.len = cmd_addr + 1;// Address = buf_size * 63
		ReadingStrEnc1.FIFO_current_ptr = 0;
		ReadingStrEnc1.FIFO_start_ptr = 0;
		ReadingStrEnc1.ToL_cnt = 0;
	} 
	else if(BiSS_SPI_Ch == BISS_SPI_CH_2) {
		ReadingStrEnc2.len = cmd_addr + 1;// Address = buf_size * 63
		ReadingStrEnc2.FIFO_current_ptr = 0;
		ReadingStrEnc2.FIFO_start_ptr = 0;
		ReadingStrEnc2.ToL_cnt = 0;
	}
	queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
	queue_cnt--;
}

static void handle_idle_state(void) {
//	if (IsBiSSReqBusy() != BISS_READ_FINISHED){
//		UART_Transmit(read_buf, RX_BUFFER_SIZE);
//	}
	uint8_t bytes_received;
	new_cnt = RX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA_LPUART_RX);
	if (dma_rx_cnt != new_cnt) {
			bytes_received = (new_cnt - dma_rx_cnt + RX_BUFFER_SIZE) % RX_BUFFER_SIZE;
			uart_length = usb_rx_buffer[dma_rx_cnt];
			uart_expected_length = uart_length + HEXLEN_ADR_CMD_CRC_LEN;
			if (bytes_received >= uart_expected_length) {
				UART_State = UART_STATE_RECEIVE;
			}
	} else {
		if (queue_cnt > 0){
			UART_State = UART_STATE_RUNCMD;
		}
	}
}

static void handle_receive_state(uint8_t crc, uint8_t calculated_crc) {
	memset(hex_line_buffer, 0, UART_LINE_SIZE);
	if (uart_length > 0) {		
		uint8_t crc_position = (dma_rx_cnt + uart_expected_length - 1U) % RX_BUFFER_SIZE;
		crc = usb_rx_buffer[crc_position];
		calculated_crc = CalculateCRCCircularBuffer((uint8_t *)usb_rx_buffer, RX_BUFFER_SIZE, dma_rx_cnt, uart_expected_length - 1U);
	
		if (crc == calculated_crc) {
			if (dma_rx_cnt + uart_expected_length <= RX_BUFFER_SIZE) {
					memcpy(hex_line_buffer, (uint8_t *)&usb_rx_buffer[dma_rx_cnt], uart_expected_length);
			} else {
					uint32_t part_size = RX_BUFFER_SIZE - dma_rx_cnt;
					memcpy(hex_line_buffer, (uint8_t *)&usb_rx_buffer[dma_rx_cnt], part_size);
					memcpy(hex_line_buffer + part_size, (uint8_t *)usb_rx_buffer, uart_expected_length - part_size);
			}

			uint8_t cmd_data_len = hex_line_buffer[0];
			uint16_t cmd_addr = (hex_line_buffer[1] << 8) | hex_line_buffer[2];
			UART_Command_t command = hex_line_buffer[3];
			uint8_t *cmd_data = &hex_line_buffer[4];	
					
			if (EnqueueCommand(command, cmd_addr, cmd_data_len, cmd_data) == QUEUE_OK) {
				UART_State = UART_STATE_RUNCMD;
			} else {
				UART_Error = UART_ERROR_QUEUE_FULL;
				UART_State = UART_STATE_ABORT;  
			}
		} else {
				UART_Error = UART_ERROR_CRC;
				UART_State = UART_STATE_CHECKCRC;
		}
	} else {
		UART_Error = UART_ERROR_LEN_DATA_IS_ZERO;
		UART_State = UART_STATE_ABORT;
	}
	dma_rx_cnt = (dma_rx_cnt + uart_expected_length) % RX_BUFFER_SIZE;
}

static void handle_run_command_state(void) {
	if (queue_cnt > 0) {
		uint8_t cmd_data_len = CommandQueue[queue_read_cnt].len;
		uint16_t cmd_addr = CommandQueue[queue_read_cnt].addr;
		UART_Command_t command = CommandQueue[queue_read_cnt].cmd;
		uint8_t *cmd_data = CommandQueue[queue_read_cnt].data;

		switch (command) {
			// add command 00 
			//
			// cmd 04 -> set page and set bank 5
			//								A0 
			// A0 -> 5 (1010 0000 > 0000 0101)
			// A1 -> 6 (1010 0000 > 0000 0110)
			//
			// cmd 03 -> send crc and run command load2k
			case UART_COMMAND_WRITE_BANK:
				handle_write_bank_command(cmd_data_len, cmd_addr, cmd_data);
				break;
				
			case UART_COMMAND_CHANGE_MODE:
				if(cmd_data_len > 1) {
					UART_State = UART_STATE_ABORT;
					UART_Error = UART_ERROR_LEN_DATA_IS_ZERO;
					queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
					queue_cnt--;
				} else {
					handle_change_mode_command(cmd_data[cmd_data_len-1]);
					complete_command_processing();
				}
				break;
												
			case UART_COMMAND_PAGE:
				if(cmd_data_len > 1) {
					UART_State = UART_STATE_ABORT;
					UART_Error = UART_ERROR_LEN_DATA_IS_ZERO;
					queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
					queue_cnt--;
				} else {
					handle_page_command(cmd_data[cmd_data_len-1]);
				}
				break;
				
			case UART_COMMAND_ENC1_POWER_OFF:
				EncoderPowerDisable();
				complete_command_processing();	
				break;
			
			case UART_COMMAND_ENC1_POWER_ON:
				if(Current_Mode != BISS_MODE_DEFAULT_SPI){
					EncoderPowerEnable();
				}
				complete_command_processing();
				break;	
			
			case UART_COMMAND_ENC2_POWER_OFF:
				EncoderSecondPowerDisable();
				complete_command_processing();	
				break;
			
			case UART_COMMAND_ENC2_POWER_ON:
				EncoderSecondPowerEnable();
				complete_command_processing();
				break;
						
			case UART_COMMAND_SELECT_SPI_CH:
				if(cmd_data_len > 1) {
					UART_State = UART_STATE_ABORT;
					UART_Error = UART_ERROR_LEN_DATA_IS_ZERO;
					queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
					queue_cnt--;
				} else {
					handle_select_spi_ch_command(cmd_data[cmd_data_len-1]);
					complete_command_processing();
				}
				break;
					
			case UART_COMMAND_CHANGE_CH1_MODE:
				if(cmd_data_len > 1) {
					UART_State = UART_STATE_ABORT;
					UART_Error = UART_ERROR_LEN_DATA_IS_ZERO;
					queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
					queue_cnt--;
				} else {
					handle_change_ch1_mode_command(cmd_data[cmd_data_len-1]);			
					complete_command_processing();
				}
				break;
				
			case UART_COMMAND_WRITE_REG:
				handle_write_reg_command(cmd_data_len, cmd_addr, cmd_data);
				break;

			case UART_COMMAND_READ_REG:
				handle_read_reg_command(cmd_data_len, cmd_addr, command);
				break;
								
			case UART_COMMAND_WRITE_READ_ENC_USART2:
				handle_write_read_enc_usart2_command(cmd_data_len, cmd_data, command);
				complete_command_processing();
				break;
			
			case UART_COMMAND_READ_ANGLE_ENC_SPI_INSTANT:
				handle_read_angle_enc_spi_instant_command(cmd_data_len, command);
				complete_command_processing();
				break;

			case UART_COMMAND_CHANGE_CURRENT_SENSOR_MODE:
				if(cmd_data_len > 1) {
					UART_State = UART_STATE_ABORT;
					UART_Error = UART_ERROR_LEN_DATA_IS_ZERO;
					queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
					queue_cnt--;
				} else {
					handle_change_current_sensor_mode_command(cmd_data[cmd_data_len-1]);
					complete_command_processing();
				}
				break;
			
			case UART_COMMAND_READ_ENC2_CURRENT:
				handle_read_enc2_current_command(cmd_data_len, command);
				complete_command_processing();
				break;
					
			case UART_COMMAND_READ_ANGLE_TWO_ENC_AB_SPI:
				prepare_encoder_ab_spi_reading(cmd_addr, command);
				UART_State = UART_STATE_ANGLE_READING_TWO_ENC_AB_SPI;
				break;
			
			case UART_COMMAND_READ_ANGLE_TWO_ENC_AB_UART:
				prepare_encoder_ab_uart_reading(cmd_addr, command);
				UART_State = UART_STATE_ANGLE_READING_TWO_ENC_AB_UART;
				break;
				
			case UART_COMMAND_READ_ANGLE_TWO_ENC_SPI:
				prepare_encoder_spi_spi_reading(cmd_addr, command);				
				UART_State = UART_STATE_ANGLE_READING_TWO_ENC_SPI;
				break;
			
			case UART_COMMAND_READ_ANGLE_ENC_SPI:
				prepare_encoder_spi_reading(cmd_addr, command);
				UART_State = UART_STATE_ANGLE_READING_ENC_SPI;
				break;

			case UART_COMMAND_NRST:		
				NVIC_SystemReset();
				break;
			
			case UART_COMMAND_REBOOT_TO_BL:
				JumpToBootloader();
				break;
						
			default:
				UART_Error = UART_ERROR_INVALID_CMD;
				UART_State = UART_STATE_ABORT;
				break;
		}
	}
}

static void handle_reading_encoder_ab_spi_state(void) {
	if(ReadingStrEnc2.len > 0) {
		AngleDataRenishaw_t angle_data1 = getAngleRenishaw();
		AngleData_t angle_data2 = getAngle2();
		
		if(angle_data2.time_of_life_counter != ReadingStrEnc2.ToL_cnt) {
			
			ReadingStrRenishaw.AngleFIFO[ReadingStrRenishaw.FIFO_current_ptr] = angle_data1;
			ReadingStrRenishaw.FIFO_current_ptr++;
			
			ReadingStrEnc2.ToL_cnt = angle_data2.time_of_life_counter;
			ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.FIFO_current_ptr] = angle_data2;
			ReadingStrEnc2.FIFO_current_ptr++;
			
			if((((uint16_t)ReadingStrEnc2.FIFO_current_ptr + 256 - ReadingStrEnc2.FIFO_start_ptr) & 0xFFU) >= UART_ANGLE_TWO_ENC_AB_UART_LEN) {
				uint8_t TxBufCnt = 0;
				while(ReadingStrEnc2.FIFO_start_ptr != ReadingStrEnc2.FIFO_current_ptr) {
					*((AngleData_t*)&UART_TX.Buf[TxBufCnt]) = ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.FIFO_start_ptr];
					TxBufCnt += ANGLE_DATA_SIZE;
					*((AngleDataRenishaw_t*)&UART_TX.Buf[TxBufCnt]) = ReadingStrRenishaw.AngleFIFO[ReadingStrRenishaw.FIFO_start_ptr];
					TxBufCnt += RENISHAW_ANGLE_DATA_SIZE;
					ReadingStrEnc2.FIFO_start_ptr++; 
					ReadingStrRenishaw.FIFO_start_ptr++; 
				}
				ReadingStrRenishaw.len--;
				ReadingStrEnc2.len--;
				UART_TX.adr_h = (ReadingStrEnc2.len >> 8U) & 0xFFU;
				UART_TX.adr_l = ReadingStrEnc2.len & 0xFFU;
				UART_Transmit(&UART_TX);
			}
		}
	} else {
		UART_State = UART_STATE_IDLE;
	}
}

static void handle_reading_encoder_spi_spi_state(void) {
	if(ReadingStrEnc2.len > 0) {
		AngleData_t angle_data1 = getAngle1();
		AngleData_t angle_data2 = getAngle2();

		if((angle_data2.time_of_life_counter != ReadingStrEnc2.ToL_cnt) || (angle_data1.time_of_life_counter != ReadingStrEnc1.ToL_cnt)) {
			
			ReadingStrEnc1.ToL_cnt = angle_data1.time_of_life_counter;
			ReadingStrEnc1.AngleFIFO[ReadingStrEnc1.FIFO_current_ptr] = angle_data1;
			ReadingStrEnc1.FIFO_current_ptr++;
			
			ReadingStrEnc2.ToL_cnt = angle_data2.time_of_life_counter;
			ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.FIFO_current_ptr] = angle_data2;
			ReadingStrEnc2.FIFO_current_ptr++;
			
			if((((uint16_t)ReadingStrEnc2.FIFO_current_ptr + 256 - ReadingStrEnc2.FIFO_start_ptr) & 0xFFU) >= UART_ANGLE_TWO_ENC_LEN) {
				uint8_t TxBufCnt = 0;
				while(ReadingStrEnc2.FIFO_start_ptr != ReadingStrEnc2.FIFO_current_ptr) {
					*((AngleData_t*)&UART_TX.Buf[TxBufCnt]) = ReadingStrEnc1.AngleFIFO[ReadingStrEnc1.FIFO_start_ptr];
					TxBufCnt += ANGLE_DATA_SIZE;
					*((AngleData_t*)&UART_TX.Buf[TxBufCnt]) = ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.FIFO_start_ptr];
					TxBufCnt += ANGLE_DATA_SIZE;
					ReadingStrEnc1.FIFO_start_ptr++; 
					ReadingStrEnc2.FIFO_start_ptr++; 
				}
				ReadingStrEnc1.len--; 
				ReadingStrEnc2.len--; 
				UART_TX.adr_h = (ReadingStrEnc2.len >> 8U) & 0xFFU;
				UART_TX.adr_l = ReadingStrEnc2.len & 0xFFU;
				UART_Transmit(&UART_TX);
			}
		} else {
			UART_State = UART_STATE_IDLE;
		}			
	}	else {
		UART_State = UART_STATE_IDLE;
	}
}

static void handle_reading_encoder_ab_uart_state(void) {
	if(ReadingStrEnc2.len > 0) {
		AngleData_t angle_data2 = getAngle2();
		AngleDataRenishaw_t angle_data1 = getAngleRenishaw();
		
		if(angle_data2.time_of_life_counter != ReadingStrEnc2.ToL_cnt) {

			ReadingStrEnc2.ToL_cnt = angle_data2.time_of_life_counter;
			ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.FIFO_current_ptr] = angle_data2;
			ReadingStrEnc2.FIFO_current_ptr++;
			
			ReadingStrRenishaw.AngleFIFO[ReadingStrRenishaw.FIFO_current_ptr] = angle_data1;
			ReadingStrRenishaw.FIFO_current_ptr++;
			
			if((((uint16_t)ReadingStrEnc2.FIFO_current_ptr + 256 - ReadingStrEnc2.FIFO_start_ptr) & 0xFFU) >= UART_ANGLE_TWO_ENC_AB_UART_LEN) {
				uint8_t TxBufCnt = 0;
				while(ReadingStrEnc2.FIFO_start_ptr != ReadingStrEnc2.FIFO_current_ptr) {
					*((AngleData_t*)&UART_TX.Buf[TxBufCnt]) = ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.FIFO_start_ptr];
					TxBufCnt += ANGLE_DATA_SIZE;
					*((AngleDataRenishaw_t*)&UART_TX.Buf[TxBufCnt]) = ReadingStrRenishaw.AngleFIFO[ReadingStrRenishaw.FIFO_start_ptr];
					TxBufCnt += RENISHAW_ANGLE_DATA_SIZE;
					ReadingStrEnc2.FIFO_start_ptr++; 
					ReadingStrRenishaw.FIFO_start_ptr++; 
				}
				ReadingStrEnc2.len--; 
				ReadingStrRenishaw.len--; 
				UART_TX.adr_h = (ReadingStrEnc2.len >> 8U) & 0xFFU;
				UART_TX.adr_l = ReadingStrEnc2.len & 0xFFU;
				UART_Transmit(&UART_TX);
			}
		}
	} else {
		UART_State = UART_STATE_IDLE;
	}
}

static void handle_reading_encoder_spi_state(void) {
	if(BiSS_SPI_Ch == BISS_SPI_CH_1) {
		if(ReadingStrEnc1.len > 0) {    
			AngleData_t angle_data = getAngle1();
			if(angle_data.time_of_life_counter != ReadingStrEnc1.ToL_cnt) {
				ReadingStrEnc1.ToL_cnt = angle_data.time_of_life_counter;
				ReadingStrEnc1.AngleFIFO[ReadingStrEnc1.FIFO_current_ptr] = angle_data;
				ReadingStrEnc1.FIFO_current_ptr++;
				if((((uint16_t)ReadingStrEnc1.FIFO_current_ptr + 256 - ReadingStrEnc1.FIFO_start_ptr) & 0xFFU) >= UART_ANGLE_LEN) {
					uint8_t TxBufCnt = 0;
					while(ReadingStrEnc1.FIFO_start_ptr != ReadingStrEnc1.FIFO_current_ptr) {
							*((AngleData_t*)&UART_TX.Buf[TxBufCnt]) = ReadingStrEnc1.AngleFIFO[ReadingStrEnc1.FIFO_start_ptr];
							TxBufCnt += ANGLE_DATA_SIZE;
							ReadingStrEnc1.FIFO_start_ptr++;
					}
					ReadingStrEnc1.len--;
					UART_TX.adr_h = (ReadingStrEnc1.len >> 8U) & 0xFFU;
					UART_TX.adr_l = ReadingStrEnc1.len & 0xFFU;
					UART_Transmit(&UART_TX);
				}
			}
		} else {
				UART_State = UART_STATE_IDLE;
		}
	}
	else if(BiSS_SPI_Ch == BISS_SPI_CH_2) {
		if(ReadingStrEnc2.len > 0) {    
			AngleData_t angle_data = getAngle2();
			if(angle_data.time_of_life_counter != ReadingStrEnc2.ToL_cnt) {
				ReadingStrEnc2.ToL_cnt = angle_data.time_of_life_counter;
				ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.FIFO_current_ptr] = angle_data;
				ReadingStrEnc2.FIFO_current_ptr++;
				if((((uint16_t)ReadingStrEnc2.FIFO_current_ptr + 256 - ReadingStrEnc2.FIFO_start_ptr) & 0xFFU) >= UART_ANGLE_LEN) {
					uint8_t TxBufCnt = 0;
					while(ReadingStrEnc2.FIFO_start_ptr != ReadingStrEnc2.FIFO_current_ptr) {
							*((AngleData_t*)&UART_TX.Buf[TxBufCnt]) = ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.FIFO_start_ptr];
							TxBufCnt += 4;
							ReadingStrEnc2.FIFO_start_ptr++;
					}
					ReadingStrEnc2.len--;
					UART_TX.adr_h = (ReadingStrEnc2.len >> 8U) & 0xFFU;
					UART_TX.adr_l = ReadingStrEnc2.len & 0xFFU;
					UART_Transmit(&UART_TX);
				}
			}
		} else {
				UART_State = UART_STATE_IDLE;
		}
	}
}

static void handle_abort_state(void) {
	switch(UART_Error) {
		case UART_ERROR_CRC:
			UART_Error_Type = ERROR_TYPE_UART;
			UART_TX.cmd = (uint8_t)UART_Error_Type;
			UART_TX.len = 1;
			UART_TX.adr_h = 0;
			UART_TX.adr_l = 0;
//			UART_TX.Buf[0] = (uint8_t)UART_Error_Type;
			UART_TX.Buf[0] = (uint8_t)UART_Error;
			UART_Transmit(&UART_TX);
			break;
				
		case UART_ERROR_QUEUE_FULL:
			UART_Error_Type = ERROR_TYPE_UART;
			UART_TX.cmd = (uint8_t)UART_Error_Type;
			UART_TX.len = 1;
			UART_TX.adr_h = 0;
			UART_TX.adr_l = 0;
//			UART_TX.Buf[0] = (uint8_t)UART_Error_Type;
			UART_TX.Buf[0] = (uint8_t)UART_Error;
			UART_Transmit(&UART_TX);
			break;
				
		case UART_ERROR_BISS:
			UART_Error_Type = ERROR_TYPE_BISS;
			UART_TX.cmd = (uint8_t)UART_Error_Type;
			UART_TX.len = 1;
			UART_TX.adr_h = 0;
			UART_TX.adr_l = 0;
//			UART_TX.Buf[0] = (uint8_t)UART_Error_Type;
			UART_TX.Buf[0] = (uint8_t)BiSSGetFaultState();
			UART_Transmit(&UART_TX);
			break;
				
		case UART_ERROR_LEN_DATA_IS_ZERO:
			UART_Error_Type = ERROR_TYPE_UART;
			UART_TX.cmd = (uint8_t)UART_Error_Type;
			UART_TX.len = 1;
			UART_TX.adr_h = 0;
			UART_TX.adr_l = 0;
//			UART_TX.Buf[0] = (uint8_t)UART_Error_Type;
			UART_TX.Buf[0] = (uint8_t)UART_Error;
			UART_Transmit(&UART_TX);
			break;
		
		case UART_ERROR_LEN_IS_NOT_CORRECT:
			UART_Error_Type = ERROR_TYPE_UART;
			UART_TX.cmd = (uint8_t)UART_Error_Type;
			UART_TX.len = 1;
			UART_TX.adr_h = 0;
			UART_TX.adr_l = 0;
			UART_TX.Buf[0] = (uint8_t)UART_Error;
			UART_Transmit(&UART_TX);
			break;
		
		case UART_ERROR_INVALID_CMD:
			queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
			queue_cnt--;
			break;
		
		default:
			__NOP();
			__NOP();
			__NOP();
			break;
	}
	
//	UART_Error = UART_ERROR_NONE;
	UART_State = UART_STATE_IDLE;
}

void UART_StateMachine(void) {
    uint8_t crc;
		uint8_t calculated_crc;
//    uint32_t new_cnt;

//		if(IsBiSSReqBusy() == BISS_FAULT) {
//			cnt_error_cycles++;
//			if (cnt_error_cycles == BISS_ABORT_CNT_CYCLES) {
//				cnt_error_cycles = 0;
//				UART_Error = UART_ERROR_BISS;
//				UART_State = UART_STATE_ABORT;
//				BiSSResetExternalState();
//			}
//		}
	
		if(IsBiSSReqBusy() ==	BISS_READ_FINISHED) {
				UART_Transmit(&UART_TX);
				BiSSResetExternalState();
		}
		
    switch (UART_State) {
        case UART_STATE_IDLE:
					handle_idle_state();
					break;

        case UART_STATE_RECEIVE:
					handle_receive_state(crc, calculated_crc);
					break;

        case UART_STATE_CHECKCRC: // TODO ???
					UART_Error = UART_ERROR_CRC;
					UART_State = UART_STATE_ABORT;  // TODO  handle CRC error
					break;

        case UART_STATE_RUNCMD:
					handle_run_command_state();
					break;
						
				case UART_STATE_ANGLE_READING_TWO_ENC_AB_SPI:
					handle_reading_encoder_ab_spi_state();
					break;
						
				case UART_STATE_ANGLE_READING_TWO_ENC_SPI:
					handle_reading_encoder_spi_spi_state();
					break;
				
				case UART_STATE_ANGLE_READING_TWO_ENC_AB_UART:
					handle_reading_encoder_ab_uart_state();
					break;
				
				case UART_STATE_ANGLE_READING_ENC_SPI:
					handle_reading_encoder_spi_state();
					break;
					
        case UART_STATE_ABORT:
					handle_abort_state();
					break;

        default:
					UART_State = UART_STATE_IDLE;
          break;
    }
}