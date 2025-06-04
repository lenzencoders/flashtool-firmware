/*!
 * @file biss_c_master.c
 * @author Kirill Rostovskiy (kmrost@lenzencoders.com)
 * @brief UART library
 * @version 0.1
 * @copyright Lenz Encoders (c) 2024
 */
#include "uart.h"
 
#include "stm32g4xx_ll_lpuart.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_gpio.h"
#include "string.h"
#include "biss_c_master.h"
#include "biss_c_master_hal.h"

#define RX_BUFFER_SIZE 		256U
//#define UART_LINE_SIZE		133U
#define HEXLEN_ADR_CMD_CRC_LEN	5U // Length of data (1) + Address (2) + Cmd (1) + CRC (1) bytes
#define HEX_DATA_LEN	128U
#define UART_LINE_SIZE		HEXLEN_ADR_CMD_CRC_LEN + HEX_DATA_LEN
#define QUEUE_SIZE 	36U //FIFO
#define MAX_RETRY		3U
#define UART_ANGLE_LEN 	60U // 60U --> Encoder
#define UART_ANGLE_BUF_SIZE 	(UART_ANGLE_LEN * 4U) // *4U --> Encoder
#define UART_ANGLE_TWO_ENC_LEN 	30U // 30U --> Encoder1 + Encoder2
#define UART_ANGLE_TWO_ENC_BUF_SIZE 	(UART_ANGLE_TWO_ENC_LEN * 8U) // *(4U + 4U) --> Encoder1 + Encoder2
#define UART_ANGLE_TWO_ENC_AB_UART_LEN 	40U // 40U --> Encoder1 + Renishaw
#define UART_ANGLE_TWO_ENC_AB_UART_BUF_SIZE 	(UART_ANGLE_TWO_ENC_AB_UART_LEN * 6U) // *(4U + 2U) --> Encoder1 + Renishaw
#define ANGLE_SIZE	4U

#define PAGE_ADDR       0x18U
#define BSEL_ADDR		    0x40U
#define FIRST_USER_BANK 0x05U

#define BISS_ABORT_CNT_CYCLES 				14U		/* Cycles to abort control data frame */

const uint16_t error_biss_cmd = 0xDE;
const uint16_t error_uart_cmd = 0xEF;

static void EncoderPowerEnable(void){
	LL_GPIO_SetOutputPin(PWR1_EN_PIN.port, PWR1_EN_PIN.pin);
}

static void EncoderPowerDisable(void){
	LL_GPIO_ResetOutputPin(PWR1_EN_PIN.port, PWR1_EN_PIN.pin);
}

static void EncoderSecondPowerEnable(void){
	LL_GPIO_SetOutputPin(PWR2_EN_PIN);
}

static void EncoderSecondPowerDisable(void){
	LL_GPIO_ResetOutputPin(PWR2_EN_PIN);
}

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

volatile uint8_t cnt_error_cycles = 0;
UartTxStr_t UART_TX;
UART_Error_t UART_Error = UART_ERROR_NONE;
UART_State_t UART_State = UART_STATE_IDLE;
volatile uint8_t usb_rx_buffer[RX_BUFFER_SIZE] = {0};
uint8_t usb_tx_buffer[TX_BUFFER_SIZE] = {0};
uint8_t hex_line_buffer[UART_LINE_SIZE] = {0};

uint32_t dma_rx_cnt = 0; 
volatile uint32_t uart_expected_length = 0; 

volatile uint8_t uart_length = 0;
volatile uint32_t new_cnt = 0;
 
volatile UART_Command_t UART_Command = 0; 
CommandQueue_t CommandQueue[QUEUE_SIZE];

uint8_t queue_read_cnt = 0;
uint8_t queue_write_cnt = 0;
uint8_t queue_cnt = 0;
uint8_t retry_cnt = 0;

QUEUE_Status_t EnqueueCommand(UART_Command_t cmd, uint16_t addr, uint8_t len,	uint8_t *data) {
	if (queue_cnt < QUEUE_SIZE){
		CommandQueue[queue_write_cnt].cmd = cmd;
		CommandQueue[queue_write_cnt].addr = addr;
		CommandQueue[queue_write_cnt].len = len;
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
			CommandQueue[queue_read_cnt].cmd = cmd;
			CommandQueue[queue_read_cnt].addr = addr;
			CommandQueue[queue_read_cnt].len = len;
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

void UART_StateMachine(void) {
    uint8_t crc;
		uint8_t calculated_crc;
    uint32_t new_cnt;

		if(IsBiSSReqBusy() == BISS_FAULT) {
			cnt_error_cycles++;
			if (cnt_error_cycles == BISS_ABORT_CNT_CYCLES) {
				cnt_error_cycles = 0;
				UART_Error = UART_ERROR_BISS;
				UART_State = UART_STATE_ABORT;
				BiSSResetExternalState();
			}
		}
	
		if(IsBiSSReqBusy() ==	BISS_READ_FINISHED) {
				UART_Transmit(&UART_TX);
				BiSSResetExternalState();
		}
		
    switch (UART_State) {
				uint8_t bytes_received;
        case UART_STATE_IDLE:
//						if (IsBiSSReqBusy() != BISS_READ_FINISHED){
//							UART_Transmit(read_buf, RX_BUFFER_SIZE);
//						}

           new_cnt = RX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA_LPUART_RX);
           if (dma_rx_cnt != new_cnt) {
								bytes_received = (new_cnt - dma_rx_cnt + RX_BUFFER_SIZE) % RX_BUFFER_SIZE;
								uart_length = usb_rx_buffer[dma_rx_cnt];
								uart_expected_length = uart_length + HEXLEN_ADR_CMD_CRC_LEN;
								if (bytes_received >= uart_expected_length) {
									UART_State = UART_STATE_RECEIVE;

//                dma_rx_cnt %= RX_BUFFER_SIZE;
//                uart_length = usb_rx_buffer[dma_rx_cnt];
//                uart_expected_length = uart_length + HEXLEN_ADR_CMD_CRC_LEN;
//                UART_State = UART_STATE_RECEIVE;
//            //change
								}
						} else {
							if (queue_cnt > 0){
								UART_State = UART_STATE_RUNCMD;
							}
						}
						break;

        case UART_STATE_RECEIVE:
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
              dma_rx_cnt = (dma_rx_cnt + uart_expected_length) % RX_BUFFER_SIZE;
            } else {
							UART_Error = UART_ERROR_LEN_DATA_IS_ZERO;
							UART_State = UART_STATE_ABORT;
							dma_rx_cnt = (dma_rx_cnt + uart_expected_length) % RX_BUFFER_SIZE;
						} 
            break;

        case UART_STATE_CHECKCRC: // TODO ???
						UART_Error = UART_ERROR_CRC;
            UART_State = UART_STATE_ABORT;  // TODO  handle CRC error
            break;

        case UART_STATE_RUNCMD:
						if (queue_cnt > 0) {
							UART_Command_t command = CommandQueue[queue_read_cnt].cmd;
							uint8_t cmd_data_len = CommandQueue[queue_read_cnt].len;
							uint16_t cmd_addr = CommandQueue[queue_read_cnt].addr;
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
									if (IsBiSSReqBusy() != BISS_BUSY) {
										if (cmd_data_len == 0x40) {
											cmd_data_len +=1;
											cmd_data[cmd_data_len] = ((cmd_addr %(0x00A0U & 0xFFU)) % 0x20U) + 6;
										}
										if (BiSSRequestWrite(cmd_addr, cmd_data_len, cmd_data) == BISS_REQ_OK) {
											UART_State = UART_STATE_IDLE;
											queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
											queue_cnt--;
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
									break;
									
								case UART_COMMAND_CHANGE_MODE:
									if (cmd_data[0] == 0) {
										if (Current_Mode != BISS_MODE_SPI_SPI){
											Change_Current_Mode(BISS_MODE_SPI_SPI);
										}
									} else if (cmd_data[0] == 1) {
										if (Current_Mode != BISS_MODE_AB_UART){
											Change_Current_Mode(BISS_MODE_AB_UART);
										}
									} else if (cmd_data[0] == 2) {
										if (Current_Mode != BISS_MODE_SPI_UART_IRS){
											Change_Current_Mode(BISS_MODE_SPI_UART_IRS);
										}
									} else if (cmd_data[0] == 3) {
										if (Current_Mode != BISS_MODE_AB_SPI){
											Change_Current_Mode(BISS_MODE_AB_SPI);
										}
									} else if (cmd_data[0] == 4) {
										if (Current_Mode != BISS_MODE_DEFAULT_SPI){
											Change_Current_Mode(BISS_MODE_DEFAULT_SPI);
										}
									}
									UART_State = UART_STATE_IDLE;
									queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
									queue_cnt--;
									break;
																	
								case UART_COMMAND_PAGE:
									if (IsBiSSReqBusy() != BISS_BUSY) {
										uint8_t cmd_data_page = cmd_data[0];
										if (BiSSRequestWrite(PAGE_ADDR, 1U, &cmd_data_page) == BISS_REQ_OK) {
											UART_State = UART_STATE_IDLE;
											queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
											queue_cnt--;
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
									break;
									
								case UART_COMMAND_ENC1_POWER_OFF:
										EncoderPowerDisable();
										UART_State = UART_STATE_IDLE;
										queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
										queue_cnt--;										
										break;
								
								case UART_COMMAND_ENC1_POWER_ON:
										EncoderPowerEnable();
										UART_State = UART_STATE_IDLE;
										queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
										queue_cnt--;
										break;	
								
								case UART_COMMAND_ENC2_POWER_OFF:
										EncoderSecondPowerDisable();
										UART_State = UART_STATE_IDLE;
										queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
										queue_cnt--;										
										break;
								
								case UART_COMMAND_ENC2_POWER_ON:
										EncoderSecondPowerEnable();
										UART_State = UART_STATE_IDLE;
										queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
										queue_cnt--;										
										break;
											
								case UART_COMMAND_SELECT_SPI_CH:
										if (cmd_data[0] == 0) {
											if (BiSS_SPI_Ch != BISS_SPI_CH_1){
												SetBiSS_SPI_Ch(BISS_SPI_CH_1);
											}
										} else if (cmd_data[0] == 1) {
											if (BiSS_SPI_Ch != BISS_SPI_CH_2){
												SetBiSS_SPI_Ch(BISS_SPI_CH_2);
											}
										}
										UART_State = UART_STATE_IDLE;
										queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
										queue_cnt--;
										break;
										
								case UART_COMMAND_CHANGE_CH1_SSI_FREQ:
									if (cmd_data[0] == 0) {
											if (CH1_SSI != CH1_SSI_HALF_FREQ) {
												SetCh1SSIFreq(CH1_SSI_HALF_FREQ);
											}
										} else if (cmd_data[0] == 1) {
											if (BiSS_SPI_Ch != CH1_SSI_FULL_FREQ){
												SetCh1SSIFreq(CH1_SSI_FULL_FREQ);
											}
										}
										UART_State = UART_STATE_IDLE;
										queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
										queue_cnt--;
									break;
									
								case UART_COMMAND_WRITE_REG:
									if (IsBiSSReqBusy() != BISS_BUSY) {
										if (BiSSRequestWrite(cmd_addr, cmd_data_len, cmd_data) == BISS_REQ_OK) {
											
											UART_State = UART_STATE_IDLE;
											queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
											queue_cnt--;
											retry_cnt = 0;
									
										} else {
											retry_cnt++;
											if (retry_cnt >= MAX_RETRY) {
												if(BiSSGetFaultState() == BISS_NO_FAULTS) {
													UART_State = UART_STATE_IDLE;
													retry_cnt = 0;
												} else {
													UART_Error = UART_ERROR_BISS;
													UART_State = UART_STATE_ABORT;
													queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
													queue_cnt--;
													retry_cnt = 0;
												}
											}
										}
									} else {
											if(BiSSGetFaultState() != BISS_NO_FAULTS) {
												UART_Error = UART_ERROR_BISS;
												UART_State = UART_STATE_ABORT;
												queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
												queue_cnt--;
												retry_cnt = 0;
											}
										}
									break;

								case UART_COMMAND_READ_REG:
										if (IsBiSSReqBusy() != BISS_BUSY) { 
											UART_TX.cmd = command;
											UART_TX.len = cmd_data_len;
											UART_TX.adr_h = (cmd_addr >> 8U) & 0xFFU;
											UART_TX.adr_l = cmd_addr & 0xFFU;
											
											if (BiSSRequestRead(cmd_addr, cmd_data_len, UART_TX.Buf) == BISS_REQ_OK) {
												
												UART_State = UART_STATE_IDLE;
												
												queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
												queue_cnt--;
												retry_cnt = 0;
											
											} else {
												retry_cnt++;
												if (retry_cnt >= MAX_RETRY) {
													if(BiSSGetFaultState() == BISS_NO_FAULTS) {
														UART_State = UART_STATE_IDLE;
														retry_cnt = 0;
													} else {
														UART_Error = UART_ERROR_BISS;
														UART_State = UART_STATE_ABORT;
														queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
														queue_cnt--;
														retry_cnt = 0;
													}
												}
												// UART_State = UART_STATE_ABORT;
											}
										} else {
											if(BiSSGetFaultState() != BISS_NO_FAULTS) {
												UART_Error = UART_ERROR_BISS;
												UART_State = UART_STATE_ABORT;
												queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
												queue_cnt--;
												retry_cnt = 0;
											}
										}
									break;
													
								case UART_COMMAND_WRITE_READ_ENC_USART2:
										UART_TX.cmd = command;
										UART_TX.len = TX_BUFFER_SIZE;
										UART_TX.adr_h = 0;
										UART_TX.adr_l = 0;
										USART2_Write_Read_IRS(cmd_data, UART_TX.Buf, cmd_data_len);
										UART_Transmit(&UART_TX);
								
										queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
										queue_cnt--;
								
										UART_State = UART_STATE_IDLE;
									break;
								
								case UART_COMMAND_READ_ANGLE_ENC_SPI_CURRENT:
										UART_TX.cmd = command;
										UART_TX.len = cmd_data_len;
										UART_TX.adr_h = 0;
										UART_TX.adr_l = 0;
										if(BiSS_SPI_Ch == BISS_SPI_CH_2){
											AngleData_t angle_data2 = getAngle2();
											*((AngleData_t*)&UART_TX.Buf[0]) = angle_data2;
											UART_Transmit(&UART_TX);
										}
										queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
										queue_cnt--;
								
										UART_State = UART_STATE_IDLE;
									break;

								case UART_COMMAND_CHANGE_CURRENT_SENSOR_MODE:
									if (cmd_data[0] == 0) {
										if (Current_Sensor_Mode != CURRENT_SENSOR_MODE_DISABLE){
											Change_Current_Sensor_Mode(CURRENT_SENSOR_MODE_DISABLE);
										}
									} else if (cmd_data[0] == 1) {
										if (Current_Sensor_Mode != CURRENT_SENSOR_MODE_ENABLE){
											Change_Current_Sensor_Mode(CURRENT_SENSOR_MODE_ENABLE);
										}
									}
									UART_State = UART_STATE_IDLE;
									queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
									queue_cnt--;
									break;
								
								case UART_COMMAND_READ_ENC2_CURRENT:
									UART_TX.cmd = command;
									UART_TX.len = cmd_data_len;
									UART_TX.adr_h = 0;
									UART_TX.adr_l = 0;
									if (Current_Sensor_Mode == CURRENT_SENSOR_MODE_ENABLE){
										int32_t current = Read_Current_Enc2();
										memcpy(&UART_TX.Buf[0], &current, sizeof(current));
										UART_Transmit(&UART_TX);
									}
									UART_State = UART_STATE_IDLE;
									queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
									queue_cnt--;
									break;
										
								case UART_COMMAND_READ_ANGLE_TWO_ENC_AB_SPI:
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
								
										UART_State = UART_STATE_ANGLE_READING_TWO_ENC_AB_SPI;
									break;
								
								case UART_COMMAND_READ_ANGLE_TWO_ENC_AB_UART:		
										UART_TX.cmd = command;
										UART_TX.len = UART_ANGLE_TWO_ENC_AB_UART_BUF_SIZE;
														
										ReadingStrRenishaw.len = cmd_addr + 1;// Address = buf_size * 63
										ReadingStrRenishaw.FIFO_current_ptr = 0;
										ReadingStrRenishaw.FIFO_start_ptr = 0;
										//ReadingStrRenishaw.ToL_cnt = 0;
								
										ReadingStrEnc2.len = cmd_addr + 1;// Address = buf_size * 63
										ReadingStrEnc2.FIFO_current_ptr = 0;
										ReadingStrEnc2.FIFO_start_ptr = 0;
										ReadingStrEnc2.ToL_cnt = 0;
								
										queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
										queue_cnt--;

										UART_State = UART_STATE_ANGLE_READING_TWO_ENC_AB_UART;
										break;
									
								case UART_COMMAND_READ_ANGLE_TWO_ENC_SPI:		
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

										UART_State = UART_STATE_ANGLE_READING_TWO_ENC_SPI;
										break;
								
								case UART_COMMAND_READ_ANGLE_ENC_SPI:			
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

										UART_State = UART_STATE_ANGLE_READING_ENC_SPI;
										break;

								case UART_COMMAND_NRST:		
 										NVIC_SystemReset();
 										break;
											
								default:
										UART_State = UART_STATE_ABORT;
										break;
							}
						}
            break;
						
				case UART_STATE_ANGLE_READING_TWO_ENC_AB_SPI:
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
									TxBufCnt += 4;
									*((AngleDataRenishaw_t*)&UART_TX.Buf[TxBufCnt]) = ReadingStrRenishaw.AngleFIFO[ReadingStrRenishaw.FIFO_start_ptr];
									TxBufCnt += 2;
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
					break;
						
				case UART_STATE_ANGLE_READING_TWO_ENC_SPI:
						if(ReadingStrEnc1.len > 0) {
							AngleData_t angle_data1 = getAngle1();
							AngleData_t angle_data2 = getAngle2();
				
							if(angle_data1.time_of_life_counter != ReadingStrEnc1.ToL_cnt) {
								
								ReadingStrEnc1.ToL_cnt = angle_data1.time_of_life_counter;
								ReadingStrEnc1.AngleFIFO[ReadingStrEnc1.FIFO_current_ptr] = angle_data1;
								ReadingStrEnc1.FIFO_current_ptr++;
								
								ReadingStrEnc2.ToL_cnt = angle_data2.time_of_life_counter;
								ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.FIFO_current_ptr] = angle_data2;
								ReadingStrEnc2.FIFO_current_ptr++;
								
								if((((uint16_t)ReadingStrEnc1.FIFO_current_ptr + 256 - ReadingStrEnc1.FIFO_start_ptr) & 0xFFU) >= UART_ANGLE_TWO_ENC_LEN) {
									uint8_t TxBufCnt = 0;
									while(ReadingStrEnc1.FIFO_start_ptr != ReadingStrEnc1.FIFO_current_ptr) {
										*((AngleData_t*)&UART_TX.Buf[TxBufCnt]) = ReadingStrEnc1.AngleFIFO[ReadingStrEnc1.FIFO_start_ptr];
										TxBufCnt += 4;
										*((AngleData_t*)&UART_TX.Buf[TxBufCnt]) = ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.FIFO_start_ptr];
										TxBufCnt += 4;
										ReadingStrEnc1.FIFO_start_ptr++; 
										ReadingStrEnc2.FIFO_start_ptr++; 
									}
									ReadingStrEnc1.len--; 
									ReadingStrEnc2.len--; 
									UART_TX.adr_h = (ReadingStrEnc1.len >> 8U) & 0xFFU;
									UART_TX.adr_l = ReadingStrEnc1.len & 0xFFU;
									UART_Transmit(&UART_TX);
								}
							}
					}	else {
						UART_State = UART_STATE_IDLE;
					}
					break;
				
				case UART_STATE_ANGLE_READING_TWO_ENC_AB_UART:
					if(ReadingStrEnc2.len > 0) {
						AngleData_t angle_data2 = getAngle2();
						AngleDataRenishaw_t angle_data1 = getAngleRenishaw();
						
						if(angle_data2.time_of_life_counter != ReadingStrEnc2.ToL_cnt) {
								
								ReadingStrEnc2.ToL_cnt = angle_data2.time_of_life_counter;
								ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.FIFO_current_ptr] = angle_data2;
								ReadingStrEnc2.FIFO_current_ptr++;
								
								//ReadingStrRenishaw.ToL_cnt = angle_data1.time_of_life_counter;
								ReadingStrRenishaw.AngleFIFO[ReadingStrRenishaw.FIFO_current_ptr] = angle_data1;
								ReadingStrRenishaw.FIFO_current_ptr++;
								
								if((((uint16_t)ReadingStrEnc2.FIFO_current_ptr + 256 - ReadingStrEnc2.FIFO_start_ptr) & 0xFFU) >= UART_ANGLE_TWO_ENC_AB_UART_LEN) {
									uint8_t TxBufCnt = 0;
									while(ReadingStrEnc2.FIFO_start_ptr != ReadingStrEnc2.FIFO_current_ptr) {
										*((AngleData_t*)&UART_TX.Buf[TxBufCnt]) = ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.FIFO_start_ptr];
										TxBufCnt += 4;
										*((AngleDataRenishaw_t*)&UART_TX.Buf[TxBufCnt]) = ReadingStrRenishaw.AngleFIFO[ReadingStrRenishaw.FIFO_start_ptr];
										TxBufCnt += 2;
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
					break;
				
				case UART_STATE_ANGLE_READING_ENC_SPI:
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
											TxBufCnt += 4;
											ReadingStrEnc1.FIFO_start_ptr++;
										}
										ReadingStrEnc1.len--;
										UART_TX.adr_h = (ReadingStrEnc1.len >> 8U) & 0xFFU;
										UART_TX.adr_l = ReadingStrEnc1.len & 0xFFU;
										UART_Transmit(&UART_TX);
									}
								}
						}	else {
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
						}	else {
							UART_State = UART_STATE_IDLE;
						}
					}
					break;
					
        case UART_STATE_ABORT:
					switch(UART_Error) {
						case UART_ERROR_CRC:
							// TODO: Add UART transmit for error
							UART_TX.cmd = error_uart_cmd;
							UART_TX.len = 0+1;
							UART_TX.adr_h = 0;
							UART_TX.adr_l = 0;
							UART_TX.Buf[0] = (uint8_t)UART_Error;
							UART_Transmit(&UART_TX);
							UART_Error = UART_ERROR_NONE;
							break;
						case UART_ERROR_QUEUE_FULL:
							// TODO: Add UART transmit for error
							UART_TX.cmd = error_uart_cmd;
							UART_TX.len = 0+1;
							UART_TX.adr_h = 0;
							UART_TX.adr_l = 0;
							UART_TX.Buf[0] = (uint8_t)UART_Error;
							UART_Transmit(&UART_TX);
							UART_Error = UART_ERROR_NONE;
							break;
						case UART_ERROR_BISS:
							UART_TX.cmd = error_biss_cmd;
							UART_TX.len = 0+1;  
							UART_TX.adr_h = 0;
							UART_TX.adr_l = 0;
							UART_TX.Buf[0] = (uint8_t)BiSSGetFaultState();
							// UART_TX.Buf[1] = (uint8_t)IsBiSSReqBusy(); // TODO: add BiSSExternalState_t ??? 
							UART_Transmit(&UART_TX);
							UART_Error = UART_ERROR_NONE;
							break;
						case UART_ERROR_LEN_DATA_IS_ZERO:
							UART_TX.cmd = error_uart_cmd;
							UART_TX.len = 0+1;
							UART_TX.adr_h = 0;
							UART_TX.adr_l = 0;
							UART_TX.Buf[0] = (uint8_t)UART_Error;
							UART_Transmit(&UART_TX);
							UART_Error = UART_ERROR_NONE;
						default:
							__NOP();
							__NOP();
							__NOP();
							UART_State = UART_STATE_IDLE;
							break;
					}
					break;

        default:
					UART_State = UART_STATE_IDLE;
          break;
    }
}