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


/* Frame layout: [len][addr_h][addr_l][cmd][data...][crc] */
#define HEXLEN_ADR_CMD_CRC_LEN					5U // Length of data (1) + Address (2) + Cmd (1) + CRC (1) bytes
#define HEX_DATA_LEN							128U
#define UART_LINE_SIZE							HEXLEN_ADR_CMD_CRC_LEN + HEX_DATA_LEN
/* Command queue depth. */
#define QUEUE_SIZE 								36U //FIFO
/* Maximum number of retries for a BiSS transaction. */
#define MAX_RETRY								3U
/* Streaming angle parameters. */
#define UART_ANGLE_LEN 							60U // 60U --> Encoder
#define UART_ANGLE_BUF_SIZE 					(UART_ANGLE_LEN * 4U) // *4U --> Encoder
#define UART_ANGLE_TWO_ENC_LEN 					30U // 30U --> Encoder1 + Encoder2
#define UART_ANGLE_TWO_ENC_BUF_SIZE 			(UART_ANGLE_TWO_ENC_LEN * 8U) // *(4U + 4U) --> Encoder1 + Encoder2
#define UART_ANGLE_TWO_ENC_AB_UART_LEN 			40U // 40U --> Encoder1 + Renishaw
#define UART_ANGLE_TWO_ENC_AB_UART_BUF_SIZE 	(UART_ANGLE_TWO_ENC_AB_UART_LEN * 6U) // *(4U + 2U) --> Encoder1 + Renishaw
/* Sizes of angle data in bytes. */
#define ANGLE_DATA_SIZE							4U
#define RENISHAW_ANGLE_DATA_SIZE				2U
/* BiSS register addresses used by the protocol layer. */
#define WRITE_BANK_DATA_LEN 					64U
#define PAGE_ADDR       						0x18U
#define BSEL_ADDR		    					0x40U
#define FIRST_USER_BANK 						0x05U
/* Size of the DMA TX buffer: payload + length + addr + cmd + crc. */
#define TX_DMA_OVERHEAD                         5U
/* Bit added to the command byte in every reply frame. */
#define TX_CMD_REPLY_FLAG                       0x10U


/*==========================================================================*/
/* Type definitions                                                         */
/*==========================================================================*/

/* Error type classification. */
typedef enum
{
	ERROR_TYPE_NONE = 0x00,
	ERROR_TYPE_BISS = 0x01U,
    ERROR_TYPE_UART = 0x02U

} UART_Error_Type_t;

/* Detailed error codes reported to the host. */
typedef enum
{
	UART_ERROR_NONE               = 0x00,
	UART_ERROR_CRC                = 0x01U,
	UART_ERROR_QUEUE_FULL         = 0x02U,
	UART_ERROR_BISS               = 0x03U,
	UART_ERROR_BISS_WRITE_FAULT   = 0x04U,
	UART_ERROR_BISS_READ_FAULT    = 0x05U,
	UART_ERROR_LEN_DATA_IS_ZERO   = 0x06U,
	UART_ERROR_LEN_IS_NOT_CORRECT = 0x07U,
	UART_ERROR_INVALID_CMD        = 0x08U,
	UART_ERROR_INVALID_MODE       = 0x09U,
	UART_ERROR_INVALID_CHANNEL    = 0x10U,

} UART_Error_t;

/* Aggregated error status reported to the host. */
typedef struct 
{
	UART_Error_Type_t error_type;
	UART_Error_t      error_code;
	BiSSFaultState_t  biss_fault_state;

}UART_Error_Status_t;

/* Return status of the command queue. */
typedef enum
{
    QUEUE_OK,
    QUEUE_FULL

} QUEUE_Status_t;

/* One command in the queue. */
typedef struct
{
	uint8_t        len;
	uint16_t       addr;
	UART_Command_t cmd;
	uint8_t        data[HEX_DATA_LEN];

}CommandQueue_t;

/* One frame in the TX queue. */
typedef struct
{
    UartTxStr_t frame;
    uint8_t active;
	uint8_t is_stream_frame;

} TxQueueItem_t;

/* TX queue: fixed-size ring buffer. */
typedef struct
{
    TxQueueItem_t items[TX_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;

} TxQueue_t;

/* Streaming mode selection. */
typedef enum
{
    STREAM_NONE = 0,
    STREAM_AB_SPI,
    STREAM_AB_UART,
    STREAM_SPI_SPI,
    STREAM_SPI

} StreamMode_t;

/* Circular buffer of angle values used during streaming. */
typedef struct
{
    AngleData_t AngleFIFO[256];
    uint16_t    len;
    uint8_t     tol_cnt;
    uint8_t     fifo_start_ptr;
    uint8_t     fifo_current_ptr;

} AngleFifo_t;

/* Circular buffer of Renishaw angle values. */
typedef struct
{
    AngleDataRenishaw_t AngleFIFO[256];
    uint16_t    len;
    uint8_t     fifo_start_ptr;
    uint8_t     fifo_current_ptr;

} AngleRenishawFifo_t;

/*==========================================================================*/
/* Function prototypes (static)                                             */
/*==========================================================================*/

static QUEUE_Status_t EnqueueCommand(UART_Command_t cmd, uint16_t addr,
	                                 uint8_t len, uint8_t *data);
static QUEUE_Status_t EnqueueCommandToBegining(UART_Command_t cmd,
	                                           uint16_t addr, uint8_t len,
                                               uint8_t *data);
static uint8_t CalculateCRC(uint8_t *data, uint32_t length);
static uint8_t CalculateCRCCircularBuffer(uint8_t *buffer,
	                                      uint16_t buffer_size,
                                          uint8_t start_index,
                                          uint8_t length);
static void finalize_successful_command(void);
static void abort_current_command(UART_Command_t cmd);
static uint8_t UART_CommandHasReply(UART_Command_t cmd);

static void handle_write_bank_command(UART_Command_t cmd, uint8_t cmd_data_len,
                                      uint16_t cmd_addr, uint8_t* cmd_data);
static void handle_change_mode_command(uint8_t mode);
static void handle_page_command(UART_Command_t cmd, uint8_t data_to_page);
static void handle_select_spi_ch_command(uint8_t channel);
static void handle_change_ch1_mode_command(uint8_t ch1_mode);
static void handle_write_reg_command(uint8_t cmd_data_len, uint16_t cmd_addr,
	                                 uint8_t* cmd_data, UART_Command_t command);
static void handle_read_reg_command(uint8_t cmd_data_len, uint16_t cmd_addr,
	                                UART_Command_t command);
static void handle_write_read_enc_usart2_command(uint8_t cmd_data_len,
	                                             uint8_t* cmd_data,
                                                 UART_Command_t command);
static void handle_read_angle_enc_spi_instant_command(uint8_t cmd_data_len,
	                                                  UART_Command_t command);
static void handle_read_angle_packet_enc_spi_instant_command(
                                                uint8_t cmd_data_len,
                                                UART_Command_t command);
static void handle_change_current_sensor_mode_command(uint8_t current_mode);
static void handle_read_enc2_current_command(uint8_t cmd_data_len,
	                                         UART_Command_t command);
static void handle_read_uart_error_status(uint8_t cmd_data_len,
                                          UART_Command_t command);
static void reset_uart_error_status(void);

static void prepare_encoder_ab_spi_reading(uint16_t cmd_addr,
	                                       UART_Command_t command);
static void prepare_encoder_ab_uart_reading(uint16_t cmd_addr,
	                                        UART_Command_t command);
static void prepare_encoder_spi_spi_reading(uint16_t cmd_addr,
	                                        UART_Command_t command);
static void prepare_encoder_spi_reading(uint16_t cmd_addr,
	                                    UART_Command_t command);

static void handle_run_command_state(void);

static void LPUART1_UART_Init(void);
static void LPUART1_Config(void);

static void UART_RxTask(void);
static void UART_CmdTask(void);
static void UART_StreamTask(void);
static void UART_TxTask(void);

static void     uart_flush_rx(void);
static uint16_t uart_get_dma_write_pos(void);

static uint8_t TxQueue_Push(const UartTxStr_t *frame);
static uint8_t TxQueue_PushTagged(const UartTxStr_t *frame,
	                              uint8_t is_stream_frame);
static uint8_t TxQueue_Pop(UartTxStr_t *frame);
static void TxQueue_RemoveStreamFrames(void);
static void UART_CancelStream(void);
static uint8_t TxQueue_IsEmpty(void);
static void UART_QueueErrorResponse(uint16_t address, 
	                                     UART_Command_t offender_cmd,
                                         UART_Error_Status_t uart_error_status);

static void pending_reply_prepare(UART_Command_t cmd, uint16_t addr,
                                  uint8_t len);
static void pending_reply_finish(void);

static void UART_TransmitDMA(UartTxStr_t *TxStr);

static uint8_t stream_build_frame(UartTxStr_t *out);
static uint8_t stream_is_finished(void);
static uint8_t is_spi_ch_selection_allowed(uint8_t mode);

/*==========================================================================*/
/* Module variables                                                         */
/*==========================================================================*/

/* Last error status reported to the host. */
static UART_Error_Status_t uart_error_status =
{
    .error_type       = ERROR_TYPE_NONE,
    .error_code       = UART_ERROR_NONE,
    .biss_fault_state = BISS_NO_FAULTS
};

/* Command queue. */
static CommandQueue_t CommandQueue[QUEUE_SIZE];

/* TX queue. */
static TxQueue_t tx_queue;

/* Reply frame bound to a specific reply-command. */
static UartTxStr_t pending_reply;
static uint8_t     pending_reply_active = 0;

static uint8_t write_pending = 0;
static UART_Command_t pending_write_command;

/* RX DMA buffer and helpers. */
static volatile uint8_t usb_rx_buffer[RX_BUFFER_SIZE];
static uint8_t          usb_tx_buffer[TX_BUFFER_SIZE + TX_DMA_OVERHEAD];
static uint8_t          hex_line_buffer[UART_LINE_SIZE];

static volatile uint16_t dma_rx_cnt = 0U;
static volatile uint32_t uart_expected_length = 0U;
static volatile uint8_t  uart_length = 0U;

static uint8_t queue_read_cnt = 0U;
static uint8_t queue_write_cnt = 0U;
static uint8_t queue_cnt = 0U;
static uint8_t retry_cnt = 0U;

/* Streaming buffers. */
static volatile AngleFifo_t          ReadingStrEnc1;
static volatile AngleFifo_t          ReadingStrEnc2;
static volatile AngleRenishawFifo_t  ReadingStrRenishaw;

/* Current streaming mode. */
static StreamMode_t stream_mode = STREAM_NONE;

static volatile uint8_t uart_rx_frame_ready = 0;
static volatile uint16_t uart_rx_frame_end = 0;

/*==========================================================================*/
/* Local functions: command classification                                  */
/*==========================================================================*/

/*!
 * @brief Returns 1 if the command expects a reply from the firmware.
 *
 * @param[in] cmd Command code.
 * @return 1 if a reply frame must be sent to the host.
 * @return 0 if the command is silent (FlashTool configuration).
 */
static uint8_t UART_CommandHasReply(UART_Command_t cmd)
{
    uint8_t has_reply = 1;

    switch (cmd)
    {
        case UART_COMMAND_CHANGE_MODE:
        case UART_COMMAND_PAGE:
        case UART_COMMAND_ENC1_POWER_OFF:
        case UART_COMMAND_ENC1_POWER_ON:
        case UART_COMMAND_SELECT_SPI_CH:
        case UART_COMMAND_ENC2_POWER_OFF:
        case UART_COMMAND_ENC2_POWER_ON:
        case UART_COMMAND_CHANGE_CH1_MODE:
        case UART_COMMAND_CHANGE_CURRENT_SENSOR_MODE:
        case UART_COMMAND_NRST:
        case UART_COMMAND_REBOOT_TO_BL:
            has_reply = 0;
            break;

        default:
            has_reply = 1;
            break;
    }

    return has_reply;
}

/*==========================================================================*/
/* Local functions: TX queue                                                */
/*==========================================================================*/

/*!
 * @brief Checks whether the TX queue is empty.
 *
 * @return 1 if empty.
 */
static uint8_t TxQueue_IsEmpty(void)
{
    return (tx_queue.count == 0U);
}

/*!
 * @brief Pushes a frame into the TX queue.
 *
 * @param[in] frame Pointer to the frame to copy.
 * @return 1 on success, 0 if the queue is full.
 */
static uint8_t TxQueue_Push(const UartTxStr_t *frame)
{
    return TxQueue_PushTagged(frame, 0);
}

static uint8_t TxQueue_PushTagged(const UartTxStr_t *frame,
	                              uint8_t is_stream_frame)
{
	if (tx_queue.count >= TX_QUEUE_SIZE)
    {
        return 0;
    }

    TxQueueItem_t *item = &tx_queue.items[tx_queue.head];

    item->frame = *frame;
    item->is_stream_frame = is_stream_frame;
    item->active = 1U;

    tx_queue.head = (tx_queue.head + 1U) % TX_QUEUE_SIZE;
    tx_queue.count++;

    return 1U;
}

/*!
 * @brief Pops a frame from the TX queue.
 *
 * @param[out] frame Destination for the popped frame.
 * @return 1 on success, 0 if the queue is empty.
 */
static uint8_t TxQueue_Pop(UartTxStr_t *frame)
{
    uint8_t result = 0;

    if (tx_queue.count > 0U)
    {
        *frame = tx_queue.items[tx_queue.tail].frame;
        tx_queue.items[tx_queue.tail].active = 0;
        tx_queue.tail = (uint8_t)((tx_queue.tail + 1U) % TX_QUEUE_SIZE);
        tx_queue.count--;
        result = 1;
    }

    return result;
}

static void TxQueue_RemoveStreamFrames(void)
{
    uint16_t original_count = tx_queue.count;
    uint16_t kept_count = 0;

    for (uint16_t i = 0; i < original_count; i++)
    {
        uint16_t src =
            (tx_queue.tail + i) % TX_QUEUE_SIZE;

        if (tx_queue.items[src].is_stream_frame != 0)
        {
            tx_queue.items[src].active = 0;
            continue;
        }

        uint16_t dst =
            (tx_queue.tail + kept_count) % TX_QUEUE_SIZE;

        if (dst != src)
        {
            tx_queue.items[dst] = tx_queue.items[src];
        }

        kept_count++;
    }

    tx_queue.count = kept_count;
    tx_queue.head =
        (tx_queue.tail + kept_count) % TX_QUEUE_SIZE;
}

static void UART_CancelStream(void)
{
    if (stream_mode == STREAM_NONE)
    {
        return;
    }

    stream_mode = STREAM_NONE;

    pending_reply_active = 0;

    TxQueue_RemoveStreamFrames();
}

static void UART_QueueErrorResponse(
    uint16_t address,
    UART_Command_t offender_cmd,
    UART_Error_Status_t error_status)
{
    UartTxStr_t frame = {0};
	
	frame.len = 4U;
	frame.adr_h = (uint8_t)((address >> 8U) & 0xFFU);;
	frame.adr_l = (uint8_t)(address & 0xFFU);
	frame.cmd = (UART_Command_t)UART_CMD_ERROR_FRAME;

	frame.Buf[0] = (uint8_t)offender_cmd;
    frame.Buf[1] = (uint8_t)error_status.error_type;
    frame.Buf[2] = (uint8_t)error_status.error_code;
    frame.Buf[3] = (uint8_t)error_status.biss_fault_state;

    if (!TxQueue_Push(&frame))
	{
		uart_error_status.error_type = ERROR_TYPE_UART;
		uart_error_status.error_code = UART_ERROR_QUEUE_FULL;
	}
}

/*==========================================================================*/
/* Local functions: pending reply                                           */
/*==========================================================================*/

/*!
 * @brief Prepares the pending reply frame for a specific command.
 *
 * The BiSS HAL writes read data directly into pending_reply.Buf.
 *
 * @param[in] cmd  Command code.
 * @param[in] addr Register address.
 * @param[in] len  Expected length of the payload.
 */
static void pending_reply_prepare(UART_Command_t cmd, uint16_t addr,
                                  uint8_t len)
{
    pending_reply.len   = len;
    pending_reply.adr_h = (uint8_t)((addr >> 8U) & 0xFFU);
    pending_reply.adr_l = (uint8_t)(addr & 0xFFU);
    pending_reply.cmd   = cmd;
    pending_reply_active = 1;
}

/*!
 * @brief Pushes the pending reply into the TX queue and clears the flag.
 */
static void pending_reply_finish(void)
{
    if (pending_reply_active)
    {
        if (!TxQueue_Push(&pending_reply))
        {
            uart_error_status.error_type = ERROR_TYPE_UART;
			uart_error_status.error_code = UART_ERROR_QUEUE_FULL;
        }
        pending_reply_active = 0;
    }
}

/*==========================================================================*/
/* Local functions: helpers                                                 */
/*==========================================================================*/

/*!
 * @brief Marks the current command as successfully completed.
 */
static void finalize_successful_command(void)
{
    queue_read_cnt = (uint8_t)((queue_read_cnt + 1U) % QUEUE_SIZE);
    queue_cnt--;
}

/*!
 * @brief Aborts the current command and notifies the host if a reply
 *        was expected.
 */
static void abort_current_command(UART_Command_t cmd)
{
    if (UART_CommandHasReply(cmd))
    {
		uart_error_status.biss_fault_state = BiSSGetFaultState();
		UART_Error_Status_t error_status = uart_error_status;
		UART_QueueErrorResponse(CommandQueue[queue_read_cnt].addr, cmd,
		                        error_status);
    }

    pending_reply_active = 0;
    queue_read_cnt = (uint8_t)((queue_read_cnt + 1U) % QUEUE_SIZE);
    queue_cnt--;
}

/*!
 * @brief Discards all bytes currently present in the RX DMA buffer.
 */
static void uart_flush_rx(void)
{
	uint32_t primask = __get_PRIMASK();
	 __disable_irq();
    dma_rx_cnt = uart_get_dma_write_pos();
	uart_rx_frame_ready = 0;
	__set_PRIMASK(primask);
}

/*!
 * @brief Returns the current DMA write position in the RX buffer.
 *
 * @return Index of the next byte to be written by DMA.
 */
static uint16_t uart_get_dma_write_pos(void)
{
    return (uint16_t)(RX_BUFFER_SIZE - LL_DMA_GetDataLength(DMA_LPUART_RX));
}

/*==========================================================================*/
/* Local functions: command handler                                         */
/*==========================================================================*/

static void handle_write_bank_command(UART_Command_t cmd, uint8_t cmd_data_len,
	                                  uint16_t cmd_addr, uint8_t* cmd_data)
{
    if (IsBiSSReqBusy() == BISS_BUSY) 
	{
		return;
	}

    if (cmd_data_len == WRITE_BANK_DATA_LEN)
	{
        cmd_data_len += 1;
        cmd_data[cmd_data_len] = ((cmd_addr % (0x00A0U & 0xFFU)) % 0x20U) + 6;
    }

	if (BiSSRequestWrite(cmd_addr, cmd_data_len, cmd_data) == BISS_REQ_OK)
	{
        finalize_successful_command();
        retry_cnt = 0;
    }
	else
	{
        retry_cnt++;
        if (retry_cnt >= MAX_RETRY)
		{
            if (BiSSGetFaultState() == BISS_NO_FAULTS)
			{
                finalize_successful_command();
            }
			else
			{
				uart_error_status.error_type = ERROR_TYPE_BISS;
                uart_error_status.error_code = UART_ERROR_BISS;
                abort_current_command(cmd);
            }
            retry_cnt = 0;
        }
    }
}

static void handle_change_mode_command(uint8_t mode)
{
    switch (mode) 
	{
        case 0:
			if (Current_Mode != BISS_MODE_SPI_SPI)
			{
				Change_Current_Mode(BISS_MODE_SPI_SPI);
			}
		break;

        case 1: 
			if (Current_Mode != BISS_MODE_AB_UART)
			{
				Change_Current_Mode(BISS_MODE_AB_UART);
			}
		break;

        case 2: 
			if (Current_Mode != BISS_MODE_SPI_UART_IRS)
			{
				Change_Current_Mode(BISS_MODE_SPI_UART_IRS);
			}
		break;

        case 3: 
			if (Current_Mode != BISS_MODE_AB_SPI)
			{
				Change_Current_Mode(BISS_MODE_AB_SPI); 
			}
		break;

        case 4:
        default:
			if (Current_Mode != BISS_MODE_DEFAULT_SPI)
			{
				Change_Current_Mode(BISS_MODE_DEFAULT_SPI); 
			}
		break;
    }
}

static void handle_page_command(UART_Command_t cmd, uint8_t data_to_page)
{
    if (IsBiSSReqBusy() == BISS_BUSY) 
	{
		return;
	}

    uint8_t cmd_data_page = data_to_page;
    if (BiSSRequestWrite(PAGE_ADDR, 1U, &cmd_data_page) == BISS_REQ_OK)
	{
        finalize_successful_command();
        retry_cnt = 0;
        uint8_t bank = FIRST_USER_BANK;
        if (EnqueueCommandToBegining(
			UART_COMMAND_WRITE_REG, BSEL_ADDR, 1U, &bank) != QUEUE_OK)
		{
            uart_error_status.error_type = ERROR_TYPE_UART;
			uart_error_status.error_code = UART_ERROR_QUEUE_FULL;
			uart_error_status.biss_fault_state = BiSSGetFaultState();
        }
    } 
	else
	{
        retry_cnt++;
        if (retry_cnt >= MAX_RETRY)
		{
            if (BiSSGetFaultState() == BISS_NO_FAULTS)
			{
                finalize_successful_command();
            } 
			else
			{
				uart_error_status.error_type = ERROR_TYPE_BISS;
                uart_error_status.error_code = UART_ERROR_BISS;
                abort_current_command(cmd);
            }
            retry_cnt = 0;
        }
    }
}

static void handle_select_spi_ch_command(uint8_t channel)
{
    switch (channel)
	{
        case 0: 
			if (BiSS_SPI_Ch != BISS_SPI_CH_1)
			{
				SetBiSS_SPI_Ch(BISS_SPI_CH_1); 
			}
		break;

        case 1: 
			if (BiSS_SPI_Ch != BISS_SPI_CH_2)
			{
				SetBiSS_SPI_Ch(BISS_SPI_CH_2);
			}
		break;

		default:
			if (BiSS_SPI_Ch != BISS_SPI_CH_1)
			{
				SetBiSS_SPI_Ch(BISS_SPI_CH_1);
			}
		break;
    }
}

static void handle_change_ch1_mode_command(uint8_t ch1_mode)
{
    switch (ch1_mode)
	{
        case 0: 
			if (CH1_SPI_MODE != CH1_LENZ_BISS)
			{
				Set_Ch1_Mode(CH1_LENZ_BISS); 
			}
		break;
			
        case 1: 
			if (CH1_SPI_MODE != CH1_LIR_SSI)
			{
				Set_Ch1_Mode(CH1_LIR_SSI); 
			}
		break;

        case 2: 
			if (CH1_SPI_MODE != CH1_LIR_BISS_21B)
			{
				Set_Ch1_Mode(CH1_LIR_BISS_21B);
			}
		break;

        default:
			if (CH1_SPI_MODE != CH1_LENZ_BISS)
			{
				Set_Ch1_Mode(CH1_LENZ_BISS); 
			}
		break;
    }
}

static void handle_write_reg_command(uint8_t cmd_data_len, uint16_t cmd_addr,
	                                 uint8_t* cmd_data, UART_Command_t command)
{
	if (write_pending)
    {
        return;
    }

    if (IsBiSSReqBusy() == BISS_BUSY)
	{
		return;
	}

    if (BiSSRequestWrite(cmd_addr, cmd_data_len, cmd_data) == BISS_REQ_OK)
	{
//        finalize_successful_command();
		pending_write_command = command;
		write_pending = 1U;
        retry_cnt = 0;
    } 
	else 
	{
        retry_cnt++;
        if (retry_cnt >= MAX_RETRY)
		{
			uart_error_status.error_type = ERROR_TYPE_BISS;
            uart_error_status.error_code = UART_ERROR_BISS_WRITE_FAULT;
            abort_current_command(command);
            retry_cnt = 0;
        }
    }
}

static void handle_read_reg_command(uint8_t cmd_data_len, uint16_t cmd_addr,
	                                UART_Command_t command)
{
    if (IsBiSSReqBusy() == BISS_BUSY) 
	{
		return;
	}

    pending_reply_prepare(command, cmd_addr, cmd_data_len);

    if (BiSSRequestRead(cmd_addr, cmd_data_len, pending_reply.Buf) == BISS_REQ_OK)
	{
        finalize_successful_command();
        retry_cnt = 0;
    } 
	else 
	{
        retry_cnt++;
        if (retry_cnt >= MAX_RETRY) 
		{
			uart_error_status.error_type = ERROR_TYPE_BISS;
            uart_error_status.error_code = UART_ERROR_BISS_READ_FAULT;
            abort_current_command(command);
            retry_cnt = 0;
        }
    }
}

static void handle_write_read_enc_usart2_command(uint8_t cmd_data_len,
	                                             uint8_t* cmd_data,
                                                 UART_Command_t command)
{
    pending_reply_prepare(command, 0, cmd_data_len);
    USART2_Write_Read_IRS(cmd_data, pending_reply.Buf, cmd_data_len);
    pending_reply_finish();
}

static void handle_read_angle_enc_spi_instant_command(uint8_t cmd_data_len,
	                                                  UART_Command_t command)
{
    pending_reply_prepare(command, 0, cmd_data_len);

    if (BiSS_SPI_Ch == BISS_SPI_CH_2)
	{
        AngleData_t a = getAngle2();
        *((AngleData_t*)&pending_reply.Buf[0]) = a;
    } 
	else if (BiSS_SPI_Ch == BISS_SPI_CH_1)
	{
        AngleData_t a = getAngle1();
        *((AngleData_t*)&pending_reply.Buf[0]) = a;
    }
    pending_reply_finish();
}

static void handle_read_angle_packet_enc_spi_instant_command(
	                                            uint8_t cmd_data_len,
	                                            UART_Command_t command)
{
    pending_reply_prepare(command, 0, cmd_data_len);

    if (BiSS_SPI_Ch == BISS_SPI_CH_2)
	{
        AnglePacket_t p = getAnglePacket2();
        *((AnglePacket_t*)&pending_reply.Buf[0]) = p;
    } 
	else if (BiSS_SPI_Ch == BISS_SPI_CH_1)
	{
        AnglePacket_t p = getAnglePacket1();
        *((AnglePacket_t*)&pending_reply.Buf[0]) = p;
    }
    pending_reply_finish();
}

static void handle_change_current_sensor_mode_command(uint8_t current_mode)
{
    switch (current_mode)
	{
        case 0:
			if (Current_Sensor_Mode != CURRENT_SENSOR_MODE_DISABLE)
			{
				Change_Current_Sensor_Mode(CURRENT_SENSOR_MODE_DISABLE);
			}
		break;

        case 1:
        default:
			if (Current_Sensor_Mode != CURRENT_SENSOR_MODE_ENABLE)
			{
				Change_Current_Sensor_Mode(CURRENT_SENSOR_MODE_ENABLE); 
			}
		break;
    }
}

static void handle_read_uart_error_status(uint8_t cmd_data_len,
	                                      UART_Command_t command)
{
    pending_reply_prepare(command, 0, cmd_data_len);
    pending_reply.Buf[0] = (uint8_t)uart_error_status.error_type;
    pending_reply.Buf[1] = (uint8_t)uart_error_status.error_code;
    pending_reply.Buf[2] = (uint8_t)uart_error_status.biss_fault_state;
    pending_reply_finish();
}

static void reset_uart_error_status(void)
{
    uart_error_status.error_type = ERROR_TYPE_NONE;
    uart_error_status.error_code = UART_ERROR_NONE;
}

static void handle_read_enc2_current_command(uint8_t cmd_data_len, UART_Command_t command)
{
    pending_reply_prepare(command, 0, cmd_data_len);
    int32_t current = Read_Current_Enc2();
    memcpy(&pending_reply.Buf[0], &current, sizeof(current));
    pending_reply_finish();
}

/*==========================================================================*/
/* Local functions: prepare encoder reading                                 */
/*==========================================================================*/

static void prepare_encoder_ab_spi_reading(uint16_t cmd_addr, 
	                                       UART_Command_t command)
{
//    pending_reply_prepare(command, 0, UART_ANGLE_TWO_ENC_AB_UART_BUF_SIZE);
	pending_reply.cmd    = command;
    pending_reply.len    = UART_ANGLE_TWO_ENC_AB_UART_BUF_SIZE;
    pending_reply.adr_h  = 0;
    pending_reply.adr_l  = 0;
    pending_reply_active = 0;
	
    ReadingStrRenishaw.len = cmd_addr + 1U;
    ReadingStrRenishaw.fifo_current_ptr = 0;
    ReadingStrRenishaw.fifo_start_ptr = 0;
    ReadingStrEnc2.len = cmd_addr + 1U;
    ReadingStrEnc2.fifo_current_ptr = 0;
    ReadingStrEnc2.fifo_start_ptr = 0;
    ReadingStrEnc2.tol_cnt = 0;
    queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
    queue_cnt--;
    stream_mode = STREAM_AB_SPI;
}

static void prepare_encoder_ab_uart_reading(uint16_t cmd_addr,
	                                        UART_Command_t command)
{
//    pending_reply_prepare(command, 0, UART_ANGLE_TWO_ENC_AB_UART_BUF_SIZE);
	pending_reply.cmd    = command;
    pending_reply.len    = UART_ANGLE_TWO_ENC_AB_UART_BUF_SIZE;
    pending_reply.adr_h  = 0;
    pending_reply.adr_l  = 0;
    pending_reply_active = 0;

    ReadingStrRenishaw.len = cmd_addr + 1U;
    ReadingStrRenishaw.fifo_current_ptr = 0;
    ReadingStrRenishaw.fifo_start_ptr = 0;
    ReadingStrEnc2.len = cmd_addr + 1U;
    ReadingStrEnc2.fifo_current_ptr = 0;
    ReadingStrEnc2.fifo_start_ptr = 0;
    ReadingStrEnc2.tol_cnt = 0;
    queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
    queue_cnt--;
    stream_mode = STREAM_AB_UART;
}

static void prepare_encoder_spi_spi_reading(uint16_t cmd_addr,
	                                        UART_Command_t command)
{
//    pending_reply_prepare(command, 0, UART_ANGLE_TWO_ENC_BUF_SIZE);
	pending_reply.cmd    = command;
    pending_reply.len    = UART_ANGLE_TWO_ENC_BUF_SIZE;
    pending_reply.adr_h  = 0;
    pending_reply.adr_l  = 0;
    pending_reply_active = 0;

    ReadingStrEnc1.len = cmd_addr + 1U;
    ReadingStrEnc1.fifo_current_ptr = 0;
    ReadingStrEnc1.fifo_start_ptr = 0;
    ReadingStrEnc1.tol_cnt = 0;
    ReadingStrEnc2.len = cmd_addr + 1U;
    ReadingStrEnc2.fifo_current_ptr = 0;
    ReadingStrEnc2.fifo_start_ptr = 0;
    ReadingStrEnc2.tol_cnt = 0;
    queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
    queue_cnt--;
    stream_mode = STREAM_SPI_SPI;
}

static void prepare_encoder_spi_reading(uint16_t cmd_addr,
	                                    UART_Command_t command)
{
//    pending_reply_prepare(command, 0, UART_ANGLE_BUF_SIZE);
	pending_reply.cmd    = command;
    pending_reply.len    = UART_ANGLE_BUF_SIZE;
    pending_reply.adr_h  = 0;
    pending_reply.adr_l  = 0;
    pending_reply_active = 0;

    if (BiSS_SPI_Ch == BISS_SPI_CH_1)
	{
        ReadingStrEnc1.len = cmd_addr + 1U;
        ReadingStrEnc1.fifo_current_ptr = 0;
        ReadingStrEnc1.fifo_start_ptr = 0;
        ReadingStrEnc1.tol_cnt = 0;
    }
	else if (BiSS_SPI_Ch == BISS_SPI_CH_2)
	{
        ReadingStrEnc2.len = cmd_addr + 1U;
        ReadingStrEnc2.fifo_current_ptr = 0;
        ReadingStrEnc2.fifo_start_ptr = 0;
        ReadingStrEnc2.tol_cnt = 0;
    }
    queue_read_cnt = (queue_read_cnt + 1U) % QUEUE_SIZE;
    queue_cnt--;
    stream_mode = STREAM_SPI;
}

/*==========================================================================*/
/* Local functions: Command execution                                       */
/*==========================================================================*/

static uint8_t is_spi_ch_selection_allowed(uint8_t mode)
{
    switch (mode) 
	{
        case BISS_MODE_SPI_SPI:     return 1;
        case BISS_MODE_AB_UART:     return 1;
        case BISS_MODE_SPI_UART_IRS:return 0;
        case BISS_MODE_AB_SPI:      return 1;
        case BISS_MODE_DEFAULT_SPI: return 1;
        default:                    return 0;
    }
}

static void handle_run_command_state(void)
{
    if (queue_cnt == 0U)
	{
		return;
	}

    uint8_t        cmd_data_len = CommandQueue[queue_read_cnt].len;
    uint16_t       cmd_addr     = CommandQueue[queue_read_cnt].addr;
    UART_Command_t command      = CommandQueue[queue_read_cnt].cmd;
    uint8_t       *cmd_data     = CommandQueue[queue_read_cnt].data;

//    if (UART_CommandHasReply(command))
//	{
//        pending_reply_prepare(command, cmd_addr, cmd_data_len);
//    }

    switch (command)
	{
		case UART_COMMAND_WRITE_BANK:
			handle_write_bank_command(command, cmd_data_len, cmd_addr, cmd_data);
		break;

		case UART_COMMAND_CHANGE_MODE:
			if (cmd_data_len != 1U)
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_LEN_IS_NOT_CORRECT;
				abort_current_command(command);
			} 
			else if (IsBiSSReqBusy() == BISS_BUSY) 
			{
			}
			else 
			{
				handle_change_mode_command(cmd_data[0]);
				finalize_successful_command();
			}
		break;

		case UART_COMMAND_PAGE:
			if (cmd_data_len != 1U)
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_LEN_IS_NOT_CORRECT;
				abort_current_command(command);
			} 
			else 
			{
				handle_page_command(command, cmd_data[0]);
			}
		break;

		case UART_COMMAND_ENC1_POWER_OFF:
			if ((Current_Mode != BISS_MODE_DEFAULT_SPI) 
				|| (Current_Mode != BISS_MODE_SPI_UART_IRS)) 
			{
				EncoderPowerDisable();
				finalize_successful_command();
			}
			else
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_INVALID_MODE;
				abort_current_command(command);
			}
		break;

		case UART_COMMAND_ENC1_POWER_ON:
			if ((Current_Mode != BISS_MODE_DEFAULT_SPI)
				|| (Current_Mode != BISS_MODE_SPI_UART_IRS))
			{
				EncoderPowerEnable();
				finalize_successful_command();
			}
			else
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_INVALID_MODE;
				abort_current_command(command);
			}
		break;

		case UART_COMMAND_ENC2_POWER_OFF:
			EncoderSecondPowerDisable();
			finalize_successful_command();
		break;

		case UART_COMMAND_ENC2_POWER_ON:
			EncoderSecondPowerEnable();
			finalize_successful_command();
		break;

		case UART_COMMAND_SELECT_SPI_CH:
			if (cmd_data_len != 1U)
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_LEN_IS_NOT_CORRECT;
				abort_current_command(command);
			} 
			else 
			{
				if (!is_spi_ch_selection_allowed(Current_Mode))
				{
					uart_error_status.error_type = ERROR_TYPE_UART;
					uart_error_status.error_code = UART_ERROR_INVALID_MODE;
					abort_current_command(command);
				}
				else 
				{
					handle_select_spi_ch_command(cmd_data[0]);
					finalize_successful_command();
				}
			} 
		break;

		case UART_COMMAND_CHANGE_CH1_MODE:
			if (cmd_data_len != 1U) 
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_LEN_IS_NOT_CORRECT;
				abort_current_command(command);
			} 
			else 
			{
				if (Current_Mode != BISS_MODE_SPI_SPI)
				{
					uart_error_status.error_type = ERROR_TYPE_UART;
					uart_error_status.error_code = UART_ERROR_INVALID_MODE;
					abort_current_command(command);
				}
				else
				{
					handle_change_ch1_mode_command(cmd_data[0]);
					finalize_successful_command();
				}
			}
		break;

		case UART_COMMAND_WRITE_REG:
			if (Current_Mode == BISS_MODE_SPI_SPI)
			{
				handle_write_reg_command(cmd_data_len, cmd_addr, cmd_data, command);
			}
			else if ((Current_Mode == BISS_MODE_AB_UART) ||
					 (Current_Mode == BISS_MODE_AB_SPI) ||
					 (Current_Mode == BISS_MODE_DEFAULT_SPI))
			{
				if (BiSS_SPI_Ch == BISS_SPI_CH_2)
				{
					handle_write_reg_command(cmd_data_len, cmd_addr, cmd_data, command);
				}
				else
				{
					uart_error_status.error_type = ERROR_TYPE_UART;
					uart_error_status.error_code = UART_ERROR_INVALID_CHANNEL;
					abort_current_command(command);
				}
			}
			else
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_INVALID_MODE;
				abort_current_command(command);
			}
		break;

		case UART_COMMAND_READ_REG:
			if (Current_Mode == BISS_MODE_SPI_SPI)
			{
				handle_read_reg_command(cmd_data_len, cmd_addr, command);
			}
			else if ((Current_Mode == BISS_MODE_AB_UART) ||
					 (Current_Mode == BISS_MODE_AB_SPI) ||
					 (Current_Mode == BISS_MODE_DEFAULT_SPI))
			{
				if (BiSS_SPI_Ch == BISS_SPI_CH_2)
				{
					handle_read_reg_command(cmd_data_len, cmd_addr, command);
				}
				else
				{
					uart_error_status.error_type = ERROR_TYPE_UART;
					uart_error_status.error_code = UART_ERROR_INVALID_CHANNEL;
					abort_current_command(command);
				}
			}
			else
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_INVALID_MODE;
				abort_current_command(command);
			}
		break;

		case UART_COMMAND_WRITE_READ_ENC_USART2:
			if (Current_Mode == BISS_MODE_SPI_UART_IRS)
			{
				handle_write_read_enc_usart2_command(cmd_data_len, cmd_data, command);
				finalize_successful_command();
			}
			else
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_INVALID_MODE;
				abort_current_command(command);
			}
		break;

		case UART_COMMAND_READ_ANGLE_ENC_SPI_INSTANT:
			if (Current_Mode == BISS_MODE_SPI_SPI)
			{
				handle_read_angle_enc_spi_instant_command(cmd_data_len, command);
				finalize_successful_command();
			}
			else if ((Current_Mode == BISS_MODE_AB_UART) ||
					 (Current_Mode == BISS_MODE_AB_SPI) ||
					 (Current_Mode == BISS_MODE_DEFAULT_SPI))
			{
				if (BiSS_SPI_Ch == BISS_SPI_CH_2)
				{
					handle_read_angle_enc_spi_instant_command(cmd_data_len, command);
					finalize_successful_command();
				}
				else
				{
					uart_error_status.error_type = ERROR_TYPE_UART;
					uart_error_status.error_code = UART_ERROR_INVALID_CHANNEL;
					abort_current_command(command);
				}
			}
			else if (Current_Mode == BISS_MODE_SPI_UART_IRS)
			{
				if (BiSS_SPI_Ch == BISS_SPI_CH_1)
				{
					handle_read_angle_enc_spi_instant_command(cmd_data_len, command);
					finalize_successful_command();
				}
				else
				{
					uart_error_status.error_type = ERROR_TYPE_UART;
					uart_error_status.error_code = UART_ERROR_INVALID_CHANNEL;
					abort_current_command(command);
				}
			}
			else
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_INVALID_MODE;
				abort_current_command(command);
			}
		break;

		case UART_COMMAND_CHANGE_CURRENT_SENSOR_MODE:
			if (cmd_data_len != 1U)
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_LEN_IS_NOT_CORRECT;
				abort_current_command(command);
			}
			else 
			{
				handle_change_current_sensor_mode_command(cmd_data[0]);
				finalize_successful_command();
			}
		break;

		case UART_COMMAND_READ_ENC2_CURRENT:
			if (Current_Sensor_Mode != CURRENT_SENSOR_MODE_ENABLE)
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_INVALID_MODE;
				abort_current_command(command);
			}
			else
			{
				handle_read_enc2_current_command(cmd_data_len, command);
				finalize_successful_command();
			}
		break;

		case UART_COMMAND_READ_ANGLE_PACKET_ENC_SPI_INSTANT:
			if (Current_Mode == BISS_MODE_SPI_SPI)
			{
				handle_read_angle_packet_enc_spi_instant_command(cmd_data_len, command);
				finalize_successful_command();
			}
			else if ((Current_Mode == BISS_MODE_AB_SPI) ||
					 (Current_Mode == BISS_MODE_DEFAULT_SPI))
			{
				if (BiSS_SPI_Ch == BISS_SPI_CH_2)
				{
					handle_read_angle_packet_enc_spi_instant_command(cmd_data_len, command);
					finalize_successful_command();
				}
				else
				{
					uart_error_status.error_type = ERROR_TYPE_UART;
					uart_error_status.error_code = UART_ERROR_INVALID_CHANNEL;
					abort_current_command(command);
				}
			}
			else if (Current_Mode == BISS_MODE_SPI_UART_IRS)
			{
				if (BiSS_SPI_Ch == BISS_SPI_CH_1)
				{
					handle_read_angle_packet_enc_spi_instant_command(cmd_data_len, command);
					finalize_successful_command();
				}
				else
				{
					uart_error_status.error_type = ERROR_TYPE_UART;
					uart_error_status.error_code = UART_ERROR_INVALID_CHANNEL;
					abort_current_command(command);
				}
			}
			else
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_INVALID_MODE;
				abort_current_command(command);
			}
		break;

		case UART_COMMAND_READ_ANGLE_TWO_ENC_AB_SPI:
			if (Current_Mode == BISS_MODE_AB_SPI)
			{
				prepare_encoder_ab_spi_reading(cmd_addr, command);
			}
			else
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_INVALID_MODE;
				abort_current_command(command);
			}
		break;

		case UART_COMMAND_READ_ANGLE_TWO_ENC_AB_UART:
			if (Current_Mode == BISS_MODE_AB_UART)
			{
			    prepare_encoder_ab_uart_reading(cmd_addr, command);
			}
			else
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_INVALID_MODE;
				abort_current_command(command);
			}
		break;

		case UART_COMMAND_READ_ANGLE_TWO_ENC_SPI:
			if (Current_Mode == BISS_MODE_SPI_SPI)
			{
			    prepare_encoder_spi_spi_reading(cmd_addr, command);
			}
			else
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_INVALID_MODE;
				abort_current_command(command);
			}
		break;

		case UART_COMMAND_READ_ANGLE_ENC_SPI:
			prepare_encoder_spi_reading(cmd_addr, command);
		break;

		case UART_COMMAND_NRST:
			__disable_irq();
			__DSB();
			__ISB();
			NVIC_SystemReset();
		break;

		case UART_COMMAND_READ_UART_ERROR_STATUS:
			if (cmd_data_len != 3U) 
			{
				uart_error_status.error_type = ERROR_TYPE_UART;
				uart_error_status.error_code = UART_ERROR_LEN_IS_NOT_CORRECT;
				abort_current_command(command);
			} 
			else 
			{
				handle_read_uart_error_status(cmd_data_len, command);
				finalize_successful_command();
			}
		break;

		case UART_COMMAND_REBOOT_TO_BL:
			JumpToBootloader();
		break;

		default:
			uart_error_status.error_type = ERROR_TYPE_UART;
			uart_error_status.error_code = UART_ERROR_INVALID_CMD;
			abort_current_command(command);
		break;
    }
}

/*==========================================================================*/
/* Task RX, CMD, TX, STREAM                                                 */
/*==========================================================================*/

/*!
 * @brief Processes a received frame when the IDLE IRQ signals its end.
 *
 * Verifies the frame layout (length byte, CRC, minimal size) and
 * enqueues the parsed command. Frames that do not match the expected
 * length or fail CRC are discarded.
 *
 * @note Maximum frame size is UART_LINE_SIZE (133 bytes). Any frame
 *       longer than that is discarded.
 */
static void UART_RxTask(void)
{
	uint16_t frame_end = 0;
    uint16_t frame_len = 0;
	uint16_t part = 0;
	uint8_t crc = 0;
    uint8_t calc = 0;
    uint8_t crc_pos = 0;
	uint32_t primask = 0;
	uint16_t cmd_addr = 0;
	uint8_t cmd_data_len = 0;
	UART_Command_t command;
	UART_Error_Status_t parse_error;
	
	if (uart_rx_frame_ready == 0)
    {
        return;
    }
	
    primask = __get_PRIMASK();
    __disable_irq();
    frame_end = uart_rx_frame_end;
    uart_rx_frame_ready = 0;
    __set_PRIMASK(primask);

    frame_len = (uint16_t)((frame_end + RX_BUFFER_SIZE - dma_rx_cnt)
	                        % RX_BUFFER_SIZE);

    if (frame_len < (uint16_t)HEXLEN_ADR_CMD_CRC_LEN)
    {
        dma_rx_cnt = frame_end;
        return;
    }

    uart_length = usb_rx_buffer[dma_rx_cnt];
	if ((uart_length == 0) ||
		(uart_length > HEX_DATA_LEN))
	{	
		cmd_addr = (uint16_t)(
			((uint16_t)usb_rx_buffer[
				(dma_rx_cnt + 1U) % RX_BUFFER_SIZE] << 8U) |
			(uint16_t)usb_rx_buffer[
				(dma_rx_cnt + 2U) % RX_BUFFER_SIZE]);

		command = (UART_Command_t)usb_rx_buffer[
			(dma_rx_cnt + 3U) % RX_BUFFER_SIZE];
		
		uart_error_status.error_type = ERROR_TYPE_UART;
		uart_error_status.error_code =
			(uart_length == 0)
			? UART_ERROR_LEN_DATA_IS_ZERO
			: UART_ERROR_LEN_IS_NOT_CORRECT;
		uart_error_status.biss_fault_state = BiSSGetFaultState();
		parse_error = uart_error_status;
		UART_QueueErrorResponse(cmd_addr, command, parse_error);

		dma_rx_cnt = frame_end;
		return;
	}

    uart_expected_length = (uint32_t)uart_length + HEXLEN_ADR_CMD_CRC_LEN;

    if (uart_expected_length != (uint32_t)frame_len)
    {
		cmd_addr = (uint16_t)(
			((uint16_t)usb_rx_buffer[
				(dma_rx_cnt + 1U) % RX_BUFFER_SIZE] << 8U) |
			(uint16_t)usb_rx_buffer[
				(dma_rx_cnt + 2U) % RX_BUFFER_SIZE]);

		command = (UART_Command_t)usb_rx_buffer[
			(dma_rx_cnt + 3U) % RX_BUFFER_SIZE];
		
        uart_error_status.error_type = ERROR_TYPE_UART;
		uart_error_status.error_code = UART_ERROR_LEN_IS_NOT_CORRECT;
		uart_error_status.biss_fault_state = BiSSGetFaultState();
		parse_error = uart_error_status;
		UART_QueueErrorResponse(cmd_addr, command, parse_error);

        dma_rx_cnt = frame_end;
        return;
    }

    crc_pos = (uint8_t)(((uint32_t)dma_rx_cnt + uart_expected_length - 1U)
	                      % RX_BUFFER_SIZE);
    crc = usb_rx_buffer[crc_pos];
    calc = CalculateCRCCircularBuffer((uint8_t*)usb_rx_buffer,
	                                  (uint16_t)RX_BUFFER_SIZE,
                                      (uint8_t)dma_rx_cnt, 
	                                  (uint8_t)(uart_expected_length - 1U));

    if (crc != calc)
	{
		cmd_addr = (uint16_t)(
			((uint16_t)usb_rx_buffer[
				(dma_rx_cnt + 1U) % RX_BUFFER_SIZE] << 8U) |
			(uint16_t)usb_rx_buffer[
				(dma_rx_cnt + 2U) % RX_BUFFER_SIZE]);

		command = (UART_Command_t)usb_rx_buffer[
			(dma_rx_cnt + 3U) % RX_BUFFER_SIZE];
		
        uart_error_status.error_type = ERROR_TYPE_UART;
		uart_error_status.error_code = UART_ERROR_CRC;
		uart_error_status.biss_fault_state = BiSSGetFaultState();
		parse_error = uart_error_status;
		UART_QueueErrorResponse(cmd_addr, command, parse_error);
//        dma_rx_cnt = (uint16_t)((dma_rx_cnt + uart_expected_length)
//		                         % RX_BUFFER_SIZE);
		dma_rx_cnt = frame_end;
        return;
    }

    memset(hex_line_buffer, 0, UART_LINE_SIZE);

    if (((uint32_t)dma_rx_cnt + uart_expected_length) <= (uint32_t)RX_BUFFER_SIZE)
	{
        memcpy(hex_line_buffer, (uint8_t*)&usb_rx_buffer[dma_rx_cnt],
		       uart_expected_length);
    }
	else
	{
        part = (uint16_t)((uint32_t)RX_BUFFER_SIZE - (uint32_t)dma_rx_cnt);
        memcpy(hex_line_buffer, (uint8_t*)&usb_rx_buffer[dma_rx_cnt], part);
        memcpy(hex_line_buffer + part, (uint8_t*)usb_rx_buffer,
		       uart_expected_length - part);
    }

    dma_rx_cnt = (uint16_t)((dma_rx_cnt + uart_expected_length)
	                         % RX_BUFFER_SIZE);

    cmd_data_len = hex_line_buffer[0];
    cmd_addr = (uint16_t)(((uint16_t)hex_line_buffer[1] << 8)
                                                 | (uint16_t)hex_line_buffer[2]);
    command = (UART_Command_t)hex_line_buffer[3];
    uint8_t *cmd_data = &hex_line_buffer[4];

    if (EnqueueCommand(command, cmd_addr, cmd_data_len, cmd_data) != QUEUE_OK)
	{
		uart_error_status.error_type = ERROR_TYPE_UART;
		uart_error_status.error_code = UART_ERROR_QUEUE_FULL;
		uart_error_status.biss_fault_state = BiSSGetFaultState();

		UART_Error_Status_t err = uart_error_status;
		UART_QueueErrorResponse(cmd_addr, command, err);
    }
}

static void UART_CmdTask(void)
{
    if (pending_reply_active)
	{
        BiSSExternalState_t st = IsBiSSReqBusy();
        if (st == BISS_READ_FINISHED || st == BISS_WRITE_FINISHED)
		{
            pending_reply_finish();
            BiSSResetExternalState();
        }
		else if (st == BISS_FAULT)
		{
			uart_error_status.error_type = ERROR_TYPE_BISS;
            uart_error_status.error_code = UART_ERROR_BISS;
			abort_current_command(pending_reply.cmd);
//			pending_reply_finish();
            BiSSResetExternalState();
        }
        return;
    }
	
	if (write_pending)
    {
        BiSSExternalState_t st = IsBiSSReqBusy();

        if (st == BISS_WRITE_FINISHED)
        {
            write_pending = 0;
            retry_cnt = 0;

            BiSSResetExternalState();
            finalize_successful_command();
        }
        else if (st == BISS_FAULT)
        {
			uart_error_status.error_type = ERROR_TYPE_BISS;
            uart_error_status.error_code = UART_ERROR_BISS_WRITE_FAULT;
            uart_error_status.biss_fault_state = BiSSGetFaultState();

            write_pending = 0;
			retry_cnt = 0;

            BiSSResetExternalState();
            abort_current_command(pending_write_command);
        }

        return;
    }

    if ((stream_mode != STREAM_NONE) && (queue_cnt > 0U))
	{
		UART_CancelStream();
	}

    if (queue_cnt > 0U)
	{
        handle_run_command_state();
    }
}

static void UART_TxTask(void)
{
    if (LL_DMA_IsEnabledChannel(DMA_LPUART_TX))
	{
		return;
	}
    if (TxQueue_IsEmpty())
	{
		return;
	}

    UartTxStr_t frame;
    if (TxQueue_Pop(&frame))
	{
        UART_TransmitDMA(&frame);
    }
}

/*==========================================================================*/
/* Reading angle: preparation of the next frame                             */
/*==========================================================================*/

static uint8_t stream_build_frame(UartTxStr_t *out)
{
    *out = pending_reply;
    uint8_t TxBufCnt = 0;

    switch (stream_mode)
	{
		case STREAM_AB_SPI:
		case STREAM_AB_UART:
			if (ReadingStrEnc2.len == 0) 
			{
				return 0;
			}
			{
				AngleData_t a2 = getAngle2();
				AngleDataRenishaw_t a1 = getAngleRenishaw();
				if (a2.time_of_life_counter == ReadingStrEnc2.tol_cnt) 
				{
					return 0;
				}
				ReadingStrEnc2.tol_cnt = a2.time_of_life_counter;
				ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.fifo_current_ptr] = a2;
				ReadingStrEnc2.fifo_current_ptr++;
				ReadingStrRenishaw.AngleFIFO[ReadingStrRenishaw.fifo_current_ptr] = a1;
				ReadingStrRenishaw.fifo_current_ptr++;

				if ((((uint16_t)ReadingStrEnc2.fifo_current_ptr + 256U - ReadingStrEnc2.fifo_start_ptr) & 0xFFU)
					< UART_ANGLE_TWO_ENC_AB_UART_LEN)
				{
					return 0;
				}
				while (ReadingStrEnc2.fifo_start_ptr != ReadingStrEnc2.fifo_current_ptr)
				{
					*((AngleData_t*)&out->Buf[TxBufCnt]) = ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.fifo_start_ptr];
					TxBufCnt += ANGLE_DATA_SIZE;
					*((AngleDataRenishaw_t*)&out->Buf[TxBufCnt]) = ReadingStrRenishaw.AngleFIFO[ReadingStrRenishaw.fifo_start_ptr];
					TxBufCnt += RENISHAW_ANGLE_DATA_SIZE;
					ReadingStrEnc2.fifo_start_ptr++;
					ReadingStrRenishaw.fifo_start_ptr++;
				}
				ReadingStrEnc2.len--;
				ReadingStrRenishaw.len--;
				out->adr_h = (ReadingStrEnc2.len >> 8U) & 0xFFU;
				out->adr_l = ReadingStrEnc2.len & 0xFFU;
			}
		return 1;

		case STREAM_SPI_SPI:
			if (ReadingStrEnc2.len == 0) 
			{
				return 0;
			}
			{
				AngleData_t a1 = getAngle1();
				AngleData_t a2 = getAngle2();
				if ((a2.time_of_life_counter == ReadingStrEnc2.tol_cnt) &&
					(a1.time_of_life_counter == ReadingStrEnc1.tol_cnt))
				{
					return 0;
				}
				ReadingStrEnc1.tol_cnt = a1.time_of_life_counter;
				ReadingStrEnc1.AngleFIFO[ReadingStrEnc1.fifo_current_ptr] = a1;
				ReadingStrEnc1.fifo_current_ptr++;
				ReadingStrEnc2.tol_cnt = a2.time_of_life_counter;
				ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.fifo_current_ptr] = a2;
				ReadingStrEnc2.fifo_current_ptr++;

				if ((((uint16_t)ReadingStrEnc2.fifo_current_ptr + 256U - ReadingStrEnc2.fifo_start_ptr) & 0xFFU)
					< UART_ANGLE_TWO_ENC_LEN)
				{
					return 0;
				}
				while (ReadingStrEnc2.fifo_start_ptr != ReadingStrEnc2.fifo_current_ptr)
				{
					*((AngleData_t*)&out->Buf[TxBufCnt]) = ReadingStrEnc1.AngleFIFO[ReadingStrEnc1.fifo_start_ptr];
					TxBufCnt += ANGLE_DATA_SIZE;
					*((AngleData_t*)&out->Buf[TxBufCnt]) = ReadingStrEnc2.AngleFIFO[ReadingStrEnc2.fifo_start_ptr];
					TxBufCnt += ANGLE_DATA_SIZE;
					ReadingStrEnc1.fifo_start_ptr++;
					ReadingStrEnc2.fifo_start_ptr++;
				}
				ReadingStrEnc1.len--;
				ReadingStrEnc2.len--;
				out->adr_h = (ReadingStrEnc2.len >> 8U) & 0xFFU;
				out->adr_l = ReadingStrEnc2.len & 0xFFU;
			}
		return 1;

		case STREAM_SPI:
			{
				volatile uint16_t *len_ptr = (BiSS_SPI_Ch == BISS_SPI_CH_1)
											 ? &ReadingStrEnc1.len : &ReadingStrEnc2.len;
				volatile AngleData_t *fifo = (BiSS_SPI_Ch == BISS_SPI_CH_1)
											 ? ReadingStrEnc1.AngleFIFO : ReadingStrEnc2.AngleFIFO;
				volatile uint8_t *start = (BiSS_SPI_Ch == BISS_SPI_CH_1)
										  ? &ReadingStrEnc1.fifo_start_ptr : &ReadingStrEnc2.fifo_start_ptr;
				volatile uint8_t *cur = (BiSS_SPI_Ch == BISS_SPI_CH_1)
										? &ReadingStrEnc1.fifo_current_ptr : &ReadingStrEnc2.fifo_current_ptr;
				volatile uint8_t *tol = (BiSS_SPI_Ch == BISS_SPI_CH_1)
										? &ReadingStrEnc1.tol_cnt : &ReadingStrEnc2.tol_cnt;

				if (*len_ptr == 0)
				{
					return 0;
				}
				AngleData_t a = (BiSS_SPI_Ch == BISS_SPI_CH_1) ? getAngle1() : getAngle2();
				if (a.time_of_life_counter == *tol)
				{
					return 0;
				}
				*tol = a.time_of_life_counter;
				fifo[*cur] = a;
				(*cur)++;
				if ((((uint16_t)(*cur) + 256U - *start) & 0xFFU) < UART_ANGLE_LEN)
				{
					return 0;
				}
				while (*start != *cur)
				{
					*((AngleData_t*)&out->Buf[TxBufCnt]) = fifo[*start];
					TxBufCnt += ANGLE_DATA_SIZE;
					(*start)++;
				}
				(*len_ptr)--;
				out->adr_h = (*len_ptr >> 8U) & 0xFFU;
				out->adr_l = *len_ptr & 0xFFU;
			}
		return 1;

		default:
		return 0;
    }
}

static void UART_StreamTask(void)
{
    if (stream_mode == STREAM_NONE)
	{
		return;
	}

    uint8_t finished = 0;
    switch (stream_mode)
	{
		case STREAM_AB_SPI:
		case STREAM_AB_UART:
			finished = (ReadingStrEnc2.len == 0);
		break;
		
		case STREAM_SPI_SPI:
			finished = (ReadingStrEnc2.len == 0);
		break;

		case STREAM_SPI:
			finished = (BiSS_SPI_Ch == BISS_SPI_CH_1) ? 
		               (ReadingStrEnc1.len == 0) : 
		               (ReadingStrEnc2.len == 0);
		break;

		default:
		break;
    }

    if (finished)
	{
        stream_mode = STREAM_NONE;
        pending_reply_active = 0;
        return;
    }

    if (tx_queue.count >= TX_QUEUE_SIZE)
	{
		return;
	}

    UartTxStr_t frame;
    if (stream_build_frame(&frame))
	{
        frame.cmd = pending_reply.cmd;
        frame.len = pending_reply.len;

		if (!TxQueue_PushTagged(&frame, 1U))
		{
			uart_error_status.error_type = ERROR_TYPE_UART;
			uart_error_status.error_code = UART_ERROR_QUEUE_FULL;
		}
    }
}

/*==========================================================================*/
/* Main Task UART                                                           */
/*==========================================================================*/

void UartTask(void)
{
    UART_RxTask();
    UART_CmdTask();
    UART_StreamTask();
    UART_TxTask();
}

/*==========================================================================*/
/* CRC and DMA TX                                                           */
/*==========================================================================*/

static uint8_t CalculateCRC(uint8_t *data, uint32_t length)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < length; i++)
	{
		sum += data[i];
	}
    uint8_t lsb = sum & 0xFFU;
    return (uint8_t)(~lsb + 1U);
}

static uint8_t CalculateCRCCircularBuffer(uint8_t *buffer, uint16_t buffer_size,
                                          uint8_t start_index, uint8_t length)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
	{
        uint8_t idx = (start_index + i) % buffer_size;
        sum += buffer[idx];
    }
    uint8_t lsb = sum & 0xFFU;
    return (uint8_t)(~lsb + 1U);
}

static void UART_TransmitDMA(UartTxStr_t *TxStr)
{
    uint8_t size = TxStr->len;
    if (size > TX_BUFFER_SIZE)
	{
		size = TX_BUFFER_SIZE;
	}

    LL_DMA_DisableChannel(DMA_LPUART_TX);
    LL_DMA_SetDataLength(DMA_LPUART_TX, size + TX_DMA_OVERHEAD);
    memcpy(usb_tx_buffer, TxStr, size + TX_DMA_OVERHEAD - 1U);
    if (usb_tx_buffer[3] != (uint8_t)UART_CMD_ERROR_FRAME)
	{
		usb_tx_buffer[3] += TX_CMD_REPLY_FLAG;
	}
    uint8_t crc = CalculateCRC(usb_tx_buffer, size + TX_DMA_OVERHEAD - 1U);
    usb_tx_buffer[size + TX_DMA_OVERHEAD - 1U] = crc;
    LL_DMA_EnableChannel(DMA_LPUART_TX);
}

/*==========================================================================*/
/* Init                                                                     */
/*==========================================================================*/

static void LPUART1_UART_Init(void)
{
	LL_LPUART_InitTypeDef LPUART_InitStruct = {0};

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_PCLK1);

	/* Peripheral clock enable */
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_LPUART1);

	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	/**LPUART1 GPIO Configuration
	PA2   ------> LPUART1_TX
	PA3   ------> LPUART1_RX
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_12;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* LPUART1 DMA Init */

	/* LPUART1_RX Init */
	LL_DMA_SetPeriphRequest(DMA_LPUART, DMA_LPUART_RX_CHANNEL, LL_DMAMUX_REQ_LPUART1_RX);
	LL_DMA_SetDataTransferDirection(DMA_LPUART, DMA_LPUART_RX_CHANNEL, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetChannelPriorityLevel(DMA_LPUART, DMA_LPUART_RX_CHANNEL, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA_LPUART, DMA_LPUART_RX_CHANNEL, LL_DMA_MODE_CIRCULAR);
	LL_DMA_SetPeriphIncMode(DMA_LPUART, DMA_LPUART_RX_CHANNEL, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA_LPUART, DMA_LPUART_RX_CHANNEL, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA_LPUART, DMA_LPUART_RX_CHANNEL, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA_LPUART, DMA_LPUART_RX_CHANNEL, LL_DMA_MDATAALIGN_BYTE);

	/* LPUART1_TX Init */
	LL_DMA_SetPeriphRequest(DMA_LPUART, DMA_LPUART_TX_CHANNEL, LL_DMAMUX_REQ_LPUART1_TX);
	LL_DMA_SetDataTransferDirection(DMA_LPUART, DMA_LPUART_TX_CHANNEL, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetChannelPriorityLevel(DMA_LPUART, DMA_LPUART_TX_CHANNEL, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA_LPUART, DMA_LPUART_TX_CHANNEL, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA_LPUART, DMA_LPUART_TX_CHANNEL, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA_LPUART, DMA_LPUART_TX_CHANNEL, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA_LPUART, DMA_LPUART_TX_CHANNEL, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA_LPUART, DMA_LPUART_TX_CHANNEL, LL_DMA_MDATAALIGN_BYTE);

	LPUART_InitStruct.PrescalerValue = LL_LPUART_PRESCALER_DIV1;
	LPUART_InitStruct.BaudRate = 12000000;
	LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
	LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
	LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
	LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX_RX;
	LPUART_InitStruct.HardwareFlowControl = LL_LPUART_HWCONTROL_NONE;
	LL_LPUART_Init(LPUART, &LPUART_InitStruct);
	LL_LPUART_SetTXFIFOThreshold(LPUART, LL_LPUART_FIFOTHRESHOLD_1_8);
	LL_LPUART_SetRXFIFOThreshold(LPUART, LL_LPUART_FIFOTHRESHOLD_1_8);
	LL_LPUART_DisableFIFO(LPUART);

	/* Enable LPUART1 */
	LL_LPUART_Enable(LPUART);

	/* Polling LPUART1 initialisation */
	while((!(LL_LPUART_IsActiveFlag_TEACK(LPUART))) || (!(LL_LPUART_IsActiveFlag_REACK(LPUART))))
	{
	}
}

static void LPUART1_Config(void)
{
	LL_LPUART_Disable(LPUART);

    LL_DMA_SetMemoryAddress(DMA_LPUART_RX, (uint32_t)usb_rx_buffer);
    LL_DMA_SetPeriphAddress(DMA_LPUART_RX, (uint32_t)&LPUART->RDR);
    LL_DMA_SetMemoryAddress(DMA_LPUART_TX, (uint32_t)usb_tx_buffer);
    LL_DMA_SetPeriphAddress(DMA_LPUART_TX, (uint32_t)&LPUART->TDR);
    LL_DMA_SetDataLength(DMA_LPUART_RX, RX_BUFFER_SIZE);

    LL_LPUART_EnableDMAReq_RX(LPUART);
    LL_LPUART_EnableDMAReq_TX(LPUART);
    LL_DMA_EnableChannel(DMA_LPUART_RX);
	
	LL_LPUART_EnableIT_IDLE(LPUART);
	NVIC_SetPriority(LPUART_RX_IRQ,
                 NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1U, 0U));
	NVIC_EnableIRQ(LPUART_RX_IRQ);
	
	LL_DMA_EnableIT_TC(DMA_LPUART_TX);
	NVIC_SetPriority(DMA_LPUART_TX_IRQ,
                 NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1U, 0U));
    NVIC_EnableIRQ(DMA_LPUART_TX_IRQ);

	LL_LPUART_Enable(LPUART);

    (void)memset(&tx_queue, 0, sizeof(tx_queue));
    
	pending_reply_active = 0;
    stream_mode = STREAM_NONE;
	uart_rx_frame_ready = 0;
    uart_rx_frame_end = 0;
}

void LPUART_Init(void)
{
	LPUART1_UART_Init();
	LPUART1_Config();
}

/*==========================================================================*/
/* LPUART1 IDLE IRQ handler                                                 */
/*==========================================================================*/

/*!
 * @brief LPUART1 interrupt handler.
 *
 * Handles the IDLE line event, which signals the end of an incoming
 * frame. The DMA write position is captured at this moment.
 */
void LPUART_RX_IRQHandler(void)
{
    if (LL_LPUART_IsActiveFlag_IDLE(LPUART1))
    {
		LL_LPUART_ClearFlag_IDLE(LPUART);

        uint16_t current_pos = uart_get_dma_write_pos();
        uint16_t frame_len = (current_pos + RX_BUFFER_SIZE - dma_rx_cnt) % RX_BUFFER_SIZE;

        if (frame_len >= HEXLEN_ADR_CMD_CRC_LEN) 
        {
            uint8_t expected = usb_rx_buffer[dma_rx_cnt] + HEXLEN_ADR_CMD_CRC_LEN;
            if (frame_len >= expected) 
            {
                uart_rx_frame_end = current_pos;
                uart_rx_frame_ready = 1U;
            }
        }
    }
}

void DMA_LPUART_TX_IRQHandle(void)
{
	if (LL_DMA_IsActiveFlag_TC2(DMA_LPUART))
    {
		LL_DMA_ClearFlag_TC2(DMA_LPUART);
		LL_DMA_DisableChannel(DMA_LPUART_TX);
	}
}

/*==========================================================================*/
/* Command queues                                                           */
/*==========================================================================*/

static QUEUE_Status_t EnqueueCommand(UART_Command_t cmd, uint16_t addr, uint8_t len, uint8_t *data)
{
    if (queue_cnt >= QUEUE_SIZE)
	{
		return QUEUE_FULL;
	}
    if (len > HEX_DATA_LEN)
	{
		len = HEX_DATA_LEN;
	}

    CommandQueue[queue_write_cnt].len  = len;
    CommandQueue[queue_write_cnt].addr = addr;
    CommandQueue[queue_write_cnt].cmd  = cmd;
    memcpy(CommandQueue[queue_write_cnt].data, data, len);
    queue_write_cnt = (queue_write_cnt + 1U) % QUEUE_SIZE;
    queue_cnt++;
    return QUEUE_OK;
}

static QUEUE_Status_t EnqueueCommandToBegining(UART_Command_t cmd, uint16_t addr, uint8_t len, uint8_t *data)
{
    if (queue_cnt >= QUEUE_SIZE)
	{
		return QUEUE_FULL;
	}
    if (len > HEX_DATA_LEN)
	{
		len = HEX_DATA_LEN;
	}

    queue_read_cnt = (queue_read_cnt + QUEUE_SIZE - 1U) % QUEUE_SIZE;
    CommandQueue[queue_read_cnt].len  = len;
    CommandQueue[queue_read_cnt].addr = addr;
    CommandQueue[queue_read_cnt].cmd  = cmd;
    memcpy(CommandQueue[queue_read_cnt].data, data, len);
    queue_cnt++;
    return QUEUE_OK;
}

/*==========================================================================*/
/* TAMP / Bootloader                                                        */
/*==========================================================================*/

void TAMP_Init(void)
{
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

void Stay_in_FW_Config(void)
{
    ClearTampFlag(TAMP_FLAGS_STAY_BL);
    SetTampFlag(TAMP_FLAGS_STAY_MAIN_FW);
}

void JumpToBootloader(void)
{
    ClearTampFlag(TAMP_FLAGS_STAY_MAIN_FW);
    SetTampFlag(TAMP_FLAGS_STAY_BL);
    TAMP_DeInit();
    __disable_irq();
    __DSB();
    __ISB();
    NVIC_SystemReset();
}