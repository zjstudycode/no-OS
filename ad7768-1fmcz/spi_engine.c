/***************************************************************************//**
 *   @file   spi_engine.c
 *   @brief  Implementation of SPI Engine Driver.
 *   @author ADI
********************************************************************************
 * Copyright 2019(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sleep.h>
#include <xil_io.h>
#include <xscugic.h>
#include <xparameters.h>
#include <sleep.h>
#include "xil_printf.h"
#include "spi.h"
#include "spi_engine.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))



static inline struct spi_init_param_extra *cast_to_extra_init(void* void_param)
{
	return (struct spi_init_param_extra*)void_param;
}

static inline void *cast_from_extra_init(struct spi_init_param_extra *eng_param)
{
	return (uint8_t *)eng_param;
}

static inline struct spi_desc_extra *cast_to_extra_desc(void* void_desc)
{
	return (struct spi_desc_extra*)void_desc;
}

static inline void *cast_from_extra_desc(struct spi_desc_extra *eng_desc)
{
	return (uint8_t *)eng_desc;
}

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/

/***************************************************************************//**
* @brief spi_eng_write
*******************************************************************************/
int32_t spi_eng_write(spi_desc_extra *desc,
		      uint32_t reg_addr,
		      uint32_t ui32data)
{
	Xil_Out32((desc->spi_baseaddr + reg_addr), ui32data);

	return 0;
}

/***************************************************************************//**
* @brief spi_eng_read
*******************************************************************************/
int32_t spi_eng_read(spi_desc_extra *desc,
		     uint32_t reg_addr,
		     uint32_t *reg_data)
{
	*reg_data = Xil_In32((desc->spi_baseaddr + reg_addr));

	return 0;
}

/***************************************************************************//**
* @brief spi_eng_dma_write
*******************************************************************************/
int32_t spi_eng_dma_write(spi_desc_extra *desc,
			  uint32_t reg_addr,
			  uint32_t reg_data)
{
	Xil_Out32((desc->spi_offload_tx_dma_baseaddr + reg_addr), reg_data);

	return 0;
}

/***************************************************************************//**
* @brief spi_eng_dma_read
*******************************************************************************/
int32_t spi_eng_dma_read(spi_desc_extra *desc,
			 uint32_t reg_addr,
			 uint32_t *reg_data)
{
	*reg_data = Xil_In32((desc->spi_offload_rx_dma_baseaddr + reg_addr));

	return 0;
}

void spi_set_transfer_length(spi_desc*desc, uint8_t data_length)
{
	spi_desc_extra		*desc_extra;

	desc_extra = cast_to_extra_desc(desc->extra);
	if (data_length > desc_extra->max_data_width)
		desc_extra->data_width = desc_extra->max_data_width;
	else
		desc_extra->data_width = data_length;

	desc->extra = cast_from_extra_desc(desc_extra);
}

/***************************************************************************//**
* @brief get_words_number
*******************************************************************************/
uint8_t spi_get_words_number(spi_desc_extra *desc, uint8_t bytes_number)
{
	uint8_t xfer_word_len, xfer_words_number;

	/*
	 * Each spi engine transaction equals data_width bytes
	 */

 	xfer_word_len = desc->data_width / 8;
	xfer_words_number = bytes_number / xfer_word_len;

	if ((bytes_number % xfer_word_len) != 0)
		xfer_words_number++;

	return xfer_words_number;
}

/***************************************************************************//**
* @brief get_words_number
*******************************************************************************/
uint8_t spi_get_word_lenght(spi_desc_extra *desc)
{
	return desc->data_width / 8;
}

/***************************************************************************//**
* @brief check_dma_config
*******************************************************************************/
uint8_t spi_check_dma_config(spi_desc_extra *desc,
			     uint8_t rx,
			     uint8_t tx)
{
	if(desc->offload_configured) {
		if(rx && !desc->spi_offload_rx_support_en)
			return 0;

		if(tx && !desc->spi_offload_tx_support_en)
			return 0;
	}
	return 1;
}

/***************************************************************************//**
* @brief get_sleep_div
*******************************************************************************/
uint32_t spi_get_sleep_div(spi_desc *desc,
			   uint32_t sleep_time_ns)
{
	uint32_t sleep_div = 0;
	spi_desc_extra		*desc_extra;

	desc_extra = cast_to_extra_desc(desc->extra);

	sleep_div = (desc->max_speed_hz / 1000000 * sleep_time_ns / 1000) /
		    ((desc_extra->clk_div + 1) * 2) - 1;

	return sleep_div;
}

/***************************************************************************//**
* @brief spi_eng_program_add_cmd
*******************************************************************************/
void spi_eng_program_add_cmd(spi_eng_transfer_fifo *xfer,
			     uint16_t cmd)
{
	xfer->cmd_fifo[xfer->cmd_fifo_len] = cmd;
	xfer->cmd_fifo_len++;
}

/***************************************************************************//**
* @brief spi_eng_gen_transfer
*******************************************************************************/
int32_t spi_eng_gen_transfer(spi_desc_extra *desc,
			     spi_eng_transfer_fifo *xfer,
			     bool write,
			     bool read,
			     uint8_t bytes_number)
{
	uint8_t words_number;

	words_number = spi_get_words_number(desc, bytes_number);

	spi_eng_program_add_cmd(xfer,
				SPI_ENGINE_CMD_TRANSFER(write,
						read,
						words_number - 1));
	return 0;
}

/***************************************************************************//**
* @brief spi_eng_gen_cs
*******************************************************************************/
void spi_eng_gen_cs(spi_desc *desc,
		    spi_eng_transfer_fifo *xfer,
		    bool assert)
{
	uint8_t mask = 0xff;
	spi_desc_extra		*desc_extra;

	desc_extra = cast_to_extra_desc(desc->extra);
	if (!assert)
		mask ^= BIT(desc->chip_select);

	spi_eng_program_add_cmd(xfer, SPI_ENGINE_CMD_ASSERT(desc_extra->cs_delay, mask));
}

/***************************************************************************//**
* @brief spi_gen_sleep_ns
*******************************************************************************/
void spi_gen_sleep_ns(spi_desc *desc,
		      spi_eng_transfer_fifo *xfer,
		      uint32_t sleep_time_ns)
{
	uint32_t sleep_div;

	sleep_div = spi_get_sleep_div(desc, sleep_time_ns); // default 5us
	// Wait for the device to do the conversion
	spi_eng_program_add_cmd(xfer,
				SPI_ENGINE_CMD_SLEEP(sleep_div));
}

/***************************************************************************//**
* @brief spi_eng_add_user_cmd
*******************************************************************************/
void spi_eng_add_user_cmd(spi_desc *desc,
			  spi_eng_transfer_fifo *xfer,
			  uint32_t cmd)
{
	uint32_t cmd_msk = (0xF << 28);
	uint32_t param_msk = cmd & (~cmd_msk);
	uint32_t command;
	uint16_t param;
	spi_desc_extra		*desc_extra;

	desc_extra = cast_to_extra_desc(desc->extra);

	command = (cmd & cmd_msk);
	param = (cmd & param_msk);

	switch(command) {
	case CS_DEASSERT:
		spi_eng_gen_cs(desc, xfer, false); // reset chip select
		break;

	case CS_ASSERT:
		spi_eng_gen_cs(desc, xfer, true); // set chip select
		break;

	case SLEEP_CMD:
		spi_gen_sleep_ns(desc, xfer, param); // Sleep
		break;

	case TRANSFER_R_CMD:
		if(spi_check_dma_config(desc_extra, 1, 0)) {
			spi_eng_gen_transfer(desc_extra, xfer, false, true, param); // read
			desc_extra->rx_length = param;
		} else {
			printf("%s: DMA Rx not configured.\n", __func__);
			desc_extra->rx_length = 0;
		}

		break;

	case TRANSFER_W_CMD:
		if(spi_check_dma_config(desc_extra, 0, 1)) {
			spi_eng_gen_transfer(desc_extra, xfer, true, false, param); // write
			desc_extra->tx_length = param;
		} else {
			printf("%s: DMA Tx not configured.\n", __func__);
			desc_extra->tx_length = 0;
		}
		break;

	case TRANSFER_R_W_CMD:
		if(spi_check_dma_config(desc_extra, 1, 1)) {
			spi_eng_gen_transfer(desc_extra, xfer, true, true, param); // read and write
			desc_extra->tx_length = param;
			desc_extra->rx_length = param;
		} else {
			printf("%s: DMA Rx and Tx not configured.\n", __func__);
			desc_extra->tx_length = 0;
			desc_extra->rx_length = 0;
		}
		break;

	default:
		break;
	}
}

/***************************************************************************//**
* @brief spi_eng_compile_message
*******************************************************************************/
int32_t spi_eng_compile_message(spi_desc *desc,
				spi_eng_msg *msg,
				spi_eng_transfer_fifo *xfer)
{
	uint32_t i, n = 0;
	spi_desc_extra		*desc_extra;

	desc_extra = cast_to_extra_desc(desc->extra);

	n = msg->msg_cmd_len;

	// configure prescale
	spi_eng_program_add_cmd(xfer,
				SPI_ENGINE_CMD_WRITE(SPI_ENGINE_CMD_REG_CLK_DIV,
						desc_extra->clk_div));
	// SPI configuration (3W/CPOL/CPHA)
	spi_eng_program_add_cmd(xfer,
				SPI_ENGINE_CMD_WRITE(SPI_ENGINE_CMD_REG_CONFIG,
						desc->mode));

	// Data transfer length
	spi_eng_program_add_cmd(xfer,
				SPI_ENGINE_CMD_WRITE(SPI_ENGINE_CMD_DATA_TRANSFER_LEN,
						desc_extra->data_width));

	// SYNC to signal transfer beginning
	spi_eng_program_add_cmd(xfer,
				SPI_ENGINE_CMD_SYNC(SPI_ENGINE_SYNC_TRANSFER_BEGIN));

	for (i = 0; i < n; i++)
		spi_eng_add_user_cmd(desc, xfer, msg->spi_msg_cmds[i]);

	// SYNC to signal transfer end
	spi_eng_program_add_cmd(xfer,
				SPI_ENGINE_CMD_SYNC(SPI_ENGINE_SYNC_TRANSFER_END));

	return 0;
}

/***************************************************************************//**
* @brief spi_eng_transfer_message
*******************************************************************************/
int32_t spi_eng_transfer_message(spi_desc *desc, spi_eng_msg *msg)
{
	spi_eng_transfer_fifo *xfer;
	uint32_t size;
	uint32_t i;
	uint32_t data;
	uint32_t sync_id;
	spi_desc_extra		*desc_extra;

	desc_extra = cast_to_extra_desc(desc->extra);

	size = sizeof(*xfer->cmd_fifo) * (msg->msg_cmd_len + 3);

	xfer = (spi_eng_transfer_fifo *)malloc(sizeof(*xfer) + size);
	if (!xfer)
		return -1;

	xfer->cmd_fifo_len = 0;
	spi_eng_compile_message(desc, msg, xfer);

	// CMD FIFO
	for (i = 0; i < xfer->cmd_fifo_len; i++)
		spi_eng_write(desc_extra, SPI_ENGINE_REG_CMD_FIFO, xfer->cmd_fifo[i]);

	/*
	 * On each spi write command, one word is transfered. Typically 16 bits
	 * tx_length = param is deduced from TRANSFER_W(param)
	 */
	for(i = 0; i < desc_extra->tx_length; i++)
		spi_eng_write(desc_extra, SPI_ENGINE_REG_SDO_DATA_FIFO, msg->tx_buf[i]);


	/*
	 *	Wait for all the transactions to finish
	 *
	 */

	do spi_eng_read(desc_extra, SPI_ENGINE_REG_SYNC_ID, &sync_id);
	while(sync_id != SPI_ENGINE_SYNC_TRANSFER_END);

	/*
	 * On each spi read command, one word is transfered. Typically 16 bits.
	 * rx_length = param is deduced from TRANSFER_R(param)
	 */
	for(i = 0; i < desc_extra->rx_length; i++) {
		spi_eng_read(desc_extra, SPI_ENGINE_REG_SDI_DATA_FIFO, &data);
		msg->rx_buf[i] = data;
	}

	free(xfer);

	return 0;
}

/***************************************************************************//**
* @brief spi_eng_init
*******************************************************************************/

int32_t spi_init(struct spi_desc **desc,
		 const struct spi_init_param *param)
{
	struct spi_desc		*descriptor;
	struct spi_desc_extra		*desc_extra;
	struct spi_init_param_extra	*init_extra;
	uint32_t        	data_width;

	descriptor = (struct spi_desc *)malloc(sizeof(*descriptor));
	desc_extra = (struct spi_desc_extra*)malloc(sizeof(*desc_extra));

	if (!descriptor || !desc_extra)
		return -1;

	init_extra = cast_to_extra_init(param->extra);

	descriptor->max_speed_hz = param->max_speed_hz;
	descriptor->chip_select = param->chip_select;
	descriptor->mode = param->mode;

	desc_extra->rx_length = 0;
	desc_extra->tx_length = 0;
	desc_extra->clk_div =
		descriptor->max_speed_hz /
		(2 * init_extra->spi_clk_hz) - 1;
	desc_extra->spi_baseaddr =
		init_extra->spi_baseaddr;
	desc_extra->spi_clk_hz =
		init_extra->spi_clk_hz;
	desc_extra->spi_offload_rx_support_en =
		init_extra->spi_offload_rx_support_en;
	desc_extra->spi_offload_tx_support_en =
		init_extra->spi_offload_tx_support_en;
	desc_extra->spi_offload_tx_dma_baseaddr =
		init_extra->spi_offload_tx_dma_baseaddr;
	desc_extra->spi_offload_rx_dma_baseaddr =
		init_extra->spi_offload_rx_dma_baseaddr;

	// perform a reset
	spi_eng_write(desc_extra, SPI_ENGINE_REG_RESET, 0x01);
	usleep(100000);
	spi_eng_write(desc_extra, SPI_ENGINE_REG_RESET, 0x00);

	// get current data width
	spi_eng_read(desc_extra, SPI_ENGINE_REG_DATA_WIDTH, &data_width);
	desc_extra->max_data_width = data_width;
	desc_extra->data_width = data_width;
	//spi_set_transfer_length(desc_extra, desc_extra->max_data_width);

	descriptor->extra = desc_extra;
	*desc = descriptor;

	return 0;
}

/**
 * @brief Write and read data to/from SPI.
 * @param desc - The SPI descriptor.
 * @param data - The buffer with the transmitted/received data.
 * @param bytes_number - Number of bytes to write/read.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t spi_write_and_read(spi_desc *desc,
			   uint8_t *data,
			   uint8_t bytes_number)
{
	/*
	 * Note:  This function works like a classic SPI
	 */
	spi_eng_msg *msg;
	spi_desc_extra		*desc_extra;
	uint8_t i, xfer_word_len, xfer_words_number;
	uint32_t spi_eng_msg_cmds[4];
	int32_t ret;

	/*
	 * Each spi engine transaction equals data_width bytes
	 */
	desc_extra = desc->extra;

	xfer_word_len = spi_get_word_lenght(desc_extra);
	xfer_words_number = spi_get_words_number(desc_extra, bytes_number);

	spi_eng_msg_cmds[0] = CS_ASSERT;
	spi_eng_msg_cmds[1] = CS_DEASSERT;
	spi_eng_msg_cmds[2] = TRANSFER_BYTES_R_W(xfer_words_number);
	spi_eng_msg_cmds[3] = CS_ASSERT;

	msg = (spi_eng_msg *)malloc(sizeof(*msg));
	if (!msg)
		return -1;

	msg->spi_msg_cmds = malloc(sizeof(spi_eng_msg_cmds));
	msg->spi_msg_cmds = spi_eng_msg_cmds;
	msg->msg_cmd_len = ARRAY_SIZE(spi_eng_msg_cmds);

	msg->tx_buf = (uint32_t *)malloc(xfer_words_number * sizeof(msg->tx_buf));
	msg->rx_buf = (uint32_t *)malloc(xfer_words_number * sizeof(msg->rx_buf));

	// Init the rx and tx buffers with 0s
	for (i = 0; i < xfer_words_number; i++) {
		msg->tx_buf[i] = 0;
		msg->rx_buf[i] = 0;
	}

	for (i = 0; i < bytes_number; i++)
		msg->tx_buf[i / xfer_word_len] |=
			data[i] << (desc_extra->data_width - (i % xfer_word_len + 1) * 8);

	ret = spi_eng_transfer_message(desc, msg);

	// Skip the first byte ( dummy read byte )
	for (i = 1; i < bytes_number; i++)
		data[i - 1] = msg->rx_buf[(i) / xfer_word_len] >>
			(desc_extra->data_width - ((i) % xfer_word_len + 1) * 8);

	free(msg->tx_buf);
	free(msg->rx_buf);
	free(msg);

	return ret;
}

/***************************************************************************//**
* @brief spi_eng_offload_load_msg
*******************************************************************************/
int32_t spi_eng_offload_load_msg(spi_desc *desc, spi_eng_msg *msg)
{
	uint32_t i, size;
	spi_eng_transfer_fifo *xfer;
	uint8_t words_number;
	spi_desc_extra		*desc_extra;

	desc_extra = cast_to_extra_desc(desc->extra);

	desc_extra->rx_dma_startaddr = msg->rx_buf_addr;
	desc_extra->tx_dma_startaddr = msg->tx_buf_addr;

	if(desc_extra->spi_offload_rx_support_en || desc_extra->spi_offload_tx_support_en)
		desc_extra->offload_configured = 1;

	size = sizeof(*xfer->cmd_fifo) * (msg->msg_cmd_len + 3);

	xfer = (spi_eng_transfer_fifo *)malloc(sizeof(*xfer) + size);
	if (!xfer)
		return -1;

	xfer->cmd_fifo_len = 0;

	spi_eng_compile_message(desc, msg, xfer);

	// CMD OFFLOAD
	for (i = 0; i < xfer->cmd_fifo_len; i++)
		spi_eng_write(desc_extra, SPI_ENGINE_REG_OFFLOAD_CMD_MEM(0), xfer->cmd_fifo[i]);

	// TX OFFLOAD
	words_number = spi_get_words_number(desc_extra, desc_extra->tx_length);
	for(i = 0; i < words_number; i++)
		spi_eng_write(desc_extra, SPI_ENGINE_REG_OFFLOAD_SDO_MEM(0), msg->tx_buf[i]);

	free(xfer);

	return 0;
}

/***************************************************************************//**
* @brief spi_eng_transfer_multiple_msgs
*******************************************************************************/
int32_t spi_eng_transfer_multiple_msgs(spi_desc *desc, uint32_t no_of_messages)
{
	uint8_t alignment;
	spi_desc_extra		*desc_extra;

	desc_extra = cast_to_extra_desc(desc->extra);

	if(!desc_extra->offload_configured)
		return -1;

	if (desc_extra->data_width > 16)
		alignment = sizeof(uint32_t);
	else
		alignment = sizeof(uint16_t);

	if(desc_extra->rx_length) {
		desc_extra->rx_length = alignment * no_of_messages;

		spi_eng_dma_write(desc_extra, DMAC_REG_CTRL, 0x0);
		spi_eng_dma_write(desc_extra, DMAC_REG_CTRL, DMAC_CTRL_ENABLE);
		spi_eng_dma_write(desc_extra, DMAC_REG_IRQ_MASK, 0x0);

		spi_eng_dma_write(desc_extra, DMAC_REG_IRQ_PENDING, 0xff);

		spi_eng_dma_write(desc_extra, DMAC_REG_DEST_ADDRESS, desc_extra->rx_dma_startaddr);
		spi_eng_dma_write(desc_extra, DMAC_REG_DEST_STRIDE, 0x0);
		spi_eng_dma_write(desc_extra, DMAC_REG_X_LENGTH, desc_extra->rx_length - 1);
		spi_eng_dma_write(desc_extra, DMAC_REG_Y_LENGTH, 0x0);

		spi_eng_dma_write(desc_extra, DMAC_REG_START_TRANSFER, 0x1);
	}

	if(desc_extra->tx_length) {
		desc_extra->tx_length = alignment * no_of_messages;

		spi_eng_dma_write(desc_extra, DMAC_REG_CTRL, 0x0);
		spi_eng_dma_write(desc_extra, DMAC_REG_CTRL, DMAC_CTRL_ENABLE);
		spi_eng_dma_write(desc_extra, DMAC_REG_IRQ_MASK, 0x0);

		spi_eng_dma_write(desc_extra, DMAC_REG_IRQ_PENDING, 0xff);

		spi_eng_dma_write(desc_extra, DMAC_REG_SRC_ADDRESS, desc_extra->tx_dma_startaddr);
		spi_eng_dma_write(desc_extra, DMAC_REG_SRC_STRIDE, 0x0);
		spi_eng_dma_write(desc_extra, DMAC_REG_X_LENGTH, desc_extra->rx_length - 1);
		spi_eng_dma_write(desc_extra, DMAC_REG_Y_LENGTH, 0x0);
		spi_eng_dma_write(desc_extra, DMAC_REG_FLAGS, 0x1);

		spi_eng_dma_write(desc_extra, DMAC_REG_START_TRANSFER, 0x1);
	}

	usleep(100000);
	// Enable SPI engine
	spi_eng_write(desc_extra, SPI_ENGINE_REG_OFFLOAD_CTRL(0), 0x0001);

	desc_extra->offload_configured = 0;

	return 0;
}

/**
 * @brief Free the resources allocated by spi_init().
 * @param desc - The SPI descriptor.
 * @return 0 in case of success, -1 otherwise.
 */
int32_t spi_remove(spi_desc *desc)
{
	free(desc);

	return 0;
}
