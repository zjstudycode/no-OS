/*******************************************************************************
 *   @file   spi_engine_fifo.c
 *   @brief  Implementation of SPI Engine fifo feature.
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
 ******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdlib.h>
#include <sleep.h>
#include "error.h"
#include "spi_engine_core.h"


/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/

/*******************************************************************************
 *
 * @func	spi_init
 *
 * @brief	Initialize the spi engine
 *
 * @params
 *		desc		- Spi engine descriptor
 *		param		- Structure containing the spi init parameters
 *
 * @return			- SUCCESS if the transfer finished
 *				- FAILURE if the memory allocation failed
 *
 ******************************************************************************/
int32_t spi_init(struct spi_desc **desc,
		 const struct spi_init_param *param)
{
	uint32_t		data_width;
	struct spi_desc		*descriptor;
	struct spi_desc_extra	*desc_extra;
	struct spi_init_param_extra *init_extra;

	descriptor = (struct spi_desc *)malloc(sizeof(*descriptor));
	desc_extra = (struct spi_desc_extra*)malloc(sizeof(*desc_extra));

	if (!descriptor || !desc_extra)
		return FAILURE;

	init_extra = (struct spi_init_param_extra*)param->extra;

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

	/* Perform a reset */
	spi_eng_write(desc_extra, SPI_ENGINE_REG_RESET, 0x01);
	usleep(1000);
	spi_eng_write(desc_extra, SPI_ENGINE_REG_RESET, 0x00);

	/* Get current data width */
	spi_eng_read(desc_extra, SPI_ENGINE_REG_DATA_WIDTH, &data_width);
	desc_extra->max_data_width = data_width;
	desc_extra->data_width = data_width;

	descriptor->extra = desc_extra;
	*desc = descriptor;

	return SUCCESS;
}

/*******************************************************************************
 *
 * @func	spi_write_and_read
 *
 * @brief	Write/read on the spi interface
 *
 * @params
 *		desc		- Spi engine descriptor
 *		data		- Pointer to data buffer
 *		bytes_number	- Number of bytes to transfer
 *

 * @return			- SUCCESS if the transfer finished
 *				- FAILURE if the memory allocation
 *				or transfer failed
 *
 ******************************************************************************/
int32_t spi_write_and_read(spi_desc *desc,
			   uint8_t *data,
			   uint8_t bytes_number)
{
	uint8_t 	i;
	uint8_t 	xfer_word_len;
	uint8_t 	xfer_words_number;
	uint32_t	spi_eng_msg_cmds[4];
	int32_t 	ret;
	spi_eng_msg	*msg;
	spi_desc_extra	*desc_extra;

	desc_extra = desc->extra;

	xfer_words_number = spi_get_words_number(desc_extra, bytes_number);

	/* Make sure the CS is HIGH before starting a transaction */
	spi_eng_msg_cmds[0] = CS_ASSERT;
	spi_eng_msg_cmds[1] = CS_DEASSERT;
	spi_eng_msg_cmds[2] = TRANSFER_BYTES_R_W(xfer_words_number);
	spi_eng_msg_cmds[3] = CS_ASSERT;

	msg = (spi_eng_msg *)malloc(sizeof(*msg));
	if (!msg)
		return FAILURE;

	msg->spi_msg_cmds = spi_eng_msg_cmds;
	msg->msg_cmd_len = ARRAY_SIZE(spi_eng_msg_cmds);

	msg->tx_buf =(uint32_t*)malloc(xfer_words_number * sizeof(msg->tx_buf[0]));
	msg->rx_buf =(uint32_t*)malloc(xfer_words_number * sizeof(msg->rx_buf[0]));

	/* Init the rx and tx buffers with 0s */
	for (i = 0; i < xfer_words_number; i++) {
		msg->tx_buf[i] = 0;
		msg->rx_buf[i] = 0;
	}

	xfer_word_len = spi_get_word_lenght(desc_extra);

	/* Pack the bytes into engine WORDS */
	for (i = 0; i < bytes_number; i++)
		msg->tx_buf[i / xfer_word_len] |=
			data[i] << (desc_extra->data_width -
				    (i % xfer_word_len + 1) * 8);

	ret = spi_eng_transfer_message(desc, msg);

	/* Skip the first byte ( dummy read byte ) */
	for (i = 1; i < bytes_number; i++)
		data[i - 1] = msg->rx_buf[(i) / xfer_word_len] >>
			      (desc_extra->data_width - ((i) % xfer_word_len + 1) * 8);

	free(msg->tx_buf);
	free(msg->rx_buf);
	free(msg);

	return ret;
}

/*******************************************************************************
 *
 * @func	spi_remove
 *
 * @brief	Free the resources allocated by spi_init().
 *
 * @params
 *		desc		- Spi engine descriptor extra parameters
 *
 * @return			- This function allways returns SUCCESS
 *
 ******************************************************************************/
int32_t spi_remove(spi_desc *desc)
{
	free(desc->extra);
	free(desc);

	return SUCCESS;
}