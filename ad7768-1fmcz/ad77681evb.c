/***************************************************************************//**
* @file ad77681_evb.c
* @brief Implementation of Main Function.
* @author SPopa (stefan.popa@analog.com)
********************************************************************************
* Copyright 2017(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* - Neither the name of Analog Devices, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
* - The use of this software may or may not infringe the patent rights
* of one or more patent holders. This license does not release you
* from the requirement that you obtain separate licenses from these
* patent holders to use this software.
* - Use of the software either in source or binary form, must be run
* on or directly connected to an Analog Devices Inc. component.
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
#include <stdio.h>
#include <sleep.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <xil_cache.h>
#include <xparameters.h>
#include "xil_printf.h"
#include "ad77681.h"
#include "spi.h"
#include "spi_engine.h"


/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))

#define DISABLE 				0x00
#define ENABLE 					0x01

#define AD77681_DMA_1_BASEADDR			XPAR_AXI_AD77681_DMA_1_BASEADDR
#define AD77681_SPI_ENGINE_BASEADDR		XPAR_SPI_ADC1_AXI_1_BASEADDR
#define AD77681_SPI2_ENGINE_BASEADDR		XPAR_SPI_ADC2_AXI_2_BASEADDR
#define AD77681_SPI_CS				0
#define AD77681_SPI_MAX_SPEED_HZ		100000000UL
#define AD77681_SPI_CLK_HZ			1000000UL
#define AD77681_SPI_MODE			SPI_MODE_3
#define AD77681_SPI_CS_DELAY			1


#define GPIO_DEVICE_ID				XPAR_PS7_GPIO_0_DEVICE_ID
#define GPIO_OFFSET				32

#define GPIO_0_0				GPIO_OFFSET + 6 // 38
#define GPIO_0_1				GPIO_OFFSET + 5 // 37
#define GPIO_0_2				GPIO_OFFSET + 4 // 36
#define GPIO_0_3				GPIO_OFFSET + 3 // 35
#define GPIO_0_SYNC_IN				GPIO_OFFSET + 2 // 34
#define GPIO_0_SYNC_OUT				GPIO_OFFSET + 1 // 33
#define GPIO_0_RESET				GPIO_OFFSET + 0 // 32

#define GPIO_1_0				GPIO_OFFSET + 22 // 54
#define GPIO_1_1				GPIO_OFFSET + 21 // 53
#define GPIO_1_2				GPIO_OFFSET + 20 // 52
#define GPIO_1_3				GPIO_OFFSET + 19 // 51
#define GPIO_1_SYNC_IN				GPIO_OFFSET + 18 // 50
#define GPIO_1_SYNC_OUT				GPIO_OFFSET + 17 // 49
#define GPIO_1_RESET				GPIO_OFFSET + 16 // 48

static struct ad77681_init_param ADC_default_init_param = {
	/* spi config */
	{0},
	/* ad7768-1 config */
	AD77681_FAST,				// power_mode
	AD77681_MCLK_DIV_8,			// mclk_div
	AD77681_CONV_CONTINUOUS,		// conv_mode
	AD77681_POSITIVE_FS,			// diag_mux_sel
	DISABLE,				// conv_diag_sel
	AD77681_CONV_16BIT,			// conv_len
	AD77681_NO_CRC, 			// crc_sel
	DISABLE, 				// status_bit
};

static struct spi_eng_init_param SPI_init_param = {
	AD77681_SPI_MAX_SPEED_HZ,
	AD77681_SPI_CS,
	AD77681_SPI_MODE,
	/* spi engine extra parameters */
	{
		AD77681_SPI_CLK_HZ,
		AD77681_SPI_ENGINE_BASEADDR,
		AD77681_SPI_CS_DELAY,
		ENABLE,	/* rx offload support enabled */
		AD77681_DMA_1_BASEADDR,
		ENABLE,	/* tx offload support enabled */
		AD77681_DMA_1_BASEADDR,
		0,	/* not used */
	},
};


uint32_t spi_msg_cmds[4] = {CS_ASSERT,
			    CS_DEASSERT,
			    TRANSFER_BYTES_R_W(1),
			    CS_ASSERT
			   };


/**
 * @brief Generate miliseconds delay.
 * @param msecs - Delay in miliseconds.
 * @return None.
 */
void mdelay(uint32_t msecs)
{
	usleep(msecs * 1000);
}

#define SPI_ENGINE_OFFLOAD_EXAMPLE	1

int main()
{
	struct ad77681_dev	*adc_dev;
	struct spi_init_param	spi_param;
	spi_eng_msg 		*msg;
	uint32_t 		i;
	uint8_t			adc_data[3];
	uint32_t 		*data;

	Xil_ICacheEnable();
	Xil_DCacheEnable();

	spi_param.max_speed_hz = SPI_init_param.max_speed_hz;
	spi_param.chip_select = SPI_init_param.chip_select;
	spi_param.mode = SPI_init_param.mode;
	spi_param.extra = &(SPI_init_param.extra);

	ADC_default_init_param.spi_eng_dev_init = spi_param;

	ad77681_setup(&adc_dev, ADC_default_init_param);

	if (SPI_ENGINE_OFFLOAD_EXAMPLE == 0) {
		while(1) {
			ad77681_spi_read_adc_data(adc_dev, adc_data);
			xil_printf("[ADC DATA]: 0x");
			for(i = 0; i < sizeof(adc_data) / sizeof(uint8_t); i++) {
				xil_printf("%x", adc_data[i]);
			}
			xil_printf("\r\n");
			mdelay(1000);
		}
	} else { // offload example
		msg = (spi_eng_msg *)malloc(sizeof(*msg));
		if (!msg)
			return -1;

		msg->spi_msg_cmds = malloc(sizeof(spi_msg_cmds));
		msg->spi_msg_cmds = spi_msg_cmds;
		msg->rx_buf_addr = 0x800000;
		msg->tx_buf_addr = 0xA000000;
		msg->msg_cmd_len = sizeof(spi_msg_cmds) / sizeof(uint32_t);
		spi_set_transfer_length(adc_dev->spi_desc, 32);
		msg->tx_buf[0] = AD77681_REG_READ(AD77681_REG_ADC_DATA) << 24;
		spi_eng_offload_load_msg(adc_dev->spi_desc, msg);
		spi_eng_transfer_multiple_msgs(adc_dev->spi_desc, 1);
		mdelay(10000);

		data = (uint32_t*)msg->rx_buf_addr;

		mdelay(1);

		printf("%x\r\n", (unsigned int)*data);
		free(msg);
	}

	printf("Bye\n");

	Xil_DCacheDisable();
	Xil_ICacheDisable();

	return 0;
}
