/*
 * adrv9009_wrappers.c
 *
 *  Created on: Sep 19, 2019
 *      Author: amiclaus
 */

#include "adrv9009_xilinx_wrappers.h"

#include <stdlib.h>

void xilinx_wrapper(spi_init_param *param){
	xil_spi_init_param *xil_spi_param = calloc(1, sizeof(xil_spi_init_param));
	xil_spi_param->id = 0;
	xil_spi_param->flags = SPI_CS_DECODE;
	param->extra = xil_spi_param;
}

int32_t xilinx_read_wrapper(uint32_t base, uint32_t offset){
	return Xil_In32(base + offset);
}

void xilinx_write_wrapper(uint32_t base, uint32_t offset, uint32_t data){
	Xil_Out32(base + offset, data);
}

void xilinx_cache_flush_wrapper(){
	Xil_DCacheFlush();
}
