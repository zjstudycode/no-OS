/*
 * adrv9009_wrapper.h
 *
 *  Created on: Sep 19, 2019
 *      Author: amiclaus
 */

#include "spi.h"
#include "xilinx_platform_drivers.h"

void xilinx_wrapper(spi_init_param *param);

int32_t xilinx_read_wrapper(uint32_t base, uint32_t offset);

void xilinx_write_wrapper(uint32_t base, uint32_t offset, uint32_t data);

void xilinx_cache_flush_wrapper();


