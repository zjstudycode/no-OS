/*
 * adrv9009_altera_wrappers.c
 *
 *  Created on: Sep 20, 2019
 *      Author: amiclaus
 */

#include <stdint.h>
#include "adrv9009_altera_wrappers.h"

int32_t altera_read_wrapper(uint32_t base, uint32_t offset){
	//IORD_32DIRECT(base, offset);
}

void altera_write_wrapper(uint32_t base, uint32_t offset, uint32_t data){
	//IOWR_32DIRECT(base, offset, data);
}

