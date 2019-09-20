/*
 * adrv9009_altera_wrappers.h
 *
 *  Created on: Sep 20, 2019
 *      Author: amiclaus
 */

#ifndef SRC_DEVICES_TALISE_ADRV9009_ALTERA_WRAPPERS_H_
#define SRC_DEVICES_TALISE_ADRV9009_ALTERA_WRAPPERS_H_

int32_t altera_read_wrapper(uint32_t base, uint32_t offset);

void altera_write_wrapper(uint32_t base, uint32_t offset, uint32_t data);


#endif /* SRC_DEVICES_TALISE_ADRV9009_ALTERA_WRAPPERS_H_ */
