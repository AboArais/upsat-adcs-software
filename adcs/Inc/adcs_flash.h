/*
 * adcs_flash.h
 *
 *  Created on: Jul 4, 2016
 *      Author: azisi
 */

#ifndef ADCS_FLASH_H_
#define ADCS_FLASH_H_

typedef enum {
    FLASH_ERROR = 0, FLASH_NORMAL
} flash_status;

flash_status flash_init();
flash_status flash_write_enable();
uint8_t flash_isWrite_busy(flash_status *status);
flash_status flash_readID(uint8_t *id);
flash_status flash_chip_erase();
flash_status flash_erase_block4K(uint32_t address);
flash_status flash_write_byte(uint8_t data, uint32_t address);
flash_status flash_write_page(uint8_t *data, uint32_t address);
flash_status flash_read_byte(uint8_t *data, uint32_t address);

extern uint32_t adcs_boot_cnt;
flash_status flash_increment_boot_counter();

#endif /* ADCS_FLASH_H_ */
