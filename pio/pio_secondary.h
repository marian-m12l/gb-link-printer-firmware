/**
 * FIXME: License
 */

#ifndef _PIO_SECONDARY_H
#define _PIO_SECONDARY_H

#include "hardware/pio.h"
#include "secondary.pio.h"

typedef struct pio_secondary_inst {
    PIO pio;
    uint sm;
    uint cs_pin;
} pio_secondary_inst_t;

uint8_t pio_secondary_read8_blocking(const pio_secondary_inst_t *secondary);
uint8_t pio_secondary_read8(const pio_secondary_inst_t *secondary);
bool pio_secondary_available(const pio_secondary_inst_t *secondary);

#endif
