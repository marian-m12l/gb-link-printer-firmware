/**
 * FIXME: License
 */

#include "pio_secondary.h"

uint8_t __time_critical_func(pio_secondary_read8_blocking)(const pio_secondary_inst_t *secondary) {
    return pio_sm_get_blocking(secondary->pio, secondary->sm);
}

uint8_t __time_critical_func(pio_secondary_read8)(const pio_secondary_inst_t *secondary) {
    return pio_sm_get(secondary->pio, secondary->sm);
}

bool __time_critical_func(pio_secondary_available)(const pio_secondary_inst_t *secondary) {
    return !pio_sm_is_rx_fifo_empty(secondary->pio, secondary->sm);
}

