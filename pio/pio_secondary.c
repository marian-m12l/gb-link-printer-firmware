/**
 * FIXME: License
 */

#include "pio_secondary.h"

void __time_critical_func(pio_secondary_read8_blocking)(const pio_secondary_inst_t *secondary, uint8_t *dst, size_t len) {
    //return pio_sm_get_blocking(secondary->pio, secondary->sm);
    size_t /*tx_remain = len,*/ rx_remain = len;
    //io_rw_8 *txfifo = (io_rw_8 *) &secondary->pio->txf[secondary->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &secondary->pio->rxf[secondary->sm];
    while (/*tx_remain ||*/ rx_remain) {
        /*if (tx_remain && !pio_sm_is_tx_fifo_full(secondary->pio, secondary->sm)) {
            *txfifo = 0;
            --tx_remain;
        }*/
        if (rx_remain && !pio_sm_is_rx_fifo_empty(secondary->pio, secondary->sm)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(pio_secondary_read8)(const pio_secondary_inst_t *secondary, uint8_t *dst, size_t len) {
    //return pio_sm_get(secondary->pio, secondary->sm);
    
    //io_rw_8 *rxfifo = (io_rw_8 *) &secondary->pio->rxf[secondary->sm];
    //return *rxfifo;

    size_t tx_remain = len, rx_remain = len;
    io_rw_8 *txfifo = (io_rw_8 *) &secondary->pio->txf[secondary->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &secondary->pio->rxf[secondary->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain) {
            *txfifo = 0;
            --tx_remain;
        }
        if (rx_remain) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
}

bool __time_critical_func(pio_secondary_available)(const pio_secondary_inst_t *secondary) {
    return !pio_sm_is_rx_fifo_empty(secondary->pio, secondary->sm);
}

