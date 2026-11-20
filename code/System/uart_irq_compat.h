#ifndef UART_IRQ_COMPAT_H
#define UART_IRQ_COMPAT_H

#include "zf_common_headfile.h"

#ifndef UART_IRQ_COMPAT_MAX_UART
#define UART_IRQ_COMPAT_MAX_UART    (7u)
#endif

static uint32 uart_irq_rx_error_flags[UART_IRQ_COMPAT_MAX_UART];

static inline uint8 uart_irq_isr_mask(uart_index_enum uart_n)
{
    volatile stc_SCB_t *scb = get_scb_module(uart_n);
    uint8 isr_type = 0;
    uint32 rx_status = Cy_SCB_GetRxInterruptStatusMasked(scb);
    uint32 tx_status = Cy_SCB_GetTxInterruptStatusMasked(scb);
    uint32 rx_errors = Cy_SCB_GetRxInterruptStatus(scb) &
                       (CY_SCB_UART_RX_OVERFLOW |
                        CY_SCB_UART_RX_ERR_FRAME |
                        CY_SCB_UART_RX_ERR_PARITY);

    if(((uint32)uart_n < UART_IRQ_COMPAT_MAX_UART) && (rx_errors != 0u))
    {
        uart_irq_rx_error_flags[(uint32)uart_n] |= rx_errors;
    }

    if(rx_status & CY_SCB_UART_RX_NOT_EMPTY)
    {
        Cy_SCB_ClearRxInterrupt(scb, CY_SCB_UART_RX_NOT_EMPTY);
        isr_type = 1;
    }

    if(tx_status & CY_SCB_UART_TX_DONE)
    {
        Cy_SCB_ClearTxInterrupt(scb, CY_SCB_UART_TX_DONE);
    }

    if(rx_errors != 0u)
    {
        Cy_SCB_ClearRxInterrupt(scb, rx_errors);
    }

    return isr_type;
}

static inline uint8 uart_irq_query_byte(uart_index_enum uart_n, uint8 *dat)
{
    volatile stc_SCB_t *scb = get_scb_module(uart_n);

    if(Cy_SCB_GetNumInRxFifo(scb))
    {
        *dat = (uint8)Cy_SCB_ReadRxFifo(scb);
        return 1;
    }

    return 0;
}

static inline uint32 uart_irq_get_rx_error_flags(uart_index_enum uart_n)
{
    uint32 flags = 0;

    if((uint32)uart_n < UART_IRQ_COMPAT_MAX_UART)
    {
        flags = uart_irq_rx_error_flags[(uint32)uart_n];
        uart_irq_rx_error_flags[(uint32)uart_n] = 0;
    }

    return flags;
}

#endif
