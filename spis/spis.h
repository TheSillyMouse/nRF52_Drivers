#ifndef SPIS_H
#define SPIS_H

#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

struct spis
{
    uint8_t instance;
    struct
    {
        uint32_t mosi;
        uint32_t miso;
        uint32_t clock;
        uint32_t slave_select;
    }pins;    
};

void spis_clear_event_end(void);

void spis_clear_event_acquired(void);

void spis_interrupt_enable(uint32_t mask);

void spis_interrupt_disable(uint32_t mask);

void spis_enable(void);

void spis_init(struct spis * spis);

void spis_set_irq_callback(void (*callback)(void));

void spis_enable_task_acquire(void);

bool spis_get_event_acquired(void);

bool spis_get_event_end(void);

void spis_enable_task_release(void);

bool spis_get_event_endrx(void);

void spis_clear_event_endrx(void);

void spis_set_rx_buffer(uint8_t * buff, size_t length);

void spis_set_tx_buffer(uint8_t * buff, size_t length);

void spis_dma_get_tx_pointer(uint32_t * data_ptr);

void spis_dma_get_rx_pointer(uint32_t * data_ptr);

#endif