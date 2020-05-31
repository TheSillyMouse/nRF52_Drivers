#include <string.h>
#include <stdbool.h>
#include "spis.h"
#include "nrf52.h"
#include "nRF52_Drivers/gpio/gpio.h"


static void spis_pins_init(struct spis * spis);

static void spis_configure(void);

static void spis_def_set(uint8_t ch);

static void spis_orc_set(uint8_t orc);

static void spis_shorts_enable(uint32_t mask);

/**
 * @brief callback pointer to function of EVENTS_ACQUIRED
 * it is called inside SPIS_IRQ.
 */
static volatile void (*spis_irq_cb)(void);



static void spis_pins_init(struct spis * spis)
{
    gpio_config_input(spis->pins.mosi, GPIO_PIN_CNF_PULL_Disabled);
    gpio_config_input(spis->pins.miso, GPIO_PIN_CNF_PULL_Disabled);
    gpio_config_input(spis->pins.clock, GPIO_PIN_CNF_PULL_Disabled);
    gpio_config_input(spis->pins.slave_select, GPIO_PIN_CNF_PULL_Disabled);
    
    NRF_SPIS1->PSEL.MOSI = spis->pins.mosi;
    NRF_SPIS1->PSEL.MISO = spis->pins.miso;
    NRF_SPIS1->PSEL.SCK  = spis->pins.clock;
    NRF_SPIS1->PSEL.CSN  = spis->pins.slave_select;
}

void spis_set_rx_buffer(uint8_t * buff, size_t length)
{
    NRF_SPIS1->RXD.PTR    = (uint32_t)buff;
    NRF_SPIS1->RXD.MAXCNT = length;
}

void spis_set_tx_buffer(uint8_t * buff, size_t length)
{
    NRF_SPIS1->TXD.PTR    = (uint32_t)buff;
    NRF_SPIS1->TXD.MAXCNT = length;
}

void spis_dma_get_tx_pointer(uint32_t * data_ptr)
{
    *data_ptr = NRF_SPIS1->TXD.PTR;
}

void spis_dma_get_rx_pointer(uint32_t * data_ptr)
{
    *data_ptr = NRF_SPIS1->RXD.PTR;
}


static void spis_mode_config(void)
{
    uint32_t config = SPIS_CONFIG_ORDER_MsbFirst;
    config |= (SPIS_CONFIG_CPOL_ActiveHigh << SPIS_CONFIG_CPOL_Pos) |
             (SPIS_CONFIG_CPHA_Leading << SPIS_CONFIG_CPHA_Pos);
    NRF_SPIS1->CONFIG = config;
}

// default 0xFF
static void spis_def_set(uint8_t ch)
{
    NRF_SPIS1->DEF = ch;
}

// default 0xFF
static void spis_orc_set(uint8_t orc)
{
    NRF_SPIS1->ORC = orc;
}

static void spis_shorts_enable(uint32_t mask)
{
    NRF_SPIS1->SHORTS |= mask;
}

void spis_enable(void)
{
    NRF_SPIS1->ENABLE = (SPIS_ENABLE_ENABLE_Enabled << SPIS_ENABLE_ENABLE_Pos);
}

void spis_interrupt_enable(uint32_t mask)
{
    NRF_SPIS1->INTENSET = mask;
}

void spis_interrupt_disable(uint32_t mask)
{
    NRF_SPIS1->INTENCLR = mask;
}

void spis_clear_event_end(void)
{
    NRF_SPIS1->EVENTS_END = 0x0UL;
}

void spis_clear_event_acquired(void)
{
    NRF_SPIS1->EVENTS_ACQUIRED = 0x0UL;
}

void spis_enable_task_acquire(void)
{
    NRF_SPIS1->TASKS_ACQUIRE = true;
}

void spis_enable_task_release(void)
{
    NRF_SPIS1->TASKS_RELEASE = true;
}

bool spis_get_event_acquired(void)
{
    return  NRF_SPIS1->EVENTS_ACQUIRED;
}

bool spis_get_event_end(void)
{
    return NRF_SPIS1->EVENTS_END;
}

bool spis_get_event_endrx(void)
{
    return NRF_SPIS1->EVENTS_ENDRX;
}

void spis_clear_event_endrx(void)
{
    NRF_SPIS1->EVENTS_ENDRX = false;
}

void spis_set_irq_callback(void (*callback)(void))
{
    if (callback == NULL)
        return;
    spis_irq_cb = callback;
}



void spis_init(struct spis * spis)
{
    spis_pins_init(spis);

    spis_set_tx_buffer(NULL, 0);
    spis_set_rx_buffer(NULL, 0);

    spis_mode_config();

    spis_def_set(0xFF);
    spis_orc_set(0xFF);

    spis_clear_event_end();
    spis_clear_event_acquired();
    
    spis_shorts_enable(SPIS_SHORTS_END_ACQUIRE_Msk);
    
    spis_enable(); 
}

void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void)
{
    if (spis_irq_cb != NULL)
    {
        spis_irq_cb();
    }
}