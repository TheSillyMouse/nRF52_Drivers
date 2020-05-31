#include "nrf52.h"
#include "uart.h"


void uart_set_baudrate(uint32_t val)
{
    NRF_UART0->BAUDRATE = val;
}

void uart_configure(uint32_t parity, uint32_t hwfc)
{
    NRF_UART0->CONFIG = (parity << UART_CONFIG_PARITY_Pos) | (hwfc << UART_CONFIG_HWFC_Pos);
}

void uart_set_pins(uint32_t txpin, uint32_t rxpin)
{
    NRF_UART0->PSELRXD = rxpin;
    NRF_UART0->PSELTXD = txpin;
}

void uart_enable(void)
{
    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Enabled;
}

void uart_init(void)
{
    uart_set_baudrate(UART_BAUDRATE_BAUDRATE_Baud115200);
    uart_configure(UART_CONFIG_PARITY_Excluded, UART_CONFIG_HWFC_Disabled);
    uart_set_pins(6, 8);

    uart_enable();
    
    NRF_UART0->TASKS_STARTTX = 1;
}

void uart_write(uint8_t data)
{
    NRF_UART0->TXD = data;
}

void uart_write_string(uint8_t * str, int size)
{
    do
    {
        NRF_UART0->TXD = *str;
        NRF_UART0->EVENTS_TXDRDY = 0;
        str++;
        while((NRF_UART0->EVENTS_TXDRDY != 1));
        size--;
    }while(size);
}