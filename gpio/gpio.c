#include "nrf52.h"
#include "gpio.h"
#include <stdbool.h>


void gpio_out_set(uint32_t pin_num)
{
    NRF_GPIO_Type * reg = NRF_P0;
    reg->OUTSET = (1 << pin_num);
}

void gpio_config_input(uint32_t pin_num, uint32_t pull)
{
    NRF_GPIO_Type * reg = NRF_P0;
    reg->PIN_CNF[pin_num] = ((uint32_t)GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                               | ((uint32_t)pull << GPIO_PIN_CNF_PULL_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                               | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);     // input sense options
}

void gpio_config_output(uint32_t pin_num)
{
    NRF_GPIO_Type * reg = NRF_P0;
    reg->PIN_CNF[pin_num] = ((uint32_t)GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos)
                                | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}