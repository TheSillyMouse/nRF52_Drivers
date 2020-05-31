#ifndef GPIO_H
#define GPIO_H

enum gpio_dir
{
    GPIO_OUTPUT,
    GPIO_INPUT
};

enum gpio_pull
{
    GPIO_NOPULL = 0,
    GPIO_PULLUP = 1,
    GPIO_PULLDOWN = 3,
};

enum gpio_drive_cfg
{
    GPIO_DRIVE_S0S1 = 0,
    GPIO_DRIVE_H0S1 = 1,
    GPIO_DRIVE_S0H1 = 2,
    GPIO_DRIVE_H0H1 = 3,
    GPIO_DRIVE_D0S1 = 4,
    GPIO_DRIVE_D0H1 = 5,
    GPIO_DRIVE_S0D1 = 6,
    GPIO_DRIVE_H0D1 = 7,
};

enum gpio_sense
{
    GPIO_NOSENSE = 0,
    GPIO_SENSE_HIGH = 2,
    GPIO_SENSE_LOW = 3,
};

/**@brief Sets individual pins as HIGH level.
 * 
 * @param pin_num Pin number of PORT.
 */
void gpio_out_set(uint32_t pin_num);

void gpio_config_input(uint32_t pin_num, uint32_t pull);

void gpio_config_output(uint32_t pin_num);

#endif