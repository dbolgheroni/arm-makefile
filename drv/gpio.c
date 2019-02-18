#include <stm32f103xb.h>
#include "gpio.h"

/* parameters: port */
void gpio_init(GPIO_TypeDef *gpio) {
    if (gpio == GPIOA) {
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    } else if (gpio == GPIOB) {
        RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    } else if (gpio == GPIOC) {
        RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    } else if (gpio == GPIOD) {
        RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
    } else if (gpio == GPIOE) {
        RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;
    } else {
        return;
    }
}

/* parameters: port, pin, mode (input/output) */
void gpio_mode(GPIO_TypeDef *gpio, uint8_t pin, uint8_t mode) {
    if (pin >= 0 && pin < 8) {
        if (mode == GPIO_OUTPUT) { /* output */
            gpio->CRL |= GPIO_CRL_MODEx_1(pin);
            gpio->CRL &= ~GPIO_CRL_CNFx_0(pin);
        } else { /* input */
            gpio->CRL &= GPIO_CRL_CNFx_0(pin);
            gpio->CRL |= GPIO_CRL_CNFx_1(pin);
        }
    } else if (pin >= 8 && pin < 16) {
        if (mode == GPIO_OUTPUT) { /* output */
            gpio->CRH |= GPIO_CRH_MODEx_1(pin);
            gpio->CRH &= ~GPIO_CRH_CNFx_0(pin);
        } else { /* input */
            gpio->CRH &= GPIO_CRH_CNFx_0(pin);
            gpio->CRH |= GPIO_CRH_CNFx_1(pin);
        }
    } else {
        return;
    }
}

/* parameters: port, pin */
void gpio_set(GPIO_TypeDef *gpio, uint8_t pin) {
    if (pin >= 0 && pin < 16) {
        gpio->BSRR |= GPIO_BSRR_BSx(pin);
    } else {
        return;
    }
}

/* parameters: port, pin */
void gpio_reset(GPIO_TypeDef *gpio, uint8_t pin) {
    if (pin >= 0 && pin < 16) {
        gpio->BSRR |= GPIO_BSRR_BRx(pin);
    } else {
        return;
    }
}

/* parameters: port, pin */
void gpio_toggle(GPIO_TypeDef *gpio, uint8_t pin) {
    if (pin >= 0 && pin < 16) {
        gpio->ODR ^= GPIO_ODR_ODRx(pin);
    } else {
        return;
    }
}
