/*
 * MIT License
 *
 * Copyright (c) 2019, Daniel Bolgheroni.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stm32f103xb.h>
#include "gpio.h"

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

void gpio_set(GPIO_TypeDef *gpio, uint8_t pin) {
    if (pin >= 0 && pin < 16) {
        gpio->BSRR |= GPIO_BSRR_BSx(pin);
    } else {
        return;
    }
}

void gpio_reset(GPIO_TypeDef *gpio, uint8_t pin) {
    if (pin >= 0 && pin < 16) {
        gpio->BSRR |= GPIO_BSRR_BRx(pin);
    } else {
        return;
    }
}

void gpio_toggle(GPIO_TypeDef *gpio, uint8_t pin) {
    if (pin >= 0 && pin < 16) {
        gpio->ODR ^= GPIO_ODR_ODRx(pin);
    } else {
        return;
    }
}
