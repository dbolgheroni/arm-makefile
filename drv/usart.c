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

#include <usart.h>
#include <gpio.h>

void usart_init(USART_TypeDef *usart) {
    /* USART1: TX=PA9, RX=PA10 */
    if (usart == USART1) {
        /* enable USART1 in APB2 */
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN |
            RCC_APB2ENR_IOPAEN;

        NVIC_EnableIRQ(USART1_IRQn);

        /* configure AF GPIO pins for TX and RX */
        GPIOA->CRH |= GPIO_CRH_CNF9_1; /* AF out push-pull */
        GPIOA->CRH |= GPIO_CRH_MODE9_1; /* out max speed 2 MHz */

        GPIOA->CRH &= ~(GPIO_CRH_MODE10_1 | GPIO_CRH_MODE10_0); /* input mode */
        GPIOA->CRH |= GPIO_CRH_CNF10_0; /* floating input */

        usart->CR1 |= USART_CR1_UE; /* USART1 enable */
        usart->CR1 |= USART_CR1_TE; /* transmitter enable */
        usart->CR1 |= USART_CR1_RE; /* receiver enable */
        usart->CR1 |= USART_CR1_RXNEIE; /* irq for receiver */

        /* brr for 9600 (sysclk / baudrate) */
        usart->BRR |= 7500;
    }
}

void usart_putc(USART_TypeDef *usart, uint8_t c) {
    while (!(usart->SR & USART_SR_TXE));
    usart->DR = c;
}

void usart_puts(USART_TypeDef *usart, uint8_t *s) {
    while (*s != 0) {
        usart_putc(usart, *s);
        s++;
    }
}

/* irq handler */
void USART1_IRQHandler(void) {
    gpio_toggle(GPIOC, 13);
    USART1->SR &= ~USART_SR_RXNE;
}
