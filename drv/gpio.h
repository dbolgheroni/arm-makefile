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

#define GPIO_OUTPUT     0x1
#define GPIO_INPUT      0x2

#define GPIO_CRL_MODEx_Pos(x)           (4U * (x))
#define GPIO_CRL_MODEx_Msk(x)           (0x3U << GPIO_CRL_MODEx_Pos(x))
#define GPIO_CRL_MODEx(x)               GPIO_CRL_MODEx_Msk(x)
#define GPIO_CRL_MODEx_0(x)             (0x1U << GPIO_CRL_MODEx_Pos(x))
#define GPIO_CRL_MODEx_1(x)             (0x2U << GPIO_CRL_MODEx_Pos(x))

#define GPIO_CRH_MODEx_Pos(x)           (4U * (x - 8))
#define GPIO_CRH_MODEx_Msk(x)           (0x3U << GPIO_CRH_MODEx_Pos(x))
#define GPIO_CRH_MODEx(x)               GPIO_CRL_MODEx_Msk(x)
#define GPIO_CRH_MODEx_0(x)             (0x1U << GPIO_CRH_MODEx_Pos(x))
#define GPIO_CRH_MODEx_1(x)             (0x2U << GPIO_CRH_MODEx_Pos(x))

#define GPIO_CRL_CNFx_Pos(x)            (4U * (x))
#define GPIO_CRL_CNFx_Msk(x)            (0x3U << GPIO_CRL_CNFx_Pos(x))
#define GPIO_CRL_CNFx(x)                GPIO_CRL_CNFx_Msk(x)
#define GPIO_CRL_CNFx_0(x)              (0x1U << GPIO_CRL_CNFx_Pos(x))
#define GPIO_CRL_CNFx_1(x)              (0x2U << GPIO_CRL_CNFx_Pos(x))

#define GPIO_CRH_CNFx_Pos(x)            (4U * (x - 8))
#define GPIO_CRH_CNFx_Msk(x)            (0x3U << GPIO_CRH_CNFx_Pos(x))
#define GPIO_CRH_CNFx(x)                GPIO_CRH_CNFx_Msk(x)
#define GPIO_CRH_CNFx_0(x)              (0x1U << GPIO_CRH_CNFx_Pos(x))
#define GPIO_CRH_CNFx_1(x)              (0x2U << GPIO_CRH_CNFx_Pos(x))

#define GPIO_BSRR_BSx_Pos(x)            (x)
#define GPIO_BSRR_BSx_Msk(x)            (0x1U << GPIO_BSRR_BSx_Pos(x))
#define GPIO_BSRR_BSx(x)                GPIO_BSRR_BSx_Msk(x)

#define GPIO_BSRR_BRx_Pos(x)            (16U + x)
#define GPIO_BSRR_BRx_Msk(x)            (0x1U << GPIO_BSRR_BRx_Pos(x))
#define GPIO_BSRR_BRx(x)                GPIO_BSRR_BRx_Msk(x)

#define GPIO_ODR_ODRx_Pos(x)            (0U + x)
#define GPIO_ODR_ODRx_Msk(x)            (0x1U << GPIO_ODR_ODRx_Pos(x))
#define GPIO_ODR_ODRx(x)                GPIO_ODR_ODRx_Msk(x)

void gpio_init(GPIO_TypeDef *);
void gpio_mode(GPIO_TypeDef *, uint8_t, uint8_t);
void gpio_set(GPIO_TypeDef *, uint8_t);
void gpio_reset(GPIO_TypeDef *, uint8_t);
void gpio_toggle(GPIO_TypeDef *, uint8_t);
