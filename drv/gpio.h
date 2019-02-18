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
