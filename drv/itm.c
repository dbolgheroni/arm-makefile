#include <itm.h>

int globalCounter;

void enable_itm(void) {
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    AFIO->MAPR |= (2 << 24);
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;

    if (!(DBGMCU->CR & DBGMCU_CR_TRACE_IOEN))
    {
        return;
    }
    
    /* Configure Trace Port Interface Unit */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    TPI->ACPR = 8;
    TPI->SPPR = 2;
    TPI->FFCR = 0x102; /* ITM and DWT */
    
    /* Configure PC sampling and exception trace  */
    DWT->CTRL = (1 << DWT_CTRL_CYCTAP_Pos) // Prescaler for PC sampling
                                           // 0 = x32, 1 = x512
              | (0 << DWT_CTRL_POSTPRESET_Pos) // Postscaler for PC sampling
                                                // Divider = value + 1
              | (1 << DWT_CTRL_PCSAMPLENA_Pos) // Enable PC sampling
              | (2 << DWT_CTRL_SYNCTAP_Pos)    // Sync packet interval
                                               // 0 = Off, 1 = Every 2^23 cycles,
                                               // 2 = Every 2^25, 3 = Every 2^27
              | (1 << DWT_CTRL_EXCTRCENA_Pos)  // Enable exception trace
              | (1 << DWT_CTRL_CYCCNTENA_Pos); // Enable cycle counter
    
    /* Configure instrumentation trace macroblock */
    ITM->LAR = 0xC5ACCE55;
    ITM->TCR = (1 << ITM_TCR_TraceBusID_Pos) // Trace bus ID for TPIU
             | (1 << ITM_TCR_DWTENA_Pos) // Enable events from DWT
             | (1 << ITM_TCR_SYNCENA_Pos) // Enable sync packets
             | (1 << ITM_TCR_ITMENA_Pos); // Main enable for ITM
    ITM->TER = 0xFFFFFFFF; // Enable all stimulus ports
}

void enable_dwt(void) {
    /* This is an example of how to configure DWT to monitor a watchpoint.
       The data value is reported when the watchpoint is hit. */
    
    /* Monitor all accesses to GPIOC (range length 32 bytes) */
    DWT->COMP0 = (uint32_t)GPIOC;
    DWT->MASK0 = 5;
    DWT->FUNCTION0 = (2 << DWT_FUNCTION_FUNCTION_Pos) // Report data and addr on watchpoint hit
                   | (1 << DWT_FUNCTION_EMITRANGE_Pos);
    
    /* Monitor all accesses to globalCounter (range length 4 bytes) */
    DWT->COMP1 = (uint32_t)&globalCounter;
    DWT->MASK1 = 2;
    DWT->FUNCTION1 = (3 << DWT_FUNCTION_FUNCTION_Pos); // Report data and PC on watchpoint hit
}

// Print a given string to ITM.
// This uses 8 bit writes, as that seems to be the most common way to write text
// through ITM. Otherwise there is no good way for the PC software to know what
// is text and what is some other data.
void ITM_Print(int port, const char *p)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << port)))
    {
        while (*p)
        {
            while (ITM->PORT[port].u32 == 0);
            ITM->PORT[port].u8 = *p++;
        }
    }
}

// Write a 32-bit value to ITM.
// This can be used as a fast way to log important values from code.
void ITM_SendValue(int port, uint32_t value)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & (1UL << port)))
    {
        while (ITM->PORT[port].u32 == 0);
        ITM->PORT[port].u32 = value;
    }
}

