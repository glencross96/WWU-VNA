/*
 * -------------------------------------------
 *    MSP432 DriverLib - v3_10_00_09 
 * -------------------------------------------
 *
 * --COPYRIGHT--,BSD,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*******************************************************************************
 * MSP432 ADC14 - Single Channel Repeated Sample w/ Timer_A Trigger
 *
 * Description: In this ADC14 code example, a single input channel is sampled
 * using the standard 3.3v reference. The source of the sample trigger for this
 * example is Timer_A CCR1. The ADC is setup to repeatedly sample/convert
 * from A0 when triggered from TIMERA0.  The DMA is used in ping-pong mode and
 * triggered by the dma to store the conversion results.
 *
 * The PWM is started once the GPIO interrupt for P1.1 is serviced.
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST         P6.1  |<--- A14
 *            |            P4.0  |<--- A13
 *            |                  |
 *            |            P1.1  |<--- GPIO trigger to Start conversions
 *            |                  |
 *            |            P1.0  |---> Debug port to show ADC ISR
 *            |            P2.4  |---> Debug TA0.1, ADC trigger
 *            |                  |
 *
 * Author: T. Logan/ C. Sterzik
 ******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

#define ARRAY_LENGTH    256

#define JUNK ((uint16_t)0x0060)
/*
 * Timer_A Compare Configuration Parameter
 * CCR1 is used to trigger the ADC14, conversion time
 * SMCLK = 24Mhz, Timer = 10Khz
 */
const Timer_A_PWMConfig timerA_PWM =
{
    .clockSource = TIMER_A_CLOCKSOURCE_SMCLK,
    .clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1,
    .timerPeriod = 2399,
    .compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1,
    .compareOutputMode = ((uint16_t)0x0060), //TIMER_A_OUTPUTMODE_SET_RESET, This is funny!
    .dutyCycle = 1200
};

/* DMA Control Table */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(MSP_EXP432P401RLP_DMAControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#elif defined(__CC_ARM)
__align(1024)
#endif
static DMA_ControlTable MSP_EXP432P401RLP_DMAControlTable[16];

/* Statics */
static volatile uint16_t resultsBufferPrimary[ARRAY_LENGTH];
static volatile uint16_t resultsBufferAlternate[ARRAY_LENGTH];

int main(void)
{
    /* Halting WDT  */
    WDT_A_holdTimer();
    Interrupt_enableSleepOnIsrExit();

    /*
     * Starting HFXT in non-bypass mode without a timeout. Before we start
     * we have to change VCORE to 1 to support the 48MHz frequency
     */
    PCM_setCoreVoltageLevel(PCM_VCORE1);

    /*
     * Revision C silicon supports wait states of 1 at 48Mhz
     */
    FlashCtl_setWaitState(FLASH_BANK0, 1);
    FlashCtl_setWaitState(FLASH_BANK1, 1);

    /*
     * Setting up clocks
     * MCLK = MCLK = 48MHz
     * SMCLK = MCLK/2 = 24Mhz
     * ACLK = REFO = 32Khz
     */
    CS_setDCOFrequency(48000000);
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_2);
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Initializing ADC (SMCLK/1/1) */
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,
            0);

    /*
     * Debug
     * Configuring P1.0 as output
     */
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    P1OUT &= ~BIT0;

    /*
     * Configuring GPIOs (6.1 A14) (4.0 A13)
     */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1,
    GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0,
    GPIO_TERTIARY_MODULE_FUNCTION);

    /*
     * Debug: set TA0.1 as output to see ADC trigger signal
     */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,
    GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * Configuring P1.1 as an input and enabling interrupt, the timer is started from
     * GPIO ISR.
     */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_interruptEdgeSelect(GPIO_PORT_P1,GPIO_PIN1,GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    REF_A_enableReferenceVoltage();

    /*
     * Configuring ADC Memory, repeat-single-channel, A0
     */
    ADC14_configureSingleSampleMode(ADC_MEM0, true);
    /*
     * Configuring ADC Memory, reference, and differential conversion
     * A0 goes to mem0, AVcc is the reference, and the conversion is
     * single-ended
     */
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_INTBUF_VREFNEG_VSS,
    ADC_INPUT_A6, ADC_NONDIFFERENTIAL_INPUTS);
    /*
     * Configuring the sample trigger to be sourced from Timer_A0 CCR1 and on the
     * rising edge, default samplemode is extended (SHP=0)
     */
    ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE1, false);
    ADC14_setSampleHoldTime(ADC_PULSE_WIDTH_8,ADC_PULSE_WIDTH_8);

    /*
     * Enabling  conversions
     */
    ADC14_enableConversion();

    /* Configuring DMA module */
    DMA_enableModule();
    DMA_setControlBase(MSP_EXP432P401RLP_DMAControlTable);

    /*
     * Setup the DMA + ADC14 interface
     */
    DMA_disableChannelAttribute(DMA_CH7_ADC14,
                                 UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                 UDMA_ATTR_HIGH_PRIORITY |
                                 UDMA_ATTR_REQMASK);

    /*
     * Setting Control Indexes. In this case we will set the source of the
     * DMA transfer to ADC14 Memory 0 and the destination to the destination
     * data array.
     */
    DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH7_ADC14,
            UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
    DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
            UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[0],
            (void*)resultsBufferPrimary, ARRAY_LENGTH);
    DMA_setChannelControl(UDMA_ALT_SELECT | DMA_CH7_ADC14,
            UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
    DMA_setChannelTransfer(UDMA_ALT_SELECT | DMA_CH7_ADC14,
            UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[0],
            (void*)resultsBufferAlternate, ARRAY_LENGTH);

    /* Assigning/Enabling Interrupts */
    DMA_assignInterrupt(DMA_INT1, 7);
    DMA_assignChannel(DMA_CH7_ADC14);
    DMA_clearInterruptFlag(7);

    /* Enabling Interrupts */
    Interrupt_enableInterrupt(INT_DMA_INT1);
    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableMaster();

    /* Going to sleep */
    PCM_gotoLPM0();
    __no_operation();
}

/* Completion interrupt for ADC14 MEM0 */
__attribute__((ramfunc))  // Requires compiler TI v15.12.1.LTS
void DMA_INT1_IRQHandler(void)
{
    P1->OUT |= BIT0;
    /*
     * Switch between primary and alternate buffers with DMA's PingPong mode
     */
    if (DMA_getChannelAttribute(7) & UDMA_ATTR_ALTSELECT)
    {
//        DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH7_ADC14,
//              UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
//        DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH7_ADC14,
//              UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[0],
//              resultsBufferPrimary, ARRAY_LENGTH);
        MSP_EXP432P401RLP_DMAControlTable[7].control =
                (MSP_EXP432P401RLP_DMAControlTable[7].control & 0xff000000 ) |
                (((ARRAY_LENGTH)-1)<<4) | 0x03;
    }
    else
    {
//        DMA_setChannelControl(UDMA_ALT_SELECT | DMA_CH7_ADC14,
//              UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
//        DMA_setChannelTransfer(UDMA_ALT_SELECT | DMA_CH7_ADC14,
//              UDMA_MODE_PINGPONG, (void*) &ADC14->MEM[0],
//              resultsBufferAlternate, ARRAY_LENGTH);
        Timer_A_stopTimer(TIMER_A0_BASE);
        DMA_disableChannel(7);
        /*
         * ISR is being called twice at the end.  Need to investigate further.
         */
        DMA_clearInterruptFlag(7);
        MSP_EXP432P401RLP_DMAControlTable[15].control =
                (MSP_EXP432P401RLP_DMAControlTable[15].control & 0xff000000 ) |
                (((ARRAY_LENGTH)-1)<<4) | 0x03;
    }
    P1->OUT &= ~BIT0;
}

void PORT1_IRQHandler(void)
{
    P1->OUT |= BIT0;
    P1IFG &= ~BIT1;
    DMA_enableChannel(7);
    Timer_A_generatePWM(TIMER_A0_BASE, &timerA_PWM);
    //P1->OUT &= ~BIT0;
}
