#pragma once

#include "TMRChannel.h"
#include "imxrt.h"

namespace TeensyTimerTool
{
    class TMR
    {
     public:
        static TMR& getInstance()
        {
            static TMR instance;

            if (!isInitialized)
            {
                for (unsigned chNr = 0; chNr < 4; chNr++)
                {
                    pTMR->CH[chNr].CTRL = 0x0000;
                }
                attachInterruptVector(irq, isr); // start
                NVIC_ENABLE_IRQ(irq);
                isInitialized = true;
            }

            return instance;
        }

        static TMRChannel& getTMR1() { return TMR::getInstance().Channel1; }
        static TMRChannel& getTMR2() { return TMR::getInstance().Channel2; }
        static TMRChannel& getTMR3() { return TMR::getInstance().Channel3; }
        static TMRChannel& getTMR4() { return TMR::getInstance().Channel4; }

     protected:
        TMR() {}
        TMR(TMR const&);
        //TMR* operatior=(TMR const& unused);

        static bool isInitialized;
        static void isr();

        // the following is calculated at compile time
        static constexpr IRQ_NUMBER_t irq = IRQ_QTIMER1;
        static IMXRT_TMR_t *const pTMR;
        static IMXRT_TMR_CH_t *const pCH0;
        static IMXRT_TMR_CH_t *const pCH1;
        static IMXRT_TMR_CH_t *const pCH2;
        static IMXRT_TMR_CH_t *const pCH3;

        static TMRChannel Channel1;
        static TMRChannel Channel2;
        static TMRChannel Channel3;
        static TMRChannel Channel4;
    };

    // IMPLEMENTATION ==================================================================

    IMXRT_TMR_t *const TMR::pTMR    = &IMXRT_TMR1;
    IMXRT_TMR_CH_t *const TMR::pCH0 = &pTMR->CH[0];
    IMXRT_TMR_CH_t *const TMR::pCH1 = &pTMR->CH[1];
    IMXRT_TMR_CH_t *const TMR::pCH2 = &pTMR->CH[2];
    IMXRT_TMR_CH_t *const TMR::pCH3 = &pTMR->CH[3];

    TMRChannel TMR::Channel1 = TMRChannel(pCH0);
    TMRChannel TMR::Channel2 = TMRChannel(pCH1);
    TMRChannel TMR::Channel3 = TMRChannel(pCH2);
    TMRChannel TMR::Channel4 = TMRChannel(pCH3);

    void TMR::isr()
    {
        // no loop to gain some time by avoiding indirections and pointer calculations
        if (pCH0->CSCTRL & TMR_CSCTRL_TCF1)
        {
            pCH0->CSCTRL &= ~TMR_CSCTRL_TCF1;
            Channel1.tcf1Count++;
        }

        if (pCH0->CSCTRL & TMR_CSCTRL_TCF2) 
        { // Timer Compare 1 Interrupt
            pCH0->CSCTRL &= ~(TMR_CSCTRL_TCF2);  // clear
        }

        if (pCH0->SCTRL & TMR_SCTRL_TCF) 
        { // Timer Compare Flag Interrupt
            pCH0->SCTRL &= ~(TMR_SCTRL_TCF);  // clear
        }

        if (pCH0->SCTRL & TMR_SCTRL_TOF) 
        { // Timer Overflow Flag Interrupt
            pCH0->SCTRL &= ~(TMR_SCTRL_TOF);  // clear
        }

        if (pCH0->SCTRL & TMR_SCTRL_IEF) 
        { // Input Edge Flag Interrupt
            if(pCH0->SCTRL & TMR_SCTRL_INPUT)
            {
                Channel1.capL = Channel1.capH;
                Channel1.capH = pCH0->CAPT;
                if(Channel1.capL < Channel1.capH)
                {
                    Channel1.capDiff = Channel1.capH - Channel1.capL;
                }
                Channel1.lastTcfCount = Channel2.tcf1Count;
            }

            pCH0->SCTRL &= ~(TMR_SCTRL_IEF);  // clear
            Channel1.iefCount++;
        }

        if (pCH1->CSCTRL & TMR_CSCTRL_TCF1)
        {
            pCH1->CSCTRL &= ~TMR_CSCTRL_TCF1;
            Channel2.tcf1Count++;

            if((Channel2.tcf1Count - Channel1.lastTcfCount) >= 10 && Channel1.capDiff > 0)
            {
                Channel1.capDiff = 0;
                Channel1.capH = 0;
                Channel1.capL = 0;
            }
        }

        if (pCH1->CSCTRL & TMR_CSCTRL_TCF2) 
        { // Timer Compare 1 Interrupt
            pCH1->CSCTRL &= ~(TMR_CSCTRL_TCF2);  // clear
        }

        if (pCH1->SCTRL & TMR_SCTRL_TCF) 
        { // Timer Compare Flag Interrupt
            pCH1->SCTRL &= ~(TMR_SCTRL_TCF);  // clear
        }

        if (pCH1->SCTRL & TMR_SCTRL_TOF) 
        { // Timer Overflow Flag Interrupt
            pCH1->SCTRL &= ~(TMR_SCTRL_TOF);  // clear
        }

        if (pCH1->SCTRL & TMR_SCTRL_IEF) 
        { // Input Edge Flag Interrupt
            pCH1->SCTRL &= ~(TMR_SCTRL_IEF);  // clear
        }

        if (pCH2->CSCTRL & TMR_CSCTRL_TCF1)
        {
            pCH2->CSCTRL &= ~TMR_CSCTRL_TCF1;
        }

        if (pCH3->CSCTRL & TMR_CSCTRL_TCF1)
        {
            pCH3->CSCTRL &= ~TMR_CSCTRL_TCF1;
        }
        asm volatile("dsb"); //wait until register changes propagated through the cache
    }

    bool TMR::isInitialized = false;
} // namespace TeensyTimerTool
