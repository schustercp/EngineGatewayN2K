#pragma once
#include "ErrorHandling/error_codes.h"
#include "config.h"
#include "imxrt.h"
#include <cmath>

namespace TeensyTimerTool
{
    class TMRChannel
    {
     public:
        inline TMRChannel(IMXRT_TMR_CH_t *regs);
        inline virtual ~TMRChannel();

        inline errorCode begin(bool periodic);
        
        inline errorCode trigger(float tcnt);
        inline float getMaxPeriod() const;

        inline void setPrescaler(uint32_t psc); // psc 0..7 -> prescaler: 1..128

        volatile uint32_t tcf1Count;
        volatile uint32_t iefCount;
        volatile uint32_t capH;
        volatile uint32_t capL;
        volatile uint32_t capDiff;
         volatile uint32_t lastTcfCount;

     protected:
        IMXRT_TMR_CH_t *regs;
        float pscValue;
        uint32_t pscBits;

        inline errorCode start(bool periodic);
        errorCode us2Ticks(const float us, uint16_t *ticks) const;
        inline float_t counterToMicrosecond(const float_t cnt) const;
    };

    // IMPLEMENTATION ==============================================

    TMRChannel::TMRChannel(IMXRT_TMR_CH_t *regs)
    {
        this->regs = regs;
        setPrescaler(TMR_DEFAULT_PSC);
    }

    TMRChannel::~TMRChannel()
    {
        regs->CTRL = 0x0000; // stop timer and mark it free
    }

    errorCode TMRChannel::start(bool periodic)
    {
        regs->CSCTRL &= ~TMR_CSCTRL_TCF1;
        regs->CSCTRL |= TMR_CSCTRL_TCF1EN;

        regs->SCTRL &= ~TMR_SCTRL_IEF;
        regs->SCTRL |= TMR_SCTRL_IEFIE;

        return errorCode::OK;
    }

    errorCode TMRChannel::begin(bool periodic)
    {
        if(periodic)
        {
            regs->CTRL   = 0x0000;
            regs->LOAD   = 0x0000;
            regs->COMP1  = 0xE4E2;
            regs->CMPLD1 = 0xE4E2;
            regs->CNTR   = 0x0000;
            regs->CSCTRL = TMR_CSCTRL_CL1(1);
            regs->CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0xF) | TMR_CTRL_LENGTH;
        }
        else
        {
            IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x01;

            regs->CTRL   = 0x0000;
            regs->LOAD   = 0x0000;
            regs->COMP1  = 0xFFFF;
            regs->COMP2  = 0xFFFF;
            regs->CMPLD1 = 0xFFFF;
            regs->CMPLD2 = 0xFFFF;
            regs->CNTR   = 0x0000;
            regs->SCTRL  = TMR_SCTRL_CAPTURE_MODE(0x01) | TMR_SCTRL_TCFIE | TMR_SCTRL_TOFIE | TMR_SCTRL_IEFIE;
            regs->CSCTRL = 0x0000; //TMR_CSCTRL_ROC;
            regs->CTRL   = TMR_CTRL_CM(0x01) | TMR_CTRL_PCS(0xF) | TMR_CTRL_SCS(0) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(0x05);
        }

        start(periodic);
        return errorCode::OK;
    }

    errorCode TMRChannel::trigger(float us) // quick and dirty, should be optimized
    {
        // const float_t t = us2Ticks(tcnt);
        // uint16_t reload = t > 0xFFFF ? 0xFFFF : (uint16_t)t;

        uint16_t reload;
        errorCode status = us2Ticks(us, &reload);

        regs->CTRL   = 0x0000;
        regs->LOAD   = 0x0000;
        regs->COMP1  = reload;
        regs->CMPLD1 = reload;
        regs->CNTR   = 0x0000;

        regs->CSCTRL &= ~TMR_CSCTRL_TCF1;
        regs->CSCTRL |= TMR_CSCTRL_TCF1EN;

        regs->CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0xF) | TMR_CTRL_ONCE | TMR_CTRL_LENGTH;

        return status;
    }

    void TMRChannel::setPrescaler(uint32_t psc) // psc 0..7 -> prescaler: 1..128
    {
        pscValue = 1 << (psc & 0b0111);
        pscBits  = 0b1000 | (psc & 0b0111);
    }

    float TMRChannel::getMaxPeriod() const
    {
        return pscValue / 150'000'000.0f * 0xFFFE;
    }

    errorCode TMRChannel::us2Ticks(const float us, uint16_t *ticks) const
    {
        constexpr uint16_t maxTicks = 0xFFFE;

        float tmpTicks = us * 150.0f / pscValue;
        if (tmpTicks > maxTicks)
        {
            *ticks = maxTicks;
            return errorCode::periodOverflow;
        }
        *ticks = (uint16_t)tmpTicks;
        return errorCode::OK;
    }

    float_t TMRChannel::counterToMicrosecond(const float_t cnt) const
    {
        return cnt * pscValue / 150.0f;
    }

} // namespace TeensyTimerTool
