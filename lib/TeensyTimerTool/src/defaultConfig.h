#pragma once

#include "boardDef.h"
namespace TeensyTimerTool
{
//--------------------------------------------------------------------------------------------
// Timer pool defintion
// Add, and sort and remove to define the timer pool. Timers will be allocted from left to right

TimerGenerator* const timerPool[] = {TMR1};

constexpr unsigned timerCnt = sizeof(timerPool) / sizeof(timerPool[0]);

//--------------------------------------------------------------------------------------------
// Default settings for various timers

// TMR (QUAD)
    constexpr int TMR_DEFAULT_PSC = PSC_128;  // Allowed prescaling values: PSC_1, PSC_2, PSC_4 ... PSC_128, clock = 150MHz

// FTM
    constexpr int FTM_DEFAULT_PSC[] =         // Allowed prescaling values: PSC_AUTO, PSC_1, PSC_2, PSC_4 ... PSC_128, clock = FBUS
    {                                         // (PSC_AUTO adjusts prescaler to get roughly 2 timer ticks per µs)
        /*FTM0*/ PSC_AUTO,
        /*FTM1*/ PSC_AUTO,
        /*FTM2*/ PSC_AUTO,
        /*FTM3*/ PSC_AUTO
    };


// GPT & PID
    constexpr bool USE_GPT_PIT_150MHz = false;// changes the clock source for GPT and PIT from 24MHz (standard) to 150MHz, might have side effects!

// TCK
    constexpr unsigned NR_OF_TCK_TIMERS = 20; // How many TCK timers shall be available

    #define YIELD_TYPE  YIELD_STANDARD        // Select the required yield strategy from the list below
                                              // YIELD_NONE:      lib doesn't touch yield. Make sure to call TeensyTimerTool::tick as often as possible
                                              // YIELD_STANDARD:  uses the standard yield function and adds a call to TeensyTimerTool::tick(). Lots of overhead in yield...
                                              // YIELD_OPTIMIZED: generate an optimized yield which only calls TeensyTimerTool::Tick()  (recommended if you don't use SerialEvents)




//--------------------------------------------------------------------------------------------
// Callback type
// Uncomment if you prefer function pointer callbacks instead of std::function callbacks

//   #define PLAIN_VANILLA_CALLBACKS


//--------------------------------------------------------------------------------------------
// Use c++14 time literals (e.g. 3.4s, 50ms, _kHz...) for time inputs. Periods without literals are
// interpreted as microseconds.
// Comment the following line if you don't want this.

    #define USE_TIME_LITERALS


//--------------------------------------------------------------------------------------------
// Advanced Features
// Uncomment if you need access to advanced features

//   #define ENABLE_ADVANCED_FEATURES
}