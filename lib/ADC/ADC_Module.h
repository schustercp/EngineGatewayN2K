/* Teensy 4.x, 3.x, LC ADC library
 * https://github.com/pedvide/ADC
 * Copyright (c) 2020 Pedro Villanueva
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* ADC_Module.h: Declarations of the fuctions of a Teensy 3.x, LC ADC module
 *
 */

/*! \page adc_module ADC Module
Control each ADC_Module independently.
See the ADC_Module class for all methods.
*/

#ifndef ADC_MODULE_H
#define ADC_MODULE_H

#include <Arduino.h>
#include <settings_defines.h>
#include <atomic.h>

using ADC_Error::ADC_ERROR;
using namespace ADC_settings;

// debug mode: blink the led light
#define ADC_debug 0

/** Class ADC_Module: Implements all functions of the Teensy 3.x, LC analog to digital converter
*
*/
class ADC_Module
{

public:
    //! Constructor
    /** Pass the ADC number and the Channel number to SC1A number arrays.
    *   \param ADC_number Number of the ADC module, from 0.
    *   \param a_channel2sc1a contains an index that pairs each pin to its SC1A number (used to start a conversion on that pin)
    *   \param a_adc_regs pointer to start of the ADC registers
    */
    ADC_Module(uint8_t ADC_number,
               const uint8_t *const a_channel2sc1a,
               ADC_REGS_t &a_adc_regs);

    //! Starts the calibration sequence, waits until it's done and writes the results
    /** Usually it's not necessary to call this function directly, but do it if the "environment" changed
    *   significantly since the program was started.
    */
    void recalibrate();

    //! Starts the calibration sequence
    void calibrate();

    //! Waits until calibration is finished and writes the corresponding registers
    void wait_for_cal();

    /////////////// METHODS TO SET/GET SETTINGS OF THE ADC ////////////////////

    //! Set the voltage reference you prefer, default is vcc
    /*!
    * \param ref_type can be ADC_REFERENCE::REF_3V3, ADC_REFERENCE::REF_1V2 (not for Teensy LC) or ADC_REFERENCE::REF_EXT
    *
    *  It recalibrates at the end.
    */
    void setReference(ADC_REFERENCE ref_type);

    //! Change the resolution of the measurement.
    /*!
    *  \param bits is the number of bits of resolution.
    *  For single-ended measurements: 8, 10, 12 or 16 bits.
    *  For differential measurements: 9, 11, 13 or 16 bits.
    *  If you want something in between (11 bits single-ended for example) select the immediate higher
    *  and shift the result one to the right.
    *
    *  Whenever you change the resolution, change also the comparison values (if you use them).
    */
    void setResolution(uint8_t bits);

    //! Returns the resolution of the ADC_Module.
    /**
    *   \return the resolution of the ADC_Module.
    */
    uint8_t getResolution();

    //! Returns the maximum value for a measurement: 2^res-1.
    /**
    *   \return the maximum value for a measurement: 2^res-1.
    */
    uint32_t getMaxValue();

    //! Sets the conversion speed (changes the ADC clock, ADCK)
    /**
    * param speed can be any from the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED, VERY_HIGH_SPEED,
    *       ADACK_2_4, ADACK_4_0, ADACK_5_2 or ADACK_6_2.
    *
    * VERY_LOW_SPEED is guaranteed to be the lowest possible speed within specs for resolutions less than 16 bits (higher than 1 MHz),
    * it's different from LOW_SPEED only for 24, 4 or 2 MHz bus frequency.
    * LOW_SPEED is guaranteed to be the lowest possible speed within specs for all resolutions (higher than 2 MHz).
    * MED_SPEED is always >= LOW_SPEED and <= HIGH_SPEED.
    * HIGH_SPEED_16BITS is guaranteed to be the highest possible speed within specs for all resolutions (lower or eq than 12 MHz).
    * HIGH_SPEED is guaranteed to be the highest possible speed within specs for resolutions less than 16 bits (lower or eq than 18 MHz).
    * VERY_HIGH_SPEED may be out of specs, it's different from HIGH_SPEED only for 48, 40 or 24 MHz bus frequency.
    *
    * Additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
    * where the numbers are the frequency of the ADC clock (ADCK) in MHz and are independent on the bus speed.
    * This is useful if you are using the Teensy at a very low clock frequency but want faster conversions,
    * but if F_BUS<F_ADCK, you can't use VERY_HIGH_SPEED for sampling speed.
    */
    void setConversionSpeed(ADC_CONVERSION_SPEED speed);

    //! Sets the sampling speed
    /** Increase the sampling speed for low impedance sources, decrease it for higher impedance ones.
    * param speed can be any of the ADC_SAMPLING_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED.
    *
    * VERY_LOW_SPEED is the lowest possible sampling speed (+24 ADCK).
    * LOW_SPEED adds +16 ADCK.
    * MED_SPEED adds +10 ADCK.
    * HIGH_SPEED adds +6 ADCK.
    * VERY_HIGH_SPEED is the highest possible sampling speed (0 ADCK added).
    */
    void setSamplingSpeed(ADC_SAMPLING_SPEED speed);

    //! Set the number of averages
    /*!
    * param num can be 0, 4, 8, 16 or 32.
    *
    *  It doesn't recalibrate at the end.
    */
    void setAveraging(uint8_t num);

    //! Enable interrupts
    /** An IRQ_ADCx Interrupt will be raised when the conversion is completed
    *  (including hardware averages and if the comparison (if any) is true).
    * param isr function (returns void and accepts no arguments) that will be executed after an interrupt.
    * param priority Interrupt priority, highest is 0, lowest is 255.
    */
    void enableInterrupts(void (*isr)(void), uint8_t priority = 255);

    //! Disable interrupts
    void disableInterrupts();

    //! Enable DMA request
    /** An ADC DMA request will be raised when the conversion is completed
    *  (including hardware averages and if the comparison (if any) is true).
    */
    void enableDMA();

    //! Disable ADC DMA request
    void disableDMA();

    //! Enable the compare function to a single value
    /** A conversion will be completed only when the ADC value
    *  is >= compValue (greaterThan=1) or < compValue (greaterThan=0)
    *  Call it after changing the resolution
    *  Use with interrupts or poll conversion completion with isComplete()
    *   \param compValue value to compare
    *   \param greaterThan true or false
    */
    void enableCompare(int16_t compValue, bool greaterThan);

    //! Enable the compare function to a range
    /** A conversion will be completed only when the ADC value is inside (insideRange=1) or outside (=0)
    *  the range given by (lowerLimit, upperLimit),including (inclusive=1) the limits or not (inclusive=0).
    *  See Table 31-78, p. 617 of the freescale manual.
    *  Call it after changing the resolution
    *  Use with interrupts or poll conversion completion with isComplete()
    *   \param lowerLimit lower value to compare
    *   \param upperLimit upper value to compare
    *   \param insideRange true or false
    *   \param inclusive true or false
    */
    void enableCompareRange(int16_t lowerLimit, int16_t upperLimit, bool insideRange, bool inclusive);

    //! Disable the compare function
    void disableCompare();

    //! Set continuous conversion mode
    void continuousMode() __attribute__((always_inline))
    {
        atomic::setBitFlag(adc_regs.GC, ADC_GC_ADCO);
    }
    //! Set single-shot conversion mode
    void singleMode() __attribute__((always_inline))
    {
        atomic::clearBitFlag(adc_regs.GC, ADC_GC_ADCO);
    }

    //! Use software to trigger the ADC, this is the most common setting
    void setSoftwareTrigger() __attribute__((always_inline))
    {
        atomic::clearBitFlag(adc_regs.CFG, ADC_CFG_ADTRG);
    }

    //! Use hardware to trigger the ADC
    void setHardwareTrigger() __attribute__((always_inline))
    {
        atomic::setBitFlag(adc_regs.CFG, ADC_CFG_ADTRG);
    }

    ////////////// INFORMATION ABOUT THE STATE OF THE ADC /////////////////

    //! Is the ADC converting at the moment?
    /**
    *   \return true or false
    */
    volatile bool isConverting() __attribute__((always_inline))
    {
        return atomic::getBitFlag(adc_regs.GS, ADC_GS_ADACT);
    }

    //! Is an ADC conversion ready?
    /**
    *  \return true if yes, false if not.
    *  When a value is read this function returns false until a new value exists,
    *  so it only makes sense to call it before analogReadContinuous() or readSingle()
    */
    volatile bool isComplete() __attribute__((always_inline))
    {
        return atomic::getBitFlag(adc_regs.HS, ADC_HS_COCO0);
    }

    //! Is the ADC in continuous mode?
    /**
    *   \return true or false
    */
    volatile bool isContinuous() __attribute__((always_inline))
    {
        return atomic::getBitFlag(adc_regs.GC, ADC_GC_ADCO);
    }

    //////////////// INFORMATION ABOUT VALID PINS //////////////////

    //! Check whether the pin is a valid analog pin
    /**
    *   \param pin to check.
    *   \return true if the pin is valid, false otherwise.
    */
    bool checkPin(uint8_t pin);

    //! Check whether the pins are a valid analog differential pair of pins
    /** If PGA is enabled it also checks that this ADCx can use PGA on this pins
    *   \param pinP positive pin to check.
    *   \param pinN negative pin to check.
    *   \return true if the pin is valid, false otherwise.
    */
    bool checkDifferentialPins(uint8_t pinP, uint8_t pinN);

    //////////////// HELPER METHODS FOR CONVERSION /////////////////

    //! Starts a single-ended conversion on the pin
    /** It sets the mux correctly, doesn't do any of the checks on the pin and
    *   doesn't change the continuous conversion bit.
    *   \param pin to read.
    */
    void startReadFast(uint8_t pin); // helper method

    //////////////// BLOCKING CONVERSION METHODS //////////////////

    //! Returns the analog value of the pin.
    /** It waits until the value is read and then returns the result.
    * If a comparison has been set up and fails, it will return ADC_ERROR_VALUE.
    * This function is interrupt safe, so it will restore the adc to the state it was before being called
    *   \param pin pin to read.
    *   \return the value of the pin.
    */
    int analogRead(uint8_t pin);

    //! Returns the analog value of the special internal source, such as the temperature sensor.
    /** It calls analogRead(uint8_t pin) internally, with the correct value for the pin for all boards.
    *   Possible values:
    *   TEMP_SENSOR,  Temperature sensor.
    *   VREF_OUT,  1.2 V reference (switch on first using VREF.h).
    *   BANDGAP, BANDGAP (switch on first using VREF.h).
    *   VREFH, High VREF.
    *   VREFL, Low VREF.
    *   \param pin ADC_INTERNAL_SOURCE to read.
    *   \return the value of the pin.
    */
    int analogRead(ADC_INTERNAL_SOURCE pin) __attribute__((always_inline))
    {
        return analogRead(static_cast<uint8_t>(pin));
    }

    /////////////// NON-BLOCKING CONVERSION METHODS //////////////

    //! Starts an analog measurement on the pin and enables interrupts.
    /** It returns immediately, get value with readSingle().
    *   If this function interrupts a measurement, it stores the settings in adc_config
    *   \param pin pin to read.
    *   \return true if the pin is valid, false otherwise.
    */
    bool startSingleRead(uint8_t pin);

    //! Reads the analog value of a single conversion.
    /** Set the conversion with with startSingleRead(pin) or startSingleDifferential(pinP, pinN).
    *   \return the converted value.
    */
    int readSingle() __attribute__((always_inline))
    {
        return analogReadContinuous();
    }

    ///////////// CONTINUOUS CONVERSION METHODS ////////////

    //! Starts continuous conversion on the pin.
    /** It returns as soon as the ADC is set, use analogReadContinuous() to read the value.
    *   \param pin can be any of the analog pins
    *   \return true if the pin is valid, false otherwise.
    */
    bool startContinuous(uint8_t pin);

    //! Reads the analog value of a continuous conversion.
    /** Set the continuous conversion with with analogStartContinuous(pin) or startContinuousDifferential(pinP, pinN).
    *   \return the last converted value.
    *   If single-ended and 16 bits it's necessary to typecast it to an unsigned type (like uint16_t),
    *   otherwise values larger than 3.3/2 V are interpreted as negative!
    */
    int analogReadContinuous() __attribute__((always_inline))
    {
        return (int16_t)(int32_t)adc_regs.R0;
    }

    //! Stops continuous conversion
    void stopContinuous();

//////////// FREQUENCY METHODS ////////
// The general API is:
// void startTimer(uint32_t freq)
// void stopTimer()
// uint32_t getTimerFrequency()
// For each board the best timer method will be selected

    //! Start the default timer (QuadTimer) triggering the ADC at the frequency
    /** The default timer in this board is the QuadTimer, you can also call it directly with startQuadTimer().
    *   Call startSingleRead or startSingleDifferential on the pin that you want to measure before calling this function.
    *   See the example adc_timer.ino.
    *   \param freq is the frequency of the ADC conversion, it can't be lower that 1 Hz
    */
    void startTimer(uint32_t freq) __attribute__((always_inline)) { startQuadTimer(freq); }
    //! Start a Quad timer to trigger the ADC at the frequency
    /** Call startSingleRead or startSingleDifferential on the pin that you want to measure before calling this function.
    *   See the example adc_timer.ino.
    *   \param freq is the frequency of the ADC conversion, it can't be lower that 1 Hz
    */
    void startQuadTimer(uint32_t freq);

    //! Stop the default timer (QuadTimer)
    void stopTimer() __attribute__((always_inline)) { stopQuadTimer(); }
    //! Stop the Quad timer
    void stopQuadTimer();

    //! Return the default timer's (QuadTimer) frequency
    /** The default timer in this board is the QuadTimer, you can also call it directly with getQuadTimerFrequency().
    *   \return the timer's frequency in Hz.
    */
    uint32_t getTimerFrequency() __attribute__((always_inline)) { return getQuadTimerFrequency(); }
    //! Return the Quad timer's frequency
    /** Return the Quad timer's frequency
    *   \return the timer's frequency in Hz.
    */
    uint32_t getQuadTimerFrequency();

    //////// OTHER STUFF ///////////

    //! Store the config of the adc
    struct ADC_Config
    {
        //! ADC registers
        uint32_t savedHC0, savedCFG, savedGC, savedGS;
    } adc_config;

    //! Was the adc in use before a call?
    uint8_t adcWasInUse;

    /** Save config of the ADC to the ADC_Config struct
    * \param config ADC_Config where the config will be stored
    */
    void saveConfig(ADC_Config *config)
    {
        config->savedHC0 = adc_regs.HC0;
        config->savedCFG = adc_regs.CFG;
        config->savedGC = adc_regs.GC;
        config->savedGS = adc_regs.GS;
    }

    /** Load config to the ADC
    * \param config ADC_Config from where the config will be loaded
    */
    void loadConfig(const ADC_Config *config)
    {
        adc_regs.HC0 = config->savedHC0;
        adc_regs.CFG = config->savedCFG;
        adc_regs.GC = config->savedGC;
        adc_regs.GS = config->savedGS;
    }

    //! Number of measurements that the ADC is performing
    uint8_t num_measurements;

    //! This flag indicates that some kind of error took place
    /** Use the defines at the beginning of this file to find out what caused the fail.
    */
    volatile ADC_ERROR fail_flag;

    //! Resets all errors from the ADC, if any.
    void resetError()
    {
        ADC_Error::resetError(fail_flag);
    }

    //! Which adc is this?
    const uint8_t ADC_num;

private:
    // is set to 1 when the calibration procedure is taking place
    uint8_t calibrating;

    // the first calibration will use 32 averages and lowest speed,
    // when this calibration is over the averages and speed will be set to default.
    uint8_t init_calib;

    // resolution
    uint8_t analog_res_bits;

    // maximum value possible 2^res-1
    uint32_t analog_max_val;

    // num of averages
    uint8_t analog_num_average;

    // reference can be internal or external
    ADC_REF_SOURCE analog_reference_internal;

    // conversion speed
    ADC_CONVERSION_SPEED conversion_speed;

    // sampling speed
    ADC_SAMPLING_SPEED sampling_speed;

    // translate pin number to SC1A nomenclature
    const uint8_t *const channel2sc1a;

    // are interrupts on?
    bool interrupts_enabled;

    //! Initialize ADC
    void analog_init();

    //! Switch on clock to ADC
    void startClock()
    {
        if (ADC_num == 0)
        {
            CCM_CCGR1 |= CCM_CCGR1_ADC1(CCM_CCGR_ON);
        }
        else
        {
            CCM_CCGR1 |= CCM_CCGR1_ADC2(CCM_CCGR_ON);
        }
    }

    // registers point to the correct ADC module
    typedef volatile uint32_t &reg;

    // registers that control the adc module
    ADC_REGS_t &adc_regs;

    uint8_t XBAR_IN;
    uint8_t XBAR_OUT;
    uint8_t QTIMER4_INDEX;
    uint8_t ADC_ETC_TRIGGER_INDEX;
    const IRQ_NUMBER_t IRQ_ADC; // IRQ number

protected:
};

#endif // ADC_MODULE_H
