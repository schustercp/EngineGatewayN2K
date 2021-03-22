#ifndef TC_LIB_H
#define TC_LIB_H

#include <cstdint>
#include <type_traits>
#include "Arduino.h"

namespace arduino_due
{
  namespace tc_lib
  {
    enum class timer_ids : uint32_t
    {
      TIMER_TC0 = 0,
      TIMER_TC1 = 1,
      TIMER_TC2 = 2,
      TIMER_TC3 = 3,
      TIMER_TC4 = 4,
      TIMER_TC5 = 5,
      TIMER_TC6 = 6,
      TIMER_TC7 = 7,
      TIMER_TC8 = 8,
    };

    template <timer_ids TIMER>
    struct tc_info
    {
    };

#define tc_info_specialization(timer_id, tc_x, channel_x)       \
  template <>                                                   \
  struct tc_info<                                               \
      timer_ids::TIMER_TC##timer_id>                            \
  {                                                             \
    static Tc *tc_p() { return tc_x; }                          \
    static constexpr const uint32_t channel = channel_x;        \
    static constexpr const IRQn_Type irq = TC##timer_id##_IRQn; \
  };

    tc_info_specialization(0, TC0, 0);
    tc_info_specialization(1, TC0, 1);
    tc_info_specialization(2, TC0, 2);
    tc_info_specialization(3, TC1, 0);
    tc_info_specialization(4, TC1, 1);
    tc_info_specialization(5, TC1, 2);
    tc_info_specialization(6, TC2, 0);
    tc_info_specialization(7, TC2, 1);
    tc_info_specialization(8, TC2, 2);

    template <timer_ids TIMER>
    struct tc_core
    {

      using info = tc_info<TIMER>;

      static void start_interrupts()
      {
        NVIC_ClearPendingIRQ(info::irq);
        NVIC_EnableIRQ(info::irq);
        TC_Start(info::tc_p(), info::channel);
      }

      static void stop_interrupts()
      {
        NVIC_DisableIRQ(info::irq);
        TC_Stop(info::tc_p(), info::channel);
      }

      static void config_interrupt() { NVIC_SetPriority(info::irq, 0); }
      static void enable_interrupts() { NVIC_EnableIRQ(info::irq); }
      static void disable_interrupts() { NVIC_DisableIRQ(info::irq); }

      static void enable_lovr_interrupt()
      {
        info::tc_p()->TC_CHANNEL[info::channel].TC_IER =
            TC_IER_LOVRS;
      }

      static bool is_enabled_lovr_interrupt()
      {
        return (
            info::tc_p()->TC_CHANNEL[info::channel].TC_IMR &
            TC_IMR_LOVRS);
      }

      static void disable_lovr_interrupt()
      {
        info::tc_p()->TC_CHANNEL[info::channel].TC_IDR =
            TC_IDR_LOVRS;
      }

      static void enable_ldra_interrupt()
      {
        info::tc_p()->TC_CHANNEL[info::channel].TC_IER =
            TC_IER_LDRAS;
      }

      static bool is_enabled_ldra_interrupt()
      {
        return (
            info::tc_p()->TC_CHANNEL[info::channel].TC_IMR &
            TC_IMR_LDRAS);
      }

      static void disable_ldra_interrupt()
      {
        info::tc_p()->TC_CHANNEL[info::channel].TC_IDR =
            TC_IDR_LDRAS;
      }

      static void enable_ldrb_interrupt()
      {
        info::tc_p()->TC_CHANNEL[info::channel].TC_IER =
            TC_IER_LDRBS;
      }

      static bool is_enabled_ldrb_interrupt()
      {
        return (
            info::tc_p()->TC_CHANNEL[info::channel].TC_IMR &
            TC_IMR_LDRBS);
      }

      static void disable_ldrb_interrupt()
      {
        info::tc_p()->TC_CHANNEL[info::channel].TC_IDR =
            TC_IDR_LDRBS;
      }

      static void enable_rc_interrupt()
      {
        info::tc_p()->TC_CHANNEL[info::channel].TC_IER =
            TC_IER_CPCS;
      }

      static bool is_enabled_rc_interrupt()
      {
        return (
            info::tc_p()->TC_CHANNEL[info::channel].TC_IMR &
            TC_IMR_CPCS);
      }

      static void disable_rc_interrupt()
      {
        info::tc_p()->TC_CHANNEL[info::channel].TC_IDR =
            TC_IDR_CPCS;
      }
    };

  }
}

#define capture_tc_declaration(id)                                           \
  void TC##id##_Handler(void)                                                \
  {                                                                          \
    uint32_t status = TC_GetStatus(                                          \
        arduino_due::tc_lib::tc_info<                                        \
            arduino_due::tc_lib::timer_ids::TIMER_TC##id>::tc_p(),           \
        arduino_due::tc_lib::tc_info<                                        \
            arduino_due::tc_lib::timer_ids::TIMER_TC##id>::channel);         \
                                                                             \
    arduino_due::tc_lib::capture<                                            \
        arduino_due::tc_lib::timer_ids::TIMER_TC##id>::tc_interrupt(status); \
  }                                                                          \
                                                                             \
  typedef arduino_due::tc_lib::capture<                                      \
      arduino_due::tc_lib::timer_ids::TIMER_TC##id>                          \
      capture_tc##id##_t;                                                    \
                                                                             \
  capture_tc##id##_t capture_tc##id;

#define capture_tc0_declaration() capture_tc_declaration(0)
#define capture_tc1_declaration() capture_tc_declaration(1)
#define capture_tc2_declaration() capture_tc_declaration(2)
#define capture_tc3_declaration() capture_tc_declaration(3)
#define capture_tc4_declaration() capture_tc_declaration(4)
#define capture_tc5_declaration() capture_tc_declaration(5)
#define capture_tc6_declaration() capture_tc_declaration(6)
#define capture_tc7_declaration() capture_tc_declaration(7)
#define capture_tc8_declaration() capture_tc_declaration(8)

#define capture_tc_declaration_with_callback(id)                     \
  void TC##id##_Handler(void)                                        \
  {                                                                  \
    uint32_t status = TC_GetStatus(                                  \
        arduino_due::tc_lib::tc_info<                                \
            arduino_due::tc_lib::timer_ids::TIMER_TC##id>::tc_p(),   \
        arduino_due::tc_lib::tc_info<                                \
            arduino_due::tc_lib::timer_ids::TIMER_TC##id>::channel); \
                                                                     \
    arduino_due::tc_lib::capture<                                    \
        arduino_due::tc_lib::timer_ids::TIMER_TC##id,                \
        true>::tc_interrupt(status);                                 \
  }                                                                  \
                                                                     \
  typedef arduino_due::tc_lib::capture<                              \
      arduino_due::tc_lib::timer_ids::TIMER_TC##id,                  \
      true>                                                          \
      capture_tc##id##_t;                                            \
                                                                     \
  capture_tc##id##_t capture_tc##id;

#define capture_tc0_declaration_with_callback() capture_tc_declaration_with_callback(0)
#define capture_tc1_declaration_with_callback() capture_tc_declaration_with_callback(1)
#define capture_tc2_declaration_with_callback() capture_tc_declaration_with_callback(2)
#define capture_tc3_declaration_with_callback() capture_tc_declaration_with_callback(3)
#define capture_tc4_declaration_with_callback() capture_tc_declaration_with_callback(4)
#define capture_tc5_declaration_with_callback() capture_tc_declaration_with_callback(5)
#define capture_tc6_declaration_with_callback() capture_tc_declaration_with_callback(6)
#define capture_tc7_declaration_with_callback() capture_tc_declaration_with_callback(7)
#define capture_tc8_declaration_with_callback() capture_tc_declaration_with_callback(8)

#define action_tc_declaration(id)                                            \
  void TC##id##_Handler(void)                                                \
  {                                                                          \
    uint32_t status = TC_GetStatus(                                          \
        arduino_due::tc_lib::tc_info<                                        \
            arduino_due::tc_lib::timer_ids::TIMER_TC##id>::tc_p(),           \
        arduino_due::tc_lib::tc_info<                                        \
            arduino_due::tc_lib::timer_ids::TIMER_TC##id>::channel);         \
                                                                             \
    arduino_due::tc_lib::action<                                             \
        arduino_due::tc_lib::timer_ids::TIMER_TC##id>::tc_interrupt(status); \
  }                                                                          \
                                                                             \
  typedef arduino_due::tc_lib::action<                                       \
      arduino_due::tc_lib::timer_ids::TIMER_TC##id>                          \
      action_tc##id##_t;                                                     \
                                                                             \
  action_tc##id##_t action_tc##id;

#define action_tc0_declaration() action_tc_declaration(0)
#define action_tc1_declaration() action_tc_declaration(1)
#define action_tc2_declaration() action_tc_declaration(2)
#define action_tc3_declaration() action_tc_declaration(3)
#define action_tc4_declaration() action_tc_declaration(4)
#define action_tc5_declaration() action_tc_declaration(5)
#define action_tc6_declaration() action_tc_declaration(6)
#define action_tc7_declaration() action_tc_declaration(7)
#define action_tc8_declaration() action_tc_declaration(8)

namespace arduino_due
{
  namespace tc_lib
  {
    template <bool BOOL_VALUE>
    struct bool_to_type
    {
      static constexpr bool value = BOOL_VALUE;
    };

    using callback_t = void (*)(void *);

    using period_callback_t = void (*)();

    template <bool BV>
    struct period_callback_wrapper
    {
      period_callback_wrapper(period_callback_t callback = NULL) {}
    };

    template <>
    struct period_callback_wrapper<true>
    {
      period_callback_wrapper(period_callback_t callback = NULL) : cllbck{callback} {}
      period_callback_t cllbck;

      void operator()()
      {
        if (cllbck)
          cllbck();
      }
    };

    template <timer_ids TIMER, bool WITH_CALLBACK = false>
    class capture
    {
    public:
      static constexpr const uint32_t DEFAULT_MAX_OVERRUNS = 100;

      capture() {}
      ~capture() {}
      capture(const capture &) = delete;
      capture(capture &&) = delete;
      capture &operator=(const capture &) = delete;
      capture &operator=(capture &&) = delete;

      static void tc_interrupt(uint32_t the_status)
      {
        _ctx_.tc_interrupt(the_status);
      }

      // NOTE: the_capture_window parameter in config() refers to
      // the time window for measuring the duty of a PWM signal. As
      // a rule of thumb if the PWM signal you want to measure has
      // a period T, the capture window should be at least twice this
      // time, that is, 2T. Parameter the_overruns specify how many
      // loading overruns are tolerated before ceasing capturing.
      bool config(uint32_t the_capture_window,
                  uint32_t the_overruns = DEFAULT_MAX_OVERRUNS,
                  period_callback_t period_callback = NULL)
      {
        return _ctx_.config(the_capture_window, the_overruns, period_callback);
      }

      constexpr uint32_t ticks_per_usec() { return _ctx_.ticks_per_usec(); }
      constexpr uint32_t max_capture_window() { return _ctx_.max_capture_window(); }

      // NOTE: member function get_duty_and_period() returns
      // the status of the capture object. Returns in arguments
      // the last valid measured and period. By default, the
      // capture object get restarted when stopped, setting
      // do_restart to false avoid restarting
      uint32_t get_duty_and_period(
          uint32_t &the_duty,
          uint32_t &the_period,
          bool do_restart = true)
      {
        return _ctx_.get_duty_and_period(
            the_duty,
            the_period,
            do_restart);
      }

      // NOTE: member function get_duty_period_and_pulses() returns
      // the status of the capture object. Returns in arguments
      // the last valid measured and period, and the count of
      // pulses accumulated since the last config. By default, the
      // capture object get restarted when stopped, setting
      // do_restart to false avoid restarting

      uint32_t get_duty_period_and_pulses(
          uint32_t &the_duty,
          uint32_t &the_period,
          uint32_t &the_pulses,
          bool do_restart = true)
      {
        return _ctx_.get_duty_period_and_pulses(
            the_duty,
            the_period,
            the_pulses,
            do_restart);
      }

      uint32_t get_capture_window() { return _ctx_.capture_window; }

      bool is_overrun() { return _ctx_.is_overrun(); }

      bool is_overrun(uint32_t the_status)
      {
        return _ctx_.is_overrun(the_status);
      }

      // NOTE: when too much loading overruns have been detected
      // the capture stops measuring to avoid the use of compu-
      // tational resources. Take into account that if the sig-
      // nal measured has a frequency around 1 Mhz or higher the
      // interrupts due to the capture object will consume all
      // CPU time. If this is the case, the capture object stops
      // capturing when the overun threshold is surpassed.
      bool is_stopped() { return _ctx_.is_stopped(); }

      bool is_stopped(uint32_t the_status)
      {
        return _ctx_.is_stopped(the_status);
      }

      // NOTE:capture object is unset when not configured
      bool is_unset() { return _ctx_.is_unset(); }

      bool is_unset(uint32_t the_status)
      {
        return _ctx_.is_unset(the_status);
      }

      void stop() { _ctx_.stop(); }
      void restart() { _ctx_.restart(); }

      void lock()
      {
        if (is_unset() || is_stopped())
          return;
        timer::disable_interrupts();
      }

      void unlock()
      {
        if (is_unset() || is_stopped())
          return;
        timer::enable_interrupts();
      }

    private:
      using timer = tc_core<TIMER>;

      struct _capture_ctx_ : period_callback_wrapper<WITH_CALLBACK>
      {
        enum status_codes : uint32_t
        {
          UNSET = 0,
          SET = 1,
          OVERRUN = 2,
          STOPPED = 4
        };

        _capture_ctx_() {}

        bool config(
            uint32_t the_capture_window,
            uint32_t the_overruns,
            period_callback_t period_cllbck = NULL)
        {
          return config_(
              the_capture_window,
              the_overruns,
              period_cllbck,
              bool_to_type<WITH_CALLBACK>());
        }

        bool config_(
            uint32_t the_capture_window,
            uint32_t the_overruns,
            period_callback_t period_cllbck,
            bool_to_type<false>);

        bool config_(
            uint32_t the_capture_window,
            uint32_t the_overruns,
            period_callback_t period_cllbck,
            bool_to_type<true>)
        {
          this->cllbck = period_cllbck;

          return config_(
              the_capture_window,
              the_overruns,
              period_cllbck,
              bool_to_type<false>());
        }

        void tc_interrupt(uint32_t the_status);

        static constexpr uint32_t ticks_per_usec()
        {
          // NOTE: we will be using the fastest clock for TC ticks
          // just using a prescaler of 2
          return static_cast<uint32_t>(((VARIANT_MCK) >> 1) / 1000000);
        }

        static constexpr uint32_t max_capture_window()
        {
          return static_cast<uint32_t>(
                     (static_cast<long long>(1) << 32) - static_cast<long long>(1)) /
                 ticks_per_usec();
        }

        uint32_t get_duty_and_period(
            uint32_t &the_duty,
            uint32_t &the_period,
            bool do_restart)
        {
          uint32_t the_status = status;
          if (is_unset(the_status))
            return the_status;

          timer::disable_interrupts();
          the_duty = duty;
          the_period = period;
          the_status = status;
          status = status & (~status_codes::OVERRUN);
          timer::enable_interrupts();

          if (is_stopped(the_status) && do_restart)
            restart();

          return the_status;
        }

        uint32_t get_duty_period_and_pulses(
            uint32_t &the_duty,
            uint32_t &the_period,
            uint32_t &the_pulses,
            bool do_restart)
        {
          uint32_t the_status = status;
          if (is_unset(the_status))
            return the_status;

          timer::disable_interrupts();
          the_duty = duty;
          the_period = period;
          the_pulses = pulses;
          the_status = status;
          status = status & (~status_codes::OVERRUN);
          timer::enable_interrupts();

          if (is_stopped(the_status) && do_restart)
            restart();

          return the_status;
        }

        bool is_overrun()
        {
          return (status & status_codes::OVERRUN);
        }

        bool is_overrun(uint32_t the_status)
        {
          return (the_status & status_codes::OVERRUN);
        }

        bool is_stopped()
        {
          return (status & status_codes::STOPPED);
        }

        bool is_stopped(uint32_t the_status)
        {
          return (the_status & status_codes::STOPPED);
        }

        bool is_unset() { return !status; }

        bool is_unset(uint32_t the_status) { return !the_status; }

        void stop()
        {
          uint32_t the_status = status;
          if (
              is_unset(the_status) ||
              is_stopped(the_status))
            return;

          timer::stop_interrupts();
          status = status | status_codes::STOPPED;
        }

        void restart()
        {
          uint32_t the_status = status;
          if (
              is_unset(the_status) ||
              !is_stopped(the_status))
            return;

          ra = duty = period = overruns = 0;
          status &=
              ~(status_codes::OVERRUN | status_codes::STOPPED);

          // clearing pending interrupt flags
          //uint32_t dummy=TC_GetStatus(
          TC_GetStatus(
              timer::info::tc_p(),
              timer::info::channel);
          timer::start_interrupts();
        }

        void end()
        {
          uint32_t the_status = status;
          if (is_unset(the_status))
            return;

          timer::stop_interrupts();
          timer::disable_lovr_interrupt();
          timer::disable_ldra_interrupt();
          timer::disable_ldrb_interrupt();
          timer::disable_rc_interrupt();
          pmc_disable_periph_clk(uint32_t(timer::info::irq));

          status = status_codes::UNSET;
        }

        void load_overrun()
        {
          status = status | status_codes::OVERRUN;
          if ((++overruns) > max_overruns)
          {
            timer::stop_interrupts();
            status = status | status_codes::STOPPED;
          }
        }

        void ra_loaded()
        {
          ra = timer::info::tc_p()->TC_CHANNEL[timer::info::channel].TC_RA;
        }

        void rb_loaded_(bool_to_type<false>)
        {
          period = timer::info::tc_p()->TC_CHANNEL[timer::info::channel].TC_RB;
          duty = period - ra;
          pulses++;
        }

        void rb_loaded_(bool_to_type<true>)
        {
          rb_loaded_(bool_to_type<false>());

          //calling this->cllbck() through base class
          (*this)();
        }

        void rb_loaded() { rb_loaded_(bool_to_type<WITH_CALLBACK>()); }

        void rc_matched() { ra = duty = period = 0; }

        // capture values
        volatile uint32_t ra;
        volatile uint32_t duty;
        volatile uint32_t period;
        volatile uint32_t pulses;
        volatile uint32_t overruns;
        volatile uint32_t status;

        uint32_t rc;
        uint32_t capture_window;
        uint32_t max_overruns;
      };

      static _capture_ctx_ _ctx_;
    };

    template <timer_ids TIMER, bool WITH_CALLBACK>
    typename capture<TIMER, WITH_CALLBACK>::_capture_ctx_ capture<TIMER, WITH_CALLBACK>::_ctx_;

    template <timer_ids TIMER, bool WITH_CALLBACK>
    bool capture<TIMER, WITH_CALLBACK>::_capture_ctx_::config_(
        uint32_t the_capture_window, // in microseconds
        uint32_t the_overruns,
        period_callback_t period_cllbck,
        bool_to_type<false>)
    {
      if ((the_capture_window > max_capture_window()) || !the_overruns)
        return false;

      capture_window = the_capture_window;
      ra = duty = period = pulses = overruns = 0;
      max_overruns = the_overruns;
      status = status_codes::SET;

      // capture window in ticks
      rc = capture_window * ticks_per_usec();

      // PMC settings
      pmc_set_writeprotect(0);
      pmc_enable_periph_clk(uint32_t(timer::info::irq));

      // timing setings in capture mode
      TC_Configure(
          timer::info::tc_p(),
          timer::info::channel,
          TC_CMR_TCCLKS_TIMER_CLOCK1 | // clock prescaler set to /2
              TC_CMR_CPCTRG |          // timer reset on RC match
              TC_CMR_LDRA_RISING |     // capture to RA on rising edge
              TC_CMR_LDRB_FALLING |    // capture to RB on falling edge
              TC_CMR_ETRGEDG_FALLING | // external trigger on falling edge
              TC_CMR_ABETRG            // external trigger on TIOA
      );

      // seting RC to the capture window
      TC_SetRC(timer::info::tc_p(), timer::info::channel, rc);

      timer::enable_lovr_interrupt();
      timer::enable_ldra_interrupt();
      timer::enable_ldrb_interrupt();
      timer::enable_rc_interrupt();

      timer::config_interrupt();
      timer::start_interrupts();

      return true;
    }

    template <timer_ids TIMER, bool WITH_CALLBACK>
    void capture<TIMER, WITH_CALLBACK>::_capture_ctx_::tc_interrupt(
        uint32_t the_status)
    {
      if ( // load overrun on RA or RB
          (the_status & TC_SR_LOVRS) &&
          timer::is_enabled_lovr_interrupt())
        load_overrun();

      if ( // RA loaded?
          (the_status & TC_SR_LDRAS) &&
          timer::is_enabled_ldra_interrupt())
        ra_loaded();

      if ( // RB loaded?
          (the_status & TC_SR_LDRBS) &&
          timer::is_enabled_ldrb_interrupt())
        rb_loaded();

      if ( // RC compare interrupt?
          (the_status & TC_SR_CPCS) &&
          timer::is_enabled_rc_interrupt())
        rc_matched();
    }

    template <timer_ids TIMER>
    class action
    {

    public:
      action() {}

      ~action() {}

      action(const action &) = delete;
      action(action &&) = delete;
      action &operator=(const action &) = delete;
      action &operator=(action &&) = delete;

      bool start(
          uint32_t the_period, // hundreths of usecs. (1e-8 secs.)
          callback_t the_callback,
          void *the_user_ctx)
      {
        return _ctx_.start(the_period, the_callback, the_user_ctx);
      }

      void stop() { _ctx_.stop(); }

      void lock() { timer::disable_tc_interrupts(); }
      void unlock() { timer::enable_tc_interrupts(); }

      constexpr uint32_t max_period() // hundreths of usecs.
      {
        return _ctx_.max_period();
      }

      // NOTE: get_period() returns 0 if the action is stopped
      uint32_t get_period() // hundreths of usecs. (1e-8 secs.)
      {
        return _ctx_.period;
      }

      uint32_t get_ticks()
      {
        return _ctx_.ticks(_ctx_.period);
      }

      constexpr uint32_t ticks(uint32_t period)
      {
        return _ctx_.ticks(period);
      }

      static void tc_interrupt(uint32_t the_status)
      {
        _ctx_.tc_interrupt(the_status);
      }

      using timer = tc_core<TIMER>;

    private:
      struct _action_ctx_
      {

        _action_ctx_() { init(); }

        void init()
        {
          period = 0;
          callback = [](void *dummy) {};
          user_ctx = nullptr;
        }

        void tc_interrupt(uint32_t the_status)
        {
          // RC compare interrupt
          if (
              (the_status & TC_SR_CPCS) &&
              timer::is_enabled_rc_interrupt())
          {
            callback(user_ctx);
          }
        }

        bool start(
            uint32_t the_period,
            callback_t the_callback,
            void *user_ctx);

        void stop()
        {
          timer::disable_rc_interrupt();

          timer::stop_interrupts();
          pmc_disable_periph_clk(
              static_cast<uint32_t>(timer::info::irq));

          init();
        }

        static constexpr uint32_t ticks(
            uint32_t period // hundreths of usecs (1e-8 secs.)
        )
        {
          // NOTE: we will be using the fastest clock for TC ticks
          // just using a prescaler of 2
          return static_cast<uint32_t>(
              static_cast<long long>(period) * static_cast<long long>(VARIANT_MCK >> 1) / static_cast<long long>(100000000));
        }

        static constexpr uint32_t ticks_per_usec()
        {
          return ticks(100);
        }

        static constexpr uint32_t max_period() // hundreths of usecs.
        {
          return static_cast<uint32_t>(
              (static_cast<long long>(1) << 32) - static_cast<long long>(1));
        }

        uint32_t period; // usecs.
        uint32_t rc;     // timer ticks

        callback_t callback;
        void *user_ctx;
      };

      static _action_ctx_ _ctx_;
    };

    template <timer_ids TIMER>
    typename action<TIMER>::_action_ctx_ action<TIMER>::_ctx_;

    template <timer_ids TIMER>
    bool action<TIMER>::_action_ctx_::start(
        uint32_t the_period, // hundreths of usecs. (1e-8 secs.)
        callback_t the_callback,
        void *the_user_ctx)
    {
      if (!the_callback || (the_period > max_period()))
        return false;

      period = the_period;
      callback = the_callback;
      user_ctx = the_user_ctx;

      // period in timer ticks
      rc = ticks(period);

      // PMC settings
      pmc_set_writeprotect(0);
      pmc_enable_periph_clk(uint32_t(timer::info::irq));

      // timing setings in capture mode
      TC_Configure(
          timer::info::tc_p(),
          timer::info::channel,
          TC_CMR_TCCLKS_TIMER_CLOCK1 | // clock prescaler set to /2
              TC_CMR_CPCTRG |          // timer reset on RC match
              TC_CMR_ETRGEDG_NONE      // no external trigger
      );

      // seting RC to the given period
      TC_SetRC(timer::info::tc_p(), timer::info::channel, rc);

      timer::enable_rc_interrupt();
      timer::config_interrupt();

      timer::start_interrupts();

      return true;
    }
  }
}

#endif // TC_LIB_H