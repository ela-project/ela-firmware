#ifndef ANALYZER_H
#define ANALYZER_H

#include <ela_protocol.h>
#include <stdlib.h>

#include "ela.h"

class Analyzer {
 public:
  Analyzer(){};

  /* Public: Types */
  typedef byte_pin_number pin_number_t;
  typedef byte_samplerate samplerate_t;
  typedef byte_sample_count sample_count_t;
  typedef byte_pretrig_count pretrig_count_t;
  typedef elap_pinmode_t pin_mode_t;

  static unsigned long temp_var;

  inline static bool is_running(void) {
    return m_running;
  };
  inline static sample_count_t get_sample_count(void) {
    return m_sample_count;
  };
  inline static pretrig_count_t get_pretrig_count(void) {
    return m_pretrig_count;
  };
  inline static sample_count_t get_postrig_count(void) {
    return m_postrig_count;
  };
  inline static samplerate_t get_samplerate(void) {
    return m_samplerate;
  };
  inline static pin_mode_t get_pin_mode(pin_number_t index) {
    if (index < NUM_OF_DIGITAL_PINS) {
      return m_pin_current_mode[index];
    } else {
      return PM_INVALID;
    }
  };

  /* Public: Member functions */
  static void init(void);
  static void idle(void);
  static void reset(void);
  static void arm_sampler(void);
  static void stop_sampler(void);
  static void finish_sampler(void);  // Called only by sampler
  static void set_samplerate(samplerate_t samplerate);
  static void set_sample_count(sample_count_t read_count);
  static void set_pretrig_count(pretrig_count_t pretrig_count);
  static ela_status_t set_pin_mode(pin_number_t pin_number, pin_mode_t trig_cond);
  static bool get_any_trigger_enabled(void);

 private:
  /* Private: Member variables */
  static samplerate_t m_samplerate;
  static sample_count_t m_postrig_count;
  static sample_count_t m_sample_count;
  static pretrig_count_t m_pretrig_count;
  static bool m_running;
  static bool m_signal_in_buffer;
  static pin_mode_t m_pin_current_mode[NUM_OF_DIGITAL_PINS];
};  // END class Analyzer

extern Analyzer analyzer_agent;

#endif