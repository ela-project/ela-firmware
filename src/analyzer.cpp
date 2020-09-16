
#include "analyzer.h"

#include "comms.h"
#include "hardware.h"
#include "hw/main.h"

Analyzer analyzer_agent;
/* Member variables */
Analyzer::samplerate_t Analyzer::m_samplerate{DEFAULT_SAMPLERATE};
Analyzer::sample_count_t Analyzer::m_postrig_count{0};
Analyzer::sample_count_t Analyzer::m_sample_count{DEFAULT_SAMPLE_COUNT};
Analyzer::pretrig_count_t Analyzer::m_pretrig_count{DEFAULT_PRETRIG_COUNT};
bool Analyzer::m_running{false};
bool Analyzer::m_signal_in_buffer{false};
Analyzer::pin_mode_t Analyzer::m_pin_current_mode[NUM_OF_DIGITAL_PINS];
unsigned long Analyzer::temp_var{0};

/* Member functions */

/**
 * @brief Initializes Firmware
 * @param None
 * @returns None
 */
void Analyzer::init(void) {
  for (int i{0}; i < NUM_OF_DIGITAL_PINS; i++) {
    m_pin_current_mode[i] = PM_DIGITAL_ON;
  }
  hw_agent.init();
  reset();
  comms_agent.init();
}

/**
 * @brief Idle function called in main loop
 * @param None
 * @returns None
 */
void Analyzer::idle(void) {
  comms_agent.handle_serial();
  if (m_running && hw_agent.m_current_state == Hardware::HW_STATE_SAMPLING_DONE) {
    finish_sampler();
  }
  temp_var++;
#ifdef DEBUG
  if (temp_var > 1000000) {
    temp_var = 0;
    hw_agent.toggle_LED();
  }
#endif
}

/**
 * @brief Resets logic analyzer to default values
 * @param None
 * @returns None
 */
void Analyzer::reset(void) {
  if (hw_agent.m_current_state == Hardware::HW_STATE_BUSY_SAMPLING) {
    hw_agent.abort_sampling();
  }
  m_running = false;
  m_samplerate = DEFAULT_SAMPLERATE;
  m_sample_count = DEFAULT_SAMPLE_COUNT;
  m_pretrig_count = DEFAULT_PRETRIG_COUNT;
  for (unsigned int i{0}; i < NUM_OF_DIGITAL_PINS; i++) {
    m_pin_current_mode[i] = PM_DIGITAL_ON;
  }
}

/**
 * @brief Turns on sampling of input signal
 * @param None
 * @returns None
 */
void Analyzer::arm_sampler(void) {
  m_running = true;
  m_signal_in_buffer = false;
  m_postrig_count = m_sample_count - m_pretrig_count;
  if (hw_agent.start_sampling() != STATUS_OK) {
    hw_agent.error_handler();
  }
}

/**
 * @brief Aborts sampling of input signal
 * @param None
 * @returns None
 */
void Analyzer::stop_sampler(void) {
  if (hw_agent.m_current_state == Hardware::HW_STATE_BUSY_SAMPLING) {
    hw_agent.abort_sampling();
  }
  return;
}

/**
 * @brief Sets the samplerate parameter of Logic analyzer
 * @param samplerate
 * @returns None
 */
void Analyzer::set_samplerate(samplerate_t samplerate) {
  if (samplerate > MAX_SAMPLERATE) {
    m_samplerate = MAX_SAMPLERATE;
  } else if (samplerate < MIN_SAMPLERATE) {
    m_samplerate = MIN_SAMPLERATE;
  } else {
    m_samplerate = samplerate;
  }
}

/**
 * @brief Sets the sample limit of Logic analyzer
 * @param sample_count
 * @returns None
 */
void Analyzer::set_sample_count(sample_count_t sample_count) {
  if (sample_count > MAX_SAMPLES) {
    m_sample_count = MAX_SAMPLES;
  } else if (sample_count < MIN_SAMPLES) {
    m_sample_count = MIN_SAMPLES;
  } else {
    m_sample_count = sample_count;
  }
}

/**
 * @brief Sets the amount of samples before trigger
 * @param pretrig_count
 * @returns None
 */
void Analyzer::set_pretrig_count(sample_count_t pretrig_count) {
  if (pretrig_count > MAX_PRETRIG_COUNT) {
    pretrig_count = MAX_PRETRIG_COUNT;
  } else {
    m_pretrig_count = pretrig_count;
  }
  return;
}

/**
 * @brief Sets the mode of a partucular pin
 * @param pin_number
 * @param pin_mode
 * @returns Status
 */
ela_status_t Analyzer::set_pin_mode(pin_number_t pin_number, pin_mode_t pin_mode) {
  if (pin_number >= NUM_OF_DIGITAL_PINS) {
    return STATUS_ERROR;
  } else if (pin_mode > PM_ENUM_END) {
    return STATUS_ERROR;
  } else {
    m_pin_current_mode[pin_number] = pin_mode;
  }
  return STATUS_OK;
}

/**
 * @brief Finishes sampling by sending data to pc and reseting hw_agent
 * @param None
 * @returns None
 */
void Analyzer::finish_sampler() {
  m_signal_in_buffer = true;
  comms_agent.report_sample_buffer();
  hw_agent.reset();
  m_running = false;
}

/**
 * @brief Checks if any pin has trigger enabled
 * @param None
 * @returns bool
 */
bool Analyzer::get_any_trigger_enabled(void) {
  for (pin_number_t i{0}; i < NUM_OF_DIGITAL_PINS; i++) {
    if (get_pin_mode(i) > PM_TRIGGER_BEGIN) {
      return true;
    }
  }
  return false;
}