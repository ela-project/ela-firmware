#ifndef _HARDWARE_H
#define _HARDWARE_H

#include "analyzer.h"
#include "ela.h"
#include "stm32f303xe.h"

#define DEVICE_CLK 72000000U
#define TIM2_CLK 72000000U * 2
#define RX_BUFFER_SIZE 128
#define NUM_OF_EXTI 7
#define TIM_IT_ALL_FLAGS \
  (TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_TRIGGER)

#define __RETURN_IF_ERROR(x)                   \
  do {                                         \
    if ((x) != STATUS_OK) return STATUS_ERROR; \
  } while (0)

/* Class acts as a middle man between aplication and HAL */
class Hardware {
 public:
  Hardware(){};

  enum hw_state_t : uint8_t {
    HW_STATE_IDLE = 0x00U,
    HW_STATE_BUSY_SAMPLING = 0x01U,
    HW_STATE_SAMPLING_DONE = 0x02U,
  };

  struct hw_trigger_struct_t {
    bool enable;
    uint16_t gpio_pin;
    IRQn_Type irq;
  };

  /* Public: Member variables */
  static hw_trigger_struct_t m_trigger_struct[NUM_OF_DIGITAL_PINS];
  static uint8_t m_sample_buffer[SAMPLE_BUFFER_SIZE];
  static uint8_t m_rx_buffer[RX_BUFFER_SIZE];
  static arrat_index_t m_sample_buffer_start;
  static arrat_index_t m_sample_buffer_end;
  static arrat_index_t m_sample_buffer_count;
  static arrat_index_t m_sample_buffer_trig_index;
  static hw_state_t m_current_state;

  /* Public: Inline member functions */

  static inline uint8_t receive_byte(void) {  // Pop value out of RX serial buffer
    return rx_buffer_pop();
  };
  static inline void set_dma_did_more_cycles(void) {
    m_dma_more_cycles = true;
  }

  /* Public: Member functions */
  static int data_avalible(void);  // Data in RX serial buffer
  static void error_handler(void);
  static void init(void);
  static void reset(void);
  static ela_status_t send_byte(const uint8_t &data);
  static uint8_t rx_buffer_pop(void);
  static ela_status_t rx_buffer_push(uint8_t &data);
  static int num_of_data_in_rx_buffer();
  static void rx_buffer_clear(int number);
  static void toggle_LED(void);

  static ela_status_t start_sampling(void);
  static void abort_sampling(void);
  static void end_sampling(void);

#ifdef DEBUG
  /* Functions for debugging analyzer */
  static int read_registers_DMA(uint8_t *buffer);
  static int read_registers_TIM2(uint8_t *buffer);
  static int read_registers_TIM3(uint8_t *buffer);
#endif

 private:
  /* Private: Member variables */
  static bool m_data_in_rx_buffer;
  static int m_rx_buffer_start;
  static int m_rx_buffer_end;
  static bool m_dma_more_cycles;
  static bool m_any_trigger_enabled;

  /* Private: Member functions */
  static bool serial_writeable(void);
  static bool serial_readable(void);

  static ela_status_t hw_prepare_sample_counter(void);
  static ela_status_t hw_prepare_DMA_counter(void);
  static ela_status_t hw_prepare_trigger_it(void);
  static ela_status_t hw_enable_trigger_it(void);
  static ela_status_t hw_disable_all_triggers(void);

};  // END class Hardware

extern Hardware hw_agent;

#endif