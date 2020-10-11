#include "hardware.h"

#include <stdint.h>

#include "analyzer.h"
#include "ela.h"
#include "hw/dma.h"
#include "hw/gpio.h"
#include "hw/main.h"
#include "hw/tim.h"
#include "hw/usart.h"

#ifdef DEBUG
#include "comms.h"
#endif

extern uint32_t trig_index;
Hardware hw_agent;

/* Member variables */
Hardware::hw_trigger_struct_t Hardware::m_trigger_struct[NUM_OF_DIGITAL_PINS] = {
    {false, GPIO_PIN_0, EXTI0_IRQn},     {false, GPIO_PIN_1, EXTI1_IRQn},
    {false, GPIO_PIN_2, EXTI2_TSC_IRQn}, {false, GPIO_PIN_3, EXTI3_IRQn},
    {false, GPIO_PIN_4, EXTI4_IRQn},     {false, GPIO_PIN_5, EXTI9_5_IRQn},
    {false, GPIO_PIN_6, EXTI9_5_IRQn},   {false, GPIO_PIN_7, EXTI9_5_IRQn}};
uint8_t Hardware::m_sample_buffer[SAMPLE_BUFFER_SIZE]{0};
arrat_index_t Hardware::m_sample_buffer_start{0};
arrat_index_t Hardware::m_sample_buffer_end{0};
arrat_index_t Hardware::m_sample_buffer_count{0};
arrat_index_t Hardware::m_sample_buffer_trig_index{0};

uint8_t Hardware::m_rx_buffer[RX_BUFFER_SIZE]{0};
bool Hardware::m_data_in_rx_buffer{false};
bool Hardware::m_dma_more_cycles{false};
bool Hardware::m_any_trigger_enabled{false};
int Hardware::m_rx_buffer_start{0};
int Hardware::m_rx_buffer_end{0};
Hardware::hw_state_t Hardware::m_current_state{Hardware::HW_STATE_IDLE};

/* Member functions */

/**
 * @brief Calls Error_Handler of HAL
 * @param None
 * @returns None
 */
void Hardware::error_handler(void) {
  Error_Handler();
}

/**
 * @brief Pops desired ammount of data from rx buffer
 * @param number number of bytes to pop
 * @returns None
 */
void Hardware::rx_buffer_clear(int number) {
  m_rx_buffer_start = (m_rx_buffer_start + number) % RX_BUFFER_SIZE;
  if (m_rx_buffer_start == m_rx_buffer_end) {
    m_data_in_rx_buffer = false;
  }
  return;
}

/**
 * @brief Checks the amount of bytes in rx buffer
 * @param None
 * @returns Amount of avalible bytes
 */
int Hardware::data_avalible(void) {  // Data in RX serial buffer
  int temp = m_rx_buffer_end - m_rx_buffer_start;
  if (temp < 0) {
    return RX_BUFFER_SIZE + temp;
  } else {
    return temp;
  }
}

/**
 * @brief Initializes HAL and HW Agent
 * @param None
 * @returns None
 */
void Hardware::init(void) {
  cube_main_init();
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
  m_sample_buffer_start = 0;
  m_sample_buffer_end = 0;
  m_sample_buffer_count = 0;
  m_sample_buffer_trig_index = 0;
  m_dma_more_cycles = false;
  m_current_state = HW_STATE_IDLE;
}

/**
 * @brief Resets HW agent
 * @param None
 * @returns None
 */
void Hardware::reset(void) {
  m_sample_buffer_start = 0;
  m_sample_buffer_end = 0;
  m_sample_buffer_count = 0;
  m_sample_buffer_trig_index = 0;
  m_dma_more_cycles = false;
  m_current_state = HW_STATE_IDLE;
  if (m_any_trigger_enabled) hw_disable_all_triggers();
}

/**
 * @brief Send byte using USART peripheral
 * @param data byte to send
 * @returns Status
 */
ela_status_t Hardware::send_byte(const uint8_t &data) {
  while (!serial_writeable()) {
  }
  huart2.Instance->TDR = data;
  return STATUS_OK;
}

/**
 * @brief Checks if USART peripheral has data avalible
 * @param None
 * @returns True or False
 */
bool Hardware::serial_readable(void) {
  /*  To avoid a target blocking case, let's check for
   *  possible OVERRUN error and discard it
   */
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
    __HAL_UART_CLEAR_OREFLAG(&huart2);
  }
  return ((__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET) ? true : false);
}

/**
 * @brief Checks if USART peripheral has sent all data
 * @param None
 * @returns True or False
 */
bool Hardware::serial_writeable(void) {
  // return (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) != RESET) ? true : false;
  return ((huart2.Instance->ISR & USART_ISR_TXE) ? true : false);
}

/**
 * @brief Starts sampling using pheripherals
 * @param None
 * @returns Status
 */
ela_status_t Hardware::start_sampling(void) {
  m_current_state = HW_STATE_BUSY_SAMPLING;
  m_dma_more_cycles = false;
  if (analyzer_agent.get_any_trigger_enabled()) {
    hw_prepare_trigger_it();
  }
  __RETURN_IF_ERROR(hw_prepare_sample_counter());
  __RETURN_IF_ERROR(hw_prepare_DMA_counter());
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  if (!m_any_trigger_enabled) {
    __HAL_TIM_ENABLE(&htim3);
  }
  HAL_DMA_Start(&hdma_tim2_up, ((uint32_t) & (GPIOC->IDR)), (uint32_t)m_sample_buffer,
                SAMPLE_BUFFER_SIZE);
  HAL_TIM_Base_Start(&htim2);
  if (m_any_trigger_enabled) hw_enable_trigger_it();
  return STATUS_OK;
}

/**
 * @brief Aborts sampling
 * @param None
 * @returns None
 */
void Hardware::abort_sampling(void) {
  if (m_any_trigger_enabled) hw_disable_all_triggers();
  __HAL_TIM_DISABLE(&htim2);
  __HAL_TIM_DISABLE(&htim3);
  __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
  HAL_DMA_Abort(&hdma_tim2_up);
  reset();
}

/**
 * @brief Finish succesfuul sampling
 * @param None
 * @returns None
 */
void Hardware::end_sampling(void) {
  uint32_t trig_index_calc = SAMPLE_BUFFER_SIZE - trig_index;
  if (m_any_trigger_enabled) {
    hw_disable_all_triggers();
    if (!m_dma_more_cycles && analyzer_agent.get_sample_count() > m_sample_buffer_end) {
      m_sample_buffer_start = 0;
      if (m_sample_buffer_end >= analyzer_agent.get_postrig_count()) {
        m_sample_buffer_trig_index = trig_index_calc;
        // m_sample_buffer_trig_index = m_sample_buffer_end - analyzer_agent.get_postrig_count();
      } else {
        m_sample_buffer_trig_index = 0;
      }

      m_sample_buffer_count = m_sample_buffer_end;
    } else {
      /*if (m_sample_buffer_end < analyzer_agent.get_postrig_count()) {
        m_sample_buffer_trig_index =
            SAMPLE_BUFFER_SIZE - (analyzer_agent.get_postrig_count() - m_sample_buffer_end);
      } else {
        m_sample_buffer_trig_index = m_sample_buffer_end - analyzer_agent.get_postrig_count();
      }*/
      if (m_sample_buffer_end < analyzer_agent.get_sample_count()) {
        m_sample_buffer_start =
            SAMPLE_BUFFER_SIZE - (analyzer_agent.get_sample_count() - m_sample_buffer_end);
      } else {
        m_sample_buffer_start = m_sample_buffer_end - analyzer_agent.get_sample_count();
      }
      if (trig_index_calc >= m_sample_buffer_start)
        m_sample_buffer_trig_index = trig_index_calc - m_sample_buffer_start;
      else
        m_sample_buffer_trig_index =
            (SAMPLE_BUFFER_SIZE - (m_sample_buffer_start - trig_index_calc));
      m_sample_buffer_count = analyzer_agent.get_sample_count();
    }
  } else {
    m_sample_buffer_start = 0;
    m_sample_buffer_trig_index = 0;
    m_sample_buffer_count = m_sample_buffer_end;
  }
  hw_agent.m_current_state = Hardware::HW_STATE_SAMPLING_DONE;
}

/**
 * @brief Pop single byte from rx buffer
 * @param None
 * @returns Poped byte
 */
uint8_t Hardware::rx_buffer_pop(void) {
  if (m_rx_buffer_start == m_rx_buffer_end) {
    return 0xFF;
  }
  uint8_t ret = m_rx_buffer[m_rx_buffer_start];
  m_rx_buffer_start = (m_rx_buffer_start + 1) % RX_BUFFER_SIZE;
  if (m_rx_buffer_start == m_rx_buffer_end) {
    m_data_in_rx_buffer = false;
  }
  return ret;
}

/**
 * @brief Push byte to rx buffer
 * @param data byte to push
 * @returns Status
 */
ela_status_t Hardware::rx_buffer_push(uint8_t &data) {
  int temp = m_rx_buffer_end;
  m_rx_buffer_end = (m_rx_buffer_end + 1) % RX_BUFFER_SIZE;
  if (m_rx_buffer_end == m_rx_buffer_start) {
    m_rx_buffer_end = temp;
    return STATUS_ERROR;
  }
  m_rx_buffer[temp] = data;
  m_data_in_rx_buffer = true;
  return STATUS_OK;
}

/**
 * @brief Toggles Nucleo LED
 * @param None
 * @returns None
 */
void Hardware::toggle_LED(void) {
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

/**
 * @brief Prepares Timer 3 for counting samples
 * @param None
 * @returns Status
 */
ela_status_t Hardware::hw_prepare_sample_counter(void) {
  __HAL_TIM_URS_ENABLE(&htim3);
  if (m_any_trigger_enabled) {
    __HAL_TIM_SET_COUNTER(&htim3, (uint32_t)(analyzer_agent.get_pretrig_count() & 0xFFFFU));
  } else {
    __HAL_TIM_SET_COUNTER(&htim3, (uint32_t)(0x0U));
  }
  __HAL_TIM_SET_AUTORELOAD(&htim3, (uint32_t)(analyzer_agent.get_sample_count() & 0xFFFFU));
  __HAL_TIM_SET_PRESCALER(&htim3, (uint16_t)(0x00U));
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_ALL_FLAGS);
  return STATUS_OK;
}

/**
 * @brief Prepares Timer 2 for sampling
 * @param None
 * @returns Status
 */
ela_status_t Hardware::hw_prepare_DMA_counter(void) {
  Analyzer::samplerate_t samplerate{analyzer_agent.get_samplerate()};
  uint32_t arr = (TIM2_CLK / samplerate) - 1;
  __HAL_TIM_SET_COUNTER(&htim2, (uint32_t)0x0U);
  __HAL_TIM_SET_AUTORELOAD(&htim2, (uint32_t)(arr & 0xFFFFFFFFU));
  __HAL_TIM_SET_PRESCALER(&htim2, (uint16_t)0x00U);
  return STATUS_OK;
}

/**
 * @brief Prepares GPIO for EXTI interrupts
 * @param None
 * @returns Status
 */
ela_status_t Hardware::hw_prepare_trigger_it(void) {
  Analyzer::pin_number_t i;
  Analyzer::pin_mode_t pin_trig_cond;
  GPIO_InitTypeDef GPIO_InitStruct{0};

  for (i = 0; i < NUM_OF_DIGITAL_PINS; i++) {
    pin_trig_cond = analyzer_agent.get_pin_mode(i);
    GPIO_InitStruct.Pin = m_trigger_struct[i].gpio_pin;

    if (pin_trig_cond >= Analyzer::pin_mode_t::PM_TRIGGER_BEGIN &&
        pin_trig_cond <= Analyzer::pin_mode_t::PM_ENUM_END) {
      if (pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_RISING ||
          pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_HIGH) {
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
      } else if (pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_FALLING ||
                 pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_LOW) {
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
      } else if (pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_BOTH) {
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
      }
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
      m_any_trigger_enabled = true;
      m_trigger_struct[i].enable = true;
      break;
    }
  }

  /*  Analyzer::pin_number_t pin_num{0};
    Analyzer::pin_mode_t pin_trig_cond{analyzer_agent.get_pin_mode(pin_num)};
    if (pin_trig_cond >= Analyzer::pin_mode_t::PM_TRIGGER_BEGIN &&
        pin_trig_cond <= Analyzer::pin_mode_t::PM_ENUM_END) {
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = GPIO_PIN_0;
      if (pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_RISING ||
          pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_HIGH) {
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
      } else if (pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_FALLING ||
                 pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_LOW) {
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
      } else if (pin_trig_cond == Analyzer::pin_mode_t::PM_TRIGGER_BOTH) {
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
      }
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
      m_any_trigger_enabled = true;
      m_trigger_struct[0].enable = true;
    }*/

  __HAL_DMA_ENABLE_IT(&hdma_tim2_up, DMA_IT_TC);
  return STATUS_OK;
}

/**
 * @brief Disables all enabled EXTI interrupts
 * @param None
 * @returns Status
 */
ela_status_t Hardware::hw_disable_all_triggers(void) {
  for (int i{0}; i < NUM_OF_EXTI; i++) {
    if (m_trigger_struct[i].enable) {
      // HAL_NVIC_DisableIRQ(m_trigger_struct[i].irq);
      m_trigger_struct[i].enable = false;
    }
  }
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
                        GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  __HAL_DMA_DISABLE_IT(&hdma_tim2_up, DMA_IT_TC);
  m_any_trigger_enabled = false;
  return STATUS_OK;
}

/**
 * @brief Enables EXTI interrupts for trigger
 * @param None
 * @returns Status
 */
ela_status_t Hardware::hw_enable_trigger_it(void) {
  uint16_t i;
  for (i = 0; i < NUM_OF_DIGITAL_PINS; i++) {
    if (m_trigger_struct[i].enable) {
      __HAL_GPIO_EXTI_CLEAR_IT(m_trigger_struct[i].gpio_pin);
      HAL_NVIC_ClearPendingIRQ(m_trigger_struct[i].irq);
      HAL_NVIC_SetPriority(m_trigger_struct[i].irq, 0, 2);
      HAL_NVIC_EnableIRQ(m_trigger_struct[i].irq);
      __HAL_GPIO_EXTI_CLEAR_IT(m_trigger_struct[i].gpio_pin);
    }
  }
  return STATUS_OK;
}

#ifdef DEBUG
int Hardware::read_registers_DMA(uint8_t *buffer) {
  uint32_t reg{0};
  int index{0};
  buffer[index] = 0xFFU;
  index += 1;
  reg = hdma_tim2_up.Instance->CNDTR & 0xFFFFFFFFU;
  index = comms_agent.uint_to_bytes(buffer, reg, index);
  buffer[index] = 0xFFU;
  index += 1;
  reg = hdma_tim2_up.Instance->CPAR & 0xFFFFFFFFU;
  index = comms_agent.uint_to_bytes(buffer, reg, index);
  buffer[index] = 0xFFU;
  index += 1;
  reg = hdma_tim2_up.Instance->CMAR & 0xFFFFFFFFU;
  index = comms_agent.uint_to_bytes(buffer, reg, index);
  buffer[index] = 0xFFU;
  index += 1;
  buffer[index] = '\n';
  index += 1;
  return index;
}

int Hardware::read_registers_TIM2(uint8_t *buffer) {
  uint32_t reg{0};
  int index{0};
  buffer[index] = 0xFEU;
  index += 1;
  reg = htim2.Instance->CNT & 0xFFFFFFFFU;
  index = comms_agent.uint_to_bytes(buffer, reg, index);
  buffer[index] = 0xEEU;
  index += 1;
  reg = htim2.Instance->ARR & 0xFFFFFFFFU;
  index = comms_agent.uint_to_bytes(buffer, reg, index);
  buffer[index] = 0xEEU;
  index += 1;
  reg = htim2.Instance->PSC & 0xFFFFFFFFU;
  index = comms_agent.uint_to_bytes(buffer, reg, index);
  buffer[index] = 0xEEU;
  index += 1;
  reg = htim2.Instance->CR1 & 0xFFFFFFFFU;
  index = comms_agent.uint_to_bytes(buffer, reg, index);
  buffer[index] = 0xEEU;
  index += 1;
  reg = htim2.Instance->DIER & 0xFFFFFFFFU;
  index = comms_agent.uint_to_bytes(buffer, reg, index);
  buffer[index] = '\n';
  index += 1;
  return index;
}

int Hardware::read_registers_TIM3(uint8_t *buffer) {
  uint32_t reg{0};
  int index{0};
  buffer[index] = 0xFDU;
  index += 1;
  reg = htim3.Instance->CNT & 0xFFFFFFFFU;
  index = comms_agent.uint_to_bytes(buffer, reg, index);
  buffer[index] = 0xDDU;
  index += 1;
  reg = htim3.Instance->ARR & 0xFFFFFFFFU;
  index = comms_agent.uint_to_bytes(buffer, reg, index);
  buffer[index] = 0xDDU;
  index += 1;
  reg = htim3.Instance->PSC & 0xFFFFFFFFU;
  index = comms_agent.uint_to_bytes(buffer, reg, index);
  buffer[index] = 0xDDU;
  index += 1;
  reg = htim3.Instance->CR1 & 0xFFFFFFFFU;
  index = comms_agent.uint_to_bytes(buffer, reg, index);
  buffer[index] = 0xDDU;
  index += 1;
  reg = htim3.Instance->DIER & 0xFFFFFFFFU;
  index = comms_agent.uint_to_bytes(buffer, reg, index);
  buffer[index] = 0xDDU;
  index += 1;
  buffer[index] = '\n';
  index += 1;
  return index;
}
#endif