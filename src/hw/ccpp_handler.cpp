#include "hw/ccpp_handler.h"

#include <stdint.h>

#include "ela.h"
#include "hardware.h"

/**
 * @brief Pushes byte to rx buffer of hw_agent
 * @param data byte to be pushed
 * @returns CCPP_STATUS_OK if push was sucessful, CCPP_STATUS_ERROR otherwise
 */
uint8_t ccpp_push_to_rx_buffer(uint8_t data) {
  if (hw_agent.rx_buffer_push(data) != STATUS_OK) {
    return CCPP_STATUS_ERROR;
  }
  return CCPP_STATUS_OK;
}

/**
 * @brief Tells hw_agent that sampling is done
 * @param None
 * @returns None
 */
void ccpp_sampling_done(void) {
  hw_agent.end_sampling();
}

/**
 * @brief Tells hw_agent that DMA did at least one cycle
 * @param None
 * @returns None
 */
void ccpp_dma_more_cycles(void) {
  hw_agent.set_dma_did_more_cycles();
}

/**
 * @brief Tells hw_agent last index written to by DMA
 * @param index index from DMA register
 * @returns None
 */
void ccpp_end_index(uint32_t index) {
  hw_agent.m_sample_buffer_end = SAMPLE_BUFFER_SIZE - 1 - index;
}