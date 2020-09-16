/* Functions to handle compatibility between CPP and C */
#ifndef _CCPP_HANDLER_H
#define _CCPP_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#define CCPP_STATUS_OK 0x00U
#define CCPP_STATUS_ERROR 0x01U

uint8_t ccpp_push_to_rx_buffer(uint8_t data);
void ccpp_sampling_done(void);
void ccpp_dma_more_cycles(void);
void ccpp_end_index(uint32_t index);

#ifdef __cplusplus
}
#endif

#endif