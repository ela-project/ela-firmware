#ifndef COMMS_H
#define COMMS_H

#include <ela_protocol.h>

#include "ela.h"
#include "hardware.h"

#define _DIVIDER_TO_SAMPLERATE(x) (100000000 / ((x) + 1))

/* Class handles commands from PC based on communication protocol */
class Comms {
 public:
  Comms(){};

  enum comms_status_t : int8_t { IDLE_TYPE, IDLE_SUBTYPE, RECEIVING_NEW_COMMAND, COMMAND_RECEIVED };

  /* Public: Inline member functions */
  inline static int send_byte(const uint8_t &data) {
    return hw_agent.send_byte(data);
  };

  /* Public: Member functions */
  static void init();
  static void handle_serial();
  static int send_buffer(const uint8_t *data, const int length);
  static int send_buffer_circular(const uint8_t *data, const arrat_index_t offset,
                                  const arrat_index_t buffer_len, const arrat_index_t data_length);
  static int send_buffer(const char *data, bool send_last_symbol);
  static int report_sample_buffer(void);

  static uint64_t bytes_to_uint(uint8_t *buffer, int num_of_bytes, int buffer_offset);
  static int uint_to_bytes(uint8_t *buffer, uint16_t num, int buffer_offset);
  static int uint_to_bytes(uint8_t *buffer, uint32_t num, int buffer_offset);
  static int uint_to_bytes(uint8_t *buffer, uint64_t num, int buffer_offset);

 private:
  /* Private: Member variables */

  static comms_status_t m_comms_status;
  static elap_cmd_t m_elap_cmd;
  static uint8_t m_rx_tx_buffer[ELAP_CMD_MAX_SIZE];
  static uint32_t m_trig_val;
  static uint32_t m_trig_mask;
  static int m_data_to_receive;
  static int m_rx_tx_buffer_index;
  static byte_cmd_type temp_type;
  static byte_cmd_subtype temp_subtype;

  /* Private: Inline member functions */
  inline static int data_avalible() {
    return hw_agent.data_avalible();
  };
  inline static uint8_t receive_byte() {
    return hw_agent.receive_byte();
  };

  inline static void _uint_to_bytes(uint8_t *buffer, uint64_t num, int buffer_offset,
                                    int num_of_bytes) {
    for (int i{0}; i < num_of_bytes; i++) {
      buffer[i + buffer_offset] = (num >> ((num_of_bytes - 1 - i) * 8)) & 0xFFU;
    }
  };

#ifdef DEBUG
  static void report_settings();
#endif

};  // END class Comms

extern Comms comms_agent;

#endif