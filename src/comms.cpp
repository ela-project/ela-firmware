#include "comms.h"

#include "analyzer.h"

Comms comms_agent;

/* Member variables */
int Comms::m_rx_tx_buffer_index{0};
int Comms::m_data_to_receive{0};
uint8_t Comms::m_rx_tx_buffer[ELAP_CMD_MAX_SIZE]{0};
uint32_t Comms::m_trig_val{0};
uint32_t Comms::m_trig_mask{0};
elap_cmd_t Comms::m_elap_cmd;
Comms::comms_status_t Comms::m_comms_status{Comms::comms_status_t::IDLE_TYPE};
byte_cmd_type Comms::temp_type{0};
byte_cmd_subtype Comms::temp_subtype{0};
bool bagr{false};

/* Member functions */

/**
 * @brief Send byte buffer to serial
 * @param length amount of data
 * @param *data pointer to byte buffer
 * @returns Amount of data sent
 */
int Comms::send_buffer(const uint8_t *data, const int length) {
  for (int i{0}; i < length; i++) {
    if (send_byte(data[i])) return i;
  }
  return 0;
}

/**
 * @brief Send byte buffer to serial in circular mode
 * @param *data pointer to byte buffer
 * @param offset Starting index
 * @param buffer_len Size of byte buffer
 * @param data_length Amount of data
 * @returns Amount of data sent
 */
int Comms::send_buffer_circular(const uint8_t *data, const arrat_index_t offset,
                                const arrat_index_t buffer_len, const arrat_index_t data_length) {
  for (arrat_index_t i{offset}; i < offset + data_length; i++) {
    if (send_byte(data[i % buffer_len])) return i;
  }
  return 0;
}

/**
 * @brief Send string to serial
 * @param *data pointer to C style string
 * @param send_last_symbol Include \0 symbol
 * @returns Amount of data sent
 */
int Comms::send_buffer(const char *data, bool send_last_symbol = true) {
  int i{0};
  while (data[i] != '\0') {
    send_byte(data[i]);
    i++;
  }
  if (send_last_symbol) {
    send_byte(data[i]);
  }
  return 0;
}

/**
 * @brief Convert byte to unsigned intieger
 * @param *buffer pointer to byte buffer
 * @param num_of_bytes Number of bytes to convert
 * @param buffer_offset index of byte buffer to convert from, default = 0
 * @returns Unsigned intieger
 */
uint64_t Comms::bytes_to_uint(uint8_t *buffer, int num_of_bytes, int buffer_offset = 0) {
  return elap_bytes_to_uint(buffer, num_of_bytes, &buffer_offset);
}

/**
 * @brief Convert unsigned intieger to byte buffer
 * @param *buffer pointer to byte buffer
 * @param num Number to convert
 * @param buffer_offset index of byte buffer to start filling from, default = 0
 * @returns Last index of buffer that has been read
 */
int Comms::uint_to_bytes(uint8_t *buffer, uint16_t num, int buffer_offset = 0) {
  unsigned int num_of_bytes{sizeof(uint16_t)};
  elap_uint_to_bytes(num, buffer, num_of_bytes, &buffer_offset);
  //_uint_to_bytes(buffer, num, buffer_offset, num_of_bytes);
  // return buffer_offset + num_of_bytes;
  return buffer_offset;
}

int Comms::uint_to_bytes(uint8_t *buffer, uint32_t num, int buffer_offset = 0) {
  unsigned int num_of_bytes{sizeof(uint32_t)};
  elap_uint_to_bytes(num, buffer, num_of_bytes, &buffer_offset);
  //_uint_to_bytes(buffer, num, buffer_offset, num_of_bytes);
  // return buffer_offset + num_of_bytes;
  return buffer_offset;
}

int Comms::uint_to_bytes(uint8_t *buffer, uint64_t num, int buffer_offset = 0) {
  unsigned int num_of_bytes{sizeof(uint64_t)};
  elap_uint_to_bytes(num, buffer, num_of_bytes, &buffer_offset);
  //_uint_to_bytes(buffer, num, buffer_offset, num_of_bytes);
  // return buffer_offset + num_of_bytes;
  return buffer_offset;
}

/**
 * @brief Initializes Comms Agent
 * @param None
 * @returns None
 */
void Comms::init() {
  temp_type = 0;
  temp_subtype = 0;
  m_comms_status = IDLE_TYPE;
  m_rx_tx_buffer_index = 0;
  m_data_to_receive = 0;
#ifdef DEBUG
  send_buffer("Hello There\n");
#endif
}

/**
 * @brief Handle commands sent from PC
 * @param None
 * @returns None
 */
void Comms::handle_serial() {
  switch (m_comms_status) {
    case IDLE_TYPE:
      if (data_avalible() >= ELAP_CMD_TYPE_SIZE) {
        m_rx_tx_buffer_index = 0;
        temp_type = 0;
        temp_subtype = 0;
        m_data_to_receive = 0;
        for (int i{0}; i < ELAP_CMD_TYPE_SIZE; i++) {
          m_rx_tx_buffer[i] = receive_byte();
          m_rx_tx_buffer_index++;
        }
        temp_type = bytes_to_uint(m_rx_tx_buffer, ELAP_CMD_TYPE_SIZE);
        int ret = elap_has_subtype_raw(temp_type);
        if (ret == 0) {
          m_data_to_receive = elap_bytes_in_cmd_raw(temp_type, 0);
          m_comms_status = RECEIVING_NEW_COMMAND;
        } else if (ret == 1) {
          m_comms_status = IDLE_SUBTYPE;
        }
#ifdef DEBUG
        else {
          send_buffer("Unknown type\n");
        }
#endif
      }
      break;  // case IDLE_TYPE
    case IDLE_SUBTYPE:
      if (data_avalible() >= ELAP_CMD_SUBTYPE_SIZE) {
        for (int i{0}; i < ELAP_CMD_TYPE_SIZE; i++) {
          m_rx_tx_buffer[m_rx_tx_buffer_index] = receive_byte();
          m_rx_tx_buffer_index++;
        }
        temp_subtype = bytes_to_uint(m_rx_tx_buffer, ELAP_CMD_SUBTYPE_SIZE, ELAP_CMD_TYPE_SIZE);
        int ret = elap_bytes_in_cmd_raw(temp_type, temp_subtype);
        if (ret <= ELAP_RET_FAIL) {
#ifdef DEBUG
          send_buffer("Unknown subtype\n");
#endif
          m_comms_status = IDLE_TYPE;
        } else {
          m_data_to_receive = ret;
          m_comms_status = RECEIVING_NEW_COMMAND;
        }
      }
      break;  // case IDLE_SUBTYPE
    case RECEIVING_NEW_COMMAND:
      if (m_data_to_receive == 0 || data_avalible() >= m_data_to_receive) {
        if (m_data_to_receive > 0) {
          for (int i{0}; i < m_data_to_receive; i++) {
            m_rx_tx_buffer[m_rx_tx_buffer_index] = receive_byte();
            m_rx_tx_buffer_index++;
          }
        }
        int ret = elap_packet_to_cmd(&m_elap_cmd, m_rx_tx_buffer, 0);
        if (ret <= ELAP_RET_FAIL) {
#ifdef DEBUG
          send_buffer("CMD Failed\n");
#endif
          m_comms_status = IDLE_TYPE;
        } else {
          m_comms_status = COMMAND_RECEIVED;
        }
      }
      break;  // case RECEIVING_NEW_COMMAND
    case COMMAND_RECEIVED:
      switch (m_elap_cmd.type) {
        case CMD_RESET:
          analyzer_agent.reset();
          break;
        case CMD_HANDSHAKE:
          send_buffer(ELAP_HANDSHAKE_REPLY);
          break;

        case CMD_START:
          analyzer_agent.arm_sampler();
          break;
        case CMD_STOP:
          analyzer_agent.stop_sampler();
          break;
        case CMD_SET:
          switch (m_elap_cmd.subtype) {
            case SUB_SAMPLERATE:
              analyzer_agent.set_samplerate(m_elap_cmd.data.samplerate);
              break;
            case SUB_SAMPLE_COUNT:
              analyzer_agent.set_sample_count(m_elap_cmd.data.sample_cout);
              break;
            case SUB_PRETRIG_COUNT:
              analyzer_agent.set_pretrig_count(m_elap_cmd.data.pretrig_count);
              break;
            case SUB_PIN_MODE:
              analyzer_agent.set_pin_mode(m_elap_cmd.data.pin_mode.number,
                                          m_elap_cmd.data.pin_mode.mode);
              break;
            default:
              break;
          }
          break;  // case CMD_SET

        case CMD_GET:
          switch (m_elap_cmd.subtype) {
            case SUB_SAMPLERATE:
              m_elap_cmd.type = CMD_REPORT;
              m_elap_cmd.subtype = SUB_SAMPLERATE;
              m_elap_cmd.data.samplerate = analyzer_agent.get_samplerate();
              m_rx_tx_buffer_index = elap_cmd_to_packet(&m_elap_cmd, m_rx_tx_buffer, 0);
              send_buffer(m_rx_tx_buffer, m_rx_tx_buffer_index);
              break;
            case SUB_SAMPLE_COUNT:
              m_elap_cmd.type = CMD_REPORT;
              m_elap_cmd.subtype = SUB_SAMPLE_COUNT;
              m_elap_cmd.data.sample_cout = analyzer_agent.get_sample_count();
              m_rx_tx_buffer_index = elap_cmd_to_packet(&m_elap_cmd, m_rx_tx_buffer, 0);
              send_buffer(m_rx_tx_buffer, m_rx_tx_buffer_index);
              break;
            case SUB_PRETRIG_COUNT:
              m_elap_cmd.type = CMD_REPORT;
              m_elap_cmd.subtype = SUB_PRETRIG_COUNT;
              m_elap_cmd.data.pretrig_count = analyzer_agent.get_pretrig_count();
              m_rx_tx_buffer_index = elap_cmd_to_packet(&m_elap_cmd, m_rx_tx_buffer, 0);
              send_buffer(m_rx_tx_buffer, m_rx_tx_buffer_index);
              break;
            case SUB_PIN_MODE: {
              byte_pin_number temp_pin_num = m_elap_cmd.data.pin_mode.number;
              m_elap_cmd.type = CMD_REPORT;
              m_elap_cmd.subtype = SUB_PIN_MODE;
              m_elap_cmd.data.pin_mode.number = temp_pin_num;
              m_elap_cmd.data.pin_mode.mode = analyzer_agent.get_pin_mode(temp_pin_num);
              m_rx_tx_buffer_index = elap_cmd_to_packet(&m_elap_cmd, m_rx_tx_buffer, 0);
              send_buffer(m_rx_tx_buffer, m_rx_tx_buffer_index);
            } break;
            case SUB_METADATA:
              m_elap_cmd.type = CMD_REPORT;
              m_elap_cmd.subtype = SUB_METADATA;
              m_elap_cmd.data.metadata.max_sample_cout = MAX_SAMPLES;
              m_elap_cmd.data.metadata.max_samplerate = MAX_SAMPLERATE;
              m_elap_cmd.data.metadata.numof_pins = NUM_OF_DIGITAL_PINS;
              m_elap_cmd.data.metadata.name = (char *)DEVICE_NAME;
              m_rx_tx_buffer_index = elap_cmd_to_packet(&m_elap_cmd, m_rx_tx_buffer, 0);
              send_buffer(m_rx_tx_buffer, m_rx_tx_buffer_index);
              send_buffer(m_elap_cmd.data.metadata.name);
              break;
            case SUB_SAMPLED_DATA:
#ifdef DEBUG
              send_buffer("BAGR\n");
#endif
              report_sample_buffer();
              break;
            default:
              break;
          }
          break;  // case CMD_GET
#ifdef ELAP_DBG_CMDS
        case CMD_DBG:
          m_rx_tx_buffer_index = hw_agent.read_registers_DMA(m_tx_buffer);
          send_buffer(m_tx_buffer, m_rx_tx_buffer_index);
          m_rx_tx_buffer_index = hw_agent.read_registers_TIM2(m_tx_buffer);
          send_buffer(m_tx_buffer, m_rx_tx_buffer_index);
          m_rx_tx_buffer_index = hw_agent.read_registers_TIM3(m_tx_buffer);
          send_buffer(m_tx_buffer, m_rx_tx_buffer_index);
          break;
          // case CMD_DBG
#endif
        default:
#ifdef DEBUG
          send_buffer("Invalid command\n");
#endif
          break;
      }  // switch (m_elap_cmd.type)
      m_comms_status = IDLE_TYPE;
      break;
      // case COMMAND_RECEIVED
  }
}

/**
 * @brief Report sampled data to PC
 * @param None
 * @returns None
 */
int Comms::report_sample_buffer() {
  m_elap_cmd.type = CMD_REPORT;
  m_elap_cmd.subtype = SUB_SAMPLED_DATA;
  m_elap_cmd.data.sampled_data_info.sampled = hw_agent.m_sample_buffer_count;
  m_elap_cmd.data.sampled_data_info.trigger = hw_agent.m_sample_buffer_trig_index;
  m_rx_tx_buffer_index = elap_cmd_to_packet(&m_elap_cmd, m_rx_tx_buffer, 0);
  send_buffer(m_rx_tx_buffer, m_rx_tx_buffer_index);
  send_buffer_circular(hw_agent.m_sample_buffer, hw_agent.m_sample_buffer_start, SAMPLE_BUFFER_SIZE,
                       hw_agent.m_sample_buffer_count);
  return 0;
}