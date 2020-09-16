#ifndef _ELA_H
#define _ELA_H

#include <stdint.h>

#include "ela_configuration.h"

typedef uint32_t arrat_index_t;

enum ela_status_t : uint8_t {
  STATUS_OK = 0x00U,
  STATUS_ERROR = 0x01U,
};

#endif