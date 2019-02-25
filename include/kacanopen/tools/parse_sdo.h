#pragma once
#include <boost/variant.hpp>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

enum SDO_PARSE_TYPE {
  unsigned_int32 = 1,
  signed_int32 = 2,
  unsigned_int16 = 3,
  signed_int16 = 4,
  unsigned_int8 = 5,
  signed_int8 = 6,
  string = 7
};

int32_t parse_sdo_read_as_int(std::vector<uint8_t> sdo_to_read);

boost::variant<int32_t, uint32_t, uint16_t, int16_t, uint8_t, int8_t,
               std::string>
parse_sdo(std::vector<uint8_t> sdo_to_read, SDO_PARSE_TYPE parse_type);
