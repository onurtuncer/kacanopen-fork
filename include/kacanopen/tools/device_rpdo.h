#pragma once
#include <iostream>
#include <vector>
#include "kacanopen/core/core.h"
#include "kacanopen/master/device.h"
#include "kacanopen/master/master.h"

enum RPDO_NO {
  RPDO_1,
  RPDO_2,
  RPDO_3,
  RPDO_4

};

void writeEntry(uint16_t index, std::vector<uint32_t> entries,
                 std::shared_ptr<kaco::Device> device);
void mapRPDOInDevice(RPDO_NO tpdo_no,
                        std::vector<uint32_t> entries_to_be_mapped,
                        uint8_t transmit_type,
                        std::shared_ptr<kaco::Device> device);
