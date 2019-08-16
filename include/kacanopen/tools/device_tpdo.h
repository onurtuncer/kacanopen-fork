#pragma once
#include <iostream>
#include <vector>
#include "kacanopen/core/core.h"
#include "kacanopen/master/device.h"
#include "kacanopen/master/master.h"

enum TPDO_NO {
  TPDO_1,
  TPDO_2,
  TPDO_3,
  TPDO_4

};

void writeEntry(uint16_t index, std::vector<uint32_t> entries,
                 std::shared_ptr<kaco::Device> device);
void mapTPDOinDevice(TPDO_NO tpdo_no,
                        std::vector<uint32_t> entries_to_be_mapped,
                        uint8_t transmit_type, uint16_t inhibit_time,
                        uint16_t event_timer,
                        std::shared_ptr<kaco::Device> device);

void mapTPDOinDevice(TPDO_NO tpdo_no,
                        std::vector<uint32_t> entries_to_be_mapped,
                        uint8_t transmit_type, uint16_t inhibit_time,
                        std::shared_ptr<kaco::Device> device);

void mapTPDOinDevice(TPDO_NO tpdo_no,
                        std::vector<uint32_t> entries_to_be_mapped,
                        uint8_t transmit_type,
                        std::shared_ptr<kaco::Device> device);
