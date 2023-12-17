/*
 * Copyright (c) 2018-2019, Musarraf Hossain
 * All rights reservd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// #include <ros/package.h>
#include <signal.h>
#include <boost/filesystem.hpp>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>
#include "kacanopen/core/canopen_error.h"
#include "kacanopen/core/core.h"
#include "kacanopen/core/logger.h"
#include "kacanopen/master/device.h"
#include "kacanopen/tools/device_rpdo.h"
#include "kacanopen/tools/device_tpdo.h"

static volatile int keepRunning = 1;

void intHandler(int dummy) {
  (void)dummy;
  keepRunning = 0;
}

bool printDeviceInfo(std::shared_ptr<kaco::Device> device_) {
  try {
    auto device_type =
        device_->get_entry(0x1000, 0x0, kaco::ReadAccessMethod::sdo);
    auto device_name =
        device_->get_entry(0x1008, 0x0, kaco::ReadAccessMethod::sdo);
    auto hardware_version =
        device_->get_entry(0x1009, 0x0, kaco::ReadAccessMethod::sdo);
    auto firmware_version =
        device_->get_entry(0x100A, 0x0, kaco::ReadAccessMethod::sdo);
    std::cout << "" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "* Device Name found as '" << device_name << "'" << std::endl;
    std::cout << "* Device Type found as CiA-" << device_type << "'"
              << std::endl;
    std::cout << "* Hardware version=" << hardware_version << std::endl;
    std::cout << "* Firmware version=" << firmware_version << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "" << std::endl;
    std::cout << "" << std::endl;
    return true;
  } catch (...) {
    std::cout << "Exception occured while acquiring device information"
              << std::endl;
    return false;
  }
}

void initializeDevice(std::shared_ptr<kaco::Device> device,
                      uint16_t heartbeat_interval, uint8_t node_id) {
  // Load eds file. The eds file must be in the same folder in which the
  // binary is being executed.
  // std::string path = ros::package::getPath("kacanopen");
  boost::filesystem::path full_path =
      /*path + */ "resources/eds_library/roboteq_motor_controllers_v80beta.eds";
  device->load_dictionary_from_eds(full_path.string());
  // set the our desired heartbeat_interval time
  device->set_entry(0x1017, 0x0, heartbeat_interval,
                    kaco::WriteAccessMethod::sdo);
  // This is optional verbosity
  device->print_dictionary();

  // Master side rpdo1 mapping
  device->add_receive_pdo_mapping(0x180 + node_id, "qry_abspeed/channel_1", 0);
  device->add_receive_pdo_mapping(0x180 + node_id, "qry_abspeed/channel_2", 2);
  device->add_receive_pdo_mapping(0x180 + node_id, "qry_batamps/channel_1", 4);
  device->add_receive_pdo_mapping(0x180 + node_id, "qry_batamps/channel_2", 6);
  // Master side rpdo2 mapping
  device->add_receive_pdo_mapping(0x280 + node_id, "qry_volts/v_int", 0);
  device->add_receive_pdo_mapping(0x280 + node_id, "qry_volts/v_bat", 2);
  device->add_receive_pdo_mapping(0x280 + node_id, "qry_volts/v_5vout", 4);
  device->add_receive_pdo_mapping(0x280 + node_id, "qry_digout", 6);

  // Mater side Periodic Tranmit pdo1 value initialization
  device->set_entry("cmd_cango/cmd_cango_1", 0x0, kaco::WriteAccessMethod::pdo);
  // Mater side Periodic Tranmit pdo2 value initialization
  device->set_entry("cmd_cango/cmd_cango_2", static_cast<int>(0x0),
                    kaco::WriteAccessMethod::pdo);

  // Master side tpdo1 mapping
  device->add_transmit_pdo_mapping(
      0x200 + node_id, {{"cmd_cango/cmd_cango_1", 0}},
      kaco::TransmissionType::ON_CHANGE, std::chrono::milliseconds(250));

  // Master side tpdo2 mapping
  device->add_transmit_pdo_mapping(
      0x300 + node_id, {{"cmd_cango/cmd_cango_2", 0}},
      kaco::TransmissionType::ON_CHANGE, std::chrono::milliseconds(250));

  // Device side tpdo1 mapping entries and mapping
  const std::vector<uint32_t> tpdo1_entries_to_be_mapped{
      0x21030110, 0x21030210, 0x210C0110, 0x210C0210};
  map_tpdo_in_device(TPDO_1, tpdo1_entries_to_be_mapped, 255, 100, 250, device);

  // Device side tpdo2 mapping entries and mapping
  const std::vector<uint32_t> tpdo2_entries_to_be_mapped{
      0x210D0110, 0x210D0210, 0x210D0310,
      0x21130010};  // {0x210D0110, 0x210D0210, 0x210D0310, 0x21030110}
  map_tpdo_in_device(TPDO_2, tpdo2_entries_to_be_mapped, 255, 100, 250, device);
  // Device side rpdo1 mapping entries and mapping
  const std::vector<uint32_t> rpdo1_entries_to_be_mapped{0x20000120};
  map_rpdo_in_device(RPDO_1, rpdo1_entries_to_be_mapped, 255, device);

  // Device side rpdo2 mapping entries and mapping
  const std::vector<uint32_t> rpdo2_entries_to_be_mapped{0x20000220};
  map_rpdo_in_device(RPDO_2, rpdo2_entries_to_be_mapped, 255, device);
}

int main() {
  // Signal handleing
  signal(SIGINT, intHandler);

  // ----------- //
  // Preferences //
  // ----------- //

  // A Roboteq motor driver was used to test this program.//

  // This example allows to control multiple similar CANOpen slave devices. All
  // the devices must of same type.

  // Set the name of your CAN bus. "slcan0" is a common bus name
  // for the first SocketCAN device on a Linux system.
  const std::string busname = "can0";

  // Set the baudrate of your CAN bus. Most drivers support the values
  // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
  const std::string baudrate = "500K";

  // Set the heartbeat interval for slave device. Most drivers support the
  // values can be "125", "250", "500" and "1000" millisecond.
  const uint16_t heartbeat_interval = 250;

  // Define the node_id list for the slaves we want to communicate with.
  uint8_t node_id[] = {5, 4, 3, 2, 7};

  // Set the heartbeat time out, after which the system should detect slave
  // disconnection; values can be "250", "500", "1000" and "2000" millisecond.
  // Temporary disabled the timeout parameters; A gloabl 2 second time is now
  // used in device_alive and device_dead callback
  // const uint16_t heartbeat_timeout = heartbeat_interval * 3;

  // -------------- //
  // Initialization //
  // -------------- //

  // Create core.
  // kaco::Core core;

  // kaco::Master master;
  kaco::Core core;
  std::mutex device_mutex;

  std::cout << "Starting master (connect to the driver and start the receiver "
               "thread)..."
            << std::endl;
  if (!core.start(busname, baudrate)) {
    std::cout << "Starting master failed." << std::endl;
    return EXIT_FAILURE;
  }
  std::shared_ptr<kaco::Device> device[sizeof(node_id)];
  std::unordered_map<uint8_t, std::shared_ptr<kaco::Device>> DeviceMap;

  std::cout << "Registering a callback which is called when a device is "
               "detected via NMT..."
            << std::endl;

  core.nmt.register_device_alive_callback([&](const uint8_t new_node_id) {
    // Check if this is the node we are looking for.
    if (std::any_of(std::begin(node_id), std::end(node_id),
                    [=](uint8_t n) { return n == new_node_id; })) {
      std::lock_guard<std::mutex> lock(device_mutex);
      // Check whether we already have this node in the Device map
      if (DeviceMap.find(new_node_id) == DeviceMap.end()) {
        // finding the node_id index
        auto index = std::distance(
            std::begin(node_id),
            std::find(std::begin(node_id), std::end(node_id), new_node_id));
        // reseting the device pointer with new_node_id
        device[index].reset(new kaco::Device(core, new_node_id));
        // Adding the device pointer in Device map
        DeviceMap.insert({new_node_id, device[index]});
        // Initialize the new node
        initializeDevice(device[index], heartbeat_interval, new_node_id);
      }
    }
  });

  core.nmt.register_device_dead_callback([&](const uint8_t current_node_id) {
    // Check whether we have this current_node_id in the Device map
    if (DeviceMap.find(current_node_id) != DeviceMap.end()) {
      std::lock_guard<std::mutex> lock(device_mutex);
      // Remove the device map entry for this current_node_id
      DeviceMap.erase(DeviceMap.find(current_node_id));
    }
  });

  int channel1_speed_ref = 0;
  int channel2_speed_ref = 0;
  bool max = false;

  while (keepRunning) {
    for (uint8_t i = 0; i < sizeof(node_id); i++) {
      if (DeviceMap.find(node_id[i]) != DeviceMap.end()) {
        // Lock device mutex
        std::lock_guard<std::mutex> lock(device_mutex);
        std::cout << "Now controlling device with node id= " << std::hex
                  << node_id[i] << std::endl;
        try {
          // Early exception handling
          //      try {
          //        //        DUMP_HEX(
          //        //            device->get_entry("qry_abspeed/channel_1",
          //        // kaco::ReadAccessMethod::pdo_request_and_wait));
          //        //        DUMP_HEX(
          //        //            device->get_entry("qry_abspeed/channel_2",
          //        // kaco::ReadAccessMethod::pdo_request_and_wait));
          //        //        DUMP_HEX(
          //        //            device->get_entry("qry_batamps/channel_1",
          //        // kaco::ReadAccessMethod::pdo_request_and_wait));
          //        //        DUMP_HEX(
          //        //            device->get_entry("qry_batamps/channel_2",
          //        // kaco::ReadAccessMethod::pdo_request_and_wait));
          //      } catch (kaco::canopen_error exception) {
          //        // No specific action is decided
          //      }

          // Prepare the commands; master side tpdo1 and tpdo2
          if (800 > channel1_speed_ref && max == false) {
            // channel1_speed_ref++;
            channel1_speed_ref = channel1_speed_ref + 10;
            if (800 == channel1_speed_ref) {
              max = true;
            }
          }
          if (-800 < channel1_speed_ref && max == true) {
            // channel1_speed_ref--;
            channel1_speed_ref = channel1_speed_ref - 10;
            if (-800 == channel1_speed_ref) {
              max = false;
            }
          }
          channel2_speed_ref = channel1_speed_ref;
          DeviceMap.at(node_id[i])
              ->set_entry("cmd_cango/cmd_cango_1",
                          static_cast<int>(channel1_speed_ref),
                          kaco::WriteAccessMethod::pdo);
          std::cout << "Channel 1 speed command = " << std::dec
                    << channel1_speed_ref << std::endl;
          int16_t ch1_speed_feedback =
              DeviceMap.at(node_id[i])
                  ->get_entry("qry_abspeed/channel_1",
                              kaco::ReadAccessMethod::pdo_request_and_wait);
          std::cout << "Channel 1 speed feedback = " << std::dec
                    << (ch1_speed_feedback) << std::endl;
          DeviceMap.at(node_id[i])
              ->set_entry("cmd_cango/cmd_cango_2",
                          static_cast<int>(channel2_speed_ref),
                          kaco::WriteAccessMethod::pdo);
          std::cout << "Channel 2 speed command = " << std::dec
                    << channel1_speed_ref << std::endl;
          int16_t ch2_speed_feedback =
              DeviceMap.at(node_id[i])
                  ->get_entry("qry_abspeed/channel_2",
                              kaco::ReadAccessMethod::pdo_request_and_wait);
          std::cout << "Channel 2 speed feedback = " << std::dec
                    << ch2_speed_feedback << std::endl;
          uint16_t v_int =
              DeviceMap.at(node_id[i])
                  ->get_entry("qry_volts/v_int",
                              kaco::ReadAccessMethod::pdo_request_and_wait);
          std::cout << "Internal Voltage = " << std::dec
                    << static_cast<float>(static_cast<float>(v_int) /
                                          static_cast<float>(10))
                    << "V" << std::endl;
          uint16_t v_bat =
              DeviceMap.at(node_id[i])
                  ->get_entry("qry_volts/v_bat",
                              kaco::ReadAccessMethod::pdo_request_and_wait);
          std::cout << "Battery Voltage = " << std::dec
                    << static_cast<float>(static_cast<float>(v_bat) /
                                          static_cast<float>(10))
                    << "V" << std::endl;
          uint16_t v_5vout =
              DeviceMap.at(node_id[i])
                  ->get_entry("qry_volts/v_5vout",
                              kaco::ReadAccessMethod::pdo_request_and_wait);
          std::cout << "Internal 5V supply = " << std::dec
                    << static_cast<float>(static_cast<float>(v_5vout) /
                                          static_cast<float>(1000))
                    << "V" << std::endl;
          uint16_t digout =
              DeviceMap.at(node_id[i])
                  ->get_entry("qry_digout",
                              kaco::ReadAccessMethod::pdo_request_and_wait);
          std::cout << "Status of Digital Outs = " << std::hex << digout << ""
                    << std::endl;
        } catch (...) {
          std::cout << "Exception in main!" << std::endl;
        }
      }
    }

    for (uint8_t i = 0; i < sizeof(node_id); i++) {
      auto temp = DeviceMap.find(node_id[i]);
      if (temp != DeviceMap.end()) {
        std::cout << "Node id=0x" << node_id[i] << " is connected" << std::endl;
      } else {
        std::cout << "Node id=0x" << node_id[i] << " is disconnected"
                  << std::endl;
      }
    }
    // Sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "Finished." << std::endl;
  return EXIT_SUCCESS;
}
