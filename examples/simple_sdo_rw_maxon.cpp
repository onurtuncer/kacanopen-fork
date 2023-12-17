/*
 * Copyright (c) 2018-2019, Musarraf Hossain
 * All rights reserved.
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
#include <iostream>
#include <memory>
#include <vector>
#include "kacanopen/core/core.h"
#include "kacanopen/core/logger.h"
#include "kacanopen/master/device.h"
#include "kacanopen/master/master.h"
#include "kacanopen/tools/parse_sdo.h"

static volatile int keepRunning = 1;

void intHandler(int dummy) {
  (void)dummy;
  keepRunning = 0;
}

int main() {
  // Signal handleing
  signal(SIGINT, intHandler);

  // ----------- //
  // Preferences //
  // ----------- //

  // A Roboteq motor driver was used to test this program //
  // The firmware version of roboteq driver: v2.0beta07032018//
  // The node ID of the slave we want to communicate with.
  const uint8_t node_id = 01;

  // Set the name of your CAN bus. "slcan0" is a common bus name
  // for the first SocketCAN device on a Linux system.
  const std::string busname = "slcan0";

  // Set the baudrate of your CAN bus. Most drivers support the values
  // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
  const std::string baudrate = "500K";

  const std::vector<uint8_t> ch1_speed{
      0x22, 0xF0, 0xFF, 0xFF};  // speed reference to roboteq : +3276 .This is
                                // the lowest valid reference.
  // const std::vector<uint8_t> ch1_speed {0x34,0xF3,0xFF,0xFF}; //speed
  // reference to roboteq :-3276 .This is the lowest valid reference.
  const std::vector<uint8_t> ch2_speed{
      0xCC, 0x0C, 0x00, 0x00};  // speed reference to roboteq : +3276 .This is
                                // the lowest valid reference.
  // const std::vector<uint8_t> ch2_speed {0x34,0xF3,0xFF,0xFF}; //speed
  // reference to roboteq :-3276 .This is the lowest valid reference.

  // Alternative: CiA-402 (motor) control word has two bytes. Command: shutdown
  // (little-endian!)
  // const std::vector<uint8_t> data { 0x06, 0x00 };

  // -------------- //
  // Initialization //
  // -------------- //
  // Create core.
  kaco::Core core;
  std::cout << "Starting Core (connect to the driver and start the receiver "
               "thread)..."
            << std::endl;
  if (!core.start(busname, baudrate)) {
    std::cout << "Starting core failed." << std::endl;
    return EXIT_FAILURE;
  }
  // Create device pointer
  std::shared_ptr<kaco::Device> device;
  // This will be set to true by the callback below.
  bool found_node = false;

  std::cout << "Registering a callback which is called when a device is "
               "detected via NMT..."
            << std::endl;
  core.nmt.register_device_alive_callback([&](
      const uint8_t new_node_id) mutable {
    // Check if this is the node we are looking for.
    if (new_node_id == node_id) {
      if (!found_node) {
        found_node = true;
        device.reset(new kaco::Device(core, node_id));
        // Load eds file. The eds file must be in the same folder in which the
        // binary is being executed.
        // std::string path = ros::package::getPath("kacanopen");
        boost::filesystem::path full_path =
          /*  path +  */ "/resources/eds_library/MaxonMotor/maxon_motor_EPOS4.eds";
        device->load_dictionary_from_eds(full_path.string());
        device->start();
        // device->load_dictionary_from_library();
        std::vector<uint8_t> read_device_type =
            core.sdo.upload(node_id, 0x1000, 0x0);
        std::vector<uint8_t> read_device_name =
            core.sdo.upload(node_id, 0x1008, 0x0);
        std::vector<uint8_t> read_hardware_version =
            core.sdo.upload(node_id, 0x1009, 0x0);
        std::vector<uint8_t> read_firmware_version =
            core.sdo.upload(node_id, 0x100A, 0x0);
        uint16_t profile = parse_sdo_read_as_int(read_device_type);
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "*******************************************************"
                  << std::endl;
        std::cout << "*******************************************************"
                  << std::endl;
        std::cout << "* Device Profile number found CiA-" << std::dec << profile
                  << "" << std::endl;
        std::cout << "* Device Name found as '";
        std::string device_name(
            reinterpret_cast<char const*>(read_device_name.data()),
            read_device_name.size());
        std::cout << device_name << "'" << std::endl;
        std::cout << "* Hardware version=";
        std::string hardware_version(
            reinterpret_cast<char const*>(read_hardware_version.data()),
            read_hardware_version.size());
        std::cout << hardware_version << "" << std::endl;
        std::cout << "* Firmware version=";
        std::string firmware_version(
            reinterpret_cast<char const*>(read_firmware_version.data()),
            read_firmware_version.size());
        std::cout << hardware_version << "" << std::endl;
        std::cout << "*******************************************************"
                  << std::endl;
        std::cout << "*******************************************************"
                  << std::endl;
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;

        std::cout << "Dumping Manufacturer device name " << std::endl;
        DUMP(device->get_entry(
            "Manufacturer device name"));  // Manufacturer device name
        std::cout << "Dumping Manufacturer software version " << std::endl;
        DUMP(device->get_entry("Manufacturer software version"));
        std::cout << "read_state_16_input_lines/number_of_elements.. "
                  << std::endl;
        DUMP(device->get_entry("read_state_16_input_lines/number_of_elements"));
        std::cout << "Dumping the available object dictionary in the device->.."
                  << std::endl;
      }
    }
  });

  std::cout << "Giving the devices one second time to respond..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::cout << "Printing Device Object Dictionary" << std::endl;
  device->print_dictionary();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  // ------------ //
  // Device usage //
  // ------------ //
  const std::vector<uint8_t> control_word_value1 = {0x06, 0x00};
  const std::vector<uint8_t> control_word_value2 = {0x0F, 0x00};
  const uint16_t index_control_word = 0x6040;
  const uint16_t subindex_control_word = 0x00;
  core.sdo.download(node_id, index_control_word, subindex_control_word,
                    control_word_value1.size(), control_word_value1);
  while (keepRunning) {
    if (found_node) {
      std::cout << "Reading the sdo at " << std::hex << 0x6064 << "...."
                << std::endl;
      std::vector<uint8_t> ch1_speed_feedback =
          core.sdo.upload(node_id, 0x6064, 0x0);
      std::vector<uint8_t> status_word;
      int readout1 = parse_sdo_read_as_int(ch1_speed_feedback);
      std::cout << "0x6064, 0x00=" << std::dec << readout1 << std::endl;
      std::cout << "setting target velocity=" << 300 << std::endl;
      const uint16_t index_target_velocity = 0x60FF;
      const uint16_t subindex_target_velocity = 0x00;
      const std::vector<uint8_t> target_velocity = {
          0x10, 0x10, 0x00, 0x00};  //{0x10, 0x10, 0x00, 0x00};

      status_word = core.sdo.upload(node_id, 0x6041, 0x0);
      int status_decoded = parse_sdo_read_as_int(status_word);
      std::cout << "status word value=" << std::dec << status_decoded << "...."
                << std::endl;
      core.sdo.upload(node_id, 0x6041, 0x0);
      core.sdo.download(node_id, index_control_word, subindex_control_word,
                        control_word_value2.size(), control_word_value2);
      status_word = core.sdo.upload(node_id, 0x6041, 0x0);
      status_decoded = parse_sdo_read_as_int(status_word);
      std::cout << "status word value=" << std::dec << status_decoded << "...."
                << std::endl;

      core.sdo.download(node_id, index_target_velocity,
                        subindex_target_velocity, target_velocity.size(),
                        target_velocity);

      std::cout << "Enable control word " << std::hex << 0x6040 << "...."
                << std::endl;
      //      core.sdo.download(node_id, index_control_word,
      //      subindex_control_word,
      //                        control_word_value2.size(),
      //                        control_word_value2);
      uint16_t control_word_value = {0x000F};
      device->set_entry(0x6040, 0x00, control_word_value,
                        kaco::WriteAccessMethod::sdo);
    }
  }
  std::cout << "Finished." << std::endl;
  return EXIT_SUCCESS;
}
