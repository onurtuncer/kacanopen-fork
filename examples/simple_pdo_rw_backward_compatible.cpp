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
#include <iostream>
#include <memory>
#include <vector>
#include "kacanopen/core/canopen_error.h"
#include "kacanopen/core/core.h"
#include "kacanopen/core/logger.h"
#include "kacanopen/master/device.h"
#include "kacanopen/master/master.h"
#include "kacanopen/tools/device_rpdo.h"
#include "kacanopen/tools/device_tpdo.h"
static volatile int keepRunning = 1;
static bool legacy_mode = false;
static int ch1_mode = 1;  // for velocity mode
static int ch2_mode = 1;  // for velocity mode
static bool initialized = false;
static uint16_t firmware_major_version = 2;
static uint16_t firmware_release_year = 2019;
static uint16_t firmware_release_month = 02;
static uint16_t firmware_relase_day = 01;
void intHandler(int dummy) {
  (void)dummy;
  keepRunning = 0;
}

std::string util_demangle(std::string to_demangle) {
  int status = 0;
  char* buff =
      __cxxabiv1::__cxa_demangle(to_demangle.c_str(), NULL, NULL, &status);
  std::string demangled = buff;
  std::free(buff);
  return demangled;
}
bool initializeDevice(std::shared_ptr<kaco::Device> device,
                      uint16_t heartbeat_interval, uint8_t node_id) {
  // set the our desired heartbeat_interval time
  device->set_entry(0x1017, 0x0, heartbeat_interval,
                    kaco::WriteAccessMethod::sdo);
  /// Master side RPDO mapping starts here. This must be in line with device
  /// side TPDOs.
  // Master side rpdo1 mapping
  if (legacy_mode) {
    device->add_receive_pdo_mapping(
        0x180 + node_id, 0x2103, static_cast<uint8_t>(0x01),
        0);  // 16bit unsigned entry for Channel 1 speed
    device->add_receive_pdo_mapping(
        0x180 + node_id, 0x2103, static_cast<uint8_t>(0x02),
        2);  // 16bit unsigned entry for Channel 2 speed
  } else {
    device->add_receive_pdo_mapping(
        0x180 + node_id, 0x2103, static_cast<uint8_t>(0x01),
        0);  // 32bit unsigned entry for Channel 1 speed
    device->add_receive_pdo_mapping(
        0x180 + node_id, 0x2103, static_cast<uint8_t>(0x02),
        4);  // 32bit unsigned entry for Channel 2 speed
  }
  // Master side rpdo2 mapping
  device->add_receive_pdo_mapping(0x280 + node_id, 0x2104,
                                  static_cast<uint8_t>(0x01),
                                  0);  // 32bit entry for Channel 1 position
  device->add_receive_pdo_mapping(0x280 + node_id, 0x2104,
                                  static_cast<uint8_t>(0x02),
                                  4);  // 32bit entry for Channel 2 position
  // Master side rpdo3 mapping
  device->add_receive_pdo_mapping(0x380 + node_id, 0x210D,
                                  static_cast<uint8_t>(0x01),
                                  0);  // 16bit entry for V internal
  device->add_receive_pdo_mapping(0x380 + node_id, 0x210D,
                                  static_cast<uint8_t>(0x02),
                                  2);  // 16bit entry for V BAT
  device->add_receive_pdo_mapping(0x380 + node_id, 0x210D,
                                  static_cast<uint8_t>(0x03),
                                  4);  // 16bit entry for V_5V
  device->add_receive_pdo_mapping(
      0x380 + node_id, 0x210F, static_cast<uint8_t>(0x02),
      6);  // 8bit entry for Channel 1 MOSFET Temperature
  device->add_receive_pdo_mapping(
      0x380 + node_id, 0x210F, static_cast<uint8_t>(0x03),
      7);  // 8bit entry for Channel 2 MOSFET Temperature
  // Master side rpdo4 mapping
  device->add_receive_pdo_mapping(0x480 + node_id, 0x2100,
                                  static_cast<uint8_t>(0x01),
                                  0);  // 16bit entry for Channel 1 motor Amps
  device->add_receive_pdo_mapping(0x480 + node_id, 0x2100,
                                  static_cast<uint8_t>(0x02),
                                  2);  // 16bit entry for Channel 2 motor Amps
  device->add_receive_pdo_mapping(0x480 + node_id, 0x2102,
                                  static_cast<uint8_t>(0x01),
                                  4);  // 16bit entry for Channel 1 motor Power
  device->add_receive_pdo_mapping(0x480 + node_id, 0x2102,
                                  static_cast<uint8_t>(0x02),
                                  6);  // 16bit entry for Channel 2 motor Power

  /// Master side RPDO mapping ends here

  /// Device side TPDO mapping starts here. This must be in line with the
  /// master side RPDOs.
  // Device side tpdo1 mapping entries and mapping
  if (legacy_mode) {
    const std::vector<uint32_t> tpdo1_entries_to_be_mapped{0x21030110,
                                                           0x21030210};
    map_tpdo_in_device(TPDO_1, tpdo1_entries_to_be_mapped, 255, 100, 250,
                       device);
  } else {
    const std::vector<uint32_t> tpdo1_entries_to_be_mapped{0x21030120,
                                                           0x21030220};
    map_tpdo_in_device(TPDO_1, tpdo1_entries_to_be_mapped, 255, 100, 250,
                       device);
  }

  // Device side tpdo2 mapping entries and mapping
  const std::vector<uint32_t> tpdo2_entries_to_be_mapped{
      0x21040120, 0x21040220,
  };
  map_tpdo_in_device(TPDO_2, tpdo2_entries_to_be_mapped, 255, 100, 250, device);

  // Device side tpdo3 mapping entries and mapping
  const std::vector<uint32_t> tpdo3_entries_to_be_mapped{
      0x210D0110, 0x210D0210, 0x210D0310, 0x210F0208, 0x210F0308};
  const std::vector<uint32_t> tpdo3_test_entries{0x210D0110, 0x210D0210,
                                                 0x210D0110, 0x210D0210};
  map_tpdo_in_device(TPDO_3, tpdo3_entries_to_be_mapped, 255, 100, 250, device);

  // Device side tpdo4 mapping entries and mapping
  const std::vector<uint32_t> tpdo4_entries_to_be_mapped{
      0x21000110, 0x21000210, 0x21020110, 0x21020210};
  map_tpdo_in_device(TPDO_4, tpdo4_entries_to_be_mapped, 255, 100, 250, device);
  /// Device side TPDO mapping ends here.

  /// Master side TPDO mapping starts here.This must be in line with device
  /// side RPDOs.
  if (1 == ch1_mode /*Close Loop Speed Mode*/) {
    // Mater side Periodic Tranmit pdo1 value initialization
    device->set_entry(0x2000, static_cast<uint8_t>(0x01), 0x0,
                      kaco::WriteAccessMethod::sdo);
    // Master side Periodic Tranmit pdo1 mapping
    device->add_transmit_pdo_mapping(0x200 + static_cast<uint16_t>(node_id),
                                     {{0x2000, static_cast<uint8_t>(0x01), 0}},
                                     kaco::TransmissionType::PERIODIC,
                                     std::chrono::milliseconds(50));
  } else if (3 == ch1_mode /*Close Loop Position Count Mode*/) {
    // Mater side Periodic Tranmit pdo1 value initialization
    device->set_entry(0x2001, static_cast<uint8_t>(0x01), 0x0,
                      kaco::WriteAccessMethod::sdo);
    // Master side Periodic Tranmit pdo1 mapping
    device->add_transmit_pdo_mapping(0x200 + static_cast<uint16_t>(node_id),
                                     {{0x2001, static_cast<uint8_t>(0x01), 0}},
                                     kaco::TransmissionType::PERIODIC,
                                     std::chrono::milliseconds(50));
  }
  if (1 == ch2_mode /*Close Loop Speed Mode*/) {
    // Mater side Periodic Tranmit pdo1 value initialization
    device->set_entry(0x2000, static_cast<uint8_t>(0x02), 0x0,
                      kaco::WriteAccessMethod::sdo);
    // Master side Periodic Tranmit pdo1 mapping
    device->add_transmit_pdo_mapping(0x300 + static_cast<uint16_t>(node_id),
                                     {{0x2000, static_cast<uint8_t>(0x02), 0}},
                                     kaco::TransmissionType::PERIODIC,
                                     std::chrono::milliseconds(50));
  } else if (3 == ch2_mode /*Close Loop Position Count Mode*/) {
    // Mater side Periodic Tranmit pdo1 value initialization
    device->set_entry(0x2001, static_cast<uint8_t>(0x02), 0x0,
                      kaco::WriteAccessMethod::sdo);
    // Master side Periodic Tranmit pdo1 mapping
    device->add_transmit_pdo_mapping(0x300 + static_cast<uint16_t>(node_id),
                                     {{0x2001, static_cast<uint8_t>(0x02), 0}},
                                     kaco::TransmissionType::PERIODIC,
                                     std::chrono::milliseconds(50));
  }
  /// Master side TPDO mapping ends here

  if (1 == ch1_mode /*Close Loop Speed Mode*/) {
    // Device side rpdo1 mapping entries and mapping
    const std::vector<uint32_t> rpdo1_entries_to_be_mapped{0x20000120};
    map_rpdo_in_device(RPDO_1, rpdo1_entries_to_be_mapped, 255, device);
  } else if (3 == ch1_mode /*Close Loop Position Count Mode*/) {
    // Device side rpdo1 mapping entries and mapping
    const std::vector<uint32_t> rpdo1_entries_to_be_mapped{0x20010120};
    map_rpdo_in_device(RPDO_1, rpdo1_entries_to_be_mapped, 255, device);
  }
  if (1 == ch2_mode /*Close Loop Speed Mode*/) {
    // Device side rpdo2 mapping entries and mapping
    const std::vector<uint32_t> rpdo2_entries_to_be_mapped{0x20000220};
    map_rpdo_in_device(RPDO_2, rpdo2_entries_to_be_mapped, 255, device);
  } else if (3 == ch2_mode /*Close Loop Position Count Mode*/) {
    // Device side rpdo2 mapping entries and mapping
    const std::vector<uint32_t> rpdo2_entries_to_be_mapped{0x20010220};
    map_rpdo_in_device(RPDO_2, rpdo2_entries_to_be_mapped, 255, device);
  }

  initialized = true;
  return initialized;
}

int main() {
  // Signal handleing
  signal(SIGINT, intHandler);

  // ----------- //
  // Preferences //
  // ----------- //

  // A Roboteq motor driver with firmware version v2.0beta07032018 was used to
  // test this program.//

  // The node ID of the slave we want to communicate with.
  const uint8_t node_id = 4;

  // Set the name of your CAN bus. "slcan0" is a common bus name
  // for the first SocketCAN device on a Linux system.
  const std::string busname = "slcan0";

  // Set the baudrate of your CAN bus. Most drivers support the values
  // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
  const std::string baudrate = "500K";

  // Set the heartbeat interval for slave device. Most drivers support the
  // values can be "125", "250", "500" and "1000" millisecond.
  const uint16_t heartbeat_interval = 250;

  // Set the heartbeat time out, after which the system should detect slave
  // disconnection; values can be "250", "500", "1000" and "2000" millisecond.
  // Temporary disabled the timeout parameters; A gloabl 2 second time is now
  // used in device_alive and device_dead callback
  // const uint16_t heartbeat_timeout = heartbeat_interval * 3;

  // -------------- //
  // Initialization //
  // -------------- //

  // Create core.
  kaco::Core core;
  volatile bool found_node = false;
  volatile bool device_connected = false;
  std::mutex device_mutex;

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

  std::cout << "Registering a callback which is called when a device is "
               "detected via NMT..."
            << std::endl;
  // make sure that the node is reset and goes back to NMT preoperational
  // core.nmt.send_nmt_message(node_id, kaco::NMT::Command::reset_node);
  // std::this_thread::sleep_for(std::chrono::milliseconds(300));
  core.nmt.register_device_alive_callback([&](const uint8_t new_node_id) {
    // Check if this is the node we are looking for.
    if (new_node_id == node_id) {
      // lock
      if (!found_node) {
        found_node = true;
        // Lock device mutex
        std::lock_guard<std::mutex> lock(device_mutex);
        try {
          device.reset(new kaco::Device(core, node_id));
          core.nmt.send_nmt_message(node_id,
                                    kaco::NMT::Command::enter_preoperational);
          // Load eds file. The eds file must be in the same folder in which the
          // binary is being executed.
          // std::string path = ros::package::getPath("kacanopen");
          if (!legacy_mode) {
            boost::filesystem::path full_path =
              /*  path + */
                "/resources/eds_library/Roboteq/"
                "roboteq_motor_controllers_v2.0-30-01-"
                "2019.eds";
            device->load_dictionary_from_eds(full_path.string());
          } else {
            std::cout << "Legacy Mode is enabled" << std::endl;
            boost::filesystem::path full_path =
               /* path + */
                "/resources/eds_library/Roboteq/"
                "roboteq_motor_controllers_v80beta.eds";
            device->load_dictionary_from_eds(full_path.string());
          }

          uint16_t version = device->get_entry(
              0x2137, static_cast<uint8_t>(0x01), kaco::ReadAccessMethod::sdo);
          uint16_t month = device->get_entry(0x2137, static_cast<uint8_t>(0x02),
                                             kaco::ReadAccessMethod::sdo);
          uint16_t day = device->get_entry(0x2137, static_cast<uint8_t>(0x03),
                                           kaco::ReadAccessMethod::sdo);
          uint16_t year = device->get_entry(0x2137, static_cast<uint8_t>(0x04),
                                            kaco::ReadAccessMethod::sdo);
          uint16_t major = version / 10;
          if ((year == 2018) && (month == 07) && (day == 03) && (major == 2))
            legacy_mode = true;
          if ((firmware_major_version != major) ||
              (firmware_relase_day != day) ||
              (firmware_release_month != month)) {
            std::cout << "Firmware version mismatch" << std::endl;
            exit(1);
          }
          if (initializeDevice(device, heartbeat_interval, node_id)) {
            device->start();
            device_connected = true;
          }
        } catch (...) {
          std::cout
              << "Exception occured while registering device alive callback! "
                 "Type: "
              << util_demangle(
                     __cxxabiv1::__cxa_current_exception_type()->name())
              << std::endl;
          found_node = false;
          device_connected = false;
        }
      }
    }
  });
  core.nmt.register_device_dead_callback([&](const uint8_t new_node_id) {
    if ((device_connected && found_node) && (new_node_id == node_id)) {
      // Lock device mutex
      std::lock_guard<std::mutex> lock(device_mutex);
      // Check if our node is disconnected.
      found_node = false;
      device_connected = false;
      device.reset();
      std::cout << "Device with Node ID=0x" << std::hex << node_id
                << " is disconnected...." << std::endl;
    }
  });

  int channel1_speed_ref = 0;
  int channel2_speed_ref = 0;
  int max_rpm = 300;
  bool max = false;
  while (keepRunning) {
    if (device_connected) {
      // Lock device mutex
      std::lock_guard<std::mutex> lock(device_mutex);
      try {
        // Prepare the commands; master side tpdo1 and tpdo2
        if (max_rpm > channel1_speed_ref && max == false) {
          // channel1_speed_ref++;
          channel1_speed_ref = channel1_speed_ref + 20;
          if (max_rpm == channel1_speed_ref) {
            max = true;
          }
        }
        if (-max_rpm < channel1_speed_ref && max == true) {
          // channel1_speed_ref--;
          channel1_speed_ref = channel1_speed_ref - 20;
          if (-max_rpm == channel1_speed_ref) {
            max = false;
          }
        }
        channel2_speed_ref = channel1_speed_ref;
        device->set_entry(0x2000, static_cast<uint8_t>(0x01),
                          static_cast<int>(channel1_speed_ref),
                          kaco::WriteAccessMethod::pdo);
        std::cout << "Channel 1 speed command = " << std::dec
                  << channel1_speed_ref << std::endl;
        int32_t ch1_speed_feedback =
            device->get_entry(0x2103, static_cast<uint8_t>(0x01),
                              kaco::ReadAccessMethod::pdo_request_and_wait);
        std::cout << "Channel 1 speed feedback = " << std::dec
                  << (ch1_speed_feedback) << std::endl;
//        device->set_entry(0x2000, static_cast<uint8_t>(0x02),
//                          static_cast<int>(channel2_speed_ref),
//                          kaco::WriteAccessMethod::pdo);
        std::cout << "Channel 2 speed command = " << std::dec
                  << channel1_speed_ref << std::endl;
        int32_t ch2_speed_feedback =
            device->get_entry(0x2103, static_cast<uint8_t>(0x02),
                              kaco::ReadAccessMethod::pdo_request_and_wait);
        std::cout << "Channel 2 speed feedback = " << std::dec
                  << ch2_speed_feedback << std::endl;
        uint16_t v_int =
            device->get_entry(0x210D, static_cast<uint8_t>(0x01),
                              kaco::ReadAccessMethod::pdo_request_and_wait);
        std::cout << "Internal Voltage = " << std::dec
                  << static_cast<float>(static_cast<float>(v_int) /
                                        static_cast<float>(10))
                  << "V" << std::endl;
        uint16_t v_bat =
            device->get_entry(0x210D, static_cast<uint8_t>(0x02),
                              kaco::ReadAccessMethod::pdo_request_and_wait);
        std::cout << "Battery Voltage = " << std::dec
                  << static_cast<float>(static_cast<float>(v_bat) /
                                        static_cast<float>(10))
                  << "V" << std::endl;
        uint16_t v_5vout =
            device->get_entry(0x210D, static_cast<uint8_t>(0x03),
                              kaco::ReadAccessMethod::pdo_request_and_wait);
        std::cout << "Internal 5V supply = " << std::dec
                  << static_cast<float>(static_cast<float>(v_5vout) /
                                        static_cast<float>(1000))
                  << "V" << std::endl;
        uint16_t digout = device->get_entry(0x2113, static_cast<uint8_t>(0x00),
                                            kaco::ReadAccessMethod::sdo);
        std::cout << "Status of Digital Outs = " << std::hex << digout << ""
                  << std::endl;
      } catch (...) {
        std::cout << "Exception in main!" << std::endl;
      }
    }

    // Sleep
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "Finished." << std::endl;
  return EXIT_SUCCESS;
}
