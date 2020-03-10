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

#include <ros/package.h>
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
#include "kacanopen/tools/parse_sdo.h"
static volatile int keepRunning = 1;

void intHandler(int dummy) {
  (void)dummy;
  keepRunning = 0;
}

bool printDeviceInfo(std::shared_ptr<kaco::Device> device, kaco::Core &core,
                     uint8_t node_id) {
  try {
    std::vector<uint8_t> device_type_vector =
        core.sdo.upload(node_id, 0x1000, 0x0);
    uint32_t device_type = boost::get<uint32_t>(
        parseSDO(device_type_vector, SDO_PARSE_TYPE::UNSIGNED_INT32));
    std::vector<uint8_t> name_vector = core.sdo.upload(node_id, 0x1008, 0x00);
    std::string device_name(name_vector.begin(), name_vector.end());
    std::vector<uint8_t> hw_ver_vector = core.sdo.upload(node_id, 0x1009, 0x00);
    std::string hardware_version(hw_ver_vector.begin(), hw_ver_vector.end());
    std::vector<uint8_t> sw_ver_vector = core.sdo.upload(node_id, 0x100A, 00);
    std::string software_version(sw_ver_vector.begin(), sw_ver_vector.end());
    std::cout << "" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "* Device Name found as '" << device_name << "'" << std::endl;
    std::cout << "* Device Type found as CiA-" << device_type << "'"
              << std::endl;
    std::cout << "* Manufacturer Hardware version = " << hardware_version
              << std::endl;
    std::cout << "* Manufacturer Software version = " << software_version
              << std::endl;
    std::cout << "* Node ID# " << static_cast<uint16_t>(device->get_node_id())
              << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "" << std::endl;
    std::cout << "" << std::endl;
    return true;
  } catch (...) {
    std::cout << "Exception occured while acquiring device information.!"
              << std::endl;
    return false;
  }
}

void initializeDevice(std::shared_ptr<kaco::Device> device,
                      uint16_t heartbeat_interval, uint8_t node_id) {
  // set the our desired heartbeat_interval time
  device->set_entry(0x1017, 0x0, heartbeat_interval,
                    kaco::WriteAccessMethod::sdo);

  // Mater side Periodic Tranmit pdo1 value initialization
  device->set_entry("target_velocity", 0x0, kaco::WriteAccessMethod::sdo);
  device->set_entry("controlword", static_cast<uint16_t>(0x0006),
                    kaco::WriteAccessMethod::sdo);
  // Master side tpdo1 mapping
  device->add_transmit_pdo_mapping(
      0x200 + node_id, {{"target_velocity", 0}, {"controlword", 4}},
      kaco::TransmissionType::PERIODIC, std::chrono::milliseconds(250));
  /// Master side RPDO mapping starts here; This must be in line with device
  /// side TPDOs
  // Master side rpdo1 mapping
  device->add_receive_pdo_mapping(0x180 + node_id, "velocity_actual_value",
                                  0);                                 // 32bit
  device->add_receive_pdo_mapping(0x180 + node_id, "statusword", 4);  // 16bit
  device->add_receive_pdo_mapping(0x180 + node_id, "error_code", 6);  // 16bit
  // Master side rpdo2 mapping
  device->add_receive_pdo_mapping(0x280 + node_id, "position_actual_value",
                                  0);  // 32bit
  device->add_receive_pdo_mapping(0x280 + node_id, "position_actual_value",
                                  4);  // 32bit
  // Master side rpdo3 mapping
  device->add_receive_pdo_mapping(0x380 + node_id, "digital_inputs",
                                  0);  // 32bit
  device->add_receive_pdo_mapping(0x380 + node_id, "torque_actual_value",
                                  4);  // 16bit
  device->add_receive_pdo_mapping(0x380 + node_id, "torque_actual_value",
                                  6);  // 16bit
  /// Master side RPDO mapping ends here

  /***************** TPDO MAPPING in DEVICE *****************/
  /// Device side TPDO mapping starts here. This must be in line with the
  /// master side RPDOs.
  // Device side tpdo1 mapping entries and mapping
  std::vector<uint32_t> tpdo1_entries_to_be_mapped = {0x606C0020, 0x60410010,
                                                      0x603F0010};
  mapTPDOinDevice(TPDO_1, tpdo1_entries_to_be_mapped, 255, device);
  // Device side tpdo2 mapping entries and mapping
  std::vector<uint32_t> tpdo2_entries_to_be_mapped = {0x60640020, 0x60640020};
  mapTPDOinDevice(TPDO_2, tpdo2_entries_to_be_mapped, 255, device);
  // Device side tpdo3 mapping entries and mapping
  std::vector<uint32_t> tpdo3_entries_to_be_mapped = {0x60FD0020, 0x60770010,
                                                      0x60770010};
  mapTPDOinDevice(TPDO_3, tpdo3_entries_to_be_mapped, 255, device);

  /// Device side TPDO mapping ends here;
  /***************** RPDO MAPPING in DEVICE *****************/
  /// Device side RPDO mapping starts here.This must be in line with the
  /// master side TPDOs.
  std::vector<uint32_t> rpdo1_entries_to_be_mapped = {
      0x60FF0020, 0x60400010,
  };
  mapRPDOInDevice(RPDO_1, rpdo1_entries_to_be_mapped, 255, device);
  /// Device side RPDO mapping ends here

  // Try to clear all possible errors in the CANOpen device
  device->set_entry("controlword", static_cast<uint16_t>(0x0080),
                    kaco::WriteAccessMethod::sdo);
}

int main() {
  // Signal handleing
  signal(SIGINT, intHandler);
  // Create core.
  kaco::Core core;

  // ----------- //
  // Preferences //
  // ----------- //

  // A Roboteq motor driver with firmware version v2.0beta07032018 was used to
  // test this program.//

  // The node ID of the slave we want to communicate with.
  const uint8_t node_id = 4;

  // Set the name of your CAN bus. "slcan0" is a common bus name
  // for the first SocketCAN device on a Linux system.
  const std::string busname = "test_can";

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

  volatile bool found_node = false;
  volatile bool device_connected = false;
  static int target_velocity = 0;
  static bool max = false;
  static bool min = false;
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
  core.nmt.send_nmt_message(node_id, kaco::NMT::Command::enter_preoperational);
  core.nmt.register_device_alive_callback([&](const uint8_t new_node_id) {
    // Check if this is the node we are looking for.
    if (new_node_id == node_id) {
      // lock
      if (!found_node) {
        found_node = true;
        // Lock device mutex
        std::lock_guard<std::mutex> lock(device_mutex);
        try {
          // Initialize the device
          core.nmt.send_nmt_message(node_id,
                                    kaco::NMT::Command::enter_preoperational);
          device.reset(new kaco::Device(core, node_id));
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          printDeviceInfo(device, core, node_id);
          std::string path = ros::package::getPath("kacanopen");
          boost::filesystem::path full_path =
              path + "/resources/eds_library/nanotec/CL4-E-2-12.eds";
          device->load_dictionary_from_eds(full_path.string());
          std::cout << "Printing Device Object Dictionary" << std::endl;
          device->print_dictionary();
          initializeDevice(device, heartbeat_interval, node_id);
          device->set_entry("controlword", static_cast<uint16_t>(0x0007),
                            kaco::WriteAccessMethod::sdo);
          // device->request_heartbeat(heartbeat_interval);
          device->set_entry("target_velocity", static_cast<int>(000),
                            kaco::WriteAccessMethod::sdo);
          int8_t set_mode_of_operation = 0x3;
          device->set_entry(0x6060, 0x00, set_mode_of_operation,
                            kaco::WriteAccessMethod::sdo);
          device->set_entry("controlword", static_cast<uint16_t>(0x000F),
                            kaco::WriteAccessMethod::sdo);
          device->start();
          device_connected = true;
        } catch (...) {
          std::cout << "Exception in device alive!" << std::endl;
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

  while (keepRunning) {
    if (device_connected) {
      // Lock device mutex
      std::lock_guard<std::mutex> lock(device_mutex);

      try {
        int32_t actual_vel =
            device->get_entry("velocity_actual_value",
                              kaco::ReadAccessMethod::pdo_request_and_wait);
        uint16_t statusword = device->get_entry(
            "statusword", kaco::ReadAccessMethod::pdo_request_and_wait);
        uint16_t controlword =
            device->get_entry("controlword", kaco::ReadAccessMethod::sdo);
        std::cout << "controlword=" << controlword << std::endl;
        int32_t actual_pos =
            device->get_entry("position_actual_value",
                              kaco::ReadAccessMethod::pdo_request_and_wait);
        //        int32_t current =
        //            device->get_entry("torque_actual_value",
        //                              kaco::ReadAccessMethod::pdo_request_and_wait);
        int16_t torque_actual =
            device->get_entry("torque_actual_value",
                              kaco::ReadAccessMethod::pdo_request_and_wait);
        double torque_to_display = static_cast<double>(torque_actual) / 1000;
        uint16_t error_code = device->get_entry(
            "error_code", kaco::ReadAccessMethod::pdo_request_and_wait);
        int16_t torque_actual_average =
            device->get_entry("torque_actual_value",
                              kaco::ReadAccessMethod::pdo_request_and_wait);
        double torque_to_display_avg =
            static_cast<double>(torque_actual_average) / 1000;
        //        double current_to_display = static_cast<double>(current) /
        //        1000;
        std::cout << "actual achieved velocity=" << actual_vel << std::endl;
        std::cout << "position_actual_value=" << actual_pos << std::endl;
        //        std::cout << "Motor Current=" << current_to_display <<
        //        std::endl;
        std::cout << "Torque Actual=" << torque_to_display << std::endl;
        std::cout << "Torque Actual Average=" << torque_to_display_avg
                  << std::endl;
        std::cout << "statusword=" << statusword << std::endl;
        std::cout << "Error Code=" << error_code << std::endl;
        int max_rpm = 1500;
        // Prepare the commands; master side tpdo1 and tpdo2
        if (max_rpm > target_velocity && max == false) {
          // channel1_speed_ref++;
          target_velocity = target_velocity + 10;
          if (max_rpm == target_velocity) {
            max = true;
          }
        }
        if (-max_rpm < target_velocity && max == true) {
          // channel1_speed_ref--;
          target_velocity = target_velocity - 10;
          if (-max_rpm == target_velocity) {
            max = false;
          }
        }
        device->set_entry("target_velocity", target_velocity,
                          kaco::WriteAccessMethod::pdo);
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
