^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kacanopen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2020-03-23)
------------------
* Merged in bugfix/datatype-mismatch (pull request #4)
  Bugfix/datatype mismatch
  Approved-by: Musarraf Hossain Sekh <mhs@blue-ocean-robotics.com>
* Guarding get entry from invalid entries
* Guarding the get value from ivnalid entries
* NOT loading default EDS files at device->start(). This seems to be causing the original bug
* added backwards_ros
* disabled unnecessary eds loading warning
* reverted the temporary changes of defult CiA eds entries
* disabled default CiA dictionary
* heartbeat sending thread closing exposed as public menthod
* added send_consumer_heartbeat method
* reverted to original author's naming convention
* cleaned up send_heartbeat method to make it generic
* added nanotec example
* Contributors: Jorge Rodriguez, Musarraf, Musarraf Hossain, Seto

0.2.6 (2019-08-19)
------------------
* method naming convention fix
* renamed enums
* Contributors: Musarraf

0.2.5 (2019-04-01)
------------------
* added more pdo mapping entries
* EDS file added for Infranor
* cleaned up and added generic sdo parsing
* fixed the eds path
* fixed the speed index
* Contributors: Musarraf

0.2.4 (2019-02-15)
------------------
* reduced the speed step size
* renamed the sdo parsing method
* general cleanup and some casting fix
* thread_created flag is now false after the thread is joined
* renamed the method and corrected compiler warning
* Contributors: Musarraf

0.2.3 (2019-02-13)
------------------
* request_heartbeat thread closing was corrected for the devices only  which has created the thread
* added legacy mode and firmware check
* Contributors: Musarraf

0.2.2 (2019-02-07)
------------------
* Merged in feature (pull request #3)
  Feature
  Approved-by: Aswin Thomas <ata@blue-ocean-robotics.com>
  Approved-by: Naveed Usmani <nu@blue-ocean-robotics.com>
* fixed PR review comments
* moved the object dictionary to the appropriate folder
* added backward compatible example for roboteq firmware v2.0 beta and v2.0 RC
* added overloaded method for transmit pdo mapping by index and subindex
* overloaded the prototype  the add_transmit_pdo_mapping
* added MappingByIndex struct
* removed roboteq_object_dictionary_v2.0.txt from eds_library
* overloaded add add_receive_pdo_mapping method to map entry by  index and subindex
* fixed the eds path after refactoring
* first complete port for roboteq firmware version 2.0
* edited the eds as per roboteq's suggestion to fix the PDO mapping issue
* some minor cleanup
* moved the object dictionary file to the appropriate folder
* added roboteq eds released on 30-01-2019
* Contributors: Musarraf, Musarraf Hossain, Musarraf Hossain Sekh

0.2.1 (2019-01-30)
------------------
* changed the request_heartbeat_thread\_ to smart pointer to avoid duplicate thread creation
* added sdo transaction for clearing all possible errors in the canopen device.
* resolved merge conflicts
* fixed the device exception
* added more TPDO mappingg in device
* checked master side heartbeat
* added master side heartbeat request method
* mapped two  entries in master tdpo1 and  tested with EPOS4
* added object_dictionaries folder in resources
* added few more examples
* replaced "CMAKE_BUILD_TYPE EQUAL" with "CMAKE_BUILD_TYPE STREQUAL" to fix the example sub directory build issue
* overloaded map_tpdo_in_device to support maxon EPOS series
* added new eds file for Maxon EPOS4
* Contributors: Musarraf, Musarraf Hossain

0.2.0 (2018-10-28)
------------------
* Merged in hotfix/catkin-install (pull request #1)
  Refactoring
* using rosdoc instead  of raw doxygen
* ignored manifest.yaml from rosdoclite
* merged files to simplify file tree
* formated with clang-format
* removed unused CMakeSettings
* removed unused libs folder
* moved socket driver to projects structure
* cleanup cmakefile
* removed unnecesary scripts and installing all in the installation step
* renamed dev for scripts to follow convention
* removed travis ci file
* removed dummy driver option
* removed cmake hardcoded variables
* removed unused flags
* using ros package to remove the hardcoding paths
* ignored binarization folders
* compiling examples only if not Relase
* applied catkin_lint recommendations
* compiling examples succesfully
* updated examples and moved  roboteeqs eds to share folder
* using package format 2 in pacakge.xml
* formated with clang-format
* updated headers and keep refactoring
* moved files to catkin structured folders
* installing headers as defined in catkin_package macro
* installing headers in same path as specified by catkin_package include_dirs
* added pipelines yml
* Started porting to v2.0 firmware
* Little house keeping
* first working position control
* added max_rpm as variable
* Added roboteq driver firmware version as newer version of roboteq firmware is not compatible anymore
* fixed simple_pdo_rw.cpp for periodic transmit pdo mapping.
* Added the fix for Periodic Transmit PDO mapping.
* removed 'alive_devices\_.erase' to avoid invalid iterator
* rewritten with unordered map
* temporary fix to invalid iterator while erasing dead device in NMT::register_device_dead_callback
* Random housekeeping
* added dynamic file path for eds file
* removed console printing from device_rpdo.cpp and device_tpdo.cpp
* Fixed compiler warnings
* House keping in example and tools.
* used tempoary fix to handle channel2 speed feedback due to roboteq bug
* Added new example for multiple slave
* Merge branch 'master' of bitbucket.org:blue-ocean-robotics/kacanopen
* Fixed the memory access viloation issue while remapping pdos..
* Fixed the memory access viloation issue while remapping pdos..
* Added device reconnection in simple_pdo_rw.cpp
* Handled device disconnection. Device reconnection is yet to be implemented
* Added register_device_dead_callback
* Merge branch 'master' of bitbucket.org:blue-ocean-robotics/kacanopen
* Added seprate mapping for channel 1 and channel 2 speed command
* Added seprate mapping for channel 1 and channel 2 speed command
* fixed qry_digout mapping issue
* Added device side rpdo mapping.
* Implemented device side tpdo mapping. README file typo corrected.
* Edited the readme file for the example codes.
* Edited the readme file for the example codes.
* Finaly PDO mapping is working. Jorge Rodriguez, You are a jem !!
* separated the parse.h into a different folder 'utils'
* Fixed linking issue with parse_sdo.h
* Separate the sdo parsing code into a header
* Added dictionary printing, and few PDO read
* Initializing entry with default contructor instead of emtpy constructor.
  It fixes bug where variables where not initialized (e.g. disabled flag)
* Added current read sdo
* separated the the sdo parser code as function and corrected for 32bit error
* added sdo read perser to simple_sdo_rw.cpp
* Added a new example for PDO read write
* We got CAN communication to change speed of motors via roboteq
* added a infinite while loop in  simple_sdo_rw
* removed reset all command from simple_sdo_rw
* created a new example for simple sdo read wrire
* Removed compiler checking
* added generic master
* Merge pull request #7 from KITmedical/issue5
  Fixing issue #5
* Fixing issue #5
* Fixing issue #4
* Docs: Readme / Installation update.
* Examples: Set busname and baudrate locally.
* Numbering duplicate EDS object entries.
* Fixing return value of Device::load_dictionary_from_library().
* Adding a new Master example.
* Device::load_dictionary_from_library returns the EDS file path.
* Making Master::start() compatible to Core::start(). (#cfc7c09 regression)
* Renaming misleading NMT::NewDeviceCallback to DeviceAliveCallback.
* Better Core library example.
* Adding missing documentation.
* Fixing Doxyfile and update_online_docs.sh.
* Improving compatibility to CanFestival drivers by passing baudrates like "500K" instead of "500000".
* Fixing https://github.com/KITmedical/kacanopen/commit/e02bfde21ac3eb9c5de8be12084e2805ebe642c7#commitcomment-17958453
* Merge pull request #1 from a-w/master
  Fixed missing initialization of is_generic and m_valid
* Fixed missing initialization of is_generic and m_valid
* get/set_entry_via_sdo(): Sleep between repetitions and better logging.
* Schunk EBRo hack no longer needed with latest generic 402.eds.
* Also load generic names in Device::load_dictionary_from_eds().
* Error handling in Device class completely based on exceptions now. Mind missing return value in start() and load_dictionary_from_eds()!
  - Missing include in Utils class.
* 402.eds: Position actual/demand value: Stick to standard regarding unit distinction (with star (*) and without).
* Exception based error handling for Value and Entry.
* Adding Device::add_entry() method.
* Breaking change: Removing deprecated array index functionality. Adding index/subindex overloads. Making get/set_entry_via_sdo private.
  KaCanOpen uses separate entries for each array index for a while now...
* Adding generated 301.eds.
* Reverting 301.eds and 401.eds to master branch version until CiA_document_to_eds.py works properly.
* Dictionary and EDS library redesign. Now there can be multiple names for one entry. CiA standard-conformal names are added on top of manufacturer-specific dictionaries.
* Better error handling in Utils::hexstr_to_uint() and Utils::decstr_to_uint().
* Renaming pdftoeds.py.
* Fixing pdftoeds.py.
* Dictionary is now a hash map from address to entry together with a separate name to address mapping.
* Merge branch 'eds_redesign' into eds_redesign_intermediate_merge
* Adding USBtin init script.
* Fixing Device::set_entry_via_sdo()
* Better logging in get/set_entry_via_sdo().
* Fixing SDO timeout error. Access to m_send_and_wait_receivers still has to be synchronized...
* Fixing set/unset_debug_flags.sh
* Removing unnecessary stop().
* TODO list update.
* Adding development scripts for setting debug flags.
* Reducing debug logging a bit.
* Fixing download URLs.
* Adding gloal runtime config class. get_entry_via_sdo() can now be repeated when an SDO timeout occurs. Set Config::repeats_on_sdo_timeout accordingly.
* Don't terminate on SDO timeout in ros_bridge.
* Simplifying SDO callbacks using arrays -> less synchronization.
* Fixing missing-braces warning.
* Relaxed locking in send_sdo_and_wait().
* Adding data types UNSIGNED64, INTEGER64 and OCTET_STRING.
* Adding ability to disable entries after device reports non-existance of OD entry.
* pdftoeds.py: Adding support for record types + some fixes. Regen of 301.eds.
* Making symlink relative.
* Removing outdated eds directory. Adding symlink to eds_library instead.
* Utils: Better logging.
* EDSReader now removes trailing comments from INI values.
* New system for reliably matching manufacturer specific EDS files to connected devices using a JSON config file. Also adding some more EDS files by SYSTECelectronic and MaxonMotor.
* Adding a python script which parses a CiA standard profile document (PDF) and generates an EDS file from it.
  This is still in development and part of a redesign of the EDS subsystem. Standard EDS files are planned
  to be preferred over manufacturer-specific files for common fields in future.
* Adding Device::read_complete_dictionary().
* SDOReceivedCallback takes response by value so it's prepared to be called asynchronously (in future). Also cleaning up send_sdo_and_wait() interface and adding some comments and verbal asserts.
* Possible fix of concurrency bug in send_sdo_and_wait() at high bus load.
  When a timeout occurred, the receiver was not removed and it accessed invalid
  data on the stack (in the small timeframe before std::terminate or when
  catching sdo_error).
* Minor change in README.md
* README shields
* Introducing Travis continuous integration
* Restructuring drivers and adding new dummy driver. New CMake arguments DRIVER and BUILD_DRIVERS (see docs). CAN_DRIVER_NAME is now deprecated. Different license of CanFestival drivers is more explicit now.
* Merge branch 'master' of github.com:KITmedical/kacanopen
* PEBCAK
* Installation docs.
* no comment
* Docs
* Fixing identation.
* Merge branch 'async_bridge'
* Relevant parts of Master and Device are now thread-safe - see documentation of Device class for details.
  PDO mappings are stored in a forward_list now and some copy/move constructors are deleted.
* Revert "enable async spinner; no problems in preliminary tests and greatly improves performance (poll frequency) on low-end systems"
  Thread-safety not guaranteed. Use / merge async_spinner branch locally.
  This reverts commit 385c2a24913dd219fd232c8c7063c48b3f807a25.
* enable async spinner; no problems in preliminary tests and greatly improves performance (poll frequency) on low-end systems
* remove another debug output
* remove debug output
* many lines of code to make cmake 2.8 compatible to CMAKE_CXX_COMPILER\_* flags
* Adding script for automatic online documentation updates.
* Updating link to online docs.
* Docs. Copy constructors removed explicitly.
* Entry objects are now thread-safe.
* Getting rid of Entry's copy constructor.
* SDO: Fixing thread-safety of callback removal.
* Decoupling Publishers and Subscribers. You can set individual loop rates now.
* Fixing .gitignore
* Core is now thread-safe.
* Store futures returned by std::async -> avoid immediate blocking.
* Replacing typedef -> alias.
* Core example update.
* Adding Cia 402 controlword and statusword flags to constants. New convenience operations for target position and cw/sw flag setting.
* Load operations and constants on device startup so they can be used internally.
* Value: Adding string literal constructor for better overload resolution.
* Better error handling in JointState pub/sub.
* Increasing pause between two consecutively sent frames. Buffer in socket driver could otherwise overflow. New CMake parameter CONSECUTIVE_SEND_PAUSE_MS.
  TODO: Improve socket driver so it blocks when buffer is full?
* Hotplugging support for motor_and_io_brigde example.
* Device discovery is now based on node guard protocol. Call master.core.nmt.reset_all_nodes() explicitly if you need that. Attention: Semantics of new_device_callbacks have changed. It's more like a device is alive callback. Furthermore cachting node id collisions in master now.
* Fixing NMT::process_incoming_message()
* Renaming joint state example to motor_and_io_bridge
* Fixing identation
* TODO: The good ones go into the pot, the bad ones go into your crop.
* set header.stamp
* joint_state example: Use all connected CiA 402 devices.
* Making PDO example independent of node ID and number of nodes.
* send_sdo_and_wait(): Using std::future in order to avoid busy waiting.
* listdevices
* Introducing runtime convenience operations and constants.
* TODO list: Doxygen mainpage finished.
* EDSLibrary: Adding some debug output.
* EDSReader: Check result of parse_var.
* Disabling debug output by default.
  Note: Reverted to in-class static const initialization. Fine for integral types in C++11.
* Entry: read_write_mutex is stored in unique_ptr now. Added appropriate copy constructor and operator.
* Clear dictionary in EDSLibrary instead of in Device.
* Storing device objects as unique_ptr now. This is necessary in order to have
  persistent references, like they are used for example in eds_reader. User
  code still works with references, internal storage is abstracted away now.
* Don't move data into callback - there could be more than one.
* Ignore warnings in external libraries + correct c++14 compiler flag.
* Fixing warnings.
* Consistent identation.
* Fixing warnings (-Wall and -Wextra).
* do not initialize ros node in bridge (must be done outside)
  Conflicts resolved:
  examples/ros/joint_state.cpp
  examples/ros/ros.cpp
  ros_bridge/src/bridge.cpp
* add some error handling
  Conflicts resolved:
  ros_bridge/include/joint_state_publisher.h
  ros_bridge/include/joint_state_subscriber.h
  ros_bridge/src/joint_state_publisher.cpp
  ros_bridge/src/joint_state_subscriber.cpp
* +Device::has_entry()
  Conflicts resolved:
  master/include/device.h
  master/src/device.cpp
* fix catkin_package exports
* for consistency also print dictionary
  Conflicts resolved:
  examples/ros/ros.cpp
* fix c&p error
* Merge commit 'd3b97ca373d962c13c9c04fa6ca62e366038625b' into merge_ahb
  This is everything before clang-format.
  Using SDO_TIMEOUT_MS directly.
  TODO: Static initializer is good in C++14!
  Conflicts:
  core/include/sdo.h
* Fixing debug build. Minimum Clang version is now 3.6!
* old logic did not actually work after catkin clean; note to self: after changing a CMakeLists.txt ALWAYS test from an empty build dir
* indent (which has to be fixed everywhere), mix of tabs and spaces
* ups forgot breaks
* more specific errors
* static members, which are declared in the .h must be initialized in the .cpp file
  Add wait-for-device loop.
* important TODO
* do not force optimization level (achieved by catkin profile); consistent DOS line endings;
* relax cmake_minimum_required; auto detect g++-4.9 (e.g. on Ubuntu 14.04 with ppa:ubuntu-toolchain-r/test
* Updating drivers README.
* Serial driver: Fixing warning on clang-3.6.
* Making PEAK linux driver build process more robust.
* Making lincan driver more portable. Fixes warnings on 64-bit machines.
* Fixing LinCAN driver makefile.
* Adding link to Doxygen docs on gh-pages.
* Doxygen: Output in /html, turned off Latex.
* Removing outdated .gitremotes.
* Docs: Detailed build instructions, new design, and better Markdown/GitHub integration.
* TODO + Ideas for slave library
* Missing Doxygen documentation added.
* Removing hard-coded busname and baudrate. You might need to rerun CMake.
* Fixing PDO class.
* Adding Doxyfile and documentation for the PDO class and all examples.
* PDO class: move semantics and better logging and error handling.
* Master: Introducing proper error handling. See class dictionary_error.
  Minor change: Move semantics for pdo_received_callback binding.
* Core: Adding proper error handling. See canopen_error and sdo_error.
* Adding CMake parameter for SDO response timeout.
* Removing explicit move from return values. Could prevent elision.
* Adding floating point data types REAL32 and REAL64.
* Using EDS library for device specialization now.
  Attention (1): Entries associated with a subindex are now prefixed with parent's name!
  Attention (2): Boost filesystem is now a run-time dependency!
  - Updating examples to the new entry names.
  - New eds_library example.
  - Removing hard-coded CiA profiles (except CiA 402 ModeOfOperation)
* Improving regular expression for EDS section.
* Adding EDS library located in master/share. Improved file lookup path for eds_exmample. The path is now platform and installation independent.
* All user defined entry names are being escaped now. Also making entry name parameters constant references.
* Adding Utils::data_type_to_string -> better error messages.
* Adding missing CanOpen data types.
* Fixing PDO received callback.
* eds Faulhaber
* systec eds
* Fixing Entry default constructor.
* Calling stop() in Core and Master destructors -> shutting down properly in case of abortion.
* Adding EDSReader class. It imports entries from a CiA-306 EDS file into a dictionary map. There is also an example program and an example EDS file.
* New build requirements: GCC>=4.9 (first version with regex support) and boost>=1.46.1 (first version with bug-free property_tree).
* Adding Device::print_dictionary() -> prints all available entries together with current values and other properties.
* Fixing TransmitPDOMapping.
* Disabling debug loggin in Value class.
* Adding various conversion methods to Utils class. Better doxygen comments.
* Entry class: New methods valid(), print() and operator<() for sorting. Better doxygen comments.
* Moving all type enums into types.h.
* Fixing logger.
* Merge branch 'master' of gitlab.ira.uka.de:thomaskeh/kacanopen
* Adding TODO document.
  Little changes to README.md.
* Better packaging. Added install targets. Some reordering. New option INSTALL_EXAMPLES (default is OFF).
* Merge branch 'master' of gitlab.ira.uka.de:thomaskeh/kacanopen
* Adding JointStateSubscriber.
  Correct initialization of motor device.
* +remotes
* Adding a basic JointStatePublisher class, which publishes CiA 402 motor states as JointState messages.
  Adding an example for JointStatePublisher usage.
* Outsourcing CiA profile specific things into seperate files and namespaces.
* Adding instructions for specifying the CAN driver when using catkin_make.
* Fixing catkin_make.
* Adding a Subscriber interface and an EntrySubscriber class for writing dictionary entries from ROS. EntryPublisher and EntrySubscriber now use ROS std_msgs matching the entry type.
  Further changes:
  - Removing Master dependency from Bridge class.
  - ROS advertising transferred to advertise(), called by Bridge, so there are no more conflicts with ros::init().
* Adding Utils::escape() which escapes characters which are illegal in ROS topic names.
* Adding access methods for dictionary entry type.
* Adding kacanopen_ros library.
  See README.md and kacanopen_examples/ros_example.txt for details.
  Note that ROS Jade base is now required: http://wiki.ros.org/jade/Installation
  Build process has changed. You can still build with CMake and without ROS
  using the CMake flag -DNO_ROS=On.
  A how-to about kacanopen_ros usage will follow.
* Little fix concerning Value comparison
* Adding device::get_node_id()
* Introducing device specialization according to CiA profile number.
* Adding boolean data type.
* PDO mappings: now using a single offset variable instead of first_byte and last_byte.
* All code concerning the byte representation of values is now concentrated in the Value class.
* Adding a tag to Entry constructor to discriminate between array and variable entries.
* Adding a PDO example: PDO based counter for CiA 401 devices.
* Introducing transmit PDO mappings.
  Further changes:
  - Splitting read/write access method
  - Entry class: mutex for set/get_value()
  - CMake: C++14 flag for kacanopen_profiles
  - Fixing logging in release mode
  - More documentation
* Value class: Adding get_bytes() method and compare operators.
* Calling message received callbacks forced asynchronously.
* Fixing sdo_response for expedited transfer.
* Improved Logging. Adding a Value printer. New CMake option EXHAUSTIVE_DEBUGGING.
* - Adding PDO mapping functionality.
  - get_entry() and set_entry() can now be called with a new AccessMethod argument,
  - which specifies if the value shoud be fetched/set via SDO or if only the
  - cached value should be returned (probably set by a PDO mapping).
  - PDO callbacks now use cob_id instead of node_id -> more generic approach.
  - Entry type now fully supports arrays.
  - Minor changes:
  - splitting Entry struct in header and implementation.
  - new Utils method get_type_size().
* Refactoring:
  - Renaming structs and enums so they are camel-cased, expressive and have no trailing _type.
  - message_type -> Message
  - command -> Command
  - callback_type -> MessageReceivedCallback
  - ...
  - Splitting value.h in header and implementation -> reducing macro pollution.
  - Splitting utils.h in header and implementation.
  - Moving CANBoard and CANHandle types from defines.h to core.h.
  - Renaming defines.h to logger.h and including it _only\_ in .cpp-files -> reducing macro pollution.
* Fixing typos in README.md.
* Initial commit. The work on KaCanOpen originally started on October 16, 2015. See README.md for details.
* Contributors: Adrian Weiler, Aswin Thomas, Jorge Rodriguez, Julien Mintenbeck, Musarraf, Musarraf Hossain Sekh, Thomas Keh, ahb, clio, thk1
