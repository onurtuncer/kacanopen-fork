file(GLOB TARGETS_SRC
	"*.cpp"
	"test/*.cpp"
  # "ros/*.cpp"
)

foreach(TARGET_SRC ${TARGETS_SRC})
	get_filename_component(TARGET_NAME ${TARGET_SRC} NAME_WE)
	set(TARGET_NAME "kacanopen_example_${TARGET_NAME}")
	add_executable(${TARGET_NAME} ${TARGET_SRC})
  target_link_libraries(${TARGET_NAME}
    kacanopen_core
    kacanopen_master
  #  kacanopen_ros_bridge
    kacanopen_tools
  )
	list(APPEND KACANOPEN_EXAMPLES_TARGET_NAMES ${TARGET_NAME})
endforeach()

# # Install
# install(
#   TARGETS ${KACANOPEN_EXAMPLES_TARGET_NAMES}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

