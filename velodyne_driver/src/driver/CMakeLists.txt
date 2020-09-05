# build the driver node
add_executable(pcap_to_bag pcap_to_bag.cc)
add_dependencies(pcap_to_bag velodyne_driver_gencfg)
target_link_libraries(pcap_to_bag
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

add_executable(velodyne_node velodyne_node.cc driver.cc)
add_dependencies(velodyne_node velodyne_driver_gencfg)
target_link_libraries(velodyne_node
  velodyne_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# build the nodelet version
add_library(driver_nodelet nodelet.cc driver.cc)
add_dependencies(driver_nodelet velodyne_driver_gencfg)
target_link_libraries(driver_nodelet
  velodyne_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# install runtime files
install(TARGETS velodyne_node pcap_to_bag
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
install(TARGETS driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
