#!/bin/bash

if [[ $# != 2 && $# != 3 ]]; then
  echo "Usage: pcap_to_bag.sh <input pcap file> <output bag file> [force laser model #]"
  exit 1
fi

infile="$(realpath "$1")"
outfile="$(realpath "$2")"

if [ -f "$infile" ]; then
  # rosparam set use_sim_time false
  force_arg=""
  if [ $# == 3 ]; then
    force_arg="$3"
  fi

  rosrun velodyne_driver pcap_to_bag "$infile" "${outfile%.bag}_raw_conv.bag" "$3"
  res_pcap_to_bag=$?

  echo "Done converting pcap to bag"
  if [ ! $res_pcap_to_bag -eq 0 ]; then
    echo "pcap to bag exited with code $res_pcap_to_bag. Cannot continue"
    exit 10
  fi
  if [ ! -f "${outfile%.bag}_raw_conv.bag" ]; then
    echo "ERROR: No bag file was created by driver"
    exit 10
  fi

  # get rosbag info to see if there are packets in it
  rosbag_info=$(rosbag info "${outfile%.bag}_raw_conv.bag")
  echo "$rosbag_info"
  has_lidar_packets=$(echo "$rosbag_info" | grep -E '/velodyne_packets|/os1' && echo "true" || echo "false")
  if [[ "$has_lidar_packets" == "false" ]]; then
    echo "ERROR: Bag file does not contain lidar data. Please select a different bag file. Rosbag info = $rosbag_info"
    exit 15
  fi

  echo "Verifiying/Fixing bag integrity"
  
  rosrun stencil_scripts kaarta_bag_verification.py "${outfile%.bag}_raw_conv.bag" --fix --no_velo_timesync --output "$outfile"
  res_bag_fixing=$?
  if [ ! $res_bag_fixing -eq 0 ]; then
    echo "Failed to fix bag. Exit code = $res_bag_fixing"
    exit 11
  fi
  if [ ! -f "$outfile" ]; then
    echo "ERROR: Could not fix bag file"
    exit 11
  fi

  # get rosbag info to see if there are packets in it
  rosbag_info=$(rosbag info "$outfile")
  echo "$rosbag_info"
  has_lidar_packets=$(echo "$rosbag_info" | grep -E '/velodyne_packets|/os1' && echo "true" || echo "false")
  if [[ "$has_lidar_packets" == "false" ]]; then
    echo "ERROR: Bag file does not contain lidar data. Please select a different bag file. Rosbag info = $rosbag_info"
    exit 15
  fi
  real_path="$(realpath "$outfile")"
  echo "Finished converting pcap to bag success: Output = $real_path"
  rm "${outfile%.bag}_raw_conv.bag"
else
  echo "ERROR: Input file: $infile does not exist"
  exit 1
fi

exit 0
