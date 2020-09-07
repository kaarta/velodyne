#include <pcap/pcap.h>
#include <string>
#include <rosbag/recorder.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_msgs/VelodynePositionPacket.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <arpa/inet.h>

ros::Time parseInternalTime(uint8_t* bytes, ros::Time system_time) {
  uint32_t usec = (*(uint32_t*)bytes);

  double sensorSec = usec / 1000000.0;
  double computerTime = system_time.toSec();
  double sensorTime = 3600 * int(computerTime / 3600.0) + sensorSec;

  if (sensorTime - computerTime > 1800) sensorTime -= 3600;
  else if (sensorTime - computerTime < -1800) sensorTime += 3600;
  
  return ros::Time(sensorTime);
}

int getAzimuth(const velodyne_msgs::VelodynePacket& packet)
{
  static const int azimuth_data_pos = 100*0+2;
  int azimuth = *( (u_int16_t*) (&packet.data[azimuth_data_pos]));
  return azimuth;
}

  // from https://www.winpcap.org/docs/docs_412/html/group__wpcap__tut6.html
  /* 4 bytes IP address */
  typedef struct ip_address{
      u_char byte1;
      u_char byte2;
      u_char byte3;
      u_char byte4;
  }ip_address;

  /* IPv4 header */
  typedef struct ip_header{
      u_char  ver_ihl;        // Version (4 bits) + Internet header length (4 bits)
      u_char  tos;            // Type of service 
      u_short tlen;           // Total length 
      u_short identification; // Identification
      u_short flags_fo;       // Flags (3 bits) + Fragment offset (13 bits)
      u_char  ttl;            // Time to live
      u_char  proto;          // Protocol
      u_short crc;            // Header checksum
      ip_address  saddr;      // Source address
      ip_address  daddr;      // Destination address
      u_int   op_pad;         // Option + Padding
  }ip_header;

  /* UDP header*/
  typedef struct udp_header{
      u_short sport;          // Source port
      u_short dport;          // Destination port
      u_short len;            // Datagram length
      u_short crc;            // Checksum
  }udp_header;

  static const size_t packet_size =
    sizeof(velodyne_msgs::VelodynePacket().data);

int main(int argc, char*argv[])
{
  if (argc != 3 && argc != 4)
  {
    printf("Usage: pcap_to_bag pcap_in.pcap bag_out.bag [force model # (0 - VLP16, 1 - VLP32/32MR, or 2 - HDL32)]\n");
    return 1;
  }

  int force_model = -1;
  if(argc == 4 && strlen(argv[3]) > 0)
  {
    try
    {
      force_model = boost::lexical_cast<int>(argv[3]);
      std::cout << force_model << '\n';
    }
    catch (const boost::bad_lexical_cast &e)
    {
      std::cerr <<"Invalid 'force model' parameter given to pcap_to_bag: \""<<argv[3]<<"\"" << std::endl;
      std::cerr << e.what() << '\n';
      return 1;
    }
    
    printf("Forcing model to %d\n", force_model);
    if (force_model < 0 || force_model > 2)
    {
      std::cerr <<"Invalid 'force model' parameter given to pcap_to_bag: \""<<argv[3]<<"\" converts to integer: "
                << force_model <<" which is invalid"<<std::endl;
      return 1;
    }
  }

  pcap_t *pcap;
  bpf_program pcap_packet_filter;
  char errbuf[PCAP_ERRBUF_SIZE];

  std::string filename = argv[1];
  std::string filename_out = argv[2];

  if (!boost::filesystem::exists(filename))
  {
    printf("Input file does not exist: '%s'\n", filename.c_str());
    return 2;
  }

  pcap = NULL;
  printf("Opening PCAP file \"%s\"\n", filename.c_str());
  if ((pcap = pcap_open_offline(filename.c_str(), errbuf) ) == NULL)
  {
    printf("Error opening pcap file.\n");
    return 2;
  }

  int data_port = 2368;
  int position_port = 8308;

  std::stringstream filter;

  filter << "udp dst port (" << data_port << " || " << position_port << ")";
  pcap_compile(pcap, &pcap_packet_filter,
                filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);

  struct pcap_pkthdr *header;
  const u_char *pkt_data;
  static uint64_t num_position_packets = 0;
  static uint64_t num_lidar_data_packets = 0;

  rosbag::Bag bag;
  printf("Opening bag file: \"%s\"\n", filename_out.c_str());
  bag.open(filename_out, rosbag::bagmode::Write);

  bool has_wrote_pcap_identifier = false;

  velodyne_msgs::VelodynePositionPacket position_packet;
  velodyne_msgs::VelodynePacket lidar_packet;
  velodyne_msgs::VelodyneScan lidar_scan;

  lidar_scan.header.frame_id = "velodyne";

  auto filesize = boost::filesystem::file_size(filename);

  int detected_model = -1;

  uintmax_t num_packets_read = 0;
  bool file_is_empty = true;
  int res;
  printf("Processing pcap... \"%s\"\n", filename_out.c_str());
  while ((res = pcap_next_ex(pcap, &header, &pkt_data)) >= 0)
  {
    ++num_packets_read;

    ip_header *ih;
    udp_header *uh;
    u_int ip_len;
    u_short sport/* ,dport */;
    ih = (ip_header *) (pkt_data + 14); //length of ethernet header

    if (num_packets_read % 1000000 == 0)
    {
      printf("%lu packets read so far...\n", num_packets_read);
    }
    
    /* retireve the position of the ip header */
    if (0 == pcap_offline_filter(&pcap_packet_filter,
                                  header, pkt_data))
      continue;

    /* retireve the position of the udp header */
    ip_len = (ih->ver_ihl & 0xf) * 4;
    uh = (udp_header *) ((u_char*)ih + ip_len);

    /* convert from network byte order to host byte order */
    sport = ntohs( uh->sport );
    // dport = ntohs( uh->dport );

    if (sport == position_port){
      ++num_position_packets;
      memcpy(position_packet.data.data(), pkt_data+42, sizeof(velodyne_msgs::VelodynePositionPacket().data));
      position_packet.stamp = parseInternalTime( (uint8_t*)&position_packet.data[0]+0xC6,
                                                ros::Time(header->ts.tv_sec, header->ts.tv_usec));
      bag.write("/velodyne_position_packets", position_packet.stamp, position_packet);
      if (!has_wrote_pcap_identifier)
      {
        std_msgs::Empty empty;
        bag.write("/generated_pcap_to_bag", position_packet.stamp, empty);
        has_wrote_pcap_identifier = true;
      }
    }
    else if (sport == data_port)
    {
      ++num_lidar_data_packets;

      // velodyne point data
      memcpy(lidar_packet.data.data(), pkt_data+42, packet_size);
      lidar_packet.stamp = parseInternalTime((uint8_t*)pkt_data + 1242, ros::Time(header->ts.tv_sec, header->ts.tv_usec)); // time_offset not considered here, as no synchronization required

      switch(lidar_packet.data[0x4b5]){
        case 0x22:
          if (detected_model != 0)
          {
            detected_model = 0;
            std_msgs::Int32 laser_model;
            laser_model.data = force_model >= 0 ? force_model : detected_model;
            bag.write("/laser_model", lidar_packet.stamp, laser_model);
            printf("detected model: %d\n", detected_model);
          }
          break;
        case 0x28:
          if (detected_model != 1)
          {
            detected_model = 1;
            std_msgs::Int32 laser_model;
            laser_model.data = force_model >= 0 ? force_model : detected_model;
            bag.write("/laser_model", lidar_packet.stamp, laser_model);
            printf("detected model: %d\n", detected_model);
          }
          break;
        case 0x21:
          if (detected_model != 2)
          {
            detected_model = 2;
            std_msgs::Int32 laser_model;
            laser_model.data = force_model >= 0 ? force_model : detected_model;
            bag.write("/laser_model", lidar_packet.stamp, laser_model);
            printf("detected model: %d\n", detected_model);
          }
          break;

      }

      switch(force_model){
        case 0:
          lidar_packet.data[0x4b5] = 0x22;
          break;
        case 1:
          lidar_packet.data[0x4b5] = 0x28;
          break;
        case 2:
          lidar_packet.data[0x4b5] = 0x21;
          break;
        // default:
        //   break;
      }

      if (lidar_scan.packets.size() == 0)
      {
        lidar_scan.header.stamp = lidar_packet.stamp;
        lidar_scan.packets.push_back(lidar_packet);
      }
      // we've gone 360?
      else if (getAzimuth(lidar_scan.packets.back()) > getAzimuth(lidar_packet))
      {
        bag.write("/velodyne_packets", lidar_packet.stamp, lidar_scan);
        lidar_scan.packets.clear();
        lidar_scan.packets.push_back(lidar_packet);
        lidar_scan.header.stamp = lidar_packet.stamp;

        if (!has_wrote_pcap_identifier)
        {
          std_msgs::Empty empty;
          bag.write("/generated_pcap_to_bag", lidar_packet.stamp, empty);
          has_wrote_pcap_identifier = true;
        }
      }
      else
        lidar_scan.packets.push_back(lidar_packet);
    }

    file_is_empty = false;
  }

  printf("Finished reading file. %lu lidar data packets and %lu position packets read\n", num_lidar_data_packets, num_position_packets);

  if (file_is_empty)                 // no data in file?
  {
    ROS_WARN("Error %d reading Velodyne packet: %s. File has no velodyne data", 
              res, pcap_geterr(pcap));
    return 3;
  }

  // I can't figure out how to rewind the file, because it
  // starts with some kind of header.  So, close the file
  // and reopen it with pcap.
  pcap_close(pcap);

  bag.close();
  return 0;
}