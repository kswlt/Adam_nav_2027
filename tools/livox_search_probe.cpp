// Discovery only. Reuse Livox SDK2 packet/CRC code; never initialize its device manager.
// The SDK's full viewer initialization may change device data destinations.
#include "comm/comm_port.h"
#include "comm/define.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <set>

bool parse_reply(livox::lidar::CommPort& codec, uint8_t* bytes, int n,
                 livox::lidar::DetectionData& info) {
  using namespace livox::lidar;
  if (n < 24) return false;
  uint16_t length = 0; memcpy(&length, bytes+2, sizeof(length));
  if (length != n) return false;
  CommPacket ack{};
  if (!codec.ParseCommStream(bytes, n, &ack) || ack.cmd_id != kCommandIDLidarSearch ||
      ack.cmd_type != kCommandTypeAck || ack.sender_type != kLidarSend ||
      ack.data_len < sizeof(DetectionData)) return false;
  memcpy(&info, ack.data, sizeof(info));
  return info.ret_code == 0;
}

int main(int argc, char **argv) {
  if (argc != 2) { std::cerr << "usage: livox_search_probe INTERFACE\n"; return 2; }
  using namespace livox::lidar;
  CommPort codec;
  uint8_t bytes[1500]{};
  uint32_t length = 0;
  CommPacket request{};
  request.protocol = kLidarSdk; request.version = 0; request.seq_num = 1;
  request.cmd_id = kCommandIDLidarSearch; request.cmd_type = kCommandTypeCmd;
  request.sender_type = kHostSend;
  if (std::string(argv[1]) == "--self-test") {
    DetectionData fixture{}; fixture.dev_type = kLivoxLidarTypeMid360;
    memcpy(fixture.sn, "TEST_ONLY", 9); fixture.cmd_port = 56100;
    request.cmd_type = kCommandTypeAck; request.sender_type = kLidarSend;
    request.data = reinterpret_cast<uint8_t*>(&fixture); request.data_len = sizeof(fixture);
    DetectionData parsed{};
    if (codec.Pack(bytes, sizeof(bytes), &length, request) != 0 ||
        !parse_reply(codec, bytes, length, parsed) || parsed.dev_type != fixture.dev_type ||
        parsed.cmd_port != 56100 || strncmp(parsed.sn, "TEST_ONLY", 9) != 0 ||
        parse_reply(codec, bytes, length-1, parsed) || parse_reply(codec, bytes, 0, parsed)) return 4;
    request.cmd_type = kCommandTypeCmd;
    codec.Pack(bytes, sizeof(bytes), &length, request);
    if (parse_reply(codec, bytes, length, parsed)) return 4;
    std::cout << "SELF_TEST_ONLY: valid search ACK accepted; truncation and non-ACK rejected\n";
    return 0;
  }
  if (codec.Pack(bytes, sizeof(bytes), &length, request) != 0) return 3;
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) { perror("socket"); return 3; }
  int on = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE, argv[1], strlen(argv[1])+1) < 0 ||
      setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) < 0) {
    perror("socket options"); close(fd); return 3;
  }
  timeval tv{0, 100000}; setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  sockaddr_in local{}; local.sin_family = AF_INET; local.sin_port = htons(kDetectionPort);
  if (bind(fd, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
    perror("bind (discovery port must be unused)"); close(fd); return 3;
  }
  unsigned sent = 0, failed = 0, received = 0;
  std::set<std::string> identities;
  const auto begin = std::chrono::steady_clock::now();
  auto next_send = begin;
  while (std::chrono::steady_clock::now()-begin < std::chrono::seconds(10)) {
    if (std::chrono::steady_clock::now() >= next_send) {
      // Limited broadcast is not restricted to the remembered radar IP.
      for (const char* target : {"255.255.255.255", "192.168.1.52", "192.168.1.152"}) {
        sockaddr_in dest{}; dest.sin_family = AF_INET; dest.sin_port = htons(kDetectionPort);
        inet_pton(AF_INET, target, &dest.sin_addr);
        if (sendto(fd, bytes, length, 0, reinterpret_cast<sockaddr*>(&dest), sizeof(dest)) >= 0) ++sent;
        else ++failed;
      }
      next_send += std::chrono::seconds(1);
    }
    uint8_t response[1500]{}; sockaddr_in peer{}; socklen_t peer_size = sizeof(peer);
    const auto n = recvfrom(fd, response, sizeof(response), 0, reinterpret_cast<sockaddr*>(&peer), &peer_size);
    if (n < 24) continue;
    ++received;
    // SDK parser trusts the encoded length; parse_reply checks truncation first.
    DetectionData info{};
    if (!parse_reply(codec, response, n, info)) continue;
    char ip[INET_ADDRSTRLEN]{}; inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
    const std::string sn(info.sn, strnlen(info.sn, sizeof(info.sn)));
    if (identities.insert(std::string(ip)+":"+sn).second)
      std::cout << "LIVOX_DEVICE source_ip=" << ip << " sn=" << sn
                << " type=" << unsigned(info.dev_type) << " command_port=" << info.cmd_port << std::endl;
  }
  close(fd);
  std::cout << "interface=" << argv[1] << " send_accepted=" << sent << " send_errors=" << failed
            << " datagrams=" << received << " devices=" << identities.size() << std::endl;
  return identities.empty() ? 1 : 0;
}
