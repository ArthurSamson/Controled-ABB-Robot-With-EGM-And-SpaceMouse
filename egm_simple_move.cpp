#include <arpa/inet.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

#include "egm.pb.h"

namespace
{
constexpr std::size_t kMaxUdpPayload = 2048;

struct OffsetMm
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct RpyDeg
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

abb::egm::EgmQuaternion rpyDegToQuaternion(const RpyDeg &rpy_deg)
{
  const double roll = rpy_deg.roll * M_PI / 180.0;
  const double pitch = rpy_deg.pitch * M_PI / 180.0;
  const double yaw = rpy_deg.yaw * M_PI / 180.0;

  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  abb::egm::EgmQuaternion q;
  q.set_u0(cr * cp * cy + sr * sp * sy); // w
  q.set_u1(sr * cp * cy - cr * sp * sy); // x
  q.set_u2(cr * sp * cy + sr * cp * sy); // y
  q.set_u3(cr * cp * sy - sr * sp * cy); // z
  return q;
}

std::uint64_t monotonicMs()
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1000ULL +
         static_cast<std::uint64_t>(ts.tv_nsec) / 1000000ULL;
}

} // namespace

int main(int argc, char **argv)
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0]
              << " <local_udp_port> [dx_mm dy_mm dz_mm roll_deg pitch_deg yaw_deg]\\n";
    return 1;
  }

  const int local_port = std::stoi(argv[1]);
  OffsetMm target_offset;
  RpyDeg target_rpy;

  if (argc == 8)
  {
    target_offset.x = std::stod(argv[2]);
    target_offset.y = std::stod(argv[3]);
    target_offset.z = std::stod(argv[4]);
    target_rpy.roll = std::stod(argv[5]);
    target_rpy.pitch = std::stod(argv[6]);
    target_rpy.yaw = std::stod(argv[7]);
  }
  else if (argc != 2)
  {
    std::cerr << "Invalid argument count. Provide either only the port or all 6 offsets.\\n";
    return 1;
  }

  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0)
  {
    std::perror("socket");
    return 1;
  }

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY;
  local_addr.sin_port = htons(static_cast<uint16_t>(local_port));

  if (bind(sock, reinterpret_cast<sockaddr *>(&local_addr), sizeof(local_addr)) < 0)
  {
    std::perror("bind");
    close(sock);
    return 1;
  }

  std::cout << "Listening on UDP port " << local_port << " and waiting for EGM packets...\\n";

  std::uint32_t seqno = 1;
  uint8_t buffer[kMaxUdpPayload]{};

  while (true)
  {
    sockaddr_in robot_addr{};
    socklen_t robot_addr_len = sizeof(robot_addr);

    const ssize_t received = recvfrom(sock,
                                      buffer,
                                      sizeof(buffer),
                                      0,
                                      reinterpret_cast<sockaddr *>(&robot_addr),
                                      &robot_addr_len);
    if (received < 0)
    {
      std::perror("recvfrom");
      break;
    }

    abb::egm::EgmRobot feedback;
    if (!feedback.ParseFromArray(buffer, static_cast<int>(received)))
    {
      std::cerr << "Warning: invalid EGM feedback frame, ignored.\\n";
      continue;
    }

    if (!feedback.has_feedBack() || !feedback.feedBack().has_cartesian())
    {
      std::cerr << "Warning: feedback without cartesian pose, ignored.\\n";
      continue;
    }

    const auto &current_pose = feedback.feedBack().cartesian();
    const auto &current_pos = current_pose.pos();

    abb::egm::EgmSensor command;
    auto *header = command.mutable_header();
    header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
    header->set_seqno(seqno++);
    header->set_tm(static_cast<uint32_t>(monotonicMs()));

    auto *planned_pose = command.mutable_planned()->mutable_cartesian();

    auto *planned_pos = planned_pose->mutable_pos();
    planned_pos->set_x(current_pos.x() + target_offset.x);
    planned_pos->set_y(current_pos.y() + target_offset.y);
    planned_pos->set_z(current_pos.z() + target_offset.z);

    const auto q = rpyDegToQuaternion(target_rpy);
    auto *planned_orient = planned_pose->mutable_orient();
    planned_orient->set_u0(q.u0());
    planned_orient->set_u1(q.u1());
    planned_orient->set_u2(q.u2());
    planned_orient->set_u3(q.u3());

    std::string out;
    if (!command.SerializeToString(&out))
    {
      std::cerr << "Error: failed to serialize EGM command frame.\\n";
      break;
    }

    const ssize_t sent = sendto(sock,
                                out.data(),
                                out.size(),
                                0,
                                reinterpret_cast<sockaddr *>(&robot_addr),
                                robot_addr_len);

    if (sent < 0)
    {
      std::perror("sendto");
      break;
    }

    std::cout << "Sent correction seq=" << (seqno - 1)
              << " pos(mm)=[" << planned_pos->x() << ", " << planned_pos->y() << ", "
              << planned_pos->z() << "] rpy(deg)=[" << target_rpy.roll << ", "
              << target_rpy.pitch << ", " << target_rpy.yaw << "]\\r" << std::flush;
  }

  close(sock);
  google::protobuf::ShutdownProtobufLibrary();
  std::cout << "\\nStopped.\\n";
  return 0;
}
