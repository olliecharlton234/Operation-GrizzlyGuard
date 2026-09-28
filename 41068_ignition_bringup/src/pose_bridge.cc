// Resident set_pose client.
//
// Shelling out to `ign service` costs ~330 ms per call, which caps the bear
// animation at roughly 1.4 Hz and makes it visibly jerky. This keeps one
// ignition-transport node alive and pushes poses through it, so each call is
// sub-millisecond and the gait can run at 30 Hz.
//
// Reads one pose per line on stdin:
//     <model> <x> <y> <z> <qx> <qy> <qz> <qw>
// A line "sync" flushes nothing and simply acknowledges, for rate control.
#include <iostream>
#include <sstream>
#include <string>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/boolean.pb.h>

int main(int argc, char **argv)
{
  if (argc < 2) { std::cerr << "usage: pose_bridge <world>\n"; return 1; }
  const std::string service = std::string("/world/") + argv[1] + "/set_pose";

  ignition::transport::Node node;
  std::string line;
  while (std::getline(std::cin, line))
  {
    if (line.empty()) continue;
    std::istringstream is(line);
    std::string name;
    double x, y, z, qx, qy, qz, qw;
    if (!(is >> name >> x >> y >> z >> qx >> qy >> qz >> qw)) continue;

    ignition::msgs::Pose req;
    req.set_name(name);
    req.mutable_position()->set_x(x);
    req.mutable_position()->set_y(y);
    req.mutable_position()->set_z(z);
    req.mutable_orientation()->set_x(qx);
    req.mutable_orientation()->set_y(qy);
    req.mutable_orientation()->set_z(qz);
    req.mutable_orientation()->set_w(qw);

    ignition::msgs::Boolean rep;
    bool ok = false;
    node.Request(service, req, 200, rep, ok);
  }
  return 0;
}
