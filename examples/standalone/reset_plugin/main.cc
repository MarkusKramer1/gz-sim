#include <iostream>

#include "gz/common/Console.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/DetachableJoint.hh"
#include "gz/sim/components/JointPositionReset.hh"
#include "gz/transport/Node.hh"

#include "plugin.hh"

using namespace gz;
using namespace gz::sim;

int main(int argc, char** argv) {
  ::gz::common::Console::SetVerbosity(4);

  if (argc < 2) {
    ignerr << "You did not pass a path to a SDF file.\n";
    return -1;
  }

  std::string sdf_path = argv[argc - 1];
  gz::sim::ServerConfig server_config;

  if (!server_config.SetSdfFile(sdf_path)) {
    std::cerr << "Failed to set SDF file [" << sdf_path << "]\n";
    return -1;
  }

  gz::sim::Server server(server_config);
  server.Run(true, 5000, false);
  ignmsg << "Server ran for " << server.IterationCount().value_or(0)
            << " iterations \n";

  // First reset
  ::gz::transport::Node node;
  gz::msgs::WorldControl reset_msg;
  reset_msg.mutable_reset()->set_all(true);
  gz::msgs::Boolean resp;
  bool success;
  if (!node.Request("world/default/control", reset_msg, 3000, resp, success) ||
      !success) {
    ignerr << "Failed to reset \n";
    return -1;
  } else {
    igndbg << "First reset" << std::endl;
  }
  kDidReset = true;

  server.Run(true, 5000, false);
  ignmsg << "Server ran for " << server.IterationCount().value_or(0)
         << " iterations \n";

  // Second reset
  if (!node.Request("world/default/control", reset_msg, 3000, resp, success) ||
      !success) {
    ignerr << "Failed to reset \n";
    return -1;
  } else {
    igndbg << "Second reset" << std::endl;
  }


  server.Run(true, 1000, false);

  return 0;
}
