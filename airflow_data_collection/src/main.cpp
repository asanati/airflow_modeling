
#include "Quad.h"
#include "fshelper.h"
#include <csignal>
#include <vector>

#define SWOOP
//#define PICK_AND_PLACE
//#define BOX_LIFT

std::string g_log;

void sigintHandler(int signum) { exit(signum); }

void exitHandler() {
  // check length
  if (g_log.length() > 2) {
    std::cout << "Safe log" << std::endl;
    saveLog(g_log);
  }
}

int main() {
  // register signal
  signal(SIGINT, sigintHandler);
  int x = atexit(exitHandler);
  // FASTDDS DEFAULT PARTICIPANT
  std::unique_ptr<DefaultParticipant> dp =
      std::make_unique<DefaultParticipant>(0, "raptor");

  /* CREATE PARTICIPANTS */
  // Gripper gripper("Gripper", &g_log, dp, "grip_cmd", GripperType::grip_rot);
  Quad quad("Quad", &g_log, dp, "mocap_srl_raptor_multi", "pos_cmd");

  Item vision_box("box", &g_log, dp, "vision_srl_box");
  Item mocap_box("mocap_srl_box", &g_log, dp, "mocap_srl_box");

  int grip_open = 5;
  int grip_close = 92;

  // gripper.setAngleAsym(grip_open, grip_open);

//   const std::vector<float> box_vec = box.getPoseAsVector();
  if (!quad.takeOff()) {
    std::cerr << "Terminate Process (" << __FILE__ << ":" << __LINE__ << ")"
              << std::endl;
    return 1;
  }
  std::vector<float> box_vec;


  std::cout << "initialized all subscribers" << std::endl;
  std::vector<float> mocap_box_coords = mocap_box.getPoseAsVector();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
 
 std::vector<float> x_vals = {-2.5, -2.25, -2.0, -1.75, -1.5, -1.25, -1.0, -0.75};

 float z_val = 0.35;
 quad.goToPos(mocap_box_coords, -1, 0.0, z_val+0.5, 0.0, 5000, false);
  // for(int i = 0; i<x_vals.size(); i++) {
  //   float x_coord = x_vals.at(i);

  //   quad.goToPos(mocap_box_coords, x_coord, 0.0, z_val, 0.0, 5000, false);
  //   std::this_thread::sleep_for(std::chrono::milliseconds(10000));

  //   // quad.goToPos(mocap_box_coords, x_coord, 1.5, z_val, 0.0, 5000, false);
  //   // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  //   // quad.goToPos(mocap_box_coords, x_coord, 1.0, z_val, 0.0, 5000, false);
  //   // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  //   // quad.goToPos(mocap_box_coords, x_coord, 0.5, z_val, 0.0, 5000, false);
  //   // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  //   // quad.goToPos(mocap_box_coords, x_coord, 0.0, z_val, 0.0, 5000, false);
  //   // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  //   // quad.goToPos(mocap_box_coords, x_coord, -0.5, z_val, 0.0, 5000, false);
  //   // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  //   // quad.goToPos(mocap_box_coords, x_coord, -1.0, z_val, 0.0, 5000, false);
  //   // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  //   // quad.goToPos(mocap_box_coords, x_coord, -1.5, z_val, 0.0, 5000, false);
  //   // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  // }

  
 
  
  quad.goToPos(-0.5, -0.5, 1.5, 0, 4000, true);
  // SWOOP FOR BOX
  // quad.quickSwoop(box_vec, gripper, 2, -0.06, 0.02, 0.04, 2.0, 0, 5);
  // quad.goToPos(2.82842, -0.490158, 2, 0, 4000, false);
  // quad.goToPos(drop_vec.at(0), -0.490158, 2, 0, 4000, true);
  // quad.goToPos(drop_vec.at(0) - 0.15, drop_vec.at(1) - 0.2, 2, 0, 3000,
  // true); quad.goToPos(drop_vec, -0.15, -0.1, 0.5, 0, 4000, true);
  // gripper.setAngleAsym(grip_open,grip_open);
  // quad.goToPos(drop_vec.at(0) - 0.15, drop_vec.at(1) - 0.0, 2.75, 0, 3000,
  // true);

  // intermediate land
  quad.goToPos(-0.5, -0.5, 1.5, 0, 4000, true);
  // gripper.setAngleSym(110);
  quad.goToPos(-0.5, -0.5, 0.1, 0, 2000, false);
  quad.emergencyLand();
}