#include <chrono>
#include <cmath>
#include <fstream>
#include <google/protobuf/util/time_util.h>
#include <iostream>
#include <string>
#include <ctime>

#include "detection_msg.pb.h"

#include "frame_conversions.h"
#include <zmq.hpp>

#include "Mocap_msg.h"
#include "Mocap_msgPubSubTypes.h"
#include "domain_participant.h"
#include "publisher.h"

#include "Item.h"
#include "Quad.h"

int main() {
  long frame_id = 0;
  std::string log;
  // FastDDS default participant
  std::unique_ptr<DefaultParticipant> dp =
      std::make_unique<DefaultParticipant>(0, "raptor_vision");
  Item quad("Drone", dp, "mocap_srl_raptor_multi");
  Item box("Box", dp, "mocap_srl_box");
  //   Item quad("Cam", dp, "mocap_srl_realsense");

  DDSPublisher pub = DDSPublisher(idl_msg::Mocap_msgPubSubType(),
                                  "vision_srl_box", dp->participant());

 

  //  Prepare our context and socket
  zmq::context_t context(1);
  zmq::socket_t socket_quad(context, ZMQ_REQ);
  zmq::socket_t socket_obj(context, ZMQ_REQ);
  socket_quad.bind("tcp://*:2222");
  socket_obj.bind("tcp://*:4444");

  std::ofstream output;
  std::time_t timestamp =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::string date = std::ctime(&timestamp);

  output.open("logs/" + date + ".csv");
  output << "vision_x"
         << ","
         << "vision_y"
         << ","
         << "vision_z"
         << ","
         << "mocap_x"
         << ","
         << "mocap_y"
         << ","
         << "mocap_z"
         << ","
         << "quad_x"
         << ","
         << "quad_y"
         << ","
         << "quad_z"
         << ","
         << "quad_roll"
         <<  ","
         << "quad_pitch"
         << ","
         << "quad_yaw"
         << ","
         << "error_x"
         << ","
         << "error_y"
         << ","
         << "error_z"
         << ","
         << "trans_x"
         << ","
         << "trans_y"
         << ","
         << "trans_z"
         << ","
         << "t"
         << "\n";

  cpp_msg::Mocap_msg last_known_position;
  last_known_position.position.x = 0.0;
  last_known_position.position.y = 0.0;
  last_known_position.position.z = 0.0;
  last_known_position.occluded = 0;
  
  while (true) {
    std::cout << frame_id << std::endl;
    frame_id++;
    //  Send initial message that detection can move on
    // Save position of the drone - this should be synced up with the detection
    // data from the camera
    std::vector<float> quad_position;
    std::vector<float> quad_orientation;
    cpp_msg::Mocap_msg quad_pose = quad.getPose();

    cpp_msg::Mocap_msg item_pose = box.getPose();
    std::vector<float> item_position;

    // std::cout << "got quad pose from mocap" << std::endl;

    quad_position.push_back(quad_pose.position.x);
    quad_position.push_back(quad_pose.position.y);
    quad_position.push_back(quad_pose.position.z);

//     std::cout << "Quad position:" << std::endl;
//     std::cout << quad_position.at(0) << std::endl;
//     std::cout << quad_position.at(1) << std::endl;
//     std::cout << quad_position.at(2) << std::endl;

//     std::cout << "Box position:" << std::endl;
//     std::cout << box.getPose().position.x << std::endl;
//     std::cout << box.getPose().position.y << std::endl;
//     std::cout << box.getPose().position.z << std::endl;

    quad_orientation.push_back(quad_pose.orientation.roll * M_PI / 180.0f);
    quad_orientation.push_back(quad_pose.orientation.pitch * M_PI / 180.0f);
    quad_orientation.push_back(quad_pose.orientation.yaw * M_PI / 180.0f);


    item_position.push_back(item_pose.position.x);
    item_position.push_back(item_pose.position.y);
    item_position.push_back(item_pose.position.z);

//     std::cout << "Quad orientation:" << std::endl;
//     std::cout << quad_orientation.at(0) << std::endl;
//     std::cout << quad_orientation.at(1) << std::endl;
//     std::cout << quad_orientation.at(2) << std::endl;

    std::cout << "Translation difference:" << std::endl;
    float trans_x = quad_position.at(0) - item_position.at(0);
    float trans_y = quad_position.at(1) - item_position.at(1);
    float trans_z = quad_position.at(2) - item_position.at(2);
    std::cout << trans_x << std::endl;
    std::cout << trans_y << std::endl;
    std::cout << trans_z << std::endl;
    std::cout << quad_pose.orientation.yaw << std::endl;

    Vision::Detection quad_pose_detection;
    quad_pose_detection.set_x(quad_position.at(0));
    quad_pose_detection.set_y(quad_position.at(1));
    quad_pose_detection.set_z(quad_position.at(2));
    quad_pose_detection.set_roll(quad_orientation.at(0));
    quad_pose_detection.set_pitch(quad_orientation.at(1));
    quad_pose_detection.set_yaw(quad_orientation.at(2));

    Vision::Detection item_pose_detection;
    item_pose_detection.set_x(item_position.at(0));
    item_pose_detection.set_y(item_position.at(1));
    item_pose_detection.set_z(item_position.at(2));

    std::string quad_msg_string;
    quad_pose_detection.SerializeToString(&quad_msg_string);
    zmq::message_t quad_request(quad_msg_string.size());
    memcpy((void *)quad_request.data(), quad_msg_string.c_str(), quad_msg_string.size());
    socket_quad.send(quad_request, zmq::send_flags::none);

    // Receive answer with detection data
    // auto res = socket.recv(request, zmq::recv_flags::none);

    std::string item_msg_string;
    item_pose_detection.SerializeToString(&item_msg_string);
    zmq::message_t item_request(item_msg_string.size());
    memcpy((void *)item_request.data(), item_msg_string.c_str(), item_msg_string.size());
    socket_obj.send(item_request, zmq::send_flags::none);

    auto quad_res = socket_quad.recv(quad_request, zmq::recv_flags::none);
    auto item_res = socket_obj.recv(item_request, zmq::recv_flags::none);

    // Receive answer with detection data
    // auto res = socket.recv(request, zmq::recv_flags::none);

    // Deserialize protobuf message
    Vision::Detection quad_det;
    if (!quad_det.ParseFromString(quad_request.to_string())) {
      std::cerr << "Failed to parse protobuf message." << std::endl;
      return -1;
    }

    // Deserialize protobuf message
    Vision::Detection item_det;
    if (!item_det.ParseFromString(item_request.to_string())) {
      std::cerr << "Failed to parse protobuf message." << std::endl;
      return -1;
    }

    // std::cout << det.DebugString() << std::endl;

    // Frame conversions - we need to go from camera frame to drone frame and
    // then from drone frame to global frame

    // Point in camera frame - this is what we're getting back from the camera
   

    if(quad_det.label() == "closing") {
      output.close();
      std::cout << "closing process" << std::endl;
      break;
    }
    
    std::vector<float> point_global{quad_det.x(), quad_det.y(), quad_det.z()};

    float errx = item_position.at(0) - point_global.at(0);
    float erry = item_position.at(1) - point_global.at(1);
    float errz = item_position.at(2) - point_global.at(2);
    std::time_t ms = std::time(nullptr);
    
    if(quad_det.label() != "Nothing") {


      std::cout << "ERROR ------------" << std::endl;
      std::cout << errx << std::endl;
      std::cout << erry << std::endl;
      std::cout << errz << std::endl;
    }

    output << point_global.at(0) << "," << point_global.at(1) << ","
           << point_global.at(2) << ",";
    output << item_position.at(0) << "," << item_position.at(1) << ","
           << item_position.at(2) << ",";
    output << quad_position.at(0) << "," << quad_position.at(1) << ","
           << quad_position.at(2) << ",";
    output << quad_orientation.at(0) << "," << quad_orientation.at(1) << ","
           << quad_orientation.at(2) << ",";
    output << errx << "," << erry << "," << errz << ",";
    output << trans_x << "," << trans_y << "," << trans_z << ",";

    output << ms << "\n";


    cpp_msg::Mocap_msg mocap;
    mocap.position.x = point_global.at(0);
    mocap.position.y = point_global.at(1);
    mocap.position.z = point_global.at(2);
    mocap.occluded = 0;

    if(quad_det.label() == "Nothing") {
      std::cout << "received nothing, publishing position" << std::endl;
      std::cout << last_known_position.position.x << "\t" << last_known_position.position.y << "\t" << last_known_position.position.z << "\t" << std::endl;;
      // pub.publish(last_known_position);
      continue;
    }
    
    std::cout << "publishing position" << std::endl;
    std::cout << mocap.position.x << "\t" << mocap.position.y << "\t" << mocap.position.z << "\t" << std::endl;;
    // pub.publish(mocap);
    last_known_position = mocap;
  }
  output.close();
  return 0;
}
