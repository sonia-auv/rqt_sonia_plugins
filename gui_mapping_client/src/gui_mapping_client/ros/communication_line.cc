/**
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	26/07/2016
 *
 * \copyright Copyright (c) 2016 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include "gui_mapping_client/ros/communication_line.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sonia_msgs/ChangeParameter.h>
#include <sonia_msgs/ChangeProcTree.h>
#include <sonia_msgs/GetCurrentProcTree.h>
#include <sonia_msgs/GetProcTreeList.h>
#include <sonia_msgs/ResetMap.h>

namespace gui_mapping_client {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
CommunicationLine::CommunicationLine(const ros::NodeHandle &nh,
                                     QObject *const parent)
    : QObject(parent),
      nh_(nh),
      image_transport_(nh),
      rosout_sub_(nh_.subscribe("/rosout", 100,
                                &CommunicationLine::RosOutCallback, this)),
      reset_map_pub_(),
      image_sub_(),
      image_() {
  reset_map_pub_ =
      nh_.advertise<sonia_msgs::ResetMap>("/proc_mapping/reset_map", 100);
}

//------------------------------------------------------------------------------
//
CommunicationLine::~CommunicationLine() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void CommunicationLine::ShutdownImageSubscriber() { image_sub_.shutdown(); }

//------------------------------------------------------------------------------
//
void CommunicationLine::ChangeImageSubscriberTopic(const std::string &topic) {
  ShutdownImageSubscriber();
  image_transport::TransportHints hints("compressed", ros::TransportHints());
  std::string topic_name = topic;
  image_sub_ = image_transport_.subscribe(
      topic_name, 1, &CommunicationLine::ImageCallback, this, hints);
}

//------------------------------------------------------------------------------
//
void CommunicationLine::RosOutCallback(
    const rosgraph_msgs::Log::ConstPtr &msg) {
  if (msg->name == "/proc_mapping") {
    std::string log{"["};

    log += std::to_string(msg->header.stamp.sec) + "." +
           std::to_string(msg->header.stamp.nsec);
    log += "] ";

    if (msg->level == msg->DEBUG) {
      log += "[DEBUG] ";
    } else if (msg->level == msg->INFO) {
      log += "[INFO] ";
    } else if (msg->level == msg->WARN) {
      log += "[WARN] ";
    } else if (msg->level == msg->ERROR) {
      log += "[ERROR] ";
    } else if (msg->level == msg->FATAL) {
      log += "[FATAL] ";
    }

    log += msg->msg;

    QString colored_log;
    if (msg->level == msg->DEBUG) {
      colored_log = QString(
          "<span style=\" color:#ffcc66; font-weight:bold;\">%1</span>");
    } else if (msg->level == msg->INFO) {
      colored_log = QString(
          "<span style=\" color:#3399ff; font-weight:bold;\">%1</span>");
    } else if (msg->level == msg->WARN) {
      colored_log = QString(
          "<span style=\" color:#ffcc66; font-weight:bold;\">%1</span>");
    } else if (msg->level == msg->ERROR) {
      colored_log = QString(
          "<span style=\" color:#ff0000; font-weight:bold;\">%1</span>");
    } else if (msg->level == msg->FATAL) {
      colored_log = QString(
          "<span style=\" color:#ff0000; font-weight:bold;\">%1</span>");
    }

    colored_log = colored_log.arg(log.c_str());
    emit ReceivedLogMessage(colored_log);
  }
}

//------------------------------------------------------------------------------
//
void CommunicationLine::ImageCallback(const sensor_msgs::Image::ConstPtr &msg) {
  try {
    cv_bridge::CvImageConstPtr ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    image_ = ptr->image;
    emit ReceivedNewImage(image_);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR(
        "Cannot convert the sensor_msgs::Image message to cv::Mat image"
        ".");
  }
}

//------------------------------------------------------------------------------
//
std::vector<sonia_msgs::ProcTree> CommunicationLine::GetProcTreeList() {
  ros::ServiceClient client = nh_.serviceClient<sonia_msgs::GetProcTreeList>(
      "/proc_mapping/get_proc_tree_list");
  sonia_msgs::GetProcTreeList srv;
  if (client.call(srv)) {
    return srv.response.proc_tree_list;
  } else {
    ROS_ERROR("Failed to call service /proc_mapping/get_proc_tree_list");
    return {};
  }
}

//------------------------------------------------------------------------------
//
std::shared_ptr<sonia_msgs::ProcTree> CommunicationLine::GetCurrentProcTree() {
  ros::ServiceClient client = nh_.serviceClient<sonia_msgs::GetCurrentProcTree>(
      "/proc_mapping/get_current_proc_tree");
  sonia_msgs::GetCurrentProcTree srv;
  if (client.call(srv)) {
    return std::make_shared<sonia_msgs::ProcTree>(
        srv.response.current_proc_tree);
  } else {
    ROS_ERROR("Failed to call service /proc_mapping/get_current_proc_tree");
    return nullptr;
  }
}

//------------------------------------------------------------------------------
//
void CommunicationLine::ChangeCurrentProcTree(unsigned char proc_tree) {
  ros::ServiceClient client = nh_.serviceClient<sonia_msgs::ChangeProcTree>(
      "/proc_mapping/change_proc_tree");
  sonia_msgs::ChangeProcTree srv;
  srv.request.target = proc_tree;
  if (!client.call(srv)) {
    ROS_ERROR("Failed to call service /proc_mapping/change_proc_tree");
  }
}

//------------------------------------------------------------------------------
//
void CommunicationLine::SetProcUnitParameterValue(
    const std::string &proc_tree, const std::string &proc_unit,
    const sonia_msgs::ProcUnitParameter &parameter) {
  ros::ServiceClient client = nh_.serviceClient<sonia_msgs::ChangeParameter>(
      "/proc_mapping/change_parameter");
  sonia_msgs::ChangeParameter srv;
  srv.request.proc_tree_name = proc_tree;
  srv.request.proc_unit_name = proc_unit;
  srv.request.proc_unit_parameter = parameter;
  if (!client.call(srv)) {
    ROS_ERROR("Failed to call service /proc_mapping/change_proc_tree");
  }
}

//------------------------------------------------------------------------------
//
void CommunicationLine::SendResetMapMessage() {
  sonia_msgs::ResetMap msg;
  reset_map_pub_.publish(msg);
}

}  // namespace gui_mapping_client