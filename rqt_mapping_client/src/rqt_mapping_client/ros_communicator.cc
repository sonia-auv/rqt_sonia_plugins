/**
 * \file	ros_communicator.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	25/07/2016
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

#include "rqt_mapping_client/ros_communicator.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sonia_msgs/GetProcTreeList.h>

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
RosCommunicator::RosCommunicator(const ros::NodeHandle &nh)
    : nh_(nh), subscriber_() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
QSet<QString> RosCommunicator::GetImageTopicList(
    const QSet<QString> &message_types, const QSet<QString> &message_sub_types,
    const QList<QString> &transports) {
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics_name;
  for (const auto &topic : topic_info) {
    all_topics_name.insert(topic.name.c_str());
  }

  QSet<QString> topic_names;
  for (const auto &topic : topic_info) {
    if (message_types.contains(topic.datatype.c_str())) {
      QString topic_name = topic.name.c_str();

      // add raw topic
      topic_names.insert(topic_name);

      // add transport specific sub-topics
      for (const auto &transport : transports) {
        if (all_topics_name.contains(topic_name + "/" + transport)) {
          QString sub = topic_name + " " + transport;
          topic_names.insert(sub);
        }
      }
    }

    if (message_sub_types.contains(topic.datatype.c_str())) {
      QString topic_name = topic.name.c_str();
      int index = topic_name.lastIndexOf("/");
      if (index != -1) {
        topic_name.replace(index, 1, " ");
        topic_names.insert(topic_name);
      }
    }
  }
  return topic_names;
}

//------------------------------------------------------------------------------
//
void RosCommunicator::CallbackImage(const sensor_msgs::Image::ConstPtr &msg) {
  try {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr =
        cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;
  } catch (cv_bridge::Exception &e) {
    try {
      // If we're here, there is no conversion that makes sense, but let's try
      // to imagine a few first
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      if (msg->encoding == "CV_8UC3") {
        // assuming it is rgb
        conversion_mat_ = cv_ptr->image;
      } else if (msg->encoding == "8UC1") {
        // convert gray to rgb
        cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
      } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
        cv::Mat img_scaled_8u;
        cv::Mat(cv_ptr->image).convertTo(img_scaled_8u, CV_8UC1, 255.);
        cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
      } else {
        qWarning(
            "RosCommunicator.callback_image() could not convert image from "
            "'%s' to "
            "'rgb8' (%s)",
            msg->encoding.c_str(), e.what());
        emit NewImageReady(QImage());
        return;
      }
    } catch (cv_bridge::Exception &e) {
      qWarning(
          "RosCommunicator.callback_image() while trying to convert image from "
          "'%s' "
          "to 'rgb8' an exception was thrown (%s)",
          msg->encoding.c_str(), e.what());
      emit NewImageReady(QImage());
      return;
    }
  }

  // image must be copied since it uses the conversion_mat_ for storage which is
  // asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows,
               conversion_mat_.step[0], QImage::Format_RGB888);
  emit NewImageReady(image);
}

//------------------------------------------------------------------------------
//
void RosCommunicator::ChangeImageTopic(const std::string &topic_name,
                                       const std::string &transport) {
  image_transport::ImageTransport it(nh_);
  image_transport::TransportHints hints(transport);

  subscriber_ =
      it.subscribe(topic_name, 1, &RosCommunicator::CallbackImage, this, hints);
}

//------------------------------------------------------------------------------
//
void RosCommunicator::ShutdownSubscriber() { subscriber_.shutdown(); }

//------------------------------------------------------------------------------
//
std::vector<sonia_msgs::ProcTree> RosCommunicator::GetProcTreeList() {
  ros::ServiceClient client = nh_.serviceClient<sonia_msgs::GetProcTreeList>(
      "/proc_mapping/get_proc_tree_list");
  sonia_msgs::GetProcTreeList srv;
  if (client.call(srv)) {
    return srv.response.proc_tree_list;
  } else {
    ROS_ERROR("Failed to call service add_two_ints");
    return {};
  }
}

//------------------------------------------------------------------------------
//
void RosCommunicator::ChangeProcTree(const std::string &proc_tree_name) const {}

//------------------------------------------------------------------------------
//
void RosCommunicator::SetParameterValue(
    const std::string &proc_tree_name, const std::string &proc_unit_name,
    const sonia_msgs::ProcUnitParameter &parameter) const {}
