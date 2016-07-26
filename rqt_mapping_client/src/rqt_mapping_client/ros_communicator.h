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

#ifndef RQT_MAPPING_CLIENT_ROS_COMMUNICATOR_H_
#define RQT_MAPPING_CLIENT_ROS_COMMUNICATOR_H_

#include <image_transport/subscriber.h>
#include <opencv/cv.h>
#include <sensor_msgs/Image.h>
#include <sonia_msgs/ProcTree.h>
#include <sonia_msgs/ProcUnitParameter.h>
#include <QtCore/QString>
#include <QtGui/QtGui>
#include <memory>
#include <vector>

/// We want to inherit from a QObject here because we don't want to have
/// several implementation of the Subject-Observer pattern.
/// We are going to use the sig-slot provided by Qt.
class RosCommunicator : public QObject {
  Q_OBJECT

 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<RosCommunicator>;
  using ConstPtr = std::shared_ptr<const RosCommunicator>;
  using PtrList = std::vector<RosCommunicator::Ptr>;
  using ConstPtrList = std::vector<RosCommunicator::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  RosCommunicator(const ros::NodeHandle &nh);
  ~RosCommunicator() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void CallbackImage(const sensor_msgs::Image::ConstPtr &msg);

  virtual QSet<QString> GetImageTopicList(
      const QSet<QString> &message_types,
      const QSet<QString> &message_sub_types, const QList<QString> &transports);

  void ChangeImageTopic(const std::string &topic_name,
                        const std::string &transport);

  std::vector<sonia_msgs::ProcTree> GetProcTreeList();

  void ChangeProcTree(const std::string &proc_tree_name) const;

  void SetParameterValue(const std::string &proc_tree_name,
                         const std::string &proc_unit_name,
                         const sonia_msgs::ProcUnitParameter &parameter) const;

  void ShutdownSubscriber();

 signals:

  void NewImageReady(QImage image);

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandle nh_;
  image_transport::Subscriber subscriber_;
  cv::Mat conversion_mat_;
};

#endif  // RQT_MAPPING_CLIENT_ROS_COMMUNICATOR_H_
