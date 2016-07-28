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

#ifndef GUI_MAPPING_CLIENT_ROS_COMMUNICATION_LINE_H_
#define GUI_MAPPING_CLIENT_ROS_COMMUNICATION_LINE_H_

#include <QMap>
#include <QObject>
#include <QString>
#include <QStringList>
#include <QVariant>
#include <QVector>
#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <sensor_msgs/Image.h>
#include <sonia_msgs/ProcTree.h>
#include "rosgraph_msgs/Log.h"

namespace gui_mapping_client {

class CommunicationLine : public QObject {
  Q_OBJECT

 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CommunicationLine>;
  using ConstPtr = std::shared_ptr<const CommunicationLine>;
  using PtrList = std::vector<CommunicationLine::Ptr>;
  using ConstPtrList = std::vector<CommunicationLine::ConstPtr>;

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit CommunicationLine(const ros::NodeHandle &nh,
                             QObject *const parent = nullptr);

  virtual ~CommunicationLine();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void ChangeImageSubscriberTopic(const std::string &topic);
  void ImageCallback(const sensor_msgs::Image::ConstPtr &msg);
  void RosOutCallback(const rosgraph_msgs::Log::ConstPtr &msg);
  void ShutdownImageSubscriber();

  std::vector<sonia_msgs::ProcTree> GetProcTreeList();
  std::shared_ptr<sonia_msgs::ProcTree> GetCurrentProcTree();

  void ChangeCurrentProcTree(unsigned char proc_tree_name);

  void SetProcUnitParameterValue(
      const std::string &proc_tree, const std::string &proc_unit,
      const sonia_msgs::ProcUnitParameter &parameter);

 signals:
  //==========================================================================
  // P U B L I C   S I G N A L S

  void ReceivedNewImage(const cv::Mat &) const;
  void ReceivedLogMessage(const QString &) const;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandle nh_;

  image_transport::ImageTransport image_transport_;

  ros::Subscriber rosout_sub_;

  image_transport::Subscriber image_sub_;

  cv::Mat image_;
};

#endif  // GUI_MAPPING_CLIENT_ROS_COMMUNICATION_LINE_H_

}  // namespace gui_mapping_client
