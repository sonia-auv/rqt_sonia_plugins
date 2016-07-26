/**
 * \file	mapping_client.h
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

#ifndef rqt_mapping_client__MappingClient_H
#define rqt_mapping_client__MappingClient_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_main_window.h>

#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <ros/macros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>

#include <QList>
#include <QSize>
#include <QString>
#include <QWidget>

#include <vector>
#include "ros_communicator.h"

namespace rqt_mapping_client {

class MappingClient : public rqt_gui_cpp::Plugin {
  Q_OBJECT

 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MappingClient>;
  using ConstPtr = std::shared_ptr<const MappingClient>;
  using PtrList = std::vector<MappingClient::Ptr>;
  using ConstPtrList = std::vector<MappingClient::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  MappingClient();
  ~MappingClient() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void initPlugin(qt_gui_cpp::PluginContext& context) override;

  virtual void shutdownPlugin() override;

  virtual void saveSettings(
      qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const override;

  virtual void restoreSettings(
      const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings) override;

 private slots:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  virtual void UpdateTopicList();

  virtual void OnTopicChanged(int index);

  virtual void SaveImage();

 private:
  virtual void SelectTopic(const QString& topic);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  Ui::ImageViewWidget ui_;
  QWidget* widget_;

  QString arg_topic_name_;

  std::unique_ptr<RosCommunicator> ros_com_;
};

} // namespace rq_mapping_client

#endif  // rqt_mapping_client__MappingClient_H
