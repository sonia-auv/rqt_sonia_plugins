/**
 * \file	mapping_client.cc
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

#include <rqt_mapping_client/mapping_client.h>
#include <rqt_mapping_client/ratio_layouted_frame.h>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QSet>
#include <QtGui/QRegExpValidator>

namespace rqt_mapping_client {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MappingClient::MappingClient()
    : rqt_gui_cpp::Plugin(), widget_(0), ros_com_(nullptr) {
  sleep(10);
  setObjectName("MappingClient");
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void MappingClient::initPlugin(qt_gui_cpp::PluginContext& context) {
  ROS_WARN("Init");
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() + " (" +
                            QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  UpdateTopicList();
  ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(""));
  connect(ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this,
          SLOT(OnTopicChanged(int)));

  connect(ui_.refresh_topics_push_button, SIGNAL(pressed()), this,
          SLOT(UpdateTopicList()));

  connect(ui_.save_as_image_push_button, SIGNAL(pressed()), this,
          SLOT(SaveImage()));

  // set topic name if passed in as argument
  const QStringList& argv = context.argv();
  if (!argv.empty()) {
    arg_topic_name_ = argv[0];
    // add topic name to list if not yet in
    int index = ui_.topics_combo_box->findText(arg_topic_name_);
    if (index == -1) {
      QString label(arg_topic_name_);
      label.replace(" ", "/");
      ui_.topics_combo_box->addItem(label, QVariant(arg_topic_name_));
      ui_.topics_combo_box->setCurrentIndex(
          ui_.topics_combo_box->findText(arg_topic_name_));
    }
  }

  ros_com_.reset(new RosCommunicator{getNodeHandle()});
}

//------------------------------------------------------------------------------
//
void MappingClient::shutdownPlugin() { ros_com_->ShutdownSubscriber(); }

//------------------------------------------------------------------------------
//
void MappingClient::saveSettings(
    qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const {
  QString topic = ui_.topics_combo_box->currentText();
  instance_settings.setValue("topic", topic);
}

//------------------------------------------------------------------------------
//
void MappingClient::restoreSettings(
    const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings) {
  QString topic = instance_settings.value("topic", "").toString();
  // don't overwrite topic name passed as command line argument
  if (!arg_topic_name_.isEmpty()) {
    arg_topic_name_ = "";
  } else {
    SelectTopic(topic);
  }
}

//------------------------------------------------------------------------------
//
void MappingClient::SelectTopic(const QString& topic) {
  int index = ui_.topics_combo_box->findText(topic);
  if (index == -1) {
    index = ui_.topics_combo_box->findText("");
  }
  ui_.topics_combo_box->setCurrentIndex(index);
}

//------------------------------------------------------------------------------
//
void MappingClient::UpdateTopicList() {
  ROS_WARN("UppdateList");
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");
  QSet<QString> message_sub_types;
  message_sub_types.insert("sensor_msgs/CompressedImage");

  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it(getNodeHandle());
  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin();
       it != declared.end(); it++) {
    // qDebug("RosCommunicator::updateTopicList() declared transport '%s'",
    // it->c_str());
    QString transport = it->c_str();

    // strip prefix from transport name
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix)) {
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  QString selected = ui_.topics_combo_box->currentText();

  // fill combo box
  QList<QString> topics =
      ros_com_->GetImageTopicList(message_types, message_sub_types, transports)
          .values();
  topics.append("");
  qSort(topics);
  ui_.topics_combo_box->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end();
       it++) {
    QString label(*it);
    label.replace(" ", "/");
    ui_.topics_combo_box->addItem(label, QVariant(*it));
  }

  // restore previous selection
  SelectTopic(selected);
}

//------------------------------------------------------------------------------
//
void MappingClient::OnTopicChanged(int index) {
  ROS_WARN("TopicChnage");
  ros_com_->ShutdownSubscriber();

  ui_.image_frame->SetImage(QImage());

  QStringList parts =
      ui_.topics_combo_box->itemData(index).toString().split(" ");
  QString topic = parts.first();

  QString transport = parts.length() == 2 ? parts.last() : "raw";

  if (!topic.isEmpty()) {
    try {
      ros_com_->ChangeImageTopic(topic.toStdString(), transport.toStdString());
    } catch (image_transport::TransportLoadException& e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed"),
                           e.what());
    }
  }
}

//------------------------------------------------------------------------------
//
void MappingClient::SaveImage() {
  // take a snapshot before asking for the filename
  QImage img = ui_.image_frame->GetImageCopy();

  QString file_name =
      QFileDialog::getSaveFileName(widget_, tr("Save as image"), "image.png",
                                   tr("Image (*.bmp *.jpg *.png *.tiff)"));
  if (file_name.isEmpty()) {
    return;
  }

  img.save(file_name);
}

}  // namespace rqt_mapping_client

PLUGINLIB_EXPORT_CLASS(rqt_mapping_client::MappingClient, rqt_gui_cpp::Plugin)
