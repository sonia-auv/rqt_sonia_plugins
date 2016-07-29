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

#include "gui_mapping_client/gui/main_window_controller.h"

#include <sonia_msgs/ChangeProcTree.h>
#include <QInputDialog>

namespace gui_mapping_client {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MainWindowController::MainWindowController(const ros::NodeHandle &nh,
                                           QWidget *const parent)
    : QMainWindow(parent),
      nh_(nh),
      ui_(std::make_shared<Ui::MainWindow>()),
      communication_(nh) {
  ui_->setupUi(this);

  qRegisterMetaType<cv::Mat>("cv::Mat");

  // When the server send a new image, call method handleImageChange
  connect(&communication_, SIGNAL(ReceivedNewImage(const cv::Mat &)), this,
          SLOT(ChangeFrameImage(const cv::Mat &)));
  connect(&communication_, SIGNAL(ReceivedLogMessage(const QString &)), this,
          SLOT(AddMessageToLogWindow(const QString &)));

  SetButtonsIcons();

  ResetUI();
}

//------------------------------------------------------------------------------
//
MainWindowController::~MainWindowController() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void MainWindowController::SetButtonsIcons() {
  QIcon icon(":/icons/process-stop.png");
  ui_->reset_map_button->setIcon(icon);
  connect(ui_->reset_map_button, SIGNAL(released()), this,
          SLOT(SendResetMapMessage()));

  icon = QIcon(":/icons/view-refresh.png");
  ui_->refresh_button->setIcon(icon);
  connect(ui_->refresh_button, SIGNAL(released()), this, SLOT(ResetUI()));
}

//------------------------------------------------------------------------------
//
void MainWindowController::ResetUI() {
  current_proc_tree_ = communication_.GetCurrentProcTree();
  ResetProcTreeComboBox();
  ResetProcUnitComboBox();
  ui_->log_widget->setPlainText("");
}

//------------------------------------------------------------------------------
//
void MainWindowController::ResetProcTreeComboBox() {
  disconnect(ui_->proc_tree_combobox,
             SIGNAL(currentIndexChanged(const QString &)), this,
             SLOT(ChangeCurrentProcTree(const QString &)));

  ui_->proc_tree_combobox->clear();
  auto proc_tree_list = communication_.GetProcTreeList();

  for (auto &&proc_tree : proc_tree_list) {
    ui_->proc_tree_combobox->addItem(proc_tree.name.c_str());
    if (current_proc_tree_ && proc_tree.name == current_proc_tree_->name) {
      ui_->proc_tree_combobox->setCurrentIndex(
          ui_->proc_tree_combobox->count() - 1);
    }
  }

  if (!current_proc_tree_) {
    ui_->proc_tree_combobox->setCurrentIndex(-1);
  }

  connect(ui_->proc_tree_combobox, SIGNAL(currentIndexChanged(const QString &)),
          this, SLOT(ChangeCurrentProcTree(const QString &)));

  ResetParameterGroupWidget();
}

//------------------------------------------------------------------------------
//
void MainWindowController::ResetProcUnitComboBox() {
  disconnect(ui_->proc_unit_combobox,
             SIGNAL(currentIndexChanged(const QString &)), this,
             SLOT(ChangeCurrentProcUnit(const QString &)));
  ui_->proc_unit_combobox->clear();
  ui_->proc_unit_combobox->addItem("Raw Map");

  if (current_proc_tree_) {
    for (auto &&pu : current_proc_tree_->proc_unit_list) {
      ui_->proc_unit_combobox->addItem(pu.name.c_str());
    }
  }

  ui_->proc_unit_combobox->addItem("Semantic Map");

  ui_->proc_unit_combobox->setCurrentIndex(-1);

  connect(ui_->proc_unit_combobox, SIGNAL(currentIndexChanged(const QString &)),
          this, SLOT(ChangeCurrentProcUnit(const QString &)));
}

//------------------------------------------------------------------------------
//
void gui_mapping_client::MainWindowController::ResetImageView() {
  disconnect(&communication_, SIGNAL(ReceivedNewImage(const cv::Mat &)),
             ui_->video_frame_left->GetImageFrame(),
             SLOT(ChangeImage(const cv::Mat &)));

  ui_->video_frame_left->GetImageFrame()->ChangeImage(nullptr);

  connect(&communication_, SIGNAL(ReceivedNewImage(const cv::Mat &)),
          ui_->video_frame_left->GetImageFrame(),
          SLOT(ChangeImage(const cv::Mat &)));
}

//------------------------------------------------------------------------------
//
void MainWindowController::ResetParameterGroupWidget() {
  parameters_.clear();
  for (auto &&tree_label_list_ : proc_tree_label_list_) {
    delete tree_label_list_;
  }
  proc_tree_label_list_.clear();

  if (current_proc_tree_) {
    for (auto &&proc_unit : current_proc_tree_->proc_unit_list) {
      if (proc_unit.proc_unit_parameter_list.size()) {
        auto proc_unit_name_upper = proc_unit.name;

        std::transform(proc_unit_name_upper.begin(), proc_unit_name_upper.end(),
                       proc_unit_name_upper.begin(), ::toupper);

        auto proc_unit_label = new QLabel(QString(proc_unit_name_upper.c_str()),
                                          ui_->parameter_container_widget);
        proc_unit_label->setStyleSheet("font: bold 1.5em");
        ui_->parameter_container_widget->layout()->addWidget(proc_unit_label);
        proc_tree_label_list_.push_back(proc_unit_label);
      }

      for (auto &&param : proc_unit.proc_unit_parameter_list) {
        std::shared_ptr<Parameter> param_widget{nullptr};
        auto msg_param = std::make_shared<sonia_msgs::ProcUnitParameter>(param);

        if (param.type == sonia_msgs::ProcUnitParameter::TYPE_INT) {
          param_widget = std::make_shared<IntParameter>(
              msg_param, ui_->parameter_container_widget);
        } else if (param.type == sonia_msgs::ProcUnitParameter::TYPE_DOUBLE) {
          param_widget = std::make_shared<DoubleParameter>(
              msg_param, ui_->parameter_container_widget);
        } else if (param.type == sonia_msgs::ProcUnitParameter::TYPE_BOOL) {
          param_widget = std::make_shared<BoolParameter>(
              msg_param, ui_->parameter_container_widget);
        }

        connect(param_widget.get(),
                SIGNAL(ParameterChanged(
                    const Parameter *,
                    const std::shared_ptr<sonia_msgs::ProcUnitParameter> &)),
                this,
                SLOT(ChangeServerParameter(
                    const Parameter *,
                    const std::shared_ptr<sonia_msgs::ProcUnitParameter> &)));
        parameters_.insert(std::pair<decltype(param_widget), std::string>(
            param_widget, proc_unit.name));
      }
    }
  }
}

//------------------------------------------------------------------------------
//
void MainWindowController::ChangeCurrentProcTree(const QString &proc_tree) {
  auto pt_name = proc_tree.toStdString();
  if (pt_name == "buoys") {
    communication_.ChangeCurrentProcTree(
        sonia_msgs::ChangeProcTreeRequest::BUOYS);
  } else if (pt_name == "far_buoys") {
    communication_.ChangeCurrentProcTree(
        sonia_msgs::ChangeProcTreeRequest::FAR_BUOYS);
  } else if (pt_name == "fence") {
    communication_.ChangeCurrentProcTree(
        sonia_msgs::ChangeProcTreeRequest::FENCE);
  } else if (pt_name == "wall") {
    communication_.ChangeCurrentProcTree(
        sonia_msgs::ChangeProcTreeRequest::WALL);
  } else {
    ROS_ERROR(
        "Cannot parse the name of the proc tree, sending UNKNOWN proc "
        "tree to the server");
    communication_.ChangeCurrentProcTree(
        sonia_msgs::ChangeProcTreeRequest::UNKNOWN);
  }

  current_proc_tree_ = communication_.GetCurrentProcTree();

  ResetProcUnitComboBox();
  ResetParameterGroupWidget();
  ResetImageView();
}

//------------------------------------------------------------------------------
//
void MainWindowController::ChangeCurrentProcUnit(const QString &proc_unit) {
  current_proc_unit_ = nullptr;

  if (proc_unit.toStdString() == "Raw Map") {
    auto topic_name = "/proc_mapping/raw_map";
    communication_.ChangeImageSubscriberTopic(topic_name);
  } else if (proc_unit.toStdString() == "Semantic Map") {
    auto topic_name = "/proc_mapping/semantic_map";
    communication_.ChangeImageSubscriberTopic(topic_name);
  } else if (current_proc_tree_) {
    for (auto &&pu : current_proc_tree_->proc_unit_list) {
      if (pu.name == proc_unit.toStdString()) {
        current_proc_unit_ = std::make_shared<sonia_msgs::ProcUnit>(pu);
      }
    }

    if (current_proc_unit_) {
      auto topic_name = "/proc_mapping/" + current_proc_tree_->name + "/" +
                        current_proc_unit_->name;
      communication_.ChangeImageSubscriberTopic(topic_name);
    } else {
      ROS_ERROR("Cannot find a proc_unit with the given name");
      communication_.ShutdownImageSubscriber();
    }
  } else {
    ROS_ERROR("There is no proc tree currently selected.");
    communication_.ShutdownImageSubscriber();
  }

  ResetImageView();
}

//------------------------------------------------------------------------------
//
void MainWindowController::ChangeFrameImage(const cv::Mat &image) {
  ui_->video_frame_left->GetImageFrame()->ChangeImage(image);
}

//------------------------------------------------------------------------------
//
void MainWindowController::ChangeServerParameter(
    const Parameter *param,
    const std::shared_ptr<sonia_msgs::ProcUnitParameter> &msg) {
  auto proc_tree = current_proc_tree_->name;
  std::string proc_unit = "";
  for (auto &&parameter : parameters_) {
    if (parameter.first.get() == param) {
      proc_unit = parameter.second;
    }
  }

  if (proc_unit.compare("") != 0) {
    sonia_msgs::ProcUnitParameter parameter_msg{*msg};
    communication_.SetProcUnitParameterValue(proc_tree, proc_unit,
                                             parameter_msg);
  } else {
    ROS_ERROR("Cannot find a proc unit associated with this parameter");
  }
}

//------------------------------------------------------------------------------
//
void MainWindowController::AddMessageToLogWindow(const QString &log) {
  ui_->log_widget->appendHtml(log);
}

//------------------------------------------------------------------------------
//
void MainWindowController::SendResetMapMessage() {
  communication_.SendResetMapMessage();
}

}  // namespace gui_mapping_client
