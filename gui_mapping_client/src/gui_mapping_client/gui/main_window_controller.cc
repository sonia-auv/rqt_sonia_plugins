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

  ResetUI();
}

//------------------------------------------------------------------------------
//
MainWindowController::~MainWindowController() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void MainWindowController::ResetUI() {
  ResetProcUnitComboBox();
  ResetProcTreeComboBox();
}

//------------------------------------------------------------------------------
//
void MainWindowController::ResetProcTreeComboBox() {
  ui_->proc_tree_combobox->clear();
  ui_->proc_tree_combobox->addItem("--");
  auto proc_tree_list = communication_.GetProcTreeList();

  for (auto &&proc_tree : proc_tree_list) {
    ui_->proc_tree_combobox->addItem(proc_tree.name.c_str());
  }

  ui_->proc_unit_combobox->setCurrentIndex(0);
}

//------------------------------------------------------------------------------
//
void MainWindowController::ResetProcUnitComboBox() {
  ui_->proc_unit_combobox->clear();
  ui_->proc_unit_combobox->addItem("--");
  ui_->proc_unit_combobox->addItem("Raw Map");

  auto proc_tree = communication_.GetCurrentProcTree();

  if (proc_tree) {
    for (auto &&pu : communication_.GetCurrentProcTree()->proc_unit_list) {
      ui_->proc_unit_combobox->addItem(pu.name.c_str());
    }
  }

  ui_->proc_unit_combobox->addItem("Semantic Map");

  ui_->proc_unit_combobox->setCurrentIndex(0);
}

//------------------------------------------------------------------------------
//
void gui_mapping_client::MainWindowController::ResetImageView() {
  ui_->video_frame_left->GetImageFrame()->ChangeImage(nullptr);
}

//------------------------------------------------------------------------------
//
void MainWindowController::LoadProcTreeParameters() {}

//------------------------------------------------------------------------------
//
void MainWindowController::ChangeCurrentProcTree(
    const sonia_msgs::ProcTree &proc_tree) {
  auto proc_unit_names = GetProcUnitNames(proc_tree);

  ResetProcUnitList(proc_unit_names);
  SelectProcUnit("Semantic Map");
  ResetImageView();

  current_proc_tree_ = std::make_shared<sonia_msgs::ProcTree>(proc_tree);
}

//------------------------------------------------------------------------------
//
void gui_mapping_client::MainWindowController::SelectProcUnit(
    const std::string &string) {}

//------------------------------------------------------------------------------
//
void gui_mapping_client::MainWindowController::ResetProcUnitList(
    std::vector<std::string> vector) {}

//------------------------------------------------------------------------------
//
void MainWindowController::ChangeProcUnit(
    const sonia_msgs::ProcUnit &proc_unit) {}

//------------------------------------------------------------------------------
//
void MainWindowController::AddParameterToWidget(
    const sonia_msgs::ProcUnitParameter &parameter) {}

//------------------------------------------------------------------------------
//
void MainWindowController::ChangeFrameImage(const cv::Mat &image) {
  ui_->video_frame_left->GetImageFrame()->ChangeImage(image);
}

}  // namespace gui_mapping_client
