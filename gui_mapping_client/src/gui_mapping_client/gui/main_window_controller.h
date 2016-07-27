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

#pragma once

#include <ros/ros.h>
#include <sonia_msgs/ProcTree.h>
#include <sonia_msgs/ProcUnit.h>
#include <sonia_msgs/ProcUnitParameter.h>
#include <ui_main_window.h>
#include <QLabel>
#include <QListWidgetItem>
#include <QMainWindow>
#include <QPalette>
#include <ctime>
#include "gui_mapping_client/gui/focused_frame_controller.h"
#include "gui_mapping_client/ros/communication_line.h"
#include "gui_mapping_client/widgets/parameter.h"

namespace Ui {
class MainWindow;
}

namespace gui_mapping_client {

class MainWindowController : public QMainWindow {
  Q_OBJECT

 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MainWindowController>;
  using ConstPtr = std::shared_ptr<const MainWindowController>;
  using PtrList = std::vector<MainWindowController::Ptr>;
  using ConstPtrList = std::vector<MainWindowController::ConstPtr>;

  enum class ImageViewType {
    RAW_MAP = 0,
    PROC_UNIT = 1,
    SEMANTIC_MAP = 2,
    BLANK_IMAGE = 3
  };

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit MainWindowController(const ros::NodeHandle &nh,
                                QWidget *const parent = nullptr);
  virtual ~MainWindowController();

 public slots:
  //==========================================================================
  // P U B L I C   S L O T S

  void ChangeCurrentProcTree(const QString &);

  void ChangeCurrentProcUnit(const QString &);

  void ChangeFrameImage(const cv::Mat &);

  /// Return the current image view type.
  /// The image view type can be either the raw map, the semantic map or a
  /// proc unit. If there is non of them, a blank image is displayed.
  const ImageViewType &GetCurrentImageViewType() const;

  void ChangeServerParameter(const Parameter *,
                   const std::shared_ptr<sonia_msgs::ProcUnitParameter> &);

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /// This method will delete every thing in the ParameterContainer widget
  /// and recreate the parameters from the currently selected proc_tree.
  /// If no proc_tree is set (the pointer is nullptr), this method will simply
  /// delete everything and recreate nothing.
  void LoadProcTreeParameters();

  /// This method will extract the name of all the proc_unit from a proc_tree
  /// message
  std::vector<std::string> GetProcUnitNames(
      const sonia_msgs::ProcTree &proc_tree) const;

  /// Getter for the current_proc_tree member.
  /// This will return nullptr whenever the current image view type is not
  std::shared_ptr<sonia_msgs::ProcTree> GetCurrentProcTree() const;

  void ResetUI();
  void ResetProcUnitComboBox();
  void ResetProcTreeComboBox();
  void ResetImageView();
  void ResetParameterGroupWidget();

  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandle nh_;

  std::shared_ptr<Ui::MainWindow> ui_;

  /// The CommunicationLine message will wrap all the ROS messages and
  /// services interface.
  CommunicationLine communication_;

  /// When we change the proc_tree, the server will send us the proc_tree
  /// message corresponding to the name we sent. This will allow us to load
  /// the parameters dynamically instead of statically.
  std::shared_ptr<sonia_msgs::ProcTree> current_proc_tree_;

  /// If the currently image view type is proc_unit, this member stores the
  /// currently selected proc_unit (from the last ROS message that we received).
  std::shared_ptr<sonia_msgs::ProcUnit> current_proc_unit_;

  /// Store the current state of the frame view.
  /// It can be either raw map, semantic map, proc_tree or blank image.
  ImageViewType current_image_view_type_;

  /// The parameters that have been instanciated and displayed.
  /// We want to keep a reference of the proc_unit name with it so we can
  /// send the SetParameter to the communication when we receive a signal
  /// from the parameter.
  std::map<std::shared_ptr<Parameter>, std::string> parameters_;

  /// We keep a reference toall the label that we added to the parameter
  /// container as the Parameter class has no way to delete it by itself.
  std::vector<QLabel *> proc_tree_label_list_;
};

//------------------------------------------------------------------------------
//
inline std::vector<std::string> MainWindowController::GetProcUnitNames(
    const sonia_msgs::ProcTree &proc_tree) const {
  std::vector<std::string> proc_unit_names{};
  for (auto &&item : proc_tree.proc_unit_list) {
    proc_unit_names.push_back(item.name);
  }
  return proc_unit_names;
}

//------------------------------------------------------------------------------
//
inline std::shared_ptr<sonia_msgs::ProcTree>
MainWindowController::GetCurrentProcTree() const {
  return current_proc_tree_;
}

//------------------------------------------------------------------------------
//
inline const MainWindowController::ImageViewType &
MainWindowController::GetCurrentImageViewType() const {
  return current_image_view_type_;
}

}  // namespace gui_mapping_client
