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

#include <ui_focused_frame.h>
#include <QCheckBox>
#include <QComboBox>
#include <QFileDialog>
#include <QFrame>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPushButton>
#include <QToolButton>
#include <QWidget>
#include <memory>
#include "gui_mapping_client/widgets/image_frame.h"

namespace Ui {
class MainWindow;
class FocusedFrame;
}

namespace gui_mapping_client {

class FocusedFrameController : public QFrame {
  Q_OBJECT

 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<FocusedFrameController>;
  using ConstPtr = std::shared_ptr<const FocusedFrameController>;
  using PtrList = std::vector<FocusedFrameController::Ptr>;
  using ConstPtrList = std::vector<FocusedFrameController::ConstPtr>;

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit FocusedFrameController(QWidget *const parent = nullptr);

  virtual ~FocusedFrameController() = default;

  //==========================================================================
  // G E T T E R S   A N D   S E T T E R S

  inline ImageFrame *const &GetImageFrame() const;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R

  std::shared_ptr<Ui::FocusedFrame> ui_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline ImageFrame *const &FocusedFrameController::GetImageFrame() const {
  return ui_->video_image_frame;
}

}  // namespace gui_mapping_client
