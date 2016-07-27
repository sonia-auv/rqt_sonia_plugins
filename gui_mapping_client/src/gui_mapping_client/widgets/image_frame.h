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

#include <QFrame>
#include <QImage>
#include <QLayout>
#include <QLayoutItem>
#include <QMutex>
#include <QPainter>
#include <QRect>
#include <QSize>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <memory>
#include <mutex>

namespace gui_mapping_client {

class ImageFrame : public QFrame {
  Q_OBJECT

 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ImageFrame>;
  using ConstPtr = std::shared_ptr<const ImageFrame>;
  using PtrList = std::vector<ImageFrame::Ptr>;
  using ConstPtrList = std::vector<ImageFrame::ConstPtr>;

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit ImageFrame(QWidget *const parent = nullptr,
                      Qt::WFlags flags = nullptr);

  virtual ~ImageFrame() = default;

 public slots:
  //==========================================================================
  // P U B L I C   S L O T S

  void ChangeImage(const cv::Mat &image);
  void ChangeImage(QImage *image);

 signals:
  //==========================================================================
  // P U B L I C   S I G N A L S

  void delayed_update();

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  void paintEvent(QPaintEvent *event) override;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  QImage image_;
  mutable std::mutex image_mutex_;
};

}  // namespace gui_mapping_client
