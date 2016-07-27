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

#include "gui_mapping_client/widgets/image_frame.h"

namespace gui_mapping_client {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ImageFrame::ImageFrame(QWidget *const parent, Qt::WFlags flags)
    : QFrame(parent), image_(), image_mutex_() {
  connect(this, SIGNAL(delayed_update()), this, SLOT(update()),
          Qt::QueuedConnection);
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ImageFrame::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  std::lock_guard<std::mutex> guard{image_mutex_};
  if (!image_.isNull()) {
    painter.drawImage(contentsRect(), image_);
  } else {
    // default image with gradient
    QLinearGradient gradient(0, 0, frameRect().width(), frameRect().height());
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::black);
    painter.setBrush(gradient);
    painter.drawRect(0, 0, frameRect().width() + 1, frameRect().height() + 1);
  }
}

//==============================================================================
// Q T   S L O T S   S E C T I O N

//------------------------------------------------------------------------------
//
void ImageFrame::ChangeImage(const cv::Mat &image) {
  auto q_image = new QImage(image.data, image.cols, image.rows, image.step[0],
                            QImage::Format_RGB888);

  std::lock_guard<std::mutex> guard{image_mutex_};
  image_ = q_image->copy();
  emit delayed_update();
}

//------------------------------------------------------------------------------
//
void ImageFrame::ChangeImage(QImage *image) {
  std::lock_guard<std::mutex> guard{image_mutex_};
  if (image != nullptr) {
    image_ = image->copy();
  } else {
    image_ = QImage();
    repaint();
  }
  emit delayed_update();
}

}  // namespace gui_mapping_client
