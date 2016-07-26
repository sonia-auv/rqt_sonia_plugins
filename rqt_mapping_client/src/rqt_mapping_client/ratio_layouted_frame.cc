/**
 * \file	ratio_layout_frame.cc
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

#include <rqt_mapping_client/ratio_layouted_frame.h>

namespace rqt_mapping_client {

//------------------------------------------------------------------------------
//
RatioLayoutedFrame::RatioLayoutedFrame(QWidget* parent, Qt::WindowFlags flags)
    : QFrame(parent, flags) {
  connect(this, SIGNAL(delayed_update()), this, SLOT(update()),
          Qt::QueuedConnection);
}

//------------------------------------------------------------------------------
//
RatioLayoutedFrame::~RatioLayoutedFrame() {}

//------------------------------------------------------------------------------
//
const QImage& RatioLayoutedFrame::GetImage() const { return qimage_; }

//------------------------------------------------------------------------------
//
QImage RatioLayoutedFrame::GetImageCopy() const {
  QImage img;
  qimage_mutex_.lock();
  img = qimage_.copy();
  qimage_mutex_.unlock();
  return img;
}

//------------------------------------------------------------------------------
//
void RatioLayoutedFrame::SetImage(const QImage& image) {
  qimage_mutex_.lock();
  qimage_ = image.copy();
  qimage_mutex_.unlock();
  emit delayed_update();
}

//------------------------------------------------------------------------------
//
void RatioLayoutedFrame::paintEvent(QPaintEvent* event) {
  QPainter painter(this);
  qimage_mutex_.lock();
  if (!qimage_.isNull()) {
    painter.drawImage(contentsRect(), qimage_);
  } else {
    // default image with gradient
    QLinearGradient gradient(0, 0, frameRect().width(), frameRect().height());
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::black);
    painter.setBrush(gradient);
    painter.drawRect(0, 0, frameRect().width() + 1, frameRect().height() + 1);
  }
  qimage_mutex_.unlock();
}

//------------------------------------------------------------------------------
//
int RatioLayoutedFrame::GreatestCommonDivisor(int a, int b) {
  if (b == 0) {
    return a;
  }
  return GreatestCommonDivisor(b, a % b);
}

}  // namespace rqt_mapping_client
