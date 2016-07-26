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

#ifndef rqt_mapping_client__RatioLayoutedFrame_H
#define rqt_mapping_client__RatioLayoutedFrame_H

#include <QFrame>
#include <QImage>
#include <QLayout>
#include <QLayoutItem>
#include <QMutex>
#include <QPainter>
#include <QRect>
#include <QSize>

namespace rqt_mapping_client {

/// RatioLayoutedFrame is a layout containing a single frame with a fixed aspect
/// ratio.
class RatioLayoutedFrame : public QFrame {
  Q_OBJECT

 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<RatioLayoutedFrame>;
  using ConstPtr = std::shared_ptr<const RatioLayoutedFrame>;
  using PtrList = std::vector<RatioLayoutedFrame::Ptr>;
  using ConstPtrList = std::vector<RatioLayoutedFrame::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit RatioLayoutedFrame(QWidget* parent, Qt::WindowFlags flags = 0);

  virtual ~RatioLayoutedFrame();

  //==========================================================================
  // P U B L I C   M E T H O D S

  const QImage& GetImage() const;

  QImage GetImageCopy() const;

  void SetImage(const QImage& image);

 signals:

  void delayed_update();

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  virtual void paintEvent(QPaintEvent* event) override;

  static int GreatestCommonDivisor(int a, int b);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  QImage qimage_;
  mutable QMutex qimage_mutex_;
};

}  // namespace rqt_mapping_client

#endif  // rqt_mapping_client__RatioLayoutedFrame_H
