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

#include <sonia_msgs/ProcUnitParameter.h>
#include <QWidget>
#include <QtGui/QCheckBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QLabel>
#include <QtGui/QSlider>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>

namespace gui_mapping_client {

class Parameter : public QWidget {
  Q_OBJECT

 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Parameter>;
  using ConstPtr = std::shared_ptr<const Parameter>;
  using PtrList = std::vector<Parameter::Ptr>;
  using ConstPtrList = std::vector<Parameter::ConstPtr>;

  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  explicit Parameter(
      const std::shared_ptr<sonia_msgs::ProcUnitParameter> &parameter,
      QWidget *const parent = nullptr);

  virtual ~Parameter();

 signals:
  /// Signal that is going to be emitted by the derived class whenever the
  /// Parameter has been changed.
  void ParameterChanged(const Parameter *,
                        const std::shared_ptr<sonia_msgs::ProcUnitParameter> &);

 protected:
  std::shared_ptr<sonia_msgs::ProcUnitParameter> parameter_;
  QLabel *label_;
};

class IntParameter : public Parameter {
  Q_OBJECT
 public:
  explicit IntParameter(
      std::shared_ptr<sonia_msgs::ProcUnitParameter> &parameter,
      QWidget *const parent = nullptr);
  virtual ~IntParameter();

 private slots:
  void OnSliderChanged(int value);

  void OnSpinboxChanged(int value);

 private:
  QWidget *container_widget_;
  QSlider *slider_widget_;
  QSpinBox *spinbox_widget_;
};

class DoubleParameter : public Parameter {
  Q_OBJECT
 public:
  explicit DoubleParameter(
      std::shared_ptr<sonia_msgs::ProcUnitParameter> &parameter,
      QWidget *const parent = nullptr);
  virtual ~DoubleParameter();

 private slots:
  void OnSliderChanged(int value);

  void OnSpinboxChanged(double value);

 private:
  QWidget *container_widget_;
  QSlider *slider_widget_;
  QDoubleSpinBox *spinbox_widget_;
};

class BoolParameter : public Parameter {
  Q_OBJECT
 public:
  explicit BoolParameter(
      std::shared_ptr<sonia_msgs::ProcUnitParameter> &parameter,
      QWidget *const parent = nullptr);

  virtual ~BoolParameter();

 private slots:
  void OnParameterChanged(int state);

 private:
  QCheckBox *widget_;
};

}  // namespace gui_mapping_client
