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

#include "gui_mapping_client/widgets/parameter.h"

namespace gui_mapping_client {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
Parameter::Parameter(
    const std::shared_ptr<sonia_msgs::ProcUnitParameter> &parameter,
    QWidget *const parent)
    : QWidget(parent),
      parameter_(parameter),
      label_(new QLabel(QString(parameter_->name.c_str()), parent)) {
  if (parent) {
    parent->layout()->addWidget(label_);
  }
}

//------------------------------------------------------------------------------
//
Parameter::~Parameter() { delete label_; }

//------------------------------------------------------------------------------
//
IntParameter::IntParameter(
    std::shared_ptr<sonia_msgs::ProcUnitParameter> &parameter,
    QWidget *const parent)
    : Parameter(parameter, parent),
      container_widget_(new QWidget(parent)),
      slider_widget_(new QSlider(Qt::Horizontal, nullptr)),
      spinbox_widget_(new QSpinBox(nullptr)) {
  assert(parameter->type == sonia_msgs::ProcUnitParameter::TYPE_INT);

  container_widget_->setLayout(new QHBoxLayout());
  container_widget_->layout()->addWidget(slider_widget_);
  container_widget_->layout()->addWidget(spinbox_widget_);

  connect(slider_widget_, SIGNAL(valueChanged(int)), spinbox_widget_,
          SLOT(setValue(int)));
  connect(spinbox_widget_, SIGNAL(valueChanged(int)), slider_widget_,
          SLOT(setValue(int)));

  connect(slider_widget_, SIGNAL(valueChanged(int)), this,
          SLOT(OnSliderChanged(int)));
  connect(spinbox_widget_, SIGNAL(valueChanged(int)), this,
          SLOT(OnSpinboxChanged(int)));

  if (parent) {
    parent->layout()->addWidget(container_widget_);
  }

  spinbox_widget_->setValue(std::stoi(parameter_->value));
}

//------------------------------------------------------------------------------
//
IntParameter::~IntParameter() { delete container_widget_; }

//------------------------------------------------------------------------------
//
DoubleParameter::DoubleParameter(
    std::shared_ptr<sonia_msgs::ProcUnitParameter> &parameter,
    QWidget *const parent)
    : Parameter(parameter, parent),
      container_widget_(new QWidget(parent)),
      slider_widget_(new QSlider(nullptr)),
      spinbox_widget_(new QDoubleSpinBox(nullptr)) {
  assert(parameter->type == sonia_msgs::ProcUnitParameter::TYPE_DOUBLE);

  container_widget_->setLayout(new QHBoxLayout());
  container_widget_->layout()->addWidget(slider_widget_);
  container_widget_->layout()->addWidget(spinbox_widget_);

  connect(slider_widget_, SIGNAL(valueChanged(int)), spinbox_widget_,
          SLOT(setValue(int)));
  connect(spinbox_widget_, SIGNAL(valueChanged(int)), slider_widget_,
          SLOT(setValue(int)));

  connect(slider_widget_, SIGNAL(valueChanged(int)), this,
          SLOT(OnSliderChanged(int)));
  connect(spinbox_widget_, SIGNAL(valueChanged(int)), this,
          SLOT(OnSpinboxChanged(int)));

  if (parent) {
    parent->layout()->addWidget(container_widget_);
  }

  spinbox_widget_->setValue(std::stoi(parameter_->value));
}

//------------------------------------------------------------------------------
//
DoubleParameter::~DoubleParameter() { delete container_widget_; }

//------------------------------------------------------------------------------
//
BoolParameter::BoolParameter(
    std::shared_ptr<sonia_msgs::ProcUnitParameter> &parameter,
    QWidget *const parent)
    : Parameter(parameter, parent), widget_(new QCheckBox(parent)) {
  assert(parameter->type == sonia_msgs::ProcUnitParameter::TYPE_BOOL);

  auto parameter_value = parameter->value;
  std::transform(parameter_value.begin(), parameter_value.end(),
                 parameter_value.begin(), ::tolower);

  connect(widget_, SIGNAL(stateChanged(int)), this,
          SLOT(OnParameterChanged(int)));

  if (parent) {
    parent->layout()->addWidget(widget_);
  }

  widget_->setChecked(parameter->value == "true");
}

//------------------------------------------------------------------------------
//
BoolParameter::~BoolParameter() { delete widget_; }

//------------------------------------------------------------------------------
//
void IntParameter::OnSliderChanged(int value) {
  parameter_->value = std::to_string(value);
  emit ParameterChanged(this, parameter_);
}

//------------------------------------------------------------------------------
//
void IntParameter::OnSpinboxChanged(int value) {
  parameter_->value = std::to_string(value);
  emit ParameterChanged(this, parameter_);
}

//------------------------------------------------------------------------------
//
void DoubleParameter::OnSliderChanged(int value) {
  parameter_->value = std::to_string(value);
  emit ParameterChanged(this, parameter_);
}

//------------------------------------------------------------------------------
//
void DoubleParameter::OnSpinboxChanged(double value) {
  parameter_->value = std::to_string(value);
  emit ParameterChanged(this, parameter_);
}

//------------------------------------------------------------------------------
//
void BoolParameter::OnParameterChanged(int state) {
  if (state) {
    parameter_->value = "true";
  } else {
    parameter_->value = "false";
  }
  emit ParameterChanged(this, parameter_);
}

}  // namespace gui_mapping_client
