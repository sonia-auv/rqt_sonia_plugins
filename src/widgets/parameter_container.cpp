/**
 * \file	parameter_container.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	13/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "parameter_container.h"

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
vision_client::ParameterContainer::ParameterContainer(QWidget *const parent)
    : QScrollArea(parent),
      ContainerWidget<Parameter>(),
      _current_parameter(nullptr),
      _layout(nullptr),
      _spacer(nullptr) {
  setLineWidth(0);
  _spacer =
      new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
}

vision_client::ParameterContainer::~ParameterContainer() {
  if (_current_parameter) {
    delete _current_parameter;
  }
  if (_layout) {
    delete _layout;
  }
  if (_spacer) {
    delete _spacer;
  }
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void vision_client::ParameterContainer::removeAll() {
  for (auto &component : _components) {
    delete component;
  }
  _components.clear();
}

//------------------------------------------------------------------------------
//
void vision_client::ParameterContainer::createWidgets(
    const Parameter *const &parameter) {
  connect(parameter, SIGNAL(valueChanged(Parameter * const &)), this,
          SLOT(onParameterValueChanged(Parameter * const &)));
  _layout->addWidget(parameter->getMainWidget());
}

//==============================================================================
// Q T   S L O T S   S E C T I O N

//------------------------------------------------------------------------------
//
void vision_client::ParameterContainer::onParameterValueChanged(
    Parameter *const &parameter) {
  emit parameterValueChanged(parameter);
}
