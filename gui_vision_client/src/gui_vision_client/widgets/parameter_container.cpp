/**
 * \file	parameter_container.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	13/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#include "gui_vision_client/widgets/parameter_container.h"

namespace gui_vision_client {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ParameterContainer::ParameterContainer(QWidget *const parent)
    : QScrollArea(parent),
      ContainerWidget<Parameter>(),
      _current_parameter(nullptr),
      _layout(nullptr),
      _spacer(nullptr) {
  setLineWidth(0);
  _spacer =
      new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
}

ParameterContainer::~ParameterContainer() {
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
void ParameterContainer::removeAll() {
  for (auto &component : _components) {
    delete component;
  }
  _components.clear();
}

//------------------------------------------------------------------------------
//
void ParameterContainer::createWidgets(const Parameter *const &parameter) {
  connect(parameter, SIGNAL(valueChanged(Parameter * const &)), this,
          SLOT(onParameterValueChanged(Parameter * const &)));
  _layout->addWidget(parameter->getMainWidget());
}

//==============================================================================
// Q T   S L O T S   S E C T I O N

//------------------------------------------------------------------------------
//
void ParameterContainer::onParameterValueChanged(Parameter *const &parameter) {
  emit parameterValueChanged(parameter);
}

}  // namespace gui_vision_client
