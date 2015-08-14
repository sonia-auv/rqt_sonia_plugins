/**
 * \file	filter_container.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	13/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "filter_container.h"

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

vision_client::FilterContainer::FilterContainer(QWidget *const parent)
    : ContainerWidget<Filter>(), QListWidget(parent) {
  // When the selected filter changed, call method changeFilter
  connect(
      this, SIGNAL(currentItemChanged(QListWidgetItem *, QListWidgetItem *)),
      this, SLOT(onCurrentItemChanged(QListWidgetItem *, QListWidgetItem *)));
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void vision_client::FilterContainer::removeAll() {
  for (auto &component : _components) {
    removeItemWidget((component)->getWidget());
    delete component;
  }
  _components.clear();
}

//=============================================================================
// Q T   S L O T S   S E C T I O N

//------------------------------------------------------------------------------
//
void vision_client::FilterContainer::onCurrentItemChanged(
    QListWidgetItem *current, QListWidgetItem *const &previous) {
  if (!current) {
    return;
  }

  for (auto &component : _components) {
    if (component->getWidget() == current) {
      _current = component;
      emit focusChanged(component);
      break;
    }
  }
}
