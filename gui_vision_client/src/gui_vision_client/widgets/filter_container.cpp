/**
 * \file	filter_container.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	13/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#include "gui_vision_client/widgets/filter_container.h"

namespace gui_vision_client {

//==============================================================================
// C / D T O R S   S E C T I O N

FilterContainer::FilterContainer(QWidget *const parent)
    : QListWidget(parent), ContainerWidget<Filter>() {
  // When the selected filter changed, call method changeFilter
  connect(
      this, SIGNAL(currentItemChanged(QListWidgetItem *, QListWidgetItem *)),
      this, SLOT(onCurrentItemChanged(QListWidgetItem *, QListWidgetItem *)));
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void FilterContainer::removeAll() {
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
void FilterContainer::onCurrentItemChanged(QListWidgetItem *current,
                                           QListWidgetItem *const &previous) {
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

}  // namespace gui_vision_client
