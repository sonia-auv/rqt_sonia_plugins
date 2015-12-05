/**
 * \file	execution.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	13/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#include "gui_vision_client/widgets/execution_container.h"

namespace gui_vision_client {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ExecutionContainer::ExecutionContainer(QWidget *const parent)
    : QTableWidget(parent), ContainerWidget<Execution>() {
  // When the selected execution changed, call method changeExecution
  connect(
      this, SIGNAL(currentItemChanged(QTableWidgetItem *, QTableWidgetItem *)),
      this, SLOT(onCurrentItemChanged(QTableWidgetItem *, QTableWidgetItem *)));
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ExecutionContainer::createWidgets(const Execution *const &execution) {
  const unsigned int size = _components.size();
  setRowCount(size);

  setItem(size - 1, 0, execution->getExecutionWidget());
  setItem(size - 1, 1, execution->getMediaWidget());
  setItem(size - 1, 2, execution->getFilterChainWidget());
}

//------------------------------------------------------------------------------
//
void ExecutionContainer::removeAll() {
  for (int i = 0; i < _components.size(); i++) {
    takeItem(i, 0);
    takeItem(i, 1);
    takeItem(i, 2);
  }

  for (auto &component : _components) {
    delete component;
  }
  _components.clear();
}

//==============================================================================
//	Q T   S L O T S   S E C T I O N

//------------------------------------------------------------------------------
//
void ExecutionContainer::onCurrentItemChanged(QTableWidgetItem *current,
                                              QTableWidgetItem *previous) {
  if (!current) {
    return;
  }

  for (auto &component : _components) {
    if ((component)->getExecutionWidget() == current ||
        (component)->getFilterChainWidget() == current ||
        (component)->getMediaWidget() == current) {
      _current = component;
      emit focusChanged(component);
      break;
    }
  }
}

//------------------------------------------------------------------------------
//
void ExecutionContainer::onOutsideExecutionChanged(
    const QString &execution_name) {
  for (auto &component : _components) {
    if ((component)->getName() == execution_name) {
      _current = component;
      setCurrentItem((component)->getExecutionWidget());
      emit focusChanged(component);
      break;
    }
  }
}

}  // namespace gui_vision_client
