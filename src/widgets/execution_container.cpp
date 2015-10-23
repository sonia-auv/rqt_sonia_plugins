/**
 * \file	execution.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	13/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "execution_container.h"

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
vision_client::ExecutionContainer::ExecutionContainer(QWidget *const parent)
    : QTableWidget(parent) , ContainerWidget<Execution>() {
  // When the selected execution changed, call method changeExecution
  connect(
      this, SIGNAL(currentItemChanged(QTableWidgetItem *, QTableWidgetItem *)),
      this, SLOT(onCurrentItemChanged(QTableWidgetItem *, QTableWidgetItem *)));
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void vision_client::ExecutionContainer::createWidgets(
    const Execution *const &execution) {
  const unsigned int size = _components.size();
  setRowCount(size);

  setItem(size - 1, 0, execution->getExecutionWidget());
  setItem(size - 1, 1, execution->getMediaWidget());
  setItem(size - 1, 2, execution->getFilterChainWidget());
}

//------------------------------------------------------------------------------
//
void vision_client::ExecutionContainer::removeAll() {
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
void vision_client::ExecutionContainer::onCurrentItemChanged(
    QTableWidgetItem *current, QTableWidgetItem *previous) {
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
void vision_client::ExecutionContainer::onOutsideExecutionChanged(
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
