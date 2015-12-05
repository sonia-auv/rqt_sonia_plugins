/**
 * \file	execution.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	12/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#include "gui_vision_client/widgets/execution.h"

namespace gui_vision_client {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
Execution::Execution(const QString &execution_name,
                     const QString &filter_chain_name,
                     const QString &media_name)
    : QObject(),
      _name(execution_name),
      _media_name(media_name),
      _filter_chain_name(filter_chain_name),
      _is_screenshot_enabled(false),
      _execution_widget(new QTableWidgetItem(execution_name)),
      _filter_chain_widget(new QTableWidgetItem(filter_chain_name)),
      _media_widget(new QTableWidgetItem(media_name)) {}

//------------------------------------------------------------------------------
//
Execution::~Execution() {
  if (_execution_widget) {
    delete _execution_widget;
  }
  if (_filter_chain_widget) {
    delete _filter_chain_widget;
  }
  if (_media_widget) {
    delete _media_widget;
  }
}

}  // namespace gui_vision_client
