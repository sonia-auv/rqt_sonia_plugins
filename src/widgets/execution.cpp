/**
 * \file	execution.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	12/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "execution.h"

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
vision_client::Execution::Execution(const QString &execution_name,
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
vision_client::Execution::~Execution() {
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
