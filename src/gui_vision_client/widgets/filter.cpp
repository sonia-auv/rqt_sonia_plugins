/**
 * \file	filter.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	02/05/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#include "filter.h"

namespace gui_vision_client {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
Filter::Filter(const QString &name, unsigned int index, QObject *const parent)
    : QObject(parent),
      _name(name),
      _index(index),
      _widget(new QListWidgetItem(name)) {}

//------------------------------------------------------------------------------
//
Filter::~Filter() {
  if (_widget) {
    delete _widget;
  }
}

}  // namespace gui_vision_client
