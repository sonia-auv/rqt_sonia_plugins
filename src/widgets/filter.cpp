/**
 * \file	filter.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	02/05/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "filter.h"

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
vision_client::Filter::Filter(const QString &name, unsigned int index,
                              QObject *const parent)
    : QObject(parent),
      _name(name),
      _index(index),
      _widget(new QListWidgetItem(name)) {}

//------------------------------------------------------------------------------
//
vision_client::Filter::~Filter() {
  if (_widget) {
    delete _widget;
  }
}