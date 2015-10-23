/**
 * \file	manage_filter_chains_window_controller.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "manage_filter_chains_window_controller.h"

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
vision_client::ManageFilterChainsWindowController::
    ManageFilterChainsWindowController(CommunicationLine &communication,
                                       QWidget *const parent)
    : QDialog(parent),
      _ui(),
      _current_filter_chain(),
      _communication(communication){
  _ui.setupUi(this);

  loadFilterChains();

  QObject::connect(_ui.add_filterchain_button, SIGNAL(clicked()), this,
                   SLOT(onAddFilterChainClicked()));

  QObject::connect(_ui.remove_filterchain_button, SIGNAL(clicked()), this,
                   SLOT(onRemoveFilterChainClicked()));

  QObject::connect(_ui.edit_filterchain_button, SIGNAL(clicked()), this,
                   SLOT(onRenameFilterChainClicked()));

  QObject::connect(_ui.save_filterchain_button, SIGNAL(clicked()), this,
                   SLOT(onCopyFilterChainClicked()));

  QObject::connect(_ui.filterchain_list,
                   SIGNAL(currentTextChanged(const QString &)), this,
                   SLOT(onFilterChainChanged(const QString &)));
}

//------------------------------------------------------------------------------
//
vision_client::ManageFilterChainsWindowController::
    ~ManageFilterChainsWindowController() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void vision_client::ManageFilterChainsWindowController::
    onAddFilterChainClicked() {
  const auto filter_chain = QInputDialog::getText(
      this, "Add a Filter Chain", "Enter the name of the new Filter Chain");

  _communication.createFilterChain(filter_chain);
  loadFilterChains();
}

//------------------------------------------------------------------------------
//
void vision_client::ManageFilterChainsWindowController::
    onRemoveFilterChainClicked() {
  _communication.deleteFilterChain(_current_filter_chain);
  loadFilterChains();
}

//------------------------------------------------------------------------------
//
void vision_client::ManageFilterChainsWindowController::
    onRenameFilterChainClicked() {
  auto ok = false;
  const auto filter_chain = QInputDialog::getText(
      this, "Rename Filter Chain", "Enter the name of the Filter Chain",
      QLineEdit::Normal, _current_filter_chain, &ok);

  if (ok) {
    _communication.renameFilterChain(_current_filter_chain, filter_chain);
    loadFilterChains();
  }
}

//------------------------------------------------------------------------------
//
void vision_client::ManageFilterChainsWindowController::
    onCopyFilterChainClicked() {
  auto ok = false;
  const auto filter_chain = QInputDialog::getText(
      this, "Copy Filter Chain", "Enter the name of the new Filter Chain",
      QLineEdit::Normal, _current_filter_chain, &ok);

  if (ok) {
    _communication.copyFilterChain(_current_filter_chain, filter_chain);
    loadFilterChains();
  }
}

//------------------------------------------------------------------------------
//
void vision_client::ManageFilterChainsWindowController::onFilterChainChanged(
    const QString &new_filter_chain) {
  _current_filter_chain = new_filter_chain;
}
