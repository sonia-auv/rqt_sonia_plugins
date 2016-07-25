/**
 * \file	create_execution_window_controller.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#include "gui_vision_client/gui/create_execution_window_controller.h"

namespace gui_vision_client {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
CreateExecutionWindowController::CreateExecutionWindowController(
    QWidget *const parent)
    : QDialog(parent),
      _ui(std::make_shared<Ui::CreateExecutionWindow>()),
      _execution(),
      _filter_chain(),
      _media() {
  _ui->setupUi(this);

  // Do not send accept when OK button is clicked
  QObject::disconnect(_ui->buttonbox, SIGNAL(accepted()), this, SLOT(accept()));

  // When OK button is clicked, call method #onOKClicked()
  QObject::connect(_ui->buttonbox, SIGNAL(accepted()), this,
                   SLOT(onOKClicked()));

  // When toolButtonFile is clicked, call method #onSelectFile()
  QObject::connect(_ui->file_tool_button, SIGNAL(clicked()), this,
                   SLOT(onSelectFile()));
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void CreateExecutionWindowController::setFilterChainList(
    const QVector<QString> &filter_chains) {
  // TODO change QVector to QStringList
  _ui->filter_chain_combobox->clear();

  QVector<QString>::const_iterator it;
  for (it = filter_chains.begin(); it != filter_chains.end(); ++it) {
    _ui->filter_chain_combobox->addItem(*it);
  }
}

//------------------------------------------------------------------------------
//
void CreateExecutionWindowController::setMediaList(
    const QVector<QString> &medias) {
  _ui->media_combobox->clear();
  for (const auto &media : medias) {
    _ui->media_combobox->addItem(media);
  }
}

//==============================================================================
// Q T   S L O T S

//------------------------------------------------------------------------------
//
void CreateExecutionWindowController::onSelectFile() {
  const auto fileName = QFileDialog::getOpenFileName(
      this, tr("Open File..."), QString(), tr("All Files (*)"));
  _ui->file_line_edit->setText(fileName);
}

//------------------------------------------------------------------------------
//
void CreateExecutionWindowController::onSelectMediaRadio() {
  _ui->media_radio_button->toggle();
}

//------------------------------------------------------------------------------
//
void CreateExecutionWindowController::onSelectFileRadio() {
  _ui->file_radio_button->toggle();
}

//------------------------------------------------------------------------------
//
void CreateExecutionWindowController::onOKClicked() {
  if (_ui->execution_line_edit->text().isEmpty()) {
    (void)QMessageBox::information(
        this, tr("No Execution Name"),
        tr("Please supply a name for the execution."), QMessageBox::Cancel);
  } else if (!_ui->file_radio_button->isChecked() &&
             !_ui->media_radio_button->isChecked()) {
    (void)QMessageBox::information(this, tr("No Inpu Method Choosed"),
                                   tr("Please choose an input method method."),
                                   QMessageBox::Cancel);
  } else if (_ui->file_radio_button->isChecked() &&
             _ui->file_line_edit->text().isEmpty()) {
    (void)QMessageBox::information(this, tr("No File Path"),
                                   tr("Please supply a path for the file."),
                                   QMessageBox::Cancel);
  } else {
    _execution = _ui->execution_line_edit->text();
    _filter_chain = _ui->filter_chain_combobox->currentText();
    if (_ui->file_radio_button->isChecked()) {
      _media = _ui->file_line_edit->text();
    } else {
      _media = _ui->media_combobox->currentText();
    }
    accept();
  }
}

}  // namespace gui_vision_client