/**
 * \file	focused_frame.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	15/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "focused_frame_controller.h"

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
vision_client::FocusedFrameController::FocusedFrameController(
    QWidget *const parent)
    : QFrame(parent) {
  _ui.setupUi(this);

  // When the record checkbox is checked, call onRecordVideoClicked
  QObject::connect(getRecordButton(), SIGNAL(clicked()), this,
                   SLOT(onRecordButtonClicked()));

  // When the record checkbox is checked, call onScreenshotButtonClicked
  QObject::connect(getScreenshotButton(), SIGNAL(clicked()), this,
                   SLOT(onSreenshotButtonClicked()));
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void vision_client::FocusedFrameController::focus() {
  setFrameShape(QFrame::Panel);
  setFrameShadow(QFrame::Sunken);
  setLineWidth(4);
}

//------------------------------------------------------------------------------
//
void vision_client::FocusedFrameController::unfocus() {
  setFrameShape(QFrame::NoFrame);
  setFrameShadow(QFrame::Plain);
  setLineWidth(0);
}

//=============================================================================
// Q T   S L O T S   S E C T I O N

//------------------------------------------------------------------------------
//
void vision_client::FocusedFrameController::mouseReleaseEvent(
    QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    emit clicked(this);
  }
  event->accept();
}

//------------------------------------------------------------------------------
//
void vision_client::FocusedFrameController::onRecordButtonClicked() {
  if (getImageFrame()->isRecording()) {
    if (getImageFrame()->stopRecordingVideo()) {
      _ui.video_record_button->setIcon(getIconFromTheme("media_record"));
    }
  } else {
    const auto file_path =
        QFileDialog::getSaveFileName(this, tr("Save video as..."));
    if (!file_path.isEmpty()) {
      if (getImageFrame()->startRecordingVideo(file_path)) {
        _ui.video_record_button->setIcon(
            getIconFromTheme("media-playback-stop"));
      }
    }
  }
}

//------------------------------------------------------------------------------
//
void vision_client::FocusedFrameController::onSreenshotButtonClicked() {
  getImageFrame()->takeScreenshot();
}