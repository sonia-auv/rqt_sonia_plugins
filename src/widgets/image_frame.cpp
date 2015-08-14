/**
 * \file	image_frame.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author  Dirk Thomas
 * \author  TU Darmstadt
 * \date	6/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 *
 * This file is based on @Dirk Thomas and @TU Darmstadt
 * work. (https://github.com/ros-visualization/rqt_common_plugins)
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "image_frame.h"

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
vision_client::ImageFrame::ImageFrame(QWidget *const parent, Qt::WFlags flags)
    : QFrame(parent),
      _aspect_ratio(),
      _image(),
      _mutex(),
      _recording(false),
      _screenshot_enabled(false) {
  connect(this, SIGNAL(delayed_update()), this, SLOT(update()),
          Qt::QueuedConnection);
}

//------------------------------------------------------------------------------
//
vision_client::ImageFrame::~ImageFrame() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void vision_client::ImageFrame::setInnerFrameMinimumSize(const QSize &size) {
  int border = lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  setMinimumSize(new_size);
  emit delayed_update();
}

//------------------------------------------------------------------------------
//
void vision_client::ImageFrame::setInnerFrameMaximumSize(const QSize &size) {
  int border = lineWidth();
  QSize new_size = size;
  new_size += QSize(2 * border, 2 * border);
  setMaximumSize(new_size);
  emit delayed_update();
}

//------------------------------------------------------------------------------
//
void vision_client::ImageFrame::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  _mutex.lock();
  if (!_image.isNull()) {
    // TODO: check if full draw is really necessary
    // QPaintEvent* paint_event = dynamic_cast<QPaintEvent*>(event);
    // painter.drawImage(paint_event->rect(), _image);
    painter.drawImage(contentsRect(), _image);
  } else {
    // default image with gradient
    QLinearGradient gradient(0, 0, frameRect().width(), frameRect().height());
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::black);
    painter.setBrush(gradient);
    painter.drawRect(0, 0, frameRect().width() + 1, frameRect().height() + 1);
  }
  _mutex.unlock();
}

//------------------------------------------------------------------------------
//
void vision_client::ImageFrame::saveImage(const cv::Mat &image) {
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  compression_params.push_back(100);

  auto log_path = std::string{getenv("SONIA_WORKSPACE_ROOT")};
  log_path += "/log/";
  cv::imwrite(log_path + getCurrentTimeAndDate() + ".jpg", image,
              compression_params);
}

//------------------------------------------------------------------------------
//
bool vision_client::ImageFrame::startRecordingVideo(const QString &filename) {
  if (isRecording() || _output_video.isOpened()) {
    // ROS_INFO("[VISION_CLIENT] startVideoCapture not opened.");
    return false;
  }

  // mjpg et divx fonctionnels aussi, HFYU est en lossless
  _output_video = cv::VideoWriter(
      (filename.toStdString() + ".avi"), CV_FOURCC('H', 'F', 'Y', 'U'), 15.0,
      cv::Size(_image.size().width(), _image.size().height()));

  if (!_output_video.isOpened()) {
    // ROS_INFO("[VISION_CLIENT] startVideoCapture not opened.");
    return false;
  }
  _recording = true;
  return true;
}

//------------------------------------------------------------------------------
//
bool vision_client::ImageFrame::stopRecordingVideo() {
  if (_output_video.isOpened() && isRecording()) {
    _output_video.release();
    _recording = false;
    return true;
  } else {
    // ROS_INFO("[VISION_CLIENT] stopVideoCapture video is not
    // running.");
    return false;
  }
}

//------------------------------------------------------------------------------
//
std::string vision_client::ImageFrame::getCurrentTimeAndDate() {
  time_t rawtime;
  struct tm *timeinfo;
  char buffer[80];

  time(&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer, 80, "%d-%m-%Y %I:%M:%S", timeinfo);
  std::string str(buffer);
  return str;
}

//==============================================================================
// Q T   S L O T S   S E C T I O N

//------------------------------------------------------------------------------
//
void vision_client::ImageFrame::changeImage(const cv::Mat &image) {
  // on sauvegarde la vidéo
  if (_output_video.isOpened() && isRecording()) {
    cv::Mat temp;
    cv::cvtColor(image, temp, CV_BGR2RGB);
    _output_video.write(temp);
  }

  if (isScreenshotEnabled()) {
    saveImage(image);
    _screenshot_enabled = false;
  }

  auto q_image = new QImage(image.data, image.cols, image.rows, image.step[0],
                            QImage::Format_RGB888);
  _mutex.lock();
  _image = q_image->copy();
  _mutex.unlock();
  emit delayed_update();
}
