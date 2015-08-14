/**
 * \file	image_frame.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \author	Dirk Thomas
 * \author	TU Darmstadt
 * \date	12/03/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>, 2011, Dirk Thomas,
 * TU Darmstadt
 */

#pragma once

//==============================================================================
// I N C L U D E   F I L E S

// Others librairies and .h
#include <QFrame>
#include <QImage>
#include <QLayout>
#include <QLayoutItem>
#include <QMutex>
#include <QPainter>
#include <QRect>
#include <QSize>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>

//==============================================================================
// N A M E S P A C E S   D E C L A R A T I O N S

/**
 * The namespace containing the whole code of this ROS package,
 * not polluating the global namespace is always a good practice and it became
 * a norme at SONIA. Please define your class in specific namespace.
 */
namespace vision_client {
class ImageFrame;
}

//==============================================================================
// C L A S S E S

/**
 * An image frame.
 * Multiligne.
 */
class vision_client::ImageFrame : public QFrame {
  /**
   * The Q_OBJECT constant provided by Qt.
   * Allow the class to behave as a Widget (provides SLOTS, SIGNALS, etc.)
   */
  Q_OBJECT

 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  /**
   * Constructor.
   *
   * \param [in,out]	parent	If non-null, the parent.
   * \param	flags		  	The flags.
   */
  explicit ImageFrame(QWidget *const parent = nullptr,
                      Qt::WFlags flags = nullptr);

  /** Destructor */
  ~ImageFrame();

  /**
   * Start to capture a video.
   *
   * When this method is called, all the images coming after are saved in a
   * .avi video file.
   * This file is encoded with the HFYU lossless codec.
   * The extension is always .avi.
   * Call the stopVideoCapture to stop the video capture.
   *
   * \param	filename Name of the file with the complete path and without the
   *                 extension. This is given by a dialog box in the GUI.
   * \return	true if the recording as started, false otherwise.
   */
  bool startRecordingVideo(const QString &filename);

  /**
   * Stop to capture the video and save it.
   *
   * \return	true if the recording as started, false otherwise.
   */
  bool stopRecordingVideo();

  /**
   * Save the image in parameter to a jpg file.
   *
   * Use path=../../ in our context
   *
   * \param image The image to save.
   * \param path The absolute or relative path to the directory to save the
   * image in.
   */
  static void saveImage(const cv::Mat &image);

  //==========================================================================
  // G E T T E R S   A N D   S E T T E R S

  /**
   * Sets inner frame minimum size.
   * \param	size	The size.
   */
  void setInnerFrameMinimumSize(const QSize &size);

  /**
   * Sets inner frame maximum size.
   * \param	size	The size.
   */
  void setInnerFrameMaximumSize(const QSize &size);

  /**
   * Gets current time and date.
   *
   * \return	The current time and date.
   */
  static std::string getCurrentTimeAndDate();

  /**
   * Sets inner frame fixed size.
   * \param	size	The size.
   */
  inline void setInnerFrameFixedSize(const QSize &size);

  /**
   * Return either if the video is currently being recorded or not
   *
   * \return True if the video is recorded.
   */
  inline bool isRecording() const;

  /**
   * Return either if the flag for the screenshot saving is true or false
   */
  inline bool isScreenshotEnabled() const;

  /**
   * Set the screenshot flag to true in order to save the next received image
   */
  inline void takeScreenshot();

 public slots:
  //==========================================================================
  // P U B L I C   S L O T S

  /**
   * Handle the signal @Communication.commLineReceivedImage from
   *@Communication
   * Tell the ImageFrame to change its image to the fresh image from
   * communication
   *
   * \param	image	The image.
   */
  void changeImage(const cv::Mat &image);

signals:
  //==========================================================================
  // P U B L I C   S I G N A L S

  /** Delayed update. */
  void delayed_update();

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * TODO Comment this method/attribute.
   * \param [in,out]	event	If non-null, the event.
   */
  void paintEvent(QPaintEvent *event) override;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * TODO Comment this method/attribute.
   * \param	a	The int to process.
   * \param	b	The int to process.
   * \return	An int.
   */
  inline static int greatestCommonDivisor(const int a, const int b);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  /** TODO Comment this method/attribute. */
  QSize _aspect_ratio;

  QImage _image;

  /** TODO Comment this method/attribute. */
  QMutex _mutex;

  /** Allows to save a video using OpenCV VideoWriter class. */
  cv::VideoWriter _output_video;

  /** Is the video recording or not */
  bool _recording;

  /**
   * If this member is true, the next image received will be stored.
   */
  bool _screenshot_enabled;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline void vision_client::ImageFrame::setInnerFrameFixedSize(
    const QSize &size) {
  setInnerFrameMinimumSize(size);
  setInnerFrameMaximumSize(size);
}

//------------------------------------------------------------------------------
//
inline int vision_client::ImageFrame::greatestCommonDivisor(const int a,
                                                            const int b) {
  if (b == 0) {
    return a;
  }
  return greatestCommonDivisor(b, a % b);
}

//------------------------------------------------------------------------------
//
inline bool vision_client::ImageFrame::isRecording() const {
  return _recording;
}

//------------------------------------------------------------------------------
//
inline bool vision_client::ImageFrame::isScreenshotEnabled() const {
  return _screenshot_enabled;
}

//------------------------------------------------------------------------------
//
inline void vision_client::ImageFrame::takeScreenshot() {
  _screenshot_enabled = true;
}