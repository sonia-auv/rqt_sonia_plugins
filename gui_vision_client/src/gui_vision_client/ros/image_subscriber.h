/**
 * \file	image_subscriber.h
 * \author Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author Thomas Fuhrmann <tomesman@gmail.com>
 * \author Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	06/10/2014
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <proc_image_processing/VisionTarget.h>
#include <QObject>
#include <QString>
#include <opencv2/opencv.hpp>
#include <string>

namespace gui_vision_client {

/**
 * Subscribe to the topics from an execution of the VisionServer.
 *
 * This class suscribes to the _img and to the _result topic associated with
 * an execution which is running on the
 * VisionServer. When an item is published on one topic, this class handles
 * the processing thanks to callback functions.
 * When an image or a result is received, it notifies the CommunicationLine
 * thanks to Qt signals.
 * This mechanism allows to transmit data (image or result) to the
 * CommunicationLine which can send them to the GUI.
 */
class ImageSubscriber : public QObject, public image_transport::Subscriber {
  /**
   * The Q_OBJECT constant provided by Qt.
   * Allow the class to behave as a Widget (provides SLOTS, SIGNALS, etc.)
   */
  Q_OBJECT

 public:
  //==========================================================================
  // C O N S T R U C T O R S   A N D   D E S T R U C T O R

  /**
   * Default constructor.
   *
   * Need the ROS handler to subscribe to the topics and to initialize the
   * ImageTransport.
   *
   * \param	hdl	The ROS handler.
   */
  explicit ImageSubscriber(const ros::NodeHandle &hdl);

  /** Destructor */
  virtual ~ImageSubscriber() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  /**
   * Change the subscribed execution.
   *
   * Stop to subscribe to the former execution's topics and subscribe to the
   * topics from the execution given as parameter.
   *
   * \param	exec_name	Name of the execution to subscribe.
   */
  void change(const std::string exec_name);

  /**
   * Close the connexion with the ROS topics.
   */
  void stop();

 signals:
  //==========================================================================
  // Q T   S I G N A L S

  /**
   * Qt signal emitted when the callback method imageCallback is called.
   *
   * This signal advertises the CommunicationLine that a new image is
   *available.
   *
   * \param	parameter1	The image received.
   * \param	parameter2	The ImageSubscriber which received the image.
   */
  void imgSubsriberReceivedImage(const cv::Mat &,
                                 const ImageSubscriber *) const;

  /**
   * Qt signal emitted when the callback method resultCallback is called.
   *
   * This signal advertises the CommunicationLine that a new result is
   *available.
   *
   * \param	parameter1  The result received.
   * \param	parameter2	The ImageSubscriber which received the result.
   */
  void imgSubscriberReceivedResult(const QString &,
                                   const ImageSubscriber *) const;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /**
   * Function called when an image is published on the _img topic of the
   * execution corresponding to the ImageSubscriber.
   *
   * This callback copy the image and if the recording is enable, save the
   *image
   * in the video.
   * It emits the imgSubsriberReceivedImage signal.
   *
   * \param	msg	The image published on the topic.
   */
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  /**
   * Function called when a result messages is published on the _result topic
   * of the execution corresponding to the ImageSubscriber.
   *
   * It emits the imgSubscriberReceivedResult signal.
   *
   * \param	msg	The result published on the topic.
   */
  inline void resultCallback(
      const proc_image_processing::VisionTarget::ConstPtr &msg) const;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  /** ROS Node handler. */
  ros::NodeHandle _node_handler;

  /** The image transport connexion. */
  image_transport::ImageTransport _img_transport;

  /** Subscriber to the image topic. */
  image_transport::Subscriber _subscriber;

  /** Contains the most recent image. */
  cv::Mat _image;

  /** Subscriber to the result topic. */
  ros::Subscriber _subscriber_result;
};

}  // namespace gui_vision_client
