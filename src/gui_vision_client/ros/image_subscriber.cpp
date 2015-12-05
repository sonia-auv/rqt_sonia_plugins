/**
 * \file	image_subscriber.cpp
 * \author  Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author  Thomas Fuhrmann <tomesman@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	06/10/2014
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#include "gui_vision_client/ros/image_subscriber.h"

namespace gui_vision_client {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ImageSubscriber::ImageSubscriber(const ros::NodeHandle &hdl)
    : _node_handler(hdl),
      _img_transport(hdl),
      _subscriber(),
      _image(),
      _subscriber_result() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ImageSubscriber::stop() {
  _subscriber.shutdown();
  _subscriber_result.shutdown();
}

//------------------------------------------------------------------------------
//
void ImageSubscriber::change(const std::string exec_name) {
  stop();
  // subscribe to the image topic compressed
  image_transport::TransportHints hints("compressed", ros::TransportHints());
  std::string _topic_name = exec_name + "_image";
  _subscriber = _img_transport.subscribe(
      _topic_name, 1, &ImageSubscriber::imageCallback, this, hints);
  // subscribe to the result topic
  std::string _topic_result_name = exec_name + "_result";
  _subscriber_result = _node_handler.subscribe(
      _topic_result_name, 50, &ImageSubscriber::resultCallback, this);
}

//------------------------------------------------------------------------------
//
void ImageSubscriber::imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
  try {
    cv_bridge::CvImageConstPtr ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    _image = ptr->image;
    emit imgSubsriberReceivedImage(_image, this);
  } catch (cv_bridge::Exception &e) {
    // ROS_INFO( "Unable to convert %s image to bgr8",
    // msg->encoding.c_str());
  }
}

}  // namespace gui_vision_client
