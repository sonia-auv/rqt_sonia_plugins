/**
 * \file	image_subscriber.cpp
 * \author  Jérémie St-Jules <jeremie.st.jules.prevost@gmail.com>
 * \author  Thomas Fuhrmann <tomesman@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	06/10/2014
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

//==============================================================================
// I N C L U D E   F I L E S

#include "image_subscriber.h"

//==============================================================================
// C O N S T R U C T O R / D E S T R U C T O R   S E C T I O N

//------------------------------------------------------------------------------
//
vision_client::ImageSubscriber::ImageSubscriber(const ros::NodeHandle &hdl)
    : _node_handler(hdl),
      _img_transport(hdl),
      _subscriber(),
      _image(),
      _subscriber_result() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void vision_client::ImageSubscriber::stop() {
  _subscriber.shutdown();
  _subscriber_result.shutdown();
}

//------------------------------------------------------------------------------
//
void vision_client::ImageSubscriber::change(const std::string exec_name) {
  stop();
  // subscribe to the image topic compressed
  image_transport::TransportHints hints("compressed", ros::TransportHints());
  std::string _topic_name = exec_name + "_image";
  _subscriber = _img_transport.subscribe(
      _topic_name, 1, &ImageSubscriber::imageCallback, this, hints);
  // subscribe to the result topic
  std::string _topic_result_name = exec_name + "_result";
  _subscriber_result = _node_handler.subscribe(
      _topic_result_name, 200, &ImageSubscriber::resultCallback, this);
}

//------------------------------------------------------------------------------
//
void vision_client::ImageSubscriber::imageCallback(
    const sensor_msgs::Image::ConstPtr &msg) {
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
