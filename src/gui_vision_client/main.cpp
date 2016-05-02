/**
 * \file	main.cpp
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	27/02/2015
 * \copyright	2015 SONIA AUV ETS <sonia@ens.etsmtl.ca>
 */

#include <QApplication>
#include "gui_vision_client/gui/main_window_controller.h"
#include "ros/ros.h"

/**
 * Main function of the vision_client package
 * This will launch the communication with the Vision Server and display de GUI
 */
int main(int argc, char *argv[]) {
  //** start ros */
  ros::init(argc, argv, "vision_client");

  // Qt application
  QApplication application(argc, argv);
  gui_vision_client::MainWindowController main_window;
  main_window.setWindowState(Qt::WindowMaximized);
  main_window.show();

  //** ros spinner */
  // spin sur 2 threads seulement
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // start Qt
  application.exec();
  spinner.stop();

  return EXIT_SUCCESS;
}