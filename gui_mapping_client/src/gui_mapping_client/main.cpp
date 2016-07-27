/**
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	26/07/2016
 *
 * \copyright Copyright (c) 2016 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include <QApplication>
#include "gui_mapping_client/gui/main_window_controller.h"
#include "ros/ros.h"

int main(int argc, char *argv[]) {
  //** start ros */
  ros::init(argc, argv, "vision_client");
  ros::NodeHandle nh("~");

  // Qt application
  QApplication application(argc, argv);
  gui_mapping_client::MainWindowController main_window{nh};
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