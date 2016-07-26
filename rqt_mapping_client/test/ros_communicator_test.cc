/**
 * \file	ros_communicator_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	25/07/2016
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rqt_mapping_client/mapping_client.h>

using namespace rqt_mapping_client;

ros::NodeHandle *nhp = nullptr;

TEST(RosCommunicator, get_proc_tree_list) {
  MappingClient client;
  client.initPlugin();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rqt_mapping_client");
  nhp = new ros::NodeHandle{"~"};
  return RUN_ALL_TESTS();
}
