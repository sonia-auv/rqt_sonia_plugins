/**
 * \file	can_client.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	30/06/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#ifndef CAN_CLIENT_H
#define CAN_CLIENT_H

#include <thread>
#include <QMainWindow>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sonia_msgs/SendCanMessage.h>
#include <sonia_msgs/HydrophonesParams.h>
#include <sonia_msgs/HydrophonesMsg.h>
#include <sonia_msgs/ThrusterMsg.h>
#include <sonia_msgs/BarometerMsg.h>
#include <sensor_msgs/FluidPressure.h>
#include <sonia_msgs/PowerSupplyMsg.h>
#include <qwt_plot_curve.h>

namespace Ui {
class CanClient;
}

class CanClient : public QMainWindow {
  Q_OBJECT

 public:

  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  const double SPEED_OF_SOUND = 1500;  // Fresh water
  const double NORMALIZING_VALUE = 32767;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit CanClient(QWidget *parent = 0);
  ~CanClient();

 private slots:

  //============================================================================
  // P R I V A T E   S L O T S
  void on_spinBox_Hydr_Pinger_Freq_editingFinished();

  void on_spinBox_Hydr_Gain_editingFinished();

  void on_spinBox_Hydr_Acq_Thrs_editingFinished();

  void on_spinBox_Hydr_Filt_Thrs_editingFinished();

  void on_spinBox_Hydr_Samp_Count_editingFinished();

  void on_spinBox_Hydr_Acq_Th_Mode_editingFinished();

  void on_spinBox_Hydr_Phase_Calc_Alg_editingFinished();

  void on_spinBox_Hydr_Preamp_Gain_editingFinished();

  void on_spinBox_Hydr_Fft_Thrs_editingFinished();

  void on_spinBox_Hydr_Fft_Prefilter_editingFinished();

  void on_spinBox_Hydr_Fft_Prefilter_T_editingFinished();

  void on_spinBox_Hydr_Cont_Fil_Freq_editingFinished();

  void on_spinBox_Hydr_Bw_editingFinished();

  void on_spinBox_Hydr_Fft_Trig_Mode_editingFinished();

  void on_pushButton_En_Hydros_clicked();

  void on_pushButton_Param_Req_clicked();

  void on_pushButton_En_Fft_clicked();

  void on_spinBox_Hydr_Wave_En_editingFinished();

  void on_pushButton_Thruster_Speed_Bd_clicked();

  void on_pushButton_Thruster_Speed_Bh_clicked();

  void on_pushButton_Thruster_Speed_Fd_clicked();

  void on_pushButton_Thruster_Speed_Fh_clicked();

  void on_pushButton_Thruster_Speed_P_clicked();

  void on_pushButton_Thruster_Speed_S_clicked();

  void on_pushButton_Thruster_Rot_L_clicked();

  void on_pushButton_Thruster_For_clicked();

  void on_pushButton_Thruster_Rot_R_clicked();

  void on_pushButton_Thruster_Up_clicked();

  void on_pushButton_Thruster_Left_clicked();

  void on_pushButton_Thruster_Stop_clicked();

  void on_pushButton_Thruster_Right_clicked();

  void on_pushButton_Thruster_Back_clicked();

  void on_pushButton_Thruster_Down_clicked();

  void HydrophonesParamsCallback(
      const sonia_msgs::HydrophonesParams::ConstPtr &msg);

  void HydrophonesMsgsCallback(const sonia_msgs::HydrophonesMsg::ConstPtr &msg);

  void ThrusterBackDCallback(const sonia_msgs::ThrusterMsg::ConstPtr &msg);

  void ThrusterFrontDCallback(const sonia_msgs::ThrusterMsg::ConstPtr &msg);

  void ThrusterBackHCallback(const sonia_msgs::ThrusterMsg::ConstPtr &msg);

  void ThrusterFrontHCallback(const sonia_msgs::ThrusterMsg::ConstPtr &msg);

  void ThrusterStarCallback(const sonia_msgs::ThrusterMsg::ConstPtr &msg);

  void ThrusterPortCallback(const sonia_msgs::ThrusterMsg::ConstPtr &msg);

  void BarometerPressCallback(const sensor_msgs::FluidPressure::ConstPtr &msg);

  void BarometerDepthCallback(const sonia_msgs::BarometerMsg::ConstPtr &msg);

  void PsuCallback(const sonia_msgs::PowerSupplyMsg::ConstPtr &msg);

  void on_pushButton_Hydr_Refrsh_clicked();

  void on_lineEdit_Div_Mission_string_editingFinished();

  void on_lineEdit_Div_State_String_editingFinished();

  void on_pushButton_psu_On_12V_1_clicked();

  void on_pushButton_psu_On_12V_2_clicked();

  void on_pushButton_psu_On_Pc_clicked();

  void on_pushButton_psu_On_Dvl_clicked();

  void on_pushButton_psu_On_Light_clicked();

  void on_pushButton_psu_On_Act_clicked();

  void on_pushButton_psu_Off_12V_1_clicked();

  void on_pushButton_psu_Off_12V_2_clicked();

  void on_pushButton_psu_Off_Pc_clicked();

  void on_pushButton_psu_Off_Dvl_clicked();

  void on_pushButton_psu_Off_Light_clicked();

  void on_pushButton_psu_Off_Act_clicked();

  void on_pushButton_psu_On_Motor_1_clicked();

  void on_pushButton_psu_Off_Motor_1_clicked();

  void on_pushButton_psu_On_Motor_2_clicked();

  void on_pushButton_psu_Off_Motor_2_clicked();

  void on_pushButton_Thruster_Test_clicked();


  void on_pushButton_psu_On_Motor_3_clicked();


  void on_pushButton_psu_Off_Motor_3_clicked();

  void on_pushButton_Led_Set_clicked();

  void on_pushButton_Hydr_MagDeph_clicked();

  void on_pushButton_Hydr_Plot_clicked();

 private:

  int ThrusterTest(int arg);

  //============================================================================
  // P R I V A T E   M E M B E R S

  // ROS subscribers
  ros::NodeHandle nh_;
  Ui::CanClient *ui;
  ros::Subscriber hydrophones_params_subs_;
  ros::Subscriber hydrophones_msgs_subs_;
  ros::Subscriber thruster_back_depth_subs_;
  ros::Subscriber thruster_front_depth_subs_;
  ros::Subscriber thrusters_back_heading_msgs_subs_;
  ros::Subscriber thrusters_front_heading_msgs_subs_;
  ros::Subscriber thrusters_starboard_msgs_subs_;
  ros::Subscriber thrusters_port_msgs_subs_;
  ros::Subscriber barometer_pressure_subs_;
  ros::Subscriber barometer_depth_subs_;
  ros::Subscriber psu_subs_;

  // ROS services
  ros::ServiceClient can_service_client_;

  sonia_msgs::SendCanMessage can_hydros_get_params_srv_;
  sonia_msgs::SendCanMessage can_hydros_srv_;
  sonia_msgs::SendCanMessage thrusters_back_depth_srv_;
  sonia_msgs::SendCanMessage thrusters_front_depth_srv_;
  sonia_msgs::SendCanMessage thrusters_back_heading_srv_;
  sonia_msgs::SendCanMessage thrusters_front_heading_srv_;
  sonia_msgs::SendCanMessage thrusters_star_srv_;
  sonia_msgs::SendCanMessage thrusters_port_srv_;
  sonia_msgs::SendCanMessage diver_interface_srv_;
  sonia_msgs::SendCanMessage led_indicator_srv_;
  sonia_msgs::SendCanMessage psu_srv_;

  // Hyrophones FFT graph curves
  QwtPlotCurve *fft_curve_;
  double freq_points_[64];
  double mag_points_[64];
  QwtPlotCurve *bw_curve_1;
  double bw1_freq_[2];
  QwtPlotCurve *bw_curve_2;
  double bw2_freq_[2];
  double bw_mag_[2];
  QwtPlotCurve *thresh_curve;
  double thresh_freq_[2];
  double thresh_mag_[2];

  // Hydros params enabling
  int hydros_enabled_;
  int fft_enabled_;

  std::thread *thruster_test_thread_;
};

#endif  // CANCLIENT_H
