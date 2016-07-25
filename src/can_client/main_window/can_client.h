/**
 * \file	can_client.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	20/07/2016
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
#include <sonia_msgs/MissionSwitchMsg.h>
#include <sonia_msgs/CanDevicesProperties.h>
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
  const double BATT_THRESHOLD = 25.6;
  const double BATT_MAX = 28.5;  // Fresh water

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit CanClient(QWidget *parent = 0);
  ~CanClient();

 private slots:

  //============================================================================
  // P R I V A T E   S L O T S

  /**
   * Spin box for pinger frequency. This parameter sets the FFT bandpass
   * to the selected value and applies a filter to input samples. Two possible
   * types of filters exists: chebyshev and elliptic.
   */
  void on_spinBox_Hydr_Pinger_Freq_editingFinished();

  /**
   * Spin box for ADC input gain.
   */
  void on_spinBox_Hydr_Gain_editingFinished();

  /**
   * Spin box for acquisition threshold (now disabled because unused)
   */
  void on_spinBox_Hydr_Acq_Thrs_editingFinished();

  /**
   * Spin box for filter threshold percentage. This value
   * is the minimum power that each microphone must have recorded
   * compared to the most powerful signal of the 1st microphone
   * to consider the ping valid.
   */
  void on_spinBox_Hydr_Filt_Thrs_editingFinished();

  /**
   * Number of ADC samples used to generate the FFT
   */
  void on_spinBox_Hydr_Samp_Count_editingFinished();

  /**
   * Type of phase calculation (now unused)
   */
  void on_spinBox_Hydr_Phase_Calc_Alg_editingFinished();

  /**
   * spin box for FFT Threshold.
   * FFT threshold is the minimum threshold to consider ping. Values are in
   * integer (ADC values).
   */
  void on_spinBox_Hydr_Fft_Thrs_editingFinished();

  /**
   * Spin box for Bandwidth. Pings in the range of pinger freq - bandwidth/2
   * and pinger freq + bandwidth/2 will be considered.
   */
  void on_spinBox_Hydr_Bw_editingFinished();

  /**
   * type of triggering. (Now unused)
   */
  void on_spinBox_Hydr_Fft_Trig_Mode_editingFinished();

  /**
   * Push button for enabling hydrophones.
   */
  void on_pushButton_En_Hydros_clicked();

  /**
   * Push button for data request to hydrophones.
   */
  void on_pushButton_Param_Req_clicked();

  /**
   * push button for enabling FFT. (now unused, because FFT
   * must always be active).
   */
  void on_pushButton_En_Fft_clicked();

  /**
   * Button for FFT graph display
   */
  void on_pushButton_Hydr_MagDeph_clicked();

  /**
   * Button to enable plotting the FFT
   */
  void on_pushButton_Hydr_Plot_clicked();

  /**
   * combo box for enabling FFT reception on the push of
   * pushButton_Param_Req.
   * Wave enable has 3 modes:
   *  - off: no FFT
   *  - when ping: FFT will be sent once pushButton_Param_Req is pushed
   *                and a ping has been received
   *  - raw : FFT will be sent at the moment pushButton_Param_Req is pushed.
   */
  void on_comboBox_Wave_Enable_currentIndexChanged(int index);

  /**
   * combo box acquisition threshold mode (unused)
   */
  void on_comboBox_Acq_Thrs_Mode_currentIndexChanged(int index);

  /**
   * combo box for enabling FFT prefiltering. This is useless
   * except for display. Hydrophones does not use the result of
   * this filtering. The filter that will be applied is the
   * one selected by comboBox_FFT_Prefilt_Type
   */
  void on_comboBox_Prefilter_Enable_currentIndexChanged(int index);

  /**
   * Unused
   */
  void on_comboBox_Cont_Filt_Freq_currentIndexChanged(int index);

  /**
   * Combo box for enabling preamplification gain at ADC input
   */
  void on_comboBox_Preamp_Gain_currentIndexChanged(int index);

  /**
   * Type of FFT filter to apply. See on_comboBox_Prefilter_Enable_currentIndexChanged
   */
  void on_comboBox_FFT_Prefilt_Type_currentIndexChanged(int index);

  /**
   * Spinbox for input cotuff frequency. The filter set by this spin box
   * is an input filter made to remove high frequency noise in the signal.
   */
  void on_spinBox_Hydr_Cutoff_editingFinished();

  /**
   * Spin box to select type of pinger frequency filter (chebyshev or elliptic)
   */
  void on_comboBox_Freq_Filter_Type_currentIndexChanged(int index);

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

  void MissionSwitchCallback(const sonia_msgs::MissionSwitchMsg::ConstPtr &msg);

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

  void on_pushButton_Device_Discover_clicked();

  void CarteNavPropertiesCallback(const sonia_msgs::CanDevicesProperties::ConstPtr &msg);

  void HydrosPropertiesCallback(const sonia_msgs::CanDevicesProperties::ConstPtr &msg);

  void PsuPropertiesCallback(const sonia_msgs::CanDevicesProperties::ConstPtr &msg);

  void MissionSwPropertiesCallback(const sonia_msgs::CanDevicesProperties::ConstPtr &msg);

  void SetDevicesPropertyRow(const sonia_msgs::CanDevicesProperties::ConstPtr &msg, int row);

  void DiverPropertiesCallback(const sonia_msgs::CanDevicesProperties::ConstPtr &msg);

  void on_pushButton_Plot_Current_clicked();

  void on_pushButton_Plot_Voltage_clicked();

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
  ros::Subscriber mission_switch_subs_;

  ros::Subscriber carte_nav_properties_subs_;
  ros::Subscriber mission_switch_properties_subs_;
  ros::Subscriber hydrophones_properties_subs_;
  ros::Subscriber psu_properties_subs_;
  ros::Subscriber diver_properties_subs_;

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

  sonia_msgs::SendCanMessage devices_discovery_srv_;

  // Hyrophones FFT graph curves
  QwtPlotCurve *fft_curve_;
  double freq_points_[64];
  double mag_points_[64];
  QwtPlotCurve *bw_curve_1;
  double bw1_freq_[2];
  QwtPlotCurve *bw_curve_2;
  double bw2_freq_[2];
  double bw_mag_[2];
  QwtPlotCurve *thresh_curve_;
  double thresh_freq_[2];
  double thresh_mag_[2];

  QwtPlotCurve *current_curve_;
  std::vector<double> current_values_;
  std::vector<double> current_time_values_;
  QwtPlotCurve *voltage_curve_;
  std::vector<double> voltage_values_;
  std::vector<double> voltage_time_values_;
  struct timeval psu_monitor_start_time_;
  struct timeval psu_monitor_end_time_;

  // Hydros params enabling
  int hydros_enabled_;
  int fft_enabled_;

  std::thread *thruster_test_thread_;
};

#endif  // CANCLIENT_H
