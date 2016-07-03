/**
 * \file	can_client.cc
 * \author	Alexi Demers <alexidemers@gmail.com>
 * \date	30/06/2016
 *
 * \copyright	Copyright (c) 2015 SONIA AUV ETS. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <qwt_symbol.h>
#include <thread>
#include "can_client.h"
#include "ui_can_client.h"


//==============================================================================
// S T A T I C   M E M B E R S

//==============================================================================
// C / D T O R   S E C T I O N

//------------------------------------------------------------------------------
//
CanClient::CanClient(QWidget *parent)
    : QMainWindow(parent),
      nh_(),
      ui(new Ui::CanClient),
      hydrophones_params_subs_(),
      hydrophones_msgs_subs_(),
      thruster_back_depth_subs_(),
      thruster_front_depth_subs_(),
      thrusters_back_heading_msgs_subs_(),
      thrusters_front_heading_msgs_subs_(),
      thrusters_starboard_msgs_subs_(),
      thrusters_port_msgs_subs_(),
      barometer_pressure_subs_(),
      barometer_depth_subs_(),
      psu_subs_(),
      can_hydros_get_params_srv_(),
      can_hydros_srv_(),
      thrusters_back_depth_srv_(),
      thrusters_front_depth_srv_(),
      thrusters_back_heading_srv_(),
      thrusters_front_heading_srv_(),
      thrusters_star_srv_(),
      thrusters_port_srv_(),
      diver_interface_srv_(),
      led_indicator_srv_(),
      psu_srv_(),
      hydros_enabled_(false),
      fft_enabled_(false),
      thruster_test_thread_() {
  ui->setupUi(this);

  ui->tableWidget_Hydr_Fft_Mag->setRowCount(128);
  ui->tableWidget_Hydr_Scope_samp->setRowCount(256);

  QTableWidgetItem *new_cell_index;
  QTableWidgetItem *new_cell_value;

  for (int i = 0; i < 256; i++) {
    new_cell_index = new QTableWidgetItem(QString::number(i));
    new_cell_value = new QTableWidgetItem(QString::number(0));
    ui->tableWidget_Hydr_Fft_Mag->setItem(i, 0, new_cell_index);
    ui->tableWidget_Hydr_Fft_Mag->setItem(i, 1, new_cell_value);
  }

  psu_subs_ = nh_.subscribe("/provider_can/power_supply_msgs", 10,
                            &CanClient::PsuCallback, this);

  hydrophones_params_subs_ =
      nh_.subscribe("/provider_can/hydrophones_params", 10,
                    &CanClient::HydrophonesParamsCallback, this);
  hydrophones_msgs_subs_ =
      nh_.subscribe("/provider_can/hydrophones_msgs", 10,
                    &CanClient::HydrophonesMsgsCallback, this);

  thruster_back_depth_subs_ =
      nh_.subscribe("/provider_can/thruster_back_depth_msgs", 10,
                    &CanClient::ThrusterBackDCallback, this);
  thruster_front_depth_subs_ =
      nh_.subscribe("/provider_can/thruster_front_depth_msgs", 10,
                    &CanClient::ThrusterFrontDCallback, this);
  thrusters_back_heading_msgs_subs_ =
      nh_.subscribe("/provider_can/thruster_back_heading_msgs", 10,
                    &CanClient::ThrusterBackHCallback, this);
  thrusters_front_heading_msgs_subs_ =
      nh_.subscribe("/provider_can/thruster_front_heading_msgs", 10,
                    &CanClient::ThrusterFrontHCallback, this);
  thrusters_starboard_msgs_subs_ =
      nh_.subscribe("/provider_can/thruster_starboard_msgs", 10,
                    &CanClient::ThrusterStarCallback, this);
  thrusters_port_msgs_subs_ =
      nh_.subscribe("/provider_can/thruster_port_msgs", 10,
                    &CanClient::ThrusterPortCallback, this);

  barometer_pressure_subs_ =
      nh_.subscribe("/provider_can/barometer_fluidpress_msgs", 10,
                    &CanClient::BarometerPressCallback, this);
  barometer_depth_subs_ =
      nh_.subscribe("/provider_can/barometer_intern_press_msgs", 10,
                    &CanClient::BarometerDepthCallback, this);

  can_service_client_ = nh_.serviceClient<sonia_msgs::SendCanMessage>(
      "/provider_can/send_can_message");

  can_hydros_srv_.request.device_id = can_hydros_srv_.request.DEVICE_ID_sonars;
  can_hydros_srv_.request.unique_id =
      can_hydros_srv_.request.UNIQUE_ID_SONARS_hydrophones;

  can_hydros_get_params_srv_.request.device_id =
      can_hydros_srv_.request.DEVICE_ID_sonars;
  can_hydros_get_params_srv_.request.unique_id =
      can_hydros_srv_.request.UNIQUE_ID_SONARS_hydrophones;
  can_hydros_get_params_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_get_params;

  thrusters_back_depth_srv_.request.device_id =
      thrusters_back_depth_srv_.request.DEVICE_ID_actuators;
  thrusters_back_depth_srv_.request.unique_id =
      thrusters_back_depth_srv_.request.UNIQUE_ID_ACT_back_depth_motor;
  thrusters_back_depth_srv_.request.method_number =
      thrusters_back_depth_srv_.request.METHOD_MOTOR_set_speed;
  thrusters_front_depth_srv_.request.device_id =
      thrusters_back_depth_srv_.request.DEVICE_ID_actuators;
  thrusters_front_depth_srv_.request.unique_id =
      thrusters_back_depth_srv_.request.UNIQUE_ID_ACT_front_depth_motor;
  thrusters_front_depth_srv_.request.method_number =
      thrusters_back_depth_srv_.request.METHOD_MOTOR_set_speed;
  thrusters_back_heading_srv_.request.device_id =
      thrusters_back_depth_srv_.request.DEVICE_ID_actuators;
  thrusters_back_heading_srv_.request.unique_id =
      thrusters_back_depth_srv_.request.UNIQUE_ID_ACT_back_heading_motor;
  thrusters_back_heading_srv_.request.method_number =
      thrusters_back_depth_srv_.request.METHOD_MOTOR_set_speed;
  thrusters_front_heading_srv_.request.device_id =
      thrusters_back_depth_srv_.request.DEVICE_ID_actuators;
  thrusters_front_heading_srv_.request.unique_id =
      thrusters_back_depth_srv_.request.UNIQUE_ID_ACT_front_heading_motor;
  thrusters_front_heading_srv_.request.method_number =
      thrusters_back_depth_srv_.request.METHOD_MOTOR_set_speed;
  thrusters_star_srv_.request.device_id =
      thrusters_back_depth_srv_.request.DEVICE_ID_actuators;
  thrusters_star_srv_.request.unique_id =
      thrusters_back_depth_srv_.request.UNIQUE_ID_ACT_starboard_motor;
  thrusters_star_srv_.request.method_number =
      thrusters_back_depth_srv_.request.METHOD_MOTOR_set_speed;
  thrusters_port_srv_.request.device_id =
      thrusters_back_depth_srv_.request.DEVICE_ID_actuators;
  thrusters_port_srv_.request.unique_id =
      thrusters_back_depth_srv_.request.UNIQUE_ID_ACT_port_motor;
  thrusters_port_srv_.request.method_number =
      thrusters_back_depth_srv_.request.METHOD_MOTOR_set_speed;

  diver_interface_srv_.request.device_id =
      diver_interface_srv_.request.DEVICE_ID_interfaces;
  diver_interface_srv_.request.unique_id =
      diver_interface_srv_.request.UNIQUE_ID_INTERFACE_diver;

  led_indicator_srv_.request.device_id =
      led_indicator_srv_.request.DEVICE_ID_lights;
  led_indicator_srv_.request.unique_id =
      led_indicator_srv_.request.UNIQUE_ID_LIGHT_led_indicator;

  psu_srv_.request.device_id = psu_srv_.request.DEVICE_ID_power;
  psu_srv_.request.unique_id =
      psu_srv_.request.UNIQUE_ID_POWER_power_distribution;

  can_service_client_.call(can_hydros_get_params_srv_);

  ui->plot_Hydr_Fft->setAxisScale(QwtPlot::yLeft, 0, 100000, 10000);
  ui->plot_Hydr_Fft->setAxisScale(QwtPlot::xBottom, 0, 45000, 5000);
  ui->plot_Hydr_Fft->setAxisTitle(QwtPlot::yLeft, "amplitude (int)");
  ui->plot_Hydr_Fft->setAxisTitle(QwtPlot::xBottom, "frequency (Hz)");

  fft_curve_ = new QwtPlotCurve();
  bw_curve_1 = new QwtPlotCurve();
  bw_curve_2 = new QwtPlotCurve();
  thresh_curve = new QwtPlotCurve();

  for (uint16_t i = 0; i < 64; i++) {
    freq_points_[i] = i * 813;
    mag_points_[i] = 100;
  }
  bw1_freq_[0] = ui->spinBox_Hydr_Pinger_Freq->value() * 1000 -
                 ui->spinBox_Hydr_Bw->value() * 813;
  bw1_freq_[1] = ui->spinBox_Hydr_Pinger_Freq->value() * 1000 -
                 ui->spinBox_Hydr_Bw->value() * 813;
  bw2_freq_[0] = ui->spinBox_Hydr_Pinger_Freq->value() * 1000 +
                 ui->spinBox_Hydr_Bw->value() * 813;
  bw2_freq_[1] = ui->spinBox_Hydr_Pinger_Freq->value() * 1000 +
                 ui->spinBox_Hydr_Bw->value() * 813;
  bw_mag_[0] = 0;
  bw_mag_[1] = 100000;
  thresh_freq_[0] = 0;
  thresh_freq_[1] = 45000;
  thresh_mag_[0] = ui->spinBox_Hydr_Fft_Thrs->value();
  thresh_mag_[1] = ui->spinBox_Hydr_Fft_Thrs->value();

  fft_curve_->setSamples(freq_points_, mag_points_, 64);
  fft_curve_->setPen(QColor(Qt::blue));
  fft_curve_->attach(ui->plot_Hydr_Fft);
  bw_curve_1->setSamples(bw1_freq_, bw_mag_, 2);
  bw_curve_1->setPen(QColor(Qt::green));
  bw_curve_1->attach(ui->plot_Hydr_Fft);
  bw_curve_2->setSamples(bw2_freq_, bw_mag_, 2);
  bw_curve_2->setPen(QColor(Qt::green));
  bw_curve_2->attach(ui->plot_Hydr_Fft);
  thresh_curve->setSamples(thresh_freq_, thresh_mag_, 2);
  thresh_curve->setPen(QColor(Qt::red));
  thresh_curve->attach(ui->plot_Hydr_Fft);
  ui->plot_Hydr_Fft->replot();

  on_pushButton_Hydr_MagDeph_clicked();
  //on_pushButton_Hydr_MagDeph_clicked();
}

//------------------------------------------------------------------------------
//

CanClient::~CanClient() { delete ui; }

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Pinger_Freq_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_set_pinger_freq;

  // hydrophone frequencies are coded quite randomly. this
  // switch case is for conversion from kHz purpose.
  switch (ui->spinBox_Hydr_Pinger_Freq->value()) {
    case 13:
      can_hydros_srv_.request.parameter_value = 5;
      break;
    case 15:
      can_hydros_srv_.request.parameter_value = 6;
      break;
    case 22:
      can_hydros_srv_.request.parameter_value = 7;
      break;
    case 23:
      can_hydros_srv_.request.parameter_value = 8;
      break;
    case 24:
      can_hydros_srv_.request.parameter_value = 9;
      break;
    case 25:
      can_hydros_srv_.request.parameter_value = 10;
      break;
    case 26:
      can_hydros_srv_.request.parameter_value = 11;
      break;
    case 27:
      can_hydros_srv_.request.parameter_value = 12;
      break;
    case 28:
      can_hydros_srv_.request.parameter_value = 13;
      break;
    case 29:
      can_hydros_srv_.request.parameter_value = 14;
      break;
    case 30:
      can_hydros_srv_.request.parameter_value = 15;
      break;
  }

  // updating bandwidth curves on FFT graph
  bw1_freq_[0] = ui->spinBox_Hydr_Pinger_Freq->value() * 1000 -
                 ui->spinBox_Hydr_Bw->value() * 813;
  bw1_freq_[1] = ui->spinBox_Hydr_Pinger_Freq->value() * 1000 -
                 ui->spinBox_Hydr_Bw->value() * 813;
  bw2_freq_[0] = ui->spinBox_Hydr_Pinger_Freq->value() * 1000 +
                 ui->spinBox_Hydr_Bw->value() * 813;
  bw2_freq_[1] = ui->spinBox_Hydr_Pinger_Freq->value() * 1000 +
                 ui->spinBox_Hydr_Bw->value() * 813;
  bw_curve_1->setSamples(bw1_freq_, bw_mag_, 2);
  bw_curve_2->setSamples(bw2_freq_, bw_mag_, 2);
  ui->plot_Hydr_Fft->replot();

  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Gain_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_set_gain;
  can_hydros_srv_.request.parameter_value =
      (float)ui->spinBox_Hydr_Gain->value();
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Acq_Thrs_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_set_acq_threshold;
  can_hydros_srv_.request.parameter_value =
      (float)ui->spinBox_Hydr_Acq_Thrs->value();
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Filt_Thrs_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_set_filter_threshold;
  can_hydros_srv_.request.parameter_value =
      (float)ui->spinBox_Hydr_Filt_Thrs->value();
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Wave_En_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_wave_enable;
  can_hydros_srv_.request.parameter_value =
      (float)ui->spinBox_Hydr_Wave_En->value();
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Samp_Count_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_set_sample_count;
  can_hydros_srv_.request.parameter_value =
      (float)ui->spinBox_Hydr_Samp_Count->value();
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Acq_Th_Mode_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_set_acq_thrs_mode;
  can_hydros_srv_.request.parameter_value =
      (float)ui->spinBox_Hydr_Acq_Th_Mode->value();
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Phase_Calc_Alg_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_set_phase_calc_alg;
  can_hydros_srv_.request.parameter_value =
      (float)ui->spinBox_Hydr_Phase_Calc_Alg->value();
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Preamp_Gain_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_set_preamp_gain;
  can_hydros_srv_.request.parameter_value =
      (float)ui->spinBox_Hydr_Preamp_Gain->value();
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Fft_Thrs_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_set_fft_threshold;
  can_hydros_srv_.request.parameter_value =
      (float)ui->spinBox_Hydr_Fft_Thrs->value();
  can_service_client_.call(can_hydros_srv_);

  thresh_mag_[0] = ui->spinBox_Hydr_Fft_Thrs->value();
  thresh_mag_[1] = ui->spinBox_Hydr_Fft_Thrs->value();
  thresh_curve->setSamples(thresh_freq_, thresh_mag_, 2);
  ui->plot_Hydr_Fft->replot();
}

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Fft_Prefilter_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_set_fft_prefilter;
  can_hydros_srv_.request.parameter_value =
      (float)ui->spinBox_Hydr_Fft_Prefilter->value();
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Fft_Prefilter_T_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_set_fft_prefilter_type;
  can_hydros_srv_.request.parameter_value =
      (float)ui->spinBox_Hydr_Fft_Prefilter_T->value();
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Cont_Fil_Freq_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_set_cont_filter_freq;
  can_hydros_srv_.request.parameter_value =
      (float)ui->spinBox_Hydr_Cont_Fil_Freq->value();
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Bw_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_set_fft_bandwidth;
  can_hydros_srv_.request.parameter_value = (float)ui->spinBox_Hydr_Bw->value();
  can_service_client_.call(can_hydros_srv_);

  // updating bandwidth curves on FFT graph
  bw1_freq_[0] = ui->spinBox_Hydr_Pinger_Freq->value() * 1000 -
                 ui->spinBox_Hydr_Bw->value() * 813;
  bw1_freq_[1] = ui->spinBox_Hydr_Pinger_Freq->value() * 1000 -
                 ui->spinBox_Hydr_Bw->value() * 813;
  bw2_freq_[0] = ui->spinBox_Hydr_Pinger_Freq->value() * 1000 +
                 ui->spinBox_Hydr_Bw->value() * 813;
  bw2_freq_[1] = ui->spinBox_Hydr_Pinger_Freq->value() * 1000 +
                 ui->spinBox_Hydr_Bw->value() * 813;
  bw_curve_1->setSamples(bw1_freq_, bw_mag_, 2);
  bw_curve_2->setSamples(bw2_freq_, bw_mag_, 2);
  ui->plot_Hydr_Fft->replot();
}

//------------------------------------------------------------------------------
//

void CanClient::on_spinBox_Hydr_Fft_Trig_Mode_editingFinished() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_set_fft_trig_mode;
  can_hydros_srv_.request.parameter_value =
      (float)ui->spinBox_Hydr_Fft_Trig_Mode->value();
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_En_Hydros_clicked() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_hydro_enable;
  if (!hydros_enabled_) {
    can_hydros_srv_.request.parameter_value = 1;
  } else {
    can_hydros_srv_.request.parameter_value = 0;
  }
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Param_Req_clicked() {
  if (ui->spinBox_Hydr_Wave_En->value() == 0) {
    ui->spinBox_Hydr_Wave_En->setValue(1);
    on_spinBox_Hydr_Wave_En_editingFinished();
  }
  if (!hydros_enabled_) {
    on_pushButton_En_Hydros_clicked();
  }
  if (!fft_enabled_) {
    on_pushButton_En_Fft_clicked();
  }

  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_send_data_req;
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_En_Fft_clicked() {
  can_hydros_srv_.request.method_number =
      can_hydros_srv_.request.METHOD_HYDRO_fft_enable;
  if (!fft_enabled_) {
    can_hydros_srv_.request.parameter_value = 1;

  } else {
    can_hydros_srv_.request.parameter_value = 0;
  }
  can_service_client_.call(can_hydros_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_Speed_Bd_clicked() {
  thrusters_back_depth_srv_.request.parameter_value =
      ui->spinBox_Thruster_Back_Depth->value();
  can_service_client_.call(thrusters_back_depth_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_Speed_Bh_clicked() {
  thrusters_back_heading_srv_.request.parameter_value =
      ui->spinBox_Thruster_Back_Heading->value();
  can_service_client_.call(thrusters_back_heading_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_Speed_Fd_clicked() {
  thrusters_front_depth_srv_.request.parameter_value =
      ui->spinBox_Thruster_Front_Depth->value();
  can_service_client_.call(thrusters_front_depth_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_Speed_Fh_clicked() {
  thrusters_front_heading_srv_.request.parameter_value =
      ui->spinBox_Thruster_Front_Heading->value();
  can_service_client_.call(thrusters_front_heading_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_Speed_P_clicked() {
  thrusters_port_srv_.request.parameter_value =
      ui->spinBox_Thruster_Port->value();
  can_service_client_.call(thrusters_port_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_Speed_S_clicked() {
  thrusters_star_srv_.request.parameter_value =
      ui->spinBox_Thruster_Starboard->value();
  can_service_client_.call(thrusters_star_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_Rot_L_clicked() {
  thrusters_back_heading_srv_.request.parameter_value =
      -ui->spinBox_Thruster_Back_Heading->value();
  can_service_client_.call(thrusters_back_heading_srv_);
  thrusters_front_heading_srv_.request.parameter_value =
      ui->spinBox_Thruster_Front_Heading->value();
  can_service_client_.call(thrusters_front_heading_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_For_clicked() {
  thrusters_star_srv_.request.parameter_value =
      ui->spinBox_Thruster_Starboard->value();
  can_service_client_.call(thrusters_star_srv_);
  thrusters_port_srv_.request.parameter_value =
      ui->spinBox_Thruster_Port->value();
  can_service_client_.call(thrusters_port_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_Rot_R_clicked() {
  thrusters_back_heading_srv_.request.parameter_value =
      ui->spinBox_Thruster_Back_Heading->value();
  can_service_client_.call(thrusters_back_heading_srv_);
  thrusters_front_heading_srv_.request.parameter_value =
      -ui->spinBox_Thruster_Front_Heading->value();
  can_service_client_.call(thrusters_front_heading_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_Up_clicked() {
  thrusters_front_depth_srv_.request.parameter_value =
      ui->spinBox_Thruster_Front_Depth->value();
  can_service_client_.call(thrusters_front_depth_srv_);
  thrusters_back_depth_srv_.request.parameter_value =
      ui->spinBox_Thruster_Back_Depth->value();
  can_service_client_.call(thrusters_back_depth_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_Left_clicked() {
  thrusters_back_heading_srv_.request.parameter_value =
      ui->spinBox_Thruster_Back_Heading->value();
  can_service_client_.call(thrusters_back_heading_srv_);
  thrusters_front_heading_srv_.request.parameter_value =
      ui->spinBox_Thruster_Front_Heading->value();
  can_service_client_.call(thrusters_front_heading_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_Stop_clicked() {
  thrusters_front_depth_srv_.request.parameter_value = 0;
  can_service_client_.call(thrusters_front_depth_srv_);
  thrusters_back_depth_srv_.request.parameter_value = 0;
  can_service_client_.call(thrusters_back_depth_srv_);
  thrusters_back_heading_srv_.request.parameter_value = 0;
  can_service_client_.call(thrusters_back_heading_srv_);
  thrusters_front_heading_srv_.request.parameter_value = 0;
  can_service_client_.call(thrusters_front_heading_srv_);
  thrusters_star_srv_.request.parameter_value = 0;
  can_service_client_.call(thrusters_star_srv_);
  thrusters_port_srv_.request.parameter_value = 0;
  can_service_client_.call(thrusters_port_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_Right_clicked() {
  thrusters_back_heading_srv_.request.parameter_value =
      -ui->spinBox_Thruster_Back_Heading->value();
  can_service_client_.call(thrusters_back_heading_srv_);
  thrusters_front_heading_srv_.request.parameter_value =
      -ui->spinBox_Thruster_Front_Heading->value();
  can_service_client_.call(thrusters_front_heading_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_Back_clicked() {
  thrusters_star_srv_.request.parameter_value =
      -ui->spinBox_Thruster_Starboard->value();
  can_service_client_.call(thrusters_star_srv_);
  thrusters_port_srv_.request.parameter_value =
      -ui->spinBox_Thruster_Port->value();
  can_service_client_.call(thrusters_port_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Thruster_Down_clicked() {
  thrusters_front_depth_srv_.request.parameter_value =
      -ui->spinBox_Thruster_Front_Depth->value();
  can_service_client_.call(thrusters_front_depth_srv_);
  thrusters_back_depth_srv_.request.parameter_value =
      -ui->spinBox_Thruster_Back_Depth->value();
  can_service_client_.call(thrusters_back_depth_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::HydrophonesParamsCallback(
    const sonia_msgs::HydrophonesParams::ConstPtr &msg) {
  ui->label_Hydr_Acq_Th_Mode->setNum(msg->acq_thrs_mode);
  ui->label_Hydr_Acq_Thrs->setNum(msg->acq_threshold);
  ui->label_Hydr_Bw->setNum(msg->fft_bandwidth);
  ui->label_Hydr_Cont_Fil_Freq->setNum(msg->continuous_filter_freq);
  ui->label_Hydr_Wave_En->setNum(msg->wave_enable);
  ui->label_Hydr_Cutoff->setNum(msg->set_cutoff_freq);
  ui->label_Hydr_Fft_Prefilter->setNum(msg->fft_prefilter);
  ui->label_Hydr_Fft_Prefilter_T->setNum(msg->fft_prefilter_type);
  ui->label_Hydr_Fft_Thrs->setNum(msg->fft_threshold);
  ui->label_Hydr_Fft_Trig_Mode->setNum(msg->fft_trig_mode_Param);
  ui->label_Hydr_Filt_Thrs->setNum(msg->filter_threshold);
  ui->label_Hydr_Gain->setNum(msg->gain);
  ui->label_Hydr_Phase_Calc_Alg->setNum(msg->phase_calc_alg);
  ui->label_Hydr_Samp_Count->setNum(msg->sample_count);
  ui->label_Hydr_Preamp_Gain->setNum(msg->set_preamp_gain);
  hydros_enabled_ = msg->hydro_enable;
  fft_enabled_ = msg->fft_enable;

  if (fft_enabled_) {
    ui->label_Hydr_Acq_Thrs->setEnabled(false);
    ui->spinBox_Hydr_Acq_Thrs->setEnabled(false);
    ui->pushButton_En_Fft->setText("Disable FFT");
  }

  else {
    ui->spinBox_Hydr_Acq_Thrs->setEnabled(true);
    ui->label_Hydr_Acq_Thrs->setEnabled(true);
    ui->pushButton_En_Fft->setText("Enable FFT");
  }

  if (hydros_enabled_) {
    ui->pushButton_En_Hydros->setText("Disable Hydrophone");
  } else {
    ui->pushButton_En_Hydros->setText("Enable Hydrophone");
  }

  // hydrophone frequencies are coded quite randomly. this
  // switch case is for conversion to kHz purpose.
  switch (msg->pinger_freq) {
    case 5:
      ui->label_Hydr_Ping_Freq->setNum(13);
      break;
    case 6:
      ui->label_Hydr_Ping_Freq->setNum(15);
      break;
    case 7:
      ui->label_Hydr_Ping_Freq->setNum(22);
      break;
    case 8:
      ui->label_Hydr_Ping_Freq->setNum(23);
      break;
    case 9:
      ui->label_Hydr_Ping_Freq->setNum(24);
      break;
    case 10:
      ui->label_Hydr_Ping_Freq->setNum(25);
      break;
    case 11:
      ui->label_Hydr_Ping_Freq->setNum(26);
      break;
    case 12:
      ui->label_Hydr_Ping_Freq->setNum(27);
      break;
    case 13:
      ui->label_Hydr_Ping_Freq->setNum(28);
      break;
    case 14:
      ui->label_Hydr_Ping_Freq->setNum(29);
      break;
    case 15:
      ui->label_Hydr_Ping_Freq->setNum(30);
      break;
  }
}

//------------------------------------------------------------------------------
//

void CanClient::HydrophonesMsgsCallback(
    const sonia_msgs::HydrophonesMsg::ConstPtr &msg) {
  static uint32_t freq_slowdwn_count = 0;

  // if a ping was received and dephasage calculated
  if (msg->dephasage1_updated) {
    double pro_deph_1;
    double pro_deph_2;
    double pro_deph_3;

    // sets table widget with values
    QTableWidgetItem *new_cell_d1_raw =
        new QTableWidgetItem(QString::number(msg->dephasage1_d1));
    QTableWidgetItem *new_cell_d2_raw =
        new QTableWidgetItem(QString::number(msg->dephasage1_d2));
    QTableWidgetItem *new_cell_d3_raw =
        new QTableWidgetItem(QString::number(msg->dephasage1_d3));
    QTableWidgetItem *new_cell_freq =
        new QTableWidgetItem(QString::number(msg->dephasage1_pinger_freq));

    // makes a part of the processing done by AUV6
    if (msg->dephasage1_pinger_freq != 0) {
      // distance in cm for all phase displacements
      if ((msg->dephasage1_d1 & 0x8000) == 0x8000) {
        pro_deph_1 = (pow(2, 15) - (msg->dephasage1_d1 & 0x8000)) * -1.0;
      } else {
        pro_deph_1 = msg->dephasage1_d1;
      }

      if ((msg->dephasage1_d2 & 0x8000) == 0x8000) {
        pro_deph_2 = (pow(2, 15) - (msg->dephasage1_d2 & 0x8000)) * -1.0;
      } else {
        pro_deph_2 = msg->dephasage1_d2;
      }

      if ((msg->dephasage1_d3 & 0x8000) == 0x8000) {
        pro_deph_3 = (pow(2, 15) - (msg->dephasage1_d3 & 0x8000)) * -1.0;
      } else {
        pro_deph_3 = msg->dephasage1_d3;
      }

      pro_deph_1 = (pro_deph_1 / NORMALIZING_VALUE) *
                   (SPEED_OF_SOUND / msg->dephasage1_pinger_freq) * 100;
      pro_deph_2 = (pro_deph_2 / NORMALIZING_VALUE) *
                   (SPEED_OF_SOUND / msg->dephasage1_pinger_freq) * 100;
      pro_deph_3 = (pro_deph_3 / NORMALIZING_VALUE) *
                   (SPEED_OF_SOUND / msg->dephasage1_pinger_freq) * 100;
    }

    // shows processed values
    QTableWidgetItem *new_cell_d1_pro =
        new QTableWidgetItem(QString::number(pro_deph_1));
    QTableWidgetItem *new_cell_d2_pro =
        new QTableWidgetItem(QString::number(pro_deph_2));
    QTableWidgetItem *new_cell_d3_pro =
        new QTableWidgetItem(QString::number(pro_deph_3));

    // deleting old table widget cells
    for (uint16_t i = 0; i < 4; i++) {
      delete (ui->tableWidget_Hydr_Deph->item(i, 0));
      delete (ui->tableWidget_Hydr_Deph->item(i, 1));
    }

    // setting up new cells
    ui->tableWidget_Hydr_Deph->setItem(0, 0, new_cell_d1_raw);
    ui->tableWidget_Hydr_Deph->setItem(0, 1, new_cell_d1_pro);
    ui->tableWidget_Hydr_Deph->setItem(1, 0, new_cell_d2_raw);
    ui->tableWidget_Hydr_Deph->setItem(1, 1, new_cell_d2_pro);
    ui->tableWidget_Hydr_Deph->setItem(2, 0, new_cell_d3_raw);
    ui->tableWidget_Hydr_Deph->setItem(2, 1, new_cell_d3_pro);
    ui->tableWidget_Hydr_Deph->setItem(3, 0, new_cell_freq);
  }

  // if a new maximum frequency was received
  if (msg->hydro_freq_updated) {
    freq_slowdwn_count++;
    if (freq_slowdwn_count >= 50) {
      ui->label_Hydr_Freq->setText(
          QString::number(msg->hydro_freq_index * 813));
      freq_slowdwn_count = 0;
    }
  }

  // if FFT samples are received
  if (msg->magn_samples_updated) {
    QTableWidgetItem *new_cell_index;
    QTableWidgetItem *new_cell_value;

    // updates FFT curve points
    for (uint16_t i = 0; i < 64; i++) {
      mag_points_[i] = (double)msg->magnitude_values[i];
    }

    // updates table widget values
    for (uint16_t i = 0; i < msg->magnitude_values.size(); i++) {
      delete (ui->tableWidget_Hydr_Fft_Mag->item(i, 0));
      delete (ui->tableWidget_Hydr_Fft_Mag->item(i, 1));
      new_cell_index = new QTableWidgetItem();
      new_cell_value = new QTableWidgetItem();
      new_cell_index->setData(Qt::DisplayRole, QString::number(i * 813));
      new_cell_value->setData(Qt::DisplayRole, msg->magnitude_values[i]);
      ui->tableWidget_Hydr_Fft_Mag->setItem(i, 0, new_cell_index);
      ui->tableWidget_Hydr_Fft_Mag->setItem(i, 1, new_cell_value);
    }

    ui->tableWidget_Hydr_Fft_Mag->sortByColumn(1,
                                               Qt::SortOrder::DescendingOrder);

    // highlighting frequency values between the asked interval
    for (uint32_t i = 0; i < msg->magnitude_values.size(); i++) {
      if (ui->tableWidget_Hydr_Fft_Mag->item(i, 0)->text().toDouble() / 1000 <
              ui->label_Hydr_Ping_Freq->text().toDouble() +
                  ui->label_Hydr_Bw->text().toDouble() &&
          ui->tableWidget_Hydr_Fft_Mag->item(i, 0)->text().toDouble() / 1000 >
              ui->label_Hydr_Ping_Freq->text().toDouble() -
                  ui->label_Hydr_Bw->text().toDouble()) {
        ui->tableWidget_Hydr_Fft_Mag->item(i, 0)->setBackground(Qt::green);
      }
    }
  }
  if (msg->scope_samples_updated) {
    QTableWidgetItem *new_cell_index;
    QTableWidgetItem *new_cell_value1;
    QTableWidgetItem *new_cell_value2;
    QTableWidgetItem *new_cell_value3;
    QTableWidgetItem *new_cell_value4;

    for (uint16_t i = 0; i < msg->scope_values.size(); i++) {
      delete (ui->tableWidget_Hydr_Scope_samp->item(i, 0));
      delete (ui->tableWidget_Hydr_Scope_samp->item(i, 1));
      delete (ui->tableWidget_Hydr_Scope_samp->item(i, 2));
      delete (ui->tableWidget_Hydr_Scope_samp->item(i, 3));
      delete (ui->tableWidget_Hydr_Scope_samp->item(i, 4));
      new_cell_index = new QTableWidgetItem(QString::number(i));
      new_cell_value1 = new QTableWidgetItem(
          QString::number(msg->scope_values[i].samples[0]));
      new_cell_value2 = new QTableWidgetItem(
          QString::number(msg->scope_values[i].samples[1]));
      new_cell_value3 = new QTableWidgetItem(
          QString::number(msg->scope_values[i].samples[2]));
      new_cell_value4 = new QTableWidgetItem(
          QString::number(msg->scope_values[i].samples[3]));
      ui->tableWidget_Hydr_Scope_samp->setItem(i, 0, new_cell_index);
      ui->tableWidget_Hydr_Scope_samp->setItem(i, 1, new_cell_value1);
      ui->tableWidget_Hydr_Scope_samp->setItem(i, 2, new_cell_value2);
      ui->tableWidget_Hydr_Scope_samp->setItem(i, 3, new_cell_value3);
      ui->tableWidget_Hydr_Scope_samp->setItem(i, 4, new_cell_value4);
    }
  }
}

//------------------------------------------------------------------------------
//

void CanClient::PsuCallback(const sonia_msgs::PowerSupplyMsg::ConstPtr &msg) {
  // updates all power supply values
  ui->label_Psu_12V_Cur_1->setNum(msg->volt_bus2_current);
  ui->label_Psu_12V_Cur_2->setNum(msg->volt_bus1_current);
  ui->label_Psu_Motor_Cur_1->setNum(msg->motor_bus1_current);
  ui->label_Psu_Motor_Cur_2->setNum(msg->motor_bus2_current);
  ui->label_Psu_Motor_Cur_3->setNum(msg->motor_bus3_current);
  ui->label_Psu_Act_Cur->setNum(msg->actuator_bus_current);
  ui->label_Psu_Dvl_Cur->setNum(msg->dvl_current);
  ui->label_Psu_PC_Cur->setNum(msg->pc_current);
  ui->label_Psu_Light_Cur->setNum(msg->light_current);
  ui->label_Psu_12V_Vol_1->setNum(msg->volt_bus1_voltage);
  ui->label_Psu_12V_Vol_2->setNum(msg->volt_bus2_voltage);
  ui->label_Psu_Motor_Vol_1->setNum(msg->motor_bus1_voltage);
  ui->label_Psu_Motor_Vol_2->setNum(msg->motor_bus2_voltage);
  ui->label_Psu_Motor_Vol_3->setNum(msg->motor_bus3_voltage);
  ui->label_Psu_Pc_Vol->setNum(msg->pc_voltage);
  ui->label_Psu_Dvl_Vol->setNum(msg->dvl_voltage);
  ui->label_Psu_Act_Vol->setNum(msg->actuator_bus_voltage);
  ui->label_Psu_light_Vol->setNum(msg->light_voltage);
  ui->label_Psu_12V_State_1->setNum(msg->volt_bus1_state);
  ui->label_Psu_12V_State_2->setNum(msg->volt_bus2_state);
  ui->label_Psu_Motor_State_1->setNum(msg->motor_bus1_state);
  ui->label_Psu_Motor_State_2->setNum(msg->motor_bus2_state);
  ui->label_Psu_Motor_State_3->setNum(msg->motor_bus3_state);
  ui->label_Psu_Dvl_State->setNum(msg->dvl_state);
  ui->label_Psu_Act_State->setNum(msg->actuator_bus_state);
  ui->label_Psu_Light_State->setNum(msg->light_state);
  ui->label_Psu_Pc_State->setNum(msg->pc_state);

  if (msg->kill_switch_state)
    ui->label_Kill_State->setText("On");
  else
    ui->label_Kill_State->setText("Off");
}

//------------------------------------------------------------------------------
//

void CanClient::ThrusterBackDCallback(
    const sonia_msgs::ThrusterMsg::ConstPtr &msg) {
  ui->label_Thruster_Speed_Bd->setNum(msg->speed);
}

//------------------------------------------------------------------------------
//

void CanClient::ThrusterFrontDCallback(
    const sonia_msgs::ThrusterMsg::ConstPtr &msg) {
  ui->label_Thruster_Speed_Fd->setNum(msg->speed);
}

//------------------------------------------------------------------------------
//

void CanClient::ThrusterBackHCallback(
    const sonia_msgs::ThrusterMsg::ConstPtr &msg) {
  ui->label_Thruster_Speed_Bh->setNum(msg->speed);
}

//------------------------------------------------------------------------------
//

void CanClient::ThrusterFrontHCallback(
    const sonia_msgs::ThrusterMsg::ConstPtr &msg) {
  ui->label_Thruster_Speed_Fh->setNum(msg->speed);
}

//------------------------------------------------------------------------------
//

void CanClient::ThrusterStarCallback(
    const sonia_msgs::ThrusterMsg::ConstPtr &msg) {
  ui->label_Thruster_Speed_S->setNum(msg->speed);
}

//------------------------------------------------------------------------------
//

void CanClient::ThrusterPortCallback(
    const sonia_msgs::ThrusterMsg::ConstPtr &msg) {
  ui->label_Thruster_Speed_P->setNum(msg->speed);
}

//------------------------------------------------------------------------------
//

void CanClient::BarometerDepthCallback(
    const sonia_msgs::BarometerMsg::ConstPtr &msg) {
  ui->label_Baro_press_2->setNum((double)msg->depth);
}

//------------------------------------------------------------------------------
//

void CanClient::BarometerPressCallback(
    const sensor_msgs::FluidPressure::ConstPtr &msg) {
  ui->label_Baro_press->setNum(msg->fluid_pressure);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Hydr_Refrsh_clicked() {
  can_service_client_.call(can_hydros_get_params_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_lineEdit_Div_Mission_string_editingFinished() {}

//------------------------------------------------------------------------------
//

void CanClient::on_lineEdit_Div_State_String_editingFinished() {}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Led_Set_clicked() {
  led_indicator_srv_.request.method_number =
      led_indicator_srv_.request.METHOD_LED_set_color;
  led_indicator_srv_.request.parameter_value =
      ui->comboBox_Led_Color->currentIndex();
  can_service_client_.call(led_indicator_srv_);
  led_indicator_srv_.request.method_number =
      led_indicator_srv_.request.METHOD_LED_set_mode;
  led_indicator_srv_.request.parameter_value =
      ui->comboBox_Led_Mode->currentIndex();
  can_service_client_.call(led_indicator_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_On_12V_1_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_set_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_BUS_12V1;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_On_12V_2_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_set_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_BUS_12V2;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_On_Pc_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_set_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_PC;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_On_Dvl_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_set_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_DVL;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_On_Light_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_set_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_LIGHT;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_On_Act_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_set_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_ACTUATORS;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_Off_12V_1_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_clr_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_BUS_12V1;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_Off_12V_2_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_clr_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_BUS_12V2;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_Off_Pc_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_clr_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_PC;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_Off_Dvl_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_clr_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_DVL;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_Off_Light_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_clr_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_LIGHT;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_Off_Act_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_clr_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_ACTUATORS;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_On_Motor_1_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_set_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_BUS_12V1;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_Off_Motor_1_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_clr_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_MOTOR1;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_On_Motor_2_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_set_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_MOTOR2;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_Off_Motor_2_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_clr_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_MOTOR2;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_On_Motor_3_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_set_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_MOTOR3;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_psu_Off_Motor_3_clicked() {
  psu_srv_.request.method_number = psu_srv_.request.METHOD_PSU_clr_channel;
  psu_srv_.request.parameter_value = psu_srv_.request.PSU_CHAN_MOTOR3;
  can_service_client_.call(psu_srv_);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Hydr_MagDeph_clicked() {
  static bool graph = true;

  graph = !graph;

  // show the hydrophone FFT graph. makes the tableWidgets behind disappear
  if (graph) {
    ui->tableWidget_Hydr_Fft_Mag->setVisible(false);
    ui->tableWidget_Hydr_Scope_samp->setVisible(false);
    ui->tableWidget_Hydr_Deph->setVisible(false);
    ui->label_Hydr_Freq->setVisible(false);
    ui->label_Hydr_Freq_1->setVisible(false);
    ui->label_Hydr_Freq_2->setVisible(false);
    ui->label_Hydr_Scope->setVisible(false);
    ui->label_Hydr_Mag->setVisible(false);
    ui->label_Hydr_Deph->setText("FFT Graph");
    ui->plot_Hydr_Fft->setVisible(true);
    ui->label_Hydr_Deph->setVisible(true);
  } else {// hide the hyrophone FFT graph
    ui->tableWidget_Hydr_Fft_Mag->setVisible(true);
    ui->tableWidget_Hydr_Scope_samp->setVisible(true);
    ui->tableWidget_Hydr_Deph->setVisible(true);
    ui->label_Hydr_Freq->setVisible(true);
    ui->label_Hydr_Freq_1->setVisible(true);
    ui->label_Hydr_Freq_2->setVisible(true);
    ui->label_Hydr_Scope->setVisible(true);
    ui->label_Hydr_Mag->setVisible(true);
    ui->label_Hydr_Deph->setVisible(true);
    ui->label_Hydr_Deph->setText("Dephasage");
    ui->plot_Hydr_Fft->setVisible(false);
  }
}


void CanClient::on_pushButton_Thruster_Test_clicked() {

  thruster_test_thread_ = new std::thread (&CanClient::ThrusterTest, this, 10);
}

//------------------------------------------------------------------------------
//

void CanClient::on_pushButton_Hydr_Plot_clicked() {
  fft_curve_->setSamples(freq_points_, mag_points_, 64);
  ui->plot_Hydr_Fft->replot();
}

//------------------------------------------------------------------------------
//

int CanClient::ThrusterTest(int arg) {
  struct timeval start_time;
  struct timeval end_time;

  for(uint8_t i = 1; i < 7; i++){
    thrusters_back_heading_srv_.request.unique_id = i;

    gettimeofday(&start_time, NULL);
    gettimeofday(&end_time, NULL);

    thrusters_back_heading_srv_.request.parameter_value = 30;
    can_service_client_.call(thrusters_back_heading_srv_);

    while((end_time.tv_sec - start_time.tv_sec) < 2){
      gettimeofday(&end_time, NULL);
    }
    thrusters_back_heading_srv_.request.parameter_value = 0;
    can_service_client_.call(thrusters_back_heading_srv_);

  }
  thrusters_back_heading_srv_.request.unique_id = thrusters_back_depth_srv_.request.UNIQUE_ID_ACT_back_heading_motor;

  return 0;
}

//------------------------------------------------------------------------------
//

